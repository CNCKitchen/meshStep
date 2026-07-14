// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — tessellation: BREP -> watertight indexed mesh + per-triangle CAD face id.
//
// Each EDGE is sampled once and densified to the target edge length (shared between its two
// faces => watertight seams). Faces are triangulated in (u,v) parameter space, seam-unwrapped
// for periodic surfaces so they cannot twist:
//   - PLANE / CYLINDRICAL: boundary samples + interior (u,v) grid, triangulated by Delaunay.
//   - CONICAL: structured rings lerped from the (shared) base circle to the apex (ruled surface).
//   - SPHERICAL: a direct (u,v) grid.
// Every triangle is oriented outward using the STEP face's same_sense flag and the analytic
// surface normal. Inner-loop holes / B-spline curves land in M3b; isotropic remesh is M4.
import type { Vec3 } from "../geom/vec.ts";
import { cross, dot, lerp, normalize } from "../geom/vec.ts";
import { ref, refList, list, num, numList } from "../step/entities.ts";
import type { Param } from "../step/parser.ts";
import type { BrepModel, BLoop } from "../brep/build.ts";
import type { IndexedMesh } from "../io/stl.ts";
import type { Surface } from "../geom/surfaces.ts";
import { makeSurface, isSphere, isBSpline, Sphere, type BSplineSurface } from "../geom/surfaces.ts";
import { sampleEdgePolyline } from "../geom/curves.ts";
import { constrainedTriangulate } from "./cdt2d.ts";
import { earcut } from "./earcut.ts";
import { ekey, refineTriangulation } from "./refine.ts";
import { beginWarnings, takeWarnings, warn, type MeshWarning } from "./diag.ts";

const TWO_PI = Math.PI * 2;

/** Diagnostics for the gapcheck harness (MESHSTEP_DEBUG=1); no-op in production/browser. */
const DBG = typeof process !== "undefined" && !!process.env?.MESHSTEP_DEBUG;
type P2 = [number, number];

export interface TessOptions {
  /** Max chord deviation (mm) for curve/surface sampling. */
  chordTol?: number;
  /** Target / maximum edge length (mm). */
  targetEdge?: number;
  /** Max normal turn across an edge (radians) — drives curvature-adaptive interior density. */
  normalDev?: number;
  /** Called once per face with the mesher that produced it ("grid", "band", "thinRing",
   * "untriangulated", …). Characterization hook: the fallback dispatch order is semantic, and this
   * makes "which mesher handled which face class" testable instead of tribal knowledge. */
  trace?: (faceId: number, mesher: string) => void;
  /** Progress hook: called after every work unit (edge sampled, face meshed, solid stitched) with
   * monotone counts. `total` is fixed up front, so done/total is a usable fraction — including for
   * assemblies, where every solid's faces are in the count. Callers throttle; this stays sync. */
  onProgress?: (done: number, total: number) => void;
  /** Attach the per-edge boundary polylines (part-local mm, post-densification) to the result.
   * These are the exact samplings the mesh was built from, so measurement/snap geometry derived
   * from them is coincident with rendered feature edges — resampling outside tessellate is not. */
  collectEdgePolylines?: boolean;
}

export interface MeshResult {
  mesh: IndexedMesh;
  faceOfTri: Uint32Array;
  /** STEP solid (body) id per triangle; bodies are welded independently and kept disjoint. */
  solidOfTri: Uint32Array;
  /** Ids of surface bodies built from OPEN_SHELLs: their boundary edges are open by design, so
   * watertightness checks must exclude their triangles rather than report defects. */
  openSolids: number[];
  stats: { solids: number; facesTotal: number; facesTessellated: number; skipped: Record<string, number> };
  /** Structured findings from the rescue/fallback meshing paths (dropped/skipped faces, heuristic
   * fills, degenerate boundaries) — the per-face half of the import diagnostics. */
  warnings: MeshWarning[];
  /** Per-edge boundary polylines (part-local mm) when `collectEdgePolylines` was set. */
  edgePolylines?: Map<number, Vec3[]>;
}

const bump = (o: Record<string, number>, k: string): void => { o[k] = (o[k] ?? 0) + 1; };

/** Emit a triangle, oriented so its normal points outward (surface normal × same_sense sign).
 * `refN` short-circuits the orientation reference: callers that know the triangle's (u,v) pass
 * surface.normal(u,v) directly — without it the 3D centroid is inverse-mapped first, which on a
 * B-spline is a full Newton solve PER TRIANGLE (formerly the hottest line on curved models). */
function emitTri(
  verts: number[], faceIds: number[], a: Vec3, b: Vec3, c: Vec3, fid: number, surface: Surface, sign: number,
  refN?: Vec3,
): void {
  const ng = cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
  if (ng[0] * ng[0] + ng[1] * ng[1] + ng[2] * ng[2] < 1e-18) return; // degenerate / zero-area
  let nrm = refN;
  if (!nrm) {
    const cen: Vec3 = [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
    const [u, v] = surface.project(cen);
    nrm = surface.normal(u, v);
  }
  const outward = dot(ng, nrm) * sign;
  if (outward >= 0) verts.push(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
  else verts.push(a[0], a[1], a[2], c[0], c[1], c[2], b[0], b[1], b[2]);
  faceIds.push(fid);
}

/** Unwrap a periodic coordinate (component c: 0=u, 1=v) so the loop is continuous (no full-period
 * jumps at the seam). Period is 2π for the analytic surfaces, v1-v0 for a closed B-spline. */
function unwrap(p2: P2[], c: 0 | 1, period = TWO_PI): void {
  const half = period / 2;
  for (let i = 1; i < p2.length; i++) {
    let d = p2[i]![c] - p2[i - 1]![c];
    while (d > half) { p2[i]![c] -= period; d -= period; }
    while (d < -half) { p2[i]![c] += period; d += period; }
  }
}

/** One boundary edge's pcurve: the oriented shared polyline's (u,v) image, continuous within the
 * edge (each periodic axis unwrapped independently), plus which periodic seam it hugs. The 3D
 * samples are the SHARED edge polyline verbatim — only their 2D images are computed here. */
interface EdgePC {
  p3: Vec3[];
  p2: P2[];
  hugU: boolean;
  hugV: boolean;
  /** Zero metric extent along the axis (an iso-parameter edge): period-ambiguous exactly like a
   * seam-hugging edge whenever the rest of the loop cannot pin its lift (a fully collapsed loop). */
  isoU: boolean;
  isoV: boolean;
}

/** The projected loop boundary. windU/windV: closure drift in whole periods on the universal cover
 * — non-zero means the loop WINDS the periodic surface (a bare rim), so it bounds no trimmed patch
 * and must go to the band/unroll meshers instead. tangled: the loop still self-intersects after the
 * phase-C repair (gates the tolerant-CDT path). */
interface LoopUV { p3: Vec3[]; p2: P2[]; windU: number; windV: number; tangled: boolean }

/** Hint-chained pointwise projection of a 3D polyline with two guards per point: (a) the existing
 * large-residual → stateless-reproject fallback (the chained hint sent the solver astray), and (b)
 * a METRIC step check — the (u,v) step (shortest representative modulo each period) mapped through
 * the local surface scale must be commensurate with the 3D chord between the samples. A
 * near-self-touching surface (a pinched Shapr3D freeform, residual ~1e-4 on the wrong fold) passes
 * a residual-only test yet jumps folds in parameter, folding the boundary over itself — the metric
 * check catches exactly that and retries statelessly, keeping whichever candidate steps
 * consistently; a legitimate seam crossing wraps to a SMALL step and stays untouched. */
function chainProject(surface: Surface, poly: Vec3[], hint?: P2): P2[] {
  const resid = (q: P2, pt: Vec3): number => { const e = surface.evaluate(q[0], q[1]); return Math.hypot(e[0] - pt[0], e[1] - pt[1], e[2] - pt[2]); };
  const uP = surface.periodicU ? surface.uPeriod || TWO_PI : 0;
  const vP = surface.periodicV ? surface.vPeriod || TWO_PI : 0;
  const wrapd = (d: number, P: number): number => { if (!P) return d; d %= P; if (d > P / 2) d -= P; else if (d < -P / 2) d += P; return d; };
  const p2: P2[] = [];
  let hu: number | undefined = hint?.[0], hv: number | undefined = hint?.[1];
  let uScale = 1, vScale = 1, haveScale = false;
  for (let i = 0; i < poly.length; i++) {
    const pt = poly[i]!;
    let q = surface.project(pt, hu, hv);
    if (i === 0) {
      if (resid(q, pt) > 1e-3) { const g = surface.project(pt); if (resid(g, pt) < resid(q, pt)) q = g; }
    } else {
      if (!haveScale) {
        const q0 = p2[0]!, eu = 1e-3 * (uP || 1), ev = 1e-3 * (vP || 1);
        const a = surface.evaluate(q0[0] + eu, q0[1]), b = surface.evaluate(q0[0] - eu, q0[1]);
        const c = surface.evaluate(q0[0], q0[1] + ev), d = surface.evaluate(q0[0], q0[1] - ev);
        uScale = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) / (2 * eu);
        vScale = Math.hypot(c[0] - d[0], c[1] - d[1], c[2] - d[2]) / (2 * ev);
        haveScale = true;
      }
      const prev = p2[i - 1]!, pp = poly[i - 1]!;
      const d3 = Math.hypot(pt[0] - pp[0], pt[1] - pp[1], pt[2] - pp[2]);
      const mstep = (c: P2): number => Math.hypot(wrapd(c[0] - prev[0], uP) * uScale, wrapd(c[1] - prev[1], vP) * vScale);
      const rq = resid(q, pt);
      if (rq > 1e-3 || mstep(q) > 3 * d3 + 1e-6) {
        const g = surface.project(pt);
        const rg = resid(g, pt);
        const qOk = rq <= 1e-3, gOk = rg <= 1e-3;
        if (gOk && (!qOk || mstep(g) < mstep(q))) q = g;
        else if (!qOk && rg < rq) q = g;
      }
    }
    p2.push([q[0], q[1]]); hu = q[0]; hv = q[1];
  }
  return p2;
}

/** A SLIT edge walks out and back along (nearly) the same 3D curve — a pinched zero-width cut,
 * e.g. the drill-point slit at a counterbore's bottom in a Shapr3D export. On a pinched surface the
 * two legs may project to DIFFERENT folds (both 3D-valid to ~1e-4, so no pointwise or metric test
 * can tell them apart) and the boundary grows a fold-lobe that double-covers the face in 3D while
 * staying simple in (u,v). Coincident 3D points must get coincident (u,v): copy each return-leg
 * point's image from its nearest outbound sample. The pcurve becomes a zero-width parametric spike,
 * which the CDT's ring sanitizer then collapses cleanly. */
function slitCollapse(poly: Vec3[], p2: P2[]): void {
  const n = poly.length;
  if (n < 5) return;
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  let k = 0, far = 0;
  for (let i = 0; i < n; i++) { const di = d3(poly[i]!, poly[0]!); if (di > far) { far = di; k = i; } }
  if (k < 2 || k > n - 3 || far < 1e-9) return;      // turn point must be interior
  if (d3(poly[0]!, poly[n - 1]!) > 0.25 * far) return; // doesn't come back to its start
  const out = poly.slice(0, k + 1);
  const tol = Math.max(2e-3, 0.02 * far);            // slit width: pinched, not a genuine thin U
  for (let i = k + 1; i < n; i++) if (distToPolyline(poly[i]!, out) > tol) return;
  for (let i = k + 1; i < n; i++) {
    let bj = 0, bd = Infinity;
    for (let j = 0; j <= k; j++) { const dd = d3(poly[i]!, poly[j]!); if (dd < bd) { bd = dd; bj = j; } }
    p2[i] = [p2[bj]![0], p2[bj]![1]];
  }
}

/** Phase A — per-edge pcurve: project one edge's shared polyline independently (a projection
 * failure in one edge then can't corrupt the rest of the loop), then unwrap each periodic axis so
 * the pcurve is continuous across the seam — a point ON the seam legitimately projects to either
 * side, and both lifts are valid; assembly (phase B) picks the loop-consistent representative. */
function edgePcurve(surface: Surface, poly: Vec3[]): EdgePC {
  const p2 = chainProject(surface, poly);
  slitCollapse(poly, p2);
  if (surface.periodicU) unwrap(p2, 0, surface.uPeriod || TWO_PI);
  if (surface.periodicV) unwrap(p2, 1, surface.vPeriod || TWO_PI);
  return {
    p3: poly, p2,
    hugU: hugsSeam(surface, p2, 0), hugV: hugsSeam(surface, p2, 1),
    isoU: isoParam(surface, p2, 0), isoV: isoParam(surface, p2, 1),
  };
}

/** True when an edge pcurve has (metrically) zero extent along periodic axis c — an iso-parameter
 * edge such as a straight cylinder ruling. Same metric tolerance as hugsSeam, measured against the
 * edge's own mean instead of the seam iso-line: ambiguity is a property of the edge's shape, not of
 * where the projector's branch cut happens to sit (ABC 00001709's full-wrap band has both rails at
 * u=0 while the atan2 seam is at π — hugsSeam never fires, yet the rails are period-ambiguous). */
function isoParam(surface: Surface, p2: P2[], c: 0 | 1): boolean {
  const periodic = c === 0 ? surface.periodicU : surface.periodicV;
  if (!periodic || p2.length === 0) return false;
  const period = (c === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI;
  const q = p2[p2.length >> 1]!;
  const e = 1e-3 * period;
  const a = surface.evaluate(q[0] + (c === 0 ? e : 0), q[1] + (c === 1 ? e : 0));
  const b = surface.evaluate(q[0] - (c === 0 ? e : 0), q[1] - (c === 1 ? e : 0));
  const scale = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) / (2 * e);
  const tol = Math.max(1e-3, 1e-3 * period * scale);
  let mn = Infinity, mx = -Infinity;
  for (const p of p2) { if (p[c] < mn) mn = p[c]; if (p[c] > mx) mx = p[c]; }
  return (mx - mn) * scale <= tol;
}

/** True when every point of an edge pcurve lies metrically ON the periodic seam iso-line of axis c.
 * Such an edge is the ambiguous kind: it is equally valid at the seam and at seam+period, and
 * junction continuity alone cannot tell the sides apart (its junction vertices are on the seam
 * too), so phase C may need to flip it. Tolerance is METRIC (mm via the local surface scale), never
 * a blind parameter epsilon — parameter units differ wildly between surfaces. */
function hugsSeam(surface: Surface, p2: P2[], c: 0 | 1): boolean {
  const periodic = c === 0 ? surface.periodicU : surface.periodicV;
  if (!periodic || p2.length === 0) return false;
  const period = (c === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI;
  const seam = (c === 0 ? surface.uSeam : surface.vSeam) ?? Math.PI;
  const q = p2[p2.length >> 1]!;
  const e = 1e-3 * period;
  const a = surface.evaluate(q[0] + (c === 0 ? e : 0), q[1] + (c === 1 ? e : 0));
  const b = surface.evaluate(q[0] - (c === 0 ? e : 0), q[1] - (c === 1 ? e : 0));
  const scale = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) / (2 * e);
  const tol = Math.max(1e-3, 1e-3 * period * scale); // mm; ~0.1% of the metric period
  const half = period / 2;
  for (const p of p2) {
    let d = (p[c] - seam) % period;
    if (d > half) d -= period; else if (d < -half) d += period;
    if (Math.abs(d) * scale > tol) return false;
  }
  return true;
}

/** Phase B — wire assembly: lift the loop to the universal cover. Walk the edges in loop order and
 * place edge k shifted by whole periods (each periodic axis independently) so its start meets edge
 * k-1's end — the junction is the same 3D vertex, so its two projections agree modulo the period
 * and the minimal-gap shift is exact. A seam edge traversed twice in one loop automatically lands
 * one period apart (the edges between the traversals walked the loop across the domain). `forced`
 * (whole periods per edge index & axis) overrides continuity for flagged edges — phase C's knob.
 * Returns the concatenated boundary in gridCDT's layout (per-edge closing vertex dropped; p2/p3
 * stay 1:1) plus the closure drift in whole periods. */
function assembleWire(surface: Surface, pcs: EdgePC[], forced?: Int8Array): { p3: Vec3[]; p2: P2[]; windU: number; windV: number } {
  const uP = surface.periodicU ? surface.uPeriod || TWO_PI : 0;
  const vP = surface.periodicV ? surface.vPeriod || TWO_PI : 0;
  const p3: Vec3[] = [], p2: P2[] = [];
  let eu = 0, ev = 0, su = 0, sv = 0;
  for (let k = 0; k < pcs.length; k++) {
    const e = pcs[k]!;
    const q0 = e.p2[0]!;
    let du = 0, dv = 0;
    if (k > 0) {
      if (uP) du = Math.round((eu - q0[0]) / uP) * uP;
      if (vP) dv = Math.round((ev - q0[1]) / vP) * vP;
    }
    if (forced) { du += forced[2 * k]! * uP; dv += forced[2 * k + 1]! * vP; }
    const m = e.p2.length;
    for (let i = 0; i < m - 1; i++) { p2.push([e.p2[i]![0] + du, e.p2[i]![1] + dv]); p3.push(e.p3[i]!); }
    eu = e.p2[m - 1]![0] + du; ev = e.p2[m - 1]![1] + dv;
    if (k === 0) { su = q0[0] + du; sv = q0[1] + dv; }
  }
  return {
    p3, p2,
    windU: uP ? Math.round((eu - su) / uP) : 0,
    windV: vP ? Math.round((ev - sv) / vP) : 0,
  };
}

/** Count strict self-intersections of a closed 2D polygon (segments sharing a vertex don't count).
 * Sorted-bbox sweep in x keeps it near-linear at boundary density; `cap` early-outs a hopeless
 * candidate during the phase-C search. `pairs` (optional) collects the crossing segment-index
 * pairs — the micro-face densifier uses their loop-distance to tell a curable local zigzag from
 * genuine trim overlap. */
function countSelfIntersections(pts: P2[], cap = 1 << 30, pairs?: [number, number][]): number {
  const n = pts.length;
  if (n < 4) return 0;
  const x0 = new Float64Array(n), x1 = new Float64Array(n), y0 = new Float64Array(n), y1 = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const a = pts[i]!, b = pts[(i + 1) % n]!;
    x0[i] = Math.min(a[0], b[0]); x1[i] = Math.max(a[0], b[0]);
    y0[i] = Math.min(a[1], b[1]); y1[i] = Math.max(a[1], b[1]);
  }
  const order = Array.from({ length: n }, (_, i) => i).sort((a, b) => x0[a]! - x0[b]!);
  const orient2 = (a: P2, b: P2, c: P2): number => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
  let count = 0;
  for (let oi = 0; oi < n && count < cap; oi++) {
    const i = order[oi]!;
    for (let oj = oi + 1; oj < n; oj++) {
      const j = order[oj]!;
      if (x0[j]! > x1[i]!) break; // sweep past i's extent — no later j can overlap
      if (y0[j]! > y1[i]! || y1[j]! < y0[i]!) continue;
      if ((i + 1) % n === j || (j + 1) % n === i) continue; // adjacent segments share a vertex
      const p = pts[i]!, q = pts[(i + 1) % n]!, r = pts[j]!, s = pts[(j + 1) % n]!;
      const d1 = orient2(r, s, p), d2 = orient2(r, s, q), d3 = orient2(p, q, r), d4 = orient2(p, q, s);
      if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        pairs?.push([i, j]);
        if (++count >= cap) break;
      }
    }
  }
  return count;
}

/**
 * Phase C — repair: the assembled loop self-intersects, so search the bounded space of
 * seam-representative choices. Only seam-hugging edges are ambiguous (both period lifts are
 * geometrically valid for them), so few edges have candidates and the combinations stay small
 * (offsets {0,±1} per hugging axis, capped). Edge 0's lift is fixed — shifting it just translates
 * the whole loop. Each combination re-chains the remaining edges by continuity and is scored
 * lexicographically: fewer self-intersections, then exact closure, then the expected winding sign
 * (STEP bound orientation: outer runs CW in this codebase's (u,v) when same_sense holds), then the
 * larger |area| (the un-tangled lift of a seam-split band spans the period; the tangled one nearly
 * cancels). Returns the best assembly — the caller's baseline is in the running, so this never
 * makes the loop worse.
 */
function repairWire(
  surface: Surface, pcs: EdgePC[], base: ReturnType<typeof assembleWire>, baseInts: number, expectSign: number,
  allowIso = false,
): { asm: ReturnType<typeof assembleWire>; selfInts: number } {
  // `allowIso` widens the candidate set from seam-hugging edges to ALL iso-parameter edges — only
  // safe when the caller's baseline is a COLLAPSED loop (zero width), where there is no legitimate
  // baseline lift to lose; on a healthy loop an iso edge is pinned by continuity and forcing its
  // lift could out-score the true region via the area tiebreak (a 90° wedge "unfolded" to 270°).
  const slots: number[] = []; // flat (edgeIndex, axis) pairs, axis 0=u 1=v
  for (let k = 1; k < pcs.length; k++) {
    if (surface.periodicU && (pcs[k]!.hugU || (allowIso && pcs[k]!.isoU))) slots.push(2 * k);
    if (surface.periodicV && (pcs[k]!.hugV || (allowIso && pcs[k]!.isoV))) slots.push(2 * k + 1);
    if (slots.length >= 5) break; // 3^5 = 243 combinations — bounded
  }
  let best = { asm: base, selfInts: baseInts };
  let bestScore = score(base, baseInts);
  if (slots.length === 0) return best;
  const forced = new Int8Array(2 * pcs.length);
  const offsets = [0, 1, -1];
  const total = Math.pow(3, slots.length);
  for (let combo = 1; combo < total; combo++) {
    let c = combo;
    for (const s of slots) { forced[s] = offsets[c % 3]!; c = (c / 3) | 0; }
    const asm = assembleWire(surface, pcs, forced);
    const ints = countSelfIntersections(asm.p2, best.selfInts + 1);
    const sc = score(asm, ints);
    if (sc[0] < bestScore[0] || (sc[0] === bestScore[0] && (sc[1] < bestScore[1]
      || (sc[1] === bestScore[1] && (sc[2] < bestScore[2] || (sc[2] === bestScore[2] && sc[3] < bestScore[3])))))) {
      best = { asm, selfInts: ints }; bestScore = sc;
    }
  }
  return best;

  function score(asm: ReturnType<typeof assembleWire>, ints: number): [number, number, number, number] {
    const area = polyArea(asm.p2);
    const signOk = expectSign === 0 || Math.sign(area) === expectSign ? 0 : 1;
    return [ints, Math.abs(asm.windU) + Math.abs(asm.windV), signOk, -Math.abs(area)];
  }
}

/** The original whole-loop projection: every boundary point hint-chained from the previous one,
 * then the concatenated loop unwrapped per periodic axis. On a well-behaved surface this is exact;
 * on a PINCHED surface (a flattened tube whose opposite folds coincide within ~1e-4, as Shapr3D
 * emits around counterbore rims) the global chain is actually the most robust pointwise scheme —
 * both folds contain the junction vertices, and only fold-continuity through the whole loop keeps
 * every edge on the loop-consistent fold. It stays the fast path; the per-edge cover assembly below
 * only takes over when this result is measurably tangled. */
function legacyLoopParam(surface: Surface, loop: BLoop, sampled: Map<number, Vec3[]>): { p3: Vec3[]; p2: P2[] } {
  const p3: Vec3[] = [];
  const p2: P2[] = [];
  // Chained per edge with the hint carried across the junction (the junction is the same 3D
  // vertex, so this is the whole-loop chain), which lets each edge get the slit treatment.
  let hint: P2 | undefined;
  for (const oe of loop.edges) {
    const base = sampled.get(oe.edgeId);
    if (!base) continue;
    const poly = oe.orient ? base : base.slice().reverse();
    const ep2 = chainProject(surface, poly, hint);
    slitCollapse(poly, ep2);
    hint = ep2[ep2.length - 1];
    for (let i = 0; i < poly.length - 1; i++) { p3.push(poly[i]!); p2.push(ep2[i]!); }
  }
  if (surface.periodicU) unwrap(p2, 0, surface.uPeriod || TWO_PI);
  if (surface.periodicV) unwrap(p2, 1, surface.vPeriod || TWO_PI);
  // START-RUN REPAIR (wrap-around chaining). The loop's FIRST edge has no chaining hint; its
  // grid-nearest seed is arbitrary when the patch has a COLLAPSED boundary row — S(u,0) is one 3D
  // point for EVERY u (Stealthburner's fillet patches) — so a LEADING RUN of points lands at a
  // far-off u on the degenerate row (one can even stick there with a millimetre residual: the
  // Jacobian is singular along the row), the polygon crosses the whole domain, the CDT loses
  // constraints and the rescue fills phantom area. The loop is cyclic: when the CLOSING step's
  // (u,v) jump is far larger than its 3D chord implies (metric-relative AND a real fraction of the
  // domain), re-chain the leading run hinted from the loop END, accepting each redo only while it
  // stays on the surface at least as well as the original, and stopping once the redo rejoins the
  // original chain. Non-periodic surfaces only — a wound loop on a periodic surface legitimately
  // closes a whole period apart, and the seam machinery below owns that case.
  if (!surface.periodicU && !surface.periodicV && p2.length >= 4) {
    const n = p2.length;
    const d2p = (a: P2, b: P2): number => Math.hypot(a[0] - b[0], a[1] - b[1]);
    const d3p = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    const ratios: number[] = [];
    for (let i = 0; i + 1 < n; i++) {
      const dd = d3p(p3[i]!, p3[i + 1]!);
      if (dd > 1e-9) ratios.push(d2p(p2[i]!, p2[i + 1]!) / dd);
    }
    ratios.sort((a, b) => a - b);
    const med = ratios[ratios.length >> 1] ?? 0;
    let umn = Infinity, umx = -Infinity, vmn = Infinity, vmx = -Infinity;
    for (const q of p2) { if (q[0] < umn) umn = q[0]; if (q[0] > umx) umx = q[0]; if (q[1] < vmn) vmn = q[1]; if (q[1] > vmx) vmx = q[1]; }
    const extent = Math.hypot(umx - umn, vmx - vmn);
    const jump = d2p(p2[n - 1]!, p2[0]!);
    if (med > 0 && jump > 20 * med * Math.max(d3p(p3[n - 1]!, p3[0]!), 1e-9) && jump > 0.05 * extent) {
      let hint = p2[n - 1]!;
      for (let i = 0; i < n >> 1; i++) {
        const q = surface.project(p3[i]!, hint[0], hint[1]);
        if (d2p(q, p2[i]!) < 1e-6 * Math.max(extent, 1e-9)) break; // rejoined the original chain
        const eOld = surface.evaluate(p2[i]![0], p2[i]![1]);
        const eNew = surface.evaluate(q[0], q[1]);
        const rOld = d3p(eOld, p3[i]!), rNew = d3p(eNew, p3[i]!);
        if (rNew > Math.max(2 * rOld, 1e-3)) break; // redo left the surface — keep the original
        p2[i] = q; hint = q;
      }
    }
  }
  // Repair ISOLATED (u,v) outliers. The loop's very first projection has no hint, and on a surface
  // that passes close to itself (a helical thread: adjacent turns are microns apart in 3D) the
  // nearest-grid-node seed can converge onto the WRONG TURN — one point whose (u,v) sits a dozen
  // units from both neighbours while the neighbours agree with each other. That single zigzag
  // self-intersects the polygon, inRegion then misclassifies the whole interior, and the CDT fills
  // barrel-spanning garbage (furniture-leg's thread). The loop is cyclic, so every point has two
  // CHAINED neighbours: re-project a flagged point hinted from its predecessor and accept only a
  // result that actually lands between the neighbours AND stays on the surface at the sample.
  const n = p2.length;
  if (n >= 4) {
    const d2 = (a: P2, b: P2): number => Math.hypot(a[0] - b[0], a[1] - b[1]);
    for (let i = 0; i < n; i++) {
      const a = p2[(i + n - 1) % n]!, b = p2[i]!, c = p2[(i + 1) % n]!;
      const dAC = d2(a, c), dAB = d2(a, b), dBC = d2(b, c);
      if (dAB < 8 * dAC + 1e-9 || dBC < 8 * dAC + 1e-9) continue;
      const q = surface.project(p3[i]!, a[0], a[1]);
      if (d2(q, a) + d2(q, c) >= 0.5 * (dAB + dBC)) continue; // did not land between the neighbours
      const s = surface.evaluate(q[0], q[1]);
      const p = p3[i]!;
      const sOld = surface.evaluate(b[0], b[1]);
      const rNew = Math.hypot(s[0] - p[0], s[1] - p[1], s[2] - p[2]);
      const rOld = Math.hypot(sOld[0] - p[0], sOld[1] - p[1], sOld[2] - p[2]);
      if (rNew <= Math.max(2 * rOld, 1e-3)) p2[i] = q;
    }
  }
  return { p3, p2 };
}

/** Net winding of a closed cycle around periodic axis c: sum of shortest-representative steps,
 * including the closing one, in whole periods. */
function cycleWind(p2: P2[], c: 0 | 1, period: number): number {
  if (!period) return 0;
  let w = 0;
  const half = period / 2;
  for (let i = 0; i < p2.length; i++) {
    let d = p2[(i + 1) % p2.length]![c] - p2[i]![c];
    d %= period; if (d > half) d -= period; else if (d < -half) d += period;
    w += d;
  }
  return Math.round(w / period);
}

/** Project a boundary loop to (u,v). The proven whole-loop chained projection is the fast path,
 * kept bit-for-bit for every loop it handles cleanly. When its result SELF-INTERSECTS on a periodic
 * surface (a seam-tangled loop), the seam-aware machinery takes over: per-edge pcurves (phase A)
 * assembled on the universal cover (phase B), repaired over the bounded seam-representative choices
 * (phase C) — and the better of the two candidates is returned. expectSign is the loop's expected
 * winding sign in (u,v) (repair tiebreak; 0 = unknown). Exported for tests. */
export function loopParam(surface: Surface, loop: BLoop, sampled: Map<number, Vec3[]>, expectSign = 0): LoopUV {
  const uP = surface.periodicU ? surface.uPeriod || TWO_PI : 0;
  const vP = surface.periodicV ? surface.vPeriod || TWO_PI : 0;
  const leg = legacyLoopParam(surface, loop, sampled);
  if ((!uP && !vP) || leg.p2.length < 4) return { ...leg, windU: 0, windV: 0, tangled: false };
  const legInts = countSelfIntersections(leg.p2);
  const legWind: [number, number] = [cycleWind(leg.p2, 0, uP), cycleWind(leg.p2, 1, vP)];
  // A COLLAPSED loop — near-zero enclosed area over a long perimeter — is as broken as a tangled
  // one, but has zero STRICT self-intersections (its out-and-back legs are collinear, and collinear
  // overlap never counts as a crossing), so it used to short-circuit here and the face died at the
  // zero-width gate. The signature case is a full-wrap band whose BOTH rails lie exactly ON the
  // periodic seam (ABC 00001709: a cylinder face bounded by two seam lines — one belongs at u=0,
  // the other at u=2π): continuity alone cannot split them, but the phase-C seam search can — its
  // area tiebreak prefers the full-period lift, and a genuinely degenerate slit that gets lifted
  // into a phantom band is caught by the caller's fold audit (boundary over-coverage → rollback).
  const collapsedLoop = (p2: P2[]): boolean => {
    // Only multi-edge loops qualify: a single-closed-edge rim circle (or a two-edge seam-split rim)
    // legitimately projects to a zero-area line, and dragging those through the reassembly search
    // regressed healthy B-spline bands (run5: 5 models) — the "bigger area" lift the score prefers
    // is a phantom there. The thread-band class this trigger exists for has ≥ 3 edges by shape
    // (two rails + at least one closer).
    if (loop.edges.length < 3) return false;
    let per = 0;
    for (let i = 0; i < p2.length; i++) { const a = p2[i]!, b = p2[(i + 1) % p2.length]!; per += Math.hypot(b[0] - a[0], b[1] - a[1]); }
    return 2 * Math.abs(polyArea(p2)) / Math.max(per, 1e-12) < 1e-6 * per;
  };
  if (legInts === 0 && !collapsedLoop(leg.p2)) return { ...leg, windU: legWind[0], windV: legWind[1], tangled: false };

  const pcs: EdgePC[] = [];
  for (const oe of loop.edges) {
    const base = sampled.get(oe.edgeId);
    if (!base) continue;
    pcs.push(edgePcurve(surface, oe.orient ? base : base.slice().reverse()));
  }
  if (pcs.length === 0) return { ...leg, windU: legWind[0], windV: legWind[1], tangled: true };
  let asm = assembleWire(surface, pcs);
  let selfInts = countSelfIntersections(asm.p2);
  const asmCollapsed = collapsedLoop(asm.p2);
  if (selfInts > 0 || asmCollapsed) ({ asm, selfInts } = repairWire(surface, pcs, asm, selfInts, expectSign, asmCollapsed));
  // Choose between the legacy loop and the reassembled one by the same lexicographic score the
  // repair search uses; the legacy result wins ties (bit-for-bit stability for everything the old
  // projector already handled acceptably).
  const scoreOf = (p2: P2[], ints: number, w: [number, number]): [number, number, number, number] => {
    const area = polyArea(p2);
    return [ints, Math.abs(w[0]) + Math.abs(w[1]), expectSign === 0 || Math.sign(area) === expectSign ? 0 : 1, -Math.abs(area)];
  };
  const sl = scoreOf(leg.p2, legInts, legWind);
  const sn = scoreOf(asm.p2, selfInts, [asm.windU, asm.windV]);
  const newBetter = sn[0] < sl[0] || (sn[0] === sl[0] && (sn[1] < sl[1]
    || (sn[1] === sl[1] && (sn[2] < sl[2] || (sn[2] === sl[2] && sn[3] < sl[3])))));
  return newBetter
    ? { p3: asm.p3, p2: asm.p2, windU: asm.windU, windV: asm.windV, tangled: selfInts > 0 }
    : { ...leg, windU: legWind[0], windV: legWind[1], tangled: true };
}

/** Signed area of a closed (u,v) polygon (shoelace); |area| ranks loops to find the outer boundary. */
function polyArea(poly: P2[]): number {
  let a = 0;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) a += (poly[j]![0] + poly[i]![0]) * (poly[j]![1] - poly[i]![1]);
  return a / 2;
}

function pointInPoly(p: P2, poly: P2[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i]!, b = poly[j]!;
    if ((a[1] > p[1]) !== (b[1] > p[1])) {
      const x = ((b[0] - a[0]) * (p[1] - a[1])) / (b[1] - a[1]) + a[0];
      if (p[0] < x) inside = !inside;
    }
  }
  return inside;
}

/** Unwrap a periodic loop and shift it by whole periods so its mean (component c) sits near target. */
function shiftIntoRange(p2: P2[], target: number, c: 0 | 1, period = TWO_PI): void {
  unwrap(p2, c, period);
  let mean = 0;
  for (const p of p2) mean += p[c];
  mean /= p2.length;
  const k = Math.round((target - mean) / period);
  if (k !== 0) for (const p of p2) p[c] += k * period;
}

/** Isotropic interior edge target (mm) for a face: capped by curvature (chord + normal deviation)
 * so curved faces are finely sampled, by targetEdge on flat ones, with a floor to bound density.
 * The floor must NOT scale with targetEdge alone: a CAD-style export sets a huge max edge (85 mm) to
 * mean "don't cap by length, follow curvature", and targetEdge/40 would then be ~2 mm and coarsen
 * every fillet. Floor at the finer of targetEdge/40 and 30·chordTol so curvature drives the density
 * whenever the max edge is large, while small-target corpus runs (30·chordTol ≫ targetEdge/40) keep
 * their existing floor unchanged.
 * The floor bounds only the CHORD term: the chord requirement densifies with Rc/chordTol and can
 * explode, but the angular term is self-bounding (≤ 2π/(2·normalDev) segments per full turn no
 * matter the radius), and it is an explicit user setting — flooring it away silently violates the
 * requested normal deviation (e.g. a 100mm max edge turned a 10° setting into 2.5mm edges).
 * floorAngular=true (EDGE-SAMPLING use only) floors the angular term too: the per-turn bound is
 * per turn OF THE SURFACE, but an edge target applies along the edge's whole length — a thread
 * ribbon whose root radius is 0.2mm would sample its 1.6-METRE helical rails at 0.05mm
 * (30k points/rail, StingStopp_4000_Base) to honour a normal deviation that never materialises
 * along the ruling. Genuinely curved edges still densify through the curve's own chord/turn
 * criteria in sampleEdgePolyline; interior meshing keeps the unfloored requirement. */
function faceTarget(surface: Surface, targetEdge: number, chordTol: number, normalDev: number, u = 0, v = 0, floorAngular = false): number {
  const Rc = surface.curvatureRadius(u, v);
  const floor = Math.min(targetEdge / 40, 30 * chordTol);
  if (!Number.isFinite(Rc)) return targetEdge;
  const ang = floorAngular ? Math.max(floor, Rc * normalDev) : Rc * normalDev;
  return Math.min(targetEdge, Math.max(floor, Math.sqrt(8 * Rc * chordTol)), ang);
}

/**
 * Plane / cylinder / trimmed patch: outer + hole boundary samples plus an interior (u,v) grid,
 * triangulated by Delaunay and filtered to the trimmed region (inside outer, outside holes).
 */
function tessellateParamGrid(
  surface: Surface, loops: BLoop[], sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
): boolean {
  // Project every loop, then pick the outer boundary. Prefer the STEP FACE_OUTER_BOUND flag, but some
  // kernels (e.g. Onshape via ST-DEVELOPER) mark EVERY bound FACE_BOUND and never set it — there the
  // outer loop is whichever encloses the largest area in parameter space; the rest are holes. Without
  // this an annular face picks a hole as its boundary and meshes the wrong region (open seams).
  // expectSign feeds the seam-repair tiebreak: in this codebase's parametrisations the outer bound
  // runs CW (negative area) when same_sense holds, holes the other way; unknown (0) when no bound
  // carries the FACE_OUTER_BOUND flag.
  const outerFlagged = loops.some((l) => l.outer);
  const projected = loops.map((l) => ({
    outer: l.outer,
    lp: loopParam(surface, l, sampled, outerFlagged ? (l.outer ? -sign : sign) : 0),
  }));
  const bboxExtent = (p2: P2[]): number => {
    let umn = Infinity, umx = -Infinity, vmn = Infinity, vmx = -Infinity;
    for (const q of p2) { if (q[0] < umn) umn = q[0]; if (q[0] > umx) umx = q[0]; if (q[1] < vmn) vmn = q[1]; if (q[1] > vmx) vmx = q[1]; }
    return Math.hypot(umx - umn, vmx - vmn);
  };
  // Pick the outer boundary. Prefer the STEP FACE_OUTER_BOUND flag; otherwise the outer is the loop
  // with the largest parameter-space EXTENT (a hole always sits inside the outer). Extent beats
  // signed area here: the true outer sometimes projects to near-zero signed area (a collapsed /
  // self-cancelling seam boundary) while still being the largest loop — max-area would then wrongly
  // pick an interior hole as the boundary and mesh a spurious cap across it.
  let oi = projected.findIndex((p) => p.outer);
  if (oi < 0) {
    let best = -Infinity;
    for (let i = 0; i < projected.length; i++) { const e = bboxExtent(projected[i]!.lp.p2); if (e > best) { best = e; oi = i; } }
  }
  if (oi < 0) return false;
  const outer = projected[oi]!.lp;
  if (outer.p3.length < 3) {
    if (DBG) console.error(`[grid] fid=${fid} bail: outer loop projects to ${outer.p3.length} pts`);
    return false;
  }
  // Malformed trimming: the enclosing loop collapsed to ~zero area in parameter space (a degenerate
  // boundary a CAD kernel left, or two edges that trace back over each other). There's no valid
  // region — emit nothing. A clean gap reads far better than a spurious cap sealing a hole shut.
  // The judgement must be METRIC-scaled: raw (u,v) area is meaningless on an extreme-anisotropy
  // patch — a 0.06mm knife-edge strip whose u-knot-span is 3.5e-5 (handle_v4's tooth tips) is a
  // REAL face whose raw area always fails a raw-extent test, while the downstream grid handles the
  // anisotropy fine once it's allowed through.
  {
    let umn = Infinity, umx = -Infinity, vmn = Infinity, vmx = -Infinity;
    for (const q of outer.p2) {
      if (q[0] < umn) umn = q[0]; if (q[0] > umx) umx = q[0];
      if (q[1] < vmn) vmn = q[1]; if (q[1] > vmx) vmx = q[1];
    }
    const du = Math.max(umx - umn, 1e-12), dv = Math.max(vmx - vmn, 1e-12);
    const um = (umn + umx) / 2, vm = (vmn + vmx) / 2;
    const dd = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    // Arc-length (4-chord) metric per direction — an endpoint-to-endpoint chord is ZERO on a
    // full-period wrap (evaluate(umin) == evaluate(umax)) and would reject every closed band.
    // The probed span is capped at ONE period on a periodic axis: a multi-turn helical band (a
    // thread face unwound across several periods, ABC 00001709) has du = k·2π, and quarter-chords
    // of that alias to the SAME angular position — the metric reads ~0 and a real thread band
    // dies here as "zero width". The metric is per unit parameter and periodic, so one period
    // measures it exactly.
    const arc = (dir: 0 | 1): number => {
      const span = Math.min(dir === 0 ? du : dv,
        (dir === 0 ? surface.periodicU : surface.periodicV) ? ((dir === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI) : Infinity);
      let s = 0;
      let prev = surface.evaluate(dir === 0 ? umn : um, dir === 0 ? vm : vmn);
      for (let i = 1; i <= 4; i++) {
        const t = (i / 4) * span;
        const q = surface.evaluate(dir === 0 ? umn + t : um, dir === 0 ? vm : vmn + t);
        s += dd(prev, q); prev = q;
      }
      return s / span;
    };
    const mU = arc(0), mV = arc(1);
    const extS = Math.hypot(du * mU, dv * mV);
    // Degeneracy = metric WIDTH (2·area/perimeter), not area/extent²: an aspect gate rejects any
    // strip thinner than 1e-4 of its length, which throws away REAL 0.1mm × 1m chamfer faces on
    // large parts (ABC 00000982 and friends — the mesh then opens along both 1m rails). Width
    // separates the cases exactly: a collapsed/self-cancelling loop (out-and-back seam boundary,
    // kernel junk) has near-zero net area over a long perimeter → micron width → rejected, while
    // a genuinely thin face keeps its physical width and passes at any aspect.
    const areaM = Math.abs(polyArea(outer.p2)) * mU * mV;
    let perM = 0;
    for (let i = 0; i < outer.p2.length; i++) {
      const a = outer.p2[i]!, b = outer.p2[(i + 1) % outer.p2.length]!;
      perM += Math.hypot((b[0] - a[0]) * mU, (b[1] - a[1]) * mV);
    }
    if (2 * areaM / Math.max(perM, 1e-12) < Math.max(1e-3, 1e-9 * extS)) {
      if (DBG) console.error(`[grid] fid=${fid} bail: metric width ${(2 * areaM / Math.max(perM, 1e-12)).toExponential(2)} (area=${areaM.toExponential(2)} per=${perM.toFixed(3)} ext=${extS.toFixed(3)})`);
      return false;
    }
  }

  let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
  for (const q of outer.p2) {
    if (q[0] < umin) umin = q[0]; if (q[0] > umax) umax = q[0];
    if (q[1] < vmin) vmin = q[1]; if (q[1] > vmax) vmax = q[1];
  }
  const holes = projected.filter((_, i) => i !== oi).map((p) => p.lp);
  if (surface.periodicU) for (const h of holes) shiftIntoRange(h.p2, (umin + umax) / 2, 0, surface.uPeriod || TWO_PI);
  if (surface.periodicV) for (const h of holes) shiftIntoRange(h.p2, (vmin + vmax) / 2, 1, surface.vPeriod || TWO_PI);
  // Anisotropic ruling-aligned meshing needs a trustworthy parameter domain; a face whose boundary
  // projection tangled (pinched tubes, fold-ambiguous patches) gets the conservative isotropic grid.
  const clean = !projected.some((p) => p.lp.tangled);
  // An outer loop that WINDS a periodic axis is not a disk boundary — its lift's closing chord
  // spans a whole period. When the CDT realises everything anyway, fine (some once-winding rims
  // are genuine slivers); when it can't, the rescue fill leaks by construction, and the face
  // belongs to the band/region meshers — so require a fully-realised boundary for winding outers
  // (ABC 00000083: a closed-v B-spline whose single loop winds v once; grid emitted the leak and
  // region never got the face).
  const outerWinds = (!!surface.periodicU && Math.abs(projected[oi]!.lp.windU) >= 1)
    || (!!surface.periodicV && Math.abs(projected[oi]!.lp.windV) >= 1);
  const v0 = verts.length, f0 = faceIds.length;
  if (!gridCDT(surface, outer, holes, fid, verts, faceIds, targetEdge, chordTol, normalDev, sign, clean, false, false, outerWinds)) return false;
  // FOLD AUDIT. A correct triangulation of a trimmed patch covers each boundary segment exactly as
  // often as the boundary itself traverses it (once normally, twice along an out-and-back slit). On
  // a fold-degenerate surface — a flattened tube whose opposite folds coincide in 3D — the chained
  // projection can wander between folds without any pointwise signal, and the CDT then lays a second
  // sheet over part of the region; the extra sheet lands exactly on boundary segments (OpenVessel's
  // counterbore tubes: the face's own rim segments get TWO of its triangles, the neighbour's third
  // makes them non-manifold, and a fold chord is left open). Boundary OVER-coverage on the welded 3D
  // grid is therefore a proof of a folded result: roll the face back and let the caller's rail-ribbon
  // fallback mesh between the shared rails directly. UNDER-coverage is deliberately not flagged —
  // dropped 3D-degenerate slivers legitimately leave segments uncovered (the ftc-slot family).
  const qz = (x: number): number => Math.round(x * 1e6);
  const skey = (a: Vec3, b: Vec3): string => {
    const ka = `${qz(a[0])},${qz(a[1])},${qz(a[2])}`, kb = `${qz(b[0])},${qz(b[1])},${qz(b[2])}`;
    return ka < kb ? ka + "|" + kb : kb + "|" + ka;
  };
  const bound = new Map<string, number>();
  for (const lp of [outer, ...holes]) {
    const n = lp.p3.length;
    for (let i = 0; i < n; i++) {
      const k = skey(lp.p3[i]!, lp.p3[(i + 1) % n]!);
      bound.set(k, (bound.get(k) ?? 0) + 1);
    }
  }
  const used = new Map<string, number>();
  const all = new Map<string, number>();
  // (Re)tally this face's emitted triangle edges and return whether any boundary segment is
  // OVER-covered (proof of a fold). Rebuilds `used`/`all` in place so the surgery below sees them.
  const overCovered = (): boolean => {
    used.clear(); all.clear();
    for (let t = v0; t < verts.length; t += 9) {
      for (let e = 0; e < 3; e++) {
        const i1 = t + e * 3, i2 = t + ((e + 1) % 3) * 3;
        const k = skey([verts[i1]!, verts[i1 + 1]!, verts[i1 + 2]!], [verts[i2]!, verts[i2 + 1]!, verts[i2 + 2]!]);
        if (bound.has(k)) used.set(k, (used.get(k) ?? 0) + 1);
        all.set(k, (all.get(k) ?? 0) + 1);
      }
    }
    for (const [k, m] of bound) if ((used.get(k) ?? 0) > m) return true;
    return false;
  };
  if (overCovered()) {
    // The fold is usually introduced by the Delaunay REFINEMENT (its inserted circumcentres push the
    // re-CDT into filling across a hole on a multi-loop plane — z_bed/z_stepper trays); the un-refined
    // base CDT is fold-free. Retry once with refinement disabled before handing off to the fallback,
    // so a valid (coarser) mesh replaces an empty face. If the un-refined grid still over-covers, the
    // fold is structural (a genuinely folded projection) — roll back and let the rail-ribbon fallback
    // mesh between the shared rails (OpenVessel counterbore tubes).
    verts.length = v0; faceIds.length = f0;
    if (!gridCDT(surface, outer, holes, fid, verts, faceIds, targetEdge, chordTol, normalDev, sign, clean, true, false, outerWinds) || overCovered()) {
      // KEYHOLE DE-SLIT retry: a hole reachable only through a zero-width corridor folds exactly
      // here (the corridor's doubled samples over-cover their own segments once the refinement's
      // inserts push the re-CDT across them — Meanwell's notched sphere caps at coarse
      // tolerance). Retry with the corridor removed and the enclosed outline as a real hole; the
      // retry is a no-op (fails fast) for faces without a de-slit candidate.
      verts.length = v0; faceIds.length = f0;
      if (gridCDT(surface, outer, holes, fid, verts, faceIds, targetEdge, chordTol, normalDev, sign, clean, true, true, outerWinds) && !overCovered()) {
        if (DBG) console.error(`[grid] fid=${fid} FOLD AUDIT: de-slit retry is fold-free`);
        return true;
      }
      verts.length = v0; faceIds.length = f0;
      // The un-refined base CDT still folds — a STRUCTURAL fold. On a PLANE the (u,v)->3D map is
      // affine, so the region can be filled watertight without any parity flood: bridge every hole
      // into the outer boundary and ear-clip (earcut). This rescues the multi-loop tray / embossed-
      // text plane faces whose flood misclassifies in/out (z_bed mirror, Meanwell, Front_Skirt_Logo).
      // A curved-surface fold (OpenVessel counterbore tubes) can't use a planar clip — it keeps
      // falling through to the caller's rail-ribbon fallback.
      if (surface.kind === "PLANE" && planeEarcutFill(surface, outer, holes, fid, verts, faceIds, sign, targetEdge)) {
        warn("heuristic-fill", fid, "structural fold on a multi-loop plane — face rebuilt by earcut hole-bridge fill");
        if (DBG) console.error(`[grid] fid=${fid} FOLD AUDIT: earcut hole-bridge fill (structural multi-loop plane)`);
        return true;
      }
      if (DBG) console.error(`[grid] fid=${fid} FOLD AUDIT rollback (boundary segment over-covered)`);
      verts.length = v0; faceIds.length = f0; return false;
    }
    if (DBG) console.error(`[grid] fid=${fid} FOLD AUDIT: un-refined retry is fold-free`);
  }
  // SELF-OVERLAP SURGERY: a valid single-face triangulation is a disk — no 3D edge belongs to more
  // than two of its own triangles. Three or more means the projection folded and the CDT laid a
  // second sheet over part of the region (a tangled patch corner welds non-manifold: cat-napkin /
  // bottle-cage's B-spline folds). Rolling the whole face back trades a few non-manifold edges for
  // its entire open rim — worse. Instead peel the redundant sheet: greedily drop the triangle whose
  // edges are most over-covered until every edge is ≤2, which removes a fully-pancaked flap without
  // opening anything (its edges stay covered by the base sheet underneath).
  {
    let over = 0;
    for (const [k, m] of all) if (m > 2 && (bound.get(k) ?? 0) < m) over++;
    let guard = 0, peeled = 0;
    while (over > 0 && guard++ < 256) {
      let worst = -1, worstScore = 0;
      for (let t = v0; t < verts.length; t += 9) {
        let score = 0;
        for (let e = 0; e < 3; e++) {
          const i1 = t + e * 3, i2 = t + ((e + 1) % 3) * 3;
          const k = skey([verts[i1]!, verts[i1 + 1]!, verts[i1 + 2]!], [verts[i2]!, verts[i2 + 1]!, verts[i2 + 2]!]);
          if ((all.get(k) ?? 0) > 2) score++;
        }
        if (score > worstScore) { worstScore = score; worst = t; }
      }
      if (worst < 0) break;
      // remove the triangle: update edge counts, then splice it out of verts/faceIds
      for (let e = 0; e < 3; e++) {
        const i1 = worst + e * 3, i2 = worst + ((e + 1) % 3) * 3;
        const k = skey([verts[i1]!, verts[i1 + 1]!, verts[i1 + 2]!], [verts[i2]!, verts[i2 + 1]!, verts[i2 + 2]!]);
        all.set(k, (all.get(k) ?? 1) - 1);
      }
      verts.splice(worst, 9);
      faceIds.splice(f0 + (worst - v0) / 9, 1);
      peeled++;
      over = 0;
      for (const [k, m] of all) if (m > 2 && (bound.get(k) ?? 0) < m) over++;
      if (DBG && over === 0) console.error(`[grid] fid=${fid} self-overlap surgery: dropped ${guard} folded triangle(s)`);
    }
    if (peeled > 0) warn("folded-triangles-dropped", fid, `folded (u,v) projection — ${peeled} redundant triangle(s) peeled off the face`);
  }
  return true;
}

/** Watertight fallback for a PLANE face whose CDT parity flood folded (structural multi-loop
 * over-coverage): bridge every hole into the outer boundary and ear-clip in (u,v). Valid ONLY for a
 * plane (affine (u,v)->3D); each triangle is re-oriented by emitTri against the surface normal, so
 * the ear-clip winding is immaterial. Returns false if the clip produced nothing (leaves the face to
 * the caller's ribbon fallback). The raw clip has no interior vertices (every triangle a boundary-
 * to-boundary sliver), so it is refined to the face's edge target afterwards — boundary segments
 * stay verbatim, keeping the fill watertight against the neighbouring faces. */
function planeEarcutFill(
  surface: Surface, outer: { p3: Vec3[]; p2: P2[] }, holes: { p3: Vec3[]; p2: P2[] }[], fid: number,
  verts: number[], faceIds: number[], sign: number, targetEdge: number,
): boolean {
  const flat: number[] = [];
  const p3: Vec3[] = [];
  for (let i = 0; i < outer.p2.length; i++) { flat.push(outer.p2[i]![0], outer.p2[i]![1]); p3.push(outer.p3[i]!); }
  const holeStarts: number[] = [];
  for (const h of holes) {
    if (h.p2.length < 3) continue; // a degenerate hole contributes no area — skip it, keep the rest
    holeStarts.push(flat.length / 2);
    for (let i = 0; i < h.p2.length; i++) { flat.push(h.p2[i]![0], h.p2[i]![1]); p3.push(h.p3[i]!); }
  }
  const tri = earcut(flat, holeStarts.length ? holeStarts : null);
  if (tri.length === 0) return false;
  // Weld duplicate (u,v) points (self-touching loops) so the refinement sees each geometric edge
  // once — two index-distinct copies of one edge could otherwise split differently and open a
  // T-junction between their halves.
  const widx = new Map<string, number>();
  const remap = new Array<number>(flat.length / 2);
  const wpts: number[] = [];
  const wp3: Vec3[] = [];
  for (let i = 0; i < flat.length / 2; i++) {
    const key = `${flat[i * 2]},${flat[i * 2 + 1]}`;
    let w = widx.get(key);
    if (w === undefined) { w = wpts.length / 2; widx.set(key, w); wpts.push(flat[i * 2]!, flat[i * 2 + 1]!); wp3.push(p3[i]!); }
    remap[i] = w;
  }
  const wtris: number[] = [];
  for (let t = 0; t < tri.length; t += 3) {
    const a = remap[tri[t]!]!, b = remap[tri[t + 1]!]!, c = remap[tri[t + 2]!]!;
    if (a !== b && b !== c && c !== a) wtris.push(a, b, c);
  }
  // Boundary segments = consecutive loop points (with wrap): never split, shared with neighbours.
  const bnd = new Set<number>();
  const ends = [...holeStarts, flat.length / 2];
  let s = 0;
  for (const e of ends) {
    for (let i = s; i < e; i++) {
      const j = i + 1 < e ? i + 1 : s;
      if (remap[i]! !== remap[j]!) bnd.add(ekey(remap[i]!, remap[j]!));
    }
    s = e;
  }
  const nW = wp3.length;
  refineTriangulation(wpts, wtris, bnd, targetEdge);
  const v0 = verts.length, f0 = faceIds.length;
  // Original boundary vertices keep their shared 3D polyline samples verbatim; inserted interior
  // vertices evaluate exactly on the plane.
  const pt3 = (i: number): Vec3 => (i < nW ? wp3[i]! : surface.evaluate(wpts[i * 2]!, wpts[i * 2 + 1]!));
  const planeN = surface.normal(0, 0); // PLANE-only mesher: one constant orientation reference
  for (let t = 0; t < wtris.length; t += 3) {
    emitTri(verts, faceIds, pt3(wtris[t]!), pt3(wtris[t + 1]!), pt3(wtris[t + 2]!), fid, surface, sign, planeN);
  }
  if (verts.length === v0) { faceIds.length = f0; return false; }
  return true;
}

/** CDT core shared by the trimmed-patch and seam-split meshers: interior grid + graded size field +
 * Delaunay refinement over an explicit outer/hole boundary in continuous (u,v) coordinates. */
function gridCDT(
  surface: Surface, outer: { p3: Vec3[]; p2: P2[] }, holes: { p3: Vec3[]; p2: P2[] }[], fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
  allowAniso = true, noRefine = false, forceDeslit = false, requireClosed = false,
): boolean {
  let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
  for (const q of outer.p2) {
    if (q[0] < umin) umin = q[0]; if (q[0] > umax) umax = q[0];
    if (q[1] < vmin) vmin = q[1]; if (q[1] > vmax) vmax = q[1];
  }
  const umid = (umin + umax) / 2, vmid = (vmin + vmax) / 2;

  // Per-DIRECTION curvature bounds -> ANISOTROPIC scaling. A fillet bends across one parameter and
  // is straight along the other; meshing it isotropically at the across-the-bend step makes every
  // triangle span the full curvature step BOTH ways, and Delaunay's alternating diagonals shade
  // that as diamond moiré creases. Kernel meshers look smooth because their fillet triangles are
  // LONG along the ruling and short across it. Recover that: measure the normal curvature of each
  // iso-direction at the domain midpoint and compress the flatter axis in the scaled plane by
  // (its own allowed step / the face target), so near-unit Delaunay triangles map to
  // ruling-aligned anisotropic 3D triangles. Planes and spheres measure equal bounds in both
  // directions -> stretch 1 -> bit-identical isotropic behaviour.
  // Curvature is sampled on a 3x3 grid and the WORST (largest) curvature per direction wins: a
  // true fillet is single-curved at every sample so it keeps its full stretch, while a freeform
  // patch that is flat at the midpoint but bends elsewhere measures that bend and self-limits to
  // isotropic — no surface-kind special cases needed.
  const epsU = Math.max(1e-6, (umax - umin) / 64), epsV = Math.max(1e-6, (vmax - vmin) / 64);
  let kU = 0, kV = 0;
  const uMags: number[] = [], vMags: number[] = [];
  for (const fu of [0.15, 0.5, 0.85]) for (const fv of [0.15, 0.5, 0.85]) {
    const su = umin + (umax - umin) * fu, sv = vmin + (vmax - vmin) * fv;
    const Pc = surface.evaluate(su, sv);
    const uPp = surface.evaluate(su + epsU, sv), uPm = surface.evaluate(su - epsU, sv);
    const vPp = surface.evaluate(su, sv + epsV), vPm = surface.evaluate(su, sv - epsV);
    const d1u: Vec3 = [(uPp[0] - uPm[0]) / (2 * epsU), (uPp[1] - uPm[1]) / (2 * epsU), (uPp[2] - uPm[2]) / (2 * epsU)];
    const d1v: Vec3 = [(vPp[0] - vPm[0]) / (2 * epsV), (vPp[1] - vPm[1]) / (2 * epsV), (vPp[2] - vPm[2]) / (2 * epsV)];
    uMags.push(Math.hypot(d1u[0], d1u[1], d1u[2]));
    vMags.push(Math.hypot(d1v[0], d1v[1], d1v[2]));
    const nrm = normalize(cross(d1u, d1v));
    const normCurv = (Pp: Vec3, Pn: Vec3, d1: Vec3, e: number): number => {
      const d2: Vec3 = [(Pp[0] + Pn[0] - 2 * Pc[0]) / (e * e), (Pp[1] + Pn[1] - 2 * Pc[1]) / (e * e), (Pp[2] + Pn[2] - 2 * Pc[2]) / (e * e)];
      return Math.abs(dot(d2, nrm)) / Math.max(d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2], 1e-30);
    };
    kU = Math.max(kU, normCurv(uPp, uPm, d1u, epsU));
    kV = Math.max(kV, normCurv(vPp, vPm, d1v, epsV));
  }
  // Metric scales from the MEDIAN of the 3x3 samples, not the domain midpoint: on a ruled/loft
  // surface (e.g. a degree-1xN dome) the midpoint u-metric can be 1000x below the typical value,
  // and scaling by it squashes the whole boundary polygon into numeric collinearity — the CDT then
  // "succeeds" on a hair-thin sliver and emits a shattered face (StingStopp_4000_Base #12266).
  const median = (a: number[]): number => { const s = [...a].sort((x, y) => x - y); return s[s.length >> 1]!; };
  const uScale = Math.max(1e-9, median(uMags));
  const vScale = Math.max(1e-9, median(vMags));
  const stepOf = (kappa: number): number => {
    if (!(kappa > 1e-9)) return targetEdge;
    const R = 1 / kappa;
    // Floor only the chord term (see faceTarget): the angular requirement must hold as given.
    return Math.min(targetEdge, Math.max(targetEdge / 40, Math.sqrt(8 * chordTol * R)), R * normalDev);
  };
  const stepU = stepOf(kU), stepV = stepOf(kV);

  // Sanitize each boundary ring: collapse zero-width "out-and-back" spikes. A pinched strip face
  // (Shapr3D counterbore rims) traverses the same 3D curve twice through two edges whose (u,v)
  // images COINCIDE within the pinch tolerance — coincident constraint vertices are unrealisable
  // for the CDT (it shatters the face into unconstrained fragments). Two ring points count as the
  // same vertex ONLY when their 3D samples are weld-equal (the same 1e-6 quantisation weld() uses,
  // so both adjacent faces see the identical merge — a looser, per-face (u,v) tolerance would drop
  // a sample one neighbour keeps and open a T-junction) AND their (u,v) images coincide (a seam
  // vertex is 3D-equal but a period apart — not the same boundary vertex). An immediate backtrack
  // a-b-a collapses to a, repeatedly, so a whole coincident spike vanishes and the ring around it
  // stays intact; the enclosed region is unchanged (a spike bounds zero area). Clean loops have no
  // weld-coincident boundary vertices, so this is a no-op for them.
  const snapTol = 1e-3;
  const sanitize = (lp: { p3: Vec3[]; p2: P2[] }): { p3: Vec3[]; p2: P2[] } => {
    const n = lp.p2.length;
    if (n < 4) return lp;
    const ringKey = (i: number): string => {
      const p = lp.p3[i]!, q = lp.p2[i]!;
      return `${Math.round(p[0] / 1e-6)},${Math.round(p[1] / 1e-6)},${Math.round(p[2] / 1e-6)};${Math.round((q[0] * uScale) / snapTol)},${Math.round((q[1] * vScale) / snapTol)}`;
    };
    let keep: number[] = [];
    for (let i = 0; i < n; i++) {
      const k = ringKey(i);
      while (keep.length >= 2 && ringKey(keep[keep.length - 2]!) === k) keep.pop();
      if (keep.length >= 1 && ringKey(keep[keep.length - 1]!) === k) continue;
      keep.push(i);
    }
    // Same-direction repeats: a degenerate rim traversed TWICE (a "doubled loop", another Shapr3D
    // pinch artefact — two edges tracing the same 3D circle). If (nearly) the whole stretch between
    // two visits of one position repeats at that fixed offset, the loop walks the cycle twice —
    // drop one period so it is traversed once; the region boundary is unchanged. A genuine simple
    // boundary can pass NEAR itself but never retraces a whole stretch, so this cannot misfire.
    for (let pass = 0; pass < 4; pass++) {
      const m = keep.length;
      if (m < 6) break;
      const kOf = keep.map((i) => ringKey(i));
      const firstAt = new Map<string, number>();
      let cut: [number, number] | null = null;
      for (let i = 0; i < m && !cut; i++) {
        const j = firstAt.get(kOf[i]!);
        if (j === undefined) { firstAt.set(kOf[i]!, i); continue; }
        const r = i - j;
        if (r < 2) continue;
        let L = 0;
        while (L < r && i + L < m && kOf[j + L] === kOf[i + L]) L++;
        if (L >= Math.max(2, r - 1)) cut = [j, i]; // drop one full period [j, i)
      }
      if (!cut) break;
      keep = [...keep.slice(0, cut[0]), ...keep.slice(cut[1])];
    }
    if (keep.length === n) return lp;
    if (keep.length < 3) return lp;
    return { p3: keep.map((i) => lp.p3[i]!), p2: keep.map((i) => lp.p2[i]!) };
  };
  const preOuter = outer.p2.length, preHoles = holes.reduce((s, h) => s + h.p2.length, 0);
  outer = sanitize(outer);
  holes = holes.map(sanitize);
  if (DBG && (outer.p2.length !== preOuter || holes.reduce((s, h) => s + h.p2.length, 0) !== preHoles)) {
    console.error(`[grid] fid=${fid} sanitize: outer ${preOuter}->${outer.p2.length} holes ${preHoles}->${holes.reduce((s, h) => s + h.p2.length, 0)}`);
  }
  const holeP2 = holes.map((h) => h.p2);
  const inRegion = (p: P2): boolean => pointInPoly(p, outer.p2) && !holeP2.some((h) => pointInPoly(p, h));

  // Curvature-adaptive interior density so the initial mesh is already fine on curved faces.
  const target = faceTarget(surface, targetEdge, chordTol, normalDev, umid, vmid);
  // Anisotropy-adjusted axis scales for the CDT plane (capped so triangles never exceed ~6:1 —
  // beyond that slivers start to cost more numerically than the shading gains). uScale/vScale stay
  // the TRUE metric for anything with a physical tolerance (the sanitiser's snap key above).
  const ANISO_CAP = allowAniso ? 6 : 1;
  const uSc = uScale / Math.min(ANISO_CAP, Math.max(1, stepU / target));
  const vSc = vScale / Math.min(ANISO_CAP, Math.max(1, stepV / target));
  const nU = Math.min(1200, Math.max(1, Math.round(((umax - umin) * uSc) / target)));
  const nV = Math.min(1200, Math.max(1, Math.round(((vmax - vmin) * vSc) / target)));

  const allP2: P2[] = [];
  const allP3: Vec3[] = [];
  const pushLoop = (lp: { p3: Vec3[]; p2: P2[] }): number[] => {
    const start = allP2.length;
    for (let i = 0; i < lp.p2.length; i++) { allP2.push(lp.p2[i]!); allP3.push(lp.p3[i]!); }
    return Array.from({ length: lp.p2.length }, (_, i) => start + i);
  };
  const outerIdx = pushLoop(outer);
  const holeIdx = holes.map(pushLoop);

  // Everything below works in METRIC-SCALED parameter space (scale u,v by the local surface metric
  // so 2D distance ≈ 3D arc length, then divide by the per-direction anisotropy): a plain Delaunay
  // there yields 3D triangles isotropic in "allowed steps" — square on a plane, ruling-elongated on
  // a fillet — and a cylinder (u spans 2π but R·2π in 3D) stops slivering at its seam.
  const SX = (u: number): number => u * uSc, SY = (v: number): number => v * vSc;
  // Boundary segments in scaled space, hashed, drive (a) a graded SIZE FIELD — size grows from each
  // edge's own length outward (so a tight-fillet edge shared with this flat face stays small near it
  // and coarsens away), capped at the face target — and (b) a "too close to the boundary" test.
  // Segment positions live in the (anisotropy-compressed) CDT space, but the grading LENGTH is the
  // segment's TRUE metric length: a ruling-aligned boundary edge measures short in compressed
  // space, and grading from that phantom "fine feature" would crowd micro-triangles along every
  // fillet rail.
  const sseg: [number, number, number, number, number][] = [];
  for (const idx of [outerIdx, ...holeIdx]) for (let i = 0; i < idx.length; i++) {
    const a = allP2[idx[i]!]!, b = allP2[idx[(i + 1) % idx.length]!]!;
    const ax = SX(a[0]), ay = SY(a[1]), bx = SX(b[0]), by = SY(b[1]);
    sseg.push([ax, ay, bx, by, Math.hypot((b[0] - a[0]) * uScale, (b[1] - a[1]) * vScale)]);
  }
  const csz = Math.max(target, 1e-6);
  const hkey = (ix: number, iy: number): number => Math.imul(ix, 73856093) ^ Math.imul(iy, 19349663);
  const segHash = new Map<number, number[]>();
  for (let i = 0; i < sseg.length; i++) { const k = hkey(Math.floor((sseg[i]![0] + sseg[i]![2]) / 2 / csz), Math.floor((sseg[i]![1] + sseg[i]![3]) / 2 / csz)); (segHash.get(k) ?? segHash.set(k, []).get(k)!).push(i); }
  const floor = target / 40;
  const sizeDist = (sx: number, sy: number): [number, number] => {
    // Out-of-range query = a degenerate triangle's circumcenter blew up. Without this guard the
    // cell scan below never terminates: beyond 2^53, gx++ no longer changes gx, so even a FINITE
    // 1e300 circumcenter loops forever (ABC 00004166 fid=786) — Infinity likewise.
    if (!(Math.abs(sx) < 1e12 && Math.abs(sy) < 1e12)) return [floor, Infinity];
    // Size field = min of (a) the LOCAL curvature target — so a face whose curvature varies is
    // refined where it actually bends, not just at the patch midpoint — and (b) the boundary-graded
    // size, growing from each edge's own length outward. Capped at the face target, floored to bound.
    let size = faceTarget(surface, targetEdge, chordTol, normalDev, sx / uSc, sy / vSc);
    let dist = Infinity, cx = Math.floor(sx / csz), cy = Math.floor(sy / csz);
    for (let gx = cx - 2; gx <= cx + 2; gx++) for (let gy = cy - 2; gy <= cy + 2; gy++) {
      const arr = segHash.get(hkey(gx, gy)); if (!arr) continue;
      for (const i of arr) {
        const [ax, ay, bx, by, ln] = sseg[i]!, ex = bx - ax, ey = by - ay, l2 = ex * ex + ey * ey;
        let tt = l2 > 0 ? ((sx - ax) * ex + (sy - ay) * ey) / l2 : 0; tt = tt < 0 ? 0 : tt > 1 ? 1 : tt;
        const d = Math.hypot(sx - (ax + tt * ex), sy - (ay + tt * ey));
        if (d < dist) dist = d;
        const s = ln + 0.45 * d; if (s < size) size = s;
      }
    }
    return [Math.max(floor, size), dist];
  };
  // Base interior: a uniform grid at the (coarse) face target. Refinement adds the fine detail.
  const interiorIdx: number[] = [];
  const tryAdd = (u: number, v: number): void => {
    if (!inRegion([u, v])) return;
    const [sz, dist] = sizeDist(SX(u), SY(v));
    if (dist < 0.3 * sz) return; // hugs a boundary edge -> would make it un-enforceable
    interiorIdx.push(allP2.length); allP2.push([u, v]); allP3.push(surface.evaluate(u, v));
  };
  for (let i = 1; i < nU; i++) for (let j = 1; j < nV; j++) tryAdd(umin + ((umax - umin) * i) / nU, vmin + ((vmax - vmin) * j) / nV);

  const cdtPts: P2[] = allP2.map(([u, v]) => [SX(u), SY(v)]);
  // KEYHOLE DE-SLIT candidate. A hole joined to the outer boundary by a zero-width corridor
  // arrives as ONE loop that walks the corridor out, circles the hole, and walks the same
  // samples back — every corridor point is a 3D-bitwise pair (Meanwell's sphere caps: a round
  // vent behind a 2-segment corridor). The doubled corridor corrupts the CDT, and welding it
  // would be WRONG (a single corridor wall would parity-split the face interior); removing the
  // corridor and emitting the enclosed outline as the hole it is meshes cleanly. BUT the same
  // coincident-run signature also describes slit rings the equivalence machinery already meshes
  // watertight, and no cover-space geometry test separates them reliably (Z Bearing Block's
  // seam-lens ring reads as a perfect interior keyhole). So this only BUILDS the candidate —
  // the decision is OUTCOME-VERIFIED below: triangulate with the original loops first and adopt
  // the de-slit variant only when the original demonstrably fails to cover its own boundary.
  let altOuter: number[] | null = null;
  const altHoles: number[][] = [];
  {
    const key3 = (v: number): string => `${allP3[v]![0]},${allP3[v]![1]},${allP3[v]![2]}`;
    const near2i = (a: number, b: number): boolean =>
      Math.abs(cdtPts[a]![0] - cdtPts[b]![0]) <= 1e-6 && Math.abs(cdtPts[a]![1] - cdtPts[b]![1]) <= 1e-6;
    const inPoly = (p: P2, poly: number[]): boolean => {
      let inside = false;
      for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const A = cdtPts[poly[i]!]!, B = cdtPts[poly[j]!]!;
        if ((A[1] > p[1]) !== (B[1] > p[1]) && p[0] < ((B[0] - A[0]) * (p[1] - A[1])) / (B[1] - A[1]) + A[0]) inside = !inside;
      }
      return inside;
    };
    let cur = outerIdx;
    for (let guard = 0; guard < 8; guard++) {
      const n = cur.length;
      if (n < 8) break;
      const firstAt = new Map<string, number>();
      let done = true;
      for (let jj = 0; jj < n && done; jj++) {
        const k = key3(cur[jj]!);
        const ii = firstAt.get(k);
        if (ii === undefined) { firstAt.set(k, jj); continue; }
        if (!near2i(cur[ii]!, cur[jj]!)) continue;
        // grow the corridor to its maximal extent: outward to the mouth, inward to the far end
        let i = ii, j = jj;
        while (i - 1 >= 0 && j + 1 < n
          && key3(cur[i - 1]!) === key3(cur[j + 1]!) && near2i(cur[i - 1]!, cur[j + 1]!)) { i--; j++; }
        let m = 0;
        while (i + m + 1 < j - m - 1
          && key3(cur[i + m + 1]!) === key3(cur[j - m - 1]!)
          && near2i(cur[i + m + 1]!, cur[j - m - 1]!)) m++;
        if (m === 0) continue; // point pinch — the weld below owns it
        const mid = cur.slice(i + m, j - m); // far pinch point once + enclosed outline
        const wrap = [...cur.slice(0, i + 1), ...cur.slice(j + 1)];
        if (mid.length < 3 || wrap.length < 3) continue;
        // Which piece is the hole? The loop may START on either side of the corridor, so decide
        // by containment (the piece whose far point lies inside the other is the enclosed one).
        const farOf = (p: number[], at: number): P2 => cdtPts[p[(at + (p.length >> 1)) % p.length]!]!;
        let outer: number[], hole: number[];
        if (inPoly(farOf(mid, 0), wrap)) { outer = wrap; hole = mid; }
        else if (inPoly(farOf(wrap, i), mid)) { outer = mid; hole = wrap; }
        else continue; // neither encloses the other — leave alone
        if (DBG) console.error(`[deslit] fid=${fid} candidate corridor (${i},${j}) m=${m} -> outer=${outer.length} hole=${hole.length}`);
        altHoles.push(hole);
        cur = outer;
        done = false; // rescan — the loop may hide further keyholes
      }
      if (done) break;
    }
    if (altHoles.length > 0) altOuter = cur;
  }

  // TANGENT-PINCH WELD. A hole tangent to the boundary arrives as ONE loop whose tangency vertex
  // appears twice, bitwise-equal in (u,v) AND 3D. Inserting the second copy exactly ON the first
  // corrupts the incremental triangulation (zero-area fans along any collinear boundary run
  // through the pinch: LCD Case Hinge's tangent hinge bores at coarse sampling lose the loop's
  // closing constraint, invisible to every equivalence rescue because the covering edge is
  // LONGER than the constraint). Alias the later occurrence to the first so the polygon pinches
  // at one vertex index. ISOLATED pinches only — a coincident RUN is a slit's out-and-back legs,
  // whose distinct indices the parity flood needs as separate walls (Z Bearing Block's ring).
  const weldPinches = (idx: number[]): void => {
    const n = idx.length;
    if (n < 4) return;
    const seen = new Map<string, number>(); // exact (u,v)+3D key -> position of first occurrence
    for (let i = 0; i < n; i++) {
      const v = idx[i]!;
      const k = `${cdtPts[v]![0]},${cdtPts[v]![1]},${allP3[v]![0]},${allP3[v]![1]},${allP3[v]![2]}`;
      const first = seen.get(k);
      if (first === undefined) { seen.set(k, i); continue; }
      const co = (x: number, y: number): boolean =>
        cdtPts[idx[x]!]![0] === cdtPts[idx[y]!]![0] && cdtPts[idx[x]!]![1] === cdtPts[idx[y]!]![1];
      const isolated = !co((first + n - 1) % n, (i + 1) % n) && !co((first + 1) % n, (i + n - 1) % n);
      if (isolated) idx[i] = idx[first]!;
    }
  };
  for (const idx of [outerIdx, ...holeIdx]) weldPinches(idx);
  if (altOuter) { weldPinches(altOuter); for (const h of altHoles) weldPinches(h); }
  if (DBG && Number(process.env.MESHSTEP_DUMPCDT) === fid) {
    console.error(`@@CDTDUMP@@${JSON.stringify({ fid, outerIdx, holeIdx, cdtPts, p3: allP3 })}@@END@@`);
  }
  const cdtOut: { missing: number; rescue?: string } = { missing: 0 };
  let tris = constrainedTriangulate(cdtPts, [outerIdx, ...holeIdx], interiorIdx, cdtOut);
  // OUTCOME-VERIFIED DE-SLIT. Adopt the keyhole candidate only when the original loops
  // demonstrably fail: count boundary segments absent from the triangulation's edges (the exact
  // signature of a leaking face — the mate samples the same shared polyline and finds no twin).
  // Z Bearing Block's slit ring covers all its boundary via the equivalence machinery (0 defects
  // -> candidate discarded); Meanwell's keyhole caps leave 7+ segments uncovered -> the de-slit
  // variant, which covers everything, wins.
  if (altOuter) {
    // Coverage is judged by 3D IDENTITY, not point index: an equivalence-realised constraint is
    // covered by its duplicate twin's edge (same 3D points, different indices — Z Bearing Block's
    // slit legs), while a genuinely missing segment has no 3D twin either.
    const gid = new Map<string, number>();
    const g = (v: number): number => {
      const k = `${allP3[v]![0]},${allP3[v]![1]},${allP3[v]![2]}`;
      let id = gid.get(k);
      if (id === undefined) { id = gid.size; gid.set(k, id); }
      return id;
    };
    // Multiplicity-aware: a segment the loop walks TWICE (a zero-width corridor/slit) must be
    // covered by two triangle edges — one per side. Z Bearing Block's slit genuinely has both
    // sides (0 defects); Meanwell's leaking keyhole covers only one leg (defect per segment).
    const coverDefects = (T: [number, number, number][], loopsSet: number[][]): number => {
      const key = (x: number, y: number): number => { const a = g(x), b = g(y); return a < b ? a * 0x8000000 + b : b * 0x8000000 + a; };
      const have = new Map<number, number>();
      for (const [x, y, z] of T) for (const k of [key(x, y), key(y, z), key(z, x)]) have.set(k, (have.get(k) ?? 0) + 1);
      const want = new Map<number, number>();
      for (const lp of loopsSet) for (let i = 0; i < lp.length; i++) {
        const x = lp[i]!, y = lp[(i + 1) % lp.length]!;
        if (g(x) !== g(y)) { const k = key(x, y); want.set(k, (want.get(k) ?? 0) + 1); }
      }
      let bad = 0;
      for (const [k, w] of want) bad += Math.max(0, w - (have.get(k) ?? 0));
      return bad;
    };
    const d0 = forceDeslit ? Infinity : coverDefects(tris, [outerIdx, ...holeIdx]);
    if (d0 > 0) {
      const altOut: { missing: number; rescue?: string } = { missing: 0 };
      const altTris = constrainedTriangulate(cdtPts, [altOuter, ...altHoles], interiorIdx, altOut);
      // Adopt only a FULLY clean alternative: a seam-straddling slit ring's de-slit variant is
      // always partially uncovered (the ring lands outside the outer on the cover), while a
      // genuine keyhole's variant covers everything.
      const d1 = forceDeslit ? 0 : coverDefects(altTris, [altOuter, ...altHoles]);
      if (DBG) console.error(`[deslit] fid=${fid} outcome: original defects=${d0}, de-slit defects=${d1} -> ${d1 === 0 && d1 < d0 ? "ADOPT" : "keep original"}`);
      if (d1 === 0 && d1 < d0) {
        outerIdx.length = 0;
        outerIdx.push(...altOuter);
        holeIdx.length = 0;
        for (const h of altHoles) holeIdx.push(h);
        tris = altTris;
        cdtOut.missing = altOut.missing;
        cdtOut.rescue = altOut.rescue;
      }
    }
  }
  if (DBG) console.error(`[grid] fid=${fid} nU=${nU} nV=${nV} uSc=${uSc.toExponential(2)} vSc=${vSc.toExponential(2)} boundary=${outerIdx.length}+${holeIdx.reduce((s, h) => s + h.length, 0)} interior=${interiorIdx.length} -> cdt tris=${tris.length}${cdtOut.missing ? ` MISSING=${cdtOut.missing}` : ""}`);
  // Delaunay refinement: insert the circumcentre of any triangle whose circumdiameter exceeds the
  // local size field — this grades the mesh from the fine boundary into the interior (fillet runs
  // into chamfer) and, being Delaunay, keeps the new triangles well-shaped. Capped at a multiple of
  // the base grid: well-behaved faces converge long before it, but a skewed B-spline patch (whose
  // diagonal-metric triangles always look oversized) would otherwise refine to millions of points.
  const cap = 200 + (interiorIdx.length + outerIdx.length) * 6;
  // Each refinement pass re-runs the whole CDT from scratch; on a very dense boundary (shared
  // thread rails, tens of thousands of samples) that multiplies an already-fine mesh for minutes.
  // Those boundaries are far below the size field everywhere — refinement has nothing real to add.
  const nBoundaryPts = outerIdx.length + holeIdx.reduce((s, h) => s + h.length, 0);
  const maxIter = noRefine ? 0 : nBoundaryPts > 20000 ? 0 : nBoundaryPts > 8000 ? 1 : 4;
  // A refinement iteration may never trade a fully-constrained triangulation for a rescued one.
  // Inserted circumcentres can push the re-run CDT into a collinear degeneracy (a straight boundary
  // rail on a coarse periodic band) where a constraint becomes unrealisable; the ear-clip rescue
  // then fills the ring with zero-2D-area slivers along the collinear rail whose 3D images are
  // CHORDS through the surface — and the next iteration refines the chords, compounding them
  // (ov_pokal's scalloped goblet band: one face emitted 750× its own area at 106mm deviation).
  // Constraint loss is detected via the CDT's missing count: roll back to the previous
  // triangulation and stop refining. Orphaned points from the discarded iteration stay in the
  // arrays but are unreferenced. An INITIAL missing>0 keeps the rescue as before — for a
  // metric-collapsed boundary it is the only watertight fill available.
  let prevMissing = cdtOut.missing;
  let prevRescue = cdtOut.rescue;
  // Exact-duplicate guard for refinement inserts, persistent across iterations. The per-iteration
  // dedup below hashes by the POSITION-DEPENDENT local size, so two ulp-separated circumcentres —
  // every grid rectangle is cyclic, its two diagonal triangles propose the SAME cell centre — that
  // straddle an rc-lattice border get different bucket sizes and are never compared. The CDT then
  // carries coincident vertices whose triangles land on identical quantised 3D edges 4×: a "fold"
  // that is pure bookkeeping (cat-napkin / bottle-cage ripple fields). Cell = 1e-3 of the face
  // target: 25× below the size-field floor (never rejects a legitimate insert), orders of
  // magnitude above fp noise.
  const dupCell = 1e-3 * csz;
  const dupKey = (x: number, y: number): number => hkey(Math.round(x / dupCell), Math.round(y / dupCell));
  const dupSeen = new Set<number>();
  const dupHas = (x: number, y: number): boolean => {
    const cx = Math.round(x / dupCell), cy = Math.round(y / dupCell);
    for (let gx = cx - 1; gx <= cx + 1; gx++) for (let gy = cy - 1; gy <= cy + 1; gy++) {
      if (dupSeen.has(hkey(gx, gy))) return true;
    }
    return false;
  };
  for (const p of cdtPts) dupSeen.add(dupKey(p[0], p[1]));
  for (let iter = 0; iter < maxIter && interiorIdx.length < cap; iter++) {
    const fresh: [number, number, number][] = []; // (px, py, local size) — sizeDist carried to the dedup pass
    for (const [a, b, c] of tris) {
      const A = cdtPts[a]!, B = cdtPts[b]!, C = cdtPts[c]!;
      const d = 2 * (A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1]));
      if (Math.abs(d) < 1e-12) continue;
      const a2 = A[0] * A[0] + A[1] * A[1], b2 = B[0] * B[0] + B[1] * B[1], c2 = C[0] * C[0] + C[1] * C[1];
      let px = (a2 * (B[1] - C[1]) + b2 * (C[1] - A[1]) + c2 * (A[1] - B[1])) / d;
      let py = (a2 * (C[0] - B[0]) + b2 * (A[0] - C[0]) + c2 * (B[0] - A[0])) / d;
      // Degenerate (near-zero-area) triangle: the circumcenter blows up. Beyond ~2^53 the integer
      // cell loops in sizeDist/dupHas cannot terminate (x+1 === x), so bound the magnitude — 1e12
      // in metric-scaled units is far beyond any real face.
      if (!(Math.abs(px) < 1e12 && Math.abs(py) < 1e12)) continue;
      const r = Math.hypot(px - A[0], py - A[1]);
      let [sz, dist] = sizeDist(px, py);
      // Refine where the circumradius exceeds 0.65× the local size: fills the graded band near a
      // fine boundary (fillet-into-chamfer), follows intra-face curvature the base grid under-sampled,
      // and improves Delaunay quality generally. The remesh later coarsens over-dense flat regions.
      if (r <= sz * 0.65) continue;
      let u = px / uSc, v = py / vSc;
      if (!inRegion([u, v])) {
        px = (A[0] + B[0] + C[0]) / 3; py = (A[1] + B[1] + C[1]) / 3; u = px / uSc; v = py / vSc;
        if (!inRegion([u, v])) continue;
        [sz, dist] = sizeDist(px, py); // moved to the centroid — re-query there
      }
      if (dist < 0.5 * sz) continue;
      fresh.push([px, py, sz]);
    }
    if (!fresh.length) break;
    // Dedup new points that fall within ~half a local size of each other (avoid over-insertion).
    const acc = new Map<number, [number, number][]>();
    let added = false;
    for (const [px, py, sz] of fresh) {
      if (dupHas(px, py)) continue;
      const hx = Math.floor(px / sz), hy = Math.floor(py / sz);
      let ok = true;
      for (let gx = hx - 1; gx <= hx + 1 && ok; gx++) for (let gy = hy - 1; gy <= hy + 1 && ok; gy++) {
        for (const [qx, qy] of acc.get(hkey(gx, gy)) ?? []) if ((px - qx) ** 2 + (py - qy) ** 2 < (0.5 * sz) ** 2) ok = false;
      }
      if (!ok) continue;
      (acc.get(hkey(hx, hy)) ?? acc.set(hkey(hx, hy), []).get(hkey(hx, hy))!).push([px, py]);
      dupSeen.add(dupKey(px, py));
      interiorIdx.push(allP2.length); allP2.push([px / uSc, py / vSc]); allP3.push(surface.evaluate(px / uSc, py / vSc)); cdtPts.push([px, py]);
      added = true;
    }
    if (!added) break;
    const prevTris = tris;
    tris = constrainedTriangulate(cdtPts, [outerIdx, ...holeIdx], interiorIdx, cdtOut);
    if (cdtOut.missing > prevMissing) {
      if (DBG) console.error(`[grid] fid=${fid} refinement iter ${iter} lost ${cdtOut.missing - prevMissing} constraint(s) — rolled back`);
      tris = prevTris;
      break;
    }
    prevMissing = cdtOut.missing;
    prevRescue = cdtOut.rescue;
  }
  // requireClosed: the caller knows this boundary MUST fully realise (a winding outer on a
  // periodic surface — its period-spanning closing chord is unenforceable exactly when the lift
  // is the wrong topology, and the rescue fill then emits a guaranteed-leaky region). Return
  // false instead so the dispatch chain falls through to the band/region meshers that own
  // winding rims. Nothing has been emitted into verts yet at this point.
  if (requireClosed && prevMissing > 0) {
    if (DBG) console.error(`[grid] fid=${fid} requireClosed: ${prevMissing} constraint(s) unrealised — face declined`);
    return false;
  }
  // Surface the CDT's own distress to the import diagnostics: unenforceable boundary constraints
  // mean the face region was reconstructed rather than derived (the class behind every remaining
  // watertight-but-wrong result), and a rescue label means the parity flood was out-voted even
  // with all constraints equivalence-realised. prevMissing/prevRescue describe the triangulation
  // actually kept (a rolled-back refinement run's counts are discarded with its triangles).
  if (prevMissing > 0) warn("cdt-degenerate-boundary", fid, `${prevMissing} boundary constraint(s) unenforceable — region rebuilt by rescue fill`);
  else if (prevRescue) warn("cdt-degenerate-boundary", fid, `degenerate boundary (duplicate/collinear vertices) — region rebuilt by ${prevRescue} fill`);
  // Delaunay picks each quad's diagonal by circumcircle in the (flat) scaled plane, which on a
  // curved face alternates diagonal direction from quad to quad — and every alternation is a
  // shading crease (the diamond moiré on fillets). Kernel meshers look smooth because their
  // diagonals run consistently ALONG the ruling. Recover that property without knowing where the
  // rulings are: greedily flip interior diagonals whenever the flip brings the two adjacent
  // triangles' 3D normals closer together. On a single-curved surface the dihedral is minimal
  // exactly when the diagonal follows the ruling, so the flips converge to kernel-style rows; on a
  // plane every choice ties (no-op); constraint/boundary edges are never flipped, so the shared
  // edge samples — and watertightness — are untouched.
  const triNormal = (a: number, b: number, c: number): [number, number, number, number] => {
    const A = allP3[a]!, B = allP3[b]!, C = allP3[c]!;
    const ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2];
    const vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
    const n: Vec3 = [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx];
    const l = Math.hypot(n[0], n[1], n[2]);
    const s = l || 1;
    return [n[0] / s, n[1] / s, n[2] / s, l];
  };
  const area2s = (a: number, b: number, c: number): number => {
    const A = cdtPts[a]!, B = cdtPts[b]!, C = cdtPts[c]!;
    return (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]);
  };
  const NPTS = cdtPts.length;
  const ekey = (i: number, j: number): number => (i < j ? i * NPTS + j : j * NPTS + i);
  const constrained = new Set<number>();
  for (const idx of [outerIdx, ...holeIdx]) for (let i = 0; i < idx.length; i++) constrained.add(ekey(idx[i]!, idx[(i + 1) % idx.length]!));
  // Boundary vertices are pushed before any interior point, so "is boundary" is an index compare.
  // A flipped diagonal may not SHORTCUT between two nearby vertices of the same boundary ring: a
  // face that abuts (or mirrors) a twin along that boundary — a periodic seam side, a symmetric
  // half-cone pair — sees its twin make the identical ruling-aligned shortcut over the SAME shared
  // samples, and the coincident duplicated edge welds non-manifold. Ring-DISTANT joins stay
  // allowed: on a narrow two-rail fillet (no interior points at all) rail-to-rail flips are the
  // ONLY way to fix the alternating-diagonal moiré, and opposite rails sit far apart on the ring.
  const nBoundary = outerIdx.length + holeIdx.reduce((s, h) => s + h.length, 0);
  const ringOf = new Int32Array(nBoundary), ringPos = new Int32Array(nBoundary), ringLen: number[] = [];
  {
    let r = 0;
    for (const idx of [outerIdx, ...holeIdx]) {
      for (let i = 0; i < idx.length; i++) { ringOf[idx[i]!] = r; ringPos[idx[i]!] = i; }
      ringLen.push(idx.length); r++;
    }
  }
  const boundaryShortcut = (i: number, j: number): boolean => {
    if (i >= nBoundary || j >= nBoundary) return false;
    if (ringOf[i] !== ringOf[j]) return false;
    const L = ringLen[ringOf[i]!]!;
    const d = Math.abs(ringPos[i]! - ringPos[j]!);
    return Math.min(d, L - d) <= 4;
  };
  for (let pass = 0; pass < 6; pass++) {
    const use = new Map<number, [number, number][]>(); // edge -> [triIndex, oppositeVertex]
    for (let ti = 0; ti < tris.length; ti++) {
      const t = tris[ti]!;
      for (let e = 0; e < 3; e++) {
        const k = ekey(t[e]!, t[(e + 1) % 3]!);
        (use.get(k) ?? use.set(k, []).get(k)!).push([ti, t[(e + 2) % 3]!]);
      }
    }
    const dirty = new Set<number>();
    let flips = 0;
    for (const [k, ts] of use) {
      if (ts.length !== 2 || constrained.has(k)) continue;
      const [[t1, c1], [t2, c2]] = ts as [[number, number], [number, number]];
      if (dirty.has(t1) || dirty.has(t2)) continue;
      if (boundaryShortcut(c1, c2)) continue;
      let a = Math.floor(k / NPTS), b = k % NPTS;
      // The key sorts the edge's endpoints; recover the direction it runs in t1 (c1 sits to the
      // LEFT of t1's directed edge, since triangles are CCW) so the flipped pair stays CCW.
      if (area2s(a, b, c1) < 0) { const tmp = a; a = b; b = tmp; }
      // Flip (a,b) -> (c1,c2). Both new triangles must stay CCW with non-vanishing area in the
      // scaled plane (else the quad is non-convex and the flip would fold it).
      const eps = 1e-9 * (Math.abs(area2s(...tris[t1]!)) + Math.abs(area2s(...tris[t2]!)));
      const nA1 = area2s(c1, a, c2), nA2 = area2s(c2, b, c1);
      if (nA1 <= eps || nA2 <= eps) continue;
      const o1 = triNormal(...tris[t1]!), o2 = triNormal(...tris[t2]!);
      const f1 = triNormal(c1, a, c2), f2 = triNormal(c2, b, c1);
      // Fold guards: a quad can be convex in the 2D plane yet FOLD in 3D near a degenerate or
      // pinched patch (zero-area triangles make the dihedral criterion pure noise, and a folded
      // pair overlaps its neighbours into non-manifold edges after welding). Require the flipped
      // triangles to be non-degenerate in 3D and their normals to stay on the originals' side.
      if (f1[3]! < 1e-6 * (o1[3]! + o2[3]!) || f2[3]! < 1e-6 * (o1[3]! + o2[3]!)) continue;
      if (f1[0] * o1[0] + f1[1] * o1[1] + f1[2] * o1[2] <= 0 || f2[0] * o2[0] + f2[1] * o2[1] + f2[2] * o2[2] <= 0) continue;
      if (f1[0] * o2[0] + f1[1] * o2[1] + f1[2] * o2[2] <= 0 || f2[0] * o1[0] + f2[1] * o1[1] + f2[2] * o1[2] <= 0) continue;
      // SHAPE GUARD: a flip may choose the smoother diagonal of a quad, but it may not butcher
      // triangle quality for it. The greedy normal criterion alone happily un-Delaunays a gently
      // curved patch into needle fans — the normal gain is microscopic (dihedrals there are near
      // zero) while the 2D aspect explodes 1000-fold, which reads as sliver stripes in a slicer's
      // wireframe. Allow a flip only while the flipped pair's worst aspect (in the scaled CDT
      // plane, where near-unit is ideal) stays below 4, or does not worsen an already-bad quad.
      const asp2 = (i: number, j: number, l: number): number => {
        const A = cdtPts[i]!, B = cdtPts[j]!, C = cdtPts[l]!;
        const e1 = (B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2;
        const e2 = (C[0] - B[0]) ** 2 + (C[1] - B[1]) ** 2;
        const e3 = (A[0] - C[0]) ** 2 + (A[1] - C[1]) ** 2;
        const ar = Math.abs((B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]));
        return ar > 1e-30 ? Math.max(e1, e2, e3) / ar : 1e9;
      };
      const curAsp = Math.max(asp2(...tris[t1]!), asp2(...tris[t2]!));
      const flpAsp = Math.max(asp2(c1, a, c2), asp2(c2, b, c1));
      if (flpAsp > Math.max(4, curAsp)) continue;
      // Score the WHOLE 1-ring, not just the flipped pair against each other: each configuration's
      // smoothness is the summed normal agreement across the diagonal AND the quad's four outer
      // edges (whose neighbour triangles stay fixed). A pair-only criterion happily makes two
      // triangles coplanar while shearing against the rail neighbours — adjacent quads then flip
      // inconsistently and the fillet silhouette saw-tooths.
      const nbr = (x: number, y: number, self: number): [number, number, number, number] | null => {
        const arr = use.get(ekey(x, y));
        if (!arr || arr.length !== 2) return null;
        const other = arr[0]![0] === self ? arr[1]![0] : arr[0]![0];
        if (dirty.has(other)) return null;
        return triNormal(...tris[other]!);
      };
      const dotN = (p: [number, number, number, number] | null, q: [number, number, number, number]): number =>
        p ? p[0] * q[0] + p[1] * q[1] + p[2] * q[2] : 0;
      const nAC1 = nbr(a, c1, t1), nC1B = nbr(c1, b, t1), nBC2 = nbr(b, c2, t2), nC2A = nbr(c2, a, t2);
      const cur = dotN(o1, o2) + dotN(nAC1, o1) + dotN(nC1B, o1) + dotN(nBC2, o2) + dotN(nC2A, o2);
      const flp = dotN(f1, f2) + dotN(nAC1, f1) + dotN(nC1B, f2) + dotN(nBC2, f2) + dotN(nC2A, f1);
      if (flp <= cur + 1e-9) continue;
      tris[t1] = [c1, a, c2]; tris[t2] = [c2, b, c1];
      dirty.add(t1); dirty.add(t2); flips++;
    }
    if (!flips) break;
  }
  if (DBG) console.error(`[grid] fid=${fid} final tris=${tris.length} pts=${cdtPts.length}`);
  // Every vertex carries its (u,v) — orient by the surface normal at the triangle's uv centroid
  // instead of inverse-mapping the 3D centroid (a Newton solve per triangle on B-splines). The
  // pcurves are seam-unwrapped into one continuous chart, so averaging u/v across a triangle is
  // safe; evaluate/normal wrap periodic coordinates themselves.
  for (const [a, b, c] of tris) {
    const A2 = allP2[a]!, B2 = allP2[b]!, C2 = allP2[c]!;
    const nrm = surface.normal((A2[0] + B2[0] + C2[0]) / 3, (A2[1] + B2[1] + C2[1]) / 3);
    emitTri(verts, faceIds, allP3[a]!, allP3[b]!, allP3[c]!, fid, surface, sign, nrm);
  }
  return tris.length > 0;
}

/**
 * Seam-split ("unroll") mesher for a PERIODIC surface whose full-period rims arrive as SEPARATE
 * loops (no seam edge joining them) AND which also carries interior window holes — a cylindrical /
 * conical pocket wall with cut-outs, as some kernels emit. The band mesher bails (it can't subtract
 * the windows) and the param grid bails (each bare rim projects to a zero-area horizontal line, so
 * there's no enclosing outer loop). Here the periodic domain is cut at a seam chosen to miss every
 * window, giving a rectangular (u,v) region: bottom rim along v=vlo, top rim along v=vhi, the two
 * seam sides identical in 3D (so they weld into a watertight seam), windows as ordinary holes. The
 * shared rim samples are reused verbatim, so the seam with the neighbouring cap/plane stays tight.
 * Returns false (emitting nothing) unless there are exactly two full-period rims.
 */
function tessellatePeriodicUnroll(
  surface: Surface, loops: BLoop[], sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
): boolean {
  const collect = (lp: BLoop): Vec3[] => {
    const p3: Vec3[] = [];
    for (const oe of lp.edges) {
      const base = sampled.get(oe.edgeId); if (!base) continue;
      const poly = oe.orient ? base : base.slice().reverse();
      for (let i = 0; i < poly.length - 1; i++) p3.push(poly[i]!);
    }
    return p3;
  };
  // c = the wrapping ("around") coordinate; stackC = the one the rims are stacked along.
  const attempt = (c: 0 | 1): boolean => {
    const stackC: 0 | 1 = c === 0 ? 1 : 0;
    const period = (c === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI;
    const proj = loops.map((lp) => {
      const p3 = collect(lp);
      if (p3.length < 2) return null;
      const p2: P2[] = [];
      let hu: number | undefined, hv: number | undefined;
      for (const pt of p3) { const q = surface.project(pt, hu, hv); p2.push(q); hu = q[0]; hv = q[1]; }
      let wind = 0;
      for (let i = 0; i < p2.length; i++) {
        let d = p2[(i + 1) % p2.length]![c] - p2[i]![c];
        while (d > period / 2) d -= period; while (d < -period / 2) d += period;
        wind += d;
      }
      return { p3, p2, wind };
    });
    if (proj.some((p) => p === null)) return false;
    const rims = proj.filter((p): p is NonNullable<typeof p> => Math.abs(p!.wind) >= 0.9 * period);
    const holes = proj.filter((p): p is NonNullable<typeof p> => Math.abs(p!.wind) < 0.9 * period);
    if (rims.length !== 2) return false;
    const meanStack = (p: { p2: P2[] }): number => { let s = 0; for (const q of p.p2) s += q[stackC]; return s / p.p2.length; };
    rims.sort((a, b) => meanStack(a) - meanStack(b));

    // Seam in the widest window-free gap of the around-coordinate so no hole straddles the cut.
    const norm = (x: number): number => ((x % period) + period) % period;
    let seam: number;
    const angs: number[] = [];
    for (const h of holes) for (const q of h.p2) angs.push(norm(q[c]));
    if (angs.length) {
      angs.sort((a, b) => a - b);
      let bestGap = -1; seam = 0;
      for (let i = 0; i < angs.length; i++) {
        const a = angs[i]!, b = i + 1 < angs.length ? angs[i + 1]! : angs[0]! + period;
        if (b - a > bestGap) { bestGap = b - a; seam = (a + b) / 2; }
      }
    } else {
      seam = norm(rims[0]!.p2[0]![c]);
    }
    const normTo = (x: number): number => { let d = (x - seam) % period; if (d < 0) d += period; return seam + d; };
    const toP2 = (a: number, s: number): P2 => (c === 0 ? [a, s] : [s, a]);

    // Rim -> polyline spanning ~[seam, seam+period], net-ascending in the around-coord, with a
    // closing duplicate at start+period (3D-identical to the start; it becomes the far seam corner).
    // The loop is CUT at its vertex nearest the seam and UNWRAPPED in polyline order — never sorted.
    // Sorting by the around-coord only works for a rim that is a single-valued profile s(a); a
    // once-winding boundary can legally be a STAIRCASE (a half-buried cylinder boss: buried half
    // ends at one axial station, exposed half at another, joined by two constant-a waterline
    // segments). Sorting collapses each vertical segment into one a-slot in arbitrary order and
    // zigzags the boundary, which tangles the CDT constraints into surface-crossing triangles.
    const buildRim = (rim: { p3: Vec3[]; p2: P2[]; wind: number }, atSeam = seam): { p3: Vec3[]; p2: P2[] } => {
      let p3 = rim.p3, p2 = rim.p2;
      if (rim.wind < 0) { p3 = p3.slice().reverse(); p2 = p2.slice().reverse(); }
      const n = p2.length;
      let k = 0, best = Infinity;
      for (let i = 0; i < n; i++) { const d = norm(p2[i]![c] - atSeam); if (d < best) { best = d; k = i; } }
      const outP3: Vec3[] = [p3[k]!];
      const auv: [number, number][] = [[atSeam + best, p2[k]![stackC]]];
      for (let i = 1; i < n; i++) {
        const j = (k + i) % n, pj = (k + i - 1) % n;
        let d = p2[j]![c] - p2[pj]![c];
        while (d > period / 2) d -= period; while (d < -period / 2) d += period;
        outP3.push(p3[j]!); auv.push([auv[i - 1]![0] + d, p2[j]![stackC]]);
      }
      outP3.push(p3[k]!); auv.push([auv[0]![0] + period, auv[0]![1]]);
      return { p3: outP3, p2: auv.map(([a, s]) => toP2(a, s)) };
    };
    const bottom = buildRim(rims[0]!), top = buildRim(rims[1]!);
    // Outer boundary: bottom left->right, then top right->left. The two vertical sides are the seam
    // (same 3D line at seam and seam+period) -> weld closes it.
    let outer = {
      p3: [...bottom.p3, ...top.p3.slice().reverse()],
      p2: [...bottom.p2, ...top.p2.slice().reverse()],
    };
    let holeLoops = holes.map((h) => ({
      p3: h.p3,
      p2: h.p2.map((q) => toP2(normTo(q[c]), q[stackC])),
    }));
    // The vertex-cut construction above cuts each rim at its own nearest vertex, so the two seam
    // sides are only NEAR-vertical; on a rim that weaves (the Ontos muzzle wall: slot fingers
    // excursing 40mm along the axis), a seam side or the closing chord then CROSSES rim segments —
    // crossing constraints are unrealisable, the CDT drops one, and the rescue fills 65mm garbage
    // chords across the face. When the polygon self-intersects, rebuild with an EXACT common seam:
    // scan for a seam angle whose meridian crosses each rim exactly once and no hole, and cut both
    // rims THERE with an interpolated point. The cut point lies ON its segment's 3D chord, so the
    // neighbouring face's identical chord gains only an exactly-collinear T-vertex (closed by the
    // T-junction zip). Faces whose polygon is already simple keep the vertex-cut result bit-for-bit.
    // Conflict test covers BOTH failure modes: the outer polygon self-intersecting AND a window
    // hole crossing the outer (a hole hugging the seam lands split across the domain by normTo —
    // its constraints then cross the seam sides, equally unrealisable).
    const conflicted = (): boolean => {
      if (countSelfIntersections(outer.p2) > 0) return true;
      const o2 = (a: P2, b: P2, q: P2): number => (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]);
      const sx = (p: P2, q: P2, r: P2, s: P2): boolean => {
        const d1 = o2(r, s, p), d2 = o2(r, s, q), d3 = o2(p, q, r), d4 = o2(p, q, s);
        return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
      };
      const on = outer.p2.length;
      for (const h of holeLoops) {
        for (let i = 0; i < h.p2.length; i++) {
          const a = h.p2[i]!, b = h.p2[(i + 1) % h.p2.length]!;
          for (let j = 0; j < on; j++) {
            if (sx(a, b, outer.p2[j]!, outer.p2[(j + 1) % on]!)) return true;
          }
        }
      }
      return false;
    };
    // cross/buildRimCut take an optional SHEAR slope: with slope s they operate on the sheared
    // around-coordinate a = around − s·stack (a slanted meridian in (u,v)); slope 0 is the plain
    // vertical meridian, bit-identical to the original code. buildRimCut's OUTPUT stays in the
    // original (u,v): the accumulator unwraps in a-space and each emitted point adds s·stack back.
    {
      const cross = (p2: P2[], uk: number, slope = 0): number => {
        let cnt = 0;
        for (let i = 0; i < p2.length; i++) {
          const a = p2[i]![c] - slope * p2[i]![stackC];
          const j = (i + 1) % p2.length;
          let d = p2[j]![c] - slope * p2[j]![stackC] - a;
          while (d > period / 2) d -= period; while (d < -period / 2) d += period;
          if (Math.abs(d) < 1e-12) continue;
          const r = norm(uk - a);
          if ((d > 0 && r > 0 && r < d) || (d < 0 && r - period > d && r < period)) cnt++;
        }
        return cnt;
      };
      const buildRimCut = (rim: { p3: Vec3[]; p2: P2[]; wind: number }, uStar: number, slope = 0): { p3: Vec3[]; p2: P2[] } | null => {
        let p3 = rim.p3, p2 = rim.p2;
        if (rim.wind < 0) { p3 = p3.slice().reverse(); p2 = p2.slice().reverse(); }
        const n = p2.length;
        const wrapD = (d: number): number => { while (d > period / 2) d -= period; while (d < -period / 2) d += period; return d; };
        const aOf = (q: P2): number => q[c] - slope * q[stackC];
        let ci = -1, ct = 0, cs = 0;
        for (let i = 0; i < n; i++) {
          const a = aOf(p2[i]!), j = (i + 1) % n;
          const d = wrapD(aOf(p2[j]!) - a);
          if (Math.abs(d) < 1e-12) continue;
          const r = norm(uStar - a);
          let t = -1;
          if (d > 0 && r > 0 && r < d) t = r / d;
          else if (d < 0 && r - period > d && r < period) t = (r - period) / d;
          if (t > 0 && t < 1) {
            if (ci >= 0) return null; // crosses more than once — the scan should have excluded uStar
            ci = i; ct = t;
            cs = p2[i]![stackC] + (p2[j]![stackC] - p2[i]![stackC]) * t;
          }
        }
        if (ci < 0) return null;
        const j = (ci + 1) % n;
        const cut3: Vec3 = [
          p3[ci]![0] + (p3[j]![0] - p3[ci]![0]) * ct,
          p3[ci]![1] + (p3[j]![1] - p3[ci]![1]) * ct,
          p3[ci]![2] + (p3[j]![2] - p3[ci]![2]) * ct,
        ];
        const outP3: Vec3[] = [cut3];
        const auv: [number, number][] = [[uStar + slope * cs, cs]];
        let acc = uStar + (1 - ct) * wrapD(aOf(p2[j]!) - aOf(p2[ci]!));
        outP3.push(p3[j]!); auv.push([acc + slope * p2[j]![stackC], p2[j]![stackC]]);
        for (let k = 1; k < n; k++) {
          const idx = (j + k) % n, prv = (j + k - 1) % n;
          acc += wrapD(aOf(p2[idx]!) - aOf(p2[prv]!));
          outP3.push(p3[idx]!); auv.push([acc + slope * p2[idx]![stackC], p2[idx]![stackC]]);
        }
        outP3.push(cut3); auv.push([uStar + period + slope * cs, cs]);
        return { p3: outP3, p2: auv.map(([a, s]) => toP2(a, s)) };
      };
      const K = 512;
      if (conflicted()) {
        let uStar = NaN, bestMargin = -1;
        for (let k = 0; k < K; k++) {
          const uk = ((k + 0.5) * period) / K;
          if (cross(rims[0]!.p2, uk) !== 1 || cross(rims[1]!.p2, uk) !== 1) continue;
          let ok = true;
          for (const h of holes) if (cross(h.p2, uk) > 0) { ok = false; break; }
          if (!ok) continue;
          let margin = Infinity;
          for (const arr of [rims[0]!.p2, rims[1]!.p2, ...holes.map((h) => h.p2)]) {
            for (const q of arr) { const dd = norm(q[c] - uk); const m = Math.min(dd, period - dd); if (m < margin) margin = m; }
          }
          if (margin > bestMargin) { bestMargin = margin; uStar = uk; }
        }
        if (DBG && !Number.isFinite(uStar)) console.error(`[unroll] fid=${fid} exact-seam: no meridian crosses each rim exactly once (weaving rims)`);
        if (Number.isFinite(uStar)) {
          const b2 = buildRimCut(rims[0]!, uStar), t2 = buildRimCut(rims[1]!, uStar);
          if (b2 && t2) {
            const normTo2 = (x: number): number => { let d = (x - uStar) % period; if (d < 0) d += period; return uStar + d; };
            outer = { p3: [...b2.p3, ...t2.p3.slice().reverse()], p2: [...b2.p2, ...t2.p2.slice().reverse()] };
            holeLoops = holes.map((h) => ({ p3: h.p3, p2: h.p2.map((q) => toP2(normTo2(q[c]), q[stackC])) }));
            if (DBG) console.error(`[unroll] fid=${fid} exact-seam rebuild at ${uStar.toFixed(4)} (margin ${bestMargin.toExponential(1)})`);
          }
        }
      }
      // A hole whose consecutive (u,v) points jump more than half the period got FOLDED by the
      // seam normalisation — its signature and consumers are described at the seam-path rescue
      // below (declared here because the vertex-cut retry must not adopt a seam that folds one).
      const foldedHole = (): boolean => holeLoops.some((h) => {
        for (let i = 0; i < h.p2.length; i++) {
          if (Math.abs(h.p2[(i + 1) % h.p2.length]![c] - h.p2[i]![c]) > period / 2) return true;
        }
        return false;
      });
      // VERTEX-CUT SEAM RETRY. The exact-seam rebuild above needs a meridian crossing each rim
      // exactly ONCE — a rim that weaves across the whole circumference has no such meridian (a
      // slitted collet cone: every slit notches the band, so the top rim backtracks at every
      // angle), and the conflicted polygon used to fall through to the CDT, which drops the
      // crossed constraint and rescue-fills garbage chords across the face. But that conflict is
      // a sampling lottery, not topology: only the two SLANTED seam sides (each rim cut at its
      // own vertex nearest the seam) can cross a rim chord passing near the cut, so a different
      // seam angle — different cut vertices, different slants — is usually simple. Scan the
      // period for one and adopt the first conflict-free vertex-cut; every cut lands on an
      // existing rim vertex, so the weld/watertight story is identical to the original
      // construction. Faces whose first polygon was already simple never reach this.
      if (conflicted()) {
        const wasFolded = foldedHole();
        const prevOuter = outer, prevHoles = holeLoops;
        for (let k = 0; k < K; k++) {
          const seamK = ((k + 0.5) * period) / K;
          const b = buildRim(rims[0]!, seamK), t = buildRim(rims[1]!, seamK);
          outer = { p3: [...b.p3, ...t.p3.slice().reverse()], p2: [...b.p2, ...t.p2.slice().reverse()] };
          const normToK = (x: number): number => { let d = (x - seamK) % period; if (d < 0) d += period; return seamK + d; };
          holeLoops = holes.map((h) => ({ p3: h.p3, p2: h.p2.map((q) => toP2(normToK(q[c]), q[stackC])) }));
          if (!conflicted() && (wasFolded || !foldedHole())) {
            if (DBG) console.error(`[unroll] fid=${fid} vertex-cut seam retry: adopted seam=${seamK.toFixed(4)} (k=${k})`);
            break;
          }
          outer = prevOuter; holeLoops = prevHoles;
        }
      }
      // SEAM-PATH RESCUE (slanted / re-placed seam with sampled sides). Two fold classes reach
      // here broken and invisible to conflicted():
      //  - a hole that WINDS the around-coordinate (a helical thread slit crosses EVERY vertical
      //    meridian, ABC 00000122): normTo folds each rail continuation into a period-spanning
      //    jump chord that crosses its own siblings — the folded hole SELF-intersects;
      //  - a hole that STRADDLES the chosen seam (a full-height window on the barrel, ABC
      //    00000002): normTo splits it across the domain with two jump chords that cross nothing
      //    strictly, but the folded steps are near-period-long.
      // Trigger on either signature (self-intersection / any folded step > period/2), then retry
      // the cut with slope candidates: 0 (a plain re-placed vertical seam — enough for the
      // straddler class) and the winding holes' own regressed climbs (the thread class: in the
      // sheared coordinate a = around − slope·stack a helical slit becomes a COMPACT hole, rims
      // still cross an a-meridian exactly once, and the vertical machinery applies verbatim).
      // The seam sides are sampled at IDENTICAL stack stations one period apart — identical 3D
      // points, so the weld seals the cut (the tessellatePeriodicRegion seam-pair trick). Output
      // stays in original (u,v): a parallelogram domain, which the grid CDT handles unchanged.
      if (holes.length && (conflicted() || foldedHole() || holeLoops.some((h) => countSelfIntersections(h.p2) > 0))) {
        const wrapD = (d: number): number => { while (d > period / 2) d -= period; while (d < -period / 2) d += period; return d; };
        const unw = holes.map((h) => {
          const a: number[] = [norm(h.p2[0]![c])];
          for (let i = 1; i < h.p2.length; i++) a.push(a[i - 1]! + wrapD(h.p2[i]![c] - h.p2[i - 1]![c]));
          return a;
        });
        const slopes: number[] = [];
        for (let hi = 0; hi < holes.length; hi++) {
          const a = unw[hi]!;
          let mn = Infinity, mx = -Infinity;
          for (const x of a) { if (x < mn) mn = x; if (x > mx) mx = x; }
          if (mx - mn < 0.95 * period) continue; // fits a vertical cut — not a slope driver
          const hp = holes[hi]!.p2;
          let sm = 0, am = 0;
          for (let i = 0; i < a.length; i++) { sm += hp[i]![stackC]; am += a[i]!; }
          sm /= a.length; am /= a.length;
          let num = 0, den = 0;
          for (let i = 0; i < a.length; i++) { const ds = hp[i]![stackC] - sm; num += ds * (a[i]! - am); den += ds * ds; }
          if (den > 1e-18 && Math.abs(num / den) > 1e-12) slopes.push(num / den);
        }
        slopes.sort((x, y) => x - y);
        const cands: number[] = [0];
        if (slopes.length) cands.push(slopes[slopes.length >> 1]!);
        for (const s of slopes) if (!cands.some((x) => Math.abs(x - s) <= 1e-9 * Math.max(1, Math.abs(s)))) cands.push(s);
        if (DBG) console.error(`[unroll] fid=${fid} seam-path rescue: ${slopes.length} winding hole(s), slope candidates=[${cands.map((s) => s.toExponential(2)).join(",")}]`);
        for (const slope of cands) {
          const aOf = (q: P2): number => q[c] - slope * q[stackC];
          let aStar = NaN, aMargin = -1;
          for (let k = 0; k < K; k++) {
            const ak = ((k + 0.5) * period) / K;
            if (cross(rims[0]!.p2, ak, slope) !== 1 || cross(rims[1]!.p2, ak, slope) !== 1) continue;
            let ok = true;
            for (const h of holes) if (cross(h.p2, ak, slope) > 0) { ok = false; break; }
            if (!ok) continue;
            let margin = Infinity;
            for (const arr of [rims[0]!.p2, rims[1]!.p2, ...holes.map((h) => h.p2)]) {
              for (const q of arr) { const dd = norm(aOf(q) - ak); const m = Math.min(dd, period - dd); if (m < margin) margin = m; }
            }
            if (margin > aMargin) { aMargin = margin; aStar = ak; }
          }
          if (!Number.isFinite(aStar)) {
            if (DBG) console.error(`[unroll] fid=${fid} seam-path: no clean meridian at slope=${slope.toExponential(2)}`);
            continue;
          }
          const b2 = buildRimCut(rims[0]!, aStar, slope);
          const t2 = b2 ? buildRimCut(rims[1]!, aStar, slope) : null;
          if (!b2 || !t2) {
            if (DBG) console.error(`[unroll] fid=${fid} seam-path: rim cut failed at a*=${aStar.toFixed(4)} slope=${slope.toExponential(2)}`);
            continue;
          }
          // Seam sides: sampled along a = aStar between the two cut points at the face target
          // (a slanted cut is a helix in 3D — a single chord would gouge the surface). Both
          // sides reference the SAME evaluated 3D objects so the weld unifies them exactly.
          const csB = b2.p2[0]![stackC], csT = t2.p2[0]![stackC];
          const qMid = toP2(aStar + slope * ((csB + csT) / 2), (csB + csT) / 2);
          const fT = faceTarget(surface, targetEdge, chordTol, normalDev, qMid[0], qMid[1]);
          let len = 0, prev = b2.p3[0]!;
          for (let i = 1; i <= 16; i++) {
            const s = csB + ((csT - csB) * i) / 16;
            const q = toP2(aStar + slope * s, s);
            const p = surface.evaluate(q[0], q[1]);
            len += Math.hypot(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]); prev = p;
          }
          const nSeam = Math.min(4096, Math.max(1, Math.ceil(len / Math.max(fT, 1e-9))));
          const seam3: Vec3[] = [], seamS: number[] = [];
          for (let i = 1; i < nSeam; i++) {
            const s = csB + ((csT - csB) * i) / nSeam;
            const q = toP2(aStar + slope * s, s);
            seam3.push(surface.evaluate(q[0], q[1])); seamS.push(s);
          }
          const prevOuter = outer, prevHoles = holeLoops;
          outer = {
            p3: [...b2.p3, ...seam3, ...t2.p3.slice().reverse(), ...seam3.slice().reverse()],
            p2: [
              ...b2.p2,
              ...seamS.map((s) => toP2(aStar + period + slope * s, s)),
              ...t2.p2.slice().reverse(),
              ...seamS.slice().reverse().map((s) => toP2(aStar + slope * s, s)),
            ],
          };
          holeLoops = holes.map((h, hi) => {
            const a = unw[hi]!;
            let bMin = Infinity;
            for (let i = 0; i < a.length; i++) { const b = a[i]! - slope * h.p2[i]![stackC]; if (b < bMin) bMin = b; }
            const kk = Math.ceil((aStar - bMin) / period);
            return { p3: h.p3, p2: h.p2.map((q, i) => toP2(a[i]! + kk * period, q[stackC])) };
          });
          if (conflicted() || foldedHole()) {
            if (DBG) console.error(`[unroll] fid=${fid} seam-path: rebuild at slope=${slope.toExponential(2)} still conflicted — reverted`);
            outer = prevOuter; holeLoops = prevHoles;
            continue;
          }
          if (DBG) console.error(`[unroll] fid=${fid} seam-path rebuild slope=${slope.toExponential(2)} a*=${aStar.toFixed(4)} seamPts=${nSeam - 1} (margin ${aMargin.toExponential(1)})`);
          break;
        }
      }
    }
    return gridCDT(surface, outer, holeLoops, fid, verts, faceIds, targetEdge, chordTol, normalDev, sign);
  };
  return (!!surface.periodicU && attempt(0)) || (!!surface.periodicV && attempt(1));
}

/**
 * Stitch two concentric (closed) rings of unequal point counts into a triangle band, advancing
 * whichever ring is behind in its angular fraction. Both rings must start at the same angle and run
 * the same way; a count-1 ring is a pole (fan). Shared by the cone / sphere / B-spline pole meshers.
 * By default a point's angular fraction is its INDEX fraction — right for evenly spaced rings. A
 * ring whose points are NOT evenly spaced in angle (a staircase rim: several boundary points share
 * one angle along a constant-angle step) must pass explicit fractions (fracA/fracB, cumulative
 * angular progress normalised to [0,1)); index pairing there skews partners by up to the step's
 * index share of the ring — chords spanning ~100° of arc that cut straight through the surface.
 */
function stitchRings(
  verts: number[], faceIds: number[], A: Vec3[], B: Vec3[], fid: number, surface: Surface, sign: number,
  fracA?: number[], fracB?: number[],
): void {
  const na = A.length, nb = B.length;
  if (na === 1) { for (let j = 0; j < nb; j++) emitTri(verts, faceIds, A[0]!, B[j]!, B[(j + 1) % nb]!, fid, surface, sign); return; }
  if (nb === 1) { for (let k = 0; k < na; k++) emitTri(verts, faceIds, A[k]!, A[(k + 1) % na]!, B[0]!, fid, surface, sign); return; }
  const fa = (i: number): number => (i >= na ? 1 : fracA ? fracA[i]! : i / na);
  const fb = (i: number): number => (i >= nb ? 1 : fracB ? fracB[i]! : i / nb);
  let ia = 0, ib = 0;
  while (ia < na || ib < nb) {
    if (ia < na && (ib >= nb || fa(ia) < fb(ib))) { emitTri(verts, faceIds, A[ia % na]!, A[(ia + 1) % na]!, B[ib % nb]!, fid, surface, sign); ia++; }
    else { emitTri(verts, faceIds, A[ia % na]!, B[(ib + 1) % nb]!, B[ib % nb]!, fid, surface, sign); ib++; }
  }
}

/**
 * Reparametrise a trimmed sphere patch so the pole and seam lie AWAY from the patch. A corner-blend
 * patch often runs straight THROUGH the sphere's parametrisation pole (three fillets meet at the
 * axis point): u is undefined there, atan2 flips by π across it, the projected loop fake-winds a
 * full period and the CDT meshes a degenerate strip — a coarse flat corner. The parametrisation is
 * OURS to choose: aim the new x-axis at the patch's mean direction and the new pole perpendicular
 * to it, so the patch sits in a clean window around (u,v)=(0,0), clear of pole and seam. Only valid
 * when the patch fits a spherical cap well inside ±90° (always true for corner blends); anything
 * larger (a sphere minus a small cap) keeps the original frame.
 */
function reorientSphere(s: Sphere, loops: BLoop[], sampled: Map<number, Vec3[]>): Sphere {
  const c = s.f.o;
  const dirs: Vec3[] = [];
  let mx = 0, my = 0, mz = 0;
  for (const lp of loops) for (const oe of lp.edges) {
    const base = sampled.get(oe.edgeId);
    if (!base) continue;
    for (const p of base) {
      const dx = p[0] - c[0], dy = p[1] - c[1], dz = p[2] - c[2];
      const L = Math.hypot(dx, dy, dz);
      if (L < 1e-12) continue;
      dirs.push([dx / L, dy / L, dz / L]);
      mx += dx / L; my += dy / L; mz += dz / L;
    }
  }
  const mL = Math.hypot(mx, my, mz);
  if (dirs.length < 3 || mL < 1e-9 * dirs.length) return s; // no privileged centre (band-like patch)
  const m: Vec3 = [mx / mL, my / mL, mz / mL];
  let minDot = 1;
  for (const d of dirs) minDot = Math.min(minDot, dot(d, m));
  if (minDot < Math.cos(1.31)) return s; // a point sits > ~75° from the centre — can't clear the poles
  const up: Vec3 = Math.abs(m[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const z = normalize(cross(m, up));
  return new Sphere({ o: [c[0], c[1], c[2]], x: m, y: cross(z, m), z }, s.r);
}

/**
 * Strip zero-width POLE DARTS from a sphere face's loops. A face that contains a parametrisation
 * pole in its INTERIOR (an umbrella-valve dome: a full cap minus a slit lune that stops short of
 * the apex) is cut open by the kernel with a DART — the same meridian edge traversed out and back
 * (rim -> pole -> rim, adjacent in the loop with opposite senses) so the pole is reachable from
 * the boundary. The chained projection lifts both traversals at the SAME u (the pole pins no
 * longitude), the spike encloses no area, sanitize collapses it, and the CDT meshes the loop's
 * residual signed area — exactly the cut-out lune, the COMPLEMENT of the face (Midea drain valve:
 * the dome's concave seat came out as a convex bump). The dart is pure topology — zero width, and
 * both uses live in THIS loop, so no neighbour shares its samples — so removing it changes no
 * shared boundary. The caller then hands the face to the periodic-REGION mesher, which rebuilds
 * it as a band from the once-winding rim to a synthetic pole row (welded into a fan). Returns
 * null when no pole dart is found.
 */
function stripSpherePoleDarts(s: Sphere, loops: BLoop[], sampled: Map<number, Vec3[]>): BLoop[] | null {
  const tol = Math.max(1e-9, 1e-6 * s.r);
  const isPole = (p: Vec3): boolean => {
    for (const sgn of [1, -1]) {
      if (Math.hypot(
        p[0] - (s.f.o[0] + sgn * s.r * s.f.z[0]),
        p[1] - (s.f.o[1] + sgn * s.r * s.f.z[1]),
        p[2] - (s.f.o[2] + sgn * s.r * s.f.z[2]),
      ) < tol) return true;
    }
    return false;
  };
  let found = false;
  const out: BLoop[] = [];
  for (const lp of loops) {
    let es = lp.edges;
    // WHOLE-LOOP dart: the loop is exactly one meridian edge out-and-back between the two poles —
    // how a kernel cuts open a FULL-sphere ball face (SKF bearing balls). Zero width, both
    // traversals live in this loop so no neighbour shares its samples: drop the loop entirely.
    // Punching it as a CDT hole instead is what this function exists to prevent — the degenerate
    // slit sheds unenforceable constraints and parity-floods a phantom lune out of the face.
    if (es.length === 2 && es[0]!.edgeId === es[1]!.edgeId && es[0]!.orient !== es[1]!.orient) {
      const base = sampled.get(es[0]!.edgeId);
      if (base && base.length > 0 && isPole(base[0]!) && isPole(base[base.length - 1]!)) {
        found = true;
        continue; // the whole loop is the dart
      }
    }
    for (let i = 0; es.length > 2 && i < es.length; i++) {
      const a = es[i]!, b = es[(i + 1) % es.length]!;
      if (a.edgeId !== b.edgeId || a.orient === b.orient) continue;
      const base = sampled.get(a.edgeId);
      if (!base || base.length === 0) continue;
      const turn = a.orient ? base[base.length - 1]! : base[0]!;
      if (!isPole(turn)) continue;
      const j = (i + 1) % es.length;
      es = es.filter((_, k) => k !== i && k !== j);
      found = true;
      i = -1; // rescan: a loop can carry one dart per slit
    }
    out.push(es === lp.edges ? lp : { outer: lp.outer, edges: es });
  }
  return found ? out : null;
}

/**
 * Full-revolution band (a cylinder/cone hole wall, etc.) whose rims are separate FULL-PERIOD circle
 * loops with no seam edges — how some kernels (Onshape/ST-DEVELOPER) represent a drilled hole. Each
 * rim projects to an open horizontal line in (u,v) enclosing no area, so the param grid meshes
 * nothing and the hole's edges open. Stitch the shared rim samples directly into a triangle band
 * instead (intermediate rings for height), wrapping cyclically so no seam is needed. Returns false
 * WITHOUT emitting if the loops aren't all full-period rims, so the caller falls back to the grid.
 */
function tessellateRevolutionBand(
  surface: Surface, loops: BLoop[], sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
): boolean {
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  // c = the coordinate the rims WIND in (0: u-rims stacked along v, e.g. a drilled hole; 1: v-rims
  // stacked along u, e.g. a torus tube segment — a pipe elbow's ends are tube cross-sections).
  const evalAt = (c: 0 | 1, ang: number, h: number): Vec3 => (c === 0 ? surface.evaluate(ang, h) : surface.evaluate(h, ang));

  const bandAlong = (c: 0 | 1): boolean => {
    const period = (c === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI;
    const stackPeriodic = c === 0 ? !!surface.periodicV : !!surface.periodicU;
    const stackPeriod = (c === 0 ? surface.vPeriod : surface.uPeriod) || TWO_PI;
    const angleOf = (p: Vec3): number => surface.project(p)[c];
    const rims: { pts: Vec3[]; h: number; wind: number }[] = [];
    for (const lp of loops) {
      const pts: Vec3[] = [];
      for (const oe of lp.edges) {
        const base = sampled.get(oe.edgeId); if (!base) continue;
        const poly = oe.orient ? base : base.slice().reverse();
        for (let i = 0; i < poly.length - 1; i++) pts.push(poly[i]!);
      }
      if (pts.length < 3) return false;
      // Full-period rim by WINDING NUMBER (sum of wrapped angular steps around the closed loop) OR
      // by span. Winding catches coarsely sampled rims — 8 points around a tiny circle span only
      // 7π/4, failing the span test, yet wind exactly ±period. Span catches rims that carry seam
      // edges and double back (net winding 0) which the old mesher still stitched acceptably.
      let wind = 0, absWind = 0, amin = Infinity, amax = -Infinity, hcos = 0, hsin = 0, hsum = 0;
      const as: number[] = [], hs: number[] = [];
      for (const p of pts) {
        const uv = surface.project(p);
        const a = uv[c], h = uv[1 - c]!;
        as.push(a); hs.push(h); hsum += h;
        // Circular mean for a periodic stack coordinate (rim points may straddle its seam).
        hcos += Math.cos((h * TWO_PI) / stackPeriod); hsin += Math.sin((h * TWO_PI) / stackPeriod);
        if (a < amin) amin = a; if (a > amax) amax = a;
      }
      for (let i = 0; i < as.length; i++) {
        let d = as[(i + 1) % as.length]! - as[i]!;
        while (d > period / 2) d -= period; while (d < -period / 2) d += period;
        wind += d; absWind += Math.abs(d);
        // A rim that is MULTIVALUED in angle — an axial STEP, e.g. a staircase rim's constant-angle
        // riser — breaks the per-angle loft below: the interior rings inherit BOTH heights at that
        // angle, the zero-width "wall" quads between the doubled columns are dropped as degenerate,
        // and the two column ladders subdivide the shared meridian differently (T-junction opens).
        // Per-angle lofting is only well-defined for a single-valued height-vs-angle rim; bail and
        // let the CDT-based unroll take the face (its rim builder handles staircases).
        let dh = hs[(i + 1) % hs.length]! - hs[i]!;
        if (stackPeriodic) { while (dh > stackPeriod / 2) dh -= stackPeriod; while (dh < -stackPeriod / 2) dh += stackPeriod; }
        if (Math.abs(d) < 1e-8 && Math.abs(dh) > 1e-8) return false;
      }
      if (Math.abs(wind) < 0.9 * period && amax - amin < 0.9 * period) return false; // partial arc
      // The per-angle loft is only well-defined for a rim SINGLE-VALUED in angle: monotone
      // angular progression means sum(|du|) == |sum(du)|. A rim whose trim curves BACKTRACK in
      // angle (a cone cut by a wavy B-spline boundary: 180°->152°->170°->56°, Ontos) passes the
      // winding test and the exact constant-angle staircase check above, but pairing its points
      // by angular progress then folds — the loft/stitch chords straight through the surface
      // (5mm sagitta on an 8mm cone). Bail; the CDT-based unroll handles backtracking rims
      // (its rim builder unwraps in polyline order and triangulates the true boundary).
      // ONLY for a non-periodic stack coordinate: the unroll's rectangle assumes a LINEAR stack,
      // so kicking a torus tube segment out of the band mesher hands it to a fallback that is
      // wrong by construction (Ontos torus #92192: 1.5mm loft -> 92mm unroll garbage). On a
      // periodic stack the loft's bounded fold error stays the best available result.
      if (!stackPeriodic && absWind > Math.abs(wind) + 0.05 * period) return false;
      const h = stackPeriodic ? (Math.atan2(hsin, hcos) * stackPeriod) / TWO_PI : hsum / pts.length;
      rims.push({ pts, h, wind });
    }
    if (rims.length < 2) return false;

    // Order the rims along the stack coordinate. Non-periodic stack (cylinder/cone v): plain sort.
    // Periodic stack (torus): "between h0 and h1" is ambiguous (two arcs) — use the boundary
    // orientation: a CCW face in (u,v) travels its v0 rim in +u and its v1 rim in -u (c=0), and its
    // u1 rim in +v and u0 rim in -v (c=1), so the winding SIGN says which rim starts the stack.
    let stack: { from: number; width: number; bottom: Vec3[]; top: Vec3[]; bWind: number; tWind: number }[] = [];
    if (!stackPeriodic) {
      rims.sort((a, b) => a.h - b.h);
      for (let r = 0; r + 1 < rims.length; r++) {
        stack.push({ from: rims[r]!.h, width: rims[r + 1]!.h - rims[r]!.h, bottom: rims[r]!.pts, top: rims[r + 1]!.pts, bWind: rims[r]!.wind, tWind: rims[r + 1]!.wind });
      }
    } else {
      if (rims.length !== 2) return false;
      const want = c === 0 ? 1 : -1; // winding sign (after sameSense) of the rim the stack STARTS at
      const A = rims.find((r) => Math.sign(r.wind) * sign === want);
      const B = rims.find((r) => r !== A);
      if (!A || !B) return false;
      let width = (B.h - A.h) % stackPeriod;
      if (width <= 1e-9) width += stackPeriod;
      if (width >= stackPeriod - 1e-9) return false; // degenerate: rims coincide in stack coordinate
      stack.push({ from: A.h, width, bottom: A.pts, top: B.pts, bWind: A.wind, tWind: B.wind });
    }

    const half = period / 2;
    const wrap = (d: number): number => { while (d > half) d -= period; while (d < -half) d += period; return d; };
    // Rotate/flip `ring` so it starts at `ref[0]`'s angle and runs the same rotational direction.
    // Direction comes from the rim's NET WINDING when it has one: a wavy rim's FIRST segment can
    // locally backtrack (Ontos crown rim: u steps −0.004 twice before marching +2π), and reading
    // direction off it reverses the partner ring — the stitch then pairs diametrally opposite
    // points and chords straight through the surface. First-segment stays as the fallback for
    // span-qualified rims with zero net wind (seam-carrying doubled rims).
    const align = (ref: Vec3[], ring: Vec3[], refWind = 0, ringWind = 0): Vec3[] => {
      const a0 = angleOf(ref[0]!);
      const dirRef = refWind !== 0 ? Math.sign(refWind) : wrap(angleOf(ref[1]!) - a0) >= 0 ? 1 : -1;
      const r = ring.slice();
      const dirRing = ringWind !== 0 ? Math.sign(ringWind) : wrap(angleOf(r[1]!) - angleOf(r[0]!)) >= 0 ? 1 : -1;
      if (dirRing !== dirRef) r.reverse();
      let bi = 0, bd = Infinity;
      for (let i = 0; i < r.length; i++) { const dd = Math.abs(wrap(angleOf(r[i]!) - a0)); if (dd < bd) { bd = dd; bi = i; } }
      return [...r.slice(bi), ...r.slice(0, bi)];
    };

    for (const band of stack) {
      const bottom = band.bottom, top = align(bottom, band.top, band.bWind, band.tWind);
      const angs = bottom.map(angleOf);
      const h0 = band.from, h1 = band.from + band.width;
      // Rims need not sit at constant stack height — a cylinder crossed by another cylinder has a
      // WAVY intersection-curve rim. Loft PER ANGLE between the rims' true heights (blending only
      // the stack coordinate keeps every ring on the surface); constant-height interior rings would
      // fold across a wavy rim and leave open seams. Rims are matched by angular progress so
      // unequal point counts interpolate cleanly; heights unwrap near the band for a periodic stack.
      const hOf = (p: Vec3): number => surface.project(p)[1 - c]!;
      const nearTo = (h: number, ref: number): number => {
        if (!stackPeriodic) return h;
        while (h - ref > stackPeriod / 2) h -= stackPeriod;
        while (h - ref < -stackPeriod / 2) h += stackPeriod;
        return h;
      };
      const progressOf = (pts: Vec3[]): number[] => {
        const out = [0];
        for (let i = 1; i < pts.length; i++) out.push(out[i - 1]! + Math.abs(wrap(angleOf(pts[i]!) - angleOf(pts[i - 1]!))));
        return out;
      };
      const hB = bottom.map((p) => nearTo(hOf(p), h0));
      const hT = top.map((p) => nearTo(hOf(p), h1));
      const progB = progressOf(bottom), progT = progressOf(top);
      const scaleT = progB[progB.length - 1]! > 1e-12 ? progT[progT.length - 1]! / progB[progB.length - 1]! : 1;
      let ti = 0;
      const hTopAt = (s: number): number => {
        s *= scaleT; // map bottom progress into top progress domain
        while (ti > 0 && progT[ti]! > s) ti--;
        while (ti + 1 < progT.length && progT[ti + 1]! < s) ti++;
        if (ti + 1 >= progT.length) return hT[hT.length - 1]!;
        const d = progT[ti + 1]! - progT[ti]!;
        const f = d > 1e-12 ? (s - progT[ti]!) / d : 0;
        return hT[ti]! + (hT[(ti + 1) % hT.length]! - hT[ti]!) * Math.max(0, Math.min(1, f));
      };
      const target = faceTarget(surface, targetEdge, chordTol, normalDev, c === 0 ? angs[0]! : (h0 + h1) / 2, c === 0 ? (h0 + h1) / 2 : angs[0]!);
      // Ring count from the LONGEST stack traverse over a few sample angles (a torus elbow's outer
      // side is much longer than its inner side).
      let span = 0;
      for (let j = 0; j < angs.length; j += Math.max(1, angs.length >> 3)) {
        ti = 0;
        span = Math.max(span, d3(bottom[j]!, evalAt(c, angs[j]!, hTopAt(progB[j]!))));
      }
      const nRings = Math.max(1, Math.min(400, Math.ceil(span / target)));
      // Angular-progress fractions for the final stitch: the loft rings inherit bottom's angles
      // (index-aligned 1:1 among themselves), but `top` may distribute its points unevenly in angle
      // (a staircase rim parks many points on one constant-angle step), so the closing band must
      // pair by TRUE angular progress, not index share. Fractions are normalised over the progress
      // INCLUDING the closing segment back to the ring start.
      const fracOf = (prog: number[], pts: Vec3[]): number[] => {
        const tot = prog[prog.length - 1]! + Math.abs(wrap(angleOf(pts[0]!) - angleOf(pts[pts.length - 1]!)));
        return prog.map((p) => (tot > 1e-12 ? p / tot : 0));
      };
      let prev = bottom;
      for (let k = 1; k < nRings; k++) {
        ti = 0;
        const f = k / nRings;
        const ring = angs.map((a, j) => evalAt(c, a, hB[j]! + (hTopAt(progB[j]!) - hB[j]!) * f));
        stitchRings(verts, faceIds, prev, ring, fid, surface, sign);
        prev = ring;
      }
      stitchRings(verts, faceIds, prev, top, fid, surface, sign, fracOf(progB, bottom), fracOf(progT, top));
    }
    return true;
  };

  return (!!surface.periodicU && bandAlong(0)) || (!!surface.periodicV && bandAlong(1));
}

/** Distance from point q to a polyline (its nearest segment). */
function distToPolyline(q: Vec3, poly: Vec3[]): number {
  let best = Infinity;
  for (let i = 0; i < poly.length - 1; i++) {
    const a = poly[i]!, b = poly[i + 1]!;
    const ex = b[0] - a[0], ey = b[1] - a[1], ez = b[2] - a[2], l2 = ex * ex + ey * ey + ez * ez;
    let t = l2 > 0 ? ((q[0] - a[0]) * ex + (q[1] - a[1]) * ey + (q[2] - a[2]) * ez) / l2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    const d = (q[0] - a[0] - t * ex) ** 2 + (q[1] - a[1] - t * ey) ** 2 + (q[2] - a[2] - t * ez) ** 2;
    if (d < best) best = d;
  }
  return Math.sqrt(best);
}

/**
 * Mesh a degenerate "sliver" face — a strip far thinner than the mesh resolution (a sub-micron crack
 * a CAD kernel left between two surfaces). The constrained triangulation chokes on the extreme aspect
 * ratio and yields garbage that doesn't span the strip, leaving its two long rails unconnected (open
 * seams). Instead, split the boundary loop at its two most distant vertices into two rail chains and
 * stitch them directly into a triangle ribbon — guaranteed to connect the rails, hence watertight.
 *
 * Returns false WITHOUT emitting if the face isn't actually a thin strip (the two rails are more than
 * `tol` apart somewhere), so the caller falls through to the normal triangulator. The separation test
 * uses true rail geometry (a curved cylinder's boundary has near-zero planar area but isn't thin).
 */
function tessellateThinFace(
  surface: Surface, outerLoop: BLoop, sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], sign: number, tol: number,
): boolean {
  const p: Vec3[] = [];
  for (const oe of outerLoop.edges) {
    const base = sampled.get(oe.edgeId); if (!base) continue;
    const poly = oe.orient ? base : base.slice().reverse();
    for (let i = 0; i < poly.length - 1; i++) p.push(poly[i]!);
  }
  const m = p.length; if (m < 3) return false;
  // Approximate diameter (centroid -> farthest A -> farthest B): the two strip ends. O(m), no O(m²).
  let cx = 0, cy = 0, cz = 0; for (const q of p) { cx += q[0]; cy += q[1]; cz += q[2]; }
  cx /= m; cy /= m; cz /= m;
  const far = (ox: number, oy: number, oz: number): number => {
    let bi = 0, bd = -1;
    for (let i = 0; i < m; i++) { const d = (p[i]![0] - ox) ** 2 + (p[i]![1] - oy) ** 2 + (p[i]![2] - oz) ** 2; if (d > bd) { bd = d; bi = i; } }
    return bi;
  };
  const ia = far(cx, cy, cz);
  const ib = far(p[ia]![0], p[ia]![1], p[ia]![2]);
  if (ia === ib) return false;
  const c1: Vec3[] = [], c2: Vec3[] = [];
  for (let i = ia; ; i = (i + 1) % m) { c1.push(p[i]!); if (i === ib) break; }
  for (let i = ib; ; i = (i + 1) % m) { c2.push(p[i]!); if (i === ia) break; }
  c2.reverse(); // both chains now run from the ia end to the ib end
  const na = c1.length, nb = c2.length;
  if (na < 2 || nb < 2) return false;
  // Thinness test: every rail vertex must lie within tol of the opposite rail (sampled, early-exit).
  const step1 = Math.max(1, Math.floor(na / 64)), step2 = Math.max(1, Math.floor(nb / 64));
  for (let i = 0; i < na; i += step1) if (distToPolyline(c1[i]!, c2) > tol) return false;
  for (let j = 0; j < nb; j += step2) if (distToPolyline(c2[j]!, c1) > tol) return false;
  // Stitch the two rails into a ribbon, advancing whichever is behind in arc-length fraction.
  let i = 0, j = 0;
  while (i < na - 1 || j < nb - 1) {
    if (j >= nb - 1 || (i < na - 1 && (i + 1) / na <= (j + 1) / nb)) {
      emitTri(verts, faceIds, c1[i]!, c1[i + 1]!, c2[j]!, fid, surface, sign); i++;
    } else {
      emitTri(verts, faceIds, c1[i]!, c2[j + 1]!, c2[j]!, fid, surface, sign); j++;
    }
  }
  return true;
}

/**
 * LAST-RESORT sliver rail-stitch for a single-loop face every regular mesher rejected. The class it
 * exists for (ABC untriangulated census): real micron-scale sliver faces — a 1µm × 150mm plane strip,
 * a 90nm B-spline blend crumb, a collapsed offset ribbon — whose loop projects to near-zero area, so
 * the param grid's metric-width gate bails, while tessellateThinFace's fixed 0.005mm rail test also
 * fails: on CURVED rails the sampled polylines bow apart by up to the chord tolerance, which dwarfs
 * the true width on any large part.
 *
 * Thinness is instead PROVEN sampling-independently by the loop's 3D vector area: width =
 * |Σ (pᵢ−c)×(pᵢ₊₁−c)| / perimeter (= 2·area/perimeter). A thin ribbon — flat OR bent along its
 * length — keeps width ≈ its physical width, while any boundary enclosing a real region (a disk rim,
 * a tube's unwrapped boundary) reads its full extent. The rail-distance test then only guards the
 * cancellation pathologies (a saddle loop whose lobes cancel), with a sag-aware tolerance of a few
 * chord tolerances — a face that thin meshed as a ribbon deviates from the surface by less than the
 * sampling itself. Runs AFTER the param grid (and every other mesher) so genuine faces are untouched;
 * the rails are the shared edge samples, so the ribbon is watertight with every neighbour.
 */
function tessellateSliverLoop(
  surface: Surface, outerLoop: BLoop, sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], sign: number, chordTol: number,
): boolean {
  const p: Vec3[] = [];
  for (const oe of outerLoop.edges) {
    const base = sampled.get(oe.edgeId); if (!base) continue; // healed-away micro-edge: ring closes without it
    const poly = oe.orient ? base : base.slice().reverse();
    for (let i = 0; i < poly.length - 1; i++) p.push(poly[i]!);
  }
  const m = p.length; if (m < 4) return false;
  let cx = 0, cy = 0, cz = 0; for (const q of p) { cx += q[0]; cy += q[1]; cz += q[2]; }
  cx /= m; cy /= m; cz /= m;
  let ax = 0, ay = 0, az = 0, per = 0;
  for (let i = 0; i < m; i++) {
    const a = p[i]!, b = p[(i + 1) % m]!;
    ax += (a[1] - cy) * (b[2] - cz) - (a[2] - cz) * (b[1] - cy);
    ay += (a[2] - cz) * (b[0] - cx) - (a[0] - cx) * (b[2] - cz);
    az += (a[0] - cx) * (b[1] - cy) - (a[1] - cy) * (b[0] - cx);
    per += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  }
  const width = Math.hypot(ax, ay, az) / Math.max(per, 1e-12);
  // Pre-gate at 2·chordTol: cheap rejection of clearly-wide faces (a disk rim reads r/2). The rail
  // test below is the true guard — cancellation shapes (a full tube's boundary reads ~0) fail it.
  if (width > Math.max(1e-3, 2 * chordTol)) {
    if (DBG) console.error(`[tess] fid=${fid} sliverLoop bail: 3D width ${width.toExponential(2)} > ${Math.max(1e-3, 2 * chordTol).toExponential(2)}`);
    return false;
  }
  const far = (ox: number, oy: number, oz: number): number => {
    let bi = 0, bd = -1;
    for (let i = 0; i < m; i++) { const d = (p[i]![0] - ox) ** 2 + (p[i]![1] - oy) ** 2 + (p[i]![2] - oz) ** 2; if (d > bd) { bd = d; bi = i; } }
    return bi;
  };
  const ia = far(cx, cy, cz), ib = far(p[ia]![0], p[ia]![1], p[ia]![2]);
  if (ia === ib) return false;
  const c1: Vec3[] = [], c2: Vec3[] = [];
  for (let i = ia; ; i = (i + 1) % m) { c1.push(p[i]!); if (i === ib) break; }
  for (let i = ib; ; i = (i + 1) % m) { c2.push(p[i]!); if (i === ia) break; }
  c2.reverse();
  const na = c1.length, nb = c2.length;
  if (na < 2 || nb < 2) return false;
  const dtol = Math.max(3 * width, 2.5 * chordTol, 1e-3);
  const step1 = Math.max(1, Math.floor(na / 64)), step2 = Math.max(1, Math.floor(nb / 64));
  for (let i = 0; i < na; i += step1) {
    const d = distToPolyline(c1[i]!, c2);
    if (d > dtol) { if (DBG) console.error(`[tess] fid=${fid} sliverLoop bail: rail1[${i}] ${d.toExponential(2)} > dtol ${dtol.toExponential(2)}`); return false; }
  }
  for (let j = 0; j < nb; j += step2) {
    const d = distToPolyline(c2[j]!, c1);
    if (d > dtol) { if (DBG) console.error(`[tess] fid=${fid} sliverLoop bail: rail2[${j}] ${d.toExponential(2)} > dtol ${dtol.toExponential(2)}`); return false; }
  }
  if (DBG) console.error(`[tess] fid=${fid} sliverLoop: width=${width.toExponential(2)} rails=${na}/${nb}`);
  let i = 0, j = 0;
  while (i < na - 1 || j < nb - 1) {
    if (j >= nb - 1 || (i < na - 1 && (i + 1) / na <= (j + 1) / nb)) {
      emitTri(verts, faceIds, c1[i]!, c1[i + 1]!, c2[j]!, fid, surface, sign); i++;
    } else {
      emitTri(verts, faceIds, c1[i]!, c2[j + 1]!, c2[j]!, fid, surface, sign); j++;
    }
  }
  return true;
}

/**
 * Thin planar STRIP sliver: a single-loop PLANE face that is a long thin crescent/ribbon (two nearly-
 * coincident long rails — arcs or lines — closed by short ends), a sub-tolerance knife-edge a CAD
 * kernel leaves where two surfaces almost meet (the 6020 fan-shroud venturi lips: 17µm wide × 45mm
 * long). Like the thin ring, the param grid seats no interior point (the whole strip is inside the
 * boundary keep-out) and its CDT drops the rail constraints, so the strip and its neighbours open.
 * Split the loop at its two most-distant vertices into two rails and ribbon-stitch them (the rails are
 * the shared edge samples → watertight). Width is measured sampling-independently as 2·area/perimeter
 * (shoelace area in the plane's (u,v); a point-to-polyline distance is inflated by the sampling
 * chord tolerance and cannot separate a 17µm sliver from a real strip on a large part). A face wider
 * than `tol` fails and keeps the param grid, which triangulates a genuine thin face acceptably.
 */
function tessellateThinStrip(
  surface: Surface, outerLoop: BLoop, sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], sign: number, tol: number,
): boolean {
  if (surface.kind !== "PLANE") return false;
  const p: Vec3[] = [];
  for (const oe of outerLoop.edges) {
    const base = sampled.get(oe.edgeId); if (!base) return false;
    const poly = oe.orient ? base : base.slice().reverse();
    for (let i = 0; i < poly.length - 1; i++) p.push(poly[i]!);
  }
  const m = p.length; if (m < 4) return false;
  // Only stitch when the grid genuinely CAN'T: the boundary self-intersects in (u,v). Two nearly-
  // coincident rails (curved sliver) cross their own chords, which the CDT cannot realise — the exact
  // faces that come out untriangulated or MISSING (Toolhead Revo's 5-pt sliver crosses twice, the
  // 6020 crescents 28×). A SIMPLE boundary (a clean thin quad, however sub-tolerance) triangulates
  // fine in the grid and must be left to it — re-stitching it would open the ribbon's short-rail ends
  // (VORONDESIGN XY-Endstop's crumb faces cross zero times and regressed under a point-count gate).
  const p2 = p.map((q) => surface.project(q));
  if (countSelfIntersections(p2) === 0) return false;
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  let per = 0; for (let i = 0; i < m; i++) per += d3(p[i]!, p[(i + 1) % m]!);
  let a2 = 0;
  for (let i = 0, j = m - 1; i < m; j = i++) { a2 += (p2[j]![0] + p2[i]![0]) * (p2[j]![1] - p2[i]![1]); }
  const width = (Math.abs(a2)) / Math.max(1e-9, per); // = 2·(|a2|/2)/perimeter
  if (width > tol) return false;
  // Split at the two most-distant vertices (the strip ends), as tessellateThinFace does.
  let cx = 0, cy = 0, cz = 0; for (const q of p) { cx += q[0]; cy += q[1]; cz += q[2]; }
  cx /= m; cy /= m; cz /= m;
  const far = (ox: number, oy: number, oz: number): number => {
    let bi = 0, bd = -1;
    for (let i = 0; i < m; i++) { const d = (p[i]![0] - ox) ** 2 + (p[i]![1] - oy) ** 2 + (p[i]![2] - oz) ** 2; if (d > bd) { bd = d; bi = i; } }
    return bi;
  };
  const ia = far(cx, cy, cz), ib = far(p[ia]![0], p[ia]![1], p[ia]![2]);
  if (ia === ib) return false;
  const c1: Vec3[] = [], c2: Vec3[] = [];
  for (let i = ia; ; i = (i + 1) % m) { c1.push(p[i]!); if (i === ib) break; }
  for (let i = ib; ; i = (i + 1) % m) { c2.push(p[i]!); if (i === ia) break; }
  c2.reverse();
  const na = c1.length, nb = c2.length;
  if (na < 2 || nb < 2) return false;
  let i = 0, j = 0;
  while (i < na - 1 || j < nb - 1) {
    if (j >= nb - 1 || (i < na - 1 && (i + 1) / na <= (j + 1) / nb)) {
      emitTri(verts, faceIds, c1[i]!, c1[i + 1]!, c2[j]!, fid, surface, sign); i++;
    } else {
      emitTri(verts, faceIds, c1[i]!, c2[j + 1]!, c2[j]!, fid, surface, sign); j++;
    }
  }
  return true;
}

/**
 * Thin ANNULAR sliver: a face bounded by two SEPARATE closed-ring loops (an outer circle and a
 * concentric inner circle) that lie within `tol` of each other everywhere — a sub-tolerance flat
 * washer a CAD kernel leaves between two coincident rims (the knife-edge annulus that caps a Voron
 * stepper/Motor_Body cylinder). Its two loops project to two near-coincident circles in (u,v), so the
 * param grid can seat NO interior point (the whole ring is inside the boundary keep-out) and its CDT
 * cannot realise the ring constraints — the face meshes to garbage and its rims open against the
 * neighbouring cylinder. The two rims ARE the shared edge samples, so stitching them directly into a
 * ribbon (the same cyclic rail loft the revolution band uses) is watertight with both neighbours.
 * Returns false — leaving the param grid to handle it — unless the two loops are thin everywhere, so a
 * genuine annular FACE (a washer whose width ≫ tol, which the grid triangulates well) is untouched.
 */
function tessellateThinRing(
  surface: Surface, loops: BLoop[], sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], sign: number, tol: number,
): boolean {
  // Only a FLAT annular sliver: two concentric rims lying in the face plane. (A curved thin ring would
  // come as a periodic band, not two separate loops.) Restricting to planes lets the width be measured
  // from AREA and PERIMETER, which converge under sampling — a point-to-polyline distance does not (a
  // rim's chords bow by up to the sampling chord tolerance, which grows with part size, so on a metre-
  // scale part it cannot tell a 6µm sliver from a 0.5mm washer).
  if (loops.length !== 2 || surface.kind !== "PLANE") return false;
  const rail = (lp: BLoop): Vec3[] | null => {
    const p: Vec3[] = [];
    for (const oe of lp.edges) {
      const base = sampled.get(oe.edgeId); if (!base) return null;
      const poly = oe.orient ? base : base.slice().reverse();
      for (let i = 0; i < poly.length - 1; i++) p.push(poly[i]!);
    }
    return p.length >= 3 ? p : null;
  };
  const r1 = rail(loops[0]!), r2raw = rail(loops[1]!);
  if (!r1 || !r2raw) { if (DBG) console.error(`[tess] fid=${fid} thinRing bail: rail missing (${r1 ? r1.length : "null"}/${r2raw ? r2raw.length : "null"})`); return false; }
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  // Sampling-independent width = annulus area / mean circumference. Shoelace area in the plane's own
  // (u,v) (mm² for a plane) and 3D perimeter both converge as the rims are refined, so their ratio is
  // the true ring width regardless of how coarsely the part was sampled. A genuine washer's width is
  // orders of magnitude above `tol`; a CAD knife-edge sliver is a few microns.
  const perim = (r: Vec3[]): number => { let s = 0; for (let i = 0; i < r.length; i++) s += d3(r[i]!, r[(i + 1) % r.length]!); return s; };
  const area = (r: Vec3[]): number => {
    let a = 0;
    for (let i = 0, j = r.length - 1; i < r.length; j = i++) {
      const pi = surface.project(r[i]!), pj = surface.project(r[j]!);
      a += (pj[0] + pi[0]) * (pj[1] - pi[1]);
    }
    return Math.abs(a / 2);
  };
  const p1 = perim(r1), p2 = perim(r2raw);
  const width = Math.abs(area(r1) - area(r2raw)) / Math.max(1e-9, (p1 + p2) / 2);
  if (width > tol) { if (DBG) console.error(`[tess] fid=${fid} thinRing bail: width ${width.toExponential(2)} > ${tol}`); return false; }
  // Nesting guard: |A1−A2|/perimeter also reads ≈0 for two SIDE-BY-SIDE loops of similar area
  // (surfaced when the gate scaled up with part size) — and stitching those shreds two healthy
  // grid faces. A true annular sliver has its rims pointwise close; verify in 3D before stitching.
  const near = 3 * tol;
  const closed2 = [...r2raw, r2raw[0]!], closed1 = [...r1, r1[0]!];
  const st1 = Math.max(1, Math.floor(r1.length / 48)), st2 = Math.max(1, Math.floor(r2raw.length / 48));
  for (let i = 0; i < r1.length; i += st1) {
    if (distToPolyline(r1[i]!, closed2) > near) { if (DBG) console.error(`[tess] fid=${fid} thinRing bail: rims not nested (r1[${i}] > ${near.toExponential(2)})`); return false; }
  }
  for (let j = 0; j < r2raw.length; j += st2) {
    if (distToPolyline(r2raw[j]!, closed1) > near) { if (DBG) console.error(`[tess] fid=${fid} thinRing bail: rims not nested (r2[${j}] > ${near.toExponential(2)})`); return false; }
  }
  // Align rail 2 to rail 1: rotate to start nearest r1[0], then flip if the reverse tracks r1 better.
  let bi = 0, bd = Infinity;
  for (let i = 0; i < r2raw.length; i++) { const d = d3(r1[0]!, r2raw[i]!); if (d < bd) { bd = d; bi = i; } }
  let r2 = [...r2raw.slice(bi), ...r2raw.slice(0, bi)];
  if (d3(r1[1 % r1.length]!, r2[r2.length - 1]!) < d3(r1[1 % r1.length]!, r2[1 % r2.length]!)) {
    r2 = [r2[0]!, ...r2.slice(1).reverse()];
  }
  const frac = (c: Vec3[]): number[] => {
    const f = [0];
    for (let i = 1; i <= c.length; i++) f.push(f[i - 1]! + d3(c[i - 1]!, c[i % c.length]!));
    const tot = f[f.length - 1]! || 1;
    return f.slice(0, c.length).map((x) => x / tot);
  };
  stitchRings(verts, faceIds, r1, r2, fid, surface, sign, frac(r1), frac(r2));
  return true;
}

/**
 * Rail-ribbon fallback for a thin curved strip whose loops are DEGENERATE in parameter space — a
 * counterbore/hole rim, a rounded-corner blend, a lens between two nearly-parallel curves. The param
 * grid bails on these (the boundary projects to a ~zero-area sliver) and leaves a hole; the multi-loop
 * ones never reach tessellateThinFace either (it only takes a single loop). Such a face's boundary
 * reduces to exactly two distinct RAIL curves, however they are packaged: two edges of one loop (a
 * lens), or two loops each doubled "there and back" (a pinched strip, as some kernels emit a fillet
 * rim). We recover the two rails and loft a triangle ribbon between them. The rails ARE the shared edge
 * samples, so the ribbon stays watertight with its neighbours; the strip is thin enough that straight
 * rulings across it faithfully fill it. Returns false (emitting nothing) unless the boundary reduces to
 * exactly two rails — the caller then keeps its clean gap rather than a wrong fill.
 *
 * Two packagings are recognised. (A) Each rail is a single edge — a lens, or a pinched seam whose
 * doubled edge pair merges to one rail. (B) An annulus cut open by a SLIT: one loop traversing the
 * same edge twice, ring on one side of the cut, rim on the other (a fold-degenerate counterbore
 * tube whose param-grid result failed the fold audit) — dropping the slit splits the loop into
 * exactly two chains, lofted with wrap-around. Anything else (plain hole loops, >2 chains) is
 * ambiguous — lofting rings of a WIDE annulus would cut straight through 3D — and stays unmeshed:
 * a clean gap reads far better than a wrong fill.
 */
function tessellateRibbon(
  surface: Surface, loops: BLoop[], sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], sign: number,
): boolean {
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  type Entry = { id: number; poly: Vec3[] };
  const loopEntries: Entry[][] = [];
  for (const lp of loops) {
    const es: Entry[] = [];
    for (const oe of lp.edges) {
      const base = sampled.get(oe.edgeId);
      if (!base || base.length < 2) continue;
      es.push({ id: oe.edgeId, poly: oe.orient ? base.slice() : base.slice().reverse() });
    }
    if (es.length) loopEntries.push(es);
  }
  const flat = loopEntries.flat();
  if (flat.length < 2) return false;

  // Stitch an OPEN two-rail strip end-to-end by arc-length fraction (the original ribbon).
  const loftOpen = (c1: Vec3[], c2in: Vec3[]): boolean => {
    let c2 = c2in;
    if (d3(c1[0]!, c2[0]!) > d3(c1[0]!, c2[c2.length - 1]!)) c2 = c2.slice().reverse();
    const na = c1.length, nb = c2.length;
    if (na < 2 || nb < 2) return false;
    let i = 0, j = 0, emitted = 0;
    while (i < na - 1 || j < nb - 1) {
      if (j >= nb - 1 || (i < na - 1 && (i + 1) / na <= (j + 1) / nb)) {
        emitTri(verts, faceIds, c1[i]!, c1[i + 1]!, c2[j]!, fid, surface, sign); i++; emitted++;
      } else {
        emitTri(verts, faceIds, c1[i]!, c2[j + 1]!, c2[j]!, fid, surface, sign); j++; emitted++;
      }
    }
    return emitted > 0;
  };

  // Loft two CLOSED rails (an annulus strip: counterbore ring to its outer rim) with wrap-around,
  // paired by arc-length fraction. Winding is fixed by RAIL ORDER with one global flip decided by a
  // magnitude-weighted vote against the surface normal — emitTri's per-triangle projection would
  // checkerboard on a fold-degenerate surface (the two coincident folds carry OPPOSITE normals).
  const loftClosed = (c1in: Vec3[], c2in: Vec3[]): boolean => {
    const c1 = c1in.slice(0, -1);
    let c2 = c2in.slice(0, -1);
    if (c1.length < 3 || c2.length < 3) return false;
    // Rotate c2 to start nearest c1's start.
    let bi = 0, bd = Infinity;
    for (let i = 0; i < c2.length; i++) { const d = d3(c1[0]!, c2[i]!); if (d < bd) { bd = d; bi = i; } }
    c2 = [...c2.slice(bi), ...c2.slice(0, bi)];
    // Cumulative arc-length fractions including the closing segment (length n+1, last = 1).
    const fracOf = (c: Vec3[]): number[] => {
      const f = [0];
      for (let i = 1; i <= c.length; i++) f.push(f[i - 1]! + d3(c[i - 1]!, c[i % c.length]!));
      const tot = f[f.length - 1]! || 1;
      return f.map((x) => x / tot);
    };
    // The loop walks the two rails of a slit annulus in opposite senses; pick c2's direction by
    // which orientation tracks c1 more closely at matched fractions (nearest-point pairing).
    const at = (c: Vec3[], f: number[], t: number): Vec3 => {
      let i = 0;
      while (i + 1 < f.length && f[i + 1]! < t) i++;
      const d = f[i + 1]! - f[i]!;
      const w = d > 1e-12 ? (t - f[i]!) / d : 0;
      const a = c[i % c.length]!, b = c[(i + 1) % c.length]!;
      return [a[0] + (b[0] - a[0]) * w, a[1] + (b[1] - a[1]) * w, a[2] + (b[2] - a[2]) * w];
    };
    const track = (cc: Vec3[]): number => {
      const f1 = fracOf(c1), f2 = fracOf(cc);
      let s = 0;
      for (let k = 0; k < 16; k++) { const t = k / 16; s += d3(at(c1, f1, t), at(cc, f2, t)); }
      return s;
    };
    const rev = [c2[0]!, ...c2.slice(1).reverse()];
    if (track(rev) < track(c2)) c2 = rev;
    const f1 = fracOf(c1), f2 = fracOf(c2);
    const na = c1.length, nb = c2.length;
    const tris: [Vec3, Vec3, Vec3][] = [];
    let i = 0, j = 0;
    while (i < na || j < nb) {
      if (i < na && (j >= nb || f1[i + 1]! <= f2[j + 1]!)) { tris.push([c1[i % na]!, c1[(i + 1) % na]!, c2[j % nb]!]); i++; }
      else { tris.push([c1[i % na]!, c2[(j + 1) % nb]!, c2[j % nb]!]); j++; }
    }
    let vote = 0;
    for (const [a, b, c] of tris) {
      const ng = cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
      const cen: Vec3 = [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
      const [u, v] = surface.project(cen);
      vote += dot(ng, surface.normal(u, v)) * sign;
    }
    let emitted = 0;
    for (const [a, b, c] of tris) {
      const ng = cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
      if (ng[0] * ng[0] + ng[1] * ng[1] + ng[2] * ng[2] < 1e-18) continue;
      if (vote >= 0) verts.push(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
      else verts.push(a[0], a[1], a[2], c[0], c[1], c[2], b[0], b[1], b[2]);
      faceIds.push(fid);
      emitted++;
    }
    return emitted > 0;
  };

  const isClosed = (c: Vec3[]): boolean => d3(c[0]!, c[c.length - 1]!) < 1e-6;
  const tryLoft = (rails: Vec3[][] | null): boolean => {
    if (!rails || rails.length !== 2) return false;
    const [r1, r2] = [rails[0]!, rails[1]!];
    if (isClosed(r1) && isClosed(r2)) return loftClosed(r1, r2);
    return loftOpen(r1, r2);
  };

  // Packaging A — each rail is a single edge. Merge edges that trace the SAME curve twice (a pinched
  // seam: same endpoints AND same midpoint) — that doubled pair is one rail. Two edges sharing only
  // endpoints but bowing apart (a lens) stay distinct: the midpoint test is what tells a zero-width
  // seam from a real thin strip.
  const mid = (p: Vec3[]): Vec3 => p[p.length >> 1]!;
  const railsA = (): Vec3[][] => {
    const rails: Vec3[][] = [];
    const used = new Array(flat.length).fill(false);
    for (let i = 0; i < flat.length; i++) {
      if (used[i]) continue;
      used[i] = true;
      const pi = flat[i]!.poly;
      for (let j = i + 1; j < flat.length; j++) {
        if (used[j]) continue;
        const pj = flat[j]!.poly;
        const reversed = d3(pi[0]!, pj[pj.length - 1]!) < 1e-6 && d3(pi[pi.length - 1]!, pj[0]!) < 1e-6;
        if (reversed && d3(mid(pi), mid(pj)) < 1e-6) { used[j] = true; break; }
      }
      rails.push(pi);
    }
    return rails;
  };

  // Packaging B — an annulus cut open by a SLIT: one loop that traverses the same edge twice (out
  // and back), with the ring on one side of the cut and the rim on the other (Shapr3D counterbore
  // tubes). Dropping both slit traversals splits the cyclic edge sequence into runs; each run
  // concatenates into one rail. Only loops that actually contain a slit participate — a face that
  // mixes slit loops with plain hole loops is ambiguous and stays unmeshed rather than wrong.
  const railsB = (): Vec3[][] | null => {
    const chains: Vec3[][] = [];
    for (const es of loopEntries) {
      const count = new Map<number, number>();
      for (const e of es) count.set(e.id, (count.get(e.id) ?? 0) + 1);
      if (![...count.values()].some((c) => c === 2)) return null; // no slit in this loop
      const keep = es.map((e) => count.get(e.id) !== 2);
      if (!keep.some((k) => k)) return null; // nothing but slits
      const n = es.length;
      let anchor = 0;
      while (anchor < n && keep[anchor]) anchor++; // a dropped slot; exists since some edge doubled
      let run: Vec3[] | null = null;
      for (let s = 1; s <= n; s++) {
        const i = (anchor + s) % n;
        if (!keep[i]) { if (run && run.length >= 2) chains.push(run); run = null; continue; }
        const poly = es[i]!.poly;
        if (!run) run = poly.slice();
        else run.push(...(d3(run[run.length - 1]!, poly[0]!) < 1e-6 ? poly.slice(1) : poly));
      }
      if (run && run.length >= 2) chains.push(run);
    }
    return chains;
  };

  return tryLoft(railsA()) || tryLoft(railsB());
}

/**
 * Cone cap bounded by ONE once-winding rim of ARBITRARY shape — a hex-socket drive tip, a wavy
 * slice — where tessellateCone's constant-v circle-rim gate correctly bails (a hexagon's chord
 * edges dip below the rim latitude) and the param grid loses the wound polygon's constraints
 * (14 ABC chunk-0 models). Cone rulings are straight lines through the apex, so shrunken copies
 * of the rim polyline lerped toward the apex lie EXACTLY on the surface: loft rim → scaled rings
 * → apex fan. The rim samples are the shared edge polylines, so the loft is watertight with every
 * neighbour by construction. Returns false unless the loop winds the period exactly once and
 * stays clear of the apex (apex-touching slit cones belong to tessellateCone).
 */
function tessellateConeCap(
  surface: Surface, outerLoop: BLoop, sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
): boolean {
  const cone = surface as unknown as { r: number; sin: number };
  if (surface.kind !== "CONICAL_SURFACE" || !(cone.sin > 0)) return false;
  const apex = surface.evaluate(0, -cone.r / cone.sin);
  const dd = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  // An apex SLIT — a ruling from the rim down to the apex, walked out and back inside the same
  // loop (mecanum hub countersink whose rim is scalloped by the roller pockets) — is a parametric
  // seam, not rim: drop any edge with an endpoint at the apex. The slit's samples are referenced
  // by no other face (both traversals live in THIS loop) and the loft re-creates the ruling
  // continuously, so watertightness is unaffected. Wedge slices whose genuine trim edges end at
  // the apex lose those edges too, but their remaining rim is a partial arc and the winding gate
  // below still rejects them.
  let slant = 0;
  for (const oe of outerLoop.edges) {
    const base = sampled.get(oe.edgeId); if (!base || base.length === 0) return false;
    slant = Math.max(slant, dd(base[0]!, apex), dd(base[base.length - 1]!, apex));
  }
  const rim: Vec3[] = [];
  const mouths: Vec3[] = []; // dropped edges' rim-side endpoints
  for (const oe of outerLoop.edges) {
    const poly0 = sampled.get(oe.edgeId)!;
    const poly = oe.orient ? poly0 : poly0.slice().reverse();
    const a0 = dd(poly[0]!, apex) < 0.02 * slant, a1 = dd(poly[poly.length - 1]!, apex) < 0.02 * slant;
    if (a0 || a1) {
      if (!(a0 && a1)) mouths.push(a0 ? poly[poly.length - 1]! : poly[0]!);
      continue;
    }
    for (let i = 0; i < poly.length - 1; i++) rim.push(poly[i]!);
  }
  // ZERO-WIDTH slits only: all dropped legs must meet the rim at ONE point. Two rulings at
  // different angles bound a genuine pie-wedge cut — a wedge narrower than the winding gate's
  // tolerance would otherwise be lofted over. Those faces keep their old path (param grid).
  for (const m of mouths) if (dd(m, mouths[0]!) > 1e-3 * slant) return false;
  if (rim.length < 3) return false;
  const stride = Math.max(1, Math.ceil(rim.length / 512)); // winding gate needs samples, not every point
  let hu: number | undefined, hv: number | undefined, travel = 0, prevU: number | undefined;
  for (let i = 0; i < rim.length; i += stride) {
    const [u, v] = surface.project(rim[i]!, hu, hv);
    if (prevU !== undefined) {
      let d = u - prevU;
      while (d > Math.PI) d -= TWO_PI; while (d < -Math.PI) d += TWO_PI;
      travel += d;
    }
    prevU = u; hu = u; hv = v;
  }
  {
    // close the winding measurement rim-end -> rim-start
    const [u0] = surface.project(rim[0]!);
    let d = u0 - prevU!;
    while (d > Math.PI) d -= TWO_PI; while (d < -Math.PI) d += TWO_PI;
    travel += d;
  }
  if (Math.abs(Math.abs(travel) - TWO_PI) > 0.1 * TWO_PI) return false;
  let minD = Infinity, maxD = 0;
  for (const p of rim) { const d = dd(p, apex); if (d < minD) minD = d; if (d > maxD) maxD = d; }
  if (!(maxD > 0) || minD < 0.05 * maxD) return false;
  const target = Math.max(faceTarget(surface, targetEdge, chordTol, normalDev, hu ?? 0, hv ?? 0), 1e-9);
  const nRings = Math.max(1, Math.min(2000, Math.ceil(maxD / target)));
  const start = verts.length;
  let prev = rim.slice();
  for (let j = 1; j <= nRings; j++) {
    if (j === nRings) { stitchRings(verts, faceIds, prev, [apex], fid, surface, sign); break; }
    const t = 1 - j / nRings;
    const ring: Vec3[] = rim.map((p) => [apex[0] + t * (p[0] - apex[0]), apex[1] + t * (p[1] - apex[1]), apex[2] + t * (p[2] - apex[2])] as Vec3);
    stitchRings(verts, faceIds, prev, ring, fid, surface, sign);
    prev = ring;
  }
  return verts.length > start;
}

/** Cone: stack concentric rings from the shared base circle to the apex (cone is ruled by lines). */
function tessellateCone(
  surface: Surface, loop: BLoop, sampled: Map<number, Vec3[]>, brep: BrepModel, fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
  nFaceLoops: number,
): boolean {
  // Only a genuine apex cone (exactly one circle rim, tapering to a point) is handled here; a
  // frustum / trimmed cone has 2+ circle rims or straight sides and must go through the param grid
  // (which uses the shared edge samples on every side, so it stays watertight with its neighbours).
  // A rim is a FULL closed circle (start vertex == end vertex): a CIRCLE-typed edge or any closed
  // sampled loop (e.g. an INTERSECTION_CURVE wrapping a circle in legacy AP203 files). The closure
  // test is essential — a CIRCLE edge that is only a partial arc bounds a cone SLICE (a wedge cut by
  // two ruling seams), which must NOT be marched as a full revolution; it goes to tessellateConeSlice.
  const circleEdges = loop.edges.filter((oe) => {
    const e = brep.edges.get(oe.edgeId);
    if (!e) return false;
    const closed = Math.hypot(e.v0[0] - e.v1[0], e.v0[1] - e.v1[1], e.v0[2] - e.v1[2]) < 1e-9;
    if (!closed) return false;
    if (brep.table.typeOf(e.curveId) === "CIRCLE") return true;
    const p = sampled.get(oe.edgeId);
    return !!p && p.length >= 4;
  });
  let base: Vec3[] | null = null;
  let multiEdgeRim = false;
  if (circleEdges.length === 1) {
    const oe0 = circleEdges[0]!;
    const s0 = sampled.get(oe0.edgeId)!;
    base = oe0.orient ? s0.slice() : s0.slice().reverse();
    if (base.length < 4) return false;
    base = base.slice(0, base.length - 1); // drop duplicate closing point (keep index 0 = angular start)
  } else if (circleEdges.length === 0 && nFaceLoops === 1 && loop.edges.length >= 2) {
    // Full-period rim SPLIT into several arc edges (Inventor splits a countersink tip's base circle
    // at neighbouring slot corners): the sole loop's chained samples must close into ONE circle —
    // constant v (all on the rim) winding u exactly once — leaving the apex as the only closure.
    // Anything with samples off that circle (a genuine trimmed cone) falls through to the grid.
    // A slit apex cone (M5 button-head drive-socket cone) additionally carries an explicit SEAM
    // ruling from the rim down to the apex; that ruling is the parametric slit, not part of the rim,
    // so it is separated out here — the full-revolution ring march re-creates the seam continuously.
    const c2 = surface as Surface & { r: number; sin: number };
    const vApexP = -c2.r / c2.sin;
    // slant = the rim's own distance from the apex in v; an endpoint within 2% of it is "at the apex".
    let slant = 0;
    for (const oe of loop.edges) {
      const e = brep.edges.get(oe.edgeId); if (!e) return false;
      for (const p of [e.v0, e.v1]) slant = Math.max(slant, Math.abs(surface.project(p)[1] - vApexP));
    }
    const atApex = (p: Vec3): boolean => Math.abs(surface.project(p)[1] - vApexP) < 0.02 * Math.max(slant, 1e-9);
    const chain: Vec3[] = [];
    for (const oe of loop.edges) {
      const e = brep.edges.get(oe.edgeId);
      const s = sampled.get(oe.edgeId);
      if (!s || !e) return false;
      if (atApex(e.v0) || atApex(e.v1)) continue; // seam ruling to the apex — not part of the rim
      const poly = oe.orient ? s : s.slice().reverse();
      for (let i = 0; i < poly.length - 1; i++) chain.push(poly[i]!);
    }
    if (chain.length < 4) return false;
    const uv = chain.map((p) => surface.project(p));
    let vmin = Infinity, vmax = -Infinity;
    for (const q of uv) { if (q[1] < vmin) vmin = q[1]; if (q[1] > vmax) vmax = q[1]; }
    const slantV = Math.abs(vApexP - uv[0]![1]);
    if (vmax - vmin > 1e-3 * Math.max(slantV, 1e-9)) return false;
    if (Math.abs(cycleWind(uv as P2[], 0, TWO_PI)) !== 1) return false;
    base = chain;
    multiEdgeRim = true;
  } else return false;
  const cone = surface as Surface & { r: number; sin: number };
  const apex = surface.evaluate(0, -cone.r / cone.sin);
  const L = Math.hypot(base[0]![0] - apex[0], base[0]![1] - apex[1], base[0]![2] - apex[2]);
  // Genuine apex cone, by either signal:
  //  (a) some boundary vertex sits AT the apex (a degenerate seam edge carries the apex point), or
  //  (b) the face's SOLE boundary is the closed full-period base circle — no trim edges, no second
  //      rim — so the opposite side must close at the apex. (Its apex arrived as a VERTEX_LOOP, which
  //      carries no edges and is dropped at build time, so signal (a) is absent for these.)
  // A chamfer/countersink has one circle rim PLUS straight trim edges and its nearest vertex sits well
  // short of the apex; marching to the apex would spike a triangle clean out of the part. Such trimmed
  // cones go through the param grid instead (which respects all their edges).
  let minApex = Infinity;
  for (const oe of loop.edges) {
    const e = brep.edges.get(oe.edgeId); if (!e) continue;
    for (const v of [e.v0, e.v1]) minApex = Math.min(minApex, Math.hypot(v[0] - apex[0], v[1] - apex[1], v[2] - apex[2]));
  }
  let soleRim = multiEdgeRim && nFaceLoops === 1;
  if (!soleRim && circleEdges.length === 1) {
    const rimEdge = brep.edges.get(circleEdges[0]!.edgeId)!;
    const closedCircle = Math.hypot(rimEdge.v0[0] - rimEdge.v1[0], rimEdge.v0[1] - rimEdge.v1[1], rimEdge.v0[2] - rimEdge.v1[2]) < 1e-6;
    soleRim = nFaceLoops === 1 && loop.edges.length === 1 && closedCircle;
  }
  if (minApex > 0.05 * L && !soleRim) return false;
  const [theta0, vBase] = surface.project(base[0]!);
  const vApex = -cone.r / cone.sin, rBase = cone.r + vBase * cone.sin;
  let du = surface.project(base[1 % base.length]!)[0] - theta0; // base traversal direction in u
  while (du > Math.PI) du -= TWO_PI; while (du < -Math.PI) du += TWO_PI;
  const dir = du >= 0 ? 1 : -1;
  const target = faceTarget(surface, targetEdge, chordTol, normalDev, theta0, vBase);
  const nV = Math.max(1, Math.ceil(L / target));
  // March rim -> apex; the base ring keeps the SHARED samples (watertight with the cap). Each
  // interior ring is sized to ITS OWN (shrinking) circumference, so the count drops smoothly toward
  // the apex (no decimation bands, no pole-fan slivers); built from the base's start angle/direction.
  let prev = base.slice();
  for (let j = 1; j <= nV; j++) {
    const f = j / nV;
    if (j === nV) { stitchRings(verts, faceIds, prev, [apex], fid, surface, sign); break; }
    const vf = vBase + (vApex - vBase) * f, rf = Math.abs(rBase * (1 - f));
    // Ring point count from the LOCAL curvature at this ring's height, not the base target: the
    // normal curvature radius shrinks with the ring radius, so the base's allowed step violates the
    // chord tolerance near the apex (a 77mm/46° cone meshed 90° segments at rho=11 — 2.9mm sagitta).
    // The v-spacing (nV) may stay base-sized: rulings are straight, so it carries no chord error.
    const tf = faceTarget(surface, targetEdge, chordTol, normalDev, theta0, vf);
    const M = Math.max(3, Math.min(4000, Math.round((TWO_PI * rf) / tf)));
    const ring: Vec3[] = [];
    for (let k = 0; k < M; k++) ring.push(surface.evaluate(theta0 + (dir * TWO_PI * k) / M, vf));
    stitchRings(verts, faceIds, prev, ring, fid, surface, sign);
    prev = ring;
  }
  return true;
}

/**
 * Cone SLICE: an apex cone cut to a wedge by two straight ruling seams meeting at the apex, bounded
 * by one partial-arc rim (how AP242-e2 exporters split a cone that isn't a full revolution). In
 * (u,v) the rim projects to a horizontal collinear line and the wedge is a triangle-with-collinear-
 * base, which the flat CDT can't triangulate (it chords the whole rim, opening the seam). Mesh it
 * instead as rings marching from the rim down to the apex, laid out along the ARC's own angles at
 * each seam sample's v-level: every ring endpoint then lands exactly on a shared seam sample and the
 * rim ring is exactly the shared arc, so the wedge stays watertight with all three neighbours.
 * Returns false WITHOUT emitting unless the loop is exactly {2 ruling LINEs through the apex + 1
 * partial arc}; full apex cones (tessellateCone) and frustums (param grid) are left alone.
 */
function tessellateConeSlice(
  surface: Surface, loop: BLoop, sampled: Map<number, Vec3[]>, brep: BrepModel, fid: number,
  verts: number[], faceIds: number[], sign: number,
): boolean {
  const cone = surface as Surface & { r: number; sin: number };
  const apex = surface.evaluate(0, -cone.r / cone.sin);
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  // Classify edges GEOMETRICALLY, not by curve type: AP242-e2 writes every edge as a B-spline, so
  // a ruling seam is recognised as a STRAIGHT polyline with one endpoint at the apex, and the rim
  // is whatever remains — including a SLANTED-section rim (a drill-point cone cut by a part face,
  // nist_ftc_07/10's countersinks: two B-spline "lines" forming the rim diameter split at the
  // apex + a B-spline half-ellipse rim). The type-based version rejected those wedges to the param
  // grid, which walks the diameter THROUGH the apex into negative-radius (u,v), tangles, and drops
  // a constraint: 46 open edges per hole, ×16.
  const arcs: number[] = [], seams: number[] = [];
  let typed = true; // all seams LINE-typed and the rim CIRCLE-typed (the original wedge shape)
  for (const oe of loop.edges) {
    const e = brep.edges.get(oe.edgeId); if (!e) return false;
    const s = sampled.get(oe.edgeId);
    if (!s || s.length < 2) return false;
    const kind = brep.table.typeOf(e.curveId);
    const tol = 1e-6 * Math.max(1, d3(e.v0, apex), d3(e.v1, apex));
    let isSeam = d3(e.v0, apex) < tol || d3(e.v1, apex) < tol;
    if (isSeam) {
      const a = s[0]!, b = s[s.length - 1]!;
      const len = d3(a, b);
      if (len < 1e-9) isSeam = false;
      else {
        const inv = 1 / len;
        const dx = (b[0] - a[0]) * inv, dy = (b[1] - a[1]) * inv, dz = (b[2] - a[2]) * inv;
        for (const p of s) {
          const px = p[0] - a[0], py = p[1] - a[1], pz = p[2] - a[2];
          const t = px * dx + py * dy + pz * dz;
          if (Math.hypot(px - t * dx, py - t * dy, pz - t * dz) > 1e-5 * len + 1e-9) { isSeam = false; break; }
        }
      }
    }
    if (isSeam) { seams.push(oe.edgeId); if (kind !== "LINE") typed = false; }
    else { arcs.push(oe.edgeId); if (kind !== "CIRCLE") typed = false; }
  }
  const bail = (why: string): false => { if (DBG && process.env.MESHSTEP_SLICEDBG) console.error(`[coneSlice] fid=${fid} bail: ${why}`); return false; };
  if (arcs.length !== 1 || seams.length !== 2) return bail(`arcs=${arcs.length} seams=${seams.length}`);
  const arcEdge = brep.edges.get(arcs[0]!)!;
  if (d3(arcEdge.v0, arcEdge.v1) < 1e-9) return bail("closed rim"); // full circle -> tessellateCone / band
  // Both seams must run to the apex.
  const apexTol = 1e-6 * Math.max(1, d3(apex, arcEdge.v0));
  const seamCol = (id: number): Vec3[] | null => {
    const s = sampled.get(id); if (!s || s.length < 2) return null;
    const col = d3(s[0]!, apex) <= d3(s[s.length - 1]!, apex) ? s.slice() : s.slice().reverse();
    return d3(col[0]!, apex) <= apexTol ? col : null; // must start AT the apex
  };
  let Lc = seamCol(seams[0]!), Rc = seamCol(seams[1]!);
  if (!Lc || !Rc || Lc.length !== Rc.length) return bail(`seams: L=${Lc?.length ?? "far-from-apex"} R=${Rc?.length ?? "far-from-apex"} apexTol=${apexTol.toExponential(1)}`); // asymmetric sampling -> param grid
  const n = Lc.length;
  // Size cap: rings × arc samples has no interior grading (every ring keeps the full arc count all
  // the way to the apex), so a pathologically fine wedge would emit n·M·2 triangles unbounded.
  // The cap must stay GENEROUS: these wedges exist precisely because the param grid chords their
  // collinear rim open (a 100k cap re-routed nist_ctc_01's 150k-point wedges to the grid and opened
  // 2,516 edges). nist_ctc_02's wedge fan still exceeds the JS array limit at 0.002mm absolute
  // tolerance with ANY mesher — that is a documented capacity limit, not this cap's job.
  if (n * (sampled.get(arcs[0]!)?.length ?? 0) > 2_000_000) return false;
  // A GEOMETRICALLY-classified wedge (B-spline seams/rim, the ftc_07 countersink class) only
  // qualifies while SMALL: the fan is ungraded (full arc count on every ring), and stealing
  // ctc_04's ~280×760-sample wedges from the graded param grid — which handles them perfectly —
  // multiplied its output 60× and overflowed the JS array limit. Type-classified CIRCLE+LINE
  // wedges (ctc_01) keep the generous cap above, bit-for-bit.
  if (!typed && n * (sampled.get(arcs[0]!)?.length ?? 0) > 50_000) return bail(`geometric wedge too large (${n}×${sampled.get(arcs[0]!)?.length}) — graded grid handles it`);
  // Arc samples, oriented so index 0 is at the left seam's rim end.
  let arc = sampled.get(arcs[0]!)!.slice();
  const Lrim = Lc[n - 1]!, Rrim = Rc[n - 1]!;
  if (d3(arc[0]!, Lrim) > d3(arc[arc.length - 1]!, Lrim)) arc = arc.reverse();
  if (d3(arc[0]!, Lrim) > d3(arc[0]!, Rrim)) { const t = Lc; Lc = Rc; Rc = t; }
  const M = arc.length;
  if (M < 2) return false;
  // Arc angles, unwrapped monotonic so the intermediate rings don't fold at the ±π seam.
  const angs: number[] = arc.map((p) => surface.project(p)[0]);
  for (let i = 1; i < M; i++) { let d = angs[i]! - angs[i - 1]!; while (d > Math.PI) d -= TWO_PI; while (d < -Math.PI) d += TWO_PI; angs[i] = angs[i - 1]! + d; }
  // v-level of each seam sample (same on both seams by the length/tol checks above).
  const vAt = Lc.map((p) => surface.project(p)[1]);
  const ringAt = (k: number): Vec3[] => {
    if (k === 0) return [apex];
    const v = vAt[k]!;
    const r: Vec3[] = [];
    for (let j = 0; j < M; j++) r.push(surface.evaluate(angs[j]!, v));
    return r;
  };
  if (DBG && n * M > 100000) console.error(`[coneSlice] fid=${fid} n=${n} M=${M} -> ~${n * M * 2} tris`);
  // OPEN stitch (the wedge is not a full revolution, so rings must NOT wrap left-to-right).
  const openStitch = (A: Vec3[], B: Vec3[]): void => {
    if (A.length === 1) { for (let j = 0; j + 1 < B.length; j++) emitTri(verts, faceIds, A[0]!, B[j]!, B[j + 1]!, fid, surface, sign); return; }
    if (B.length === 1) { for (let j = 0; j + 1 < A.length; j++) emitTri(verts, faceIds, A[j]!, A[j + 1]!, B[0]!, fid, surface, sign); return; }
    for (let j = 0; j + 1 < Math.min(A.length, B.length); j++) {
      emitTri(verts, faceIds, A[j]!, A[j + 1]!, B[j]!, fid, surface, sign);
      emitTri(verts, faceIds, A[j + 1]!, B[j + 1]!, B[j]!, fid, surface, sign);
    }
  };
  let prev = ringAt(0);
  for (let k = 1; k < n; k++) {
    const ring = k === n - 1 ? arc : ringAt(k); // rim ring is exactly the shared arc samples
    openStitch(prev, ring);
    prev = ring;
  }
  return true;
}

/** Full sphere: concentric latitude rings, each sized to its own circumference so the poles taper
 * to a point instead of gathering a fan of slivers. */
/**
 * Bare untrimmed doubly-periodic surface — a full TORUS (spring/chain-link models are solids of
 * many such faces) or a closed-profile surface of revolution. The face has NO trimming loops, so
 * every boundary-driven mesher bails and the face used to be skipped outright (~65 ABC chunk-0
 * models). Mesh the full parametric domain as stacked closed rings, each sized to its own local
 * target (an eccentric tube's inner rings are tighter), stitched ring-to-ring; the last ring
 * re-evaluates at v0 + vPeriod == v0 so the weld closes the tube, and stitchRings' cyclic pairing
 * closes each ring in u.
 *
 * OPEN-PROFILE extension (periodic in u only, finite vDomain): some kernels emit an untrimmed
 * revolved segment — a bare cone/tube with NO loops at all — as its own 1-face solid inside a
 * sheet-style assembly whose neighbouring solids carry the rim circles as ordinary edges. The tube
 * is meshed as stacked rings over the v-domain, and each end rim BORROWS the sampled polyline of a
 * model edge that lies exactly on that rim circle (searched across ALL solids — `sampled` is
 * global), so the weld seals the tube to its neighbours point-for-point. Without a matching edge
 * the rim keeps its synthetic ring: the face is still meshed (previously skipped outright) and the
 * zips get a chance at the crack. Returns false when not periodic in u or no finite v-extent.
 */
function tessellateFullPeriodic(
  surface: Surface, fid: number, chordTol: number, targetEdge: number, normalDev: number,
  sign: number, verts: number[], faceIds: number[],
  brep?: BrepModel, sampled?: Map<number, Vec3[]>,
): boolean {
  if (!surface.periodicU) return false;
  const openV = !surface.periodicV;
  const vDom = surface.vDomain;
  if (openV && (!vDom || !(vDom[1] - vDom[0] > 1e-12))) return false;
  const uP = surface.uPeriod || TWO_PI, vP = openV ? vDom![1] - vDom![0] : surface.vPeriod || TWO_PI;
  const u0 = surface.uSeam ?? 0, v0 = openV ? vDom![0] : surface.vSeam ?? 0;
  const dd = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  // Borrow a rim: a model edge (any solid) whose sampled polyline is a closed ring lying exactly on
  // this face's rim circle. Its samples ARE the neighbour's boundary — reusing them welds exactly.
  const borrowRim = (v: number): Vec3[] | null => {
    if (!brep || !sampled) return null;
    const probe: Vec3[] = [];
    for (let i = 0; i < 8; i++) probe.push(surface.evaluate(u0 + (uP * i) / 8, v));
    let cx = 0, cy = 0, cz = 0;
    for (const q of probe) { cx += q[0]; cy += q[1]; cz += q[2]; }
    cx /= 8; cy /= 8; cz /= 8;
    let nx = 0, ny = 0, nz = 0, r = 0;
    for (let i = 0; i < 8; i++) {
      const a = probe[i]!, b = probe[(i + 1) % 8]!;
      nx += (a[1] - cy) * (b[2] - cz) - (a[2] - cz) * (b[1] - cy);
      ny += (a[2] - cz) * (b[0] - cx) - (a[0] - cx) * (b[2] - cz);
      nz += (a[0] - cx) * (b[1] - cy) - (a[1] - cy) * (b[0] - cx);
      r += dd(a, [cx, cy, cz]);
    }
    r /= 8;
    const nl = Math.hypot(nx, ny, nz);
    if (nl < 1e-18 || r < 1e-9) return null; // degenerate rim (pole) — nothing to borrow
    nx /= nl; ny /= nl; nz /= nl;
    const off = (q: Vec3): number => {
      const dx = q[0] - cx, dy = q[1] - cy, dz = q[2] - cz;
      const h = dx * nx + dy * ny + dz * nz;
      const rad = Math.hypot(dx - h * nx, dy - h * ny, dz - h * nz);
      return Math.hypot(rad - r, h);
    };
    const eps = Math.max(1e-6, 1e-6 * r);
    for (const [id, e] of brep.edges) {
      if (off(e.v0) > eps || off(e.v1) > eps) continue;
      const poly = sampled.get(id);
      if (!poly || poly.length < 4) continue;
      if (dd(poly[0]!, poly[poly.length - 1]!) > eps) continue; // want a full closed ring
      let onRing = true;
      for (const q of poly) if (off(q) > eps) { onRing = false; break; }
      if (!onRing) continue;
      return poly.slice(0, -1);
    }
    return null;
  };
  // Align a borrowed ring to the synthetic ring's start/direction so stitchRings' fraction pairing
  // doesn't twist the tube.
  const alignRing = (ref: Vec3[], ring: Vec3[]): Vec3[] => {
    let bi = 0, bd = Infinity;
    for (let i = 0; i < ring.length; i++) { const d = dd(ref[0]!, ring[i]!); if (d < bd) { bd = d; bi = i; } }
    let out = [...ring.slice(bi), ...ring.slice(0, bi)];
    const q = ref[Math.floor(ref.length / 4)]!; // quarter-way probe picks the direction
    const rev = [out[0]!, ...out.slice(1).reverse()];
    const at = (c: Vec3[], f: number): Vec3 => c[Math.min(c.length - 1, Math.round(f * c.length)) % c.length]!;
    if (dd(q, at(rev, 0.25)) < dd(q, at(out, 0.25))) out = rev;
    return out;
  };
  // v-step count from the longest v-iso arc (sampled at a few u) against the finest local target.
  const K = 32;
  let vArc = 0, tMin = targetEdge;
  for (const fu of [0, 1 / 3, 2 / 3]) {
    const u = u0 + uP * fu;
    let arc = 0, prev = surface.evaluate(u, v0);
    for (let i = 1; i <= K; i++) {
      const q = surface.evaluate(u, v0 + (vP * i) / K);
      arc += dd(prev, q); prev = q;
      tMin = Math.min(tMin, faceTarget(surface, targetEdge, chordTol, normalDev, u, v0 + (vP * i) / K));
    }
    vArc = Math.max(vArc, arc);
  }
  if (vArc < 1e-9) return false;
  const nV = Math.max(4, Math.min(2000, Math.ceil(vArc / Math.max(tMin, 1e-9))));
  const ringAt = (v: number): Vec3[] => {
    let circ = 0, prev = surface.evaluate(u0, v);
    for (let i = 1; i <= K; i++) { const q = surface.evaluate(u0 + (uP * i) / K, v); circ += dd(prev, q); prev = q; }
    const t = faceTarget(surface, targetEdge, chordTol, normalDev, u0 + uP / 2, v);
    const nu = Math.max(4, Math.min(4000, Math.round(circ / Math.max(t, 1e-9))));
    const r: Vec3[] = [];
    for (let i = 0; i < nu; i++) r.push(surface.evaluate(u0 + (uP * i) / nu, v));
    return r;
  };
  const start = verts.length;
  const rimAt = (v: number): Vec3[] => {
    if (!openV) return ringAt(v);
    const synth = ringAt(v);
    const b = borrowRim(v);
    if (DBG && b) console.error(`[tess] fid=${fid} fullPeriodic: borrowed ${b.length}-pt rim at v=${v}`);
    return b ? alignRing(synth, b) : synth;
  };
  let prev = rimAt(v0);
  for (let j = 1; j <= nV; j++) {
    const ring = j === nV ? rimAt(v0 + vP) : ringAt(v0 + (vP * j) / nV);
    stitchRings(verts, faceIds, prev, ring, fid, surface, sign);
    prev = ring;
  }
  return verts.length > start;
}

/**
 * Generic single-rim CAP/DOME: a face whose sole loop is one closed rim winding the full period of
 * the surface's periodic direction at near-constant stack parameter, on a surface that COLLAPSES
 * to a point at a stack-domain end — a surface-of-revolution dome (profile touches the axis) or a
 * closed-direction B-spline dome. The generalisation of tessellateSphereCap: the rim ring (the
 * shared edge samples, watertight with the neighbour) is marched to the pole in graded rings and
 * fanned at the pole. ~95 ABC chunk-0 untriangulated models are exactly this shape (`REVOLUTION
 * 1loop` + `B_SPLINE PvCv/PuCu 1loop`): the rim projects to a constant-stack line enclosing zero
 * parameter area, so the param grid rejects the face, and band/unroll need two rims. When BOTH
 * stack ends are poles (a closed drop/lobe), the enclosed side follows the sphere-cap orientation
 * convention: material lies left of the oriented rim, flipped with the face sense. Returns false
 * without emitting unless every gate holds.
 */
function tessellateCapDome(
  surface: Surface, loop: BLoop, sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], chordTol: number, targetEdge: number, normalDev: number, sign: number,
): boolean {
  const uPer = !!surface.periodicU, vPer = !!surface.periodicV;
  let P: 0 | 1;
  if (uPer && !vPer) P = 0; else if (vPer && !uPer) P = 1; else return false;
  const S = (1 - P) as 0 | 1;
  const sDom = P === 0 ? surface.vDomain : surface.uDomain;
  if (!sDom) return false;
  const period = (P === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI;
  const evalPS = (p: number, s: number): Vec3 => (P === 0 ? surface.evaluate(p, s) : surface.evaluate(s, p));

  // Rim = the loop's once-used edges chained (a doubled edge is a degenerate pole seam, excluded).
  const count = new Map<number, number>();
  for (const oe of loop.edges) count.set(oe.edgeId, (count.get(oe.edgeId) ?? 0) + 1);
  const rim: Vec3[] = [];
  for (const oe of loop.edges) {
    if ((count.get(oe.edgeId) ?? 0) !== 1) continue;
    const base = sampled.get(oe.edgeId); if (!base) return false;
    const poly = oe.orient ? base : base.slice().reverse();
    for (let i = 0; i < poly.length - 1; i++) rim.push(poly[i]!);
  }
  if (rim.length < 4) return false;
  // Hint-chained projection so the rim unwraps continuously across the periodic seam. STRIDED to
  // ≤512 samples: the winding/spread gates don't need every point, and projecting a 30k-point
  // B-spline rim point-by-point (multi-start Newton each) turned an 11s model into a >150s hang
  // (ABC 00008375). 512 samples of a once-winding rim step ~period/512 — far below the half-period
  // wrap ambiguity.
  const stride = Math.max(1, Math.ceil(rim.length / 512));
  const uv: P2[] = [];
  let hu: number | undefined, hv: number | undefined;
  for (let i = 0; i < rim.length; i += stride) { const q = surface.project(rim[i]!, hu, hv); uv.push(q); hu = q[0]; hv = q[1]; }
  // Full single winding in the periodic direction, near-constant in the stack direction.
  let travel = 0;
  for (let i = 0; i < uv.length; i++) {
    let d = uv[(i + 1) % uv.length]![P]! - uv[i]![P]!;
    while (d > period / 2) d -= period; while (d < -period / 2) d += period;
    travel += d;
  }
  if (Math.abs(Math.abs(travel) - period) > 0.1 * period) return false;
  let smin = Infinity, smax = -Infinity;
  for (const q of uv) { const s = q[S]!; if (s < smin) smin = s; if (s > smax) smax = s; }
  const sSpan = Math.max(sDom[1] - sDom[0], 1e-12);
  if (smax - smin > 0.2 * sSpan) return false;
  const sRim = (smin + smax) / 2;

  const dd = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  // A stack end is a POLE when its full-period ring collapses to a point. In a closed B-rep the
  // enclosed end of a single-rim face can only be degenerate (a real rim there would need its own
  // loop), so the tolerance need not be razor-thin — but it must stay below feature scale.
  const ringSpread = (s: number): number => {
    let mx = 0; const c = evalPS(0, s);
    for (let i = 1; i <= 8; i++) mx = Math.max(mx, dd(c, evalPS((period * i) / 8, s)));
    return mx;
  };
  const poleTol = Math.max(1e-3, 0.1 * chordTol);
  const pole0 = ringSpread(sDom[0]) < poleTol, pole1 = ringSpread(sDom[1]) < poleTol;
  let sPole: number;
  if (pole0 !== pole1) {
    // One degenerate end: the face can only close there — but only if the rim sits between it and
    // the other end, i.e. the non-pole side of the rim carries the neighbour faces.
    sPole = pole0 ? sDom[0] : sDom[1];
  } else if (pole0 && pole1) {
    // Both ends are poles (a closed drop): material lies LEFT of the oriented rim (sphere-cap
    // convention), flipped with the face sense. In (u,v) axes, left of +u travel is +v; left of
    // +v travel is -u.
    const leftIsPlusS = P === 0 ? travel >= 0 : travel < 0;
    const enclosedPlusS = leftIsPlusS === (sign > 0);
    sPole = enclosedPlusS ? sDom[1] : sDom[0];
  } else return false;

  // March graded rings from the rim to the pole; ring sizes track their own circumference.
  const target = faceTarget(surface, targetEdge, chordTol, normalDev, P === 0 ? uv[0]![0] : sRim, P === 0 ? sRim : uv[0]![1]);
  let sArc = 0;
  {
    const p0 = uv[0]![P]!;
    let prev = evalPS(p0, sRim);
    for (let i = 1; i <= 16; i++) { const q = evalPS(p0, sRim + ((sPole - sRim) * i) / 16); sArc += dd(prev, q); prev = q; }
  }
  // Ring-count cap keyed to the RIM size: total evaluations nV·|rim| stay ~1.5M so one dome face
  // cannot eat the whole model budget (a dense B-spline rim × 2000 rings is 60M evaluate() calls).
  const nV = Math.max(1, Math.min(2000, Math.ceil(1.5e6 / Math.max(rim.length, 8)), Math.ceil(sArc / Math.max(target, 1e-9))));
  const pStart = uv[0]![P]!, pDir = travel >= 0 ? 1 : -1;
  const start = verts.length;
  let prev = rim.slice();
  for (let j = 1; j <= nV; j++) {
    const s = sRim + ((sPole - sRim) * j) / nV;
    if (j === nV) { stitchRings(verts, faceIds, prev, [evalPS(pStart, sPole)], fid, surface, sign); break; }
    let circ = 0;
    {
      let pv = evalPS(pStart, s);
      for (let i = 1; i <= 16; i++) { const q = evalPS(pStart + pDir * (period * i) / 16, s); circ += dd(pv, q); pv = q; }
    }
    const t = faceTarget(surface, targetEdge, chordTol, normalDev, P === 0 ? pStart : s, P === 0 ? s : pStart);
    const nu = Math.max(3, Math.min(4000, Math.round(circ / Math.max(t, 1e-9))));
    const ring: Vec3[] = [];
    for (let i = 0; i < nu; i++) ring.push(evalPS(pStart + pDir * (period * i) / nu, s));
    stitchRings(verts, faceIds, prev, ring, fid, surface, sign);
    prev = ring;
  }
  return verts.length > start;
}

/**
 * PERIODIC REGION EXTRACTION — the multi-loop core of the ABC "untriangulated" class. A face on a
 * surface periodic in direction P whose loops give the param grid NO usable outer boundary: every
 * loop either WINDS the period (a rim crossing the seam — its projection's net area cancels, so
 * the zero-width gate rejects it) or is a compact HOLE with no outer loop at all (a drilled
 * sphere/ball: the "outer" region is the entire closed surface). The region is rebuilt on the
 * universal cover as a BAND in the stack direction S: between two winding rims (a sphere zone cut
 * by two tilted circles), between one rim and the enclosed collapsed domain end (a cap whose rim
 * is NOT an iso-parallel, which tessellateCapDome refuses), or the whole S-domain when only holes
 * exist. The band is closed with a SEAM POLYLINE pair — identical parameter points exactly one
 * period apart, so both copies evaluate to identical 3D points and the weld seals the tube; pole
 * rows collapse under the weld into fans. Rim and hole boundary points keep their exact shared
 * edge samples — watertight with every neighbour. Compact holes are punched as CDT holes.
 * Conservative: bails (emitting nothing) unless every loop lifts untangled and classifies as a
 * ±1-wind rim or 0-wind hole, a hole-free seam corridor exists, every hole lands inside the band,
 * and the CDT realises every constraint.
 */
function tessellatePeriodicRegion(
  surface: Surface, loops: BLoop[], sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
): boolean {
  const uPer = !!surface.periodicU, vPer = !!surface.periodicV;
  if ((!uPer && !vPer) || loops.length === 0) return false;
  // Lift every loop first; the band direction P follows the observed winding (on a torus a rim can
  // wind either way — take the direction the rims actually wind).
  // Tangled lifts are allowed through: the assembled band polygon's simplicity check and the CDT
  // missing==0 gate reject any lift that actually broke, and some genuinely meshable rims (a cap
  // rim wobbling across the seam) carry the tangled flag from the repair search.
  const lifted = loops.map((lp) => loopParam(surface, lp, sampled, 0));
  for (const l of lifted) {
    if (l.p2.length < 3 || l.p2.length !== l.p3.length) {
      if (DBG) console.error(`[tess] fid=${fid} region bail: lift p2=${l.p2.length} p3=${l.p3.length}`);
      return false;
    }
  }
  let P: 0 | 1;
  if (uPer && !vPer) P = 0;
  else if (vPer && !uPer) P = 1;
  else {
    const wU = lifted.some((l) => l.windU !== 0), wV = lifted.some((l) => l.windV !== 0);
    if (wU && wV) { if (DBG) console.error(`[tess] fid=${fid} region bail: loops wind both directions`); return false; }
    P = wV ? 1 : 0;
  }
  const S = (1 - P) as 0 | 1;
  const sPeriodic = uPer && vPer;
  const period = (P === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI;
  const sPeriod = sPeriodic ? ((S === 0 ? surface.uPeriod : surface.vPeriod) || TWO_PI) : 0;
  const sSeam = sPeriodic ? ((S === 0 ? surface.uSeam : surface.vSeam) ?? 0) : 0;
  const sDomRaw = P === 0 ? surface.vDomain : surface.uDomain;
  const sDom: [number, number] | null = sPeriodic ? [sSeam, sSeam + sPeriod]
    : sDomRaw ? [sDomRaw[0]!, sDomRaw[1]!]
      : isSphere(surface) ? [-Math.PI / 2, Math.PI / 2] : null;
  const dd = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const evalPS = (p: number, s: number): Vec3 => (P === 0 ? surface.evaluate(p, s) : surface.evaluate(s, p));

  // Classify: ±1 winding rim / compact hole; anything else is not band-shaped.
  type LL = { p2: P2[]; p3: Vec3[]; wind: number; meanS: number };
  const rims: LL[] = [], holes: LL[] = [];
  for (const l of lifted) {
    const wP = P === 0 ? l.windU : l.windV;
    const wS = P === 0 ? l.windV : l.windU;
    if (wS !== 0) { if (DBG) console.error(`[tess] fid=${fid} region bail: loop winds S (${l.windU}/${l.windV})`); return false; }
    let mean = 0; for (const q of l.p2) mean += q[S]!; mean /= l.p2.length;
    const rec = { p2: l.p2, p3: l.p3, wind: wP, meanS: mean };
    if (wP === 1 || wP === -1) rims.push(rec);
    else if (wP === 0) holes.push(rec);
    else { if (DBG) console.error(`[tess] fid=${fid} region bail: multi-wind ${wP}`); return false; }
  }
  if (rims.length > 2) { if (DBG) console.error(`[tess] fid=${fid} region bail: ${rims.length} rims`); return false; }
  if (!sPeriodic && rims.length === 2 && rims[0]!.wind === rims[1]!.wind) return false;
  if (rims.length < 2 && !sDom) return false; // open stack (cylinder/cone) needs rims for S-bounds

  // Metric scales at the band's rough middle (for CDT conditioning, grid sizing, seam density).
  const sMid = rims.length === 2 ? (rims[0]!.meanS + rims[1]!.meanS) / 2 : sDom ? (sDom[0] + sDom[1]) / 2 : rims[0]!.meanS;
  const arcOver = (dir: 0 | 1, from: number, to: number, at: number): number => {
    let arc = 0;
    let prev = dir === P ? evalPS(from, at) : evalPS(at, from);
    for (let i = 1; i <= 16; i++) {
      const t = from + ((to - from) * i) / 16;
      const q = dir === P ? evalPS(t, at) : evalPS(at, t);
      arc += dd(prev, q); prev = q;
    }
    return arc;
  };
  const p0 = rims.length ? rims[0]!.p2[0]![P]! : 0;
  const circMid = arcOver(P, p0, p0 + period, sMid);
  const target = Math.max(faceTarget(surface, targetEdge, chordTol, normalDev, P === 0 ? p0 : sMid, P === 0 ? sMid : p0), 1e-9);
  if (circMid < 1e-9) return false;
  const mP = circMid / period;

  // Pick the seam corridor u* — a P-value crossed by no hole (try the widest gaps between hole
  // spans; a rim is crossed by the seam at exactly one reanchored sample, which is fine).
  const holeSpan = (h: LL): [number, number] => {
    let mn = Infinity, mx = -Infinity;
    for (const q of h.p2) { if (q[P] < mn) mn = q[P]; if (q[P] > mx) mx = q[P]; }
    return [mn, mx];
  };
  const norm = (x: number): number => ((x % period) + period) % period;
  const spans = holes.map(holeSpan);
  const candidates: number[] = [];
  if (spans.length === 0) candidates.push(p0 + period / 2, p0 + period / 4, p0);
  else {
    // Circular gaps between hole spans (spans wider than the period leave no corridor).
    const marks = spans.map(([mn, mx]) => [norm(mn), norm(mn) + Math.min(mx - mn, period)] as [number, number])
      .sort((a, b) => a[0] - b[0]);
    for (let i = 0; i < marks.length; i++) {
      const end = marks[i]![1];
      const nextStart = i + 1 < marks.length ? marks[i + 1]![0] : marks[0]![0] + period;
      if (nextStart - end > 1e-6 * period) candidates.push(norm((end + nextStart) / 2));
    }
    candidates.sort((a, b) => {
      const gap = (c: number): number => {
        let g = Infinity;
        for (const [mn, mx] of marks) {
          for (const off of [-period, 0, period]) {
            const lo = mn + off, hi = mx + off;
            if (c >= lo && c <= hi) return -1;
            g = Math.min(g, Math.min(Math.abs(c - lo), Math.abs(c - hi)));
          }
        }
        return g;
      };
      return gap(b) - gap(a);
    });
  }
  if (!candidates.length) return false;

  // Re-anchor a rim cycle to start at its sample nearest u*, continuity-unwrapped over one period.
  const chainFrom = (l: LL, uStar: number): { p2: P2[]; p3: Vec3[] } => {
    const n = l.p2.length;
    let k = 0, bd = Infinity;
    for (let i = 0; i < n; i++) {
      const d = Math.min(norm(l.p2[i]![P]! - uStar), period - norm(l.p2[i]![P]! - uStar));
      if (d < bd) { bd = d; k = i; }
    }
    const first: P2 = [0, 0];
    first[P] = uStar + (norm(l.p2[k]![P]! - uStar) <= period / 2 ? norm(l.p2[k]![P]! - uStar) : norm(l.p2[k]![P]! - uStar) - period);
    first[S] = l.p2[k]![S]!;
    const p2: P2[] = [first];
    const p3: Vec3[] = [l.p3[k]!];
    let prevP = first[P]!;
    for (let t = 1; t <= n; t++) {
      const i = (k + t) % n;
      const q: P2 = [l.p2[i]![0]!, l.p2[i]![1]!];
      let d = q[P]! - prevP;
      while (d > period / 2) { q[P] -= period; d -= period; }
      while (d < -period / 2) { q[P] += period; d += period; }
      prevP = q[P]!;
      p2.push(q); p3.push(l.p3[i]!);
    }
    return { p2, p3 };
  };

  // The two S-bounds of the band: rim chains or synthetic domain-end rows (poles collapse; a
  // closed-but-unflagged profile's two end rows evaluate to the same 3D ring and weld shut).
  const syntheticChain = (sEnd: number, uStar: number, asc: boolean): { p2: P2[]; p3: Vec3[] } => {
    const circ = arcOver(P, uStar, uStar + period, sEnd);
    const nP = Math.max(8, Math.min(1024, Math.ceil(circ / target)));
    const p2: P2[] = [], p3: Vec3[] = [];
    for (let i = 0; i <= nP; i++) {
      const u = asc ? uStar + (period * i) / nP : uStar + period - (period * i) / nP;
      const q: P2 = [0, 0]; q[P] = u; q[S] = sEnd;
      p2.push(q); p3.push(evalPS(u, sEnd));
    }
    return { p2, p3 };
  };
  const ascending = (c: { p2: P2[]; p3: Vec3[] }): { p2: P2[]; p3: Vec3[] } =>
    c.p2[c.p2.length - 1]![P]! >= c.p2[0]![P]! ? c : { p2: c.p2.slice().reverse(), p3: c.p3.slice().reverse() };
  const descending = (c: { p2: P2[]; p3: Vec3[] }): { p2: P2[]; p3: Vec3[] } =>
    c.p2[c.p2.length - 1]![P]! <= c.p2[0]![P]! ? c : { p2: c.p2.slice().reverse(), p3: c.p3.slice().reverse() };

  const shiftS = (l: LL, by: number): LL => ({
    p2: l.p2.map((q) => { const r: P2 = [q[0]!, q[1]!]; r[S] = r[S]! + by; return r; }),
    p3: l.p3, wind: l.wind, meanS: l.meanS + by,
  });
  let sLowEnd: number | null = null, sHighEnd: number | null = null; // synthetic ends (else rim)
  let rimLow: LL | null = null, rimHigh: LL | null = null;
  if (sPeriodic && rims.length === 1) {
    // Cutting a doubly-periodic surface along ONE winding loop leaves a cylinder: the same rim
    // bounds the band on both sides, one S-period apart (identical 3D points — the weld closes it).
    rimLow = rims[0]!;
    rimHigh = shiftS(rims[0]!, sPeriod);
  } else if (rims.length === 2) {
    let [a, b] = rims[0]!.meanS <= rims[1]!.meanS ? [rims[0]!, rims[1]!] : [rims[1]!, rims[0]!];
    // On a doubly-periodic surface the two rims' S-branches are arbitrary: bring b within one
    // S-period above a.
    if (sPeriodic) {
      const k = Math.round((a.meanS + sPeriod / 2 - b.meanS) / sPeriod);
      if (k !== 0) b = shiftS(b, k * sPeriod);
      if (b.meanS < a.meanS) [a, b] = [b, a];
    }
    rimLow = a; rimHigh = b;
  } else if (rims.length === 1) {
    if (!sDom) return false;
    const rim = rims[0]!;
    // Enclosed side: material lies LEFT of the oriented rim, flipped with the face sense
    // (the sphere-cap convention; travel sign = wind sign).
    const leftIsPlusS = P === 0 ? rim.wind > 0 : rim.wind < 0;
    const enclosedPlusS = leftIsPlusS === (sign > 0);
    if (enclosedPlusS) { rimLow = rim; sHighEnd = sDom[1]; }
    else { rimHigh = rim; sLowEnd = sDom[0]; }
  } else {
    if (!sDom) return false;
    sLowEnd = sDom[0]; sHighEnd = sDom[1];
  }

  // After orientation flips a chain's window can sit one period off; re-anchor: the ascending low
  // chain STARTS near u*, the descending high chain ENDS near u*.
  const anchor = (c: { p2: P2[]; p3: Vec3[] }, uStar: number, byEnd: boolean): { p2: P2[]; p3: Vec3[] } => {
    const refU = byEnd ? c.p2[c.p2.length - 1]![P]! : c.p2[0]![P]!;
    const k = Math.round((uStar - refU) / period);
    if (k === 0) return c;
    return { p2: c.p2.map((q) => { const r: P2 = [q[0]!, q[1]!]; r[P] = r[P]! + k * period; return r; }), p3: c.p3 };
  };
  let cdtFails = 0; // an expensive CDT-missing failure rarely improves with another corridor — cap at 2
  for (const uStar of candidates.slice(0, 4)) {
    const low = anchor(ascending(rimLow ? chainFrom(rimLow, uStar) : syntheticChain(sLowEnd!, uStar, true)), uStar, false);
    const high = anchor(descending(rimHigh ? chainFrom(rimHigh, uStar) : syntheticChain(sHighEnd!, uStar, false)), uStar, true);
    // Right seam: from low's end straight (in parameter) to high's start; left seam = same points
    // one period down. Subdivided by metric length so the two identical 3D copies weld shut.
    const A = low.p2[low.p2.length - 1]!, B = high.p2[0]!;
    if (Math.abs(A[P]! - B[P]!) > 0.3 * period) { if (DBG) console.error(`[tess] fid=${fid} region: cut drift ${(A[P]! - B[P]!).toFixed(3)} at u*=${uStar.toFixed(3)}`); continue; }
    const seamMetricLen = Math.hypot((B[P]! - A[P]!) * mP, arcOver(S, A[S]!, B[S]!, A[P]!));
    const nSeam = Math.max(1, Math.min(2048, Math.ceil(seamMetricLen / target)));
    const seamR: { p2: P2[]; p3: Vec3[] } = { p2: [], p3: [] };
    for (let i = 1; i < nSeam; i++) {
      const t = i / nSeam;
      const q: P2 = [0, 0];
      q[P] = A[P]! + (B[P]! - A[P]!) * t;
      q[S] = A[S]! + (B[S]! - A[S]!) * t;
      seamR.p2.push(q); seamR.p3.push(evalPS(q[P]!, q[S]!));
    }
    const seamL: { p2: P2[]; p3: Vec3[] } = { p2: [], p3: [] };
    for (let i = seamR.p2.length - 1; i >= 0; i--) {
      const q: P2 = [seamR.p2[i]![0]!, seamR.p2[i]![1]!];
      q[P] = q[P]! - period;
      seamL.p2.push(q); seamL.p3.push(seamR.p3[i]!); // identical 3D point — the weld seals the seam
    }
    // Assemble the outer polygon: low (asc) + right seam + high (desc) + left seam.
    const outerP2: P2[] = [...low.p2, ...seamR.p2, ...high.p2, ...seamL.p2];
    const outerP3: Vec3[] = [...low.p3, ...seamR.p3, ...high.p3, ...seamL.p3];
    if (outerP2.length > 40000) { if (DBG) console.error(`[tess] fid=${fid} region: band boundary too dense (${outerP2.length})`); return false; }
    if (outerP2.length < 3 || countSelfIntersections(outerP2, 1) > 0) { if (DBG) console.error(`[tess] fid=${fid} region: band polygon self-intersects at u*=${uStar.toFixed(3)}`); continue; }
    // Holes: shift each into the band window, verify it lands inside the outer polygon.
    const holeP2: P2[][] = [], holeP3: Vec3[][] = [];
    let holesOk = true;
    for (const h of holes) {
      let hp = h.p2.map((q) => [q[0]!, q[1]!] as P2);
      let cen: P2 = [0, 0];
      const centroid = (): void => { cen = [0, 0]; for (const q of hp) { cen[0] += q[0]!; cen[1] += q[1]!; } cen[0] /= hp.length; cen[1] /= hp.length; };
      centroid();
      const lo = Math.min(outerP2[0]![P]!, A[P]! - period);
      let shift = 0;
      while (cen[P]! + shift < lo) shift += period;
      while (cen[P]! + shift > lo + period) shift -= period;
      hp = hp.map((q) => { const r: P2 = [q[0]!, q[1]!]; r[P] = r[P]! + shift; return r; });
      centroid();
      let inside = pointInPoly(cen, outerP2);
      // Wrong period branch: retry ±period in P, and on a doubly-periodic surface ±period in S too.
      const alts: [number, number][] = [[period, 0], [-period, 0]];
      if (sPeriodic) alts.push([0, sPeriod], [0, -sPeriod], [period, sPeriod], [period, -sPeriod], [-period, sPeriod], [-period, -sPeriod]);
      for (const [aP, aS] of alts) {
        if (inside) break;
        const test = hp.map((q) => { const r: P2 = [q[0]!, q[1]!]; r[P] = r[P]! + aP; r[S] = r[S]! + aS; return r; });
        let c2: P2 = [0, 0]; for (const q of test) { c2[0] += q[0]!; c2[1] += q[1]!; } c2[0] /= test.length; c2[1] /= test.length;
        if (pointInPoly(c2, outerP2)) { hp = test; inside = true; }
      }
      if (!inside) { holesOk = false; break; }
      holeP2.push(hp); holeP3.push(h.p3);
    }
    if (!holesOk) { if (DBG) console.error(`[tess] fid=${fid} region: hole outside band at u*=${uStar.toFixed(3)}`); continue; }

    // Interior grid over the band's bounding box, kept clear of every boundary segment.
    let sMin = Infinity, sMax = -Infinity;
    for (const q of outerP2) { if (q[S]! < sMin) sMin = q[S]!; if (q[S]! > sMax) sMax = q[S]!; }
    const sArc = arcOver(S, sMin, sMax, uStar + period / 2);
    const mS = sArc / Math.max(sMax - sMin, 1e-12);
    // Axis caps FIRST: the product cap alone lets a thin revolved ring through as a 4 × 31,000 grid
    // (130k triangles for one sub-mm face — six workers ground to a halt on exactly this). The cell
    // budget is deliberately RESCUE-sized: this mesher only runs when everything else failed, and a
    // coarser-than-target interior on a pathological face beats a skipped face or a 60s CDT.
    let nU = Math.max(4, Math.min(512, Math.ceil(circMid / target)));
    let nV = Math.max(2, Math.min(512, Math.ceil(sArc / target)));
    const capCells = 20000;
    if (nU * nV > capCells) { const f = Math.sqrt(capCells / (nU * nV)); nU = Math.max(4, Math.floor(nU * f)); nV = Math.max(2, Math.floor(nV * f)); }
    const uLo = A[P]! - period;
    const du = period / nU, dv = (sMax - sMin) / nV;
    // Bucket boundary segments by u-column for the keep-out test.
    const segs: [P2, P2][] = [];
    const pushSegs = (poly: P2[], closed: boolean): void => {
      for (let i = 0; i + 1 < poly.length; i++) segs.push([poly[i]!, poly[i + 1]!]);
      if (closed && poly.length > 2) segs.push([poly[poly.length - 1]!, poly[0]!]);
    };
    pushSegs(outerP2, true);
    for (const hp of holeP2) pushSegs(hp, true);
    const bins: number[][] = Array.from({ length: nU + 2 }, () => []);
    for (let si = 0; si < segs.length; si++) {
      const [a, b] = segs[si]!;
      let i0 = Math.floor((Math.min(a[P]!, b[P]!) - uLo) / du) - 1;
      let i1 = Math.floor((Math.max(a[P]!, b[P]!) - uLo) / du) + 1;
      i0 = Math.max(0, i0); i1 = Math.min(nU + 1, i1);
      for (let i = i0; i <= i1; i++) bins[i]!.push(si);
    }
    const cellM = Math.max(du * mP, dv * mS);
    const clearOf = (q: P2, col: number): boolean => {
      for (const si of bins[Math.max(0, Math.min(nU + 1, col))]!) {
        const [a, b] = segs[si]!;
        const ax = (q[P]! - a[P]!) * mP, ay = (q[S]! - a[S]!) * mS;
        const bx = (b[P]! - a[P]!) * mP, by = (b[S]! - a[S]!) * mS;
        const l2 = bx * bx + by * by;
        let t = l2 > 0 ? (ax * bx + ay * by) / l2 : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const dx = ax - t * bx, dy = ay - t * by;
        if (dx * dx + dy * dy < 0.36 * cellM * cellM) return false;
      }
      return true;
    };
    const pts2: P2[] = [...outerP2];
    const pts3: (Vec3 | null)[] = [...outerP3];
    const outerIdx = outerP2.map((_, i) => i);
    const holeIdx: number[][] = [];
    for (let hi = 0; hi < holeP2.length; hi++) {
      const base = pts2.length;
      for (let i = 0; i < holeP2[hi]!.length; i++) { pts2.push(holeP2[hi]![i]!); pts3.push(holeP3[hi]![i]!); }
      holeIdx.push(holeP2[hi]!.map((_, i) => base + i));
    }
    // Per-row u-thinning: rows near a pole (an axis-touching profile end) have near-zero ring
    // circumference — seating all nU points there makes the weld MASS-MERGE them and shed
    // thousands of degenerate triangles (open-edge storm; zipTJunctions then dominated a 2-minute
    // model). Keep only as many points per row as its own circumference supports.
    const rowStride: number[] = [];
    for (let j = 0; j < nV; j++) {
      const s = sMin + (j + 0.5) * dv;
      const circ = arcOver(P, uLo, uLo + period, s);
      const fit = Math.max(1, Math.floor(circ / target));
      rowStride.push(Math.max(1, Math.ceil(nU / fit)));
    }
    // Column scanline: crossings of every boundary segment (outer + holes) against each grid
    // column; odd parity below a row ⟺ inside the face. O(segs + cells) instead of cells × verts.
    const interiorIdx: number[] = [];
    for (let i = 0; i < nU; i++) {
      const u = uLo + (i + 0.5) * du;
      const xs: number[] = [];
      for (const [a, b] of segs) {
        const au = a[P]!, bu = b[P]!;
        if ((au <= u && bu > u) || (bu <= u && au > u)) {
          const t = (u - au) / (bu - au);
          xs.push(a[S]! + t * (b[S]! - a[S]!));
        }
      }
      if (!xs.length) continue;
      xs.sort((x, y) => x - y);
      let k = 0;
      for (let j = 0; j < nV; j++) {
        const s = sMin + (j + 0.5) * dv;
        while (k < xs.length && xs[k]! < s) k++;
        if (k % 2 === 0) continue; // even crossings below → outside
        if (i % rowStride[j]! !== 0) continue; // thinned: this row can't support full u-density
        const q: P2 = [0, 0]; q[P] = u; q[S] = s;
        if (!clearOf(q, i + 1)) continue;
        interiorIdx.push(pts2.length);
        pts2.push(q); pts3.push(null);
      }
    }
    const scaled: P2[] = pts2.map((q) => [q[P]! * mP, q[S]! * mS] as P2);
    const out = { missing: 0 } as { missing: number; rescue?: string };
    const tris = constrainedTriangulate(scaled, [outerIdx, ...holeIdx], interiorIdx, out);
    if (out.missing > 0 || tris.length === 0) {
      if (DBG) console.error(`[tess] fid=${fid} region: CDT missing=${out.missing} tris=${tris.length} at u*=${uStar.toFixed(3)} — retry/bail`);
      if (++cdtFails >= 2) return false;
      continue;
    }
    const start = verts.length;
    for (const [a, b, c] of tris) {
      const A3 = pts3[a] ?? evalPS(pts2[a]![P]!, pts2[a]![S]!);
      const B3 = pts3[b] ?? evalPS(pts2[b]![P]!, pts2[b]![S]!);
      const C3 = pts3[c] ?? evalPS(pts2[c]![P]!, pts2[c]![S]!);
      const mu = (pts2[a]![0]! + pts2[b]![0]! + pts2[c]![0]!) / 3;
      const mv = (pts2[a]![1]! + pts2[b]![1]! + pts2[c]![1]!) / 3;
      emitTri(verts, faceIds, A3, B3, C3, fid, surface, sign, surface.normal(mu, mv));
    }
    if (DBG) console.error(`[tess] fid=${fid} region: rims=${rims.length} holes=${holes.length} tris=${tris.length} nU=${nU} nV=${nV}`);
    if (verts.length > start) return true;
  }
  return false;
}

function tessellateSphere(
  s: Sphere, fid: number, chordTol: number, targetEdge: number, normalDev: number, sign: number, verts: number[], faceIds: number[],
): void {
  const R = Math.max(s.r, 1e-9);
  const dChord = 2 * Math.acos(Math.max(0, Math.min(1, 1 - chordTol / R)));
  const dEdge = targetEdge / R;
  const dTheta = Math.max(1e-4, Math.min(dChord, dEdge, 2 * normalDev));
  const target = R * dTheta; // arc-length target on the sphere
  const nV = Math.max(4, Math.min(2000, Math.ceil(Math.PI / dTheta)));
  const ringAt = (v: number): Vec3[] => {
    const circ = TWO_PI * R * Math.cos(v);
    const nu = Math.max(1, Math.min(4000, Math.round(circ / target)));
    if (nu <= 2) return [s.evaluate(0, v)]; // pole
    const r: Vec3[] = [];
    for (let i = 0; i < nu; i++) r.push(s.evaluate((TWO_PI * i) / nu, v));
    return r;
  };
  let prev = ringAt(-Math.PI / 2);
  for (let j = 1; j <= nV; j++) {
    const ring = ringAt(-Math.PI / 2 + (Math.PI * j) / nV);
    stitchRings(verts, faceIds, prev, ring, fid, s, sign);
    prev = ring;
  }
}

/**
 * Spherical cap closing to a pole: a sphere face whose SOLE boundary is one full-longitude parallel
 * circle (the other side was a VERTEX_LOOP pole, dropped at build time). The rim projects to a
 * horizontal line in (u,v) enclosing no area, so the param grid meshes nothing and the cap opens.
 * Mesh it as latitude rings from the shared rim to the enclosed pole instead. The enclosed pole is
 * the one to the LEFT of the oriented rim in parameter space (the standard trimming convention):
 * traversing the rim eastward (+u) keeps the +v hemisphere (north pole) as material.
 * Returns false WITHOUT emitting if the loop isn't a single full-revolution parallel (a partial
 * spherical patch), so the caller falls back to the param grid.
 */
function tessellateSphereCap(
  s: Sphere, loop: BLoop, sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], chordTol: number, targetEdge: number, normalDev: number, sign: number,
): boolean {
  // Separate the constant-latitude RIM from a SEAM meridian: a hemisphere/cap often carries a seam
  // edge from the pole to the rim, traversed twice (down one side, back the other). Those doubled
  // edges are the degenerate seam — exclude them; the remaining once-used edges form the true rim.
  const count = new Map<number, number>();
  for (const oe of loop.edges) count.set(oe.edgeId, (count.get(oe.edgeId) ?? 0) + 1);
  const rimEdges = loop.edges.filter((oe) => (count.get(oe.edgeId) ?? 0) === 1);
  const rim: Vec3[] = [];
  for (const oe of rimEdges) {
    const base = sampled.get(oe.edgeId); if (!base) return false;
    const poly = oe.orient ? base : base.slice().reverse();
    for (let i = 0; i < poly.length - 1; i++) rim.push(poly[i]!);
  }
  if (rim.length < 4) return false;
  const uv = rim.map((p) => s.project(p));
  let vmin = Infinity, vmax = -Infinity, umin = Infinity, umax = -Infinity;
  for (const [u, v] of uv) { if (v < vmin) vmin = v; if (v > vmax) vmax = v; if (u < umin) umin = u; if (u > umax) umax = u; }
  // Must be a near-constant-latitude circle spanning the whole longitude (a true parallel rim).
  if (vmax - vmin > 0.05 || umax - umin < 0.9 * TWO_PI) return false;
  const vRim = (vmin + vmax) / 2;
  // Net signed longitude travel of the oriented rim -> which pole is the enclosed (left-hand) side.
  // "Left of the rim" is defined against the FACE normal, not the surface normal: on a sameSense
  // face, eastward (+u) travel keeps the +v (north) hemisphere; a reversed face (sign<0) has its
  // material on the other side, so the enclosed pole flips with it (a hemispherical DIMPLE is a
  // sameSense=false sphere face whose eastward rim encloses the south hemisphere — picking north
  // meshes the complement and turns the pocket into a bump).
  let du = 0;
  for (let i = 0; i < uv.length; i++) { let d = uv[(i + 1) % uv.length]![0] - uv[i]![0]; while (d > Math.PI) d -= TWO_PI; while (d < -Math.PI) d += TWO_PI; du += d; }
  const vPole = (du >= 0) === (sign > 0) ? Math.PI / 2 : -Math.PI / 2;

  const R = Math.max(s.r, 1e-9);
  const dChord = 2 * Math.acos(Math.max(0, Math.min(1, 1 - chordTol / R)));
  const dTheta = Math.max(1e-4, Math.min(dChord, targetEdge / R, 2 * normalDev));
  const target = R * dTheta;
  const span = Math.abs(vPole - vRim);
  const nV = Math.max(1, Math.min(2000, Math.ceil(span / dTheta)));
  // Build the rim ring from the SHARED edge samples (watertight with the neighbour), then march
  // latitude rings to the pole, each sized to its own circumference so the cap tapers to a point.
  // stitchRings pairs points by angular FRACTION along each ring, so the generated rings must start
  // at the rim's own start longitude and run the rim's own way: stitching a westward (du<0) or
  // off-phase rim against eastward-from-u=0 rings makes a full-turn twisted band of bowtie triangles.
  const uStart = uv[0]![0];
  const uDir = du >= 0 ? 1 : -1;
  let prev = rim.slice();
  for (let j = 1; j <= nV; j++) {
    const v = vRim + ((vPole - vRim) * j) / nV;
    if (j === nV) { stitchRings(verts, faceIds, prev, [s.evaluate(0, vPole)], fid, s, sign); break; }
    const circ = TWO_PI * R * Math.cos(v);
    const nu = Math.max(3, Math.min(4000, Math.round(circ / target)));
    const ring: Vec3[] = [];
    for (let i = 0; i < nu; i++) ring.push(s.evaluate(uStart + uDir * (TWO_PI * i) / nu, v));
    stitchRings(verts, faceIds, prev, ring, fid, s, sign);
    prev = ring;
  }
  return true;
}

/**
 * Untrimmed (closed) B-spline patch forming a whole body — e.g. a surface of revolution with poles
 * and a periodic seam, which has no usable trimming loop. Tessellated as a structured (u,v) grid;
 * the seam columns coincide (weld) and pole rows collapse to fans (emitTri drops the degenerate half).
 */
function tessellateBSplineFull(
  s: BSplineSurface, fid: number, chordTol: number, targetEdge: number, normalDev: number,
  sign: number, verts: number[], faceIds: number[],
): boolean {
  const { u0, u1, v0, v1 } = s;
  const arcLen = (along: "u" | "v"): number => {
    let len = 0; const M = 64;
    let prev = along === "u" ? s.evaluate(u0, (v0 + v1) / 2) : s.evaluate((u0 + u1) / 2, v0);
    for (let i = 1; i <= M; i++) {
      const t = i / M;
      const p = along === "u" ? s.evaluate(u0 + (u1 - u0) * t, (v0 + v1) / 2) : s.evaluate((u0 + u1) / 2, v0 + (v1 - v0) * t);
      len += Math.hypot(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]); prev = p;
    }
    return len;
  };
  const Rc = s.rc; // global min — the full-patch grid is sized uniformly, so no local lookup
  const target = Number.isFinite(Rc) // floor only the chord term (see faceTarget)
    ? Math.min(targetEdge, Math.max(targetEdge / 40, Math.sqrt(8 * Rc * chordTol)), Rc * normalDev) : targetEdge;
  const nU = Math.max(2, Math.min(2000, Math.ceil(arcLen("u") / target)));
  // Build each u-ring at a resolution matching ITS OWN circumference, so rings shrinking toward a
  // pole don't keep a high v-count (which makes pole-fan slivers). A vanishing ring becomes a point.
  const rings: Vec3[][] = [];
  for (let i = 0; i <= nU; i++) {
    const u = u0 + ((u1 - u0) * i) / nU;
    let circ = 0; let prev = s.evaluate(u, v0); const SAMP = 48;
    for (let j = 1; j <= SAMP; j++) { const p = s.evaluate(u, v0 + ((v1 - v0) * j) / SAMP); circ += Math.hypot(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]); prev = p; }
    const nv = Math.max(1, Math.min(4000, Math.round(circ / target)));
    const ring: Vec3[] = [];
    if (nv <= 2) ring.push(s.evaluate(u, (v0 + v1) / 2)); // collapsed ring = pole point
    else for (let j = 0; j < nv; j++) ring.push(s.evaluate(u, v0 + ((v1 - v0) * j) / nv)); // closed (wraps via modulo)
    rings.push(ring);
  }
  // Stitch consecutive rings (handles unequal counts and pole fans); emitTri fixes winding.
  for (let i = 0; i < nU; i++) stitchRings(verts, faceIds, rings[i]!, rings[i + 1]!, fid, s, sign);
  return true;
}

/**
 * Read AP242 *tessellated* geometry (ISO 10303-42 tessellated_item subtree) directly into triangles.
 * Some MBE/AP242 exports ship the part as a faceted mesh instead of (or as well as) a precise B-rep —
 * a TESSELLATED_SOLID / TESSELLATED_SHELL holding TRIANGULATED_FACEs that index a shared
 * COORDINATES_LIST. There are no analytic surfaces, so the mesh is transcribed as-is (welded by
 * position downstream, oriented by orientConsistent). Only consulted when the file has no B-rep
 * solids (a dual-representation file uses its precise B-rep). Returns null if no tessellated bodies.
 *
 * Face layout (positional, tolerant of an optional geometric_link ref):
 *   (COMPLEX_)TRIANGULATED_FACE(name, coords#, pnmax, normals, [link#], pnindex, [triangles,] strips, fans)
 * pnindex maps a face-local 1-based index to a 1-based COORDINATES_LIST point; triangle_strips and
 * triangle_fans give connectivity over those local indices.
 */
function readTessellated(brep: BrepModel): { verts: number[]; faceIds: number[]; solidIds: number[]; faces: number } | null {
  const t = brep.table, s = brep.scale;
  // A TESSELLATED_SOLID is the complete closed body; only fall back to loose TESSELLATED_SHELLs when
  // there's no solid (mixing them welds supplementary feature shells into the body -> non-manifold).
  const solids = [...t.byType("TESSELLATED_SOLID")];
  const containers = solids.length > 0 ? solids : [...t.byType("TESSELLATED_SHELL")];
  if (containers.length === 0) return null;
  const verts: number[] = [], faceIds: number[] = [], solidIds: number[] = [];
  let faces = 0;
  const coordsCache = new Map<number, Vec3[]>();
  const getCoords = (id: number): Vec3[] => {
    let c = coordsCache.get(id);
    if (!c) { c = list(t.record(id).params[2]!).map((tup) => { const a = numList(tup); return [a[0]! * s, a[1]! * s, a[2]! * s] as Vec3; }); coordsCache.set(id, c); }
    return c;
  };
  for (const [cid, c] of containers) {
    for (const fref of refList(c.params[1]!)) {
      const rec = t.record(fref);
      if (rec.type !== "TRIANGULATED_FACE" && rec.type !== "COMPLEX_TRIANGULATED_FACE") continue;
      const pts = getCoords(ref(rec.params[1]!));
      const pnmax = num(rec.params[2]!);
      const params = rec.params;
      // pnindex is the flat integer list of length pnmax (normals are lists-of-tuples; link is a ref).
      let pidx = -1;
      for (let i = 3; i < params.length; i++) { const p = params[i]!; if (p.k === "list" && p.v.length === pnmax && (p.v.length === 0 || p.v[0]!.k === "num")) { pidx = i; break; } }
      if (pidx < 0) continue;
      const pnindex = numList(params[pidx]!);
      const vtx = (local: number): Vec3 => pts[pnindex[local - 1]! - 1]!;
      const pushTri = (a: number, b: number, cc: number): void => {
        if (a === b || b === cc || a === cc) return; // strip-restart degenerate (repeated index) — skip
        const A = vtx(a), B = vtx(b), C = vtx(cc);
        verts.push(A[0], A[1], A[2], B[0], B[1], B[2], C[0], C[1], C[2]); faceIds.push(fref); solidIds.push(cid);
      };
      const addList = (p: Param, kind: "tri" | "strip" | "fan"): void => {
        for (const sub of list(p)) {
          const idx = numList(sub);
          if (kind === "tri") for (let i = 0; i + 2 < idx.length; i += 3) pushTri(idx[i]!, idx[i + 1]!, idx[i + 2]!);
          else if (kind === "strip") for (let i = 0; i + 2 < idx.length; i++) (i % 2 === 0) ? pushTri(idx[i]!, idx[i + 1]!, idx[i + 2]!) : pushTri(idx[i + 1]!, idx[i]!, idx[i + 2]!);
          else for (let i = 1; i + 1 < idx.length; i++) pushTri(idx[0]!, idx[i]!, idx[i + 1]!);
        }
      };
      const rest = params.slice(pidx + 1);
      if (rec.type === "TRIANGULATED_FACE") { if (rest[0]) addList(rest[0], "tri"); }
      else if (rest.length >= 3) { addList(rest[0]!, "tri"); addList(rest[1]!, "strip"); addList(rest[2]!, "fan"); }
      else { if (rest[0]) addList(rest[0], "strip"); if (rest[1]) addList(rest[1], "fan"); }
      faces++;
    }
  }
  return { verts, faceIds, solidIds, faces };
}

export function tessellate(brep: BrepModel, opts: TessOptions = {}): MeshResult {
  const chordTol = opts.chordTol ?? 0.01;
  const targetEdge = opts.targetEdge ?? 1.0;
  const normalDev = opts.normalDev ?? (15 * Math.PI / 180);
  const trace = opts.trace;
  /** Tag a dispatch outcome: report which mesher took the face (on success) to the trace hook
   * and the MESHSTEP_DEBUG log, and pass the result through unchanged. */
  const mark = (fid: number, n: string, r: boolean): boolean => {
    if (r) { trace?.(fid, n); if (DBG) console.error(`[dispatch] fid=${fid} -> ${n}`); }
    return r;
  };
  const skipped: Record<string, number> = {};
  let facesTotal = 0;
  let facesTessellated = 0;
  // Collect diagnostics warnings for this run (module-level sink: tessellation is synchronous,
  // and the deep meshing code reports without threading a sink through every signature).
  beginWarnings();
  for (const fid of brep.droppedFaces) warn("face-dropped", fid, "malformed face record dropped while reading the B-rep — geometry missing");

  // Cache each face's surface (used to dispatch tessellation).
  const faceSurf = new Map<number, Surface | null>();
  for (const solid of brep.solids) for (const face of solid.faces) {
    faceSurf.set(face.faceId, makeSurface(brep.table, face.surfaceId, solid.scale ?? brep.scale, brep.units.radPerAngle));
  }

  // Progress accounting, weighted by measured cost: face meshing dominates wall time, edge
  // sampling is cheap (edges outnumber faces ~3:1, so an unweighted count leaps to ~80% during
  // sampling and then crawls), and each solid's stitch passes (weld/zip/T-junctions/hole fill)
  // cost roughly a quarter of its faces' meshing. The total is known before any work starts, so
  // the consumer can draw a determinate bar for single parts and assemblies alike.
  const W_FACE = 16, W_STITCH = 4;
  let pDone = 0;
  let pTotal = brep.edges.size;
  for (const solid of brep.solids) pTotal += (W_FACE + W_STITCH) * solid.faces.length;
  const tick = (units: number): void => { pDone += units; opts.onProgress?.(pDone, pTotal); };
  // Sample each edge to the FINEST interior target of its adjacent faces — not just its own curve
  // curvature. A curved face's straight seam / side edges (lines carry no curvature, so they'd be
  // sampled at targetEdge) otherwise stay far coarser than the fine interior and sliver the seam.
  // Still one shared sampling per edge, so seams remain watertight; any residual sliver lands on a
  // flat neighbour (where it's invisible) rather than on the curved face.
  // (A DIRECTIONAL variant — relaxing edges that run along a face's flat direction — was measured:
  // it halves the triangle count again but coarsens the pinched-B-spline rims through their
  // analytic neighbours and opens more of their seams; with the 1-ring flip pass the visual result
  // is identical, so the conservative isotropic rule stays.)
  const edgeMaxLen = new Map<number, number>();
  for (const solid of brep.solids) for (const face of solid.faces) {
    const surface = faceSurf.get(face.faceId);
    const t = surface ? faceTarget(surface, targetEdge, chordTol, normalDev, 0, 0, true) : targetEdge;
    for (const lp of face.loops) for (const oe of lp.edges) {
      const cur = edgeMaxLen.get(oe.edgeId);
      if (cur === undefined || t < cur) edgeMaxLen.set(oe.edgeId, t);
    }
  }
  const sampled = new Map<number, Vec3[]>();
  for (const [id, e] of brep.edges) {
    const te = edgeMaxLen.get(id) ?? targetEdge;
    sampled.set(id, sampleEdgePolyline(brep.table, e.curveId, e.v0, e.v1, e.sameSense, e.scale ?? brep.scale, chordTol, te, brep.units.radPerAngle, normalDev));
    tick(1);
  }
  // Micro-face boundary sanity: a face far smaller than the tolerance budget (a 1.3mm thread
  // run-out at 0.7mm chord tolerance) samples its edges with so few points that the boundary
  // polygon SELF-INTERSECTS in (u,v) — chords of adjacent edges swing across each other. The CDT
  // then classifies most of the polygon away (parity flips at every crossing) and the face's whole
  // rim opens against its neighbours (wallganizer's screw: 16-pt run-out plane -> 4 triangles ->
  // the cone rim it borders is fully open, ×213 instances). For such a face, TRIAL-sample its
  // edges at increasingly fine tolerances and commit the first level whose polygon is simple.
  // Outcome-verified on purpose: a polygon that stays self-intersecting at every level crosses
  // from genuine trim overlap (tangent letter engravings), where densification cannot help — it
  // only perturbs every neighbour sharing those edges (Ontos +523 open edges under a blind
  // version of this pass). This runs as a PRE-PASS before any face is meshed, so a densified edge
  // remains the SINGLE shared sampling and seams stay watertight; per edge the FINEST committed
  // level wins. Only small boundaries are checked (the crossing test is O(n²)-ish, and large
  // self-intersecting polygons are always the genuine-overlap class).
  {
    const level = new Map<number, number>(); // edgeId -> finest committed refinement factor
    const resample = (id: number, f: number): Vec3[] => {
      const e = brep.edges.get(id)!;
      const te = Math.max((edgeMaxLen.get(id) ?? targetEdge) / f, chordTol);
      return sampleEdgePolyline(brep.table, e.curveId, e.v0, e.v1, e.sameSense, e.scale ?? brep.scale, chordTol / f, te, brep.units.radPerAngle, normalDev);
    };
    for (const solid of brep.solids) for (const face of solid.faces) {
      const surface = faceSurf.get(face.faceId);
      if (!surface) continue;
      let nPts = 0;
      let lo0 = Infinity, lo1 = Infinity, lo2 = Infinity, hi0 = -Infinity, hi1 = -Infinity, hi2 = -Infinity;
      for (const lp of face.loops) for (const oe of lp.edges) {
        const s = sampled.get(oe.edgeId); if (!s) continue;
        nPts += s.length;
        for (const p of s) {
          if (p[0] < lo0) lo0 = p[0]; if (p[0] > hi0) hi0 = p[0];
          if (p[1] < lo1) lo1 = p[1]; if (p[1] > hi1) hi1 = p[1];
          if (p[2] < lo2) lo2 = p[2]; if (p[2] > hi2) hi2 = p[2];
        }
      }
      if (nPts > 512) continue;
      // Two admission paths. SMALL faces (a few chord-lengths across, the original criterion):
      // any crossing pattern qualifies — coarse chords can only swing across each other when the
      // whole face is comparable to the tolerance budget. LARGER faces qualify only when EVERY
      // crossing is LOCAL (near-adjacent segments of the loop): that is one tightly-curved edge
      // run zigzagging under coarse sampling (skirt-corner fillets, z_endstop's rounded strip
      // ends), which densification provably cures. Distant-pair crossings on a large face are
      // the pinch/genuine-overlap class where densifying perturbs the neighbours' pinch handling
      // for nothing (OpenVessel's 6mm counterbore rims at 0.002mm tolerance went watertight ->
      // 12 open under a blind version; Ontos' tangent letter engravings never become simple).
      const small = nPts <= 128 && Math.hypot(hi0 - lo0, hi1 - lo1, hi2 - lo2) <= 16 * chordTol;
      let nonLocal = false;
      const tangledLoops = face.loops.filter((lp) => {
        const p2 = loopParam(surface, lp, sampled).p2;
        if (p2.length < 4) return false;
        const pairs: [number, number][] = [];
        if (countSelfIntersections(p2, 64, pairs) === 0) return false;
        if (!small) {
          const n = p2.length;
          for (const [i, j] of pairs) {
            if (Math.min((j - i + n) % n, (i - j + n) % n) > 8) { nonLocal = true; break; }
          }
        }
        return true;
      });
      if (tangledLoops.length === 0 || nonLocal) continue;
      for (const f of [4, 16, 64]) {
        const probe = new Map(sampled);
        const ids = new Set<number>();
        for (const lp of tangledLoops) for (const oe of lp.edges) ids.add(oe.edgeId);
        for (const id of ids) probe.set(id, resample(id, f));
        const stillBad = tangledLoops.some((lp) => {
          const p2 = loopParam(surface, lp, probe).p2;
          return p2.length >= 4 && countSelfIntersections(p2, 1) > 0;
        });
        if (!stillBad) {
          if (DBG) console.error(`[tess] micro-face densify: fid=${face.faceId} simple at tol/${f} (${ids.size} edges)`);
          for (const id of ids) level.set(id, Math.max(level.get(id) ?? 0, f));
          break;
        }
        if (f === 64) {
          warn("boundary-self-intersects", face.faceId, "trim loops still self-intersect at 64× sampling density — genuine overlap in the CAD file, meshed as-is");
          if (DBG) console.error(`[tess] micro-face densify: fid=${face.faceId} still self-intersecting at tol/64 — left as-is (genuine overlap)`);
        }
      }
    }
    for (const [id, f] of level) sampled.set(id, resample(id, f));
  }

  // Weld each body independently so touching bodies don't merge into non-manifold edges.
  const positions: number[] = [];
  const indices: number[] = [];
  const faceOfTri: number[] = [];
  const solidOfTri: number[] = [];
  let voff = 0;

  for (const solid of brep.solids) {
    const verts: number[] = [];
    const faceIds: number[] = [];
    for (const face of solid.faces) {
      facesTotal++;
      tick(W_FACE);
      const surface = faceSurf.get(face.faceId) ?? null;
      if (!surface) {
        bump(skipped, face.surfaceKind);
        warn("face-unsupported-surface", face.faceId, `surface kind ${face.surfaceKind} not supported — face skipped, geometry missing`);
        continue;
      }
      const sign = face.sameSense ? 1 : -1;

      let ok = false;
      const outer = face.loops.find((l) => l.outer) ?? face.loops[0];
      if (outer && face.loops.length === 1 && solid.faces.length > 1
        && (mark(face.faceId, "thinFace", tessellateThinFace(surface, outer, sampled, face.faceId, verts, faceIds, sign, 0.005))
          || mark(face.faceId, "thinStrip", tessellateThinStrip(surface, outer, sampled, face.faceId, verts, faceIds, sign, 0.03)))) {
        ok = true; // degenerate sub-resolution sliver/crack ribbon-stitched (returns false if not thin)
      } else if (face.loops.length === 2 && solid.faces.length > 1
        // 0.015mm absolute floor; on large parts the knife-edge class scales with the part (a 53µm
        // digon annulus on a 15m assembly), so admit up to half a chord tolerance — a ribbon across
        // that width deviates less than the sampling itself.
        && mark(face.faceId, "thinRing", tessellateThinRing(surface, face.loops, sampled, face.faceId, verts, faceIds, sign, Math.max(0.015, 0.5 * chordTol)))) {
        ok = true; // sub-tolerance annular sliver (two near-coincident rims) ribbon-stitched
      } else if (face.loops.length === 0 && !(isSphere(surface) && solid.faces.length === 1)) {
        // Bare untrimmed face — no loops at all, so no boundary-driven mesher can run. A full
        // sphere inside a multi-face solid (ball seat), a bare torus (chain links / springs), a
        // closed-profile surface of revolution. Sole-face full spheres keep their branch below.
        ok = (isSphere(surface) && mark(face.faceId, "sphereFull", (tessellateSphere(surface, face.faceId, chordTol, targetEdge, normalDev, sign, verts, faceIds), true)))
          || (isBSpline(surface) && (surface.closedU || surface.closedV)
            && mark(face.faceId, "bsplineFull", tessellateBSplineFull(surface, face.faceId, chordTol, targetEdge, normalDev, sign, verts, faceIds)))
          || mark(face.faceId, "fullPeriodic", tessellateFullPeriodic(surface, face.faceId, chordTol, targetEdge, normalDev, sign, verts, faceIds, brep, sampled));
      } else if (isSphere(surface)) {
        // A full sphere is its solid's only face (degenerate seam loop); trimmed spheres
        // (e.g. roundedCube corners) are one of many faces -> param grid.
        if (solid.faces.length === 1) {
          tessellateSphere(surface, face.faceId, chordTol, targetEdge, normalDev, sign, verts, faceIds);
          ok = mark(face.faceId, "sphereFull", true);
        } else if (outer) {
          // A POLE DART (the same meridian edge out-and-back to a pole) proves the pole is
          // INTERIOR to the face: strip it and mesh the once-winding rim with the REGION mesher
          // FIRST — the grid would otherwise CDT the dart-stripped loop's residual signed area,
          // which is the complement lune, not the face (Midea umbrella-valve dome).
          const stripPoleDarts = stripSpherePoleDarts(surface, face.loops, sampled);
          const stripped = stripPoleDarts !== null;
          const loops = stripPoleDarts ?? face.loops;
          const louter = stripped ? (loops.find((l) => l.outer) ?? loops[0]!) : outer;
          // A cap closing to a pole (sole full-longitude parallel rim) fans to the pole; any other
          // spherical patch returns false from the cap mesher and uses the param grid — with the
          // sphere REPARAMETRISED so pole and seam sit away from the patch (a corner blend often
          // runs straight through the default pole, which degenerates the projected loop).
          // The cap is ONLY offered a single-loop face: handed just the outer rim of a HOLED face
          // (a screw head's dome pierced by its hex socket, Ontos) it happily fans over the hole —
          // or fans from the hole's rim and leaves the real shared rim fully open. A multi-loop
          // sphere zone is a revolution band like any other: band -> unroll -> param grid.
          const single = loops.length === 1;
          ok = (single && mark(face.faceId, "sphereCap", tessellateSphereCap(surface, louter, sampled, face.faceId, verts, faceIds, chordTol, targetEdge, normalDev, sign)))
            || (stripped && mark(face.faceId, "region", tessellatePeriodicRegion(surface, loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
            || (!single && mark(face.faceId, "band", tessellateRevolutionBand(surface, loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
            || (!single && mark(face.faceId, "unroll", tessellatePeriodicUnroll(surface, loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
            || mark(face.faceId, "grid", tessellateParamGrid(reorientSphere(surface, loops, sampled), loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign))
            || (!stripped && mark(face.faceId, "region", tessellatePeriodicRegion(surface, loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
            || (single && mark(face.faceId, "sliver", tessellateSliverLoop(surface, louter, sampled, face.faceId, verts, faceIds, sign, chordTol)));
        }
      } else if (surface.kind === "CONICAL_SURFACE" && outer
        && (mark(face.faceId, "cone", tessellateCone(surface, outer, sampled, brep, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign, face.loops.length))
          || mark(face.faceId, "coneSlice", tessellateConeSlice(surface, outer, sampled, brep, face.faceId, verts, faceIds, sign))
          || (face.loops.length === 1 && mark(face.faceId, "coneCap", tessellateConeCap(surface, outer, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign))))) {
        ok = true; // genuine apex cone, apex wedge slice, or arbitrary once-winding rim cap; frustums/trimmed cones use the param grid
      } else if (isBSpline(surface)) {
        // A standalone closed B-spline body (its solid's only face) has no usable trimming loop ->
        // full-patch grid. A patch that is one of many faces must use the param grid so its boundary
        // uses the SHARED edge samples (independent grids would crack against their neighbours).
        // A CLOSED-direction band bounded by bare full-period rim loops (StingStopp_4000 dome body:
        // two iso-u rim circles enclosing zero param-space area, so the param grid has no outer
        // loop) unrolls exactly like an analytic revolution surface — tried after the param grid so
        // ordinary trimmed patches are untouched, and before the rail ribbon, whose straight loft
        // cut that dome's bulge 4.5mm deep.
        const per = !!(surface.periodicU || surface.periodicV);
        ok = (solid.faces.length === 1 && (surface.closedU || surface.closedV))
          ? mark(face.faceId, "bsplineFull", tessellateBSplineFull(surface, face.faceId, chordTol, targetEdge, normalDev, sign, verts, faceIds))
          : (!!outer && (mark(face.faceId, "grid", tessellateParamGrid(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign))
            || (per && mark(face.faceId, "band", tessellateRevolutionBand(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
            || (per && mark(face.faceId, "unroll", tessellatePeriodicUnroll(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
            || (per && face.loops.length === 1 && mark(face.faceId, "capDome", tessellateCapDome(surface, outer, sampled, face.faceId, verts, faceIds, chordTol, targetEdge, normalDev, sign)))
            || mark(face.faceId, "ribbon", tessellateRibbon(surface, face.loops, sampled, face.faceId, verts, faceIds, sign))
            || (per && mark(face.faceId, "region", tessellatePeriodicRegion(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
            || (face.loops.length === 1 && mark(face.faceId, "sliver", tessellateSliverLoop(surface, outer, sampled, face.faceId, verts, faceIds, sign, chordTol)))));
      } else if (outer) {
        // Cylinders, cone frustums, tori, etc. Three meshers, tried in order:
        //  1. band: rims are bare full-period circles (no seam edges, e.g. Onshape) with NO other
        //     loops -> ribbon-stitch the rims directly. Bails on anything else.
        //  2. unroll: bare full-period rims PLUS window holes -> seam-split into a rectangular (u,v)
        //     domain and CDT with the windows as holes. Bails unless there are exactly two rims.
        //  3. param grid: everything with a proper seam-bounded outer loop (the common case).
        const periodic = surface.periodicU || surface.periodicV;
        ok = (periodic && mark(face.faceId, "band", tessellateRevolutionBand(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
          || (periodic && mark(face.faceId, "unroll", tessellatePeriodicUnroll(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
          || mark(face.faceId, "grid", tessellateParamGrid(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign))
          || (periodic && face.loops.length === 1 && mark(face.faceId, "capDome", tessellateCapDome(surface, outer, sampled, face.faceId, verts, faceIds, chordTol, targetEdge, normalDev, sign)))
          || mark(face.faceId, "ribbon", tessellateRibbon(surface, face.loops, sampled, face.faceId, verts, faceIds, sign))
          || (periodic && mark(face.faceId, "region", tessellatePeriodicRegion(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)))
          || (face.loops.length === 1 && mark(face.faceId, "sliver", tessellateSliverLoop(surface, outer, sampled, face.faceId, verts, faceIds, sign, chordTol)));
      }
      if (ok) facesTessellated++;
      else {
        trace?.(face.faceId, "untriangulated");
        if (DBG) console.error(`[tess] UNTRIANGULATED fid=${face.faceId} kind=${face.surfaceKind} loops=${face.loops.map((l) => l.edges.length).join("/")}`);
        bump(skipped, "untriangulated");
        warn("face-untriangulated", face.faceId, `every mesher failed on this ${face.surfaceKind} face — face skipped, geometry missing`);
      }
    }

    // One stitch-progress quarter (of the solid's W_STITCH share) per pass, so a big single
    // solid's bar keeps moving through the multi-second weld/zip/fill tail instead of stalling.
    const stitchQ = solid.faces.length;
    const { mesh } = weld(verts);
    tick(stitchQ);
    const z = zipSlivers(mesh, 0.05);
    tick(stitchQ);
    const keptFaceIds: number[] = [];
    for (let t = 0; t < faceIds.length; t++) if (z.keep[t]) keptFaceIds.push(faceIds[t]!);
    // T-junction crack repair — skipped for open-shell surface bodies, whose boundary is open by
    // design (splitting their rims against each other would only churn).
    const tj = solid.open || (DBG && process.env.MESHSTEP_NOZIP)
      ? { indices: z.indices, faceOf: keptFaceIds }
      : zipTJunctions(z.positions, z.indices, keptFaceIds, 0.02);
    tick(stitchQ);
    const fill = fillMicroHoles(z.positions, tj.indices, tj.faceOf, 0.05,
      solid.open ? undefined : { edge: targetEdge, dev: 0.75 * chordTol });
    for (const x of z.positions) positions.push(x);
    for (const ix of tj.indices) indices.push(ix + voff);
    for (const ix of fill.indices) indices.push(ix + voff);
    voff += z.positions.length / 3;
    for (const f of tj.faceOf) { faceOfTri.push(f); solidOfTri.push(solid.id); }
    for (const f of fill.faceOf) { faceOfTri.push(f); solidOfTri.push(solid.id); }
    tick(stitchQ);
  }

  // Sew jointly-closing open shells across solids (see sewSolids). Only OPEN boundary rings can
  // pair, so closed bodies in a genuine assembly are never bridged.
  if (brep.solids.length > 1) {
    const openIds = new Set(brep.solids.filter((s) => s.open).map((s) => s.id));
    const unsewn = new Set<number>();
    const sewnSolids = new Set<number>();
    const sewn = sewSolids(positions, indices, faceOfTri, solidOfTri, Math.max(0.02, 2 * chordTol), openIds, unsewn, sewnSolids);
    if (sewn > 0) warn("heuristic-fill", -1, `${sewn} coincident open rim pair(s) sewn across solids`);
    // Sheet bodies masquerading as solids: a shell whose OWN B-rep has boundary edges (an edge
    // referenced by exactly one face loop) cannot close alone — it either closes JOINTLY (the
    // sewing above) or it is a surface body in the SBSM sense no matter what CLOSED_SHELL claims
    // (a bare tube resting against another solid's FACE has no mate ring to sew — ABC 00000452's
    // pin bores). Reclassify the still-open ones as open bodies so diagnostics treat them like
    // every other sheet. B-rep structural test only — tessellation failures cannot trigger it.
    for (const solid of brep.solids) {
      if (solid.open || !unsewn.has(solid.id)) continue;
      // A solid that DID sew somewhere is load-bearing: its ribbon triangles carry ITS solid id,
      // so excluding it would orphan the mate's rim in the watertight count (run9 00009564).
      // Only solids with NO successful sew anywhere may be reclassified.
      if (sewnSolids.has(solid.id)) continue;
      const useCount = new Map<number, number>();
      for (const f of solid.faces) for (const lp of f.loops) for (const oe of lp.edges) {
        useCount.set(oe.edgeId, (useCount.get(oe.edgeId) ?? 0) + 1);
      }
      let boundary = 0;
      for (const n of useCount.values()) if (n === 1) boundary++;
      if (boundary > 0) {
        solid.open = true;
        warn("open-shell", -1, `solid ${solid.id} claims CLOSED_SHELL but its B-rep has ${boundary} boundary edge(s) and no mate to sew — classified as a surface body`);
        if (DBG) console.error(`[tess] sewSolids: solid ${solid.id} reclassified open (${boundary} B-rep boundary edges, unsewn rings)`);
      }
    }
  }

  // No precise B-rep? Fall back to AP242 tessellated geometry (a pre-faceted body in the file).
  if (brep.solids.length === 0) {
    const tg = readTessellated(brep);
    if (tg && tg.verts.length > 0) {
      const { mesh } = weld(tg.verts);
      for (const x of mesh.positions) positions.push(x);
      for (const ix of mesh.indices) indices.push(ix + voff);
      voff += mesh.positions.length / 3;
      for (let k = 0; k < tg.faceIds.length; k++) { faceOfTri.push(tg.faceIds[k]!); solidOfTri.push(tg.solidIds[k]!); }
      facesTotal += tg.faces; facesTessellated += tg.faces;
    }
  }

  return {
    mesh: { positions: Float64Array.from(positions), indices: Uint32Array.from(indices) },
    faceOfTri: Uint32Array.from(faceOfTri),
    solidOfTri: Uint32Array.from(solidOfTri),
    openSolids: brep.solids.filter((s) => s.open).map((s) => s.id),
    stats: { solids: brep.solids.length, facesTotal, facesTessellated, skipped },
    warnings: takeWarnings(),
    ...(opts.collectEdgePolylines ? { edgePolylines: sampled } : {}),
  };
}

/**
 * Close sub-tolerance "sliver" gaps: a degenerate CAD face thinner than the weld tolerance (CAD
 * kernels routinely leave faces a fraction of a micron wide) tessellates to nothing or to rejected
 * zero-area triangles, leaving its two long rails as unconnected open edges. In a closed solid every
 * open edge is such a defect, so each open-edge vertex is welded to its nearest open-edge vertex
 * within `tol` — UNLESS the two are already joined by a triangle edge. That single exclusion is the
 * safety: the partner across a sliver gap is never edge-connected, while along-rail neighbours always
 * are, so real geometry (and genuine slots wider than tol) is untouched. The two rails are separate
 * EDGE_CURVEs sampled at non-matching positions, so we match vertex-to-nearest-vertex rather than
 * edge-to-edge. Triangles that collapse to a repeated vertex are dropped.
 */
function zipSlivers(mesh: IndexedMesh, tol: number): { positions: Float64Array; indices: Uint32Array; keep: boolean[] } {
  const I = mesh.indices, P = mesh.positions, nv = P.length / 3, nt = I.length / 3;
  const keepAll = (): boolean[] => { const k = new Array(nt); for (let t = 0; t < nt; t++) k[t] = true; return k; };
  const KEY = 2 ** 26;
  const ek = (a: number, b: number): number => (a < b ? a * KEY + b : b * KEY + a);
  const use = new Map<number, number>();
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) { const k = ek(I[i + e]!, I[i + (e + 1) % 3]!); use.set(k, (use.get(k) ?? 0) + 1); }
  const openV = new Set<number>();
  // Per-vertex weld cap = half its shortest incident open-edge segment: on a part with
  // sub-tolerance features (micro-fillets, fine threads) the fixed tol exceeds real feature
  // spacing and would weld distinct geometry into non-manifold garbage. A sliver gap is always
  // far narrower than its rails' own segment length, so this cap never blocks a genuine zip.
  const vCap = new Map<number, number>();
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
    const a = I[i + e]!, b = I[i + (e + 1) % 3]!;
    if (use.get(ek(a, b)) === 1) {
      openV.add(a); openV.add(b);
      const L = Math.hypot(P[a * 3]! - P[b * 3]!, P[a * 3 + 1]! - P[b * 3 + 1]!, P[a * 3 + 2]! - P[b * 3 + 2]!);
      vCap.set(a, Math.min(vCap.get(a) ?? Infinity, L)); vCap.set(b, Math.min(vCap.get(b) ?? Infinity, L));
    }
  }
  if (!openV.size) return { positions: P, indices: I, keep: keepAll() };

  // Union-find; representative = lowest index (keeps a stable surviving vertex).
  const parent = new Int32Array(nv); for (let i = 0; i < nv; i++) parent[i] = i;
  const find = (x: number): number => { while (parent[x] !== x) { parent[x] = parent[parent[x]!]!; x = parent[x]!; } return x; };
  const uni = (a: number, b: number): void => { const ra = find(a), rb = find(b); if (ra !== rb) parent[Math.max(ra, rb)] = Math.min(ra, rb); };

  const cell = Math.max(tol, 1e-9);
  const px = (i: number): number => P[i * 3]!, py = (i: number): number => P[i * 3 + 1]!, pz = (i: number): number => P[i * 3 + 2]!;
  const ckey = (x: number, y: number, z: number): string => `${Math.round(x / cell)},${Math.round(y / cell)},${Math.round(z / cell)}`;
  const hash = new Map<string, number[]>();
  for (const v of openV) { const k = ckey(px(v), py(v), pz(v)); (hash.get(k) ?? hash.set(k, []).get(k)!).push(v); }
  // Weld each open vertex to its nearest open vertex within tol that it does NOT already share a
  // triangle edge with (across-gap partners are unconnected; along-rail neighbours are connected).
  for (const v of openV) {
    // Absolute 10 µm floor on the cap: at a DEGENERATE tip (a B-spline sliver tapering to a pole)
    // the rail segments themselves shrink to microns, so half-a-segment blocks the very zip the
    // crack needs (wallganizer's 4 µm crumb edges pinned a 9 µm crack open at the old 3 µm floor).
    // No real CAD feature lives at 10 µm, so the floor cannot weld distinct geometry.
    const cap = Math.min(tol, Math.max(0.5 * (vCap.get(v) ?? Infinity), 1e-2));
    let best = -1, bestD = cap * cap;
    const cx = Math.round(px(v) / cell), cy = Math.round(py(v) / cell), cz = Math.round(pz(v) / cell);
    for (let gx = -1; gx <= 1; gx++) for (let gy = -1; gy <= 1; gy++) for (let gz = -1; gz <= 1; gz++) {
      for (const w of hash.get(`${cx + gx},${cy + gy},${cz + gz}`) ?? []) {
        if (w === v || find(w) === find(v) || use.has(ek(v, w))) continue;
        const d = (px(v) - px(w)) ** 2 + (py(v) - py(w)) ** 2 + (pz(v) - pz(w)) ** 2;
        if (d < bestD) { bestD = d; best = w; }
      }
    }
    if (best >= 0) uni(v, best);
  }

  // Compact surviving representatives; remap; drop topologically degenerate triangles.
  const remap = new Int32Array(nv).fill(-1);
  const pos: number[] = [];
  const idxOf = (v: number): number => { const r = find(v); if (remap[r] === -1) { remap[r] = pos.length / 3; pos.push(px(r), py(r), pz(r)); } return remap[r]!; };
  const outI: number[] = [];
  const keep: boolean[] = new Array(nt);
  for (let t = 0; t < nt; t++) {
    const a = idxOf(I[t * 3]!), b = idxOf(I[t * 3 + 1]!), c = idxOf(I[t * 3 + 2]!);
    if (a === b || b === c || c === a) { keep[t] = false; continue; }
    keep[t] = true; outI.push(a, b, c);
  }
  return { positions: Float64Array.from(pos), indices: Uint32Array.from(outI), keep };
}

/**
 * Repair T-junction cracks: an open edge one of whose flank vertices lies (numerically) ON another
 * open edge — the two sides of a crack subdivide the same 3D line differently, so the vertex zip
 * can never pair them (letters' engraving overlap: vertex 0.0001mm off the opposing edge but
 * 0.057mm from its endpoints; wallganizer's residual seams likewise). Splitting the owning
 * triangle at the on-edge vertex makes both sides share identical sub-edges BY INDEX, closing the
 * crack with zero geometric change (positions untouched — the split reuses the existing vertex).
 * Conservative by construction: only OPEN edges participate (defects, never real geometry), the
 * vertex must sit within `tol` of the segment's interior and clear of its endpoints, and the pass
 * iterates at most 3 rounds. Returns retriangulated indices + faceOf (same positions).
 */
function zipTJunctions(P: Float64Array, I0: Uint32Array, faceOf0: number[], tol: number): { indices: Uint32Array; faceOf: number[] } {
  let I = I0, faceOf = faceOf0;
  const KEY = 2 ** 26;
  const ek = (a: number, b: number): number => (a < b ? a * KEY + b : b * KEY + a);
  for (let round = 0; round < 3; round++) {
    const use = new Map<number, number>();
    for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) use.set(ek(I[i + e]!, I[i + (e + 1) % 3]!), (use.get(ek(I[i + e]!, I[i + (e + 1) % 3]!)) ?? 0) + 1);
    const openE: { a: number; b: number; tri: number; edge: number }[] = [];
    const openV = new Set<number>();
    for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
      const a = I[i + e]!, b = I[i + (e + 1) % 3]!;
      if (use.get(ek(a, b)) !== 1) continue;
      openE.push({ a, b, tri: i / 3, edge: e });
      openV.add(a); openV.add(b);
    }
    if (!openE.length) break;
    // Spatial hash of open vertices at the tolerance scale. Two-level NUMERIC keys (x column →
    // packed y/z): string keys made FindOrderedHashSetEntry the top profile entry corpus-wide —
    // a model with a handful of long rescue-density open edges spent 15 MINUTES here.
    const cell = Math.max(tol, 1e-9);
    const OFF = 1 << 20, PACK = 1 << 21; // ±2^20 cells/axis (20m at 0.02mm), packed product < 2^42: exact
    const hash = new Map<number, Map<number, number[]>>();
    for (const v of openV) {
      const cx = Math.round(P[v * 3]! / cell), cy = Math.round(P[v * 3 + 1]! / cell), cz = Math.round(P[v * 3 + 2]! / cell);
      let col = hash.get(cx);
      if (!col) { col = new Map(); hash.set(cx, col); }
      const k = (cy + OFF) * PACK + (cz + OFF);
      const arr = col.get(k);
      if (arr) arr.push(v); else col.set(k, [v]);
    }
    // per-triangle-edge insertions: vertex + parametric position along (a,b)
    const ins = new Map<number, { v: number; t: number }[]>(); // key tri*3+edge
    let found = 0;
    for (const oe of openE) {
      const ax = P[oe.a * 3]!, ay = P[oe.a * 3 + 1]!, az = P[oe.a * 3 + 2]!;
      const ex = P[oe.b * 3]! - ax, ey = P[oe.b * 3 + 1]! - ay, ez = P[oe.b * 3 + 2]! - az;
      const l2 = ex * ex + ey * ey + ez * ez;
      if (l2 < 1e-24) continue;
      const len = Math.sqrt(l2);
      // Candidate cells are walked ALONG the segment (1-cell dilation), not over its bounding box:
      // a bbox scan on a long diagonal open edge visits (len/cell)³ cells — a single 95mm sliver
      // rail at the 0.02mm tolerance is 10^11 cells, which turned an 11s ABC model into a hang the
      // moment a long open edge appeared. Any vertex within tol of the segment lies inside the
      // dilated tube the walk covers, so the candidate set is identical.
      const list: { v: number; t: number }[] = [];
      // No visited-set: consecutive steps move at most one cell, so cells already covered by the
      // PREVIOUS step's 3×3×3 neighbourhood are skipped with three numeric compares. An occasional
      // re-scan when the segment wanders back over old cells is harmless (candidates are re-checked
      // and deduped), and the numeric walk replaces the string-keyed Set that dominated profiles.
      const steps = Math.min(100000, 2 * Math.ceil(len / cell) + 1);
      let pcx = Infinity, pcy = Infinity, pcz = Infinity;
      for (let si = 0; si <= steps; si++) {
        const tt = si / steps;
        const cx = Math.round((ax + ex * tt) / cell), cy = Math.round((ay + ey * tt) / cell), cz = Math.round((az + ez * tt) / cell);
        if (cx === pcx && cy === pcy && cz === pcz) continue;
        for (let gx = -1; gx <= 1; gx++) {
          const col = hash.get(cx + gx);
          if (!col) continue;
          for (let gy = -1; gy <= 1; gy++) for (let gz = -1; gz <= 1; gz++) {
            if (Math.abs(cx + gx - pcx) <= 1 && Math.abs(cy + gy - pcy) <= 1 && Math.abs(cz + gz - pcz) <= 1) continue; // covered last step
            const arr = col.get((cy + gy + OFF) * PACK + (cz + gz + OFF));
            if (!arr) continue;
            for (const v of arr) {
              if (v === oe.a || v === oe.b) continue;
              const qx = P[v * 3]! - ax, qy = P[v * 3 + 1]! - ay, qz = P[v * 3 + 2]! - az;
              const t = (qx * ex + qy * ey + qz * ez) / l2;
              if (t * len < tol || (1 - t) * len < tol) continue; // too near an endpoint — weld territory
              const d = Math.hypot(qx - t * ex, qy - t * ey, qz - t * ez);
              if (d <= tol && !list.some((x) => x.v === v)) list.push({ v, t });
            }
          }
        }
        pcx = cx; pcy = cy; pcz = cz;
      }
      if (list.length) { ins.set(oe.tri * 3 + oe.edge, list.sort((x, y) => x.t - y.t)); found += list.length; }
    }
    if (!found) break;
    // refan the affected triangles (a triangle may have insertions on several of its edges)
    const outI: number[] = [], outF: number[] = [];
    for (let t = 0; t < I.length / 3; t++) {
      const anyIns = ins.has(t * 3) || ins.has(t * 3 + 1) || ins.has(t * 3 + 2);
      if (!anyIns) { outI.push(I[t * 3]!, I[t * 3 + 1]!, I[t * 3 + 2]!); outF.push(faceOf[t]!); continue; }
      // polygon = triangle boundary with inserted vertices, fanned from the first corner
      const poly: number[] = [];
      for (let e = 0; e < 3; e++) {
        poly.push(I[t * 3 + e]!);
        for (const x of ins.get(t * 3 + e) ?? []) poly.push(x.v);
      }
      for (let i = 1; i + 1 < poly.length; i++) { outI.push(poly[0]!, poly[i]!, poly[i + 1]!); outF.push(faceOf[t]!); }
    }
    I = Uint32Array.from(outI); faceOf = outF;
  }
  return { indices: I, faceOf };
}

/**
 * Fill small open-ring holes. Both classes are defects by construction in a closed solid — the
 * B-rep itself is watertight, so every open ring is meshing loss, and the only question is whether
 * a local fill stays near the true surface:
 *  - micro (perimeter ≤ tol): a pinhole at a collapsed B-spline pole/tip whose ring vertices are
 *    all mutually edge-connected, so the vertex zip can't close it. Filled in open shells too.
 *  - patch (`big`, closed solids only): a small missing surface patch — a strip the CDT parity
 *    flood dropped between constraints (Ontos #91640), a seam sliver whose partner rail never got
 *    emitted, a corner triangle lost to the weld's degenerate-collapse drop. Empirically (ABC
 *    chunk-0: 169 residual rings across the 138 near-watertight seam-leak models) these rings have
 *    ≤ 20 vertices and best-fit-plane deviation ≤ 0.12 of their perimeter, while their ABSOLUTE
 *    perimeter runs to 40 target-edge lengths (straight rails sample as single long segments) — so
 *    the gates are a vertex cap and a PROPORTIONAL flatness cap, not an absolute size cap:
 *    n ≤ 24 and dev ≤ max(0.15·per, big.dev) (a 3-ring is trivially planar and always accepted;
 *    big.dev keeps the legacy sub-chordTol notch fills). A genuine deep notch fails the flatness
 *    gate; a failed face's rim fails the vertex cap (curved rims sample densely, one vertex per
 *    ~target edge) or the 64·edge walk bail. The fill's deviation from the true surface is bounded
 *    by the ring's own near-planarity — the same chord-error scale the surrounding mesh commits.
 * Rings are triangulated min-area (see triangulateRing) rather than fanned: the dominant residual
 * shape is an elongated zigzag slit, where a fan from one rail end folds over the opposite rail
 * while the min-area triangulation stitches rail-to-rail like the ribbon meshers. Fill triangles
 * are wound opposite the directed ring so they pair manifold-consistently with their neighbours.
 */
/**
 * CROSS-SOLID SEWING (rec ④ centerpiece). Solids are welded independently and kept disjoint — the
 * right call for genuine assemblies — but a large ABC class models ONE part as several open
 * shells that only close JOINTLY: a bare cylinder tube as its own 1-face solid, its two cap disks
 * in another solid (the run7 seam-leak census: 444 of 561 leaking models are multi-solid, and the
 * dominant open-edge class sits exactly ON a shared model edge that both solids carry as their
 * own edge entity, sampled separately). Each such junction shows up here as TWO coincident open
 * boundary RINGS in different solids. Pair them geometrically (mutual pointwise proximity within
 * the sew tolerance — the two polylines sample the same true curve, so they differ by chord sag)
 * and loft an INDEX-based ribbon between existing vertices: every previously-open segment gains
 * exactly one partner triangle (manifold), no new vertices, bodies stay separable by solidOfTri
 * except for the bridging ribbon.
 *
 * Safety: only OPEN boundary rings participate, so closed bodies in a true assembly (a screw
 * resting in a hole — coincident faces, no open edges) are never touched; a ring is sewn at most
 * once, to its best mutual match, and only when BOTH rings track each other everywhere (a rim
 * half-covered by two caps fails the mutual gate and stays open rather than sewn wrong).
 */
function sewSolids(P: number[], I: number[], faceOfTri: number[], solidOfTri: number[], tol: number, openIds: Set<number>, unsewn?: Set<number>, sewnSolids?: Set<number>): number {
  const KEY = 2 ** 26;
  const ek = (a: number, b: number): number => (a < b ? a * KEY + b : b * KEY + a);
  const use = new Map<number, number>();
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
    const k = ek(I[i + e]!, I[i + (e + 1) % 3]!);
    use.set(k, (use.get(k) ?? 0) + 1);
  }
  // Directed boundary edges and their owning triangle; walk simple rings (skip ambiguous pinches).
  const nxt = new Map<number, number>();
  const triAt = new Map<number, number>();
  const multi = new Set<number>();
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
    const a = I[i + e]!, b = I[i + (e + 1) % 3]!;
    if (use.get(ek(a, b)) !== 1) continue;
    if (nxt.has(a)) multi.add(a);
    nxt.set(a, b); triAt.set(a, i / 3);
  }
  type Ring = { v: number[]; solid: number; fid: number; per: number; frac: number[]; cx: number; cy: number; cz: number; r: number };
  const rings: Ring[] = [];
  const seen = new Set<number>();
  for (const start of nxt.keys()) {
    if (seen.has(start)) continue;
    const v: number[] = [];
    let cur = start, ok = true;
    for (let guard = 0; guard <= nxt.size; guard++) {
      if (multi.has(cur)) { ok = false; break; }
      v.push(cur); seen.add(cur);
      const n = nxt.get(cur);
      if (n === undefined) { ok = false; break; }
      cur = n;
      if (cur === start) break;
      if (guard === nxt.size) ok = false;
    }
    if (!ok || v.length < 3) continue;
    const t0 = triAt.get(start)!;
    let per = 0, cx = 0, cy = 0, cz = 0;
    for (let i = 0; i < v.length; i++) {
      const a = v[i]!, b = v[(i + 1) % v.length]!;
      per += Math.hypot(P[b * 3]! - P[a * 3]!, P[b * 3 + 1]! - P[a * 3 + 1]!, P[b * 3 + 2]! - P[a * 3 + 2]!);
      cx += P[a * 3]!; cy += P[a * 3 + 1]!; cz += P[a * 3 + 2]!;
    }
    cx /= v.length; cy /= v.length; cz /= v.length;
    let r = 0;
    for (const a of v) r = Math.max(r, Math.hypot(P[a * 3]! - cx, P[a * 3 + 1]! - cy, P[a * 3 + 2]! - cz));
    const frac = [0];
    for (let i = 1; i <= v.length; i++) {
      const a = v[i - 1]!, b = v[i % v.length]!;
      frac.push(frac[i - 1]! + Math.hypot(P[b * 3]! - P[a * 3]!, P[b * 3 + 1]! - P[a * 3 + 1]!, P[b * 3 + 2]! - P[a * 3 + 2]!));
    }
    const tot = frac[frac.length - 1]! || 1;
    rings.push({ v, solid: solidOfTri[t0]!, fid: faceOfTri[t0]!, per, frac: frac.slice(0, v.length).map((x) => x / tot), cx, cy, cz, r });
  }
  if (DBG) console.error(`[tess] sewSolids: ${rings.length} open rings: ${rings.slice(0, 12).map((r) => `s${r.solid}×${r.v.length}(per=${r.per.toFixed(2)})`).join(" ")}`);
  if (rings.length < 2) {
    for (const r of rings) unsewn?.add(r.solid);
    return 0;
  }
  // Mutual pointwise proximity, strided; a ring pair must coincide BOTH ways.
  const distToRing = (x: number, y: number, z: number, R: Ring): number => {
    let best = Infinity;
    for (let i = 0; i < R.v.length; i++) {
      const a = R.v[i]!, b = R.v[(i + 1) % R.v.length]!;
      const ax = P[a * 3]!, ay = P[a * 3 + 1]!, az = P[a * 3 + 2]!;
      const ex = P[b * 3]! - ax, ey = P[b * 3 + 1]! - ay, ez = P[b * 3 + 2]! - az;
      const l2 = ex * ex + ey * ey + ez * ez;
      let t = l2 > 0 ? ((x - ax) * ex + (y - ay) * ey + (z - az) * ez) / l2 : 0;
      t = t < 0 ? 0 : t > 1 ? 1 : t;
      const d = (x - ax - t * ex) ** 2 + (y - ay - t * ey) ** 2 + (z - az - t * ez) ** 2;
      if (d < best) best = d;
    }
    return Math.sqrt(best);
  };
  const matchScore = (A: Ring, B: Ring): number => {
    const st = Math.max(1, Math.floor(A.v.length / 48));
    let worst = 0;
    for (let i = 0; i < A.v.length; i += st) {
      const a = A.v[i]!;
      const d = distToRing(P[a * 3]!, P[a * 3 + 1]!, P[a * 3 + 2]!, B);
      if (d > worst) worst = d;
      if (worst > tol) return Infinity;
    }
    return worst;
  };
  const paired = new Set<number>();
  let sewn = 0;
  for (let i = 0; i < rings.length; i++) {
    if (paired.has(i)) continue;
    const A = rings[i]!;
    let bj = -1, bScore = Infinity;
    for (let j = 0; j < rings.length; j++) {
      if (j === i || paired.has(j)) continue;
      const B = rings[j]!;
      if (B.solid === A.solid) continue;
      // Watertightness accounting excludes open (surface-model) bodies per TRIANGLE, so a ribbon
      // between an excluded and an included shell just moves the open line — sew like with like.
      if (openIds.has(A.solid) !== openIds.has(B.solid)) continue;
      if (Math.hypot(B.cx - A.cx, B.cy - A.cy, B.cz - A.cz) > 0.5 * (A.r + B.r) + tol) continue;
      if (Math.abs(A.per - B.per) > 0.2 * Math.max(A.per, B.per) + 4 * tol) continue;
      const s = Math.max(matchScore(A, B), matchScore(B, A));
      if (s < bScore) { bScore = s; bj = j; }
    }
    if (bj < 0 || bScore > tol) continue;
    const B = rings[bj]!;
    paired.add(i); paired.add(bj);
    sewnSolids?.add(A.solid); sewnSolids?.add(B.solid);
    // Ribbon: rails = A REVERSED and B in the direction that tracks it — the loft's triangles then
    // traverse each previously-open segment OPPOSITE to its owning triangle (manifold pairing).
    const c1 = A.v.slice().reverse();
    const f1 = ((): number[] => {
      const f = [0];
      for (let k = 1; k <= c1.length; k++) {
        const a = c1[k - 1]!, b = c1[k % c1.length]!;
        f.push(f[k - 1]! + Math.hypot(P[b * 3]! - P[a * 3]!, P[b * 3 + 1]! - P[a * 3 + 1]!, P[b * 3 + 2]! - P[a * 3 + 2]!));
      }
      const tot = f[f.length - 1]! || 1;
      return f.slice(0, c1.length).map((x) => x / tot);
    })();
    // Rotate B to start nearest c1[0], flip if the reverse tracks c1 better at the quarter point.
    let bi = 0, bd = Infinity;
    for (let k = 0; k < B.v.length; k++) {
      const d = Math.hypot(P[B.v[k]! * 3]! - P[c1[0]! * 3]!, P[B.v[k]! * 3 + 1]! - P[c1[0]! * 3 + 1]!, P[B.v[k]! * 3 + 2]! - P[c1[0]! * 3 + 2]!);
      if (d < bd) { bd = d; bi = k; }
    }
    let c2 = [...B.v.slice(bi), ...B.v.slice(0, bi)];
    const at = (c: number[], f: number): number => c[Math.min(c.length - 1, Math.round(f * c.length)) % c.length]!;
    const q1 = at(c1, 0.25);
    const rev = [c2[0]!, ...c2.slice(1).reverse()];
    const dq = (v: number, w: number): number => Math.hypot(P[v * 3]! - P[w * 3]!, P[v * 3 + 1]! - P[w * 3 + 1]!, P[v * 3 + 2]! - P[w * 3 + 2]!);
    if (dq(at(rev, 0.25), q1) < dq(at(c2, 0.25), q1)) c2 = rev;
    const f2 = ((): number[] => {
      const f = [0];
      for (let k = 1; k <= c2.length; k++) {
        const a = c2[k - 1]!, b = c2[k % c2.length]!;
        f.push(f[k - 1]! + Math.hypot(P[b * 3]! - P[a * 3]!, P[b * 3 + 1]! - P[a * 3 + 1]!, P[b * 3 + 2]! - P[a * 3 + 2]!));
      }
      const tot = f[f.length - 1]! || 1;
      return f.slice(0, c2.length).map((x) => x / tot);
    })();
    // Cyclic two-rail loft by arc-length fraction (stitchRings' pairing, index-based).
    const na = c1.length, nb = c2.length;
    const fa = (k: number): number => (k >= na ? 1 : f1[k]!);
    const fb = (k: number): number => (k >= nb ? 1 : f2[k]!);
    let ia = 0, ib = 0;
    while (ia < na || ib < nb) {
      if (ia < na && (ib >= nb || fa(ia) < fb(ib))) {
        I.push(c1[ia % na]!, c1[(ia + 1) % na]!, c2[ib % nb]!);
        ia++;
      } else {
        I.push(c1[ia % na]!, c2[(ib + 1) % nb]!, c2[ib % nb]!);
        ib++;
      }
      faceOfTri.push(A.fid); solidOfTri.push(A.solid);
    }
    sewn++;
    if (DBG) console.error(`[tess] sewSolids: ring(${A.v.length}, solid ${A.solid}) <-> ring(${B.v.length}, solid ${B.solid}) worst=${bScore.toExponential(2)}`);
  }
  for (let i = 0; i < rings.length; i++) if (!paired.has(i)) unsewn?.add(rings[i]!.solid);
  return sewn;
}

function fillMicroHoles(P: Float64Array, I: Uint32Array, faceOf: number[], tol: number, big?: { edge: number; dev: number }): { indices: number[]; faceOf: number[] } {
  const KEY = 2 ** 26;
  const ek = (a: number, b: number): number => (a < b ? a * KEY + b : b * KEY + a);
  const use = new Map<number, number>();
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) use.set(ek(I[i + e]!, I[i + (e + 1) % 3]!), (use.get(ek(I[i + e]!, I[i + (e + 1) % 3]!)) ?? 0) + 1);
  // Directed boundary edges a->b (as traversed by their owning triangle) and the owning face.
  const nxt = new Map<number, number>();
  const faceAt = new Map<number, number>();
  const multi = new Set<number>(); // boundary vertices with >1 outgoing edge — walking is ambiguous
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
    const a = I[i + e]!, b = I[i + (e + 1) % 3]!;
    if (use.get(ek(a, b)) !== 1) continue;
    if (nxt.has(a)) multi.add(a);
    nxt.set(a, b); faceAt.set(a, faceOf[i / 3]!);
  }
  const planarDev = (ring: number[]): number => {
    let nx = 0, ny = 0, nz = 0, cx = 0, cy = 0, cz = 0;
    const n = ring.length;
    for (let i = 0; i < n; i++) {
      const a = ring[i]! * 3, b = ring[(i + 1) % n]! * 3;
      nx += (P[a + 1]! - P[b + 1]!) * (P[a + 2]! + P[b + 2]!);
      ny += (P[a + 2]! - P[b + 2]!) * (P[a]! + P[b]!);
      nz += (P[a]! - P[b]!) * (P[a + 1]! + P[b + 1]!);
      cx += P[a]!; cy += P[a + 1]!; cz += P[a + 2]!;
    }
    const l = Math.hypot(nx, ny, nz);
    if (l < 1e-30) return Infinity;
    nx /= l; ny /= l; nz /= l; cx /= n; cy /= n; cz /= n;
    let mx = 0;
    for (const v of ring) mx = Math.max(mx, Math.abs((P[v * 3]! - cx) * nx + (P[v * 3 + 1]! - cy) * ny + (P[v * 3 + 2]! - cz) * nz));
    return mx;
  };
  const MAXN = 24;   // patch-ring vertex cap (biggest observed defect ring: 20)
  const FLAT = 0.15; // patch-ring planar-deviation cap, as a fraction of ring perimeter
  const perCap = big ? 64 * big.edge : tol;
  const outI: number[] = [], outF: number[] = [];
  const seen = new Set<number>();
  const filled = new Set<number>(); // vertices of rings actually filled (seen also marks rejected walks)
  for (const start of nxt.keys()) {
    if (seen.has(start)) continue;
    const ring: number[] = [];
    let cur = start, per = 0, ok = true;
    for (let g = 0; g <= 64; g++) {
      ring.push(cur); seen.add(cur);
      const n = nxt.get(cur);
      if (n === undefined) { ok = false; break; }
      per += Math.hypot(P[cur * 3]! - P[n * 3]!, P[cur * 3 + 1]! - P[n * 3 + 1]!, P[cur * 3 + 2]! - P[n * 3 + 2]!);
      if (per > perCap) { ok = false; break; }
      cur = n;
      if (cur === start) break;
      if (g === 64 || seen.has(cur)) { ok = false; break; }
    }
    if (!ok || cur !== start || ring.length < 3 || ring.some((v) => multi.has(v))) {
      if (DBG && ring.length <= 10) console.error(`[fill] ring rejected: ok=${ok} closed=${cur === start} len=${ring.length} multi=${ring.some((v) => multi.has(v))} per=${per.toFixed(3)}`);
      continue;
    }
    const micro = per <= tol;
    const patch = !micro && !!big && ring.length <= MAXN && (ring.length === 3 || planarDev(ring) <= Math.max(FLAT * per, big.dev));
    if (!micro && !patch) {
      if (DBG) console.error(`[fill] ring not filled: len=${ring.length} per=${per.toFixed(3)} dev=${planarDev(ring).toExponential(2)}`);
      continue;
    }
    triangulateRing(P, ring, use, ek, per, outI, outF, faceAt.get(ring[0]!)!);
    for (const v of ring) filled.add(v);
  }
  // UNDIRECTED pass (big only): what the directed walk can't close. Two shapes land here: a notch
  // between faces whose local windings disagree (no consistent directed cycle — the directed walk
  // bails at its doubled-outgoing vertex), and a PINCHED ring — two or three holes sharing a
  // vertex (figure-8), whose pinch carries two outgoing directed edges and so poisons every
  // directed walk through it (all 17 ABC near-watertight models left after the directed pass are
  // exactly this shape: degree spectrum {2:n, 4:1-2}, no odd vertices). Walk a trail over the
  // unused open edges, popping a simple cycle each time the trail revisits a vertex still on its
  // path (Hierholzer-style); every popped cycle faces the same micro/patch gates as the directed
  // pass, so a pinch simply decomposes into its constituent holes. Each cycle is wound against the
  // direction the majority of its edges are traversed by their owning triangles; the orientation
  // pass downstream settles any stragglers.
  if (big) {
    const dirOpen = new Set<number>(); // a->b as traversed by the owning triangle (winding vote)
    const edges: { a: number; b: number; face: number; used: boolean }[] = [];
    const inc = new Map<number, number[]>(); // vertex -> incident open-edge ids
    for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
      const a = I[i + e]!, b = I[i + (e + 1) % 3]!;
      if (use.get(ek(a, b)) !== 1) continue;
      dirOpen.add(a * KEY + b);
      if (filled.has(a) || filled.has(b)) continue; // already closed by the directed pass
      const id = edges.length;
      edges.push({ a, b, face: faceOf[i / 3]!, used: false });
      (inc.get(a) ?? inc.set(a, []).get(a)!).push(id);
      (inc.get(b) ?? inc.set(b, []).get(b)!).push(id);
    }
    const cursor = new Map<number, number>();
    const unusedAt = (v: number): number => {
      const list = inc.get(v);
      if (!list) return -1;
      let c = cursor.get(v) ?? 0;
      while (c < list.length && edges[list[c]!]!.used) c++;
      cursor.set(v, c);
      return c < list.length ? list[c]! : -1;
    };
    for (let seed = 0; seed < edges.length; seed++) {
      if (edges[seed]!.used) continue;
      const path: number[] = [edges[seed]!.a];
      const at = new Map<number, number>([[edges[seed]!.a, 0]]);
      let cur = edges[seed]!.a;
      for (let guard = 0; guard <= edges.length; guard++) {
        const eid = unusedAt(cur);
        if (eid < 0) break; // dead end (odd-degree residue) — leftover path stays open
        const E = edges[eid]!;
        E.used = true;
        const next = E.a === cur ? E.b : E.a;
        if (!at.has(next)) { path.push(next); at.set(next, path.length - 1); cur = next; continue; }
        // trail closed a simple cycle: pop it off the path and gate it like any other ring
        const i0 = at.get(next)!;
        const ring = path.slice(i0);
        for (let k = i0 + 1; k < path.length; k++) at.delete(path[k]!);
        path.length = i0 + 1;
        cur = next;
        const n = ring.length;
        if (n < 3) continue; // doubled edge between one vertex pair — nothing to fill
        let per = 0;
        for (let i = 0; i < n; i++) per += Math.hypot(
          P[ring[i]! * 3]! - P[ring[(i + 1) % n]! * 3]!,
          P[ring[i]! * 3 + 1]! - P[ring[(i + 1) % n]! * 3 + 1]!,
          P[ring[i]! * 3 + 2]! - P[ring[(i + 1) % n]! * 3 + 2]!);
        const okFill = per <= tol || (n <= MAXN && per <= perCap && (n === 3 || planarDev(ring) <= Math.max(FLAT * per, big.dev)));
        if (!okFill) {
          if (DBG) console.error(`[fill] undirected cycle not filled: len=${n} per=${per.toFixed(3)} dev=${planarDev(ring).toExponential(2)}`);
          continue;
        }
        let agree = 0;
        for (let i = 0; i < n; i++) if (dirOpen.has(ring[i]! * KEY + ring[(i + 1) % n]!)) agree++;
        const r = 2 * agree >= n ? ring : ring.slice().reverse();
        triangulateRing(P, r, use, ek, per, outI, outF, E.face);
        for (const v of ring) filled.add(v);
        if (DBG) console.error(`[fill] undirected ring filled: len=${n} per=${per.toFixed(3)}`);
      }
    }
  }
  return { indices: outI, faceOf: outF };
}

/**
 * Minimum-area triangulation of a closed 3D ring (classic interval DP, O(n³), n ≤ 65 here). Area
 * is the right objective for hole patches: a flat notch gets the natural planar triangulation and
 * an elongated zigzag slit gets stitched rail-to-rail with near-zero-area triangles, where a fan
 * from one rail end would fold across the opposite rail. Chords that already exist as mesh edges
 * carry a penalty far above any real area so the fill never manufactures a non-manifold edge
 * (interior chords are otherwise new vertex pairs, used exactly twice within the fill). Triangles
 * are emitted wound AGAINST ring order — DP triangle (i,m,j), i<m<j, as (ring[j],ring[m],ring[i])
 * — so every ring edge is traversed opposite its owning open edge and pairs manifold-consistently.
 */
function triangulateRing(P: Float64Array, ring: number[], use: Map<number, number>, ek: (a: number, b: number) => number, per: number, outI: number[], outF: number[], face: number): void {
  const n = ring.length;
  if (n < 3) return;
  if (n === 3) { outI.push(ring[2]!, ring[1]!, ring[0]!); outF.push(face); return; }
  const X = (i: number): number => P[ring[i]! * 3]!, Y = (i: number): number => P[ring[i]! * 3 + 1]!, Z = (i: number): number => P[ring[i]! * 3 + 2]!;
  const area = (i: number, m: number, j: number): number => {
    const ux = X(m) - X(i), uy = Y(m) - Y(i), uz = Z(m) - Z(i);
    const vx = X(j) - X(i), vy = Y(j) - Y(i), vz = Z(j) - Z(i);
    return 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
  };
  // Ring edges (j = i+1, and the closing pair 0,n-1) are the hole boundary — free. Interior chords
  // cost their existing-edge penalty once, charged where the DP creates them as a split boundary.
  const pen = (i: number, j: number): number => (j - i <= 1 || (i === 0 && j === n - 1)) ? 0 : (use.has(ek(ring[i]!, ring[j]!)) ? 1e6 * per * per : 0);
  const cost = new Float64Array(n * n);
  const split = new Int32Array(n * n).fill(-1);
  for (let len = 2; len < n; len++) for (let i = 0; i + len < n; i++) {
    const j = i + len;
    let bc = Infinity, bm = -1;
    for (let m = i + 1; m < j; m++) {
      const c = cost[i * n + m]! + cost[m * n + j]! + area(i, m, j) + pen(i, m) + pen(m, j);
      if (c < bc) { bc = c; bm = m; }
    }
    cost[i * n + j] = bc; split[i * n + j] = bm;
  }
  const stack: [number, number][] = [[0, n - 1]];
  while (stack.length) {
    const [i, j] = stack.pop()!;
    if (j - i < 2) continue;
    const m = split[i * n + j]!;
    outI.push(ring[j]!, ring[m]!, ring[i]!); outF.push(face);
    stack.push([i, m], [m, j]);
  }
}

/** Weld coincident triangle-soup vertices into an indexed mesh (positions quantised to eps). */
export function weld(verts: number[], eps = 1e-6): { mesh: IndexedMesh } {
  const map = new Map<string, number>();
  const pos: number[] = [];
  const indices: number[] = [];
  const q = (x: number): number => Math.round(x / eps);
  for (let i = 0; i < verts.length; i += 3) {
    const x = verts[i]!, y = verts[i + 1]!, z = verts[i + 2]!;
    const key = `${q(x)},${q(y)},${q(z)}`;
    let idx = map.get(key);
    if (idx === undefined) { idx = pos.length / 3; pos.push(x, y, z); map.set(key, idx); }
    indices.push(idx);
  }
  return { mesh: { positions: Float64Array.from(pos), indices: Uint32Array.from(indices) } };
}
