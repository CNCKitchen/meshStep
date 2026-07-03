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
import { cross, dot, lerp } from "../geom/vec.ts";
import { ref, refList, list, num, numList } from "../step/entities.ts";
import type { Param } from "../step/parser.ts";
import type { BrepModel, BLoop } from "../brep/build.ts";
import type { IndexedMesh } from "../io/stl.ts";
import type { Surface } from "../geom/surfaces.ts";
import { makeSurface, isSphere, isBSpline, type Sphere, type BSplineSurface } from "../geom/surfaces.ts";
import { sampleEdgePolyline } from "../geom/curves.ts";
import { constrainedTriangulate } from "./cdt2d.ts";

const TWO_PI = Math.PI * 2;
type P2 = [number, number];

export interface TessOptions {
  /** Max chord deviation (mm) for curve/surface sampling. */
  chordTol?: number;
  /** Target / maximum edge length (mm). */
  targetEdge?: number;
  /** Max normal turn across an edge (radians) — drives curvature-adaptive interior density. */
  normalDev?: number;
}

export interface MeshResult {
  mesh: IndexedMesh;
  faceOfTri: Uint32Array;
  /** STEP solid (body) id per triangle; bodies are welded independently and kept disjoint. */
  solidOfTri: Uint32Array;
  stats: { solids: number; facesTotal: number; facesTessellated: number; skipped: Record<string, number> };
}

const bump = (o: Record<string, number>, k: string): void => { o[k] = (o[k] ?? 0) + 1; };

/** Emit a triangle, oriented so its normal points outward (surface normal × same_sense sign). */
function emitTri(
  verts: number[], faceIds: number[], a: Vec3, b: Vec3, c: Vec3, fid: number, surface: Surface, sign: number,
): void {
  const ng = cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
  if (ng[0] * ng[0] + ng[1] * ng[1] + ng[2] * ng[2] < 1e-18) return; // degenerate / zero-area
  const cen: Vec3 = [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
  const [u, v] = surface.project(cen);
  const outward = dot(ng, surface.normal(u, v)) * sign;
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
  return { p3: poly, p2, hugU: hugsSeam(surface, p2, 0), hugV: hugsSeam(surface, p2, 1) };
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
 * candidate during the phase-C search. */
function countSelfIntersections(pts: P2[], cap = 1 << 30): number {
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
): { asm: ReturnType<typeof assembleWire>; selfInts: number } {
  const slots: number[] = []; // flat (edgeIndex, axis) pairs, axis 0=u 1=v
  for (let k = 1; k < pcs.length; k++) {
    if (surface.periodicU && pcs[k]!.hugU) slots.push(2 * k);
    if (surface.periodicV && pcs[k]!.hugV) slots.push(2 * k + 1);
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
  if (legInts === 0) return { ...leg, windU: legWind[0], windV: legWind[1], tangled: false };

  const pcs: EdgePC[] = [];
  for (const oe of loop.edges) {
    const base = sampled.get(oe.edgeId);
    if (!base) continue;
    pcs.push(edgePcurve(surface, oe.orient ? base : base.slice().reverse()));
  }
  if (pcs.length === 0) return { ...leg, windU: legWind[0], windV: legWind[1], tangled: true };
  let asm = assembleWire(surface, pcs);
  let selfInts = countSelfIntersections(asm.p2);
  if (selfInts > 0) ({ asm, selfInts } = repairWire(surface, pcs, asm, selfInts, expectSign));
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
 * their existing floor unchanged. */
function faceTarget(surface: Surface, targetEdge: number, chordTol: number, normalDev: number, u = 0, v = 0): number {
  const Rc = surface.curvatureRadius(u, v);
  const floor = Math.min(targetEdge / 40, 30 * chordTol);
  return Number.isFinite(Rc)
    ? Math.max(floor, Math.min(targetEdge, Math.sqrt(8 * Rc * chordTol), Rc * normalDev))
    : targetEdge;
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
  if (outer.p3.length < 3) return false;
  // Malformed trimming: the enclosing loop collapsed to ~zero area in parameter space (a degenerate
  // boundary a CAD kernel left, or two edges that trace back over each other). There's no valid
  // region — emit nothing. A clean gap reads far better than a spurious cap sealing a hole shut.
  const ext = bboxExtent(outer.p2);
  if (Math.abs(polyArea(outer.p2)) < 1e-4 * ext * ext) return false;

  let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
  for (const q of outer.p2) {
    if (q[0] < umin) umin = q[0]; if (q[0] > umax) umax = q[0];
    if (q[1] < vmin) vmin = q[1]; if (q[1] > vmax) vmax = q[1];
  }
  const holes = projected.filter((_, i) => i !== oi).map((p) => p.lp);
  if (surface.periodicU) for (const h of holes) shiftIntoRange(h.p2, (umin + umax) / 2, 0, surface.uPeriod || TWO_PI);
  if (surface.periodicV) for (const h of holes) shiftIntoRange(h.p2, (vmin + vmax) / 2, 1, surface.vPeriod || TWO_PI);
  return gridCDT(surface, outer, holes, fid, verts, faceIds, targetEdge, chordTol, normalDev, sign);
}

/** CDT core shared by the trimmed-patch and seam-split meshers: interior grid + graded size field +
 * Delaunay refinement over an explicit outer/hole boundary in continuous (u,v) coordinates. */
function gridCDT(
  surface: Surface, outer: { p3: Vec3[]; p2: P2[] }, holes: { p3: Vec3[]; p2: P2[] }[], fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
): boolean {
  let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
  for (const q of outer.p2) {
    if (q[0] < umin) umin = q[0]; if (q[0] > umax) umax = q[0];
    if (q[1] < vmin) vmin = q[1]; if (q[1] > vmax) vmax = q[1];
  }
  const umid = (umin + umax) / 2, vmid = (vmin + vmax) / 2, eps = 1e-3;
  const dU = surface.evaluate(umid + eps, vmid), dU2 = surface.evaluate(umid - eps, vmid);
  const dV = surface.evaluate(umid, vmid + eps), dV2 = surface.evaluate(umid, vmid - eps);
  const uScale = Math.max(1e-9, Math.hypot(dU[0] - dU2[0], dU[1] - dU2[1], dU[2] - dU2[2]) / (2 * eps));
  const vScale = Math.max(1e-9, Math.hypot(dV[0] - dV2[0], dV[1] - dV2[1], dV[2] - dV2[2]) / (2 * eps));

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
  outer = sanitize(outer);
  holes = holes.map(sanitize);
  const holeP2 = holes.map((h) => h.p2);
  const inRegion = (p: P2): boolean => pointInPoly(p, outer.p2) && !holeP2.some((h) => pointInPoly(p, h));

  // Curvature-adaptive interior density so the initial mesh is already fine on curved faces.
  const target = faceTarget(surface, targetEdge, chordTol, normalDev, umid, vmid);
  const nU = Math.min(1200, Math.max(1, Math.round(((umax - umin) * uScale) / target)));
  const nV = Math.min(1200, Math.max(1, Math.round(((vmax - vmin) * vScale) / target)));

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
  // so 2D distance ≈ 3D arc length): a plain Delaunay there yields near-isotropic 3D triangles — a
  // cylinder (u spans 2π but R·2π in 3D) stops slivering at its seam; a plane is uScale=vScale=1.
  const SX = (u: number): number => u * uScale, SY = (v: number): number => v * vScale;
  // Boundary segments in scaled space, hashed, drive (a) a graded SIZE FIELD — size grows from each
  // edge's own length outward (so a tight-fillet edge shared with this flat face stays small near it
  // and coarsens away), capped at the face target — and (b) a "too close to the boundary" test.
  const sseg: [number, number, number, number, number][] = [];
  for (const idx of [outerIdx, ...holeIdx]) for (let i = 0; i < idx.length; i++) {
    const a = allP2[idx[i]!]!, b = allP2[idx[(i + 1) % idx.length]!]!;
    const ax = SX(a[0]), ay = SY(a[1]), bx = SX(b[0]), by = SY(b[1]);
    sseg.push([ax, ay, bx, by, Math.hypot(bx - ax, by - ay)]);
  }
  const csz = Math.max(target, 1e-6);
  const hkey = (ix: number, iy: number): number => Math.imul(ix, 73856093) ^ Math.imul(iy, 19349663);
  const segHash = new Map<number, number[]>();
  for (let i = 0; i < sseg.length; i++) { const k = hkey(Math.floor((sseg[i]![0] + sseg[i]![2]) / 2 / csz), Math.floor((sseg[i]![1] + sseg[i]![3]) / 2 / csz)); (segHash.get(k) ?? segHash.set(k, []).get(k)!).push(i); }
  const floor = target / 40;
  const sizeDist = (sx: number, sy: number): [number, number] => {
    // Size field = min of (a) the LOCAL curvature target — so a face whose curvature varies is
    // refined where it actually bends, not just at the patch midpoint — and (b) the boundary-graded
    // size, growing from each edge's own length outward. Capped at the face target, floored to bound.
    let size = faceTarget(surface, targetEdge, chordTol, normalDev, sx / uScale, sy / vScale);
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
  let tris = constrainedTriangulate(cdtPts, [outerIdx, ...holeIdx], interiorIdx);
  // Delaunay refinement: insert the circumcentre of any triangle whose circumdiameter exceeds the
  // local size field — this grades the mesh from the fine boundary into the interior (fillet runs
  // into chamfer) and, being Delaunay, keeps the new triangles well-shaped. Capped at a multiple of
  // the base grid: well-behaved faces converge long before it, but a skewed B-spline patch (whose
  // diagonal-metric triangles always look oversized) would otherwise refine to millions of points.
  const cap = 200 + (interiorIdx.length + outerIdx.length) * 6;
  const maxIter = 4;
  for (let iter = 0; iter < maxIter && interiorIdx.length < cap; iter++) {
    const fresh: P2[] = [];
    for (const [a, b, c] of tris) {
      const A = cdtPts[a]!, B = cdtPts[b]!, C = cdtPts[c]!;
      const d = 2 * (A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1]));
      if (Math.abs(d) < 1e-12) continue;
      const a2 = A[0] * A[0] + A[1] * A[1], b2 = B[0] * B[0] + B[1] * B[1], c2 = C[0] * C[0] + C[1] * C[1];
      let px = (a2 * (B[1] - C[1]) + b2 * (C[1] - A[1]) + c2 * (A[1] - B[1])) / d;
      let py = (a2 * (C[0] - B[0]) + b2 * (A[0] - C[0]) + c2 * (B[0] - A[0])) / d;
      const r = Math.hypot(px - A[0], py - A[1]);
      const [sz] = sizeDist(px, py);
      // Refine where the circumradius exceeds 0.65× the local size: fills the graded band near a
      // fine boundary (fillet-into-chamfer), follows intra-face curvature the base grid under-sampled,
      // and improves Delaunay quality generally. The remesh later coarsens over-dense flat regions.
      if (r <= sz * 0.65) continue;
      let u = px / uScale, v = py / vScale;
      if (!inRegion([u, v])) { px = (A[0] + B[0] + C[0]) / 3; py = (A[1] + B[1] + C[1]) / 3; u = px / uScale; v = py / vScale; if (!inRegion([u, v])) continue; }
      const [sz2, dist2] = sizeDist(px, py);
      if (dist2 < 0.5 * sz2) continue;
      fresh.push([px, py]);
    }
    if (!fresh.length) break;
    // Dedup new points that fall within ~half a local size of each other (avoid over-insertion).
    const acc = new Map<number, [number, number][]>();
    let added = false;
    for (const [px, py] of fresh) {
      const [sz] = sizeDist(px, py), hx = Math.floor(px / sz), hy = Math.floor(py / sz);
      let ok = true;
      for (let gx = hx - 1; gx <= hx + 1 && ok; gx++) for (let gy = hy - 1; gy <= hy + 1 && ok; gy++) {
        for (const [qx, qy] of acc.get(hkey(gx, gy)) ?? []) if ((px - qx) ** 2 + (py - qy) ** 2 < (0.5 * sz) ** 2) ok = false;
      }
      if (!ok) continue;
      (acc.get(hkey(hx, hy)) ?? acc.set(hkey(hx, hy), []).get(hkey(hx, hy))!).push([px, py]);
      interiorIdx.push(allP2.length); allP2.push([px / uScale, py / vScale]); allP3.push(surface.evaluate(px / uScale, py / vScale)); cdtPts.push([px, py]);
      added = true;
    }
    if (!added) break;
    tris = constrainedTriangulate(cdtPts, [outerIdx, ...holeIdx], interiorIdx);
  }
  for (const [a, b, c] of tris) emitTri(verts, faceIds, allP3[a]!, allP3[b]!, allP3[c]!, fid, surface, sign);
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

    // Rim -> polyline spanning [seam, seam+period], ascending in the around-coord, with a closing
    // duplicate at seam+period (3D-identical to the start; it becomes the far seam corner).
    const buildRim = (rim: { p3: Vec3[]; p2: P2[] }): { p3: Vec3[]; p2: P2[] } => {
      const items = rim.p2.map((q, i) => ({ a: normTo(q[c]), s: q[stackC], p3: rim.p3[i]! }));
      items.sort((x, y) => x.a - y.a);
      const first = items[0]!;
      items.push({ a: first.a + period, s: first.s, p3: first.p3 });
      return { p3: items.map((it) => it.p3), p2: items.map((it) => toP2(it.a, it.s)) };
    };
    const bottom = buildRim(rims[0]!), top = buildRim(rims[1]!);
    // Outer boundary: bottom left->right, then top right->left. The two vertical sides are the seam
    // (same 3D line at seam and seam+period) -> weld closes it.
    const outer = {
      p3: [...bottom.p3, ...top.p3.slice().reverse()],
      p2: [...bottom.p2, ...top.p2.slice().reverse()],
    };
    const holeLoops = holes.map((h) => ({
      p3: h.p3,
      p2: h.p2.map((q) => toP2(normTo(q[c]), q[stackC])),
    }));
    return gridCDT(surface, outer, holeLoops, fid, verts, faceIds, targetEdge, chordTol, normalDev, sign);
  };
  return (!!surface.periodicU && attempt(0)) || (!!surface.periodicV && attempt(1));
}

/**
 * Stitch two concentric (closed) rings of unequal point counts into a triangle band, advancing
 * whichever ring is behind in its angular fraction. Both rings must start at the same angle and run
 * the same way; a count-1 ring is a pole (fan). Shared by the cone / sphere / B-spline pole meshers.
 */
function stitchRings(
  verts: number[], faceIds: number[], A: Vec3[], B: Vec3[], fid: number, surface: Surface, sign: number,
): void {
  const na = A.length, nb = B.length;
  if (na === 1) { for (let j = 0; j < nb; j++) emitTri(verts, faceIds, A[0]!, B[j]!, B[(j + 1) % nb]!, fid, surface, sign); return; }
  if (nb === 1) { for (let k = 0; k < na; k++) emitTri(verts, faceIds, A[k]!, A[(k + 1) % na]!, B[0]!, fid, surface, sign); return; }
  let ia = 0, ib = 0;
  while (ia < na || ib < nb) {
    if (ia < na && (ib >= nb || ia / na < ib / nb)) { emitTri(verts, faceIds, A[ia % na]!, A[(ia + 1) % na]!, B[ib % nb]!, fid, surface, sign); ia++; }
    else { emitTri(verts, faceIds, A[ia % na]!, B[(ib + 1) % nb]!, B[ib % nb]!, fid, surface, sign); ib++; }
  }
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
      let wind = 0, amin = Infinity, amax = -Infinity, hcos = 0, hsin = 0, hsum = 0;
      const as: number[] = [];
      for (const p of pts) {
        const uv = surface.project(p);
        const a = uv[c], h = uv[1 - c]!;
        as.push(a); hsum += h;
        // Circular mean for a periodic stack coordinate (rim points may straddle its seam).
        hcos += Math.cos((h * TWO_PI) / stackPeriod); hsin += Math.sin((h * TWO_PI) / stackPeriod);
        if (a < amin) amin = a; if (a > amax) amax = a;
      }
      for (let i = 0; i < as.length; i++) {
        let d = as[(i + 1) % as.length]! - as[i]!;
        while (d > period / 2) d -= period; while (d < -period / 2) d += period;
        wind += d;
      }
      if (Math.abs(wind) < 0.9 * period && amax - amin < 0.9 * period) return false; // partial arc
      const h = stackPeriodic ? (Math.atan2(hsin, hcos) * stackPeriod) / TWO_PI : hsum / pts.length;
      rims.push({ pts, h, wind });
    }
    if (rims.length < 2) return false;

    // Order the rims along the stack coordinate. Non-periodic stack (cylinder/cone v): plain sort.
    // Periodic stack (torus): "between h0 and h1" is ambiguous (two arcs) — use the boundary
    // orientation: a CCW face in (u,v) travels its v0 rim in +u and its v1 rim in -u (c=0), and its
    // u1 rim in +v and u0 rim in -v (c=1), so the winding SIGN says which rim starts the stack.
    let stack: { from: number; width: number; bottom: Vec3[]; top: Vec3[] }[] = [];
    if (!stackPeriodic) {
      rims.sort((a, b) => a.h - b.h);
      for (let r = 0; r + 1 < rims.length; r++) {
        stack.push({ from: rims[r]!.h, width: rims[r + 1]!.h - rims[r]!.h, bottom: rims[r]!.pts, top: rims[r + 1]!.pts });
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
      stack.push({ from: A.h, width, bottom: A.pts, top: B.pts });
    }

    const half = period / 2;
    const wrap = (d: number): number => { while (d > half) d -= period; while (d < -half) d += period; return d; };
    // Rotate/flip `ring` so it starts at `ref[0]`'s angle and runs the same rotational direction.
    const align = (ref: Vec3[], ring: Vec3[]): Vec3[] => {
      const a0 = angleOf(ref[0]!);
      const dirRef = wrap(angleOf(ref[1]!) - a0) >= 0 ? 1 : -1;
      const r = ring.slice();
      if ((wrap(angleOf(r[1]!) - angleOf(r[0]!)) >= 0 ? 1 : -1) !== dirRef) r.reverse();
      let bi = 0, bd = Infinity;
      for (let i = 0; i < r.length; i++) { const dd = Math.abs(wrap(angleOf(r[i]!) - a0)); if (dd < bd) { bd = dd; bi = i; } }
      return [...r.slice(bi), ...r.slice(0, bi)];
    };

    for (const band of stack) {
      const bottom = band.bottom, top = align(bottom, band.top);
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
      let prev = bottom;
      for (let k = 1; k < nRings; k++) {
        ti = 0;
        const f = k / nRings;
        const ring = angs.map((a, j) => evalAt(c, a, hB[j]! + (hTopAt(progB[j]!) - hB[j]!) * f));
        stitchRings(verts, faceIds, prev, ring, fid, surface, sign);
        prev = ring;
      }
      stitchRings(verts, faceIds, prev, top, fid, surface, sign);
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
 * Deliberately NOT extended to chain >2 half-rails into inner/outer rings: an annulus whose two rings
 * are far apart (a hole in a wide face) then lofts a spurious CAP across the hole, which is much worse
 * than a clean gap. Only the unambiguous two-rail strip is filled.
 */
function tessellateRibbon(
  surface: Surface, loops: BLoop[], sampled: Map<number, Vec3[]>, fid: number,
  verts: number[], faceIds: number[], sign: number,
): boolean {
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const polys: Vec3[][] = [];
  for (const lp of loops) for (const oe of lp.edges) {
    const base = sampled.get(oe.edgeId);
    if (!base || base.length < 2) continue;
    polys.push(oe.orient ? base.slice() : base.slice().reverse());
  }
  if (polys.length < 2) return false;
  // Merge edges that trace the SAME curve twice (a pinched seam: same endpoints AND same midpoint) —
  // that doubled pair is one rail. Two edges sharing only endpoints but bowing apart (a lens) stay
  // distinct: the midpoint test is what tells a zero-width seam from a real thin strip.
  const mid = (p: Vec3[]): Vec3 => p[p.length >> 1]!;
  const rails: Vec3[][] = [];
  const used = new Array(polys.length).fill(false);
  for (let i = 0; i < polys.length; i++) {
    if (used[i]) continue;
    used[i] = true;
    const pi = polys[i]!;
    for (let j = i + 1; j < polys.length; j++) {
      if (used[j]) continue;
      const pj = polys[j]!;
      const reversed = d3(pi[0]!, pj[pj.length - 1]!) < 1e-6 && d3(pi[pi.length - 1]!, pj[0]!) < 1e-6;
      if (reversed && d3(mid(pi), mid(pj)) < 1e-6) { used[j] = true; break; }
    }
    rails.push(pi);
  }
  if (rails.length !== 2) return false;
  let c1 = rails[0]!, c2 = rails[1]!;
  // Align so both rails run the same direction (c1[0] near c2[0]); stitch by arc-length fraction.
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
  if (circleEdges.length !== 1) return false;
  const oe0 = circleEdges[0]!;
  const s0 = sampled.get(oe0.edgeId)!;
  let base: Vec3[] | null = oe0.orient ? s0.slice() : s0.slice().reverse();
  if (!base || base.length < 4) return false;
  base = base.slice(0, base.length - 1); // drop duplicate closing point (keep index 0 = angular start)
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
  const rimEdge = brep.edges.get(oe0.edgeId)!;
  const closedCircle = Math.hypot(rimEdge.v0[0] - rimEdge.v1[0], rimEdge.v0[1] - rimEdge.v1[1], rimEdge.v0[2] - rimEdge.v1[2]) < 1e-6;
  const soleRim = nFaceLoops === 1 && loop.edges.length === 1 && closedCircle;
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
    const M = Math.max(3, Math.min(4000, Math.round((TWO_PI * rf) / target)));
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
  const arcs: number[] = [], seams: number[] = [];
  for (const oe of loop.edges) {
    const e = brep.edges.get(oe.edgeId); if (!e) return false;
    const kind = brep.table.typeOf(e.curveId);
    if (kind === "CIRCLE") arcs.push(oe.edgeId);
    else if (kind === "LINE") seams.push(oe.edgeId);
    else return false; // some other boundary curve — not a clean cone slice
  }
  if (arcs.length !== 1 || seams.length !== 2) return false;
  const arcEdge = brep.edges.get(arcs[0]!)!;
  if (d3(arcEdge.v0, arcEdge.v1) < 1e-9) return false; // full circle -> tessellateCone / band
  // Both seams must run to the apex.
  const apexTol = 1e-6 * Math.max(1, d3(apex, arcEdge.v0));
  const seamCol = (id: number): Vec3[] | null => {
    const s = sampled.get(id); if (!s || s.length < 2) return null;
    const col = d3(s[0]!, apex) <= d3(s[s.length - 1]!, apex) ? s.slice() : s.slice().reverse();
    return d3(col[0]!, apex) <= apexTol ? col : null; // must start AT the apex
  };
  let Lc = seamCol(seams[0]!), Rc = seamCol(seams[1]!);
  if (!Lc || !Rc || Lc.length !== Rc.length) return false; // asymmetric sampling -> param grid
  const n = Lc.length;
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
function tessellateSphere(
  s: Sphere, fid: number, chordTol: number, targetEdge: number, sign: number, verts: number[], faceIds: number[],
): void {
  const R = Math.max(s.r, 1e-9);
  const dChord = 2 * Math.acos(Math.max(0, Math.min(1, 1 - chordTol / R)));
  const dEdge = targetEdge / R;
  const dTheta = Math.max(1e-4, Math.min(dChord, dEdge));
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
  verts: number[], faceIds: number[], chordTol: number, targetEdge: number, sign: number,
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
  let du = 0;
  for (let i = 0; i < uv.length; i++) { let d = uv[(i + 1) % uv.length]![0] - uv[i]![0]; while (d > Math.PI) d -= TWO_PI; while (d < -Math.PI) d += TWO_PI; du += d; }
  const vPole = du >= 0 ? Math.PI / 2 : -Math.PI / 2;

  const R = Math.max(s.r, 1e-9);
  const dChord = 2 * Math.acos(Math.max(0, Math.min(1, 1 - chordTol / R)));
  const dTheta = Math.max(1e-4, Math.min(dChord, targetEdge / R));
  const target = R * dTheta;
  const span = Math.abs(vPole - vRim);
  const nV = Math.max(1, Math.min(2000, Math.ceil(span / dTheta)));
  // Build the rim ring from the SHARED edge samples (watertight with the neighbour), then march
  // latitude rings to the pole, each sized to its own circumference so the cap tapers to a point.
  let prev = rim.slice();
  for (let j = 1; j <= nV; j++) {
    const v = vRim + ((vPole - vRim) * j) / nV;
    if (j === nV) { stitchRings(verts, faceIds, prev, [s.evaluate(0, vPole)], fid, s, sign); break; }
    const circ = TWO_PI * R * Math.cos(v);
    const nu = Math.max(3, Math.min(4000, Math.round(circ / target)));
    const ring: Vec3[] = [];
    for (let i = 0; i < nu; i++) ring.push(s.evaluate((TWO_PI * i) / nu, v));
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
  const Rc = s.curvatureRadius();
  const target = Number.isFinite(Rc)
    ? Math.max(targetEdge / 40, Math.min(targetEdge, Math.sqrt(8 * Rc * chordTol), Rc * normalDev)) : targetEdge;
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
  const skipped: Record<string, number> = {};
  let facesTotal = 0;
  let facesTessellated = 0;

  // Cache each face's surface (used to dispatch tessellation).
  const faceSurf = new Map<number, Surface | null>();
  for (const solid of brep.solids) for (const face of solid.faces) {
    faceSurf.set(face.faceId, makeSurface(brep.table, face.surfaceId, brep.scale, brep.units.radPerAngle));
  }
  // Sample each edge to the FINEST interior target of its adjacent faces — not just its own curve
  // curvature. A curved face's straight seam / side edges (lines carry no curvature, so they'd be
  // sampled at targetEdge) otherwise stay far coarser than the fine interior and sliver the seam.
  // Still one shared sampling per edge, so seams remain watertight; any residual sliver lands on a
  // flat neighbour (where it's invisible) rather than on the curved face.
  const edgeMaxLen = new Map<number, number>();
  for (const solid of brep.solids) for (const face of solid.faces) {
    const surface = faceSurf.get(face.faceId);
    const t = surface ? faceTarget(surface, targetEdge, chordTol, normalDev) : targetEdge;
    for (const lp of face.loops) for (const oe of lp.edges) {
      const cur = edgeMaxLen.get(oe.edgeId);
      if (cur === undefined || t < cur) edgeMaxLen.set(oe.edgeId, t);
    }
  }
  const sampled = new Map<number, Vec3[]>();
  for (const [id, e] of brep.edges) {
    const te = edgeMaxLen.get(id) ?? targetEdge;
    sampled.set(id, sampleEdgePolyline(brep.table, e.curveId, e.v0, e.v1, e.sameSense, brep.scale, chordTol, te, brep.units.radPerAngle));
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
      const surface = faceSurf.get(face.faceId) ?? null;
      if (!surface) { bump(skipped, face.surfaceKind); continue; }
      const sign = face.sameSense ? 1 : -1;

      let ok = false;
      const outer = face.loops.find((l) => l.outer) ?? face.loops[0];
      if (outer && face.loops.length === 1 && solid.faces.length > 1
        && tessellateThinFace(surface, outer, sampled, face.faceId, verts, faceIds, sign, 0.005)) {
        ok = true; // degenerate sub-resolution sliver/crack ribbon-stitched (returns false if not thin)
      } else if (isSphere(surface)) {
        // A full sphere is its solid's only face (degenerate seam loop); trimmed spheres
        // (e.g. roundedCube corners) are one of many faces -> param grid.
        if (solid.faces.length === 1) {
          tessellateSphere(surface, face.faceId, chordTol, targetEdge, sign, verts, faceIds);
          ok = true;
        } else if (outer) {
          // A cap closing to a pole (sole full-longitude parallel rim) fans to the pole; any other
          // spherical patch returns false from the cap mesher and uses the param grid.
          ok = tessellateSphereCap(surface, outer, sampled, face.faceId, verts, faceIds, chordTol, targetEdge, sign)
            || tessellateParamGrid(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign);
        }
      } else if (surface.kind === "CONICAL_SURFACE" && outer
        && (tessellateCone(surface, outer, sampled, brep, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign, face.loops.length)
          || tessellateConeSlice(surface, outer, sampled, brep, face.faceId, verts, faceIds, sign))) {
        ok = true; // genuine apex cone or apex wedge slice; frustums/trimmed cones use the param grid
      } else if (isBSpline(surface)) {
        // A standalone closed B-spline body (its solid's only face) has no usable trimming loop ->
        // full-patch grid. A patch that is one of many faces must use the param grid so its boundary
        // uses the SHARED edge samples (independent grids would crack against their neighbours).
        ok = (solid.faces.length === 1 && (surface.closedU || surface.closedV))
          ? tessellateBSplineFull(surface, face.faceId, chordTol, targetEdge, normalDev, sign, verts, faceIds)
          : (!!outer && (tessellateParamGrid(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)
            || tessellateRibbon(surface, face.loops, sampled, face.faceId, verts, faceIds, sign)));
      } else if (outer) {
        // Cylinders, cone frustums, tori, etc. Three meshers, tried in order:
        //  1. band: rims are bare full-period circles (no seam edges, e.g. Onshape) with NO other
        //     loops -> ribbon-stitch the rims directly. Bails on anything else.
        //  2. unroll: bare full-period rims PLUS window holes -> seam-split into a rectangular (u,v)
        //     domain and CDT with the windows as holes. Bails unless there are exactly two rims.
        //  3. param grid: everything with a proper seam-bounded outer loop (the common case).
        const periodic = surface.periodicU || surface.periodicV;
        ok = (periodic && tessellateRevolutionBand(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign))
          || (periodic && tessellatePeriodicUnroll(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign))
          || tessellateParamGrid(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign)
          || tessellateRibbon(surface, face.loops, sampled, face.faceId, verts, faceIds, sign);
      }
      if (ok) facesTessellated++; else bump(skipped, "untriangulated");
    }

    const { mesh } = weld(verts);
    const z = zipSlivers(mesh, 0.05);
    const keptFaceIds: number[] = [];
    for (let t = 0; t < faceIds.length; t++) if (z.keep[t]) keptFaceIds.push(faceIds[t]!);
    const fill = fillMicroHoles(z.positions, z.indices, keptFaceIds, 0.05);
    for (const x of z.positions) positions.push(x);
    for (const ix of z.indices) indices.push(ix + voff);
    for (const ix of fill.indices) indices.push(ix + voff);
    voff += z.positions.length / 3;
    for (const f of keptFaceIds) { faceOfTri.push(f); solidOfTri.push(solid.id); }
    for (const f of fill.faceOf) { faceOfTri.push(f); solidOfTri.push(solid.id); }
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
    stats: { solids: brep.solids.length, facesTotal, facesTessellated, skipped },
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
    // Absolute 3 µm floor on the cap: at a DEGENERATE tip (a B-spline sliver tapering to a pole)
    // the rail segments themselves shrink to microns, so half-a-segment blocks the very zip the
    // crack needs. No real CAD feature lives at 3 µm, so the floor cannot weld distinct geometry.
    const cap = Math.min(tol, Math.max(0.5 * (vCap.get(v) ?? Infinity), 3e-3));
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
 * Fill micro-holes left at degenerate tips: an open-edge loop whose whole perimeter is below the
 * sliver tolerance is a pinhole (a few µm-scale triangles collapsed at a B-spline pole/tip, whose
 * ring vertices are all mutually edge-connected, so the vertex zip can't close it) — not a real
 * gap. Fan it shut, wound opposite the ring so it pairs manifold-consistently with the surrounding
 * triangles. Real openings have perimeters orders of magnitude larger and are left alone.
 */
function fillMicroHoles(P: Float64Array, I: Uint32Array, faceOf: number[], tol: number): { indices: number[]; faceOf: number[] } {
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
  const outI: number[] = [], outF: number[] = [];
  const seen = new Set<number>();
  for (const start of nxt.keys()) {
    if (seen.has(start)) continue;
    const ring: number[] = [];
    let cur = start, per = 0, ok = true;
    for (let g = 0; g <= 64; g++) {
      ring.push(cur); seen.add(cur);
      const n = nxt.get(cur);
      if (n === undefined) { ok = false; break; }
      per += Math.hypot(P[cur * 3]! - P[n * 3]!, P[cur * 3 + 1]! - P[n * 3 + 1]!, P[cur * 3 + 2]! - P[n * 3 + 2]!);
      if (per > tol) { ok = false; break; }
      cur = n;
      if (cur === start) break;
      if (g === 64 || seen.has(cur)) { ok = false; break; }
    }
    if (!ok || cur !== start || ring.length < 3 || ring.some((v) => multi.has(v))) continue;
    // Fan, reversed relative to the directed ring so each ring edge is paired b->a.
    for (let i = 1; i + 1 < ring.length; i++) { outI.push(ring[0]!, ring[i + 1]!, ring[i]!); outF.push(faceAt.get(ring[0]!)!); }
  }
  return { indices: outI, faceOf: outF };
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
