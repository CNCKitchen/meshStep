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

function loopParam(surface: Surface, loop: BLoop, sampled: Map<number, Vec3[]>): { p3: Vec3[]; p2: P2[] } {
  const p3: Vec3[] = [];
  for (const oe of loop.edges) {
    const base = sampled.get(oe.edgeId);
    if (!base) continue;
    const poly = oe.orient ? base : base.slice().reverse();
    for (let i = 0; i < poly.length - 1; i++) p3.push(poly[i]!);
  }
  // Project each boundary point seeded from the previous one's (u,v) so the boundary stays
  // continuous in parameter space — across a seam too (the B-spline solver isn't clamped there). If
  // the hint sends the solver astray (residual to the surface stays large, e.g. a coarse seed grid
  // on a thin patch or a corner) fall back to the stateless nearest-grid projection. Analytic
  // surfaces ignore the hint and land exactly, so they never fall back.
  const resid = (q: P2, pt: Vec3): number => { const e = surface.evaluate(q[0], q[1]); return Math.hypot(e[0] - pt[0], e[1] - pt[1], e[2] - pt[2]); };
  const p2: P2[] = [];
  let hu: number | undefined, hv: number | undefined;
  for (const pt of p3) {
    let q = surface.project(pt, hu, hv);
    if (resid(q, pt) > 1e-3) { const g = surface.project(pt); if (resid(g, pt) < resid(q, pt)) q = g; }
    p2.push(q); hu = q[0]; hv = q[1];
  }
  if (surface.periodicU) unwrap(p2, 0, surface.uPeriod || TWO_PI);
  if (surface.periodicV) unwrap(p2, 1, surface.vPeriod || TWO_PI);
  return { p3, p2 };
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
 * so curved faces are finely sampled, by targetEdge on flat ones, with a floor to bound density. */
function faceTarget(surface: Surface, targetEdge: number, chordTol: number, normalDev: number, u = 0, v = 0): number {
  const Rc = surface.curvatureRadius(u, v);
  return Number.isFinite(Rc)
    ? Math.max(targetEdge / 40, Math.min(targetEdge, Math.sqrt(8 * Rc * chordTol), Rc * normalDev))
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
  const projected = loops.map((l) => ({ outer: l.outer, lp: loopParam(surface, l, sampled) }));
  let oi = projected.findIndex((p) => p.outer);
  if (oi < 0) {
    let best = -Infinity;
    for (let i = 0; i < projected.length; i++) { const a = Math.abs(polyArea(projected[i]!.lp.p2)); if (a > best) { best = a; oi = i; } }
  }
  if (oi < 0) return false;
  const outer = projected[oi]!.lp;
  if (outer.p3.length < 3) return false;

  let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
  for (const q of outer.p2) {
    if (q[0] < umin) umin = q[0]; if (q[0] > umax) umax = q[0];
    if (q[1] < vmin) vmin = q[1]; if (q[1] > vmax) vmax = q[1];
  }
  const holes = projected.filter((_, i) => i !== oi).map((p) => p.lp);
  if (surface.periodicU) for (const h of holes) shiftIntoRange(h.p2, (umin + umax) / 2, 0, surface.uPeriod || TWO_PI);
  if (surface.periodicV) for (const h of holes) shiftIntoRange(h.p2, (vmin + vmax) / 2, 1, surface.vPeriod || TWO_PI);
  const holeP2 = holes.map((h) => h.p2);
  const inRegion = (p: P2): boolean => pointInPoly(p, outer.p2) && !holeP2.some((h) => pointInPoly(p, h));

  const umid = (umin + umax) / 2, vmid = (vmin + vmax) / 2, eps = 1e-3;
  const dU = surface.evaluate(umid + eps, vmid), dU2 = surface.evaluate(umid - eps, vmid);
  const dV = surface.evaluate(umid, vmid + eps), dV2 = surface.evaluate(umid, vmid - eps);
  const uScale = Math.max(1e-9, Math.hypot(dU[0] - dU2[0], dU[1] - dU2[1], dU[2] - dU2[2]) / (2 * eps));
  const vScale = Math.max(1e-9, Math.hypot(dV[0] - dV2[0], dV[1] - dV2[1], dV[2] - dV2[2]) / (2 * eps));
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
      const target = faceTarget(surface, targetEdge, chordTol, normalDev, c === 0 ? angs[0]! : (h0 + h1) / 2, c === 0 ? (h0 + h1) / 2 : angs[0]!);
      // Ring count from the LONGEST stack traverse over a few sample angles (a torus elbow's outer
      // side is much longer than its inner side).
      let span = 0;
      for (let j = 0; j < angs.length; j += Math.max(1, angs.length >> 3)) {
        span = Math.max(span, d3(bottom[j]!, evalAt(c, angs[j]!, h1)));
      }
      const nRings = Math.max(1, Math.min(400, Math.ceil(span / target)));
      let prev = bottom;
      for (let k = 1; k < nRings; k++) {
        const hk = h0 + band.width * (k / nRings);
        const ring = angs.map((a) => evalAt(c, a, hk));
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

/** Cone: stack concentric rings from the shared base circle to the apex (cone is ruled by lines). */
function tessellateCone(
  surface: Surface, loop: BLoop, sampled: Map<number, Vec3[]>, brep: BrepModel, fid: number,
  verts: number[], faceIds: number[], targetEdge: number, chordTol: number, normalDev: number, sign: number,
  nFaceLoops: number,
): boolean {
  // Only a genuine apex cone (exactly one circle rim, tapering to a point) is handled here; a
  // frustum / trimmed cone has 2+ circle rims or straight sides and must go through the param grid
  // (which uses the shared edge samples on every side, so it stays watertight with its neighbours).
  // A rim is a CIRCLE edge or any closed sampled loop (e.g. an INTERSECTION_CURVE wrapping a
  // circle in legacy AP203 files) — the curve TYPE doesn't matter, the closed ring does.
  const circleEdges = loop.edges.filter((oe) => {
    const e = brep.edges.get(oe.edgeId);
    if (!e) return false;
    if (brep.table.typeOf(e.curveId) === "CIRCLE") return true;
    const p = sampled.get(oe.edgeId);
    return !!p && p.length >= 4
      && Math.hypot(e.v0[0] - e.v1[0], e.v0[1] - e.v1[1], e.v0[2] - e.v1[2]) < 1e-9;
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
  const rim: Vec3[] = [];
  for (const oe of loop.edges) {
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
        && tessellateCone(surface, outer, sampled, brep, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign, face.loops.length)) {
        ok = true; // genuine apex cone (singular vertex); frustums/trimmed cones use the param grid
      } else if (isBSpline(surface)) {
        // A standalone closed B-spline body (its solid's only face) has no usable trimming loop ->
        // full-patch grid. A patch that is one of many faces must use the param grid so its boundary
        // uses the SHARED edge samples (independent grids would crack against their neighbours).
        ok = (solid.faces.length === 1 && (surface.closedU || surface.closedV))
          ? tessellateBSplineFull(surface, face.faceId, chordTol, targetEdge, normalDev, sign, verts, faceIds)
          : (!!outer && tessellateParamGrid(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign));
      } else if (outer) {
        // Cylinders, cone frustums, etc. A full-revolution band whose rims are bare full-period
        // circles (no seam edges, e.g. Onshape) can't be meshed by the param grid -> ribbon-stitch
        // the rims; the band mesher returns false (and emits nothing) for any other case.
        ok = (surface.periodicU && tessellateRevolutionBand(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign))
          || tessellateParamGrid(surface, face.loops, sampled, face.faceId, verts, faceIds, targetEdge, chordTol, normalDev, sign);
      }
      if (ok) facesTessellated++; else bump(skipped, "untriangulated");
    }

    const { mesh } = weld(verts);
    const z = zipSlivers(mesh, 0.05);
    for (const x of z.positions) positions.push(x);
    for (const ix of z.indices) indices.push(ix + voff);
    voff += z.positions.length / 3;
    for (let t = 0; t < faceIds.length; t++) if (z.keep[t]) { faceOfTri.push(faceIds[t]!); solidOfTri.push(solid.id); }
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
    const cap = Math.min(tol, 0.5 * (vCap.get(v) ?? Infinity));
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
