// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — mapped (ruled-strip) mesher for single-curved four-sided faces (isotropic mode).
//
// WHY. A fillet bends across and runs straight (or nearly so) along. Mesh it with free
// equilateral triangles and every facet tilts a little differently against that straight
// direction: the faceted fillet is no longer a clean prism but a field of tiny alternating
// bends. A slicer cuts those into layer contours that wobble along the fillet, and the wobble
// shows on the printed part. (Shaded, the same thing reads as "diamond moire"; the default mode
// avoids it with ruling-aligned stretched triangles.)
//
// WHAT. The remedy an FE pre-processor calls a MAPPED mesh: a structured mesh that follows the
// face's two directions. Here it is built from RULINGS — lines along the flat direction, one
// through every node of the two CROSS sides (the arcs across the bend) — and every triangle
// spans exactly two neighbouring rulings. On a straight fillet the strip between two rulings is
// plane, so all its triangles are coplanar and the mesh is exactly the prism: creases only
// along rulings, none across. On a fillet with a curved spine the strips are as flat as that
// geometry allows.
//
// HOW it stays close to the isotropic look:
//   - The nodes ALONG each ruling are free. Next to a rail they follow that rail's spacing (so
//     two rails with different node counts are fine); further in they settle at a spacing that
//     suits the strip width. Every other ruling is shifted by half a step, so the triangles
//     come out isosceles and near-equilateral (~55-65 degrees) instead of the 45/45/90 halves of
//     a plain quad grid.
//   - Only the two cross sides must agree: same node count, node i of one joined to node i of
//     the other. The edge seeder is told to give such pairs equal counts.
//
// Node positions come from transfinite interpolation of the four boundary curves in the
// parameter plane, evaluated on the surface, so interior nodes lie exactly on the surface and
// the boundary nodes are the shared edge samples verbatim (watertight by construction).
//
// The function only builds the mesh. Whether a face qualifies (four sides, single-curved) is
// decided by the caller with the helpers below; anything that does not fit is declined and
// meshed freely.
import type { Vec3 } from "../geom/vec.ts";
import type { Surface } from "../geom/surfaces.ts";

type P2 = [number, number];

/** One side of the four-sided domain: its boundary samples in order, first and last being the
 * corners it shares with its neighbours. */
export interface MappedSide { p2: P2[]; p3: Vec3[] }

export interface MappedMesh {
  p2: P2[];
  p3: Vec3[];
  /** Indices into p2/p3. Winding is not normalised; the caller orients by the surface normal. */
  tris: [number, number, number][];
}

/**
 * Worst normal curvature along each iso-parameter direction over a 3x3 lattice of the box (the
 * same probe the default grid mesher uses for its fillet stretch). A true fillet is curved along
 * one parameter and flat along the other at every sample; a freeform patch that bends both ways
 * somewhere reports both.
 */
export function isoCurvature(surface: Surface, umin: number, umax: number, vmin: number, vmax: number): { kU: number; kV: number; mU: number; mV: number } {
  const epsU = Math.max(1e-6, (umax - umin) / 64), epsV = Math.max(1e-6, (vmax - vmin) / 64);
  let kU = 0, kV = 0;
  const uM: number[] = [], vM: number[] = [];
  for (const fu of [0.15, 0.5, 0.85]) for (const fv of [0.15, 0.5, 0.85]) {
    const su = umin + (umax - umin) * fu, sv = vmin + (vmax - vmin) * fv;
    const Pc = surface.evaluate(su, sv);
    const uP = surface.evaluate(su + epsU, sv), uN = surface.evaluate(su - epsU, sv);
    const vP = surface.evaluate(su, sv + epsV), vN = surface.evaluate(su, sv - epsV);
    const d1u = [(uP[0] - uN[0]) / (2 * epsU), (uP[1] - uN[1]) / (2 * epsU), (uP[2] - uN[2]) / (2 * epsU)];
    const d1v = [(vP[0] - vN[0]) / (2 * epsV), (vP[1] - vN[1]) / (2 * epsV), (vP[2] - vN[2]) / (2 * epsV)];
    const lu = Math.hypot(d1u[0]!, d1u[1]!, d1u[2]!), lv = Math.hypot(d1v[0]!, d1v[1]!, d1v[2]!);
    uM.push(lu); vM.push(lv);
    const n = [d1u[1]! * d1v[2]! - d1u[2]! * d1v[1]!, d1u[2]! * d1v[0]! - d1u[0]! * d1v[2]!, d1u[0]! * d1v[1]! - d1u[1]! * d1v[0]!];
    const nl = Math.hypot(n[0]!, n[1]!, n[2]!);
    if (!(nl > 1e-30)) continue;
    const curv = (Pp: Vec3, Pn: Vec3, l: number, e: number): number => {
      const d2 = [(Pp[0] + Pn[0] - 2 * Pc[0]) / (e * e), (Pp[1] + Pn[1] - 2 * Pc[1]) / (e * e), (Pp[2] + Pn[2] - 2 * Pc[2]) / (e * e)];
      return Math.abs(d2[0]! * n[0]! + d2[1]! * n[1]! + d2[2]! * n[2]!) / nl / Math.max(l * l, 1e-30);
    };
    kU = Math.max(kU, curv(uP, uN, lu, epsU));
    kV = Math.max(kV, curv(vP, vN, lv, epsV));
  }
  const med = (a: number[]): number => { const s = [...a].sort((x, y) => x - y); return s[s.length >> 1]!; };
  return { kU, kV, mU: Math.max(1e-9, med(uM)), mV: Math.max(1e-9, med(vM)) };
}

/** Cumulative 3D arc-length fraction (0..1) at each sample of a polyline. */
function fractions(p3: Vec3[]): number[] {
  const t = [0];
  for (let i = 1; i < p3.length; i++) t.push(t[i - 1]! + Math.hypot(p3[i]![0] - p3[i - 1]![0], p3[i]![1] - p3[i - 1]![1], p3[i]![2] - p3[i - 1]![2]));
  const L = t[t.length - 1]!;
  if (!(L > 0)) return t.map((_, i) => i / Math.max(1, t.length - 1));
  return t.map((x) => x / L);
}

/** Value of the piecewise-linear function through (knots[i], vals[i]) at x. */
function interp(knots: number[], vals: P2[], x: number): P2 {
  let lo = 0, hi = knots.length - 1;
  if (x <= knots[0]!) return vals[0]!;
  if (x >= knots[hi]!) return vals[hi]!;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (knots[m]! <= x) lo = m; else hi = m; }
  const d = knots[hi]! - knots[lo]!;
  const w = d > 0 ? (x - knots[lo]!) / d : 0;
  return [vals[lo]![0] + (vals[hi]![0] - vals[lo]![0]) * w, vals[lo]![1] + (vals[hi]![1] - vals[lo]![1]) * w];
}

/**
 * Mapped mesh of the four-sided domain bounded by crossA, rail1, crossB, rail0 — given as
 *
 *   crossA : corner P00 -> P10      (N segments, across the bend)
 *   crossB : corner P01 -> P11      (N segments, same direction as crossA)
 *   rail0  : corner P00 -> P01      (along the flat direction, at the start of the cross sides)
 *   rail1  : corner P10 -> P11      (at their end)
 *
 * `mU`, `mV` scale the parameter plane to millimetres for the validity test. Returns null when
 * the cross sides disagree in node count or the interpolated grid is not a valid (fold-free)
 * triangulation of the domain.
 */
export function mappedMesh(
  surface: Surface, crossA: MappedSide, crossB: MappedSide, rail0: MappedSide, rail1: MappedSide, mU: number, mV: number,
): MappedMesh | null {
  const N = crossA.p2.length - 1;
  if (N < 1 || crossB.p2.length - 1 !== N) return null;
  const M0 = rail0.p2.length - 1, M1 = rail1.p2.length - 1;
  if (M0 < 1 || M1 < 1) return null;
  const t0 = fractions(rail0.p3), t1 = fractions(rail1.p3);
  const P00 = crossA.p2[0]!, P10 = crossA.p2[N]!, P01 = crossB.p2[0]!, P11 = crossB.p2[N]!;
  // NODE SPACING ALONG THE RULINGS. Next to a rail a ruling follows that rail's spacing, so the
  // rail's fixed (shared) nodes pair up with their first neighbours. Further in, the spacing
  // fades GROW-fold per ruling toward what suits the strip itself: between 0.8 and ASPECT times
  // its width. So a rail graded toward a fine corner does not stamp that grading onto every
  // ruling, and strips much narrower than the rails' spacing are refined along instead of
  // carrying needle-long triangles. Up to ASPECT the triangles simply stay a little long along
  // the fillet: that costs nothing in print quality (the strip is flat that way) and saves the
  // triangles a full refinement would add.
  const GROW = 1.5, ASPECT = 1.6, SHORT = 0.8;
  const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const len = (p: Vec3[]): number => { let L = 0; for (let i = 1; i < p.length; i++) L += d3(p[i]!, p[i - 1]!); return L; };
  const L0 = len(rail0.p3), L1 = len(rail1.p3);
  /** A rail's local node spacing at fraction x of its length (linear between segment middles). */
  const spacing = (t: number[], L: number, x: number): number => {
    const m = t.length - 1;
    let lo = 0, hi = m;
    while (hi - lo > 1) { const k = (lo + hi) >> 1; if (t[k]! <= x) lo = k; else hi = k; }
    const seg = (k: number): number => (t[k + 1]! - t[k]!) * L, mid = (k: number): number => (t[k]! + t[k + 1]!) / 2;
    const k2 = x < mid(lo) ? lo - 1 : lo;
    if (k2 < 0) return seg(0);
    if (k2 >= m - 1) return seg(m - 1);
    const w = (x - mid(k2)) / Math.max(mid(k2 + 1) - mid(k2), 1e-300);
    return seg(k2) + (seg(k2 + 1) - seg(k2)) * Math.min(1, Math.max(0, w));
  };
  const Q = Math.min(4096, 8 * Math.max(M0, M1) + 16); // integration steps along a ruling
  /** Parameter point of ruling i (at w = i/N across) at fraction tt along it: transfinite
   * interpolation of the four boundary curves (a Coons patch) in (u,v). */
  const uvAt = (w: number, tt: number, i: number): P2 => {
    const A = crossA.p2[i]!, B = crossB.p2[i]!;
    const r0 = interp(t0, rail0.p2, tt), r1 = interp(t1, rail1.p2, tt);
    return [
      (1 - w) * r0[0] + w * r1[0] + (1 - tt) * A[0] + tt * B[0] - ((1 - w) * (1 - tt) * P00[0] + w * (1 - tt) * P10[0] + (1 - w) * tt * P01[0] + w * tt * P11[0]),
      (1 - w) * r0[1] + w * r1[1] + (1 - tt) * A[1] + tt * B[1] - ((1 - w) * (1 - tt) * P00[1] + w * (1 - tt) * P10[1] + (1 - w) * tt * P01[1] + w * tt * P11[1]),
    ];
  };
  const at = (w: number, tt: number, i: number): Vec3 => { const q = uvAt(w, tt, i); return surface.evaluate(q[0], q[1]); };
  /** Fractions (0..1, ends excluded) of the nodes on ruling i; `half` shifts them by half a step. */
  const nodesOn = (i: number, half: boolean): number[] => {
    const w = i / N;
    // The ruling's own 3D length, measured along it: a face whose cross sides approach each
    // other in the middle (a pocket wall that gets shallower) has rulings far shorter than
    // either rail there.
    let L = 0;
    { let prev = crossA.p3[i]!; for (let k = 1; k <= 12; k++) { const q = k === 12 ? crossB.p3[i]! : at(w, k / 12, i); L += d3(prev, q); prev = q; } }
    const width = (d3(crossA.p3[i]!, crossA.p3[i - 1]!) + d3(crossA.p3[i + 1]!, crossA.p3[i]!) + d3(crossB.p3[i]!, crossB.p3[i - 1]!) + d3(crossB.p3[i + 1]!, crossB.p3[i]!)) / 4;
    const fade = 1 - 1 / GROW ** Math.min(i, N - i);
    const size = (x: number): number => {
      const hb = (1 - w) * spacing(t0, L0, x) + w * spacing(t1, L1, x);
      const own = Math.min(ASPECT * width, Math.max(SHORT * width, hb));
      return Math.max(1e-12, hb * (own / hb) ** fade);
    };
    // Equal steps of the integral of 1/size, exactly as the edge seeder places nodes on an edge.
    const A = new Float64Array(Q + 1);
    let prev = 1 / size(0);
    for (let q = 1; q <= Q; q++) { const cur = 1 / size(q / Q); A[q] = A[q - 1]! + (0.5 * (prev + cur) * L) / Q; prev = cur; }
    const M = Math.max(1, Math.round(A[Q]!));
    const out: number[] = [];
    const shift = half && M >= 2;
    let j = 1;
    for (let k = 0; k < (shift ? M : M - 1); k++) {
      const target = (A[Q]! * (shift ? k + 0.5 : k + 1)) / M;
      while (j < Q && A[j]! < target) j++;
      const da = A[j]! - A[j - 1]!;
      out.push((j - 1 + (da > 0 ? (target - A[j - 1]!) / da : 0)) / Q);
    }
    return out;
  };
  const p2: P2[] = [], p3: Vec3[] = [];
  const rulings: { idx: number[]; tau: number[] }[] = [];
  const add = (q: P2, p: Vec3): number => { p2.push(q); p3.push(p); return p2.length - 1; };
  // Rulings 0 and N are the rails themselves: the shared boundary samples, verbatim.
  const railRuling = (r: MappedSide, t: number[]): { idx: number[]; tau: number[] } => ({ idx: r.p2.map((q, k) => add(q, r.p3[k]!)), tau: t });
  rulings.push(railRuling(rail0, t0));
  for (let i = 1; i < N; i++) {
    const w = i / N;
    const A = crossA.p2[i]!, B = crossB.p2[i]!;
    const idx: number[] = [add(A, crossA.p3[i]!)], tau: number[] = [0];
    // Every other ruling is shifted by half a step, which turns the quad cells into
    // near-equilateral triangles; a ruling too short for two nodes keeps the plain layout.
    for (const tt of nodesOn(i, i % 2 === 1)) {
      const q = uvAt(w, tt, i);
      idx.push(add(q, surface.evaluate(q[0], q[1]))); tau.push(tt);
    }
    idx.push(add(B, crossB.p3[i]!)); tau.push(1);
    rulings.push({ idx, tau });
  }
  rulings.push(railRuling(rail1, t1));

  // Zip each pair of neighbouring rulings, always taking the SHORTER of the two possible next
  // diagonals (measured in 3D). Where the cross sides meet the rulings squarely this pairs each
  // node with the half-step-shifted nodes opposite it; where they meet them at a slant (a wall
  // whose top edge runs obliquely) the strip is a sheared ribbon, and following the parameter
  // fractions instead would lean every triangle over into a sliver.
  const tris: [number, number, number][] = [];
  for (let i = 0; i < N; i++) {
    const L = rulings[i]!, R = rulings[i + 1]!;
    let l = 0, r = 0;
    const nl = L.idx.length - 1, nr = R.idx.length - 1;
    while (l < nl || r < nr) {
      const advL = r >= nr || (l < nl && d3(p3[L.idx[l + 1]!]!, p3[R.idx[r]!]!) <= d3(p3[L.idx[l]!]!, p3[R.idx[r + 1]!]!));
      if (advL) { tris.push([L.idx[l]!, R.idx[r]!, L.idx[l + 1]!]); l++; }
      else { tris.push([L.idx[l]!, R.idx[r]!, R.idx[r + 1]!]); r++; }
    }
  }
  // Validity: a correct mesh of the domain is consistently wound in the parameter plane with no
  // collapsed triangle. A skewed or self-overlapping boundary quadrilateral (where the Coons
  // patch folds) fails here and the face goes to the free mesher instead.
  let pos = 0, neg = 0, total = 0, tiny = 0;
  const areas: number[] = [];
  for (const [a, b, c] of tris) {
    const A = p2[a]!, B = p2[b]!, C = p2[c]!;
    const ar = ((B[0] - A[0]) * mU) * ((C[1] - A[1]) * mV) - ((B[1] - A[1]) * mV) * ((C[0] - A[0]) * mU);
    areas.push(ar); total += Math.abs(ar);
    if (ar > 0) pos++; else if (ar < 0) neg++;
  }
  if (pos > 0 && neg > 0) return null;
  const mean = total / Math.max(1, tris.length);
  for (const ar of areas) if (Math.abs(ar) < 1e-6 * mean) tiny++;
  if (tiny > 0 || tris.length === 0) return null;
  return { p2, p3, tris };
}
