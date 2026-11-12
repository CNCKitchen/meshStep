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
// HOW. Two layouts, chosen by the rails (the two sides that run along the flat direction):
//   - GRID, when both rails carry the same number of nodes (at a spacing that gives cells of a
//     reasonable shape). Node k of every ruling then sits on
//     the RIB that joins node k of one rail to node k of the other, so the face is a lattice of
//     quads, each with two sides along the fillet and two across it, cut into two triangles by
//     a diagonal. On a straight fillet the quads are plane, so the diagonal does not show at
//     all: the fillet is a clean prism of rectangles. The caller makes this the normal case by
//     seeding opposite rails alike (matchRailStations below).
//   - RULED otherwise: when the rails could not be given equal counts (one of them is fixed
//     by another face), are graded toward a corner much finer than the strips are wide, or
//     the face has only one rail or none (it ends in a slant or a point). The rulings then
//     carry common STATIONS wherever they reach, a grid trimmed by the boundary: rectangles
//     in the interior, triangles zipped in along a slanted end and beside a rail whose own
//     nodes do not sit at the stations.
//   - Either way the two cross sides must agree: same node count, node i of one joined to node
//     i of the other. The edge seeder is told to give such pairs equal counts.
//
// Interior nodes are placed in the parameter plane (transfinite interpolation of the four sides
// for the grid, straight rulings otherwise) and evaluated on the surface, so they lie exactly on
// it; the boundary nodes are the shared edge samples verbatim (watertight by construction).
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
 * somewhere reports both. `skew` is the largest |cosine| of the angle between the two parameter
 * directions over the same samples (0 where they cross at right angles).
 */
export function isoCurvature(surface: Surface, umin: number, umax: number, vmin: number, vmax: number): { kU: number; kV: number; mU: number; mV: number; skew: number } {
  const epsU = Math.max(1e-6, (umax - umin) / 64), epsV = Math.max(1e-6, (vmax - vmin) / 64);
  let kU = 0, kV = 0, skew = 0;
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
    if (lu > 0 && lv > 0) skew = Math.max(skew, Math.abs(d1u[0]! * d1v[0]! + d1u[1]! * d1v[1]! + d1u[2]! * d1v[2]!) / (lu * lv));
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
  return { kU, kV, mU: Math.max(1e-9, med(uM)), mV: Math.max(1e-9, med(vM)), skew };
}

/** Cumulative 3D arc-length fraction (0..1) at each sample of a polyline. */
function fractions(p3: Vec3[]): number[] {
  const t = [0];
  for (let i = 1; i < p3.length; i++) t.push(t[i - 1]! + Math.hypot(p3[i]![0] - p3[i - 1]![0], p3[i]![1] - p3[i - 1]![1], p3[i]![2] - p3[i - 1]![2]));
  const L = t[t.length - 1]!;
  if (!(L > 0)) return t.map((_, i) => i / Math.max(1, t.length - 1));
  return t.map((x) => x / L);
}

/** Largest side ratio a cell may have for a face to take the grid layout. */
const GRID_ASPECT = 2.2;

/**
 * Mapped mesh of a domain bounded by two cross sides and up to two rails — given as
 *
 *   crossA : corner P00 -> P10      (N segments, across the bend)
 *   crossB : corner P01 -> P11      (N segments, same direction as crossA)
 *   rail0  : corner P00 -> P01      (along the flat direction, at the start of the cross sides)
 *   rail1  : corner P10 -> P11      (at their end)
 *
 * A rail may be missing (null): the two cross sides then meet in a point at that end, as on a
 * face that ends in a slant. `C` is the index of the curved parameter (the one the cross sides
 * advance in), `mU`, `mV` scale the parameter plane to millimetres.
 *
 * Two rails with equal node counts, spaced so that no cell is longer or wider than GRID_ASPECT
 * to one, give the GRID layout; everything else the RULED layout (see the top of the file).
 * Returns null when the cross sides disagree in node count or the result is not a valid
 * (fold-free) triangulation of the domain.
 */
export function mappedMesh(
  surface: Surface, crossA: MappedSide, crossB: MappedSide, rail0: MappedSide | null, rail1: MappedSide | null, mU: number, mV: number, C: 0 | 1,
): MappedMesh | null {
  const N = crossA.p2.length - 1;
  if (N < 1 || crossB.p2.length - 1 !== N) return null;
  if ((rail0 && rail0.p2.length < 2) || (rail1 && rail1.p2.length < 2)) return null;
  const width = (polyLength(crossA.p3) + polyLength(crossB.p3)) / (2 * N);
  /** True when every cell of the rail is within GRID_ASPECT to one of the strip width. */
  const fits = (r: MappedSide): boolean => {
    for (let k = 0; k + 1 < r.p3.length; k++) {
      const cell = dist3(r.p3[k]!, r.p3[k + 1]!);
      if (cell * GRID_ASPECT < width || cell > GRID_ASPECT * width) return false;
    }
    return true;
  };
  // The grid repeats the rails' spacing on every ruling. That is only wanted while it gives
  // cells of a reasonable shape: a rail graded down to a fine corner would lay a row of
  // needle-thin cells across the whole face.
  if (rail0 && rail1 && rail0.p2.length === rail1.p2.length && fits(rail0) && fits(rail1)) {
    return gridMesh(surface, crossA, crossB, rail0, rail1, mU, mV);
  }
  return ruledMesh(surface, crossA, crossB, rail0, rail1, mU, mV, C, width, fits);
}

const dist3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
function polyLength(p: Vec3[]): number {
  let L = 0;
  for (let i = 1; i < p.length; i++) L += dist3(p[i]!, p[i - 1]!);
  return L;
}

/**
 * RULED layout: a grid trimmed by its boundary.
 *
 * Ruling i is the straight parameter line from node i of one cross side to node i of the other.
 * Its nodes sit at STATIONS, values of the flat parameter shared by all rulings, so that nodes
 * of neighbouring rulings face each other and the cells between them are rectangles; a ruling
 * simply carries the stations that fall inside it, and one too close to its end is dropped
 * (the end node takes its place, the cell there is up to a third longer or shorter). Where
 * two neighbouring rulings carry different stations — along a slanted end, or beside a rail
 * whose own nodes are elsewhere — the strip between them is zipped with triangles.
 *
 * The stations are the nodes of the face's one rail when it has exactly one and its spacing
 * suits the strips (the cells beside it are then rectangles too); otherwise they are evenly
 * spaced at the strip width, i.e. square cells.
 */
function ruledMesh(
  surface: Surface, crossA: MappedSide, crossB: MappedSide, rail0: MappedSide | null, rail1: MappedSide | null,
  mU: number, mV: number, C: 0 | 1, width: number, fits: (r: MappedSide) => boolean,
): MappedMesh | null {
  const N = crossA.p2.length - 1, F = (1 - C) as 0 | 1, mF = F === 0 ? mU : mV;
  let fMin = Infinity, fMax = -Infinity;
  for (const s of [crossA, crossB]) for (const q of s.p2) { if (q[F] < fMin) fMin = q[F]; if (q[F] > fMax) fMax = q[F]; }
  if (!(fMax > fMin) || !(width > 0)) return null;
  // Stations, ascending in the flat parameter, covering [fMin, fMax].
  let stations: number[];
  const guide = rail0 && !rail1 ? rail0 : rail1 && !rail0 ? rail1 : null;
  const gf = guide && fits(guide) ? guide.p2.map((q) => q[F]) : null;
  if (gf && gf.length >= 2 && gf.every((x, k) => k === 0 || (x - gf[k - 1]!) * (gf[gf.length - 1]! - gf[0]!) > 0)) {
    stations = gf[0]! < gf[gf.length - 1]! ? gf.slice() : gf.slice().reverse();
    // Continue with the end spacing where other rulings reach beyond the rail.
    const lo = stations[1]! - stations[0]!, hi = stations[stations.length - 1]! - stations[stations.length - 2]!;
    for (let k = 0; k < 4096 && stations[0]! > fMin; k++) stations.unshift(stations[0]! - lo);
    for (let k = 0; k < 4096 && stations[stations.length - 1]! < fMax; k++) stations.push(stations[stations.length - 1]! + hi);
  } else {
    // Even spacing in LENGTH, not in the parameter (a spline surface's parameter rarely runs
    // at constant speed): measured along the ruling that reaches farthest, and continued at
    // its end spacing where others reach beyond it.
    let ref = 0, reach = -1;
    for (let i = 0; i <= N; i++) { const d = Math.abs(crossB.p2[i]![F] - crossA.p2[i]![F]); if (d > reach) { reach = d; ref = i; } }
    const A = crossA.p2[ref]!, B = crossB.p2[ref]!;
    const Q = Math.min(4096, Math.max(16, Math.ceil((8 * reach * mF) / width)));
    const cum = new Float64Array(Q + 1);
    let prev = surface.evaluate(A[0], A[1]);
    for (let q = 1; q <= Q; q++) {
      const cur = surface.evaluate(A[0] + ((B[0] - A[0]) * q) / Q, A[1] + ((B[1] - A[1]) * q) / Q);
      cum[q] = cum[q - 1]! + dist3(prev, cur); prev = cur;
    }
    const K = Math.min(8192, Math.max(1, Math.round(cum[Q]! / width)));
    const fs: number[] = [A[F]];
    let j = 1;
    for (let k = 1; k < K; k++) {
      const target = (cum[Q]! * k) / K;
      while (j < Q && cum[j]! < target) j++;
      const d = cum[j]! - cum[j - 1]!;
      fs.push(A[F] + ((B[F] - A[F]) * (j - 1 + (d > 0 ? (target - cum[j - 1]!) / d : 0))) / Q);
    }
    fs.push(B[F]);
    stations = B[F] > A[F] ? fs : fs.reverse();
    if (stations.length >= 2 && stations[stations.length - 1]! > stations[0]!) {
      const lo = stations[1]! - stations[0]!, hi = stations[stations.length - 1]! - stations[stations.length - 2]!;
      for (let k = 0; k < 4096 && lo > 0 && stations[0]! > fMin; k++) stations.unshift(stations[0]! - lo);
      for (let k = 0; k < 4096 && hi > 0 && stations[stations.length - 1]! < fMax; k++) stations.push(stations[stations.length - 1]! + hi);
    } else {
      const K2 = Math.min(8192, Math.max(1, Math.round(((fMax - fMin) * mF) / width)));
      stations = Array.from({ length: K2 + 1 }, (_, k) => fMin + ((fMax - fMin) * k) / K2);
    }
  }
  const p2: P2[] = [], p3: Vec3[] = [];
  const add = (q: P2, p: Vec3): number => { p2.push(q); p3.push(p); return p2.length - 1; };
  const DROP = 0.35;
  const rulings: number[][] = [];
  for (let i = 0; i <= N; i++) {
    const rail = i === 0 ? rail0 : i === N ? rail1 : null;
    if (rail) { rulings.push(rail.p2.map((q, k) => add(q, rail.p3[k]!))); continue; }
    const A = crossA.p2[i]!, B = crossB.p2[i]!;
    const idx: number[] = [add(A, crossA.p3[i]!)];
    // A missing rail: the cross sides meet there, the ruling is that one point.
    if ((i === 0 || i === N) && dist3(crossA.p3[i]!, crossB.p3[i]!) < 1e-9 * (1 + width)) { rulings.push(idx); continue; }
    const fa = A[F], fb = B[F], lo = Math.min(fa, fb), hi = Math.max(fa, fb);
    if (hi > lo) {
      const ts: number[] = [];
      for (let k = 0; k < stations.length; k++) {
        const S = stations[k]!;
        const cell = Math.min(k > 0 ? S - stations[k - 1]! : Infinity, k + 1 < stations.length ? stations[k + 1]! - S : Infinity);
        if (S - lo > DROP * cell && hi - S > DROP * cell) ts.push((S - fa) / (fb - fa));
      }
      if (fb < fa) ts.reverse();
      for (const t of ts) {
        const q: P2 = [A[0] + (B[0] - A[0]) * t, A[1] + (B[1] - A[1]) * t];
        idx.push(add(q, surface.evaluate(q[0], q[1])));
      }
    }
    idx.push(add(B, crossB.p3[i]!));
    rulings.push(idx);
  }
  // Zip each pair of neighbouring rulings. Where both carry the same stations the two sides
  // advance in turn, which cuts every rectangle along the same diagonal; a side is advanced
  // out of turn only when its next diagonal is clearly the shorter in 3D, which is how the
  // triangles follow a slanted end or a rail with its own spacing.
  const tris: [number, number, number][] = [];
  for (let i = 0; i < N; i++) {
    const L = rulings[i]!, R = rulings[i + 1]!;
    let l = 0, r = 0, lastL = false;
    const nl = L.length - 1, nr = R.length - 1;
    while (l < nl || r < nr) {
      let advL: boolean;
      if (r >= nr) advL = true;
      else if (l >= nl) advL = false;
      else {
        const dL = dist3(p3[L[l + 1]!]!, p3[R[r]!]!), dR = dist3(p3[L[l]!]!, p3[R[r + 1]!]!);
        advL = lastL ? dL < 0.85 * dR : dL <= dR / 0.85;
      }
      if (advL) { tris.push([L[l]!, R[r]!, L[l + 1]!]); l++; }
      else { tris.push([L[l]!, R[r]!, R[r + 1]!]); r++; }
      lastL = advL;
    }
  }
  return valid(p2, p3, tris, mU, mV);
}

/** Validity: a correct mesh of the domain is consistently wound in the parameter plane with no
 * collapsed triangle. A skewed or self-overlapping boundary quadrilateral (where the
 * interpolated patch folds) fails here and the face goes to the free mesher instead. */
function valid(p2: P2[], p3: Vec3[], tris: [number, number, number][], mU: number, mV: number): MappedMesh | null {
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

/**
 * GRID layout: N strips by M cells, node (i, k) on ruling i and rib k.
 *
 * Positions are a transfinite interpolation of the four sides in the parameter plane with the
 * blending weights taken from the sides' own node spacing (Soni's boundary-conforming form), so
 * a rail graded toward a fine corner grades the ribs with it, and where the two rails disagree
 * the ribs turn gradually from one rail's spacing to the other's. On a fillet whose cross sides
 * were pinned to the same values of the curved parameter the rulings come out exactly straight
 * whatever the weights are.
 */
function gridMesh(
  surface: Surface, crossA: MappedSide, crossB: MappedSide, rail0: MappedSide, rail1: MappedSide, mU: number, mV: number,
): MappedMesh | null {
  const N = crossA.p2.length - 1, M = rail0.p2.length - 1;
  const t0 = fractions(rail0.p3), t1 = fractions(rail1.p3), wA = fractions(crossA.p3), wB = fractions(crossB.p3);
  const P00 = rail0.p2[0]!, P01 = rail0.p2[M]!, P10 = rail1.p2[0]!, P11 = rail1.p2[M]!;
  const p2: P2[] = [], p3: Vec3[] = [];
  const add = (q: P2, p: Vec3): number => { p2.push(q); p3.push(p); return p2.length - 1; };
  const id: number[][] = [];
  for (let i = 0; i <= N; i++) {
    const row: number[] = [];
    for (let k = 0; k <= M; k++) {
      if (i === 0) { row.push(add(rail0.p2[k]!, rail0.p3[k]!)); continue; }
      if (i === N) { row.push(add(rail1.p2[k]!, rail1.p3[k]!)); continue; }
      if (k === 0) { row.push(add(crossA.p2[i]!, crossA.p3[i]!)); continue; }
      if (k === M) { row.push(add(crossB.p2[i]!, crossB.p3[i]!)); continue; }
      const da = wB[i]! - wA[i]!, dt = t1[k]! - t0[k]!;
      const w = (wA[i]! + t0[k]! * da) / (1 - dt * da), s = t0[k]! + w * dt;
      const R0 = rail0.p2[k]!, R1 = rail1.p2[k]!, A = crossA.p2[i]!, B = crossB.p2[i]!;
      const q: P2 = [0, 0];
      for (const c of [0, 1] as const) {
        q[c] = (1 - w) * R0[c] + w * R1[c] + (1 - s) * A[c] + s * B[c]
          - ((1 - w) * (1 - s) * P00[c] + w * (1 - s) * P10[c] + (1 - w) * s * P01[c] + w * s * P11[c]);
      }
      row.push(add(q, surface.evaluate(q[0], q[1])));
    }
    id.push(row);
  }
  // One diagonal per quad. A rectangle's two diagonals are equal, so the same one is taken
  // throughout (a regular pattern); only a quad sheared by some ten degrees or more, as next
  // to a slanted end of the fillet, takes the other when that is clearly the shorter in 3D.
  const d3 = (a: number, b: number): number => Math.hypot(p3[a]![0] - p3[b]![0], p3[a]![1] - p3[b]![1], p3[a]![2] - p3[b]![2]);
  const tris: [number, number, number][] = [];
  for (let i = 0; i < N; i++) for (let k = 0; k < M; k++) {
    const a = id[i]![k]!, b = id[i + 1]![k]!, c = id[i + 1]![k + 1]!, d = id[i]![k + 1]!;
    if (d3(b, d) < 0.85 * d3(a, c)) tris.push([a, b, d], [b, c, d]);
    else tris.push([a, b, c], [a, c, d]);
  }
  return valid(p2, p3, tris, mU, mV);
}

/**
 * Common node stations for two opposite sides of a mapped face (its two rails, so that it can
 * take the GRID layout; or two cross sides made of several edges).
 *
 * `xa`, `xb` are the sides' present nodes as ascending positions 0..1 (each side normalised over
 * its own run of the parameter it advances in, so equal positions face each other across the
 * face); `va`, `vb` index the nodes that are B-rep vertices inside a side (joints between its
 * edges), which cannot move; `lockA`, `lockB` are node index ranges [first, last] whose nodes
 * may not change at all (edges that another face owns). Returned are the new positions for each
 * side, equal in number:
 *   - every vertex and every locked node stays; a fixed node of one side that lies within half
 *     a cell of a fixed node of the other shares its station (the line between them leans by
 *     that much; further apart, each gets a station of its own and the cell between them is
 *     at least half a cell long);
 *   - between two stations the cell count is what the DENSER side asks for at each point (the
 *     integral of the larger of the two node densities), so neither side ends up coarser than
 *     its own seeding wanted, and a grading of either side is kept; inside a locked range
 *     there is nothing to choose, and the other side takes that range's nodes.
 *
 * Null when no sensible common set exists: two vertices too close together for a cell of their
 * own yet too far apart to share a station, locked ranges of the two sides that disagree, or
 * sides whose spacings differ too much somewhere for one set of stations to suit both.
 */
export function matchRailStations(
  xa: number[], va: number[], xb: number[], vb: number[], lockA: [number, number][] = [], lockB: [number, number][] = [],
): { sa: number[]; sb: number[] } | null {
  const Ma = xa.length - 1, Mb = xb.length - 1;
  if (Ma < 1 || Mb < 1) return null;
  const REFINE = 3, COARSEN = 2;
  const xs = [xa, xb], locks = [lockA, lockB];
  /** Index of the cell of side r that contains position x. */
  const cellAt = (r: number, x: number): number => {
    const a = xs[r]!;
    let lo = 0, hi = a.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (a[m]! <= x) lo = m; else hi = m; }
    return lo;
  };
  const cell = (r: number, x: number): number => { const k = cellAt(r, x); return xs[r]![k + 1]! - xs[r]![k]!; };
  const h = (x: number): number => Math.min(cell(0, x), cell(1, x));
  /** True when cell k of side r lies in one of its locked ranges. */
  const lockedCell = (r: number, k: number): boolean => locks[r]!.some(([i0, i1]) => k >= i0 && k < i1);
  // Fixed nodes: joints and the nodes of locked ranges. `free` = a joint (it could share a
  // station with a near neighbour of its own side only by leaving a cell too thin to keep).
  const man: { x: number; r: 0 | 1; joint: boolean }[] = [];
  for (const r of [0, 1] as const) {
    const fixed = new Map<number, boolean>();
    for (const k of r === 0 ? va : vb) fixed.set(k, true);
    for (const [i0, i1] of locks[r]!) for (let k = i0; k <= i1; k++) if (k > 0 && k < xs[r]!.length - 1) fixed.set(k, false);
    for (const [k, joint] of fixed) man.push({ x: xs[r]![k]!, r, joint });
  }
  man.sort((p, q) => p.x - q.x);
  // Stations so far: position on each side, and which sides have a fixed node there.
  const st: { x: [number, number]; hard: [boolean, boolean] }[] = [{ x: [0, 0], hard: [true, true] }];
  const SNAP = 0.5, CLOSE = 0.3;
  for (const e of man) {
    const prev = st[st.length - 1]!;
    const gap = e.x - prev.x[e.r], size = h(e.x);
    if (st.length > 1 && !prev.hard[e.r] && Math.abs(e.x - prev.x[1 - e.r]!) < SNAP * size) { prev.x[e.r] = e.x; prev.hard[e.r] = true; continue; }
    if (e.joint && (gap < CLOSE * size || 1 - e.x < CLOSE * size)) return null;
    st.push({ x: [e.x, e.x], hard: e.r === 0 ? [true, false] : [false, true] });
  }
  st.push({ x: [1, 1], hard: [true, true] });
  // A station inside a locked range must be one of that range's own nodes.
  for (let i = 1; i + 1 < st.length; i++) for (const r of [0, 1]) {
    if (!st[i]!.hard[r] && lockedCell(r, cellAt(r, st[i]!.x[r]!))) return null;
  }
  /** Node count of side r up to position x (piecewise linear through its nodes). */
  const count = (r: number, x: number): number => {
    const a = xs[r]!, lo = cellAt(r, x);
    const d = a[lo + 1]! - a[lo]!;
    return lo + (d > 0 ? Math.min(1, Math.max(0, (x - a[lo]!) / d)) : 0);
  };
  const sa: number[] = [0], sb: number[] = [0];
  for (let i = 0; i + 1 < st.length; i++) {
    const p = st[i]!, q = st[i + 1]!;
    if (!(q.x[0] > p.x[0]) || !(q.x[1] > p.x[1])) return null;
    const inLock = [0, 1].some((r) => lockedCell(r, cellAt(r, (p.x[r]! + q.x[r]!) / 2)));
    if (!inLock) {
      // Both sides over this interval, each in its own coordinate, as functions of the share
      // l = 0..1 of the interval; break points are the nodes of either.
      const ls: number[] = [0, 1];
      for (const r of [0, 1]) for (const x of xs[r]!) if (x > p.x[r]! && x < q.x[r]!) ls.push((x - p.x[r]!) / (q.x[r]! - p.x[r]!));
      ls.sort((u, v) => u - v);
      const at = (r: number, l: number): number => count(r, p.x[r]! + l * (q.x[r]! - p.x[r]!));
      const A: number[] = [0];
      for (let j = 1; j < ls.length; j++) A.push(A[j - 1]! + Math.max(at(0, ls[j]!) - at(0, ls[j - 1]!), at(1, ls[j]!) - at(1, ls[j - 1]!)));
      const T = A[A.length - 1]!;
      const m = Math.max(1, Math.ceil(T - 0.3)); // as the edge seeder rounds its size count
      let j = 1;
      for (let k = 1; k < m; k++) {
        const target = (T * k) / m;
        while (j < A.length - 1 && A[j]! < target) j++;
        const dA = A[j]! - A[j - 1]!;
        const l = ls[j - 1]! + (dA > 0 ? (target - A[j - 1]!) / dA : 0) * (ls[j]! - ls[j - 1]!);
        sa.push(p.x[0] + l * (q.x[0] - p.x[0])); sb.push(p.x[1] + l * (q.x[1] - p.x[1]));
      }
    }
    sa.push(q.x[0]); sb.push(q.x[1]);
  }
  // The common stations must suit BOTH sides as they were seeded. Each side's seeding is what
  // its surroundings asked for, and its other neighbour meshes against it: a side refined far
  // beyond that (because the opposite one runs into something fine that this one is nowhere
  // near) would lay needle-thin cells across the face, and one left much coarser would
  // under-resolve its neighbour. Such a face keeps its sides. (Locked cells are as they were.)
  for (const [r, s] of [[0, sa], [1, sb]] as const) {
    for (let k = 0; k + 1 < s.length; k++) {
      const mid = (s[k]! + s[k + 1]!) / 2;
      if (lockedCell(r, cellAt(r, mid))) continue;
      const own = cell(r, mid), now = s[k + 1]! - s[k]!;
      if (own > REFINE * now || now > COARSEN * own) return null;
    }
  }
  return { sa, sb };
}
