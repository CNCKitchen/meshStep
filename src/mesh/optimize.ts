// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — per-face triangulation optimiser (isotropic mode).
//
// Takes one face's VALID triangulation — a planar triangulation of the face's trimmed region in
// its (metric-scaled) parameter plane, every vertex carrying its exact 3D point — and reworks
// the interior until the 3D triangles are as close to equilateral at the requested size as the
// fixed boundary allows. It is the local-operation remesher of Botsch & Kobbelt ("A Remeshing
// Approach to Multiresolution Modeling", SGP 2004) with the adaptive sizing of Dunyach et al.
// ("Adaptive Remeshing for Real-Time Mesh Deformation", EG 2013), run per face in parameter
// space:
//
//   split     an interior edge longer than 4/3 of its target gets a vertex at its midpoint
//   collapse  an interior vertex with an edge shorter than 4/5 of its target is merged into
//             that edge's other end
//   flip      an interior edge is swapped whenever the two angles facing it sum to more than
//             180 degrees (Delaunay, measured in 3D)
//   relax     an interior vertex moves toward the centroid of its ring, weighted so that
//             triangles too large for their target pull it in
//
// The target of an edge is the SMALLER of the size field at its two ends.
//
// What makes this safe to run on every face:
//   - Boundary vertices and boundary (constraint) edges are frozen. The edge samples shared
//     with the neighbouring faces are never touched, so the mesh stays watertight.
//   - Lengths, angles and quality are measured in 3D, but every vertex lives at a parameter
//     point and its 3D position is the surface evaluated there: vertices stay exactly on the
//     surface, with no closest-point projection (the step that makes 3D remeshers unreliable on
//     B-spline and torus faces).
//   - Every operation is rejected unless all triangles it touches keep a positive area in the
//     parameter plane. The result is therefore always a valid triangulation of the same region:
//     the optimiser cannot fold a face over itself or turn a triangle against the surface.
//
// The input must be a clean triangulation (manifold, consistently wound, no zero-area
// triangle); for anything else the function declines and the caller keeps what it had.
import type { Vec3 } from "../geom/vec.ts";

export interface OptimizeInput {
  /** Parameter-plane coordinates per vertex. Extended and modified IN PLACE. */
  x: number[];
  y: number[];
  /** 3D position per vertex, 3 numbers each. Extended and modified IN PLACE. */
  p: number[];
  /** Counter-clockwise triangles (in the parameter plane). */
  tris: [number, number, number][];
  /** Vertices with an index below this are boundary vertices: never moved, never removed. */
  nFrozen: number;
  /** True for a boundary segment (a,b): never split, flipped or collapsed. Region rim edges
   * (edges with one triangle) are constrained regardless. */
  isConstrained: (a: number, b: number) => boolean;
  /** Surface point at a parameter-plane position. */
  evalAt: (x: number, y: number) => Vec3;
  /** Target edge length (mm) at a parameter-plane position. */
  sizeAt: (x: number, y: number) => number;
  /** Rounds of split / collapse / flip / relax (default 5). */
  iterations?: number;
}

export interface OptimizeResult {
  tris: [number, number, number][];
  /** Operation counts, for diagnostics. */
  splits: number; collapses: number; flips: number; moves: number;
}

const KEY = 0x4000000; // vertex ids stay far below 2^26 for a single face

export function optimizeFace(inp: OptimizeInput): OptimizeResult | null {
  const { x: X, y: Y, p: P, nFrozen, evalAt, sizeAt } = inp;
  const nT0 = inp.tris.length;
  if (nT0 === 0 || X.length >= KEY) return null;

  // ---- triangle soup -> adjacency -------------------------------------------------------------
  let cap = Math.max(64, nT0 * 4);
  let tv = new Int32Array(cap * 3), tn = new Int32Array(cap * 3);
  let tc = new Uint8Array(cap * 3), dead = new Uint8Array(cap);
  // Flip worklist: triangles whose shape or neighbourhood changed since their edges were last
  // tested. Every operation touches the triangles it rewrites, so a flip pass only revisits
  // those instead of sweeping the whole face again.
  let queued = new Uint8Array(cap);
  const work: number[] = [];
  const touch = (t: number): void => { if (!queued[t]) { queued[t] = 1; work.push(t); } };
  let nT = nT0;
  const grow = (): void => {
    cap *= 2;
    const a = new Int32Array(cap * 3); a.set(tv); tv = a;
    const b = new Int32Array(cap * 3); b.set(tn); tn = b;
    const c = new Uint8Array(cap * 3); c.set(tc); tc = c;
    const d = new Uint8Array(cap); d.set(dead); dead = d;
    const q = new Uint8Array(cap); q.set(queued); queued = q;
  };
  const area2 = (a: number, b: number, c: number): number => (X[b]! - X[a]!) * (Y[c]! - Y[a]!) - (Y[b]! - Y[a]!) * (X[c]! - X[a]!);
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (let i = 0; i < X.length; i++) { if (X[i]! < x0) x0 = X[i]!; if (X[i]! > x1) x1 = X[i]!; if (Y[i]! < y0) y0 = Y[i]!; if (Y[i]! > y1) y1 = Y[i]!; }
  // Smallest parameter-plane area (doubled) a triangle may have: far below any real triangle,
  // far above rounding noise of the cross product at this face's coordinates.
  const EPS = 1e-13 * ((x1 - x0) ** 2 + (y1 - y0) ** 2 + 1e-300);
  const half = new Map<number, number>(); // directed edge a->b  ->  3*t + e
  for (let t = 0; t < nT0; t++) {
    const [a, b, c] = inp.tris[t]!;
    if (a === b || b === c || c === a || !(area2(a, b, c) > EPS)) return null; // degenerate input
    tv[t * 3] = a; tv[t * 3 + 1] = b; tv[t * 3 + 2] = c;
    for (let e = 0; e < 3; e++) {
      const k = tv[t * 3 + e]! * KEY + tv[t * 3 + (e + 1) % 3]!;
      if (half.has(k)) return null; // an edge used twice in the same direction: not a manifold disk
      half.set(k, t * 3 + e);
    }
  }
  for (let t = 0; t < nT0; t++) for (let e = 0; e < 3; e++) {
    const a = tv[t * 3 + e]!, b = tv[t * 3 + (e + 1) % 3]!;
    const tw = half.get(b * KEY + a);
    tn[t * 3 + e] = tw === undefined ? -1 : Math.floor(tw / 3);
    tc[t * 3 + e] = tw === undefined || inp.isConstrained(a, b) ? 1 : 0;
  }
  half.clear();

  const hv: number[] = new Array(X.length); // target size at each vertex
  const vt: number[] = new Array(X.length).fill(-1); // one live triangle per vertex
  const vdead: number[] = new Array(X.length).fill(0);
  for (let t = 0; t < nT0; t++) for (let e = 0; e < 3; e++) vt[tv[t * 3 + e]!] = t;
  for (let v = 0; v < X.length; v++) hv[v] = vt[v]! >= 0 ? sizeAt(X[v]!, Y[v]!) : 0;
  // Where each vertex's size was last read: a relaxed vertex re-reads the field only after it
  // has drifted a good fraction of that size away (the field changes slowly, the read is costly).
  const hx: number[] = X.slice(), hy: number[] = Y.slice();
  // Relaxation worklist flag: a vertex is revisited only while it or its ring keeps changing.
  const restless: number[] = new Array(X.length).fill(1);
  for (let t = 0; t < nT0; t++) touch(t);

  // ---- small helpers --------------------------------------------------------------------------
  const len3 = (a: number, b: number): number => Math.hypot(P[a * 3]! - P[b * 3]!, P[a * 3 + 1]! - P[b * 3 + 1]!, P[a * 3 + 2]! - P[b * 3 + 2]!);
  /** Radius-ratio quality (1 = equilateral, 0 = degenerate) of the 3D triangle (a,b,c). */
  const qual = (a: number, b: number, c: number): number => {
    const ab = len3(a, b), bc = len3(b, c), ca = len3(c, a);
    const s = (ab + bc + ca) / 2, d = ab * bc * ca;
    return d > 0 ? Math.max(0, (8 * (s - ab) * (s - bc) * (s - ca)) / d) : 0;
  };
  const corner = (t: number, v: number): number => (tv[t * 3] === v ? 0 : tv[t * 3 + 1] === v ? 1 : 2);
  const relink = (t: number, from: number, to: number, con: number): void => {
    if (t < 0) return;
    for (let e = 0; e < 3; e++) if (tn[t * 3 + e] === from) { tn[t * 3 + e] = to; if (con) tc[t * 3 + e] = 1; return; }
  };
  const setTri = (t: number, a: number, b: number, c: number, n0: number, n1: number, n2: number, c0: number, c1: number, c2: number): void => {
    const o = t * 3;
    tv[o] = a; tv[o + 1] = b; tv[o + 2] = c;
    tn[o] = n0; tn[o + 1] = n1; tn[o + 2] = n2;
    tc[o] = c0; tc[o + 1] = c1; tc[o + 2] = c2;
    vt[a] = t; vt[b] = t; vt[c] = t;
    restless[a] = 1; restless[b] = 1; restless[c] = 1;
    touch(t);
  };
  /** Triangles around an INTERIOR vertex, in rotation order; false if the fan is not closed. */
  const ring: number[] = [];
  const ringOf = (v: number): boolean => {
    ring.length = 0;
    const start = vt[v]!;
    let t = start;
    for (let guard = 0; guard < 256; guard++) {
      if (t < 0) return false;
      ring.push(t);
      t = tn[t * 3 + (corner(t, v) + 2) % 3]!;
      if (t === start) return true;
    }
    return false;
  };
  /** All vertices sharing an edge with v (works for boundary vertices: fans out both ways). */
  const nbr = new Set<number>();
  const neighborsOf = (v: number): void => {
    nbr.clear();
    const start = vt[v]!;
    let t = start, closed = false;
    for (let guard = 0; guard < 512 && t >= 0; guard++) {
      const i = corner(t, v);
      nbr.add(tv[t * 3 + (i + 1) % 3]!); nbr.add(tv[t * 3 + (i + 2) % 3]!);
      t = tn[t * 3 + (i + 2) % 3]!;
      if (t === start) { closed = true; break; }
    }
    if (closed) return;
    t = tn[start * 3 + corner(start, v)]!;
    for (let guard = 0; guard < 512 && t >= 0 && t !== start; guard++) {
      const i = corner(t, v);
      nbr.add(tv[t * 3 + (i + 1) % 3]!); nbr.add(tv[t * 3 + (i + 2) % 3]!);
      t = tn[t * 3 + i]!;
    }
  };
  const addVertex = (x: number, y: number): number => {
    const q = evalAt(x, y);
    const v = X.length;
    X.push(x); Y.push(y); P.push(q[0], q[1], q[2]);
    hv.push(sizeAt(x, y)); vt.push(-1); vdead.push(0);
    hx.push(x); hy.push(y); restless.push(1);
    return v;
  };
  // A runaway split loop cannot happen while the size field is bounded below, but a face whose
  // field and boundary disagree wildly must still end: stop adding vertices at this budget.
  const maxVerts = Math.min(KEY - 1, X.length * 8 + 20000);
  let splits = 0, collapses = 0, flips = 0, moves = 0;

  // ---- split ----------------------------------------------------------------------------------
  const splitEdge = (t: number, e: number): void => {
    const a = tv[t * 3 + e]!, b = tv[t * 3 + (e + 1) % 3]!, c = tv[t * 3 + (e + 2) % 3]!;
    const n = tn[t * 3 + e]!;
    const f = corner(n, b); // n holds the edge b -> a
    const d = tv[n * 3 + (f + 2) % 3]!;
    const tbc = tn[t * 3 + (e + 1) % 3]!, tca = tn[t * 3 + (e + 2) % 3]!;
    const cbc = tc[t * 3 + (e + 1) % 3]!, cca = tc[t * 3 + (e + 2) % 3]!;
    const nad = tn[n * 3 + (f + 1) % 3]!, ndb = tn[n * 3 + (f + 2) % 3]!;
    const cad = tc[n * 3 + (f + 1) % 3]!, cdb = tc[n * 3 + (f + 2) % 3]!;
    const m = addVertex((X[a]! + X[b]!) / 2, (Y[a]! + Y[b]!) / 2);
    if (nT + 2 > cap) grow();
    const t2 = nT++, n2 = nT++;
    dead[t2] = 0; dead[n2] = 0;
    setTri(t, a, m, c, n2, t2, tca, 0, 0, cca);
    setTri(t2, m, b, c, n, tbc, t, 0, cbc, 0);
    setTri(n, b, m, d, t2, n2, ndb, 0, 0, cdb);
    setTri(n2, m, a, d, t, nad, n, 0, cad, 0);
    relink(tbc, t, t2, 0);
    relink(nad, n, n2, 0);
    splits++;
  };
  const splitPass = (): boolean => {
    const cand: number[] = [], candL: number[] = [];
    for (let t = 0; t < nT; t++) {
      if (dead[t]) continue;
      for (let e = 0; e < 3; e++) {
        const n = tn[t * 3 + e]!;
        if (n < t || tc[t * 3 + e]) continue; // rim / constraint, or the twin handles it
        const a = tv[t * 3 + e]!, b = tv[t * 3 + (e + 1) % 3]!;
        const L = len3(a, b);
        if (L > (4 / 3) * Math.min(hv[a]!, hv[b]!)) { cand.push(t * 3 + e, a, b); candL.push(L); }
      }
    }
    if (candL.length === 0) return false;
    // Longest first: splitting the longest edge of a triangle keeps its halves well shaped,
    // and a shorter over-long edge beside it often no longer needs a split afterwards.
    const order = candL.map((_, i) => i).sort((i, j) => candL[j]! - candL[i]!);
    let did = false;
    for (const i of order) {
      if (X.length >= maxVerts) break;
      const te = cand[i * 3]!, a = cand[i * 3 + 1]!, b = cand[i * 3 + 2]!;
      const t = Math.floor(te / 3), e = te % 3;
      // An earlier split may have rewritten this triangle; the edge is handled next round then.
      if (dead[t] || tv[t * 3 + e] !== a || tv[t * 3 + (e + 1) % 3] !== b || tc[t * 3 + e] || tn[t * 3 + e]! < 0) continue;
      if (len3(a, b) <= (4 / 3) * Math.min(hv[a]!, hv[b]!)) continue;
      splitEdge(t, e);
      did = true;
    }
    return did;
  };

  // ---- collapse -------------------------------------------------------------------------------
  /** Merge interior vertex a into b (b keeps its position). False = rejected, nothing changed. */
  const collapse = (a: number, b: number): boolean => {
    if (!ringOf(a)) return false;
    let t = -1, n = -1;
    for (const r of ring) {
      const i = corner(r, a);
      if (tv[r * 3 + (i + 1) % 3] === b) t = r;      // a -> b
      if (tv[r * 3 + (i + 2) % 3] === b) n = r;      // b -> a
    }
    if (t < 0 || n < 0 || t === n) return false;
    const ia = corner(t, a), ja = corner(n, a);
    if (tc[t * 3 + ia]) return false; // the edge itself is a constraint
    const c = tv[t * 3 + (ia + 2) % 3]!, d = tv[n * 3 + (ja + 1) % 3]!;
    if (c === d) return false;
    // Link condition: a and b may share exactly the two vertices opposite their edge, or the
    // merge would pinch the triangulation into a non-manifold fan.
    neighborsOf(b);
    let oldMin = 1, newMin = 1;
    for (const r of ring) {
      const i = corner(r, a);
      const v1 = tv[r * 3 + (i + 1) % 3]!, v2 = tv[r * 3 + (i + 2) % 3]!;
      oldMin = Math.min(oldMin, qual(a, v1, v2));
      if (r === t || r === n) continue;
      if ((v1 !== c && v1 !== d && nbr.has(v1)) || (v2 !== c && v2 !== d && nbr.has(v2))) return false;
      if (!(area2(b, v1, v2) > EPS)) return false; // would flip or flatten in the parameter plane
      // Never create an edge the split pass would cut again (that is what makes the two passes
      // settle instead of chasing each other).
      if (len3(b, v1) > (4 / 3) * Math.min(hv[b]!, hv[v1]!) || len3(b, v2) > (4 / 3) * Math.min(hv[b]!, hv[v2]!)) return false;
      newMin = Math.min(newMin, qual(b, v1, v2));
    }
    // Removing a short edge must not leave a needle behind where there was none.
    if (newMin < 0.3 && newMin < oldMin) return false;
    const tbc = tn[t * 3 + (ia + 1) % 3]!, tca = tn[t * 3 + (ia + 2) % 3]!, cbc = tc[t * 3 + (ia + 1) % 3]!;
    const nad = tn[n * 3 + ja]!, ndb = tn[n * 3 + (ja + 1) % 3]!, cdb = tc[n * 3 + (ja + 1) % 3]!;
    for (const r of ring) {
      if (r === t || r === n) continue;
      const i = corner(r, a);
      tv[r * 3 + i] = b;
      vt[b] = r;
      restless[tv[r * 3 + (i + 1) % 3]!] = 1; restless[tv[r * 3 + (i + 2) % 3]!] = 1;
      touch(r);
    }
    restless[b] = 1;
    // Close the two gaps: the ring triangle beyond c -> a now borders whatever lay beyond b -> c
    // (inheriting a constraint there), and likewise on the d side.
    relink(tca, t, tbc, cbc); relink(tbc, t, tca, 0);
    relink(nad, n, ndb, cdb); relink(ndb, n, nad, 0);
    dead[t] = 1; dead[n] = 1; vdead[a] = 1;
    vt[c] = tca; vt[d] = nad;
    collapses++;
    return true;
  };
  const collapsePass = (): boolean => {
    let did = false;
    for (let a = nFrozen; a < X.length; a++) {
      if (vdead[a] || vt[a]! < 0 || !ringOf(a)) continue;
      // Shortest too-short edge at this vertex.
      let best = -1, bestR = 4 / 5;
      for (const r of ring) {
        const w = tv[r * 3 + (corner(r, a) + 1) % 3]!;
        const ratio = len3(a, w) / Math.min(hv[a]!, hv[w]!);
        if (ratio < bestR) { bestR = ratio; best = w; }
      }
      if (best >= 0 && collapse(a, best)) did = true;
    }
    return did;
  };

  // ---- flip -----------------------------------------------------------------------------------
  /** sin and cos of the 3D angle at `at` between the rays to u and w. */
  const ang = (at: number, u: number, w: number): [number, number] => {
    const ux = P[u * 3]! - P[at * 3]!, uy = P[u * 3 + 1]! - P[at * 3 + 1]!, uz = P[u * 3 + 2]! - P[at * 3 + 2]!;
    const wx = P[w * 3]! - P[at * 3]!, wy = P[w * 3 + 1]! - P[at * 3 + 1]!, wz = P[w * 3 + 2]! - P[at * 3 + 2]!;
    const cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
    return [Math.hypot(cx, cy, cz), ux * wx + uy * wy + uz * wz];
  };
  /** Flip edge e of triangle t if that is a valid Delaunay improvement. */
  const tryFlip = (t: number, e: number): boolean => {
    const n = tn[t * 3 + e]!;
    if (n < 0 || tc[t * 3 + e]) return false;
    const a = tv[t * 3 + e]!, b = tv[t * 3 + (e + 1) % 3]!, c = tv[t * 3 + (e + 2) % 3]!;
    const f = corner(n, b);
    const d = tv[n * 3 + (f + 2) % 3]!;
    // Delaunay in 3D: flip when the angles at c and d, facing the edge, sum past 180 degrees,
    // i.e. sin(C + D) < 0. Scale-free via the products of the two (unnormalised) sin/cos.
    const [sc, cc] = ang(c, a, b), [sd, cd] = ang(d, b, a);
    if (sc * cd + cc * sd >= -1e-9 * (Math.hypot(sc, cc) * Math.hypot(sd, cd))) return false;
    // The quad a-d-b-c must be strictly convex in the parameter plane.
    if (!(area2(a, d, c) > EPS) || !(area2(d, b, c) > EPS)) return false;
    const tbc = tn[t * 3 + (e + 1) % 3]!, tca = tn[t * 3 + (e + 2) % 3]!;
    const cbc = tc[t * 3 + (e + 1) % 3]!, cca = tc[t * 3 + (e + 2) % 3]!;
    const nad = tn[n * 3 + (f + 1) % 3]!, ndb = tn[n * 3 + (f + 2) % 3]!;
    const cad = tc[n * 3 + (f + 1) % 3]!, cdb = tc[n * 3 + (f + 2) % 3]!;
    setTri(t, a, d, c, nad, n, tca, cad, 0, cca);
    setTri(n, d, b, c, ndb, tbc, t, cdb, cbc, 0);
    relink(nad, n, t, 0);
    relink(tbc, t, n, 0);
    flips++;
    return true;
  };
  /** Drain the worklist. A flip re-queues its two triangles, so their other edges (whose facing
   * angles just changed) are tested again; the budget only guards against a numerical cycle. */
  const flipAll = (): void => {
    let budget = 20 * nT + 1000;
    while (work.length > 0 && budget-- > 0) {
      const t = work.pop()!;
      queued[t] = 0;
      if (dead[t]) continue;
      for (let e = 0; e < 3; e++) if (tryFlip(t, e)) break;
    }
    for (const t of work) queued[t] = 0;
    work.length = 0;
  };

  // ---- relax ----------------------------------------------------------------------------------
  const relaxPass = (): boolean => {
    let did = false;
    for (let v = nFrozen; v < X.length; v++) {
      if (!restless[v] || vdead[v] || vt[v]! < 0 || !ringOf(v)) continue;
      restless[v] = 0;
      // Target: centroid of the ring triangles' centroids, each weighted by its 3D area over its
      // target size squared — the number of target-size triangles it stands for. A triangle too
      // big for its place pulls the vertex in, which is what evens the sizes out while keeping
      // the gradation (a plain neighbour average would flatten it).
      let wx = 0, wy = 0, wz = 0, ws = 0, oldMin = 1;
      // Least-squares map from parameter-plane offsets to 3D offsets over the ring's spokes.
      let m00 = 0, m01 = 0, m11 = 0;
      let b00 = 0, b01 = 0, b10 = 0, b11 = 0, b20 = 0, b21 = 0;
      const px = P[v * 3]!, py = P[v * 3 + 1]!, pz = P[v * 3 + 2]!;
      for (const r of ring) {
        const i = corner(r, v);
        const u = tv[r * 3 + (i + 1) % 3]!, w = tv[r * 3 + (i + 2) % 3]!;
        const ux = P[u * 3]! - px, uy = P[u * 3 + 1]! - py, uz = P[u * 3 + 2]! - pz;
        const vx = P[w * 3]! - px, vy = P[w * 3 + 1]! - py, vz = P[w * 3 + 2]! - pz;
        const ar = Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
        const h = (hv[v]! + hv[u]! + hv[w]!) / 3;
        const wt = ar / (h * h);
        wx += wt * (ux + vx) / 3; wy += wt * (uy + vy) / 3; wz += wt * (uz + vz) / 3; ws += wt;
        oldMin = Math.min(oldMin, qual(v, u, w));
        const dx = X[u]! - X[v]!, dy = Y[u]! - Y[v]!;
        m00 += dx * dx; m01 += dx * dy; m11 += dy * dy;
        b00 += ux * dx; b01 += ux * dy; b10 += uy * dx; b11 += uy * dy; b20 += uz * dx; b21 += uz * dy;
      }
      if (!(ws > 0)) continue;
      const det = m00 * m11 - m01 * m01;
      if (!(Math.abs(det) > 1e-300)) continue;
      // J = B · M^-1 (3x2): the local surface map. Then the parameter step that best reproduces
      // the wanted 3D step d is (J^T J)^-1 J^T d, which also drops d's off-surface component.
      const i00 = m11 / det, i01 = -m01 / det, i11 = m00 / det;
      const j00 = b00 * i00 + b01 * i01, j01 = b00 * i01 + b01 * i11;
      const j10 = b10 * i00 + b11 * i01, j11 = b10 * i01 + b11 * i11;
      const j20 = b20 * i00 + b21 * i01, j21 = b20 * i01 + b21 * i11;
      const g00 = j00 * j00 + j10 * j10 + j20 * j20, g01 = j00 * j01 + j10 * j11 + j20 * j21, g11 = j01 * j01 + j11 * j11 + j21 * j21;
      const gd = g00 * g11 - g01 * g01;
      if (!(Math.abs(gd) > 1e-300)) continue;
      const tx = wx / ws, ty = wy / ws, tz = wz / ws;
      // Settled: a step under a few percent of the local size changes nothing worth a surface
      // evaluation. The vertex wakes up again when something around it moves.
      if (tx * tx + ty * ty + tz * tz < (0.03 * hv[v]!) ** 2) continue;
      const r0 = j00 * tx + j10 * ty + j20 * tz, r1 = j01 * tx + j11 * ty + j21 * tz;
      const nx = X[v]! + (g11 * r0 - g01 * r1) / gd, ny = Y[v]! + (g00 * r1 - g01 * r0) / gd;
      if (!Number.isFinite(nx + ny)) continue;
      // Accept only if the ring stays a valid fan in the parameter plane and its worst 3D
      // triangle does not get worse (unless it stays comfortably good).
      const ox = X[v]!, oy = Y[v]!;
      X[v] = nx; Y[v] = ny;
      let ok = true;
      for (const r of ring) if (!(area2(tv[r * 3]!, tv[r * 3 + 1]!, tv[r * 3 + 2]!) > EPS)) { ok = false; break; }
      if (!ok) { X[v] = ox; Y[v] = oy; continue; }
      const q = evalAt(nx, ny);
      P[v * 3] = q[0]; P[v * 3 + 1] = q[1]; P[v * 3 + 2] = q[2];
      let newMin = 1;
      for (const r of ring) newMin = Math.min(newMin, qual(tv[r * 3]!, tv[r * 3 + 1]!, tv[r * 3 + 2]!));
      if (!(newMin >= oldMin || newMin > 0.6)) {
        X[v] = ox; Y[v] = oy; P[v * 3] = px; P[v * 3 + 1] = py; P[v * 3 + 2] = pz;
        continue;
      }
      if ((nx - hx[v]!) ** 2 + (ny - hy[v]!) ** 2 > (0.25 * hv[v]!) ** 2) { hv[v] = sizeAt(nx, ny); hx[v] = nx; hy[v] = ny; }
      restless[v] = 1;
      for (const r of ring) {
        const i = corner(r, v);
        restless[tv[r * 3 + (i + 1) % 3]!] = 1;
        touch(r);
      }
      moves++; did = true;
    }
    return did;
  };

  // ---- driver ---------------------------------------------------------------------------------
  const iterations = inp.iterations ?? 5;
  flipAll();
  // Refine first: the input is as coarse as the face's base grid, and each split pass halves an
  // over-long edge once, so reaching a size 30x finer near a small feature takes several rounds.
  for (let k = 0; k < 12 && splitPass(); k++) flipAll();
  for (let it = 0; it < iterations; it++) {
    const s = splitPass();
    const c = collapsePass();
    flipAll();
    const r = relaxPass();
    flipAll();
    if (!s && !c && !r) break;
  }
  relaxPass();
  flipAll();

  const out: [number, number, number][] = [];
  for (let t = 0; t < nT; t++) if (!dead[t]) out.push([tv[t * 3]!, tv[t * 3 + 1]!, tv[t * 3 + 2]!]);
  return { tris: out, splits, collapses, flips, moves };
}
