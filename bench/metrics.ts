// SPDX-License-Identifier: AGPL-3.0-only
// The KPIs measured on every converter's output, the same way for everyone:
//
//   watertight   after merging coincident vertices (within 1e-6 x model diagonal, which any consumer
//                can do in one pass): open edges, non-manifold edges, edges whose two triangles
//                disagree on orientation. "closed" = all three are zero.
//   shared       vertices as delivered / vertices after the merge (1.0 = the library already shares
//                vertices between faces; 1.4 = 40% duplicates along face borders).
//   shape        smallest interior angle of every triangle: 5th percentile and median (60 = equilateral),
//                share under 20 degrees and under 5 degrees, mean radius ratio (1 = equilateral).
//   accuracy     area-uniform samples on the mesh, distance to the true surface (= the nearer of two
//                independent fine references, meshStep and OCCT, each meshed at a tenth of the tolerance),
//                as a multiple of the requested tolerance: max and 99th percentile.
//   flipped      share of the mesh area whose triangle normal points against the true surface.
//   missing      share of the true surface farther than 5 x tolerance from the mesh (faces left out,
//                parts not read).
import { buildBVH, type TriBVH } from "../test/closest.ts";

export interface RefMesh { positions: Float32Array | Float64Array; indices: Uint32Array; bodyOfTri?: Uint32Array }

export interface Metrics {
  tris: number;
  vertsDelivered: number;
  verts: number;
  shared: number;
  collapsed: number;
  openEdges: number;
  nonManifoldEdges: number;
  misorientedEdges: number;
  closed: boolean;
  minAngP5: number;
  minAngP50: number;
  under20: number;
  under5: number;
  radiusRatio: number;
  devMax?: number;
  devP99?: number;
  flipped?: number;
  missing?: number;
}

// ---------------------------------------------------------------- weld + edges

/** Split vertices shared by triangles of different bodies, so the merge never joins two bodies. */
function splitByBody(m: RefMesh): { P: ArrayLike<number>; I: Uint32Array; body: Uint32Array } {
  const B = m.bodyOfTri;
  if (!B) return { P: m.positions, I: m.indices, body: new Uint32Array(m.positions.length / 3) };
  const key = new Map<number, number>(), nv0 = m.positions.length / 3;
  const P: number[] = [], body: number[] = [], I = new Uint32Array(m.indices.length);
  for (let i = 0; i < m.indices.length; i++) {
    const v = m.indices[i]!, b = B[(i / 3) | 0]!, k = b * nv0 + v;
    let nv = key.get(k);
    if (nv === undefined) {
      nv = body.length; key.set(k, nv); body.push(b);
      P.push(m.positions[v * 3]!, m.positions[v * 3 + 1]!, m.positions[v * 3 + 2]!);
    }
    I[i] = nv;
  }
  return { P, I, body: new Uint32Array(body) };
}

function weld(P: ArrayLike<number>, body: Uint32Array, eps: number): { map: Uint32Array; count: number } {
  const n = P.length / 3, map = new Uint32Array(n);
  const cells = new Map<string, number[]>();
  const rep: number[] = [];
  const inv = 1 / eps;
  for (let i = 0; i < n; i++) {
    const x = P[i * 3]!, y = P[i * 3 + 1]!, z = P[i * 3 + 2]!;
    const cx = Math.floor(x * inv), cy = Math.floor(y * inv), cz = Math.floor(z * inv), b = body[i]!;
    let found = -1;
    for (let dx = -1; dx <= 1 && found < 0; dx++) for (let dy = -1; dy <= 1 && found < 0; dy++) for (let dz = -1; dz <= 1 && found < 0; dz++) {
      const list = cells.get(`${cx + dx},${cy + dy},${cz + dz}`);
      if (!list) continue;
      for (const r of list) {
        const j = rep[r]!;
        if (body[j] === b && Math.abs(P[j * 3]! - x) <= eps && Math.abs(P[j * 3 + 1]! - y) <= eps && Math.abs(P[j * 3 + 2]! - z) <= eps) { found = r; break; }
      }
    }
    if (found < 0) {
      found = rep.length; rep.push(i);
      const k = `${cx},${cy},${cz}`;
      const list = cells.get(k);
      if (list) list.push(found); else cells.set(k, [found]);
    }
    map[i] = found;
  }
  return { map, count: rep.length };
}

function edgeStats(I: Uint32Array, map: Uint32Array, nv: number) {
  // One entry per triangle side: key (lo * nv + hi) * 2 + (1 when traversed hi -> lo), sorted, so
  // the uses of an edge sit next to each other. A Map of edges overflows its ~16.7M-entry limit on
  // 10M-triangle meshes (wallganizer at 1e-4 × diagonal); a Float64Array holds the keys exactly
  // up to 2^53, i.e. nv up to 6.7e7.
  const keys = new Float64Array(I.length);
  let n = 0, collapsed = 0;
  for (let t = 0; t < I.length; t += 3) {
    const a = map[I[t]!]!, b = map[I[t + 1]!]!, c = map[I[t + 2]!]!;
    if (a === b || b === c || c === a) { collapsed++; continue; }
    keys[n++] = a < b ? (a * nv + b) * 2 : (b * nv + a) * 2 + 1;
    keys[n++] = b < c ? (b * nv + c) * 2 : (c * nv + b) * 2 + 1;
    keys[n++] = c < a ? (c * nv + a) * 2 : (a * nv + c) * 2 + 1;
  }
  const sorted = keys.subarray(0, n).sort();
  let open = 0, nonManifold = 0, misoriented = 0;
  for (let i = 0; i < n;) {
    const edge = Math.floor(sorted[i]! / 2);
    let j = i, net = 0;
    while (j < n && Math.floor(sorted[j]! / 2) === edge) { net += sorted[j]! % 2 ? -1 : 1; j++; }
    const c = j - i;
    if (c === 1) open++;
    else if (c > 2) nonManifold++;
    else if (net !== 0) misoriented++;
    i = j;
  }
  return { open, nonManifold, misoriented, collapsed };
}

// ---------------------------------------------------------------- triangle shape

function shape(P: ArrayLike<number>, I: Uint32Array) {
  const nt = I.length / 3, minAng = new Float32Array(nt);
  let q = 0, under20 = 0, under5 = 0;
  for (let t = 0; t < nt; t++) {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const la = Math.hypot(P[b]! - P[c]!, P[b + 1]! - P[c + 1]!, P[b + 2]! - P[c + 2]!);
    const lb = Math.hypot(P[a]! - P[c]!, P[a + 1]! - P[c + 1]!, P[a + 2]! - P[c + 2]!);
    const lc = Math.hypot(P[a]! - P[b]!, P[a + 1]! - P[b + 1]!, P[a + 2]! - P[b + 2]!);
    const s = (la + lb + lc) / 2;
    const area = Math.sqrt(Math.max(0, s * (s - la) * (s - lb) * (s - lc)));
    // radius ratio 2 r_in / r_circ = 16 A^2 / (la lb lc (la+lb+lc)) ... written stably:
    const rr = la * lb * lc > 0 ? (8 * area * area) / (la * lb * lc * s) : 0;
    q += rr;
    // smallest angle sits opposite the shortest side
    const m = Math.min(la, lb, lc), o1 = m === la ? lb : la, o2 = m === lc ? lb : lc;
    const cos = (o1 * o1 + o2 * o2 - m * m) / (2 * o1 * o2 || 1);
    const ang = area > 0 ? Math.acos(Math.max(-1, Math.min(1, cos))) * 180 / Math.PI : 0;
    minAng[t] = ang;
    if (ang < 20) under20++;
    if (ang < 5) under5++;
  }
  const sorted = minAng.slice().sort();
  const pct = (p: number) => nt ? sorted[Math.min(nt - 1, Math.floor(p * nt))]! : 0;
  return { minAngP5: pct(0.05), minAngP50: pct(0.5), under20: nt ? under20 / nt : 0, under5: nt ? under5 / nt : 0, radiusRatio: nt ? q / nt : 0 };
}

// ---------------------------------------------------------------- closest point with triangle

function triDistSq(T: Float64Array, o: number, px: number, py: number, pz: number): number {
  const ax = T[o]!, ay = T[o + 1]!, az = T[o + 2]!;
  const abx = T[o + 3]! - ax, aby = T[o + 4]! - ay, abz = T[o + 5]! - az;
  const acx = T[o + 6]! - ax, acy = T[o + 7]! - ay, acz = T[o + 8]! - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  let qx: number, qy: number, qz: number;
  if (d1 <= 0 && d2 <= 0) { qx = ax; qy = ay; qz = az; } else {
    const bpx = px - T[o + 3]!, bpy = py - T[o + 4]!, bpz = pz - T[o + 5]!;
    const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
    const cpx = px - T[o + 6]!, cpy = py - T[o + 7]!, cpz = pz - T[o + 8]!;
    const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
    const vc = d1 * d4 - d3 * d2, vb = d5 * d2 - d1 * d6, va = d3 * d6 - d5 * d4;
    if (d3 >= 0 && d4 <= d3) { qx = T[o + 3]!; qy = T[o + 4]!; qz = T[o + 5]!; }
    else if (d6 >= 0 && d5 <= d6) { qx = T[o + 6]!; qy = T[o + 7]!; qz = T[o + 8]!; }
    else if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); qx = ax + abx * v; qy = ay + aby * v; qz = az + abz * v; }
    else if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); qx = ax + acx * w; qy = ay + acy * w; qz = az + acz * w; }
    else if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
      const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      qx = T[o + 3]! + (T[o + 6]! - T[o + 3]!) * w; qy = T[o + 4]! + (T[o + 7]! - T[o + 4]!) * w; qz = T[o + 5]! + (T[o + 8]! - T[o + 5]!) * w;
    } else {
      const den = 1 / (va + vb + vc), v = vb * den, w = vc * den;
      qx = ax + abx * v + acx * w; qy = ay + aby * v + acy * w; qz = az + abz * v + acz * w;
    }
  }
  return (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2;
}

const boxDistSq = (B: Float64Array, o: number, x: number, y: number, z: number): number => {
  const dx = Math.max(B[o]! - x, 0, x - B[o + 3]!), dy = Math.max(B[o + 1]! - y, 0, y - B[o + 4]!), dz = Math.max(B[o + 2]! - z, 0, z - B[o + 5]!);
  return dx * dx + dy * dy + dz * dz;
};
const stack = new Int32Array(512);

/** Squared distance to, and reordered index of, the closest triangle. */
function closest(bvh: TriBVH, x: number, y: number, z: number): [number, number] {
  if (bvh.tris.length === 0) return [Infinity, -1];
  let best = Infinity, bestT = -1, top = 0;
  stack[top++] = 0;
  while (top > 0) {
    const node = stack[--top]!;
    if (boxDistSq(bvh.nodeBox, node * 6, x, y, z) >= best) continue;
    const a = bvh.nodeInfo[node * 2]!;
    if (a < 0) {
      const start = -a - 1, count = bvh.nodeInfo[node * 2 + 1]!;
      for (let t = start; t < start + count; t++) {
        const d = triDistSq(bvh.tris, t * 9, x, y, z);
        if (d < best) { best = d; bestT = t; }
      }
    } else {
      const b = bvh.nodeInfo[node * 2 + 1]!;
      const da = boxDistSq(bvh.nodeBox, a * 6, x, y, z), db = boxDistSq(bvh.nodeBox, b * 6, x, y, z);
      if (da <= db) { if (db < best) stack[top++] = b; if (da < best) stack[top++] = a; }
      else { if (da < best) stack[top++] = a; if (db < best) stack[top++] = b; }
    }
  }
  return [best, bestT];
}

/** Is there a triangle within distance r of the point whose normal agrees (cos > -0.5) with n? */
function anyAgrees(bvh: TriBVH, x: number, y: number, z: number, r: number, n: [number, number, number]): boolean {
  const r2 = r * r, ln = Math.hypot(...n);
  let top = 0;
  stack[top++] = 0;
  while (top > 0) {
    const node = stack[--top]!;
    if (boxDistSq(bvh.nodeBox, node * 6, x, y, z) > r2) continue;
    const a = bvh.nodeInfo[node * 2]!;
    if (a < 0) {
      const start = -a - 1, count = bvh.nodeInfo[node * 2 + 1]!;
      for (let t = start; t < start + count; t++) {
        if (triDistSq(bvh.tris, t * 9, x, y, z) > r2) continue;
        const b = triNormal(bvh.tris, t * 9), lb = Math.hypot(...b);
        if (lb === 0 || ln === 0 || (n[0] * b[0] + n[1] * b[1] + n[2] * b[2]) / (ln * lb) > -0.5) return true;
      }
    } else { stack[top++] = a; stack[top++] = bvh.nodeInfo[node * 2 + 1]!; }
  }
  return false;
}

const triNormal = (T: ArrayLike<number>, o: number): [number, number, number] => {
  const ux = T[o + 3]! - T[o]!, uy = T[o + 4]! - T[o + 1]!, uz = T[o + 5]! - T[o + 2]!;
  const vx = T[o + 6]! - T[o]!, vy = T[o + 7]! - T[o + 1]!, vz = T[o + 8]! - T[o + 2]!;
  return [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx];
};

/** Flat triangle array (9 numbers per triangle). */
export function soup(m: RefMesh): Float64Array {
  const I = m.indices, P = m.positions, T = new Float64Array(I.length * 3);
  for (let i = 0; i < I.length; i++) { const v = I[i]! * 3; T[i * 3] = P[v]!; T[i * 3 + 1] = P[v + 1]!; T[i * 3 + 2] = P[v + 2]!; }
  return T;
}

export const bvhOf = (m: RefMesh): TriBVH => buildBVH(soup(m));

function mulberry32(a: number) {
  return () => { a |= 0; a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

/** n area-uniform samples on a triangle soup: [x, y, z, triangle] per sample. */
function samples(T: Float64Array, n: number, seed: number): Float64Array {
  const nt = T.length / 9, cum = new Float64Array(nt);
  let acc = 0;
  for (let t = 0; t < nt; t++) { const [x, y, z] = triNormal(T, t * 9); acc += Math.hypot(x, y, z); cum[t] = acc; }
  const out = new Float64Array(n * 4), rnd = mulberry32(seed);
  if (acc === 0) return out.subarray(0, 0);
  for (let i = 0; i < n; i++) {
    const r = rnd() * acc;
    let lo = 0, hi = nt - 1;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (cum[mid]! < r) lo = mid + 1; else hi = mid; }
    let u = rnd(), v = rnd();
    if (u + v > 1) { u = 1 - u; v = 1 - v; }
    const o = lo * 9;
    for (let k = 0; k < 3; k++) out[i * 4 + k] = T[o + k]! + (T[o + 3 + k]! - T[o + k]!) * u + (T[o + 6 + k]! - T[o + k]!) * v;
    out[i * 4 + 3] = lo;
  }
  return out;
}

// ---------------------------------------------------------------- all together

export interface Reference { soup: Float64Array; bvh: TriBVH; samples: Float64Array }

export function prepareReference(m: RefMesh, nSamples: number): Reference {
  const T = soup(m);
  return { soup: T, bvh: buildBVH(T), samples: samples(T, nSamples, 12345) };
}

export function measure(m: RefMesh, diag: number, tol: number, refs: Reference[], nSamples = 20000): Metrics {
  const split = splitByBody(m);
  const { map, count } = weld(split.P, split.body, diag * 1e-6);
  const e = edgeStats(split.I, map, count);
  const delivered = m.positions.length / 3;
  const sh = shape(m.positions, m.indices);
  const out: Metrics = {
    tris: m.indices.length / 3,
    vertsDelivered: delivered,
    verts: count,
    shared: count ? delivered / count : 1,
    collapsed: e.collapsed,
    openEdges: e.open,
    nonManifoldEdges: e.nonManifold,
    misorientedEdges: e.misoriented,
    closed: e.open === 0 && e.nonManifold === 0 && e.misoriented === 0 && m.indices.length > 0,
    ...sh,
  };
  if (!refs.length || !m.indices.length) return out;

  // mesh -> true surface
  const T = soup(m);
  const S = samples(T, nSamples, 777);
  const ns = S.length / 4, dev = new Float64Array(ns);
  let flippedN = 0;
  for (let i = 0; i < ns; i++) {
    const x = S[i * 4]!, y = S[i * 4 + 1]!, z = S[i * 4 + 2]!;
    let best = Infinity, bestRef = -1, bestT = -1;
    for (let r = 0; r < refs.length; r++) {
      const [d, t] = closest(refs[r]!.bvh, x, y, z);
      if (d < best) { best = d; bestRef = r; bestT = t; }
    }
    dev[i] = Math.sqrt(best);
    if (bestT >= 0) {
      // flipped = facing against the surface it lies on. Where two bodies touch, the coincident face of
      // the other body faces the other way, so look at every reference triangle about as close.
      const a = triNormal(T, S[i * 4 + 3]! * 9), b = triNormal(refs[bestRef]!.bvh.tris, bestT * 9);
      const la = Math.hypot(...a), lb = Math.hypot(...b);
      if (la > 0 && lb > 0 && (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb) < -0.5) {
        const r = dev[i]! + Math.max(tol, diag * 1e-6);
        if (!refs.some(ref => anyAgrees(ref.bvh, x, y, z, r, a))) flippedN++;
      }
    }
  }
  dev.sort();
  out.devMax = dev[ns - 1]! / tol;
  out.devP99 = dev[Math.floor(0.99 * (ns - 1))]! / tol;
  out.flipped = flippedN / ns;

  // true surface -> mesh: what is missing (the most lenient reference counts)
  const bvh = buildBVH(T);
  const far = Math.max(5 * tol, 1e-5 * diag);
  let missing = 1;
  for (const r of refs) {
    const nr = r.samples.length / 4;
    let miss = 0;
    for (let i = 0; i < nr; i++) {
      const [d] = closest(bvh, r.samples[i * 4]!, r.samples[i * 4 + 1]!, r.samples[i * 4 + 2]!);
      if (d > far * far) miss++;
    }
    missing = Math.min(missing, nr ? miss / nr : 1);
  }
  out.missing = missing;
  return out;
}
