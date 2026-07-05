// SPDX-License-Identifier: AGPL-3.0-only
// BVH over a flat triangle array (9 doubles per triangle) for closest-point-to-mesh
// queries. The brute force in hausdorff.ts is O(samples * tris) which is fine for the
// small reference models but not for scraped real-world STEPs (hundreds of k tris).

export interface TriBVH {
  /** Triangles, 9 doubles each, reordered so each leaf's triangles are contiguous. */
  tris: Float64Array;
  /** Per node: minx,miny,minz,maxx,maxy,maxz. */
  nodeBox: Float64Array;
  /** Per node: leaf => [-(start+1), count] (start in triangles); inner => [left, right]. */
  nodeInfo: Int32Array;
  /** Original triangle index per reordered triangle (provenance survives the reorder). */
  triOrder: Uint32Array;
}

const LEAF_SIZE = 8;

/** Quickselect order[lo..hi) so that order[lo..mid) have key <= order[mid..hi). */
function partitionMedian(order: Uint32Array, key: Float64Array, lo: number, hi: number, mid: number): void {
  while (hi - lo > 1) {
    // median-of-three pivot
    const a = key[order[lo]!]!, b = key[order[(lo + hi) >> 1]!]!, c = key[order[hi - 1]!]!;
    const pivot = Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));
    let i = lo, j = hi - 1;
    while (i <= j) {
      while (key[order[i]!]! < pivot) i++;
      while (key[order[j]!]! > pivot) j--;
      if (i <= j) {
        const t = order[i]!; order[i] = order[j]!; order[j] = t;
        i++; j--;
      }
    }
    // recurse into the side containing mid
    if (mid <= j) hi = j + 1;
    else if (mid >= i) lo = i;
    else return;
  }
}

export function buildBVH(tris: Float64Array): TriBVH {
  const nT = tris.length / 9;
  const cent = new Float64Array(nT * 3);
  const cx = new Float64Array(nT), cy = new Float64Array(nT), cz = new Float64Array(nT);
  for (let t = 0; t < nT; t++) {
    const o = t * 9;
    cx[t] = (tris[o]! + tris[o + 3]! + tris[o + 6]!) / 3;
    cy[t] = (tris[o + 1]! + tris[o + 4]! + tris[o + 7]!) / 3;
    cz[t] = (tris[o + 2]! + tris[o + 5]! + tris[o + 8]!) / 3;
    cent[t * 3] = cx[t]!; cent[t * 3 + 1] = cy[t]!; cent[t * 3 + 2] = cz[t]!;
  }
  const order = new Uint32Array(nT);
  for (let i = 0; i < nT; i++) order[i] = i;

  const boxes: number[] = [];
  const info: number[] = [];

  const build = (lo: number, hi: number): number => {
    const node = info.length / 2;
    info.push(0, 0);
    let minx = Infinity, miny = Infinity, minz = Infinity, maxx = -Infinity, maxy = -Infinity, maxz = -Infinity;
    for (let i = lo; i < hi; i++) {
      const o = order[i]! * 9;
      for (let v = 0; v < 9; v += 3) {
        const x = tris[o + v]!, y = tris[o + v + 1]!, z = tris[o + v + 2]!;
        if (x < minx) minx = x; if (x > maxx) maxx = x;
        if (y < miny) miny = y; if (y > maxy) maxy = y;
        if (z < minz) minz = z; if (z > maxz) maxz = z;
      }
    }
    boxes.push(minx, miny, minz, maxx, maxy, maxz);
    if (hi - lo <= LEAF_SIZE) {
      info[node * 2] = -(lo + 1);
      info[node * 2 + 1] = hi - lo;
      return node;
    }
    // split at median of the longest centroid axis
    let cminx = Infinity, cminy = Infinity, cminz = Infinity, cmaxx = -Infinity, cmaxy = -Infinity, cmaxz = -Infinity;
    for (let i = lo; i < hi; i++) {
      const t = order[i]!;
      if (cx[t]! < cminx) cminx = cx[t]!; if (cx[t]! > cmaxx) cmaxx = cx[t]!;
      if (cy[t]! < cminy) cminy = cy[t]!; if (cy[t]! > cmaxy) cmaxy = cy[t]!;
      if (cz[t]! < cminz) cminz = cz[t]!; if (cz[t]! > cmaxz) cmaxz = cz[t]!;
    }
    const ex = cmaxx - cminx, ey = cmaxy - cminy, ez = cmaxz - cminz;
    const key = ex >= ey && ex >= ez ? cx : ey >= ez ? cy : cz;
    const mid = (lo + hi) >> 1;
    partitionMedian(order, key, lo, hi, mid);
    const left = build(lo, mid);
    const right = build(mid, hi);
    info[node * 2] = left;
    info[node * 2 + 1] = right;
    return node;
  };
  if (nT > 0) build(0, nT);

  // reorder triangles for contiguous leaves
  const rTris = new Float64Array(nT * 9);
  for (let i = 0; i < nT; i++) rTris.set(tris.subarray(order[i]! * 9, order[i]! * 9 + 9), i * 9);
  return { tris: rTris, nodeBox: Float64Array.from(boxes), nodeInfo: Int32Array.from(info), triOrder: order };
}

/** Squared distance from p to triangle at flat offset o (Ericson, Real-Time Collision Detection). */
function triDistSq(T: Float64Array, o: number, px: number, py: number, pz: number): number {
  const ax = T[o]!, ay = T[o + 1]!, az = T[o + 2]!;
  const abx = T[o + 3]! - ax, aby = T[o + 4]! - ay, abz = T[o + 5]! - az;
  const acx = T[o + 6]! - ax, acy = T[o + 7]! - ay, acz = T[o + 8]! - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz;
  const d2 = acx * apx + acy * apy + acz * apz;
  if (d1 <= 0 && d2 <= 0) return apx * apx + apy * apy + apz * apz;

  const bpx = apx - abx, bpy = apy - aby, bpz = apz - abz;
  const d3 = abx * bpx + aby * bpy + abz * bpz;
  const d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) return bpx * bpx + bpy * bpy + bpz * bpz;

  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    const dx = apx - abx * v, dy = apy - aby * v, dz = apz - abz * v;
    return dx * dx + dy * dy + dz * dz;
  }
  const cpx = apx - acx, cpy = apy - acy, cpz = apz - acz;
  const d5 = abx * cpx + aby * cpy + abz * cpz;
  const d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) return cpx * cpx + cpy * cpy + cpz * cpz;

  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    const dx = apx - acx * w, dy = apy - acy * w, dz = apz - acz * w;
    return dx * dx + dy * dy + dz * dz;
  }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    const dx = apx - (abx + (acx - abx) * w), dy = apy - (aby + (acy - aby) * w), dz = apz - (abz + (acz - abz) * w);
    return dx * dx + dy * dy + dz * dz;
  }
  const denom = 1 / (va + vb + vc);
  const v = vb * denom, w = vc * denom;
  const dx = apx - (abx * v + acx * w), dy = apy - (aby * v + acy * w), dz = apz - (abz * v + acz * w);
  return dx * dx + dy * dy + dz * dz;
}

function boxDistSq(B: Float64Array, o: number, x: number, y: number, z: number): number {
  const dx = Math.max(B[o]! - x, 0, x - B[o + 3]!);
  const dy = Math.max(B[o + 1]! - y, 0, y - B[o + 4]!);
  const dz = Math.max(B[o + 2]! - z, 0, z - B[o + 5]!);
  return dx * dx + dy * dy + dz * dz;
}

const stack = new Int32Array(256);

/** Distance from point to the closest triangle of the mesh. */
export function closestDist(bvh: TriBVH, x: number, y: number, z: number): number {
  if (bvh.tris.length === 0) return Infinity;
  let best = Infinity;
  let top = 0;
  stack[top++] = 0;
  while (top > 0) {
    const node = stack[--top]!;
    if (boxDistSq(bvh.nodeBox, node * 6, x, y, z) >= best) continue;
    const a = bvh.nodeInfo[node * 2]!;
    if (a < 0) { // leaf
      const start = -a - 1, count = bvh.nodeInfo[node * 2 + 1]!;
      for (let t = start; t < start + count; t++) {
        const d = triDistSq(bvh.tris, t * 9, x, y, z);
        if (d < best) best = d;
      }
    } else {
      const b = bvh.nodeInfo[node * 2 + 1]!;
      const da = boxDistSq(bvh.nodeBox, a * 6, x, y, z);
      const db = boxDistSq(bvh.nodeBox, b * 6, x, y, z);
      // push farther first so nearer is processed first
      if (da <= db) {
        if (db < best) stack[top++] = b;
        if (da < best) stack[top++] = a;
      } else {
        if (da < best) stack[top++] = a;
        if (db < best) stack[top++] = b;
      }
    }
  }
  return Math.sqrt(best);
}
