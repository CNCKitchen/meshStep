// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — EdgeTopo: the undirected-edge incidence of a triangle list, built without hashing.
//
// Every stitch / orientation / audit pass needs "how many triangles use this edge, and which".
// A Map (or a hash table) keyed by the vertex pair answers that with one cache miss per half-edge,
// which on a multi-million-triangle mesh made those passes the largest share of an import. Here
// the half-edges are bucketed by their LOWER vertex with a counting sort (three sequential sweeps),
// and each bucket (a handful of entries) is grouped by the higher vertex. No per-edge objects, no
// hashing, no 2^24 Map cap.

export interface EdgeTopo {
  /** Edge id per half-edge (3 per triangle slot: t*3+e runs from vertex e to vertex (e+1)%3);
   * -1 for a half-edge of a skipped triangle. */
  eid: Int32Array;
  /** Number of distinct undirected edges. */
  nE: number;
  /** Per edge: number of half-edges on it, saturating at 255. */
  cnt: Uint8Array;
  /** Per edge: the first and second triangle on it, in ascending half-edge order (-1 = none). */
  t0: Int32Array;
  t1: Int32Array;
  /** Per edge with more than two half-edges: the third and later triangles, ascending. */
  extra: Map<number, number[]>;
  /** Edge id of the undirected pair (a,b), or -1 when no triangle has that edge. */
  find(a: number, b: number): number;
}

/** Buckets longer than this are grouped by sorting instead of pairwise comparison. */
const SORT_ABOVE = 24;

/**
 * Build the incidence of `nt` triangle slots stored as vertex triples in `I` (vertex ids below
 * `nv`). `skip`, when given, marks triangle slots to leave out (dead slots, excluded bodies).
 * Triangles of one edge are reported in ascending half-edge order — the order a scan over the
 * triangle list meets them — so a consumer that used to fill per-edge lists by such a scan sees
 * the identical sequence.
 */
export function buildEdgeTopo(I: ArrayLike<number>, nt: number, nv: number, skip?: ArrayLike<number>): EdgeTopo {
  const nH = nt * 3;
  const start = new Int32Array(nv + 1);
  // 1. bucket sizes by lower vertex
  for (let t = 0; t < nt; t++) {
    if (skip && skip[t]) continue;
    const o = t * 3;
    const a = I[o]!, b = I[o + 1]!, c = I[o + 2]!;
    start[(a < b ? a : b) + 1]!++;
    start[(b < c ? b : c) + 1]!++;
    start[(c < a ? c : a) + 1]!++;
  }
  for (let v = 0; v < nv; v++) start[v + 1]! += start[v]!;
  const total = start[nv]!;
  // 2. fill each bucket from its END while walking the half-edges backwards, so a bucket ends up
  //    in ascending half-edge order (bucket v = [start[v], start[v+1])).
  const fill = start.slice(1); // fill[v] = end of bucket v
  const other = new Int32Array(total);
  const heAt = new Int32Array(total);
  for (let t = nt - 1; t >= 0; t--) {
    if (skip && skip[t]) continue;
    const o = t * 3;
    const a = I[o]!, b = I[o + 1]!, c = I[o + 2]!;
    let p: number;
    if (c < a) { p = --fill[c]!; other[p] = a; } else { p = --fill[a]!; other[p] = c; }
    heAt[p] = o + 2;
    if (b < c) { p = --fill[b]!; other[p] = c; } else { p = --fill[c]!; other[p] = b; }
    heAt[p] = o + 1;
    if (a < b) { p = --fill[a]!; other[p] = b; } else { p = --fill[b]!; other[p] = a; }
    heAt[p] = o;
  }
  // 3. group each bucket by the higher vertex
  const eid = new Int32Array(nH).fill(-1);
  const eidAt = new Int32Array(total).fill(-1); // edge id per bucket position (for find)
  const cnt = new Uint8Array(total);
  const t0 = new Int32Array(total).fill(-1), t1 = new Int32Array(total).fill(-1);
  const extra = new Map<number, number[]>();
  let nE = 0;
  let order: number[] | null = null;
  for (let v = 0; v < nv; v++) {
    const s = start[v]!, e = start[v + 1]!;
    const k = e - s;
    if (k === 0) continue;
    if (k <= SORT_ABOVE) {
      for (let i = s; i < e; i++) {
        if (eidAt[i]! >= 0) continue;
        const id = nE++;
        const hi = other[i]!;
        eidAt[i] = id; eid[heAt[i]!] = id;
        t0[id] = (heAt[i]! / 3) | 0;
        let c = 1;
        for (let j = i + 1; j < e; j++) {
          if (other[j] !== hi) continue;
          eidAt[j] = id; eid[heAt[j]!] = id;
          const tj = (heAt[j]! / 3) | 0;
          if (c === 1) t1[id] = tj;
          else { const x = extra.get(id); if (x) x.push(tj); else extra.set(id, [tj]); }
          if (c < 255) c++;
        }
        cnt[id] = c;
      }
    } else {
      // A fan apex or pole carries thousands of half-edges: sort the positions by (higher vertex,
      // position) — position order IS ascending half-edge order — and sweep the runs.
      order ??= [];
      order.length = k;
      for (let i = 0; i < k; i++) order[i] = s + i;
      order.sort((x, y) => (other[x]! - other[y]!) || (x - y));
      for (let i = 0; i < k;) {
        let j = i + 1;
        while (j < k && other[order[j]!] === other[order[i]!]) j++;
        const id = nE++;
        let c = 0;
        for (let q = i; q < j; q++) {
          const p = order[q]!;
          eidAt[p] = id; eid[heAt[p]!] = id;
          const tq = (heAt[p]! / 3) | 0;
          if (c === 0) t0[id] = tq;
          else if (c === 1) t1[id] = tq;
          else { const x = extra.get(id); if (x) x.push(tq); else extra.set(id, [tq]); }
          if (c < 255) c++;
        }
        cnt[id] = c;
        i = j;
      }
    }
  }
  const find = (a: number, b: number): number => {
    const lo = a < b ? a : b, hi = a < b ? b : a;
    if (lo < 0 || lo >= nv) return -1;
    for (let i = start[lo]!, e = start[lo + 1]!; i < e; i++) if (other[i] === hi) return eidAt[i]!;
    return -1;
  };
  return { eid, nE, cnt, t0, t1, extra, find };
}
