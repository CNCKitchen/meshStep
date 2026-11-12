// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — growth-limited mesh size field (isotropic mode).
//
// A set of point SOURCES (position, size) defines the largest edge length allowed anywhere:
//
//     h(x) = min( hMax, min_i( h_i + g · |x − x_i| ) )        clamped to >= hMin
//
// i.e. every source demands its own size at its own position and lets the size GROW linearly with
// distance at the rate g (g = growthRate − 1: neighbouring triangles may differ by the factor
// growthRate). This is the exact gradient-limited field for a point-source set (Persson 2006,
// "PDE-based gradient limiting for mesh size functions", the closed form for point sources): it
// needs no background grid and no propagation over a mesh, and because it depends on the 3D
// POSITION only, the two faces sharing an edge always read the same size along that edge.
//
// Queries run over a static kd-tree with branch-and-bound: a subtree whose smallest source size
// plus g times the distance to its bounding box cannot beat the best value found so far is
// skipped. The nearer child is visited first, so the bound tightens quickly and a query touches
// a handful of leaves regardless of how many sources the body has.

const LEAF = 8;

export class SizeField {
  /** Size growth per unit distance (growthRate − 1). */
  readonly g: number;
  readonly hMin: number;
  readonly hMax: number;
  private stage: number[] = []; // x, y, z, h per staged source (before build)
  private px = new Float64Array(0);
  private ph = new Float64Array(0);
  // kd-tree, one entry per node. Leaves hold sources [lo, hi); inner nodes have two children.
  private nLo: number[] = [];
  private nHi: number[] = [];
  private nLeft: number[] = [];
  private nRight: number[] = [];
  private nMinH: number[] = [];
  private nBox: number[] = []; // 6 per node: min xyz, max xyz
  private stack = new Int32Array(64);

  constructor(growthRate: number, hMin: number, hMax: number) {
    this.g = Math.max(1e-3, growthRate - 1);
    this.hMin = hMin;
    this.hMax = hMax;
  }

  /** Number of sources in the built field. */
  get size(): number { return this.ph.length; }

  /** Stage a source. Sources at or above hMax constrain nothing and are dropped. */
  add(x: number, y: number, z: number, h: number): void {
    if (!(h < this.hMax) || !Number.isFinite(x + y + z)) return;
    this.stage.push(x, y, z, Math.max(h, this.hMin));
  }

  /** Build the tree from the staged sources. DOMINATED sources (another source already implies
   * a smaller size at their position) can never win a query, so they are dropped first: a fine
   * fillet rail makes the coarse sources of every edge that runs into it redundant near the
   * junction, and without them the tree is smaller and its bounds tighter. */
  build(): void {
    this.index(this.stage);
    const st = this.stage, keep: number[] = [];
    for (let i = 0; i < st.length; i += 4) {
      const h = st[i + 3]!;
      if (this.at(st[i]!, st[i + 1]!, st[i + 2]!) >= h * (1 - 1e-9)) keep.push(st[i]!, st[i + 1]!, st[i + 2]!, h);
    }
    if (keep.length < st.length) this.index(keep);
    this.stage = [];
  }

  private index(src: number[]): void {
    const n = src.length / 4;
    const order = new Int32Array(n);
    for (let i = 0; i < n; i++) order[i] = i;
    this.nLo = []; this.nHi = []; this.nLeft = []; this.nRight = []; this.nMinH = []; this.nBox = [];
    const coord = (i: number, ax: number): number => src[i * 4 + ax]!;
    // Iterative build; each work item is a node id with its source range already recorded.
    const make = (lo: number, hi: number): number => {
      const id = this.nLo.length;
      let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity, mh = Infinity;
      for (let k = lo; k < hi; k++) {
        const o = order[k]! * 4;
        const x = src[o]!, y = src[o + 1]!, z = src[o + 2]!, h = src[o + 3]!;
        if (x < x0) x0 = x; if (x > x1) x1 = x;
        if (y < y0) y0 = y; if (y > y1) y1 = y;
        if (z < z0) z0 = z; if (z > z1) z1 = z;
        if (h < mh) mh = h;
      }
      this.nLo.push(lo); this.nHi.push(hi); this.nLeft.push(-1); this.nRight.push(-1); this.nMinH.push(mh);
      this.nBox.push(x0, y0, z0, x1, y1, z1);
      return id;
    };
    const work: number[] = n > 0 ? [make(0, n)] : [];
    while (work.length) {
      const id = work.pop()!;
      const lo = this.nLo[id]!, hi = this.nHi[id]!;
      if (hi - lo <= LEAF) continue;
      const b = id * 6;
      const ex = this.nBox[b + 3]! - this.nBox[b]!, ey = this.nBox[b + 4]! - this.nBox[b + 1]!, ez = this.nBox[b + 5]! - this.nBox[b + 2]!;
      const ax = ex >= ey && ex >= ez ? 0 : ey >= ez ? 1 : 2;
      if (!(Math.max(ex, ey, ez) > 0)) continue; // all sources coincide — keep as one leaf
      // Median split on the longest box axis. The subarray sort is O(n log n) per level; the
      // source count of one body (a few per boundary node) keeps the whole build in milliseconds.
      const sub = Array.from(order.subarray(lo, hi)).sort((p, q) => coord(p, ax) - coord(q, ax));
      for (let k = 0; k < sub.length; k++) order[lo + k] = sub[k]!;
      const mid = (lo + hi) >> 1;
      const l = make(lo, mid), r = make(mid, hi);
      this.nLeft[id] = l; this.nRight[id] = r;
      work.push(l, r);
    }
    // Sources in tree order, so a leaf scans contiguous memory.
    this.px = new Float64Array(n * 3);
    this.ph = new Float64Array(n);
    for (let k = 0; k < n; k++) {
      const o = order[k]! * 4;
      this.px[k * 3] = src[o]!; this.px[k * 3 + 1] = src[o + 1]!; this.px[k * 3 + 2] = src[o + 2]!;
      this.ph[k] = src[o + 3]!;
    }
    let depth = 2;
    for (let m = n; m > LEAF; m >>= 1) depth++;
    if (this.stack.length < 2 * depth + 8) this.stack = new Int32Array(2 * depth + 8);
  }

  /** Lower bound of any source value inside node `id` as seen from (x,y,z). */
  private bound(id: number, x: number, y: number, z: number): number {
    const b = id * 6, box = this.nBox;
    const dx = x < box[b]! ? box[b]! - x : x > box[b + 3]! ? x - box[b + 3]! : 0;
    const dy = y < box[b + 1]! ? box[b + 1]! - y : y > box[b + 4]! ? y - box[b + 4]! : 0;
    const dz = z < box[b + 2]! ? box[b + 2]! - z : z > box[b + 5]! ? z - box[b + 5]! : 0;
    return this.nMinH[id]! + this.g * Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  /** Allowed edge length at a point (mm). */
  at(x: number, y: number, z: number): number {
    let best = this.hMax;
    if (this.nLo.length === 0) return best;
    const g = this.g, st = this.stack, px = this.px, ph = this.ph;
    let sp = 0;
    st[sp++] = 0;
    while (sp > 0) {
      const id = st[--sp]!;
      if (this.bound(id, x, y, z) >= best) continue;
      const l = this.nLeft[id]!;
      if (l < 0) {
        for (let k = this.nLo[id]!, e = this.nHi[id]!; k < e; k++) {
          const lim = best - ph[k]!;
          if (lim <= 0) continue;
          const dx = px[k * 3]! - x, dy = px[k * 3 + 1]! - y, dz = px[k * 3 + 2]! - z;
          const d2 = dx * dx + dy * dy + dz * dz;
          if (g * g * d2 < lim * lim) best = ph[k]! + g * Math.sqrt(d2);
        }
        continue;
      }
      const r = this.nRight[id]!;
      // Push the farther child first so the nearer one is popped (and tightens `best`) first.
      if (this.bound(l, x, y, z) <= this.bound(r, x, y, z)) { st[sp++] = r; st[sp++] = l; }
      else { st[sp++] = l; st[sp++] = r; }
    }
    return best < this.hMin ? this.hMin : best;
  }
}
