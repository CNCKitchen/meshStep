// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — EdgeTable: an open-addressing hash table over undirected vertex-pair (edge) keys.
// V8 hard-caps Map/Set at 2^24 (~16.7M) entries, and a whole-assembly mesh easily carries more
// unique edges than that (wallganizer: 1.4m of shelving at 0.02mm deviation) — every per-edge
// Map in the pipeline died with "RangeError: Map maximum size exceeded". Typed arrays have no
// such cap and use a fraction of a Map's memory at these sizes.
//
// A slot carries a saturating use-count (`cnt`) plus up to two caller-owned Int32 lanes
// (`v0`/`v1` — first/second incident triangle, first face id, flag bits…). Vertex ids must stay
// below 2^26 (~67M) so the packed pair stays exact in a double — the same bound the previous
// Map keys used.

const KEY = 0x4000000; // packing base: min(a,b) * KEY + max(a,b) < 2^53 for ids < 2^26

export class EdgeTable {
  private keys: Float64Array; // packed undirected pair; -1 = empty slot
  private mask: number;
  private growAt: number;
  private lanes: 0 | 1 | 2;
  /** Occupied-slot count. */
  size = 0;
  /** Saturating per-edge use-count, 0 on an empty/never-bumped slot (255 = "many"). */
  cnt: Uint8Array;
  /** First caller lane (lanes >= 1), -1 initialised. */
  v0: Int32Array;
  /** Second caller lane (lanes = 2), -1 initialised. */
  v1: Int32Array;

  constructor(expectedEdges: number, lanes: 0 | 1 | 2 = 0) {
    let cap = 64;
    while (cap * 0.7 < expectedEdges) cap *= 2;
    this.lanes = lanes;
    this.mask = cap - 1;
    this.growAt = (cap * 0.7) | 0;
    this.keys = new Float64Array(cap).fill(-1);
    this.cnt = new Uint8Array(cap);
    this.v0 = lanes >= 1 ? new Int32Array(cap).fill(-1) : new Int32Array(0);
    this.v1 = lanes >= 2 ? new Int32Array(cap).fill(-1) : new Int32Array(0);
  }

  /** Slot count (for iterating occupied slots via cnt[s] > 0). */
  get capacity(): number { return this.keys.length; }

  private static hash(k: number, mask: number): number {
    // Split the <2^53 integer into 32-bit halves (exact for integers) and mix.
    const lo = k >>> 0, hi = (k / 4294967296) >>> 0;
    let h = Math.imul(lo, 0x9e3779b1) ^ Math.imul(hi, 0x85ebca6b);
    h ^= h >>> 15;
    h = Math.imul(h, 0x2c1b3c6d);
    h ^= h >>> 12;
    return h & mask;
  }

  /** Slot of edge (a,b), inserting an empty slot (cnt 0, lanes -1) when absent. */
  slot(a: number, b: number): number {
    if (this.size >= this.growAt) this.rehash();
    const k = a < b ? a * KEY + b : b * KEY + a;
    const keys = this.keys, mask = this.mask;
    let s = EdgeTable.hash(k, mask);
    while (keys[s] !== k) {
      if (keys[s] === -1) { keys[s] = k; this.size++; return s; }
      s = (s + 1) & mask;
    }
    return s;
  }

  /** Slot of edge (a,b), or -1 when absent. */
  find(a: number, b: number): number {
    const k = a < b ? a * KEY + b : b * KEY + a;
    const keys = this.keys, mask = this.mask;
    let s = EdgeTable.hash(k, mask);
    while (keys[s] !== k) {
      if (keys[s] === -1) return -1;
      s = (s + 1) & mask;
    }
    return s;
  }

  /** Insert-or-find + saturating count increment. Returns the slot (read cnt[slot] for the count). */
  bump(a: number, b: number): number {
    const s = this.slot(a, b);
    if (this.cnt[s]! < 255) this.cnt[s] = this.cnt[s]! + 1;
    return s;
  }

  private rehash(): void {
    const oldKeys = this.keys, oldCnt = this.cnt, oldV0 = this.v0, oldV1 = this.v1;
    const cap = oldKeys.length * 2;
    const mask = cap - 1;
    const keys = new Float64Array(cap).fill(-1);
    const cnt = new Uint8Array(cap);
    const v0 = this.lanes >= 1 ? new Int32Array(cap).fill(-1) : this.v0;
    const v1 = this.lanes >= 2 ? new Int32Array(cap).fill(-1) : this.v1;
    for (let i = 0; i < oldKeys.length; i++) {
      const k = oldKeys[i]!;
      if (k === -1) continue;
      let s = EdgeTable.hash(k, mask);
      while (keys[s] !== -1) s = (s + 1) & mask;
      keys[s] = k;
      cnt[s] = oldCnt[i]!;
      if (this.lanes >= 1) v0[s] = oldV0[i]!;
      if (this.lanes >= 2) v1[s] = oldV1[i]!;
    }
    this.keys = keys; this.cnt = cnt; this.v0 = v0; this.v1 = v1;
    this.mask = mask;
    this.growAt = (cap * 0.7) | 0;
  }
}
