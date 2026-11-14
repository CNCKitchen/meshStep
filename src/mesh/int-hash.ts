// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — small open-addressing tables over numeric keys, for the hot loops that used to build
// a string per lookup ("x,y,z" vertex keys, "a,b,c" triangle keys) and probe a Map/Set with it.
// Both tables compare the full key, so they answer exactly what the string-keyed ones did.

const mix = (h: number): number => {
  h ^= h >>> 15; h = Math.imul(h, 0x2c1b3c6d); h ^= h >>> 12;
  return h;
};

/**
 * Dense ids for (already quantised) coordinate triples: the first triple seen gets id 0, the
 * next new one id 1, and so on. Two triples are the same when all three numbers are equal
 * (-0 equals 0, NaN equals NaN — as their decimal strings would be).
 */
export class PointIds {
  private slots: Int32Array; // id + 1; 0 = empty
  private mask: number;
  /** Quantised coordinates of each id (3 per id). */
  q: Float64Array;
  /** Number of ids handed out. */
  count = 0;

  constructor(expected: number) {
    let cap = 64;
    while (cap < expected * 2) cap *= 2;
    this.slots = new Int32Array(cap);
    this.mask = cap - 1;
    this.q = new Float64Array(Math.max(16, expected) * 3);
  }

  private static hash(x: number, y: number, z: number): number {
    return mix(Math.imul(x | 0, 73856093) ^ Math.imul(y | 0, 19349663) ^ Math.imul(z | 0, 83492791));
  }

  /** Id of the triple, assigning the next free id when it is new. (NaN never equals itself, but
   * the string key this replaces, "NaN", did: the comparison below treats two NaNs as equal.) */
  idOf(x: number, y: number, z: number): number {
    const slots = this.slots, mask = this.mask;
    let q = this.q;
    let s = PointIds.hash(x, y, z) & mask;
    for (;;) {
      const v = slots[s]!;
      if (v === 0) break;
      const w = (v - 1) * 3;
      const qx = q[w]!, qy = q[w + 1]!, qz = q[w + 2]!;
      if ((qx === x || (qx !== qx && x !== x)) && (qy === y || (qy !== qy && y !== y)) && (qz === z || (qz !== qz && z !== z))) return v - 1;
      s = (s + 1) & mask;
    }
    const id = this.count++;
    if (this.count * 3 > q.length) { const nq = new Float64Array(q.length * 2); nq.set(q); q = this.q = nq; }
    const w = id * 3;
    q[w] = x; q[w + 1] = y; q[w + 2] = z;
    slots[s] = id + 1;
    if (this.count * 2 > slots.length) this.grow();
    return id;
  }

  private grow(): void {
    const cap = this.slots.length * 2, mask = cap - 1;
    const slots = new Int32Array(cap);
    const q = this.q;
    for (let k = 0; k < this.count; k++) {
      let r = PointIds.hash(q[k * 3]!, q[k * 3 + 1]!, q[k * 3 + 2]!) & mask;
      while (slots[r] !== 0) r = (r + 1) & mask;
      slots[r] = k + 1;
    }
    this.slots = slots; this.mask = mask;
  }
}

/** A set of integer triples (non-negative ids). The caller passes each triple in one canonical
 * order (e.g. sorted), as it would have when building a string key. */
export class TripleSet {
  private a: Int32Array; private b: Int32Array; private c: Int32Array; // c holds value + 1; 0 = empty
  private mask: number;
  size = 0;

  constructor(expected = 0) {
    let cap = 64;
    while (cap < expected * 2) cap *= 2;
    this.a = new Int32Array(cap); this.b = new Int32Array(cap); this.c = new Int32Array(cap);
    this.mask = cap - 1;
  }

  private static hash(x: number, y: number, z: number): number {
    return mix(Math.imul(x, 73856093) ^ Math.imul(y, 19349663) ^ Math.imul(z, 83492791));
  }

  has(x: number, y: number, z: number): boolean {
    const a = this.a, b = this.b, c = this.c, mask = this.mask;
    let s = TripleSet.hash(x, y, z) & mask;
    while (c[s] !== 0) {
      if (a[s] === x && b[s] === y && c[s] === z + 1) return true;
      s = (s + 1) & mask;
    }
    return false;
  }

  add(x: number, y: number, z: number): void {
    const a = this.a, b = this.b, c = this.c, mask = this.mask;
    let s = TripleSet.hash(x, y, z) & mask;
    while (c[s] !== 0) {
      if (a[s] === x && b[s] === y && c[s] === z + 1) return;
      s = (s + 1) & mask;
    }
    a[s] = x; b[s] = y; c[s] = z + 1;
    if (++this.size * 2 > c.length) this.grow();
  }

  private grow(): void {
    const oa = this.a, ob = this.b, oc = this.c;
    const cap = oc.length * 2, mask = cap - 1;
    const a = new Int32Array(cap), b = new Int32Array(cap), c = new Int32Array(cap);
    for (let i = 0; i < oc.length; i++) {
      if (oc[i] === 0) continue;
      let s = TripleSet.hash(oa[i]!, ob[i]!, oc[i]! - 1) & mask;
      while (c[s] !== 0) s = (s + 1) & mask;
      a[s] = oa[i]!; b[s] = ob[i]!; c[s] = oc[i]!;
    }
    this.a = a; this.b = b; this.c = c; this.mask = mask;
  }
}
