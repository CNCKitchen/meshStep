// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — make a mesh consistently oriented and outward-facing. The tessellator already emits
// every triangle outward via its analytic surface normal, so the input is essentially correct; this
// pass only cleans up stray flips (from remesh smoothing, or an unreliable B-spline normal) and
// guarantees outward-facing closed bodies.
//
// It does NOT rebuild orientation by seeded flood-fill: that propagates a flip across any bad weld
// (a thin wall whose two sides share vertices, leaving the mesh locally non-orientable) and cascades
// a handful of local errors into thousands of flipped triangles. Instead it flips a triangle only
// when it disagrees with the MAJORITY of its manifold neighbours — local consensus can't cascade —
// and only flips a whole body outward when that body is closed (an open shell has no meaningful
// signed volume).
import type { IndexedMesh } from "../io/stl.ts";

const KEY = 0x4000000; // supports up to ~67M vertices

export function orientConsistent(mesh: IndexedMesh): void {
  const idx = mesh.indices;
  const pos = mesh.positions;
  const nt = idx.length / 3;
  const ekey = (a: number, b: number): number => (a < b ? a * KEY + b : b * KEY + a);

  // Undirected edge -> incident triangles (topology is fixed; only windings change as we flip).
  const edge = new Map<number, number[]>();
  for (let t = 0; t < nt; t++) {
    for (let e = 0; e < 3; e++) {
      const k = ekey(idx[t * 3 + e]!, idx[t * 3 + ((e + 1) % 3)]!);
      const arr = edge.get(k);
      if (arr) arr.push(t); else edge.set(k, [t]);
    }
  }
  const flip = (t: number): void => { const i = t * 3 + 1, j = t * 3 + 2; const tmp = idx[i]!; idx[i] = idx[j]!; idx[j] = tmp; };
  const hasDir = (t: number, x: number, y: number): boolean => {
    for (let e = 0; e < 3; e++) if (idx[t * 3 + e] === x && idx[t * 3 + ((e + 1) % 3)] === y) return true;
    return false;
  };

  // Local-consensus consistency: repeatedly flip any triangle that disagrees with most of its
  // manifold neighbours. A neighbour agrees when it traverses the shared edge the opposite way.
  for (let iter = 0; iter < 8; iter++) {
    let flips = 0;
    for (let t = 0; t < nt; t++) {
      let agree = 0, disagree = 0;
      for (let e = 0; e < 3; e++) {
        const a = idx[t * 3 + e]!, b = idx[t * 3 + ((e + 1) % 3)]!;
        const inc = edge.get(ekey(a, b))!;
        if (inc.length !== 2) continue; // open or non-manifold edge — no orientation signal
        const nb = inc[0] === t ? inc[1]! : inc[0]!;
        if (hasDir(nb, b, a)) agree++; else disagree++;
      }
      if (disagree > agree) { flip(t); flips++; }
    }
    if (flips === 0) break;
  }

  // Outward orientation: flip a whole connected component if it is CLOSED (every edge manifold) and
  // encloses negative signed volume. Open shells are left as the tessellator oriented them.
  const signedVol6 = (t: number): number => {
    const a = idx[t * 3]! * 3, b = idx[t * 3 + 1]! * 3, c = idx[t * 3 + 2]! * 3;
    const ax = pos[a]!, ay = pos[a + 1]!, az = pos[a + 2]!;
    const bx = pos[b]!, by = pos[b + 1]!, bz = pos[b + 2]!;
    const cx = pos[c]!, cy = pos[c + 1]!, cz = pos[c + 2]!;
    return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
  };
  const seen = new Uint8Array(nt);
  for (let seed = 0; seed < nt; seed++) {
    if (seen[seed]) continue;
    const comp: number[] = [];
    const stack = [seed];
    seen[seed] = 1;
    let closed = true;
    while (stack.length) {
      const t = stack.pop()!;
      comp.push(t);
      for (let e = 0; e < 3; e++) {
        const a = idx[t * 3 + e]!, b = idx[t * 3 + ((e + 1) % 3)]!;
        const inc = edge.get(ekey(a, b))!;
        if (inc.length !== 2) closed = false; // boundary or non-manifold => not a closed body
        for (const nb of inc) {
          if (seen[nb]) continue;
          seen[nb] = 1;
          stack.push(nb);
        }
      }
    }
    if (!closed) continue;
    let vol = 0;
    for (const t of comp) vol += signedVol6(t);
    if (vol < 0) for (const t of comp) flip(t);
  }
}
