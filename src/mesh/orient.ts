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
import { buildEdgeTopo } from "./edge-topo.ts";

export function orientConsistent(mesh: IndexedMesh, solidOfTri?: ArrayLike<number>): void {
  const idx = mesh.indices;
  const pos = mesh.positions;
  const nt = idx.length / 3;

  // Undirected edge -> incident triangles (topology is fixed; only windings change as we flip).
  // The hash-free incidence (edge-topo.ts) gives each half-edge its edge id, and each edge its
  // use-count and first two triangles; the rare 3rd+ triangle of a non-manifold edge is in `extra`.
  const { eid, cnt, t0, t1, extra } = buildEdgeTopo(idx, nt, pos.length / 3);
  // Flipping (a,b,c) -> (a,c,b) swaps which undirected edge half-edges 0 and 2 lie on.
  const flip = (t: number): void => {
    const i = t * 3 + 1, j = t * 3 + 2; const tmp = idx[i]!; idx[i] = idx[j]!; idx[j] = tmp;
    const e0 = eid[t * 3]!; eid[t * 3] = eid[j]!; eid[j] = e0;
  };
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
        const s = eid[t * 3 + e]!;
        if (cnt[s] !== 2) continue; // open or non-manifold edge — no orientation signal
        const a = idx[t * 3 + e]!, b = idx[t * 3 + ((e + 1) % 3)]!;
        const nb = t0[s] === t ? t1[s]! : t0[s]!;
        if (hasDir(nb, b, a)) agree++; else disagree++;
      }
      if (disagree > agree) { flip(t); flips++; }
    }
    if (flips === 0) break;
  }

  // Outward orientation: flip a whole CLOSED component (every edge manifold) so it encloses
  // positive signed volume — or NEGATIVE when the component is a CAVITY. A hollow body arrives as
  // several disjoint closed shells (Stealthburner: six internal void pockets inside the wall);
  // orienting every shell outward silently ADDS each void's volume instead of subtracting it
  // (+1.2% on a part whose surfaces measure exact). Nesting is decided by the generalized winding
  // number of a shell's sample point against every OTHER closed component OF THE SAME SOLID whose
  // bbox contains it: odd containment count = cavity = inward. The same-solid restriction is
  // essential — this pass runs BEFORE assembly placement, where every part's mesh sits in its own
  // LOCAL frame around the origin, so different solids overlap arbitrarily (treating an assembly
  // part as another part's cavity cost Ontos 32% and bottle-cage 92% of their volume). A cavity is
  // by definition a shell of its own body. Single-shell solids never enter the winding path, and a
  // point exactly ON a sibling surface measures |w| ≈ 0.5 and stays "outside".
  const signedVol6 = (t: number): number => {
    const a = idx[t * 3]! * 3, b = idx[t * 3 + 1]! * 3, c = idx[t * 3 + 2]! * 3;
    const ax = pos[a]!, ay = pos[a + 1]!, az = pos[a + 2]!;
    const bx = pos[b]!, by = pos[b + 1]!, bz = pos[b + 2]!;
    const cx = pos[c]!, cy = pos[c + 1]!, cz = pos[c + 2]!;
    return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
  };
  interface Comp { tris: number[]; vol: number; box: [number, number, number, number, number, number]; pt: [number, number, number]; solid: number }
  const comps: Comp[] = [];
  const seen = new Uint8Array(nt);
  for (let seed = 0; seed < nt; seed++) {
    if (seen[seed]) continue;
    const comp: number[] = [];
    const stack = [seed];
    seen[seed] = 1;
    let closed = true;
    const visit = (nb: number): void => {
      if (seen[nb]) return;
      seen[nb] = 1;
      stack.push(nb);
    };
    while (stack.length) {
      const t = stack.pop()!;
      comp.push(t);
      for (let e = 0; e < 3; e++) {
        const s = eid[t * 3 + e]!;
        const c = cnt[s]!;
        // A boundary edge (count 1) means an open shell — no meaningful signed volume. An EVEN
        // count > 2 is a self-touching solid (two coincident B-rep edges welded, Stealthburner's
        // tangent-contact line): still a waterproof volume, keep it flippable and — crucially —
        // available as a nesting ENCLOSURE for its cavity shells.
        if (c === 1) closed = false;
        visit(t0[s]!);
        if (c >= 2) visit(t1[s]!);
        if (c > 2) for (const nb of extra.get(s)!) visit(nb);
      }
    }
    if (!closed) continue;
    let vol = 0;
    const box: [number, number, number, number, number, number] = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
    let px = -Infinity, py = 0, pz = 0;
    for (const t of comp) {
      vol += signedVol6(t);
      for (let e = 0; e < 3; e++) {
        const v = idx[t * 3 + e]! * 3;
        const x = pos[v]!, y = pos[v + 1]!, z = pos[v + 2]!;
        if (x < box[0]) box[0] = x; if (y < box[1]) box[1] = y; if (z < box[2]) box[2] = z;
        if (x > box[3]) box[3] = x; if (y > box[4]) box[4] = y; if (z > box[5]) box[5] = z;
        if (x > px) { px = x; py = y; pz = z; }
      }
    }
    comps.push({ tris: comp, vol, box, pt: [px, py, pz], solid: solidOfTri?.[comp[0]!] ?? 0 });
  }
  // Solid-angle winding of component c around point p (van Oosterom–Strackee), in full turns.
  const windingOf = (c: Comp, p: [number, number, number]): number => {
    let w = 0;
    for (const t of c.tris) {
      const a = idx[t * 3]! * 3, b = idx[t * 3 + 1]! * 3, cc = idx[t * 3 + 2]! * 3;
      const ax = pos[a]! - p[0], ay = pos[a + 1]! - p[1], az = pos[a + 2]! - p[2];
      const bx = pos[b]! - p[0], by = pos[b + 1]! - p[1], bz = pos[b + 2]! - p[2];
      const cx = pos[cc]! - p[0], cy = pos[cc + 1]! - p[1], cz = pos[cc + 2]! - p[2];
      const la = Math.hypot(ax, ay, az), lb = Math.hypot(bx, by, bz), lc = Math.hypot(cx, cy, cz);
      const num = ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
      const den = la * lb * lc + (ax * bx + ay * by + az * bz) * lc + (bx * cx + by * cy + bz * cz) * la + (cx * ax + cy * ay + cz * az) * lb;
      w += 2 * Math.atan2(num, den);
    }
    return w / (4 * Math.PI);
  };
  for (const c of comps) {
    let parity = 0;
    if (comps.length > 1 && solidOfTri) {
      for (const o of comps) {
        if (o === c || o.solid !== c.solid) continue;
        const [x, y, z] = c.pt;
        if (x < o.box[0] || y < o.box[1] || z < o.box[2] || x > o.box[3] || y > o.box[4] || z > o.box[5]) continue;
        if (Math.abs(windingOf(o, c.pt)) > 0.75) parity++;
      }
    }
    const wantNegative = (parity & 1) === 1;
    if (wantNegative ? c.vol > 0 : c.vol < 0) for (const t of c.tris) flip(t);
  }
}
