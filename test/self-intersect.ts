// SPDX-License-Identifier: AGPL-3.0-only
// Triangle-triangle self-intersection audit for a converted mesh. A mesher that FOLDS a face (covers
// part of it two or three times) leaves every other check green: the folded layers lie on the true
// surface (deviation ~0), the topology stays closed (watertight), the winding stays consistent
// (orientation passes) — only the volume drifts, and in an assembly that drift is diluted to nothing.
// The fold shows up as triangles of the SAME face crossing each other, which a correct
// tessellation of a valid face can never produce. Cross-face pairs are reported separately: they can
// also come from near-tangent faces meeting at a chord-sized gap (OCC's own mesh has a few).
//
//   import { selfIntersections } from "./self-intersect.ts";
//   const r = selfIntersections(mesh.positions, mesh.indices, faceOfTri);
//   r.sameFace  // pairs inside one CAD face — a mesher fold, always a bug
//   r.crossFace // pairs between different faces — inspect
//
// Pairs sharing a vertex are skipped (adjacent triangles touch by construction). Broad phase is a
// uniform grid sized from the median triangle extent; narrow phase tests each triangle's edges as
// segments against the other triangle with a small relative epsilon, so coplanar touching and
// shared-edge contact do not count.

export interface SelfIntersectionReport {
  sameFace: number;
  crossFace: number;
  /** Pair counts per face id (same-face) or "a-b" face pair (cross-face), most first. */
  byFace: [string, number][];
}

export function selfIntersections(
  P: ArrayLike<number>, I: ArrayLike<number>, faceOfTri?: ArrayLike<number>,
): SelfIntersectionReport {
  const nT = I.length / 3;
  const lo = new Float64Array(nT * 3), hi = new Float64Array(nT * 3);
  const ext: number[] = [];
  let gx0 = Infinity, gy0 = Infinity, gz0 = Infinity;
  for (let t = 0; t < nT; t++) {
    for (let k = 0; k < 3; k++) {
      const a = P[I[t * 3]! * 3 + k]!, b = P[I[t * 3 + 1]! * 3 + k]!, c = P[I[t * 3 + 2]! * 3 + k]!;
      lo[t * 3 + k] = Math.min(a, b, c); hi[t * 3 + k] = Math.max(a, b, c);
    }
    ext.push(Math.max(hi[t * 3]! - lo[t * 3]!, hi[t * 3 + 1]! - lo[t * 3 + 1]!, hi[t * 3 + 2]! - lo[t * 3 + 2]!));
    gx0 = Math.min(gx0, lo[t * 3]!); gy0 = Math.min(gy0, lo[t * 3 + 1]!); gz0 = Math.min(gz0, lo[t * 3 + 2]!);
  }
  const report: SelfIntersectionReport = { sameFace: 0, crossFace: 0, byFace: [] };
  if (nT === 0) return report;
  ext.sort((x, y) => x - y);
  const cell = Math.max(ext[nT >> 1]! * 2, 1e-9);
  // Grid: cell key -> triangle list. Keys pack the three cell coords with the grid's own extents.
  let gx1 = -Infinity, gy1 = -Infinity;
  for (let t = 0; t < nT; t++) { gx1 = Math.max(gx1, hi[t * 3]!); gy1 = Math.max(gy1, hi[t * 3 + 1]!); }
  const ci = (x: number, x0: number): number => Math.floor((x - x0) / cell);
  const NX = ci(gx1, gx0) + 1, NY = ci(gy1, gy0) + 1;
  const keyOf = (x: number, y: number, z: number): number => (z * NY + y) * NX + x;
  const grid = new Map<number, number[]>();
  for (let t = 0; t < nT; t++) {
    const x0 = ci(lo[t * 3]!, gx0), x1 = ci(hi[t * 3]!, gx0);
    const y0 = ci(lo[t * 3 + 1]!, gy0), y1 = ci(hi[t * 3 + 1]!, gy0);
    const z0 = ci(lo[t * 3 + 2]!, gz0), z1 = ci(hi[t * 3 + 2]!, gz0);
    for (let x = x0; x <= x1; x++) for (let y = y0; y <= y1; y++) for (let z = z0; z <= z1; z++) {
      const key = keyOf(x, y, z);
      let arr = grid.get(key); if (!arr) grid.set(key, arr = []); arr.push(t);
    }
  }
  const v = (i: number, k: number): number => P[i * 3 + k]!;
  // Segment pq vs triangle abc (Möller–Trumbore), strict interior on both with relative epsilon.
  const segTri = (p: number, q: number, a: number, b: number, c: number): boolean => {
    const e1x = v(b, 0) - v(a, 0), e1y = v(b, 1) - v(a, 1), e1z = v(b, 2) - v(a, 2);
    const e2x = v(c, 0) - v(a, 0), e2y = v(c, 1) - v(a, 1), e2z = v(c, 2) - v(a, 2);
    const dx = v(q, 0) - v(p, 0), dy = v(q, 1) - v(p, 1), dz = v(q, 2) - v(p, 2);
    const hx = dy * e2z - dz * e2y, hy = dz * e2x - dx * e2z, hz = dx * e2y - dy * e2x;
    const det = e1x * hx + e1y * hy + e1z * hz;
    const scale = Math.hypot(e1x, e1y, e1z) * Math.hypot(e2x, e2y, e2z) * Math.hypot(dx, dy, dz);
    if (Math.abs(det) <= 1e-12 * scale) return false; // parallel / coplanar
    const f = 1 / det, sx = v(p, 0) - v(a, 0), sy = v(p, 1) - v(a, 1), sz = v(p, 2) - v(a, 2);
    const u = f * (sx * hx + sy * hy + sz * hz);
    const EPS = 1e-9;
    if (u < EPS || u > 1 - EPS) return false;
    const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
    const w = f * (dx * qx + dy * qy + dz * qz);
    if (w < EPS || u + w > 1 - EPS) return false;
    const s = f * (e2x * qx + e2y * qy + e2z * qz);
    return s > EPS && s < 1 - EPS;
  };
  const counts = new Map<string, number>();
  for (const [cellKey, arr] of grid) {
    for (let i = 0; i < arr.length; i++) for (let j = i + 1; j < arr.length; j++) {
      const s = Math.min(arr[i]!, arr[j]!), t = Math.max(arr[i]!, arr[j]!);
      if (lo[s * 3]! > hi[t * 3]! || lo[t * 3]! > hi[s * 3]! || lo[s * 3 + 1]! > hi[t * 3 + 1]! || lo[t * 3 + 1]! > hi[s * 3 + 1]!
        || lo[s * 3 + 2]! > hi[t * 3 + 2]! || lo[t * 3 + 2]! > hi[s * 3 + 2]!) continue;
      // Each pair shares several cells when the triangles are large; test it only in the cell that
      // holds the min corner of the two boxes' overlap, so every pair is counted exactly once.
      const own = keyOf(ci(Math.max(lo[s * 3]!, lo[t * 3]!), gx0), ci(Math.max(lo[s * 3 + 1]!, lo[t * 3 + 1]!), gy0),
        ci(Math.max(lo[s * 3 + 2]!, lo[t * 3 + 2]!), gz0));
      if (own !== cellKey) continue;
      const a0 = I[s * 3]!, a1 = I[s * 3 + 1]!, a2 = I[s * 3 + 2]!, b0 = I[t * 3]!, b1 = I[t * 3 + 1]!, b2 = I[t * 3 + 2]!;
      if (a0 === b0 || a0 === b1 || a0 === b2 || a1 === b0 || a1 === b1 || a1 === b2 || a2 === b0 || a2 === b1 || a2 === b2) continue;
      const hit = segTri(a0, a1, b0, b1, b2) || segTri(a1, a2, b0, b1, b2) || segTri(a2, a0, b0, b1, b2)
        || segTri(b0, b1, a0, a1, a2) || segTri(b1, b2, a0, a1, a2) || segTri(b2, b0, a0, a1, a2);
      if (!hit) continue;
      const fs = faceOfTri ? faceOfTri[s]! : 0, ft = faceOfTri ? faceOfTri[t]! : 0;
      const key = fs === ft ? String(fs) : fs < ft ? `${fs}-${ft}` : `${ft}-${fs}`;
      counts.set(key, (counts.get(key) ?? 0) + 1);
      if (fs === ft) report.sameFace++; else report.crossFace++;
    }
  }
  report.byFace = [...counts].sort((x, y) => y[1] - x[1]);
  return report;
}
