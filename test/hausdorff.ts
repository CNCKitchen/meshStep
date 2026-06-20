// SPDX-License-Identifier: AGPL-3.0-only
// Symmetric Hausdorff distance between two triangle soups (brute force, query-sampled).
import type { Vec3 } from "../src/geom/vec.ts";
import type { TriSoup } from "../src/io/stl.ts";

function closestOnTri(p: Vec3, a: Vec3, b: Vec3, c: Vec3): number {
  // Ericson, Real-Time Collision Detection — closest point on triangle to p.
  const abx = b[0] - a[0], aby = b[1] - a[1], abz = b[2] - a[2];
  const acx = c[0] - a[0], acy = c[1] - a[1], acz = c[2] - a[2];
  const apx = p[0] - a[0], apy = p[1] - a[1], apz = p[2] - a[2];
  const d1 = abx * apx + aby * apy + abz * apz;
  const d2 = acx * apx + acy * apy + acz * apz;
  if (d1 <= 0 && d2 <= 0) return Math.hypot(apx, apy, apz);

  const bpx = p[0] - b[0], bpy = p[1] - b[1], bpz = p[2] - b[2];
  const d3 = abx * bpx + aby * bpy + abz * bpz;
  const d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) return Math.hypot(bpx, bpy, bpz);

  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    return Math.hypot(apx - abx * v, apy - aby * v, apz - abz * v);
  }
  const cpx = p[0] - c[0], cpy = p[1] - c[1], cpz = p[2] - c[2];
  const d5 = abx * cpx + aby * cpy + abz * cpz;
  const d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) return Math.hypot(cpx, cpy, cpz);

  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    return Math.hypot(apx - acx * w, apy - acy * w, apz - acz * w);
  }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    // closest point is b + w*(c-b) = a + ab + w*(ac-ab)
    return Math.hypot(
      apx - (abx + (acx - abx) * w),
      apy - (aby + (acy - aby) * w),
      apz - (abz + (acz - abz) * w),
    );
  }
  // interior: project onto plane
  const denom = 1 / (va + vb + vc);
  const v = vb * denom, w = vc * denom;
  const qx = a[0] + abx * v + acx * w;
  const qy = a[1] + aby * v + acy * w;
  const qz = a[2] + abz * v + acz * w;
  return Math.hypot(p[0] - qx, p[1] - qy, p[2] - qz);
}

/** Directed distance: for sampled query points of A, the max/mean closest distance to B. */
function directed(A: TriSoup, B: TriSoup, maxQuery: number): { max: number; mean: number } {
  const qa = A.positions; // 9 per triangle => 3 verts
  const nQ = qa.length / 3;
  const stride = Math.max(1, Math.floor(nQ / maxQuery));
  const tb = B.positions;
  const nT = tb.length / 9;
  let max = 0;
  let sum = 0;
  let count = 0;
  for (let qi = 0; qi < nQ; qi += stride) {
    const p: Vec3 = [qa[qi * 3]!, qa[qi * 3 + 1]!, qa[qi * 3 + 2]!];
    let best = Infinity;
    for (let t = 0; t < nT; t++) {
      const o = t * 9;
      const d = closestOnTri(
        p,
        [tb[o]!, tb[o + 1]!, tb[o + 2]!],
        [tb[o + 3]!, tb[o + 4]!, tb[o + 5]!],
        [tb[o + 6]!, tb[o + 7]!, tb[o + 8]!],
      );
      if (d < best) best = d;
      if (best === 0) break;
    }
    if (best > max) max = best;
    sum += best;
    count++;
  }
  return { max, mean: count ? sum / count : 0 };
}

export function hausdorff(A: TriSoup, B: TriSoup, maxQuery = 3000): { max: number; mean: number } {
  const ab = directed(A, B, maxQuery);
  const ba = directed(B, A, maxQuery);
  return { max: Math.max(ab.max, ba.max), mean: (ab.mean + ba.mean) / 2 };
}
