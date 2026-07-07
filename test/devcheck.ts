// SPDX-License-Identifier: AGPL-3.0-only
// Deviation-settings regression check: with CAD-style settings (0.01mm / 10° / 100mm max edge),
// adjacent facets on the SAME analytic face must differ by ≤ 2·normalDev (20°). Before the fix,
// a huge max edge silently coarsened both tolerances (hole rims ran at 5 segments — 72° apart).
import { readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");

for (const name of ["cylinder", "cylinderWithHole", "roundedCube", "everything"]) {
  let src: string;
  try { src = readFileSync(join(root, `${name}.step`), "utf8"); }
  catch { continue; }
  const res = importStep(src, { surfaceDeviation: 0.01, normalDeviation: 10, maxEdge: 100 });
  const m = res.mesh;
  const p = m.positions;
  const nTri = m.indices.length / 3;

  // Per-triangle unit normal.
  const normals = new Float64Array(nTri * 3);
  for (let t = 0; t < nTri; t++) {
    const a = m.indices[t * 3]! * 3, b = m.indices[t * 3 + 1]! * 3, c = m.indices[t * 3 + 2]! * 3;
    const ux = p[b]! - p[a]!, uy = p[b + 1]! - p[a + 1]!, uz = p[b + 2]! - p[a + 2]!;
    const vx = p[c]! - p[a]!, vy = p[c + 1]! - p[a + 1]!, vz = p[c + 2]! - p[a + 2]!;
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    normals[t * 3] = nx / l; normals[t * 3 + 1] = ny / l; normals[t * 3 + 2] = nz / l;
  }

  // Adjacent pairs sharing an edge, same face only.
  const edgeTri = new Map<string, number>();
  let worst = 0;
  for (let t = 0; t < nTri; t++) {
    const vs = [m.indices[t * 3]!, m.indices[t * 3 + 1]!, m.indices[t * 3 + 2]!];
    for (let e = 0; e < 3; e++) {
      const a = vs[e]!, b = vs[(e + 1) % 3]!;
      const k = a < b ? `${a}_${b}` : `${b}_${a}`;
      const other = edgeTri.get(k);
      if (other === undefined) { edgeTri.set(k, t); continue; }
      if (res.faceOfTri[other] !== res.faceOfTri[t]) continue; // feature edge between faces
      const d = normals[other * 3]! * normals[t * 3]! + normals[other * 3 + 1]! * normals[t * 3 + 1]! + normals[other * 3 + 2]! * normals[t * 3 + 2]!;
      const ang = Math.acos(Math.max(-1, Math.min(1, d))) * 180 / Math.PI;
      if (ang > worst) worst = ang;
    }
  }
  console.log(`${name.padEnd(18)} tris=${String(nTri).padStart(7)}  worst same-face adjacent-normal angle: ${worst.toFixed(1)}° (limit 2·10°=20°)`);
}
