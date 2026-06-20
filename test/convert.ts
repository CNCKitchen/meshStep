// SPDX-License-Identifier: AGPL-3.0-only
// End-to-end: import each STEP file, export STL to out/, and compare to the reference STL
// via symmetric Hausdorff. Reports coverage (faces tessellated) + deviation per file.
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep, writeBinarySTL, readSTL } from "../src/index.ts";
import { bboxOfSoup, type TriSoup } from "../src/io/stl.ts";
import { hausdorff } from "./hausdorff.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = join(root, "out");
mkdirSync(outDir, { recursive: true });

const files = ["cube", "cylinder", "cone", "sphere", "cylinderWithHole", "roundedCube", "everything"];

/** Count edges used by !=2 triangles: boundary (cracks) and non-manifold. */
const watertight = (m: { indices: Uint32Array }): { boundary: number; nonmanifold: number } => {
  const inc = new Map<string, number>();
  const add = (a: number, b: number): void => {
    const k = a < b ? `${a}_${b}` : `${b}_${a}`;
    inc.set(k, (inc.get(k) ?? 0) + 1);
  };
  for (let i = 0; i < m.indices.length; i += 3) {
    const a = m.indices[i]!, b = m.indices[i + 1]!, c = m.indices[i + 2]!;
    add(a, b); add(b, c); add(c, a);
  }
  let boundary = 0, nonmanifold = 0;
  for (const n of inc.values()) { if (n === 1) boundary++; else if (n > 2) nonmanifold++; }
  return { boundary, nonmanifold };
};

/** Percent of triangles that are slivers (min interior angle < 20°), and the worst min-angle. */
const sliverStats = (m: { positions: Float64Array; indices: Uint32Array }): { pct: number; minAngle: number } => {
  const p = m.positions;
  const ang = (ax: number, bx: number, cx: number): number => {
    // angle at vertex a of triangle (a,b,c), given the 3 vertex base offsets
    const ux = p[bx]! - p[ax]!, uy = p[bx + 1]! - p[ax + 1]!, uz = p[bx + 2]! - p[ax + 2]!;
    const vx = p[cx]! - p[ax]!, vy = p[cx + 1]! - p[ax + 1]!, vz = p[cx + 2]! - p[ax + 2]!;
    const d = (ux * vx + uy * vy + uz * vz) / (Math.hypot(ux, uy, uz) * Math.hypot(vx, vy, vz) || 1);
    return Math.acos(Math.max(-1, Math.min(1, d))) * 180 / Math.PI;
  };
  let slivers = 0, minA = 180, n = m.indices.length / 3;
  for (let i = 0; i < m.indices.length; i += 3) {
    const a = m.indices[i]! * 3, b = m.indices[i + 1]! * 3, c = m.indices[i + 2]! * 3;
    const mn = Math.min(ang(a, b, c), ang(b, c, a), ang(c, a, b));
    if (mn < 20) slivers++;
    if (mn < minA) minA = mn;
  }
  return { pct: n ? (slivers / n) * 100 : 0, minAngle: minA };
};

/** Max and mean triangle edge length (mm). */
const edgeStats = (m: { positions: Float64Array; indices: Uint32Array }): { max: number; mean: number } => {
  const p = m.positions;
  let max = 0, sum = 0, cnt = 0;
  const d = (a: number, b: number): number =>
    Math.hypot(p[a]! - p[b]!, p[a + 1]! - p[b + 1]!, p[a + 2]! - p[b + 2]!);
  for (let i = 0; i < m.indices.length; i += 3) {
    const a = m.indices[i]! * 3, b = m.indices[i + 1]! * 3, c = m.indices[i + 2]! * 3;
    for (const e of [d(a, b), d(b, c), d(c, a)]) { if (e > max) max = e; sum += e; cnt++; }
  }
  return { max, mean: cnt ? sum / cnt : 0 };
};

/** Signed volume via the divergence theorem; > 0 means triangles face outward. */
const signedVolume = (m: { positions: Float64Array; indices: Uint32Array }): number => {
  const p = m.positions;
  let vol = 0;
  for (let i = 0; i < m.indices.length; i += 3) {
    const a = m.indices[i]! * 3, b = m.indices[i + 1]! * 3, c = m.indices[i + 2]! * 3;
    const ax = p[a]!, ay = p[a + 1]!, az = p[a + 2]!;
    const bx = p[b]!, by = p[b + 1]!, bz = p[b + 2]!;
    const cx = p[c]!, cy = p[c + 1]!, cz = p[c + 2]!;
    vol += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return vol;
};

const meshToSoup = (m: { positions: Float64Array; indices: Uint32Array }): TriSoup => {
  const n = m.indices.length;
  const positions = new Float64Array(n * 3);
  for (let i = 0; i < n; i++) {
    const v = m.indices[i]! * 3;
    positions[i * 3] = m.positions[v]!;
    positions[i * 3 + 1] = m.positions[v + 1]!;
    positions[i * 3 + 2] = m.positions[v + 2]!;
  }
  return { positions, triangleCount: n / 3 };
};

console.log("file".padEnd(18), "cover".padEnd(8), "tris".padStart(7), "watertight", "norm ", "maxedge    ", "quality         ", "Hausdorff  skipped");
console.log("-".repeat(120));

for (const name of files) {
  let src: string;
  try { src = readFileSync(join(root, `${name}.step`), "utf8"); }
  catch { continue; }

  // Fusion "Custom" settings from the reference: Surface Dev 0.002, Normal Dev 10°, Max Edge 1.
  const res = importStep(src, { surfaceDeviation: 0.002, normalDeviation: 10, maxEdge: 1 });
  const cover = `${res.stats.facesTessellated}/${res.stats.facesTotal}`;
  const triCount = res.mesh.indices.length / 3;

  writeFileSync(join(outDir, `${name}.stl`), writeBinarySTL(res.mesh));

  let hMax = "-", hMean = "-";
  try {
    const ref = readSTL(new Uint8Array(readFileSync(join(root, `${name}.stl`))));
    if (triCount > 0) {
      const h = hausdorff(meshToSoup(res.mesh), ref);
      const diag = bboxOfSoup(ref).diagonal;
      hMax = `${h.max.toFixed(4)} (${((h.max / diag) * 100).toFixed(2)}%D)`;
      hMean = h.mean.toFixed(4);
    }
  } catch { /* no reference */ }

  const wt = watertight(res.mesh);
  const wtStr = wt.boundary === 0 && wt.nonmanifold === 0 ? "closed" : `B${wt.boundary}/NM${wt.nonmanifold}`;
  const vol = signedVolume(res.mesh);
  const volStr = triCount > 0 ? (vol > 0 ? "out+" : "IN-") : "-";
  const es = triCount > 0 ? edgeStats(res.mesh) : { max: 0, mean: 0 };
  const sv = triCount > 0 ? sliverStats(res.mesh) : { pct: 0, minAngle: 0 };
  const skipped = Object.entries(res.stats.skipped).map(([k, v]) => `${k}:${v}`).join(" ");
  console.log(
    name.padEnd(18),
    cover.padEnd(8),
    String(triCount).padStart(7),
    wtStr.padEnd(10),
    volStr.padEnd(5),
    `edge≤${es.max.toFixed(2)}`.padEnd(11),
    `slv${sv.pct.toFixed(1)}%/min${sv.minAngle.toFixed(0)}°`.padEnd(16),
    `${hMax.padEnd(16)} ${skipped}`,
  );
}
