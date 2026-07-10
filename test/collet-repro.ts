// SPDX-License-Identifier: AGPL-3.0-only
// Repro: "Tapered Collet Insert.step" breaks at coarse surface deviation (>= 0.1 mm).
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep, writeBinarySTL } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = join(root, "out");
mkdirSync(outDir, { recursive: true });

const file = process.argv[2] ?? "Tapered Collet Insert.step";
const src = readFileSync(join(root, file), "utf8");

const volume = (m: { positions: Float64Array; indices: Uint32Array }): number => {
  const p = m.positions, I = m.indices;
  let v = 0;
  for (let i = 0; i < I.length; i += 3) {
    const a = I[i]! * 3, b = I[i + 1]! * 3, c = I[i + 2]! * 3;
    v += (p[a]! * (p[b + 1]! * p[c + 2]! - p[b + 2]! * p[c + 1]!)
        - p[a + 1]! * (p[b]! * p[c + 2]! - p[b + 2]! * p[c]!)
        + p[a + 2]! * (p[b]! * p[c + 1]! - p[b + 1]! * p[c]!)) / 6;
  }
  return v;
};

const bbox = (p: Float64Array): string => {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3)
    for (let d = 0; d < 3; d++) { lo[d] = Math.min(lo[d]!, p[i + d]!); hi[d] = Math.max(hi[d]!, p[i + d]!); }
  return lo.map((l, d) => (hi[d]! - l).toFixed(2)).join(" x ");
};

console.log(`file: ${file}`);
console.log("case".padEnd(26) + "tris".padStart(8) + "open".padStart(6) + "nonmf".padStart(6)
  + "dropped".padStart(8) + "skipped".padStart(8) + "warn".padStart(6) + "volume".padStart(14) + "  bbox");
const cases: Array<[string, { surfaceDeviation: number; maxEdge?: number; normalDeviation?: number }]> = [
  ["dev=0.005", { surfaceDeviation: 0.005 }],
  ["dev=0.01 (default)", { surfaceDeviation: 0.01 }],
  ["dev=0.05", { surfaceDeviation: 0.05 }],
  ["dev=0.1", { surfaceDeviation: 0.1 }],
  ["dev=0.1 me=5", { surfaceDeviation: 0.1, maxEdge: 5 }],
  ["dev=0.1 me=10", { surfaceDeviation: 0.1, maxEdge: 10 }],
  ["dev=0.1 me=5 nd=30", { surfaceDeviation: 0.1, maxEdge: 5, normalDeviation: 30 }],
  ["dev=0.2 me=10 nd=30", { surfaceDeviation: 0.2, maxEdge: 10, normalDeviation: 30 }],
  ["dev=0.5 me=10 nd=45", { surfaceDeviation: 0.5, maxEdge: 10, normalDeviation: 45 }],
];
for (const [label, opts] of cases) {
  try {
    const r = importStep(src, opts);
    const d = r.diagnostics;
    console.log(label.padEnd(26) + String(r.mesh.indices.length / 3).padStart(8)
      + String(d.openEdges).padStart(6) + String(d.nonManifoldEdges).padStart(6)
      + String(d.facesDropped).padStart(8) + String(d.facesSkipped).padStart(8)
      + String(d.warnings.length).padStart(6) + volume(r.mesh).toFixed(2).padStart(14)
      + "  " + bbox(r.mesh.positions));
    writeFileSync(join(outDir, `collet_${label.replace(/[^a-z0-9.]+/gi, "_")}.stl`), writeBinarySTL(r.mesh));
    if (d.warnings.length > 0) {
      const byCode = new Map<string, number>();
      for (const w of d.warnings) byCode.set(w.code, (byCode.get(w.code) ?? 0) + 1);
      console.log("  warnings: " + [...byCode].map(([c, n]) => `${c}x${n}`).join(", "));
    }
  } catch (e) {
    console.log(label.padEnd(26) + "  ERROR: " + (e as Error).message);
  }
}
