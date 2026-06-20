// SPDX-License-Identifier: AGPL-3.0-only
// Mesh the newTestModels/*.step (no remesh) -> out/<name>.meshStep.stl
import { readFileSync, writeFileSync, mkdirSync, readdirSync } from "node:fs";
import { join, dirname, basename } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep, writeBinarySTL } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const srcDir = join(root, "newTestModels");
const outDir = join(root, "out");
mkdirSync(outDir, { recursive: true });

for (const file of readdirSync(srcDir).filter((f) => f.toLowerCase().endsWith(".step"))) {
  const name = basename(file, ".step");
  const src = readFileSync(join(srcDir, file), "utf8");
  const t0 = Date.now();
  try {
    const r = importStep(src, { remesh: false, surfaceDeviation: 0.002, maxEdge: 1 });
    const m = r.mesh;
    // watertightness + sliver check
    const edge = new Map<string, number>();
    const I = m.indices, P = m.positions;
    for (let i = 0; i < I.length; i += 3) {
      for (let e = 0; e < 3; e++) {
        const a = I[i + e]!, b = I[i + ((e + 1) % 3)]!;
        const k = a < b ? `${a}_${b}` : `${b}_${a}`;
        edge.set(k, (edge.get(k) ?? 0) + 1);
      }
    }
    let open = 0, nonman = 0;
    for (const c of edge.values()) { if (c === 1) open++; else if (c > 2) nonman++; }
    let sliver = 0; const nt = I.length / 3;
    for (let i = 0; i < I.length; i += 3) {
      const ax = P[I[i]! * 3]!, ay = P[I[i]! * 3 + 1]!, az = P[I[i]! * 3 + 2]!;
      const bx = P[I[i + 1]! * 3]!, by = P[I[i + 1]! * 3 + 1]!, bz = P[I[i + 1]! * 3 + 2]!;
      const cx = P[I[i + 2]! * 3]!, cy = P[I[i + 2]! * 3 + 1]!, cz = P[I[i + 2]! * 3 + 2]!;
      const angs = [
        ang(ax, ay, az, bx, by, bz, cx, cy, cz),
        ang(bx, by, bz, cx, cy, cz, ax, ay, az),
        ang(cx, cy, cz, ax, ay, az, bx, by, bz),
      ];
      if (Math.min(...angs) < 20 * Math.PI / 180) sliver++;
    }
    writeFileSync(join(outDir, `${name}.meshStep.stl`), writeBinarySTL(m));
    const ok = open === 0 && nonman === 0 ? "PASS" : "FAIL";
    console.log(`${ok} ${name}: tris=${nt} sliver%=${(100 * sliver / nt).toFixed(1)} open=${open} nonman=${nonman} (${Date.now() - t0}ms)`);
  } catch (e) {
    console.log(`ERROR ${name}: ${(e as Error).message}`);
  }
}

function ang(px: number, py: number, pz: number, ax: number, ay: number, az: number, bx: number, by: number, bz: number): number {
  const u = [ax - px, ay - py, az - pz], v = [bx - px, by - py, bz - pz];
  const du = Math.hypot(u[0]!, u[1]!, u[2]!), dv = Math.hypot(v[0]!, v[1]!, v[2]!);
  const d = (u[0]! * v[0]! + u[1]! * v[1]! + u[2]! * v[2]!) / (du * dv || 1e-9);
  return Math.acos(Math.max(-1, Math.min(1, d)));
}
