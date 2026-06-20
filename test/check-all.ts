// SPDX-License-Identifier: AGPL-3.0-only
// Watertightness check for all parts (no-remesh fine import). Accuracy lives in test/accuracy.ts.
import { readFileSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const files = ["cube","cylinder","cone","sphere","cylinderWithHole","roundedCube","everything","MicHolder_Inserts","splineThing","Insta360GO3_case","LampenHalter","SquAIRPot","tool"];

function watertight(m: { positions: Float64Array; indices: Uint32Array }) {
  const inc = new Map<number, number>();
  const K = 0x4000000;
  const nt = m.indices.length / 3;
  for (let t = 0; t < nt; t++) for (let e = 0; e < 3; e++) {
    const a = m.indices[t*3+e]!, b = m.indices[t*3+(e+1)%3]!;
    const k = a < b ? a*K+b : b*K+a;
    inc.set(k, (inc.get(k) ?? 0) + 1);
  }
  let open = 0, nonman = 0;
  for (const c of inc.values()) { if (c === 1) open++; else if (c > 2) nonman++; }
  return { tris: nt, verts: m.positions.length/3, open, nonman };
}

for (const f of files) {
  const sp = join(root, f + ".step");
  if (!existsSync(sp)) { console.log(`${f}: (no step)`); continue; }
  try {
    const res = importStep(readFileSync(sp, "utf8"), { remesh: false, surfaceDeviation: 0.002, maxEdge: 1 });
    const wt = watertight(res.mesh);
    const ok = wt.open === 0 && wt.nonman === 0;
    console.log(`${ok?"PASS":"FAIL"} ${f}: tris=${wt.tris} verts=${wt.verts} open=${wt.open} nonman=${wt.nonman} skipped=${JSON.stringify(res.stats.skipped)}`);
  } catch (e) {
    console.log(`ERROR ${f}: ${(e as Error).message}`);
  }
}
