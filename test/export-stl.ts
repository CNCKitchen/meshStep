// SPDX-License-Identifier: AGPL-3.0-only
// Export every part's generated mesh for inspection: out/<name>.meshStep.stl (fine, no-remesh) and
// out/<name>.remesh.stl (uniform isotropic remesh — the default import pipeline).
import { readFileSync, writeFileSync, mkdirSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep, writeBinarySTL } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = join(root, "out");
mkdirSync(outDir, { recursive: true });
const files = ["cube","cylinder","cone","sphere","cylinderWithHole","roundedCube","everything","MicHolder_Inserts","splineThing","GoProHandlePod","Insta360GO3_case","LampenHalter","tool","SquAIRPot","chamferFillet"];

for (const f of files) {
  const sp = join(root, f + ".step");
  if (!existsSync(sp)) { console.log(`${f}: (no step)`); continue; }
  const src = readFileSync(sp, "utf8");
  const fine = importStep(src, { remesh: false, surfaceDeviation: 0.002, maxEdge: 1 });
  writeFileSync(join(outDir, `${f}.meshStep.stl`), writeBinarySTL(fine.mesh));
  const t0 = Date.now();
  const rem = importStep(src, { remesh: true, surfaceDeviation: 0.01, maxEdge: 1 });
  writeFileSync(join(outDir, `${f}.remesh.stl`), writeBinarySTL(rem.mesh));
  console.log(`${f}: no-remesh ${fine.mesh.indices.length/3} tris -> ${f}.meshStep.stl | remesh ${rem.mesh.indices.length/3} tris (${Date.now()-t0}ms) -> ${f}.remesh.stl`);
}
