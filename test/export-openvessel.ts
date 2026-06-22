// SPDX-License-Identifier: AGPL-3.0-only
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep, writeBinarySTL } from "../src/index.ts";
const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = join(root, "out"); mkdirSync(outDir, { recursive: true });
const src = readFileSync(join(root, "sourceModels", "OpenVessel.step"), "utf8");
function stats(m: any) {
  const I = m.indices, nt = I.length/3, und = new Map<string,number>();
  for (let t=0;t<nt;t++) for (let e=0;e<3;e++){const a=I[t*3+e]!,b=I[t*3+(e+1)%3]!;const k=a<b?`${a}_${b}`:`${b}_${a}`;und.set(k,(und.get(k)??0)+1);}
  let open=0,nm=0; for (const c of und.values()){if(c===1)open++;else if(c>2)nm++;}
  return `tris=${nt} open=${open} nm=${nm}`;
}
const fine = importStep(src, { remesh: false, surfaceDeviation: 0.01, maxEdge: 1 });
writeFileSync(join(outDir, "OpenVessel.meshStep.stl"), writeBinarySTL(fine.mesh));
console.log("no-remesh ->", stats(fine.mesh), "-> out/OpenVessel.meshStep.stl");
const t0 = Date.now();
const rem = importStep(src, { remesh: true, surfaceDeviation: 0.01, maxEdge: 1 });
writeFileSync(join(outDir, "OpenVessel.remesh.stl"), writeBinarySTL(rem.mesh));
console.log("remesh   ->", stats(rem.mesh), `(${Date.now()-t0}ms) -> out/OpenVessel.remesh.stl`);
