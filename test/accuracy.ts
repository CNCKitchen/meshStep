// SPDX-License-Identifier: AGPL-3.0-only
// Real symmetric Hausdorff (point-to-triangle) with a uniform grid over the reference triangles.
import { readFileSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";
import { readSTL, type TriSoup, type IndexedMesh } from "../src/io/stl.ts";
import { hausdorff } from "./hausdorff.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const files = ["cube","cylinder","cone","sphere","cylinderWithHole","roundedCube","everything","MicHolder_Inserts","splineThing","GoProHandlePod"];

function toSoup(m: IndexedMesh): TriSoup {
  const nt = m.indices.length / 3;
  const p = new Float64Array(nt * 9);
  for (let t = 0; t < nt; t++) for (let e = 0; e < 3; e++) {
    const v = m.indices[t*3+e]!;
    p[t*9+e*3] = m.positions[v*3]!; p[t*9+e*3+1] = m.positions[v*3+1]!; p[t*9+e*3+2] = m.positions[v*3+2]!;
  }
  return { positions: p, triangleCount: nt };
}
function bboxDiag(s: TriSoup): number {
  let mn=[Infinity,Infinity,Infinity], mx=[-Infinity,-Infinity,-Infinity];
  for (let i=0;i<s.positions.length;i+=3) for(let k=0;k<3;k++){mn[k]=Math.min(mn[k]!,s.positions[i+k]!);mx[k]=Math.max(mx[k]!,s.positions[i+k]!);}
  return Math.hypot(mx[0]!-mn[0]!,mx[1]!-mn[1]!,mx[2]!-mn[2]!);
}

for (const f of files) {
  const sp = join(root, f + ".step"), st = join(root, f + ".stl");
  if (!existsSync(sp) || !existsSync(st)) { console.log(`${f}: (missing)`); continue; }
  const res = importStep(readFileSync(sp, "utf8"), { remesh: false, surfaceDeviation: 0.05, maxEdge: 1 });
  const mine = toSoup(res.mesh);
  const ref = readSTL(readFileSync(st));
  const diag = bboxDiag(ref);
  const h = hausdorff(mine, ref, 1200);
  console.log(`${f}: maxHaus=${h.max.toFixed(4)}mm (${(100*h.max/diag).toFixed(3)}% diag) meanHaus=${h.mean.toFixed(4)}mm  tris=${mine.triangleCount}`);
}
