import { readFileSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";
import { readSTL, type TriSoup, type IndexedMesh } from "../src/io/stl.ts";
import { hausdorff } from "./hausdorff.ts";
const root = join(dirname(fileURLToPath(import.meta.url)), "..");
function toSoup(m:IndexedMesh):TriSoup{const nt=m.indices.length/3;const p=new Float64Array(nt*9);for(let t=0;t<nt;t++)for(let e=0;e<3;e++){const v=m.indices[t*3+e]!;p[t*9+e*3]=m.positions[v*3]!;p[t*9+e*3+1]=m.positions[v*3+1]!;p[t*9+e*3+2]=m.positions[v*3+2]!;}return{positions:p,triangleCount:nt};}
for(const f of ["cylinder","sphere","cone","roundedCube","MicHolder_Inserts"]){
  const sp=join(root,f+".step"),st=join(root,f+".stl");if(!existsSync(st))continue;
  const res=importStep(readFileSync(sp,"utf8"),{remesh:true,surfaceDeviation:0.05,maxEdge:1});
  const h=hausdorff(toSoup(res.mesh),readSTL(readFileSync(st)),1200);
  console.log(`${f}: REMESH maxHaus=${h.max.toFixed(4)}mm meanHaus=${h.mean.toFixed(4)}mm`);
}
