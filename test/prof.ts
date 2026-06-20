import { readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { remesh } from "../src/mesh/remesh.ts";
import { makeSurface } from "../src/geom/surfaces.ts";
const root = join(dirname(fileURLToPath(import.meta.url)), "..");
for(const f of (process.argv.slice(2).length?process.argv.slice(2):["MicHolder_Inserts","SquAIRPot","GoProHandlePod"])){
  const src=readFileSync(join(root,f+".step"),"utf8");
  let t=Date.now(); const brep=buildBrep(src); const tBrep=Date.now()-t;
  t=Date.now(); const res=tessellate(brep,{chordTol:0.01,targetEdge:1}); const tTess=Date.now()-t;
  const surf=new Map<number,any>();const solidOf=new Map<number,number>();
  for(const solid of brep.solids)for(const face of solid.faces){surf.set(face.faceId,makeSurface(brep.table,face.surfaceId,brep.scale));solidOf.set(face.faceId,solid.id);}
  t=Date.now(); const r=remesh(res.mesh,res.faceOfTri,surf,{surfaceDev:0.01,normalDev:15*Math.PI/180,maxEdge:1}); const tRem=Date.now()-t;
  console.log(`${f}: brep=${tBrep}ms tess=${tTess}ms(${res.mesh.indices.length/3} tris) remesh=${tRem}ms(${r.mesh.indices.length/3} tris)`);
}
