import { readFileSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";
const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const files = process.argv.slice(2).length?process.argv.slice(2):["cylinder","cone","sphere","cylinderWithHole","roundedCube","MicHolder_Inserts","LampenHalter","tool","GoProHandlePod"];
function stats(m:{positions:Float64Array;indices:Uint32Array}){
  const P=m.positions,I=m.indices,nt=I.length/3;let sliver=0,minA=180;
  const K=0x4000000;const inc=new Map<number,number>();
  for(let t=0;t<nt;t++){const a=I[t*3]!,b=I[t*3+1]!,c=I[t*3+2]!;
    for(const[x,y]of[[a,b],[b,c],[c,a]]as[number,number][]){const k=x<y?x*K+y:y*K+x;inc.set(k,(inc.get(k)??0)+1);}
    const ax=P[a*3]!,ay=P[a*3+1]!,az=P[a*3+2]!,bx=P[b*3]!,by=P[b*3+1]!,bz=P[b*3+2]!,cx=P[c*3]!,cy=P[c*3+1]!,cz=P[c*3+2]!;
    const ab=Math.hypot(bx-ax,by-ay,bz-az),bc=Math.hypot(cx-bx,cy-by,cz-bz),ca=Math.hypot(ax-cx,ay-cy,az-cz);
    const ang=(o:number,p:number,q:number)=>Math.acos(Math.max(-1,Math.min(1,(o*o+p*p-q*q)/(2*o*p||1))))*180/Math.PI;
    const m1=Math.min(ang(ab,ca,bc),ang(ab,bc,ca),ang(bc,ca,ab));if(m1<20)sliver++;if(m1<minA)minA=m1;}
  let open=0,nm=0;for(const v of inc.values()){if(v===1)open++;else if(v>2)nm++;}
  return `tris=${nt} sliver%=${(100*sliver/nt).toFixed(1)} minAng=${minA.toFixed(2)} open=${open} nm=${nm}`;
}
for(const f of files){const sp=join(root,f+".step");if(!existsSync(sp))continue;
  const src=readFileSync(sp,"utf8");
  const noR=importStep(src,{remesh:false,surfaceDeviation:0.02,maxEdge:1});
  const t0=Date.now();const reM=importStep(src,{remesh:true,surfaceDeviation:0.02,maxEdge:1});
  console.log(`${f}:\n  no-remesh ${stats(noR.mesh)}\n  remesh(${Date.now()-t0}ms) ${stats(reM.mesh)}`);
}
