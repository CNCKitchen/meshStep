// SPDX-License-Identifier: AGPL-3.0-only
// No-remesh quality + watertightness for ALL parts (slivers, min angle, open/nonman edges).
import { readFileSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const files = ["cube","cylinder","cone","sphere","cylinderWithHole","roundedCube","everything","MicHolder_Inserts","splineThing","Insta360GO3_case","LampenHalter","SquAIRPot","tool"];

function stats(m: { positions: Float64Array; indices: Uint32Array }) {
  const P = m.positions, I = m.indices, nt = I.length / 3;
  let sliver = 0, minA = 180;
  const K = 0x4000000; const inc = new Map<number, number>();
  for (let t = 0; t < nt; t++) {
    const a = I[t*3]!, b = I[t*3+1]!, c = I[t*3+2]!;
    for (const [x,y] of [[a,b],[b,c],[c,a]] as [number,number][]) { const k = x<y?x*K+y:y*K+x; inc.set(k,(inc.get(k)??0)+1); }
    const ax=P[a*3]!,ay=P[a*3+1]!,az=P[a*3+2]!,bx=P[b*3]!,by=P[b*3+1]!,bz=P[b*3+2]!,cx=P[c*3]!,cy=P[c*3+1]!,cz=P[c*3+2]!;
    const ab=Math.hypot(bx-ax,by-ay,bz-az),bc=Math.hypot(cx-bx,cy-by,cz-bz),ca=Math.hypot(ax-cx,ay-cy,az-cz);
    const ang=(o:number,p:number,q:number)=>{const v=(o*o+p*p-q*q)/(2*o*p||1);return Math.acos(Math.max(-1,Math.min(1,v)))*180/Math.PI;};
    const m1=Math.min(ang(ab,ca,bc),ang(ab,bc,ca),ang(bc,ca,ab));
    if(m1<20)sliver++; if(m1<minA)minA=m1;
  }
  let open=0,nonman=0; for(const v of inc.values()){if(v===1)open++;else if(v>2)nonman++;}
  return {nt,sliverPct:+(100*sliver/nt).toFixed(1),minAngle:+minA.toFixed(2),open,nonman};
}

for (const f of files) {
  const sp = join(root, f + ".step");
  if (!existsSync(sp)) { console.log(`${f}: (no step)`); continue; }
  try {
    const res = importStep(readFileSync(sp,"utf8"), { remesh:false, surfaceDeviation:0.002, maxEdge:1 });
    const s = stats(res.mesh);
    const ok = s.open===0 && s.nonman===0;
    console.log(`${ok?"PASS":"FAIL"} ${f}: tris=${s.nt} sliver%=${s.sliverPct} minAng=${s.minAngle} open=${s.open} nonman=${s.nonman}`);
  } catch (e) { console.log(`ERROR ${f}: ${(e as Error).message}`); }
}
