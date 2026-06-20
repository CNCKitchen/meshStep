// SPDX-License-Identifier: AGPL-3.0-only
// Break down sliver triangles by the CAD surface kind of their face, to locate quality hotspots.
import { readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { makeSurface } from "../src/geom/surfaces.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const files = process.argv.slice(2).length ? process.argv.slice(2) : ["tool", "Insta360GO3_case", "LampenHalter"];

for (const f of files) {
  const src = readFileSync(join(root, f + ".step"), "utf8");
  const brep = buildBrep(src);
  const kindOf = new Map<number, string>();
  for (const solid of brep.solids) for (const face of solid.faces) {
    const s = makeSurface(brep.table, face.surfaceId, brep.scale);
    kindOf.set(face.faceId, s ? s.kind : "null");
  }
  const res = tessellate(brep, { chordTol: 0.002, targetEdge: 1 });
  const P = res.mesh.positions, I = res.mesh.indices, nt = I.length / 3;
  const byKind = new Map<string, { sliver: number; total: number }>();
  for (let t = 0; t < nt; t++) {
    const a = I[t*3]!, b = I[t*3+1]!, c = I[t*3+2]!;
    const ax=P[a*3]!,ay=P[a*3+1]!,az=P[a*3+2]!,bx=P[b*3]!,by=P[b*3+1]!,bz=P[b*3+2]!,cx=P[c*3]!,cy=P[c*3+1]!,cz=P[c*3+2]!;
    const ab=Math.hypot(bx-ax,by-ay,bz-az),bc=Math.hypot(cx-bx,cy-by,cz-bz),ca=Math.hypot(ax-cx,ay-cy,az-cz);
    const ang=(o:number,p:number,q:number)=>Math.acos(Math.max(-1,Math.min(1,(o*o+p*p-q*q)/(2*o*p||1))))*180/Math.PI;
    const m1=Math.min(ang(ab,ca,bc),ang(ab,bc,ca),ang(bc,ca,ab));
    const k = kindOf.get(res.faceOfTri[t]!) ?? "?";
    const e = byKind.get(k) ?? { sliver: 0, total: 0 };
    e.total++; if (m1 < 20) e.sliver++; byKind.set(k, e);
  }
  console.log(`\n== ${f} ==`);
  for (const [k, e] of [...byKind.entries()].sort((a,b)=>b[1].sliver-a[1].sliver)) {
    console.log(`  ${k.padEnd(22)} slivers ${e.sliver}/${e.total} (${(100*e.sliver/e.total).toFixed(1)}%)`);
  }
}
