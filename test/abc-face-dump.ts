// SPDX-License-Identifier: AGPL-3.0-only
// Dump one face's loops, edge kinds, and projected (u,v) domain.
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { makeSurface } from "../src/geom/surfaces.ts";

const path = process.argv[2]!;
const target = Number(process.argv[3]);
const brep = buildBrep(readFileSync(path, "utf8"));
const s = brep.solids[0]!.scale ?? brep.scale;
for (const solid of brep.solids) {
  for (const face of solid.faces) {
    if (face.faceId !== target) continue;
    const surf = makeSurface(brep.table, face.surfaceId, s, brep.units.radPerAngle)!;
    console.log(`face ${face.faceId} surfId=${face.surfaceId} kind=${face.surfaceKind} sameSense=${face.sameSense}`);
    console.log(`  periodicU=${surf.periodicU} periodicV=${surf.periodicV} uPeriod=${surf.uPeriod?.toFixed(3)} t0..t1=${(surf as any).t0}..${(surf as any).t1}`);
    let uMin = Infinity, uMax = -Infinity, vMin = Infinity, vMax = -Infinity;
    const loops = (face as any).loops ?? [];
    console.log(`  loops: ${loops.length}`);
    for (let li = 0; li < loops.length; li++) {
      const loop = loops[li];
      const kinds: string[] = [];
      for (const oe of loop.edges ?? []) {
        const e = brep.edges.get(oe.edgeId);
        if (!e) { kinds.push("?"); continue; }
        const ck = brep.table.typeOf(e.curveId) ?? "(complex)";
        kinds.push(ck.replace(/_CURVE.*/, "").slice(0, 8));
        for (const pt of [e.v0, e.v1]) {
          const [pu, pv] = surf.project(pt);
          uMin = Math.min(uMin, pu); uMax = Math.max(uMax, pu);
          vMin = Math.min(vMin, pv); vMax = Math.max(vMax, pv);
        }
      }
      console.log(`    loop ${li}: ${loop.edges?.length} edges [${kinds.join(",")}]`);
    }
    console.log(`  boundary-endpoint (u,v) span: u=[${uMin.toFixed(3)},${uMax.toFixed(3)}] (${((uMax - uMin) / Math.PI).toFixed(2)}π)  v=[${vMin.toFixed(4)},${vMax.toFixed(4)}]`);
    // sample the surface at domain corners to see extent
    const A = (surf as any).A, D = (surf as any).D;
    if (A) console.log(`  axis A=[${A.map((x: number) => x.toFixed(2))}] D=[${D.map((x: number) => x.toFixed(3))}]`);
  }
}
