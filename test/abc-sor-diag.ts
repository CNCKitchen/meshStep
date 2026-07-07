// SPDX-License-Identifier: AGPL-3.0-only
// Diagnostic: self-consistency of SURFACE_OF_REVOLUTION faces in a model.
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { makeSurface } from "../src/geom/surfaces.ts";

const path = process.argv[2]!;
const brep = buildBrep(readFileSync(path, "utf8"));
const s = brep.solids[0]!.scale ?? brep.scale;
for (const solid of brep.solids) {
  for (const face of solid.faces) {
    if (face.surfaceKind !== "SURFACE_OF_REVOLUTION") continue;
    const surf = makeSurface(brep.table, face.surfaceId, s, brep.units.radPerAngle);
    if (!surf) { console.log(`face ${face.faceId}: makeSurface null`); continue; }
    // BOUNDARY CONTAINMENT: does the surface pass through its own trim-edge endpoints?
    let maxBnd = 0; let bndAt: number[] = [];
    for (const loop of (face as any).loops ?? []) {
      for (const oe of loop.edges ?? []) {
        const e = brep.edges.get(oe.edgeId);
        if (!e) continue;
        for (const pt of [e.v0, e.v1] as number[][]) {
          const [pu, pv] = surf.project(pt as any);
          const q = surf.evaluate(pu, pv);
          const d = Math.hypot(q[0] - pt[0]!, q[1] - pt[1]!, q[2] - pt[2]!);
          if (d > maxBnd) { maxBnd = d; bndAt = pt; }
        }
      }
    }
    if (maxBnd > 0.05) console.log(`  face ${face.faceId} surfId=${face.surfaceId}: BOUNDARY OFF-SURFACE max=${maxBnd.toFixed(3)}mm at [${bndAt.map((x) => x.toFixed(2))}]`);
    // sample the parameter domain, check evaluate/project round-trip
    let maxRt = 0, maxRadialDrift = 0;
    const N = 8;
    for (let iu = 0; iu <= N; iu++) {
      for (let iv = 0; iv <= N; iv++) {
        const u = -Math.PI + (2 * Math.PI * iu) / N;
        // v across the (unknown) profile domain: use project of evaluate at v-fractions via a probe
        const v = iv / N; // placeholder — replaced below by using surface's own sampling
        const p = surf.evaluate(u, v);
        const [pu, pv] = surf.project(p);
        const q = surf.evaluate(pu, pv);
        const rt = Math.hypot(q[0] - p[0], q[1] - p[1], q[2] - p[2]);
        if (rt > maxRt) maxRt = rt;
      }
    }
    // radial preservation: evaluate at fixed v across u must keep constant distance to axis
    const A = (surf as any).A, D = (surf as any).D;
    if (A && D) {
      const radialOf = (p: number[]): number => {
        const w = [p[0] - A[0], p[1] - A[1], p[2] - A[2]];
        const a = w[0] * D[0] + w[1] * D[1] + w[2] * D[2];
        return Math.hypot(w[0] - a * D[0], w[1] - a * D[1], w[2] - a * D[2]);
      };
      const vmid = ((surf as any).t0 + (surf as any).t1) / 2;
      const r0 = radialOf(surf.evaluate(0, vmid));
      for (let k = 0; k < 12; k++) {
        const rr = radialOf(surf.evaluate(-Math.PI + (2 * Math.PI * k) / 12, vmid));
        maxRadialDrift = Math.max(maxRadialDrift, Math.abs(rr - r0));
      }
      // check evaluate(0,v) == curve(v)
      const curve = (surf as any).curve;
      let maxProfErr = 0;
      for (let k = 0; k <= 10; k++) {
        const v = (surf as any).t0 + ((surf as any).t1 - (surf as any).t0) * k / 10;
        const e = surf.evaluate(0, v), c = curve.evaluate(v);
        maxProfErr = Math.max(maxProfErr, Math.hypot(e[0] - c[0], e[1] - c[1], e[2] - c[2]));
      }
      console.log(`face ${face.faceId}: axis A=[${A.map((x: number) => x.toFixed(2))}] D=[${D.map((x: number) => x.toFixed(3))}] t=[${(surf as any).t0.toFixed(3)},${(surf as any).t1.toFixed(3)}] rt=${maxRt.toExponential(1)} radialDrift=${maxRadialDrift.toExponential(1)} profErr(u0=curve)=${maxProfErr.toExponential(1)} r@mid=${r0.toFixed(2)}`);
    }
  }
}
