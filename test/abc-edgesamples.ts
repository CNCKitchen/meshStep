// SPDX-License-Identifier: AGPL-3.0-only
// Print the shared sampled polyline of one edge, using the abc-worker tolerance derivation.
//   node test/abc-edgesamples.ts <step-file> <edgeId>
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { estimateBrepSize } from "../src/step/measure.ts";
import { sampleEdgePolyline } from "../src/geom/curves.ts";

const brep = buildBrep(readFileSync(process.argv[2]!, "utf8"));
const id = Number(process.argv[3]);
const diag = estimateBrepSize(brep)?.diag ?? 0;
const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
const e = brep.edges.get(id)!;
const sc = e.scale ?? brep.scale;
console.log(`curve kind=${brep.table.typeOf(e.curveId)} v0=(${e.v0.join(",")}) v1=(${e.v1.join(",")}) sameSense=${e.sameSense}`);
const rec = brep.table.record(e.curveId);
console.log(`raw params: ${JSON.stringify(rec.params).slice(0, 800)}`);
const { bsplineData, deBoor } = await import("../src/geom/curves.ts") as any;
const bs = bsplineData?.(brep.table, e.curveId, sc);
if (bs) {
  const p0 = deBoor(bs.degree, bs.cps, bs.knots, bs.u0, bs.weights);
  const p1 = deBoor(bs.degree, bs.cps, bs.knots, bs.u1, bs.weights);
  const gap = Math.hypot(p0[0] - p1[0], p0[1] - p1[1], p0[2] - p1[2]);
  console.log(`bspline deg=${bs.degree} cps=${bs.cps.length} u=[${bs.u0},${bs.u1}] closure gap=${gap.toExponential(3)}`);
}
const s = sampleEdgePolyline(brep.table, e.curveId, e.v0, e.v1, e.sameSense, sc, chordTol, 0.05 * (diag || 20), brep.units.radPerAngle, (15 * Math.PI) / 180);
console.log(`edge ${id}: ${s.length} samples (chordTol=${chordTol.toFixed(5)})`);
for (const p of s) console.log(`  (${p[0].toFixed(4)},${p[1].toFixed(4)},${p[2].toFixed(4)})`);
