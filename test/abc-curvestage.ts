// SPDX-License-Identifier: AGPL-3.0-only
// Stage-by-stage debug of sampleEdgePolyline's generic path for one edge.
//   node test/abc-curvestage.ts <step-file> <edgeId>
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { estimateBrepSize } from "../src/step/measure.ts";
import { makeCurve, sampleCurve } from "../src/geom/curves.ts";

const brep = buildBrep(readFileSync(process.argv[2]!, "utf8"));
const id = Number(process.argv[3]);
const e = brep.edges.get(id)!;
const sc = e.scale ?? brep.scale;
const diag = estimateBrepSize(brep)?.diag ?? 0;
const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
const maxSegLen = 0.05 * (diag || 20);
const c = makeCurve(brep.table, e.curveId, sc, brep.units.radPerAngle);
console.log(`makeCurve -> ${c ? c.kind : "NULL"}`);
if (c) {
  console.log(`t0=${c.t0} t1=${c.t1} closed=${(c as any).closed}`);
  const pts = sampleCurve(c, chordTol, maxSegLen, c.t0, c.t1, (15 * Math.PI) / 180);
  console.log(`sampleCurve -> ${pts.length} pts; first=(${pts[0]!.map((x) => x.toFixed(4))}) last=(${pts[pts.length - 1]!.map((x) => x.toFixed(4))})`);
  const d = Math.hypot(pts[0]![0] - pts[pts.length - 1]![0], pts[0]![1] - pts[pts.length - 1]![1], pts[0]![2] - pts[pts.length - 1]![2]);
  console.log(`ring closure dist=${d.toExponential(3)} ringClosed-thresh=${Math.max(1e-9, chordTol * 1e-3).toExponential(3)}`);
}
