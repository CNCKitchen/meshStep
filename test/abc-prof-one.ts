// SPDX-License-Identifier: AGPL-3.0-only
// One-shot ABC-model profiler: mirrors abc-worker's scale-relative tolerances and
// reports per-phase wall times. Run under --cpu-prof to get a .cpuprofile.
//   node --cpu-prof --cpu-prof-dir out/prof test/abc-prof-one.ts <step-file>
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { orientConsistent } from "../src/mesh/orient.ts";
import { estimateBrepSize } from "../src/step/measure.ts";

const path = process.argv[2]!;
let t = Date.now();
const src = readFileSync(path, "utf8");
const tRead = Date.now() - t;
t = Date.now();
const brep = buildBrep(src);
const tBrep = Date.now() - t;
t = Date.now();
const diag = estimateBrepSize(brep)?.diag ?? 0;
const tSize = Date.now() - t;
const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
const targetEdge = diag > 0 ? 0.05 * diag : 1;
t = Date.now();
const res = tessellate(brep, { chordTol, targetEdge, normalDev: (15 * Math.PI) / 180 });
const tTess = Date.now() - t;
t = Date.now();
try { orientConsistent(res.mesh, res.solidOfTri); } catch { /* cosmetic */ }
const tOrient = Date.now() - t;
console.log(
  `read=${tRead}ms brep=${tBrep}ms size=${tSize}ms tess=${tTess}ms orient=${tOrient}ms ` +
  `diag=${diag.toFixed(2)} solids=${brep.solids.length} faces=${res.stats.facesTotal} ` +
  `tris=${res.mesh.indices.length / 3} skipped=${JSON.stringify(res.stats.skipped)}`,
);
