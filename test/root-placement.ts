// SPDX-License-Identifier: AGPL-3.0-only
// Root-representation placement regression. A part rep with no assembly parent may list an
// AXIS2_PLACEMENT_3D among its items: the frame its geometry is modelled in. OCCT places the
// part with it (read.step.root.transformation), and files without an assembly tree depend on it
// (jack_in_the_box.stp: 54 ACIS parts, each modelled near its own origin, no NAUO/RRWT, so
// ignoring the frame piles every part on top of the others at the origin).
// Fixture: cube.step with a rotated + shifted placement added to its ABSR. Control: the same
// placement referenced by an ITEM_DEFINED_TRANSFORMATION is an assembly anchor and is NOT applied.
import { readFileSync } from "node:fs";
import { importStep } from "../src/index.ts";

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}`);
}

function bbox(p: Float64Array): [number[], number[]] {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3) for (let k = 0; k < 3; k++) {
    lo[k] = Math.min(lo[k]!, p[i + k]!); hi[k] = Math.max(hi[k]!, p[i + k]!);
  }
  return [lo, hi];
}
const near = (a: number, b: number, tol = 1e-6): boolean => Math.abs(a - b) <= tol;

const src = readFileSync(new URL("../cube.step", import.meta.url), "utf8").replace(/\r\n/g, "\n");
const absr = "#12=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14),#182);";
if (!src.includes(absr)) { console.error("FAIL fixture drift: cube.step no longer contains the expected #12 ABSR"); process.exit(1); }
// Placement at (100,0,0) with local x along world +Y: world = (100 - p.y, p.x, p.z).
const frame = `#400=AXIS2_PLACEMENT_3D('',#401,#402,#403);
#401=CARTESIAN_POINT('',(100.,0.,0.));
#402=DIRECTION('',(0.,0.,1.));
#403=DIRECTION('',(0.,1.,0.));`;
const placed = src.replace(absr, `#12=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#14,#400),#182);\n${frame}`);
const anchored = placed.replace(frame, `${frame}\n#404=ITEM_DEFINED_TRANSFORMATION('','',#400,#120);`);

const [lo0, hi0] = bbox(importStep(src).mesh.positions);

{
  const r = importStep(placed);
  check("placed: diagnostics ok", r.diagnostics.ok);
  const [lo, hi] = bbox(r.mesh.positions);
  const want = [[100 - hi0[1]!, lo0[0]!, lo0[2]!], [100 - lo0[1]!, hi0[0]!, hi0[2]!]];
  check("placed: root rep frame applied",
    [0, 1, 2].every(k => near(lo[k]!, want[0]![k]!) && near(hi[k]!, want[1]![k]!)),
    `got ${lo.join(",")} .. ${hi.join(",")}, want ${want[0]!.join(",")} .. ${want[1]!.join(",")}`);
  check("placed: instance carries the frame", r.instances.length === 1 && r.instances[0]!.frame !== null);
}

{
  const r = importStep(anchored);
  const [lo, hi] = bbox(r.mesh.positions);
  check("IDT anchor: frame not applied",
    [0, 1, 2].every(k => near(lo[k]!, lo0[k]!) && near(hi[k]!, hi0[k]!)),
    `got ${lo.join(",")} .. ${hi.join(",")}`);
}

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("root-placement: all passed");
