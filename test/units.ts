// SPDX-License-Identifier: AGPL-3.0-only
// Unit-conversion regression: mesh coordinates must ALWAYS come out in millimetres, whatever
// length unit the STEP file declares. Guards the documented "positions are mm" contract that
// downstream consumers (bumpmesh texture tiles, infeall voxel pitch) hard-depend on.
// Fixture: cube.step (20 mm cube, MILLI METRE context) + the same file with its length unit
// rewritten to CONVERSION_BASED_UNIT('INCH') — the same "20." coordinates must become 508 mm.
import { readFileSync } from "node:fs";
import { importStep } from "../src/index.ts";

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}`);
}

function extents(positions: Float64Array): [number, number, number] {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      const v = positions[i + k]!;
      if (v < lo[k]!) lo[k] = v;
      if (v > hi[k]!) hi[k] = v;
    }
  }
  return [hi[0]! - lo[0]!, hi[1]! - lo[1]!, hi[2]! - lo[2]!];
}

const near = (a: number, b: number, tol = 1e-6): boolean => Math.abs(a - b) <= tol;

const mmSrc = readFileSync(new URL("../cube.step", import.meta.url), "utf8").replace(/\r\n/g, "\n");

// Swap the MILLI METRE length unit (#184) for an inch defined the way real exporters write it:
// CONVERSION_BASED_UNIT('INCH', measure) where the measure expresses 25.4 of a mm base unit.
const mmUnit = `#184=(
LENGTH_UNIT()
NAMED_UNIT(*)
SI_UNIT(.MILLI.,.METRE.)
);`;
const inchUnit = `#184=(
CONVERSION_BASED_UNIT('INCH',#300)
LENGTH_UNIT()
NAMED_UNIT(*)
);
#300=MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#301);
#301=(
LENGTH_UNIT()
NAMED_UNIT(*)
SI_UNIT(.MILLI.,.METRE.)
);`;
if (!mmSrc.includes(mmUnit)) { console.error("FAIL fixture drift: cube.step no longer contains the expected #184 unit block"); process.exit(1); }
const inchSrc = mmSrc.replace(mmUnit, inchUnit);

// --- mm file: 20-unit cube in a mm context → 20 mm on every axis.
{
  const r = importStep(mmSrc);
  check("mm: units label", r.units === "mm", `got "${r.units}"`);
  check("mm: diagnostics ok", r.diagnostics.ok);
  const [x, y, z] = extents(r.mesh.positions);
  check("mm: extents 20 mm", near(x, 20) && near(y, 20) && near(z, 20), `got ${x} ${y} ${z}`);
}

// --- inch file: identical "20." coordinates in an INCH context → 508 mm on every axis.
{
  const r = importStep(inchSrc);
  check("inch: units label", r.units === "in", `got "${r.units}"`);
  check("inch: diagnostics ok", r.diagnostics.ok);
  const [x, y, z] = extents(r.mesh.positions);
  check("inch: extents 508 mm", near(x, 508, 1e-4) && near(y, 508, 1e-4) && near(z, 508, 1e-4), `got ${x} ${y} ${z}`);
}

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("units: all passed");
