// SPDX-License-Identifier: AGPL-3.0-only
// Per-face metadata regression: importStep(...).faces must report surface class, analytic
// identity, area and mean normal per B-rep face (cube = 6 planes of 400 mm²; cylinder = tube
// face whose normals cancel + 2 planar caps, radius from the STEP record).
import { readFileSync } from "node:fs";
import { importStep } from "../src/index.ts";

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}`);
}
const near = (a: number, b: number, tol: number): boolean => Math.abs(a - b) <= tol;

// --- cube.step: 20 mm cube → 6 planar faces, 400 mm² each, axis-aligned outward normals.
{
  const r = importStep(readFileSync(new URL("../cube.step", import.meta.url), "utf8"));
  const faces = [...r.faces.values()];
  check("cube: 6 faces", faces.length === 6, `got ${faces.length}`);
  check("cube: all planes", faces.every((f) => f.type === "plane" && f.surface.kind === "PLANE"));
  check("cube: areas 400 mm²", faces.every((f) => near(f.area, 400, 1e-6)),
    faces.map((f) => f.area.toFixed(3)).join(","));
  // Every mean normal is a unit axis vector, and the 6 of them sum to zero (closed box).
  const sum = [0, 0, 0];
  let unitAxis = true;
  for (const f of faces) {
    const [x, y, z] = f.meanNormal;
    if (!near(Math.abs(x) + Math.abs(y) + Math.abs(z), 1, 1e-9)) unitAxis = false;
    sum[0] += x; sum[1] += y; sum[2] += z;
  }
  check("cube: axis-aligned unit normals", unitAxis);
  check("cube: normals sum to zero", near(Math.hypot(sum[0]!, sum[1]!, sum[2]!), 0, 1e-9));
  check("cube: faceOfTri ids covered", r.faceOfTri.every((fid) => r.faces.has(fid)));
  // Total mesh area equals the summed face areas (same triangles, grouped differently).
  check("cube: total area 2400 mm²", near(faces.reduce((s, f) => s + f.area, 0), 2400, 1e-6));
}

// --- cylinder.step: one cylindrical tube (normals cancel → meanNormal 0) + two planar caps.
{
  const r = importStep(readFileSync(new URL("../cylinder.step", import.meta.url), "utf8"));
  const faces = [...r.faces.values()];
  const tubes = faces.filter((f) => f.type === "cylinder");
  const planes = faces.filter((f) => f.type === "plane");
  check("cyl: 1 cylinder + 2 planes", tubes.length === 1 && planes.length === 2,
    faces.map((f) => f.type).join(","));
  const tube = tubes[0]!;
  check("cyl: radius recorded", (tube.surface.radius ?? 0) > 0, `got ${tube.surface.radius}`);
  check("cyl: axis recorded", tube.surface.axis !== undefined);
  check("cyl: full tube meanNormal ~0", Math.hypot(...tube.meanNormal) < 1e-6,
    tube.meanNormal.join(","));
  const rr = tube.surface.radius!;
  check("cyl: cap area = πr²", planes.every((f) => near(f.area, Math.PI * rr * rr, 0.02 * rr * rr)),
    planes.map((f) => f.area.toFixed(2)).join(","));
}

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("face-info: all passed");
