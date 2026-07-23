// SPDX-License-Identifier: AGPL-3.0-only
// Analytic vertex normals + parameter-UV regression.
//  - sphere: every vertex normal must be exactly radial (the analytic win over faceted normals).
//  - cylinder: tube-interior normals radial from the axis; caps/rims crease-averaged but unit;
//    UVs: u spans the full 2π turn, v is the axial coordinate (checked against SurfaceInfo).
//  - cube: normals unit + outward.
import { readFileSync } from "node:fs";
import { importStep } from "../src/index.ts";

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}`);
}
const load = (f: string): string => readFileSync(new URL(`../${f}`, import.meta.url), "utf8");

// --- sphere: n(v) must equal (p − center)/R at every vertex, to sub-0.01° accuracy.
{
  const r = importStep(load("sphere.step"), { vertexNormals: true });
  const N = r.normals!, P = r.mesh.positions;
  check("sphere: normals present+sized", N && N.length === P.length, `${N?.length} vs ${P.length}`);
  const sph = [...r.faces.values()].find((f) => f.type === "sphere")!;
  const [cx, cy, cz] = sph.surface.origin!;
  let worst = 1;
  for (let v = 0; v < P.length / 3; v++) {
    const rx = P[v * 3]! - cx, ry = P[v * 3 + 1]! - cy, rz = P[v * 3 + 2]! - cz;
    const rl = Math.hypot(rx, ry, rz);
    const d = (rx * N[v * 3]! + ry * N[v * 3 + 1]! + rz * N[v * 3 + 2]!) / rl;
    if (d < worst) worst = d;
  }
  check("sphere: all normals radial", worst > 0.9999999, `worst dot ${worst}`);
}

// --- cylinder: interior tube vertices radial; every normal unit length; UV structure.
{
  const r = importStep(load("cylinder.step"), { vertexNormals: true, parameterUVs: true });
  const N = r.normals!, P = r.mesh.positions, I = r.mesh.indices;
  const tube = [...r.faces.values()].find((f) => f.type === "cylinder")!;
  const [ox, oy, oz] = tube.surface.origin!;
  const [ax, ay, az] = tube.surface.axis!;
  const R = tube.surface.radius!;

  // Vertices used ONLY by tube triangles (interior — rim vertices are shared with the caps).
  const nV = P.length / 3;
  const onTube = new Uint8Array(nV), offTube = new Uint8Array(nV);
  for (let t = 0; t < r.faceOfTri.length; t++) {
    const mark = r.faceOfTri[t] === tube.faceId ? onTube : offTube;
    for (let e = 0; e < 3; e++) mark[I[t * 3 + e]!] = 1;
  }
  let worst = 1, unit = true, interior = 0;
  for (let v = 0; v < nV; v++) {
    const nl = Math.hypot(N[v * 3]!, N[v * 3 + 1]!, N[v * 3 + 2]!);
    if (Math.abs(nl - 1) > 1e-5) unit = false;
    if (!onTube[v] || offTube[v]) continue;
    interior++;
    // Radial direction at p: (p − o) minus its axial component.
    const px = P[v * 3]! - ox, py = P[v * 3 + 1]! - oy, pz = P[v * 3 + 2]! - oz;
    const h = px * ax + py * ay + pz * az;
    const rx = px - h * ax, ry = py - h * ay, rz = pz - h * az;
    const rl = Math.hypot(rx, ry, rz);
    const d = (rx * N[v * 3]! + ry * N[v * 3 + 1]! + rz * N[v * 3 + 2]!) / rl;
    if (d < worst) worst = d;
  }
  check("cyl: interior tube vertices exist", interior > 10, `got ${interior}`);
  check("cyl: interior normals radial", worst > 0.9999999, `worst dot ${worst}`);
  check("cyl: all normals unit", unit);

  // UVs: tube corners non-NaN; v == axial coordinate; |radial| == R; u spans the full turn.
  const uv = r.uv!;
  check("cyl: uv present+sized", uv && uv.length === I.length * 2, `${uv?.length} vs ${I.length * 2}`);
  const fuv = r.faceUV!.get(tube.faceId)!;
  check("cyl: tube is periodic in u", fuv.uPeriod !== undefined && Math.abs(fuv.uPeriod - 2 * Math.PI) < 1e-9);
  check("cyl: u range spans 2π", fuv.uRange[1] - fuv.uRange[0] >= 2 * Math.PI - 1e-6
    && fuv.uRange[1] - fuv.uRange[0] < 2 * Math.PI + 0.5, `${fuv.uRange[0]}..${fuv.uRange[1]}`);
  let maxVErr = 0, maxRErr = 0, nanCorners = 0;
  for (let t = 0; t < r.faceOfTri.length; t++) {
    if (r.faceOfTri[t] !== tube.faceId) continue;
    for (let e = 0; e < 3; e++) {
      const u = uv[t * 6 + e * 2]!, vv = uv[t * 6 + e * 2 + 1]!;
      if (Number.isNaN(u)) { nanCorners++; continue; }
      const vi = I[t * 3 + e]! * 3;
      const px = P[vi]! - ox, py = P[vi + 1]! - oy, pz = P[vi + 2]! - oz;
      const h = px * ax + py * ay + pz * az;
      maxVErr = Math.max(maxVErr, Math.abs(vv - h));
      maxRErr = Math.max(maxRErr, Math.abs(Math.hypot(px - h * ax, py - h * ay, pz - h * az) - R));
    }
  }
  check("cyl: no NaN tube corners", nanCorners === 0, `${nanCorners}`);
  check("cyl: uv v == axial mm", maxVErr < 1e-4, `maxErr ${maxVErr}`);
  check("cyl: tube corners at radius R", maxRErr < 1e-4, `maxErr ${maxRErr}`);
}

// --- cube: normals unit + pointing away from the centroid (outward, crease-averaged corners).
{
  const r = importStep(load("cube.step"), { vertexNormals: true });
  const N = r.normals!, P = r.mesh.positions;
  const nV = P.length / 3;
  const c = [0, 0, 0];
  for (let v = 0; v < nV; v++) { c[0] += P[v * 3]!; c[1] += P[v * 3 + 1]!; c[2] += P[v * 3 + 2]!; }
  c[0] /= nV; c[1] /= nV; c[2] /= nV;
  let outward = true, unit = true;
  for (let v = 0; v < nV; v++) {
    const nl = Math.hypot(N[v * 3]!, N[v * 3 + 1]!, N[v * 3 + 2]!);
    if (Math.abs(nl - 1) > 1e-5) unit = false;
    const d = (P[v * 3]! - c[0]!) * N[v * 3]! + (P[v * 3 + 1]! - c[1]!) * N[v * 3 + 1]! + (P[v * 3 + 2]! - c[2]!) * N[v * 3 + 2]!;
    if (d <= 0) outward = false;
  }
  check("cube: all normals unit", unit);
  check("cube: all normals outward", outward);
}

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("attributes: all passed");
