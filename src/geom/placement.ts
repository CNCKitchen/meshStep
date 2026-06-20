// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — STEP placements: read points, directions, and AXIS2_PLACEMENT_3D frames.
import type { Vec3 } from "./vec.ts";
import { cross, dot, normalize, sub, scale as vscale } from "./vec.ts";
import { Table, ref, numList } from "../step/entities.ts";

export interface Frame {
  o: Vec3; // origin (mm)
  x: Vec3; // local x (unit)
  y: Vec3; // local y (unit)
  z: Vec3; // local z / axis (unit)
}

export function readPoint(t: Table, id: number, s: number): Vec3 {
  const c = numList(t.record(id).params[1]!); // CARTESIAN_POINT(name, (x,y,z))
  return [(c[0] ?? 0) * s, (c[1] ?? 0) * s, (c[2] ?? 0) * s];
}

export function readDirection(t: Table, id: number): Vec3 {
  const c = numList(t.record(id).params[1]!); // DIRECTION(name, (x,y,z))
  return normalize([c[0] ?? 0, c[1] ?? 0, c[2] ?? 0]);
}

/** Build an orthonormal frame from AXIS2_PLACEMENT_3D(name, location, axis?, refDirection?). */
export function readPlacement(t: Table, id: number, s: number): Frame {
  const r = t.record(id);
  const o = readPoint(t, ref(r.params[1]!), s);
  const z = r.params[2] && r.params[2].k === "ref" ? readDirection(t, ref(r.params[2])) : [0, 0, 1] as Vec3;
  let xref = r.params[3] && r.params[3].k === "ref" ? readDirection(t, ref(r.params[3])) : [1, 0, 0] as Vec3;
  // Orthogonalise xref against z (Gram-Schmidt); fall back if parallel.
  let x = sub(xref, vscale(z, dot(xref, z)));
  if (Math.hypot(x[0], x[1], x[2]) < 1e-9) {
    xref = Math.abs(z[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    x = sub(xref, vscale(z, dot(xref, z)));
  }
  x = normalize(x);
  const y = cross(z, x);
  return { o, x, y, z };
}
