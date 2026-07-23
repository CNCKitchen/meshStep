// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — per-face metadata for the public API: surface class, analytic identity, mesh area and
// mean normal per B-rep face. Built for consumers that select or filter whole CAD faces on the
// triangle mesh (flood by faceOfTri, "exclude all planar faces", cylinder-axis texture projection).
import type { BrepModel } from "../brep/build.ts";
import type { IndexedMesh } from "../io/stl.ts";
import { analyzeSurface, type SurfaceInfo } from "../geom/surfaces.ts";
import type { Vec3 } from "../geom/vec.ts";

/** Normalized surface class, for filtering without STEP entity-name knowledge. */
export type FaceSurfaceType =
  | "plane" | "cylinder" | "cone" | "sphere" | "torus"
  | "bspline" | "revolution" | "extrusion" | "offset" | "other";

export interface FaceInfo {
  /** B-rep face id — the values stored in `faceOfTri`. */
  faceId: number;
  /** Owning body id — the values stored in `solidOfTri`. */
  solidId: number;
  type: FaceSurfaceType;
  /** Analytic surface identity (raw STEP kind + origin/axis/radius/semiAngle, mm / radians).
   * Coordinates are in the PART-LOCAL frame: for a placed assembly occurrence, map them through
   * `instances[i].frame`. Swept/B-spline/facetted surfaces report only their kind. */
  surface: SurfaceInfo;
  /** Face area in mm², summed over one meshed copy of the face (assembly instances share it). */
  area: number;
  /** Area-weighted average of the face's triangle normals (unit length, part-local frame, pointing
   * out of the solid). Near-zero magnitude before normalization — e.g. a full cylinder whose
   * normals cancel — yields [0,0,0]. */
  meanNormal: Vec3;
  triangleCount: number;
}

const TYPE_OF_KIND: Record<string, FaceSurfaceType> = {
  PLANE: "plane",
  CYLINDRICAL_SURFACE: "cylinder",
  CONICAL_SURFACE: "cone",
  SPHERICAL_SURFACE: "sphere",
  TOROIDAL_SURFACE: "torus",
  DEGENERATE_TOROIDAL_SURFACE: "torus",
  B_SPLINE_SURFACE_WITH_KNOTS: "bspline",
  B_SPLINE_SURFACE: "bspline",
  SURFACE_OF_REVOLUTION: "revolution",
  SURFACE_OF_LINEAR_EXTRUSION: "extrusion",
  OFFSET_SURFACE: "offset",
};

/**
 * Collect per-face metadata from the tessellated mesh + the B-rep it came from. Runs on the
 * PRE-PLACEMENT mesh (part-local coordinates): area is rigid-invariant so it equals the placed
 * face's area, and `surface`/`meanNormal` are documented as part-local. Faces are enumerated from
 * `faceOfTri` so every meshing path is covered — faces of AP242 tessellated bodies (no analytic
 * surface) report kind "" / type "other".
 */
export function collectFaceInfo(
  mesh: IndexedMesh, faceOfTri: Uint32Array, solidOfTri: Uint32Array, brep: BrepModel,
): Map<number, FaceInfo> {
  // faceId -> (surfaceId, unit scale) from the B-rep, for the analytic identity.
  const surfaceOf = new Map<number, { surfaceId: number; scale: number }>();
  for (const solid of brep.solids) {
    const s = solid.scale ?? brep.scale;
    for (const face of solid.faces) surfaceOf.set(face.faceId, { surfaceId: face.surfaceId, scale: s });
  }

  const P = mesh.positions, I = mesh.indices;
  const acc = new Map<number, { solidId: number; area: number; n: [number, number, number]; tris: number }>();
  for (let t = 0; t < faceOfTri.length; t++) {
    const fid = faceOfTri[t]!;
    let a = acc.get(fid);
    if (!a) { a = { solidId: solidOfTri[t]!, area: 0, n: [0, 0, 0], tris: 0 }; acc.set(fid, a); }
    const i0 = I[t * 3]! * 3, i1 = I[t * 3 + 1]! * 3, i2 = I[t * 3 + 2]! * 3;
    const ux = P[i1]! - P[i0]!, uy = P[i1 + 1]! - P[i0 + 1]!, uz = P[i1 + 2]! - P[i0 + 2]!;
    const vx = P[i2]! - P[i0]!, vy = P[i2 + 1]! - P[i0 + 1]!, vz = P[i2 + 2]! - P[i0 + 2]!;
    // Cross product = 2·area·normal — accumulating it raw IS the area-weighted normal sum.
    const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
    a.area += Math.hypot(cx, cy, cz) / 2;
    a.n[0] += cx; a.n[1] += cy; a.n[2] += cz;
    a.tris++;
  }

  const out = new Map<number, FaceInfo>();
  for (const [fid, a] of acc) {
    const src = surfaceOf.get(fid);
    const surface: SurfaceInfo = src
      ? analyzeSurface(brep.table, src.surfaceId, src.scale, brep.units.radPerAngle)
      : { kind: "" };
    // Complex/rational B-spline surfaces have no simple typeOf → kind "" — but only call them
    // bspline when the face came from an analytic B-rep; facetted faces are genuinely "other".
    const type: FaceSurfaceType = TYPE_OF_KIND[surface.kind] ?? (src ? "bspline" : "other");
    const l = Math.hypot(a.n[0], a.n[1], a.n[2]);
    const meanNormal: Vec3 = l > 1e-9 * Math.max(1, a.area) ? [a.n[0] / l, a.n[1] / l, a.n[2] / l] : [0, 0, 0];
    out.set(fid, { faceId: fid, solidId: a.solidId, type, surface, area: a.area, meanNormal, triangleCount: a.tris });
  }
  return out;
}
