// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — public API.
import { buildBrep } from "./brep/build.ts";
import { tessellate, type MeshResult, type TessOptions } from "./mesh/tessellate.ts";
import { remesh } from "./mesh/remesh.ts";
import { orientConsistent } from "./mesh/orient.ts";
import { makeSurface, type Surface } from "./geom/surfaces.ts";
import type { Frame } from "./geom/placement.ts";
import type { IndexedMesh } from "./io/stl.ts";

/** Move each part's vertices into its assembly world placement (each vertex belongs to one solid,
 * since bodies are welded independently). Rigid transforms preserve winding/watertightness. */
function applyAssemblyPlacement(mesh: IndexedMesh, solidOfTri: Uint32Array, xf: Map<number, Frame>): void {
  if (xf.size === 0) return;
  const P = mesh.positions, I = mesh.indices;
  const vSolid = new Int32Array(P.length / 3).fill(-1);
  for (let t = 0; t < solidOfTri.length; t++) for (let e = 0; e < 3; e++) vSolid[I[t * 3 + e]!] = solidOfTri[t]!;
  for (let v = 0; v < P.length / 3; v++) {
    const f = xf.get(vSolid[v]!); if (!f) continue;
    const x = P[v * 3]!, y = P[v * 3 + 1]!, z = P[v * 3 + 2]!;
    P[v * 3] = f.o[0] + f.x[0] * x + f.y[0] * y + f.z[0] * z;
    P[v * 3 + 1] = f.o[1] + f.x[1] * x + f.y[1] * y + f.z[1] * z;
    P[v * 3 + 2] = f.o[2] + f.x[2] * x + f.y[2] * y + f.z[2] * z;
  }
}

export { writeBinarySTL, readSTL, type IndexedMesh, type TriSoup } from "./io/stl.ts";
export type { MeshResult, TessOptions } from "./mesh/tessellate.ts";
export type { BrepModel } from "./brep/build.ts";

export interface ImportOptions {
  /** Run the curvature-adaptive isotropic remesh (default true). */
  remesh?: boolean;
  /** Max chord deviation from the true surface, mm (Fusion "Surface Deviation"). Default 0.01. */
  surfaceDeviation?: number;
  /** Max angle between adjacent normals, degrees (Fusion "Normal Deviation"). Default 15. */
  normalDeviation?: number;
  /** Max edge length, mm (Fusion "Maximum Edge Length"). Default 1. */
  maxEdge?: number;
  remeshIterations?: number;
}

/** Parse a STEP file (ISO-10303-21 text) and tessellate it into a uniform, watertight mesh. */
export function importStep(src: string, opts: ImportOptions = {}): MeshResult {
  const surfaceDev = opts.surfaceDeviation ?? 0.01;
  const maxEdge = opts.maxEdge ?? 1.0;
  const normalDevRad = (opts.normalDeviation ?? 15) * Math.PI / 180;

  const brep = buildBrep(src);
  // Sample boundaries to the surface-deviation tolerance so feature edges (rims, holes) are fine
  // even without remeshing. The robust CDT handles the resulting dense/collinear boundaries.
  const tessChordTol = Math.max(surfaceDev, maxEdge / 500);
  const tess: TessOptions = { chordTol: tessChordTol, targetEdge: maxEdge, normalDev: normalDevRad };
  const result = tessellate(brep, tess);
  // Assembly placements per solid (empty for a single part); applied to the final mesh below.
  const solidXf = new Map<number, Frame>();
  for (const solid of brep.solids) if (solid.transform) solidXf.set(solid.id, solid.transform);
  // AP242 tessellated-geometry bodies have no analytic surfaces, so the curvature-adaptive remesh
  // can't project — return the (already watertight) faceted mesh as imported.
  if (opts.remesh === false || brep.solids.length === 0) {
    orientConsistent(result.mesh);
    applyAssemblyPlacement(result.mesh, result.solidOfTri, solidXf);
    return result;
  }

  const surf = new Map<number, Surface | null>();
  const solidOfFace = new Map<number, number>();
  for (const solid of brep.solids) {
    for (const face of solid.faces) {
      surf.set(face.faceId, makeSurface(brep.table, face.surfaceId, brep.scale, brep.units.radPerAngle));
      solidOfFace.set(face.faceId, solid.id);
    }
  }
  const r = remesh(result.mesh, result.faceOfTri, surf, {
    surfaceDev, normalDev: normalDevRad, maxEdge, iterations: opts.remeshIterations,
  });
  orientConsistent(r.mesh); // fix any triangles flipped by smoothing
  const solidOfTri = Uint32Array.from(r.faceOfTri, (f) => solidOfFace.get(f) ?? 0);
  applyAssemblyPlacement(r.mesh, solidOfTri, solidXf);
  return { ...result, mesh: r.mesh, faceOfTri: r.faceOfTri, solidOfTri };
}
