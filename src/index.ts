// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — public API.
import { buildBrep } from "./brep/build.ts";
import { tessellate, type MeshResult, type TessOptions } from "./mesh/tessellate.ts";
import { remesh } from "./mesh/remesh.ts";
import { orientConsistent } from "./mesh/orient.ts";
import { makeSurface, type Surface } from "./geom/surfaces.ts";
import type { Frame } from "./geom/placement.ts";
import type { IndexedMesh } from "./io/stl.ts";
import { meshDefects, type ImportDiagnostics } from "./mesh/diag.ts";
import { extractColors, type ModelColors } from "./step/styles.ts";
import { extractStructure, type PartNode } from "./step/structure.ts";

/** Move each part's vertices into its assembly world placement(s). Each vertex belongs to one solid
 * (bodies are welded independently), so the first instance transforms in place; a part used N times
 * in the assembly appends N-1 transformed copies of its vertices and triangles (each copy is its own
 * welded component, so watertightness is preserved per instance). Rigid transforms keep winding. */
function applyAssemblyPlacement(
  mesh: IndexedMesh, faceOfTri: Uint32Array, solidOfTri: Uint32Array, xf: Map<number, Frame[]>,
): { mesh: IndexedMesh; faceOfTri: Uint32Array; solidOfTri: Uint32Array } {
  const unchanged = { mesh, faceOfTri, solidOfTri };
  if (xf.size === 0) return unchanged;
  const P = mesh.positions, I = mesh.indices;
  const nV = P.length / 3;
  const vSolid = new Int32Array(nV).fill(-1);
  for (let t = 0; t < solidOfTri.length; t++) for (let e = 0; e < 3; e++) vSolid[I[t * 3 + e]!] = solidOfTri[t]!;
  const app = (f: Frame, out: Float64Array, o: number, x: number, y: number, z: number): void => {
    out[o] = f.o[0] + f.x[0] * x + f.y[0] * y + f.z[0] * z;
    out[o + 1] = f.o[1] + f.x[1] * x + f.y[1] * y + f.z[1] * z;
    out[o + 2] = f.o[2] + f.x[2] * x + f.y[2] * y + f.z[2] * z;
  };
  // Extra instances first (they read the still-untransformed local coordinates), then instance 0.
  const extraV: number[] = [], extraI: number[] = [], extraF: number[] = [], extraS: number[] = [];
  for (const [sid, frames] of xf) {
    for (let k = 1; k < frames.length; k++) {
      const f = frames[k]!;
      const remap = new Map<number, number>();
      const tmp = new Float64Array(3);
      for (let v = 0; v < nV; v++) {
        if (vSolid[v] !== sid) continue;
        remap.set(v, nV + (extraV.length / 3));
        app(f, tmp, 0, P[v * 3]!, P[v * 3 + 1]!, P[v * 3 + 2]!);
        extraV.push(tmp[0]!, tmp[1]!, tmp[2]!);
      }
      for (let t = 0; t < solidOfTri.length; t++) {
        if (solidOfTri[t] !== sid) continue;
        extraI.push(remap.get(I[t * 3]!)!, remap.get(I[t * 3 + 1]!)!, remap.get(I[t * 3 + 2]!)!);
        extraF.push(faceOfTri[t]!); extraS.push(sid);
      }
    }
  }
  for (let v = 0; v < nV; v++) {
    const f = xf.get(vSolid[v]!)?.[0]; if (!f) continue;
    app(f, P, v * 3, P[v * 3]!, P[v * 3 + 1]!, P[v * 3 + 2]!);
  }
  if (extraV.length === 0) return unchanged;
  const positions = new Float64Array(P.length + extraV.length);
  positions.set(P); positions.set(extraV, P.length);
  const indices = new Uint32Array(I.length + extraI.length);
  indices.set(I); indices.set(extraI, I.length);
  const fo = new Uint32Array(faceOfTri.length + extraF.length);
  fo.set(faceOfTri); fo.set(extraF, faceOfTri.length);
  const so = new Uint32Array(solidOfTri.length + extraS.length);
  so.set(solidOfTri); so.set(extraS, solidOfTri.length);
  return { mesh: { positions, indices }, faceOfTri: fo, solidOfTri: so };
}

/** Assemble the consolidated conversion verdict from the tessellation warnings and a final
 * edge-defect audit of the mesh actually returned (post remesh/placement). `ok` is strict: any
 * missing geometry, edge defect, or heuristic repair clears it — the consumer's cue to suggest
 * exporting a mesh directly from CAD (severity "error" / edge defects) or checking the preview
 * (only "warning"-severity repairs). */
function buildDiagnostics(result: MeshResult, mesh: IndexedMesh, solidOfTri: Uint32Array): ImportDiagnostics {
  const { openEdges, nonManifoldEdges } = meshDefects(mesh, solidOfTri, result.openSolids);
  const facesDropped = result.warnings.filter((w) => w.code === "face-dropped").length;
  const facesSkipped = Object.values(result.stats.skipped).reduce((s, n) => s + n, 0);
  const ok = openEdges === 0 && nonManifoldEdges === 0 && facesDropped === 0 && facesSkipped === 0
    && result.warnings.length === 0;
  return { ok, openEdges, nonManifoldEdges, facesDropped, facesSkipped, warnings: result.warnings };
}

export { writeBinarySTL, readSTL, type IndexedMesh, type TriSoup } from "./io/stl.ts";
export { parseStepHeader, type StepHeader } from "./step/header.ts";
export type { MeshResult, TessOptions } from "./mesh/tessellate.ts";
export type { BrepModel } from "./brep/build.ts";
export { meshDefects, type ImportDiagnostics, type MeshWarning, type WarningCode, type WarningSeverity, type EdgeDefects } from "./mesh/diag.ts";
export { extractColors, type ModelColors, type RGB } from "./step/styles.ts";
export { extractStructure, type PartNode, type PartBody } from "./step/structure.ts";
export { estimateStepSize, autoTessellation, type SizeEstimate } from "./step/measure.ts";

export interface ImportOptions {
  /** Run the curvature-adaptive isotropic remesh (default false). The raw tessellation is already
   * watertight, curvature-adaptive and ruling-aligned on fillets; the isotropic pass predates that
   * pipeline and now measurably degrades it (destroys the aligned diagonals, adds normal noise,
   * +5% triangles, +30% time). Kept as an option for uniform-triangle output (e.g. simulation). */
  remesh?: boolean;
  /** Max chord deviation from the true surface, mm (Fusion "Surface Deviation"). Default 0.01. */
  surfaceDeviation?: number;
  /** Max angle between adjacent normals, degrees (Fusion "Normal Deviation"). Default 15. */
  normalDeviation?: number;
  /** Max edge length, mm (Fusion "Maximum Edge Length"). Default 1. */
  maxEdge?: number;
  remeshIterations?: number;
  /** Diagnostic hook: called once per B-rep face with the mesher that produced it. */
  trace?: TessOptions["trace"];
}

export interface ImportResult extends MeshResult {
  /** Conversion verdict: check `diagnostics.ok` before trusting the mesh; when false, the
   * warnings/counters say whether geometry is missing or leaking (advise the user to export a
   * mesh directly from CAD) or merely heuristically repaired (advise checking the preview). */
  diagnostics: ImportDiagnostics;
  /** Length-unit label detected in the STEP file (e.g. "mm", "inch"); all mesh coordinates are in mm. */
  units: string;
  /** STEP presentation colors (STYLED_ITEM chains), or null when the file has none. Palette-indexed
   * per face/solid with the same ids as faceOfTri/solidOfTri — `palette[faceColor.get(faceOfTri[t])]`
   * is triangle t's sRGB color, and faces sharing a palette index form one color group. */
  colors: ModelColors | null;
  /** Part/component tree from the STEP product structure. The root is the top product (or the
   * single part); each node's `bodies[].id` keys into solidOfTri, so a viewer can hide or
   * highlight a part by filtering triangles on those ids. A part occurring N times in the
   * assembly is one node with `occurrences: N` (instances share solid ids). */
  structure: PartNode;
}

/** Parse a STEP file (ISO-10303-21 text) and tessellate it into a uniform, watertight mesh. */
export function importStep(src: string, opts: ImportOptions = {}): ImportResult {
  const surfaceDev = opts.surfaceDeviation ?? 0.01;
  const maxEdge = opts.maxEdge ?? 1.0;
  const normalDevRad = (opts.normalDeviation ?? 15) * Math.PI / 180;

  const brep = buildBrep(src);
  const colors = extractColors(brep.table, brep.solids);
  const structure = extractStructure(brep.table, brep.solids);
  // Sample boundaries to the surface-deviation tolerance so feature edges (rims, holes) are fine
  // even without remeshing. The robust CDT handles the resulting dense/collinear boundaries.
  // maxEdge is a pure upper CAP on segment length — it must never loosen the chord tolerance
  // (a CAD-style export sets a huge max edge to mean "follow curvature", not "coarsen 20×").
  const tess: TessOptions = { chordTol: surfaceDev, targetEdge: maxEdge, normalDev: normalDevRad, trace: opts.trace };
  const result = tessellate(brep, tess);
  // Assembly placements per solid (empty for a single part); applied to the final mesh below.
  // A part with N occurrences carries N frames and is replicated after meshing.
  const solidXf = new Map<number, Frame[]>();
  for (const solid of brep.solids) {
    if (solid.instances) solidXf.set(solid.id, solid.instances);
    else if (solid.transform) solidXf.set(solid.id, [solid.transform]);
  }
  // AP242 tessellated-geometry bodies have no analytic surfaces, so the curvature-adaptive remesh
  // can't project — return the (already watertight) faceted mesh as imported.
  if (opts.remesh !== true || brep.solids.length === 0) {
    orientConsistent(result.mesh, result.solidOfTri);
    const placed = applyAssemblyPlacement(result.mesh, result.faceOfTri, result.solidOfTri, solidXf);
    return { ...result, ...placed, diagnostics: buildDiagnostics(result, placed.mesh, placed.solidOfTri), units: brep.units.label, colors, structure };
  }

  const surf = new Map<number, Surface | null>();
  const solidOfFace = new Map<number, number>();
  for (const solid of brep.solids) {
    for (const face of solid.faces) {
      surf.set(face.faceId, makeSurface(brep.table, face.surfaceId, solid.scale ?? brep.scale, brep.units.radPerAngle));
      solidOfFace.set(face.faceId, solid.id);
    }
  }
  const r = remesh(result.mesh, result.faceOfTri, surf, {
    surfaceDev, normalDev: normalDevRad, maxEdge, iterations: opts.remeshIterations,
  });
  const solidOfTri = Uint32Array.from(r.faceOfTri, (f) => solidOfFace.get(f) ?? 0);
  orientConsistent(r.mesh, solidOfTri); // fix any triangles flipped by smoothing
  const placed = applyAssemblyPlacement(r.mesh, r.faceOfTri, solidOfTri, solidXf);
  return { ...result, ...placed, diagnostics: buildDiagnostics(result, placed.mesh, placed.solidOfTri), units: brep.units.label, colors, structure };
}
