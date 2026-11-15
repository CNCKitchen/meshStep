// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — public API.
import { buildBrep, type BrepModel } from "./brep/build.ts";
import { mergeShortEdges } from "./brep/defeature.ts";
import { tessellate, type FaceTol, type MeshResult, type TessOptions } from "./mesh/tessellate.ts";
import { MeshPool, defaultThreads, type PoolConfig, type WorkerLike } from "./mesh/pool.ts";
import { remesh } from "./mesh/remesh.ts";
import { orientConsistent } from "./mesh/orient.ts";
import { makeSurface, type Surface } from "./geom/surfaces.ts";
import type { Frame } from "./geom/placement.ts";
import type { IndexedMesh } from "./io/stl.ts";
import { meshDefects, type ImportDiagnostics } from "./mesh/diag.ts";
import { extractColors, type ModelColors } from "./step/styles.ts";
import { extractStructure, type PartNode } from "./step/structure.ts";
import { collectMeasureGeometry, type MeasureGeometry } from "./brep/measure-geometry.ts";
import { collectFaceInfo, type FaceInfo } from "./mesh/face-info.ts";
import { computeSurfaceAttributes, type FaceUV } from "./mesh/attributes.ts";

/** One placed occurrence of a solid in the assembly. A part used N times is one solid id with N
 * instances; `instanceOfTri` maps every triangle of the final mesh to its entry in this list. */
export interface SolidInstance {
  /** Body id — same ids as solidOfTri / PartBody.id. */
  solidId: number;
  /** Occurrence index within the solid's placement list — matches MeasureFace/MeasureEdge.instance. */
  instance: number;
  /** Placement applied to this occurrence's vertices (null = meshed in place, identity). */
  frame: Frame | null;
}

/** Move each part's vertices into its assembly world placement(s). Each vertex belongs to one solid
 * (bodies are welded independently), so the first instance transforms in place; a part used N times
 * in the assembly appends N-1 transformed copies of its vertices and triangles (each copy is its own
 * welded component, so watertightness is preserved per instance). Rigid transforms keep winding.
 * Also emits the per-occurrence identity (`instances` + `instanceOfTri`) so consumers can address a
 * single placed copy — the placement itself is baked into the vertices and is otherwise gone. */
function applyAssemblyPlacement(
  mesh: IndexedMesh, faceOfTri: Uint32Array, solidOfTri: Uint32Array, xf: Map<number, Frame[]>,
  attrs: { normals?: Float32Array; uv?: Float32Array } = {},
): { mesh: IndexedMesh; faceOfTri: Uint32Array; solidOfTri: Uint32Array; instances: SolidInstance[]; instanceOfTri: Uint32Array; normals?: Float32Array; uv?: Float32Array } {
  // Instance table covers every solid with triangles (identity-placed ones included), in
  // first-appearance order; instance k of a multi-placed solid sits at firstInst(sid) + k.
  const instances: SolidInstance[] = [];
  const firstInst = new Map<number, number>();
  for (let t = 0; t < solidOfTri.length; t++) {
    const sid = solidOfTri[t]!;
    if (firstInst.has(sid)) continue;
    firstInst.set(sid, instances.length);
    const frames = xf.get(sid);
    if (!frames || frames.length === 0) instances.push({ solidId: sid, instance: 0, frame: null });
    else for (let k = 0; k < frames.length; k++) instances.push({ solidId: sid, instance: k, frame: frames[k]! });
  }
  const baseInstOfTri = new Uint32Array(solidOfTri.length);
  for (let t = 0; t < solidOfTri.length; t++) baseInstOfTri[t] = firstInst.get(solidOfTri[t]!)!;
  const N = attrs.normals, UV = attrs.uv;
  const unchanged = { mesh, faceOfTri, solidOfTri, instances, instanceOfTri: baseInstOfTri, normals: N, uv: UV };
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
  // Rotation-only transform for direction vectors (frames are rigid, so normals just rotate).
  const rotD = (f: Frame, out: Float32Array | number[], o: number, x: number, y: number, z: number): void => {
    out[o] = f.x[0] * x + f.y[0] * y + f.z[0] * z;
    out[o + 1] = f.x[1] * x + f.y[1] * y + f.z[1] * z;
    out[o + 2] = f.x[2] * x + f.y[2] * y + f.z[2] * z;
  };
  // Extra instances first (they read the still-untransformed local coordinates), then instance 0.
  // Each further placement of a solid appends a copy of the solid's vertices (ascending) and of
  // its triangles (ascending), in the order of `xf` and of its frames. The solids' vertex and
  // triangle lists are gathered once, so the cost is the size of the copies, not (number of
  // placements) x (size of the whole mesh).
  const nT = solidOfTri.length;
  const multi = new Map<number, number>(); // solid id -> index among the solids with further placements
  for (const [sid, frames] of xf) if (frames.length > 1 && firstInst.has(sid)) multi.set(sid, multi.size);
  let extraVerts = 0, extraTris = 0;
  const vStart = new Int32Array(multi.size + 1), tStart = new Int32Array(multi.size + 1);
  const vList = new Int32Array(nV), tList = new Int32Array(nT);
  const rank = new Int32Array(nV); // position of a vertex among its solid's vertices
  if (multi.size > 0) {
    const vIdx = new Int32Array(nV).fill(-1), tIdx = new Int32Array(nT).fill(-1);
    for (let v = 0; v < nV; v++) { const m = multi.get(vSolid[v]!); if (m !== undefined) { vIdx[v] = m; vStart[m + 1]!++; } }
    for (let t = 0; t < nT; t++) { const m = multi.get(solidOfTri[t]!); if (m !== undefined) { tIdx[t] = m; tStart[m + 1]!++; } }
    for (let m = 0; m < multi.size; m++) { vStart[m + 1]! += vStart[m]!; tStart[m + 1]! += tStart[m]!; }
    const vFill = vStart.slice(0, multi.size), tFill = tStart.slice(0, multi.size);
    for (let v = 0; v < nV; v++) { const m = vIdx[v]!; if (m >= 0) { rank[v] = vFill[m]! - vStart[m]!; vList[vFill[m]!++] = v; } }
    for (let t = 0; t < nT; t++) { const m = tIdx[t]!; if (m >= 0) tList[tFill[m]!++] = t; }
    for (const [sid, m] of multi) {
      const copies = xf.get(sid)!.length - 1;
      extraVerts += copies * (vStart[m + 1]! - vStart[m]!);
      extraTris += copies * (tStart[m + 1]! - tStart[m]!);
    }
  }
  /** Instance 0 of every placed solid: transformed in place. */
  const placeFirst = (): void => {
    for (let v = 0; v < nV; v++) {
      const f = xf.get(vSolid[v]!)?.[0]; if (!f) continue;
      app(f, P, v * 3, P[v * 3]!, P[v * 3 + 1]!, P[v * 3 + 2]!);
      if (N) rotD(f, N, v * 3, N[v * 3]!, N[v * 3 + 1]!, N[v * 3 + 2]!);
    }
  };
  if (extraVerts === 0) { placeFirst(); return unchanged; }
  const positions = new Float64Array(P.length + extraVerts * 3);
  const indices = new Uint32Array(I.length + extraTris * 3);
  const fo = new Uint32Array(nT + extraTris);
  const so = new Uint32Array(nT + extraTris);
  const io = new Uint32Array(nT + extraTris);
  const normals = N ? new Float32Array(N.length + extraVerts * 3) : undefined;
  const uv = UV ? new Float32Array(UV.length + extraTris * 6) : undefined;
  let vo = nV, to = nT;
  for (const [sid, frames] of xf) {
    const m = multi.get(sid);
    if (m === undefined) continue;
    const base = firstInst.get(sid)!;
    const v0 = vStart[m]!, v1 = vStart[m + 1]!, t0 = tStart[m]!, t1 = tStart[m + 1]!;
    for (let k = 1; k < frames.length; k++) {
      const f = frames[k]!;
      const first = vo; // index of this copy's first vertex
      for (let q = v0; q < v1; q++, vo++) {
        const v = vList[q]!;
        app(f, positions, vo * 3, P[v * 3]!, P[v * 3 + 1]!, P[v * 3 + 2]!);
        if (N) rotD(f, normals!, vo * 3, N[v * 3]!, N[v * 3 + 1]!, N[v * 3 + 2]!);
      }
      for (let q = t0; q < t1; q++, to++) {
        const t = tList[q]!;
        for (let e = 0; e < 3; e++) {
          const v = I[t * 3 + e]!;
          // (a corner on another solid's vertex has no copy here; it read as index 0 before, too)
          indices[to * 3 + e] = vSolid[v] === sid ? first + rank[v]! : 0;
        }
        fo[to] = faceOfTri[t]!; so[to] = sid; io[to] = base + k;
        if (UV) for (let e = 0; e < 6; e++) uv![to * 6 + e] = UV[t * 6 + e]!;
      }
    }
  }
  placeFirst();
  positions.set(P);
  indices.set(I);
  fo.set(faceOfTri);
  so.set(solidOfTri);
  io.set(baseInstOfTri);
  if (N) normals!.set(N);
  if (UV) uv!.set(UV);
  return { mesh: { positions, indices }, faceOfTri: fo, solidOfTri: so, instances, instanceOfTri: io, normals, uv };
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

export { VERSION } from "./version.ts";
export { writeBinarySTL, readSTL, isBinarySTL, indexSoup, type IndexedMesh, type TriSoup } from "./io/stl.ts";
export { read3MF, type ThreeMFModel, type ThreeMFItem, type RGB3MF } from "./io/threemf.ts";
export { parseStepHeader, type StepHeader } from "./step/header.ts";
export type { FaceTol, MeshResult, TessOptions } from "./mesh/tessellate.ts";
export type { BrepModel } from "./brep/build.ts";
export { meshDefects, type ImportDiagnostics, type MeshWarning, type WarningCode, type WarningSeverity, type EdgeDefects } from "./mesh/diag.ts";
export { extractColors, type ModelColors, type RGB } from "./step/styles.ts";
export { extractStructure, type PartNode, type PartBody } from "./step/structure.ts";
export { estimateStepSize, estimateBrepSize, autoTessellation, type SizeEstimate } from "./step/measure.ts";
export type { FaceInfo, FaceSurfaceType } from "./mesh/face-info.ts";
export type { SurfaceInfo } from "./geom/surfaces.ts";
export type { FaceUV, SurfaceAttributes } from "./mesh/attributes.ts";
export type { MeasureGeometry, MeasureEdge, MeasureFace } from "./brep/measure-geometry.ts";

export interface ImportProgress {
  /** "parse" fires once before the B-rep build (duration unknown → indeterminate UI);
   * "tessellate" advances per work unit (edges + faces + solids, so assemblies progress per part);
   * "finalize" fires once before orientation / assembly placement / the diagnostics audit. */
  phase: "parse" | "tessellate" | "finalize";
  /** Work units completed / total. Both are 0 for phases without a known total. */
  done: number;
  total: number;
}

export interface ImportOptions {
  /** Progress hook for long imports. Called synchronously and often (once per tessellation work
   * unit) — keep it cheap and throttle any UI updates on the consumer side. */
  onProgress?: (p: ImportProgress) => void;
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
  /** Also collect per-edge/per-face analytic measurement geometry (exact circle centers/radii,
   * boundary polylines coincident with the mesh) into `ImportResult.measure` (default false). */
  measureGeometry?: boolean;
  /** Cooperative cancellation: when the signal aborts, the import throws `signal.reason` (a
   * DOMException "AbortError" by default) at the next work-unit boundary — one edge/face of
   * latency. The synchronous import cannot be preempted mid-face; worker termination remains the
   * hard-stop fallback for a pathological single face. */
  signal?: AbortSignal;
  /** Compute exact per-vertex normals from the B-rep surfaces into `ImportResult.normals`
   * (default false). Curved faces get the true surface normal at every vertex instead of a
   * faceted average — displacement/shading along them shows no tessellation banding. */
  vertexNormals?: boolean;
  /** Export parameter-space (u,v) per triangle corner into `ImportResult.uv` + per-face ranges
   * into `ImportResult.faceUV` (default false). Lets a consumer map textures in each CAD face's
   * own parameterization (e.g. seamlessly around a cylinder) instead of projecting externally. */
  parameterUVs?: boolean;
  /** Per-face tessellation overrides, keyed by face id (the ids `faceOfTri` reports: the STEP
   * ADVANCED_FACE record numbers). A listed face is meshed with its own `surfaceDeviation` /
   * `normalDeviation` / `maxEdge` (each falls back to the global value when omitted); the edges it
   * shares are sampled at the finest values of the faces they bound, so the mesh stays watertight
   * and the neighbours grade into it. Use it to refine a few faces (a fillet under a stress probe)
   * without re-meshing the whole part finely. Faces are per PART: an override applies to every
   * instance of that part. Not applied by the optional `remesh` pass. Default: none (output
   * bit-identical to an import without the option). */
  faceOverrides?: Record<number, FaceOverride> | Map<number, FaceOverride>;
  /** Isotropic ("pretty") mesh mode: near-equilateral triangles in the style of an FE
   * pre-processor instead of the default ruling-aligned ones. Edges are seeded evenly from one
   * growth-limited size field and faces are meshed without the fillet stretch, so small fillets
   * cost more triangles. Default false (output bit-identical to an import without the option). */
  isotropic?: boolean;
  /** Isotropic mode: size ratio allowed between neighbouring triangles, i.e. how fast the mesh
   * coarsens away from a fine feature (FE "growth rate"). Default 1.5. */
  growthRate?: number;
  /** Isotropic mode: smallest edge length the mesher may ask for, mm. Default
   * min(maxEdge / 40, 30 · surfaceDeviation). */
  minEdge?: number;
  /** Isotropic mode: mesh fillets and other single-curved four-sided faces as a mapped mesh (a
   * grid of near-square cells, two sides along the fillet and two across it, each cut into two
   * triangles) so they print without the small contour wobbles free triangles cause there;
   * their triangles are right-angled instead of equilateral. Default true. */
  mappedFillets?: boolean;
  /** Geometry clean-up before meshing, mm: an edge shorter than this collapses to a point (its
   * two end vertices merge), so a small step or offset between faces no longer forces a tiny
   * triangle and, in the isotropic mode, a patch of fine mesh around it. The surface moves by
   * up to this distance at such a spot, independently of `surfaceDeviation`. An edge stays when
   * merging it would pull a face beyond the end of its surface, press a wall or gap flat, or
   * make a face boundary touch itself; and if the merged model still meshes less soundly than
   * the untouched one, the untouched one is returned (see `ImportResult.merged`). Works in both
   * mesh modes.
   * Default 0 (off; output bit-identical to an import without the option). */
  mergeTolerance?: number;
}

/** Outcome of the geometry clean-up (`ImportOptions.mergeTolerance`). */
export interface MergeSummary {
  /** Edges removed (collapsed to a point, or replaced by the edge across a sliver). */
  edges: number;
  /** Faces removed (specks and step risers). */
  faces: number;
  /** Largest distance a face boundary moved, mm. */
  maxMove: number;
  /** True when the merged model meshed less soundly than the untouched one (open or
   * non-manifold edges, mesher warnings) and the result is therefore the UNMERGED model. */
  reverted: boolean;
}

/** One face's tessellation override (`ImportOptions.faceOverrides`); same units as the globals. */
export interface FaceOverride {
  surfaceDeviation?: number;
  normalDeviation?: number;
  maxEdge?: number;
}

export interface ImportResult extends MeshResult {
  /** Present when `mergeTolerance` was set: what the geometry clean-up did. */
  merged?: MergeSummary;
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
  /** Analytic measurement geometry (per-edge curve identity + exact boundary polylines,
   * instance-placed like the mesh) when `measureGeometry` was requested. */
  measure?: MeasureGeometry;
  /** Unit per-vertex normals (3 per vertex, aligned with mesh.positions) when `vertexNormals`
   * was requested — analytic from the B-rep surface wherever the vertex lies on one (exact on
   * curved faces, crease-averaged at feature edges), faceted fallback elsewhere. */
  normals?: Float32Array;
  /** Parameter-space (u,v) per triangle corner (2 floats per corner, aligned with mesh.indices)
   * when `parameterUVs` was requested; NaN for corners without an analytic surface. Per corner,
   * not per vertex: a welded vertex has one (u,v) PER FACE, and periodic seams need two branches. */
  uv?: Float32Array;
  /** Per-face parameter ranges/periods for normalizing `uv` into texture space. */
  faceUV?: Map<number, FaceUV>;
  /** Per-face metadata keyed by the ids in `faceOfTri`: normalized surface class, analytic
   * identity (part-local origin/axis/radius), mesh area (mm²) and mean outward normal — enough to
   * flood-select a whole CAD face from one picked triangle or filter faces by type/orientation. */
  faces: Map<number, FaceInfo>;
  /** Every placed part occurrence: a solid used ×N in the assembly is N entries here (its
   * placements are baked into the vertices — this is the only surviving per-copy identity). */
  instances: SolidInstance[];
  /** Per-triangle index into `instances`, aligned with solidOfTri/faceOfTri. */
  instanceOfTri: Uint32Array;
}

/** `faceOverrides` to internal per-face tolerances (undefined when there are none). */
function resolveFaceOverrides(
  fo: ImportOptions["faceOverrides"], surfaceDev: number, maxEdge: number, normalDevRad: number,
): Map<number, FaceTol> | undefined {
  if (!fo) return undefined;
  const entries: [number, FaceOverride][] = fo instanceof Map
    ? [...fo.entries()]
    : Object.entries(fo).map(([k, v]) => [Number(k), v] as [number, FaceOverride]);
  if (entries.length === 0) return undefined;
  const pos = (v: number | undefined, d: number): number => (v !== undefined && Number.isFinite(v) && v > 0 ? v : d);
  const out = new Map<number, FaceTol>();
  for (const [fid, o] of entries) {
    if (!Number.isFinite(fid) || !o) continue;
    out.set(fid, {
      chordTol: pos(o.surfaceDeviation, surfaceDev),
      targetEdge: pos(o.maxEdge, maxEdge),
      normalDev: o.normalDeviation !== undefined && Number.isFinite(o.normalDeviation) && o.normalDeviation > 0
        ? o.normalDeviation * Math.PI / 180 : normalDevRad,
    });
  }
  return out.size > 0 ? out : undefined;
}

/** Options of `importStepAsync` on top of the import options. */
export interface ParallelOptions {
  /** Worker threads to mesh on. Default: half the machine's hardware threads, at most 6, and
   * fewer for a very large file (each worker holds the parsed model). 1 (or less) meshes on
   * the calling thread, like `importStep`. */
  threads?: number;
  /** Creates one worker running meshStep's worker module (`meshstep/worker`). Under Node the
   * default starts worker threads itself; in a browser, or when the library is bundled into a
   * single file, pass e.g.
   * `() => new Worker(new URL("meshstep/worker", import.meta.url), { type: "module" })`.
   * Without a way to create workers the import runs on the calling thread. */
  createWorker?: () => WorkerLike;
}
export type { WorkerLike } from "./mesh/pool.ts";

/** Everything of an import before tessellation: the model read, cleaned up, and the options
 * resolved. */
interface ImportStart {
  surfaceDev: number;
  maxEdge: number;
  normalDevRad: number;
  mergeTol: number;
  brep: BrepModel;
  merged: MergeSummary | undefined;
  colors: ModelColors | null;
  structure: PartNode;
  tess: TessOptions;
}

function importBegin(src: string, opts: ImportOptions): ImportStart {
  const surfaceDev = opts.surfaceDeviation ?? 0.01;
  const maxEdge = opts.maxEdge ?? 1.0;
  const normalDevRad = (opts.normalDeviation ?? 15) * Math.PI / 180;
  const onProgress = opts.onProgress;
  const signal = opts.signal;

  signal?.throwIfAborted();
  onProgress?.({ phase: "parse", done: 0, total: 0 });
  const brep = buildBrep(src);
  const mergeTol = opts.mergeTolerance !== undefined && opts.mergeTolerance > 0 ? opts.mergeTolerance : 0;
  let merged: MergeSummary | undefined;
  if (mergeTol > 0) {
    const m = mergeShortEdges(brep, mergeTol, { chordTol: surfaceDev, targetEdge: maxEdge, normalDev: normalDevRad });
    merged = { edges: m.edges, faces: m.faces, maxMove: m.maxMove, reverted: false };
  }
  signal?.throwIfAborted();
  const colors = extractColors(brep.table, brep.solids);
  const structure = extractStructure(brep.table, brep.solids);
  // Sample boundaries to the surface-deviation tolerance so feature edges (rims, holes) are fine
  // even without remeshing. The robust CDT handles the resulting dense/collinear boundaries.
  // maxEdge is a pure upper CAP on segment length — it must never loosen the chord tolerance
  // (a CAD-style export sets a huge max edge to mean "follow curvature", not "coarsen 20×").
  const tess: TessOptions = {
    chordTol: surfaceDev, targetEdge: maxEdge, normalDev: normalDevRad, trace: opts.trace,
    // The abort check rides the progress hook: it fires once per work unit at the tessellation
    // loop's top level (never inside a face's rescue try/catch), so the throw always escapes.
    onProgress: (onProgress || signal) && ((done, total) => {
      signal?.throwIfAborted();
      onProgress?.({ phase: "tessellate", done, total });
    }),
    collectEdgePolylines: opts.measureGeometry,
    faceTol: resolveFaceOverrides(opts.faceOverrides, surfaceDev, maxEdge, normalDevRad),
    isotropic: opts.isotropic, growthRate: opts.growthRate, minEdge: opts.minEdge, mappedFillets: opts.mappedFillets,
  };
  return { surfaceDev, maxEdge, normalDevRad, mergeTol, brep, merged, colors, structure, tess };
}

/** How unsound a tessellation is (the merge safety net compares the merged model's with the
 * untouched one's). */
function trouble(r: MeshResult): number {
  const d = meshDefects(r.mesh, r.solidOfTri, r.openSolids);
  return d.openEdges + d.nonManifoldEdges + r.warnings.length;
}

/** The merge safety net decided for the untouched model: carry on with that one. */
function revertMerge(c: ImportStart, plainBrep: BrepModel): void {
  c.brep = plainBrep;
  c.merged!.reverted = true;
  c.colors = extractColors(plainBrep.table, plainBrep.solids);
  c.structure = extractStructure(plainBrep.table, plainBrep.solids);
}

/** Everything of an import after tessellation. */
function importEnd(c: ImportStart, result: MeshResult, opts: ImportOptions): ImportResult {
  const { surfaceDev, maxEdge, normalDevRad, brep, merged, colors, structure } = c;
  const signal = opts.signal;
  opts.onProgress?.({ phase: "finalize", done: 0, total: 0 });
  // Assembly placements per solid (empty for a single part); applied to the final mesh below.
  // A part with N occurrences carries N frames and is replicated after meshing.
  const solidXf = new Map<number, Frame[]>();
  for (const solid of brep.solids) {
    if (solid.instances) solidXf.set(solid.id, solid.instances);
    else if (solid.transform) solidXf.set(solid.id, [solid.transform]);
  }
  const measure = opts.measureGeometry && result.edgePolylines
    ? collectMeasureGeometry(brep, result.edgePolylines, solidXf) : undefined;
  delete result.edgePolylines; // consumed above — keep the raw polyline map out of the result
  // AP242 tessellated-geometry bodies have no analytic surfaces, so the curvature-adaptive remesh
  // can't project — return the (already watertight) faceted mesh as imported.
  if (opts.remesh !== true || brep.solids.length === 0) {
    orientConsistent(result.mesh, result.solidOfTri);
    const faces = collectFaceInfo(result.mesh, result.faceOfTri, result.solidOfTri, brep);
    const attrs = opts.vertexNormals || opts.parameterUVs
      ? computeSurfaceAttributes(result.mesh, result.faceOfTri, brep,
        { normals: opts.vertexNormals, uvs: opts.parameterUVs }, Math.max(1e-6, 4 * surfaceDev))
      : {};
    const placed = applyAssemblyPlacement(result.mesh, result.faceOfTri, result.solidOfTri, solidXf, attrs);
    return { ...result, ...placed, diagnostics: buildDiagnostics(result, placed.mesh, placed.solidOfTri), units: brep.units.label, colors, structure, measure, faces, faceUV: attrs.faceUV, merged };
  }

  const surf = new Map<number, Surface | null>();
  const solidOfFace = new Map<number, number>();
  for (const solid of brep.solids) {
    for (const face of solid.faces) {
      surf.set(face.faceId, makeSurface(brep.table, face.surfaceId, solid.scale ?? brep.scale, brep.units.radPerAngle));
      solidOfFace.set(face.faceId, solid.id);
    }
  }
  signal?.throwIfAborted();
  const r = remesh(result.mesh, result.faceOfTri, surf, {
    surfaceDev, normalDev: normalDevRad, maxEdge, iterations: opts.remeshIterations,
  });
  const solidOfTri = Uint32Array.from(r.faceOfTri, (f) => solidOfFace.get(f) ?? 0);
  orientConsistent(r.mesh, solidOfTri); // fix any triangles flipped by smoothing
  const faces = collectFaceInfo(r.mesh, r.faceOfTri, solidOfTri, brep);
  const attrs = opts.vertexNormals || opts.parameterUVs
    ? computeSurfaceAttributes(r.mesh, r.faceOfTri, brep,
      { normals: opts.vertexNormals, uvs: opts.parameterUVs }, Math.max(1e-6, 4 * surfaceDev))
    : {};
  const placed = applyAssemblyPlacement(r.mesh, r.faceOfTri, solidOfTri, solidXf, attrs);
  return { ...result, ...placed, diagnostics: buildDiagnostics(result, placed.mesh, placed.solidOfTri), units: brep.units.label, colors, structure, measure, faces, faceUV: attrs.faceUV, merged };
}

/** Parse a STEP file (ISO-10303-21 text) and tessellate it into a uniform, watertight mesh. */
export function importStep(src: string, opts: ImportOptions = {}): ImportResult {
  const c = importBegin(src, opts);
  let result = tessellate(c.brep, c.tess);
  opts.signal?.throwIfAborted();
  // SAFETY NET for the merge: a merged model may never come out less sound than the model as
  // it is. If the merged mesh has an open or non-manifold edge or any mesher warning, mesh the
  // untouched model as well and keep that one unless it is no better. Costs a second
  // tessellation only when something is wrong.
  if (c.merged && c.merged.edges > 0) {
    const tm = trouble(result);
    if (tm > 0) {
      const plainBrep = buildBrep(src);
      const plain = tessellate(plainBrep, c.tess);
      opts.signal?.throwIfAborted();
      if (trouble(plain) < tm) { result = plain; revertMerge(c, plainBrep); }
    }
  }
  return importEnd(c, result, opts);
}

/**
 * `importStep` with the faces meshed on worker threads: the same result (vertices, triangles,
 * ids, warnings), sooner on a machine with several cores. A large model takes about half the
 * time in the default mode; less is gained in the isotropic mode, where half the work is
 * preparation that one thread does. A small model is meshed on the calling thread, without a
 * worker being started.
 * Every worker reads the model itself, so memory use grows with the thread count: about one
 * more copy of the parsed model per thread.
 * `signal` aborts as in `importStep`, and here also while a worker is in the middle of a face.
 */
export async function importStepAsync(src: string, opts: ImportOptions & ParallelOptions = {}): Promise<ImportResult> {
  const threads = Math.floor(opts.threads ?? defaultThreads(src.length));
  if (!(threads > 1)) return importStep(src, opts);
  opts.signal?.throwIfAborted();
  const surfaceDev = opts.surfaceDeviation ?? 0.01;
  const maxEdge = opts.maxEdge ?? 1.0;
  const normalDevRad = (opts.normalDeviation ?? 15) * Math.PI / 180;
  const faceTol = resolveFaceOverrides(opts.faceOverrides, surfaceDev, maxEdge, normalDevRad);
  const cfg: PoolConfig = {
    chordTol: surfaceDev, targetEdge: maxEdge, normalDev: normalDevRad,
    faceTol: faceTol ? [...faceTol] : null,
    isotropic: !!opts.isotropic, growthRate: opts.growthRate, minEdge: opts.minEdge, mappedFillets: opts.mappedFillets,
    mergeTol: opts.mergeTolerance !== undefined && opts.mergeTolerance > 0 ? opts.mergeTolerance : 0,
    trace: !!opts.trace,
  };
  // The workers start reading the model now, while this thread reads it too.
  const pool = await MeshPool.open(src, cfg, threads, opts.createWorker);
  if (!pool) return importStep(src, opts);
  try {
    const c = importBegin(src, opts);
    let result = await pool.run(c.brep, c.tess, opts.signal);
    opts.signal?.throwIfAborted();
    if (c.merged && c.merged.edges > 0) { // the merge safety net, as in importStep
      const tm = trouble(result);
      if (tm > 0) {
        const plainBrep = buildBrep(src);
        pool.reinit({ ...cfg, mergeTol: 0 });
        const plain = await pool.run(plainBrep, c.tess, opts.signal);
        opts.signal?.throwIfAborted();
        if (trouble(plain) < tm) { result = plain; revertMerge(c, plainBrep); }
      }
    }
    return importEnd(c, result, opts);
  } finally {
    pool.close();
  }
}
