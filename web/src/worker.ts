// SPDX-License-Identifier: AGPL-3.0-only
/// <reference lib="webworker" />
// Runs the meshStep pipeline off the main thread so the UI stays responsive.
// Produces the raw tessellation (remesh: false) from one STEP file.
import { importStep, estimateStepSize, readSTL, indexSoup, meshDefects, type ImportOptions, type ImportDiagnostics, type MeasureGeometry, type MeshResult, type ModelColors, type PartNode, type SizeEstimate } from "../../src/index.ts";

export interface ConvertRequest {
  type: "convert";
  stepText: string;
  opts: ImportOptions;
}

/** Read an STL (binary or ASCII) and index it into the same result shape as a STEP conversion,
 * so the viewer's whole display path (edges, section, measure, part info) applies unchanged. */
export interface LoadStlRequest {
  type: "loadStl";
  buffer: ArrayBuffer;
  /** Display name for the single fabricated body (the file's base name). */
  name: string;
}

/** Fast size pre-pass (parse + point scan, no tessellation) — runs on file load so the UI can
 * pick size-adaptive tessellation defaults before the user hits Convert. */
export interface MeasureRequest {
  type: "measure";
  stepText: string;
}

export interface SizeMessage {
  type: "size";
  estimate: SizeEstimate | null;
}

export interface MeshPayload {
  positions: Float64Array; // 3 per vertex
  indices: Uint32Array; // 3 per triangle
  faceOfTri: Uint32Array; // STEP B-rep face id per triangle
  solidOfTri: Uint32Array; // STEP solid (body) id per triangle — keys of the part tree
}

export interface ConvertResult {
  type: "result";
  /** "stl" = loaded mesh passed through as-is (no tessellation, no CAD faces/colors/structure). */
  kind: "step" | "stl";
  mesh: MeshPayload;
  stats: MeshResult["stats"];
  /** Length-unit label detected in the STEP file (mesh coords are always mm). */
  units: string;
  /** Axis-aligned bounding box of the mesh, in mm: [minX,minY,minZ, maxX,maxY,maxZ]. */
  bbox: [number, number, number, number, number, number] | null;
  /** STEP face/solid colors (palette + per-face indices), or null when the file has none.
   * Maps survive postMessage via structured clone. */
  colors: ModelColors | null;
  /** Part/component tree; bodies[].id keys into mesh.solidOfTri. */
  structure: PartNode;
  /** Solid ids of surface (sheet) bodies — their boundary edges are open BY DESIGN, so the UI
   * must not count or paint them as watertightness defects. */
  openSolids: number[];
  /** Authoritative conversion verdict: openEdges/nonManifoldEdges already exclude openSolids. */
  diagnostics: ImportDiagnostics;
  /** Analytic measurement geometry (exact circle centers/radii + edge polylines coincident with
   * the mesh), instance-placed like the mesh; null for AP242 tessellated bodies with no B-rep. */
  measure: MeasureGeometry | null;
}

export interface ProgressMessage {
  type: "progress";
  stage: string;
  /** Fraction complete in [0,1]; absent when the current stage's duration is unknown
   * (the UI shows the spinner alone / an indeterminate bar). */
  fraction?: number;
}

export interface ErrorMessage {
  type: "error";
  message: string;
}

export type WorkerIn = ConvertRequest | MeasureRequest | LoadStlRequest;
export type WorkerOut = ConvertResult | ProgressMessage | ErrorMessage | SizeMessage;

const ctx = self as unknown as DedicatedWorkerGlobalScope;

function boundingBox(pos: Float64Array): ConvertResult["bbox"] {
  if (pos.length < 3) return null;
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < pos.length; i += 3) {
    const x = pos[i]!, y = pos[i + 1]!, z = pos[i + 2]!;
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  return [minX, minY, minZ, maxX, maxY, maxZ];
}

function post(msg: WorkerOut, transfer?: Transferable[]): void {
  ctx.postMessage(msg, transfer ?? []);
}

/** Label each triangle with its connected component (shell) so a multi-shell STL gets per-shell
 * part rows, defect attribution and part info. Triangles connect only across MANIFOLD edges
 * (exactly 2 incident triangles): welding coincident vertices makes touching solids share whole
 * edges at their contact faces, and those carry 4 triangles — treating them as boundaries splits
 * the shells the same way slicers (admesh / BambuStudio) do. Vertex- or plain edge-connectivity
 * would fuse them (verified on a real two-body export). */
function labelShells(indices: Uint32Array, vertexCount: number, solidOfTri: Uint32Array): number {
  const nT = solidOfTri.length;
  const edgeKey = (t: number, e: number): number => {
    const u = indices[t * 3 + e]!, v = indices[t * 3 + ((e + 1) % 3)]!;
    return (u < v ? u : v) * vertexCount + (u < v ? v : u); // < 2^53 for any real mesh
  };
  // Pass 1: incident-triangle count per undirected edge (capped at 3 — only "exactly 2" matters).
  const edgeCount = new Map<number, number>();
  for (let t = 0; t < nT; t++) {
    for (let e = 0; e < 3; e++) {
      const k = edgeKey(t, e);
      const c = edgeCount.get(k);
      if (c === undefined) edgeCount.set(k, 1);
      else if (c < 3) edgeCount.set(k, c + 1);
    }
  }
  // Pass 2: union the two triangles of every manifold edge.
  const parent = new Uint32Array(nT);
  for (let i = 0; i < nT; i++) parent[i] = i;
  const find = (x: number): number => {
    while (parent[x] !== x) { parent[x] = parent[parent[x]!]!; x = parent[x]!; }
    return x;
  };
  const firstTri = new Map<number, number>();
  for (let t = 0; t < nT; t++) {
    for (let e = 0; e < 3; e++) {
      const k = edgeKey(t, e);
      if (edgeCount.get(k) !== 2) continue;
      const first = firstTri.get(k);
      if (first === undefined) firstTri.set(k, t);
      else parent[find(t)] = find(first);
    }
  }
  const shellOfRoot = new Map<number, number>();
  for (let t = 0; t < nT; t++) {
    const r = find(t);
    let s = shellOfRoot.get(r);
    if (s === undefined) { s = shellOfRoot.size; shellOfRoot.set(r, s); }
    solidOfTri[t] = s;
  }
  return Math.max(1, shellOfRoot.size);
}

/** STL path: parse + weld into an indexed mesh — no tessellation. Each connected shell becomes
 * a body (a single-shell file is named after the file), every triangle keeps "face" 0, so
 * feature-edge extraction degrades to open edges only (an STL has no CAD faces) and the
 * part-info popup / watertight audit work exactly as for a converted STEP. */
function loadStl(req: LoadStlRequest): void {
  post({ type: "progress", stage: "Reading STL…" });
  const soup = readSTL(req.buffer);
  if (soup.triangleCount === 0) throw new Error("No triangles found — not a valid STL file?");
  post({ type: "progress", stage: "Indexing mesh…" });
  const mesh = indexSoup(soup);
  const nT = soup.triangleCount;
  const faceOfTri = new Uint32Array(nT);
  const solidOfTri = new Uint32Array(nT);
  const shells = labelShells(mesh.indices, mesh.positions.length / 3, solidOfTri);
  const bodies = shells === 1
    ? [{ id: 0, name: req.name }]
    : Array.from({ length: shells }, (_, i) => ({ id: i, name: `Shell ${i + 1}` }));
  const { openEdges, nonManifoldEdges } = meshDefects(mesh);
  post(
    {
      type: "result",
      kind: "stl",
      mesh: { positions: mesh.positions, indices: mesh.indices, faceOfTri, solidOfTri },
      stats: { solids: shells, facesTotal: 0, facesTessellated: 0, skipped: {} },
      units: "", // STL carries no units; coordinates are shown as-is (assumed mm)
      bbox: boundingBox(mesh.positions),
      colors: null,
      structure: { name: req.name, occurrences: 1, bodies, children: [] },
      openSolids: [],
      diagnostics: {
        ok: openEdges === 0 && nonManifoldEdges === 0,
        openEdges, nonManifoldEdges, facesDropped: 0, facesSkipped: 0, warnings: [],
      },
      measure: null,
    },
    [mesh.positions.buffer, mesh.indices.buffer, faceOfTri.buffer, solidOfTri.buffer],
  );
}

ctx.onmessage = (ev: MessageEvent<WorkerIn>) => {
  const req = ev.data;
  if (req.type === "measure") {
    post({ type: "size", estimate: estimateStepSize(req.stepText) });
    return;
  }
  if (req.type === "loadStl") {
    try {
      loadStl(req);
    } catch (err) {
      post({ type: "error", message: err instanceof Error ? (err.stack ?? err.message) : String(err) });
    }
    return;
  }
  if (req.type !== "convert") return;
  try {
    post({ type: "progress", stage: "Parsing…" });
    // Tessellation is synchronous in this worker, but postMessage delivers immediately, so the
    // main thread repaints the bar while we compute. Throttled: full-rate ticks (one per edge/
    // face) would flood the main thread's event loop on big assemblies.
    let lastPost = 0;
    const tess = importStep(req.stepText, {
      ...req.opts,
      remesh: false,
      measureGeometry: true,
      onProgress: (p) => {
        if (p.phase === "finalize") { post({ type: "progress", stage: "Finalizing…", fraction: 1 }); return; }
        if (p.phase !== "tessellate" || p.total <= 0) return;
        const now = Date.now();
        if (now - lastPost < 100 && p.done < p.total) return;
        lastPost = now;
        const fraction = p.done / p.total;
        post({ type: "progress", stage: `Tessellating… ${Math.round(fraction * 100)}%`, fraction });
      },
    });

    const pos = tess.mesh.positions;
    const idx = tess.mesh.indices;
    const fot = tess.faceOfTri;
    const sot = tess.solidOfTri;
    const bbox = boundingBox(pos);

    post(
      {
        type: "result",
        kind: "step",
        mesh: { positions: pos, indices: idx, faceOfTri: fot, solidOfTri: sot },
        stats: tess.stats,
        units: tess.units,
        bbox,
        colors: tess.colors,
        structure: tess.structure,
        openSolids: tess.openSolids,
        diagnostics: tess.diagnostics,
        measure: tess.measure ?? null,
      },
      // Transfer the underlying buffers to avoid a copy.
      [pos.buffer, idx.buffer, fot.buffer, sot.buffer,
        ...(tess.measure ? [tess.measure.points.buffer] : [])],
    );
  } catch (err) {
    post({ type: "error", message: err instanceof Error ? (err.stack ?? err.message) : String(err) });
  }
};
