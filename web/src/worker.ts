// SPDX-License-Identifier: AGPL-3.0-only
/// <reference lib="webworker" />
// Runs the meshStep pipeline off the main thread so the UI stays responsive.
// Produces the raw tessellation (remesh: false) from one STEP file.
import { importStep, estimateStepSize, readSTL, read3MF, indexSoup, meshDefects, type ImportOptions, type ImportDiagnostics, type MeasureGeometry, type MeshResult, type ModelColors, type PartNode, type SizeEstimate } from "../../src/index.ts";

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

/** Read a 3MF (ZIP + model XML) into the same result shape: per-build-item bodies, base-material /
 * color-group colors, transforms and units applied by the library reader. */
export interface Load3mfRequest {
  type: "load3mf";
  buffer: ArrayBuffer;
  /** Display name for the structure root (the file's base name). */
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
  /** "stl" / "3mf" = loaded mesh passed through as-is (no tessellation, no CAD faces). */
  kind: "step" | "stl" | "3mf";
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

export type WorkerIn = ConvertRequest | MeasureRequest | LoadStlRequest | Load3mfRequest;
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

/** Weld coincident vertices (quantised to eps) of one indexed mesh. 3MF meshes are indexed by
 * the spec, but some exporters still duplicate vertices per face — which would fake open edges
 * in the watertight audit exactly like an unwelded STL. No-op (same arrays) when already welded. */
function weldIndexed(positions: Float64Array, indices: Uint32Array, eps = 1e-6): { positions: Float64Array; indices: Uint32Array } {
  const nV = positions.length / 3;
  const map = new Map<string, number>();
  const remap = new Uint32Array(nV);
  const out = new Float64Array(positions.length);
  let kept = 0;
  for (let v = 0; v < nV; v++) {
    const x = positions[v * 3]!, y = positions[v * 3 + 1]!, z = positions[v * 3 + 2]!;
    const key = `${Math.round(x / eps)},${Math.round(y / eps)},${Math.round(z / eps)}`;
    let idx = map.get(key);
    if (idx === undefined) {
      idx = kept++;
      map.set(key, idx);
      out[idx * 3] = x; out[idx * 3 + 1] = y; out[idx * 3 + 2] = z;
    }
    remap[v] = idx;
  }
  if (kept === nV) return { positions, indices };
  const newIndices = new Uint32Array(indices.length);
  for (let i = 0; i < indices.length; i++) newIndices[i] = remap[indices[i]!]!;
  return { positions: out.slice(0, kept * 3), indices: newIndices };
}

/** 3MF path: unzip + parse (library) — no tessellation. Each build item becomes a body; colors
 * from base materials / color groups ride the same ModelColors path as STEP face colors by
 * keying faceOfTri with palette-index+1 (0 = unstyled). Feature edges degrade to crease edges
 * like an STL (a 3MF mesh has no CAD faces). */
async function load3mf(req: Load3mfRequest): Promise<void> {
  post({ type: "progress", stage: "Reading 3MF…" });
  const model = await read3MF(req.buffer);
  if (model.items.length === 0) throw new Error("No mesh objects found — not a valid 3MF file?");
  post({ type: "progress", stage: "Indexing mesh…" });
  const welded = model.items.map((it) => weldIndexed(it.positions, it.indices));
  let nV = 0, nT = 0;
  for (const w of welded) { nV += w.positions.length / 3; nT += w.indices.length / 3; }
  const positions = new Float64Array(nV * 3);
  const indices = new Uint32Array(nT * 3);
  const faceOfTri = new Uint32Array(nT);
  const solidOfTri = new Uint32Array(nT);
  let anyColor = false;
  let vo = 0, to = 0;
  model.items.forEach((it, s) => {
    const w = welded[s]!;
    positions.set(w.positions, vo * 3);
    const pt = w.indices.length / 3;
    for (let t = 0; t < pt; t++) {
      indices[(to + t) * 3] = w.indices[t * 3]! + vo;
      indices[(to + t) * 3 + 1] = w.indices[t * 3 + 1]! + vo;
      indices[(to + t) * 3 + 2] = w.indices[t * 3 + 2]! + vo;
      const c = it.colorOfTri ? it.colorOfTri[t]! : -1;
      faceOfTri[to + t] = c + 1;
      if (c >= 0) anyColor = true;
      solidOfTri[to + t] = s;
    }
    vo += w.positions.length / 3;
    to += pt;
  });
  const mesh = { positions, indices };
  const colors: ModelColors | null = anyColor
    ? {
        palette: model.palette,
        faceColor: new Map(model.palette.map((_, k) => [k + 1, k])),
        solidColor: new Map(),
      }
    : null;
  // "surface" / "support" objects are open BY DESIGN — exclude them from the watertight audit.
  const openSolids = model.items.flatMap((it, s) => (it.type === "surface" || it.type === "support" ? [s] : []));
  const { openEdges, nonManifoldEdges } = meshDefects(mesh, solidOfTri, openSolids);
  // Slicer exports usually carry no object names — a single body borrows the file's name (like
  // the STL path); multiple anonymous bodies get numbered.
  const bodies = model.items.map((it, s) => ({
    id: s,
    name: it.name ?? (model.items.length === 1 ? req.name : `Object ${s + 1}`),
  }));
  post(
    {
      type: "result",
      kind: "3mf",
      mesh: { positions, indices, faceOfTri, solidOfTri },
      stats: { solids: model.items.length, facesTotal: 0, facesTessellated: 0, skipped: {} },
      units: model.unit,
      bbox: boundingBox(positions),
      colors,
      structure: { name: req.name, occurrences: 1, bodies, children: [] },
      openSolids,
      diagnostics: {
        ok: openEdges === 0 && nonManifoldEdges === 0,
        openEdges, nonManifoldEdges, facesDropped: 0, facesSkipped: 0, warnings: [],
      },
      measure: null,
    },
    [positions.buffer, indices.buffer, faceOfTri.buffer, solidOfTri.buffer],
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
  if (req.type === "load3mf") {
    load3mf(req).catch((err: unknown) => {
      post({ type: "error", message: err instanceof Error ? (err.stack ?? err.message) : String(err) });
    });
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
