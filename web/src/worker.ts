// SPDX-License-Identifier: AGPL-3.0-only
/// <reference lib="webworker" />
// Runs the meshStep pipeline off the main thread so the UI stays responsive.
// Produces the raw tessellation (remesh: false) from one STEP file.
import { importStep, estimateStepSize, type ImportOptions, type ImportDiagnostics, type MeshResult, type ModelColors, type PartNode, type SizeEstimate } from "../../src/index.ts";

export interface ConvertRequest {
  type: "convert";
  stepText: string;
  opts: ImportOptions;
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

export type WorkerIn = ConvertRequest | MeasureRequest;
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

ctx.onmessage = (ev: MessageEvent<WorkerIn>) => {
  const req = ev.data;
  if (req.type === "measure") {
    post({ type: "size", estimate: estimateStepSize(req.stepText) });
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
        mesh: { positions: pos, indices: idx, faceOfTri: fot, solidOfTri: sot },
        stats: tess.stats,
        units: tess.units,
        bbox,
        colors: tess.colors,
        structure: tess.structure,
        openSolids: tess.openSolids,
        diagnostics: tess.diagnostics,
      },
      // Transfer the underlying buffers to avoid a copy.
      [pos.buffer, idx.buffer, fot.buffer, sot.buffer],
    );
  } catch (err) {
    post({ type: "error", message: err instanceof Error ? (err.stack ?? err.message) : String(err) });
  }
};
