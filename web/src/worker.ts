// SPDX-License-Identifier: AGPL-3.0-only
/// <reference lib="webworker" />
// Runs the meshStep pipeline off the main thread so the UI stays responsive.
// Produces the raw tessellation (remesh: false) from one STEP file.
import { importStep, type ImportOptions, type MeshResult } from "../../src/index.ts";

export interface ConvertRequest {
  type: "convert";
  stepText: string;
  opts: ImportOptions;
}

export interface MeshPayload {
  positions: Float64Array; // 3 per vertex
  indices: Uint32Array; // 3 per triangle
  faceOfTri: Uint32Array; // STEP B-rep face id per triangle
}

export interface ConvertResult {
  type: "result";
  mesh: MeshPayload;
  stats: MeshResult["stats"];
  /** Length-unit label detected in the STEP file (mesh coords are always mm). */
  units: string;
  /** Axis-aligned bounding box of the mesh, in mm: [minX,minY,minZ, maxX,maxY,maxZ]. */
  bbox: [number, number, number, number, number, number] | null;
}

export interface ProgressMessage {
  type: "progress";
  stage: string;
}

export interface ErrorMessage {
  type: "error";
  message: string;
}

export type WorkerOut = ConvertResult | ProgressMessage | ErrorMessage;

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

ctx.onmessage = (ev: MessageEvent<ConvertRequest>) => {
  const req = ev.data;
  if (req.type !== "convert") return;
  try {
    post({ type: "progress", stage: "Tessellating…" });
    const tess = importStep(req.stepText, { ...req.opts, remesh: false });

    const pos = tess.mesh.positions;
    const idx = tess.mesh.indices;
    const fot = tess.faceOfTri;
    const bbox = boundingBox(pos);

    post(
      {
        type: "result",
        mesh: { positions: pos, indices: idx, faceOfTri: fot },
        stats: tess.stats,
        units: tess.units,
        bbox,
      },
      // Transfer the underlying buffers to avoid a copy.
      [pos.buffer, idx.buffer, fot.buffer],
    );
  } catch (err) {
    post({ type: "error", message: err instanceof Error ? (err.stack ?? err.message) : String(err) });
  }
};
