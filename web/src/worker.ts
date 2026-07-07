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
}

export interface ConvertResult {
  type: "result";
  mesh: MeshPayload;
  stats: MeshResult["stats"];
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

    post(
      {
        type: "result",
        mesh: { positions: pos, indices: idx },
        stats: tess.stats,
      },
      // Transfer the underlying buffers to avoid a copy.
      [pos.buffer, idx.buffer],
    );
  } catch (err) {
    post({ type: "error", message: err instanceof Error ? (err.stack ?? err.message) : String(err) });
  }
};
