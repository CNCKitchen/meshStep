/// <reference lib="webworker" />
// Runs the meshStep pipeline off the main thread so the UI stays responsive.
// Produces TWO meshes from one STEP file: the raw tessellation (remesh: false)
// and the isotropically remeshed result (remesh: true), so the UI can show them
// side by side.
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
  tessellated: MeshPayload;
  remeshed: MeshPayload;
  stats: {
    tessellated: MeshResult["stats"];
    remeshed: MeshResult["stats"];
  };
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

    let remeshed: MeshResult = tess;
    if (req.opts.remesh !== false) {
      post({ type: "progress", stage: "Remeshing (isotropic)…" });
      remeshed = importStep(req.stepText, { ...req.opts, remesh: true });
    }

    const tPos = tess.mesh.positions;
    const tIdx = tess.mesh.indices;
    const rPos = remeshed.mesh.positions;
    const rIdx = remeshed.mesh.indices;

    post(
      {
        type: "result",
        tessellated: { positions: tPos, indices: tIdx },
        remeshed: { positions: rPos, indices: rIdx },
        stats: { tessellated: tess.stats, remeshed: remeshed.stats },
      },
      // Transfer the underlying buffers to avoid a copy.
      [tPos.buffer, tIdx.buffer, rPos.buffer, rIdx.buffer],
    );
  } catch (err) {
    post({ type: "error", message: err instanceof Error ? (err.stack ?? err.message) : String(err) });
  }
};
