// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — what a worker thread of the parallel import does (pool.ts; the protocol is
// described there).
//
// The worker gets the STEP text and the options and reads the model as the calling thread
// does; it is then given that thread's preparation (the sampled edges and what goes with
// them), and meshes the faces and stitches the bodies it is asked for.
//
// This module only answers messages (`serveWorkerMessage`); face-worker.ts is the ready-made
// worker script around it. A worker script that already has a message handler of its own (the
// web viewer's) can pass the pool's messages on to it instead: `isWorkerMessage` tells them
// apart.
import { buildBrep, type BrepModel, type BFace, type BSolid } from "../brep/build.ts";
import { mergeShortEdges } from "../brep/defeature.ts";
import { createTessSession, type TessPrep, type TessSession } from "./tessellate.ts";
import type { FaceExtra, FromWorker, PoolConfig, ToWorker } from "./pool.ts";

let src = "";
let cfg: PoolConfig;
let brep: BrepModel;
let session: TessSession;
let faceSolid: BSolid[] = [];
let faces: BFace[] = [];
let wantTrace = false;
let traceLog: string[] | null = null;

/** A batch's triangles are collected in plain arrays; start a new one past this many values so
 * none comes near the engine's array ceiling. */
const CHUNK_VALUES = 4_000_000;

/** Read the model, as the calling thread does. */
function init(c: PoolConfig): void {
  cfg = c;
  brep = buildBrep(src);
  if (cfg.mergeTol > 0) mergeShortEdges(brep, cfg.mergeTol, { chordTol: cfg.chordTol, targetEdge: cfg.targetEdge, normalDev: cfg.normalDev });
  faceSolid = []; faces = [];
  for (const solid of brep.solids) for (const face of solid.faces) { faceSolid.push(solid); faces.push(face); }
}

/** Open the session on the calling thread's preparation. */
function prepare(prep: TessPrep): void {
  wantTrace = cfg.trace;
  session = createTessSession(brep, {
    chordTol: cfg.chordTol, targetEdge: cfg.targetEdge, normalDev: cfg.normalDev,
    trace: cfg.trace ? (_fid, name) => { traceLog?.push(name); } : undefined,
    faceTol: cfg.faceTol ? new Map(cfg.faceTol) : undefined,
    isotropic: cfg.isotropic, growthRate: cfg.growthRate, minEdge: cfg.minEdge, mappedFillets: cfg.mappedFillets,
  }, prep);
}

function meshFaces(job: number, ids: Int32Array, post: WorkerPost): void {
  const n = ids.length;
  const nVerts = new Int32Array(n), nIds = new Int32Array(n), flags = new Uint8Array(n);
  const extra: FaceExtra[] = [];
  const doneV: number[][] = [], doneI: number[][] = [];
  let verts: number[] = [], faceIds: number[] = [];
  let cur: BSolid | null = null;
  for (let k = 0; k < n; k++) {
    const solid = faceSolid[ids[k]!]!, face = faces[ids[k]!]!;
    if (solid !== cur) { session.selectSolid(solid); cur = solid; }
    if (verts.length > CHUNK_VALUES) { doneV.push(verts); doneI.push(faceIds); verts = []; faceIds = []; }
    const v0 = verts.length, i0 = faceIds.length;
    traceLog = wantTrace ? [] : null;
    const o = session.recordFace(solid, face, verts, faceIds);
    nVerts[k] = verts.length - v0; nIds[k] = faceIds.length - i0;
    flags[k] = (o.tessellated ? 1 : 0) | (o.degenerate ? 2 : 0);
    if (o.skipped !== null || o.warnings.length > 0 || (traceLog !== null && traceLog.length > 0)) {
      extra.push({ i: k, skipped: o.skipped, warnings: o.warnings, trace: traceLog });
    }
  }
  doneV.push(verts); doneI.push(faceIds);
  let tv = 0, ti = 0;
  for (const a of doneV) tv += a.length;
  for (const a of doneI) ti += a.length;
  const V = new Float64Array(tv), I = new Uint32Array(ti);
  tv = 0; ti = 0;
  for (const a of doneV) { V.set(a, tv); tv += a.length; }
  for (const a of doneI) { I.set(a, ti); ti += a.length; }
  post({ type: "faces", job, verts: V, faceIds: I, nVerts, nIds, flags, extra },
    [V.buffer, I.buffer, nVerts.buffer, nIds.buffer, flags.buffer]);
}

function stitch(job: number, solidIndex: number, verts: Float64Array, faceIds: Uint32Array, post: WorkerPost): void {
  const m = session.stitchSolid(brep.solids[solidIndex]!, verts, Array.from(faceIds));
  const u32 = (a: ArrayLike<number>): Uint32Array => (a instanceof Uint32Array ? a : Uint32Array.from(a));
  const indices = u32(m.indices), faceOf = u32(m.faceOf), fillIndices = u32(m.fillIndices), fillFaceOf = u32(m.fillFaceOf);
  const transfer = new Set<ArrayBufferLike>([m.positions.buffer, indices.buffer, faceOf.buffer, fillIndices.buffer, fillFaceOf.buffer]);
  post({ type: "stitch", job, positions: m.positions, indices, faceOf, fillIndices, fillFaceOf }, [...transfer] as ArrayBuffer[]);
}

/** Sends an answer to the thread that runs the pool. */
export type WorkerPost = (m: FromWorker, transfer?: ArrayBuffer[]) => void;

/** True for a message the pool sends to its workers. */
export function isWorkerMessage(m: unknown): m is ToWorker {
  const t = (m as { type?: unknown } | null)?.type;
  return t === "init" || t === "prep" || t === "faces" || t === "stitch";
}

/** Do what one message of the pool asks, and answer through `post`. */
export function serveWorkerMessage(m: ToWorker, post: WorkerPost): void {
  try {
    if (m.type === "init") {
      if (m.src !== undefined) src = m.src;
      init(m.cfg);
    } else if (m.type === "prep") {
      prepare(m.prep);
      post({ type: "ready" });
    } else if (m.type === "faces") meshFaces(m.job, m.faces, post);
    else if (m.type === "stitch") stitch(m.job, m.solid, m.verts, m.faceIds, post);
  } catch (e) {
    post({ type: "error", message: e instanceof Error ? e.message : String(e), stack: e instanceof Error ? e.stack : undefined });
  }
}
