// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — parallel tessellation: the faces of a model meshed on worker threads.
//
// Once the edges are sampled every face is meshed on its own, so the faces can be shared out.
// Each worker (face-worker.ts, face-serve.ts) gets the STEP text and the options and reads the model itself
// (all threads do that at the same time, and no part of the parsed model has to be copied
// between them). The preparation that all faces share (edge sampling, in the isotropic mode the
// size field and the seeding; half the work there) is done once, on the calling thread, and its
// result is sent to the workers ("prep", see TessPrep). The calling thread then hands out work
// and collects it:
//   - "faces": a batch of face numbers (a face's number is its position when the bodies and
//     their faces are walked in model order, the same on every thread). The worker returns the
//     triangles of each face and what else meshing it did (FaceOutcome).
//   - "stitch": the triangles of one whole body, once all its faces are back; the worker welds
//     and repairs them (the per-body tail of the serial tessellation).
// The result is put together in model order, so it is the serial result exactly: same vertices,
// same triangles, same warnings.
//
// A worker starts cold (it has to read the model, and the engine has to compile the meshers
// again in every thread), so a model that takes well under a second gains nothing from them.
// The calling thread therefore begins meshing itself, cheapest faces first. A small model is
// finished that way within `poolTuning.headMs`, at the cost of the serial import and without a
// worker being started. Otherwise the workers are started and take the faces from the costly
// end while the calling thread carries on from the cheap end, until the two meet.
import type { BrepModel, BFace } from "../brep/build.ts";
import type { MeshWarning } from "./diag.ts";
import {
  createTessSession, runSession, vertsLimitError, FACE_UNITS, STITCH_UNITS, SOLID_VERTS_LIMIT,
  type FaceOutcome, type FaceTol, type MeshResult, type SolidMesh, type TessOptions, type TessPrep,
} from "./tessellate.ts";

/** The tessellation options a worker needs (TessOptions without its callbacks), plus the
 * geometry clean-up it has to repeat on its copy of the model. */
export interface PoolConfig {
  chordTol: number;
  targetEdge: number;
  normalDev: number;
  faceTol: [number, FaceTol][] | null;
  isotropic: boolean;
  growthRate?: number;
  minEdge?: number;
  mappedFillets?: boolean;
  /** `ImportOptions.mergeTolerance` (0 = off). */
  mergeTol: number;
  /** Report the mesher of every face (TessOptions.trace is set on the calling thread). */
  trace: boolean;
}

/** The uncommon part of a face's outcome; `i` is the face's position in its batch. */
export interface FaceExtra { i: number; skipped: string | null; warnings: MeshWarning[]; trace: string[] | null; }

export type ToWorker =
  /** Read the model (`src` only the first time). Answered by nothing. */
  | { type: "init"; src?: string; cfg: PoolConfig }
  /** The calling thread's preparation. Answered by "ready". */
  | { type: "prep"; prep: TessPrep }
  | { type: "faces"; job: number; faces: Int32Array }
  | { type: "stitch"; job: number; solid: number; verts: Float64Array; faceIds: Uint32Array };

export type FromWorker =
  | { type: "ready" }
  /** Per face of the batch: `nVerts` values of `verts`, `nIds` of `faceIds`, flags (1 = meshed,
   * 2 = degenerate); `extra` lists the faces with anything else to report. */
  | { type: "faces"; job: number; verts: Float64Array; faceIds: Uint32Array; nVerts: Int32Array; nIds: Int32Array; flags: Uint8Array; extra: FaceExtra[] }
  | { type: "stitch"; job: number; positions: Float64Array; indices: Uint32Array; faceOf: Uint32Array; fillIndices: Uint32Array; fillFaceOf: Uint32Array }
  | { type: "error"; message: string; stack?: string };

/** A worker as this module uses it: a browser `Worker` or a Node `worker_threads.Worker`. */
export interface WorkerLike {
  postMessage(message: unknown, transfer?: any): void; // eslint-disable-line @typescript-eslint/no-explicit-any
  terminate(): unknown;
}

interface Slot {
  w: WorkerLike;
  /** The current run's preparation has been sent. */
  prepared: boolean;
  ready: boolean;
  busy: boolean;
  dead: boolean;
  /** Face numbers of the batch being meshed, or the body being stitched. */
  batch: Int32Array | null;
  solid: number;
}

/** Scheduling thresholds. Exported for the tests, which set `headMs` to 0 to reach the workers
 * on small models; not part of the public API. */
export const poolTuning = {
  /** How long the calling thread meshes alone before the workers are started, ms. */
  headMs: 150,
  /** STEP text at least this long: start the workers before the calling thread's preparation,
   * so they read the model meanwhile (reading it is then a real share of the import). */
  eagerChars: 3_000_000,
};
/** Batches per worker the faces are cut into (more = better balance, more messages). */
const BATCHES_PER_WORKER = 32;
const BATCH_MAX_FACES = 256;

/** The default worker factory under Node: a worker thread on face-worker next to this module.
 * null in other environments (a browser needs `createWorker`). */
async function nodeWorkerFactory(): Promise<(() => WorkerLike) | null> {
  if (typeof process === "undefined" || !process.versions?.node) return null;
  try {
    // Variable specifiers, so a browser bundler does not try to resolve them.
    const wtSpec = "node:worker_threads", v8Spec = "node:v8";
    const wt = await import(/* @vite-ignore */ /* webpackIgnore: true */ wtSpec) as { Worker: new (url: URL, o?: object) => WorkerLike };
    const v8 = await import(/* @vite-ignore */ /* webpackIgnore: true */ v8Spec) as { getHeapStatistics(): { heap_size_limit: number } };
    const url = new URL(import.meta.url.endsWith(".ts") ? "./face-worker.ts" : "./face-worker.js", import.meta.url);
    // A worker's heap is not sized by the command line of the process: give it this thread's.
    const heapMb = Math.round(v8.getHeapStatistics().heap_size_limit / 2 ** 20);
    return () => new wt.Worker(url, { resourceLimits: { maxOldGenerationSizeMb: heapMb } });
  } catch {
    return null;
  }
}

/**
 * Worker threads to use when the caller does not say: half the machine's hardware threads (its
 * physical or performance cores, on the usual machines), at most 6 (more gain little: what one
 * thread must do alone then dominates), and no more than fit a memory budget. Every worker
 * holds the parsed model, measured at about 40 bytes per character of STEP text on top of
 * 120 MB; the budget is half the device memory where the browser tells it (at most 4 GB), and
 * 4 GB elsewhere. A file too large for two workers is meshed on the calling thread.
 */
export function defaultThreads(srcLength: number): number {
  const nav = (globalThis as { navigator?: { hardwareConcurrency?: number; deviceMemory?: number } }).navigator;
  const byCores = Math.min(6, Math.floor((nav?.hardwareConcurrency ?? 1) / 2));
  const budget = Math.min(4, (nav?.deviceMemory ?? 8) / 2) * 2 ** 30;
  const byMemory = Math.floor(budget / (120 * 2 ** 20 + 40 * srcLength));
  return Math.max(1, Math.min(byCores, byMemory));
}

/** Pending MessageChannels while a nextTask() yield is in flight, kept strongly rooted so
 * Firefox's worker GC/Cycle Collector cannot collect an unrooted port pair before onmessage fires. */
const pendingYields = new Set<MessageChannel>();

/** Let pending events (worker messages) be handled. */
function nextTask(): Promise<void> {
  return new Promise<void>((resolve) => {
    if (typeof MessageChannel === "function") {
      const ch = new MessageChannel();
      pendingYields.add(ch);
      ch.port1.onmessage = () => {
        pendingYields.delete(ch);
        ch.port1.close();
        ch.port2.close();
        resolve();
      };
      ch.port2.postMessage(0);
    } else setTimeout(resolve, 0);
  });
}

/** Active pools with live workers, kept strongly rooted in module scope until close().
 * Without a JS root, a suspended importStepAsync() call forms an isolated reference cycle
 * (pool -> slots[].w -> message listener -> pool -> wake -> suspended Promise); in Firefox,
 * the worker's periodic GC/Cycle Collector can sweep that cycle mid-import, silently closing
 * the nested workers and leaving the import stalled forever. */
const activePools = new Set<MeshPool>();

/** Worker threads for one model: started when first needed, usable for several runs. */
export class MeshPool {
  private slots: Slot[] = [];
  private spawned = false;
  private closed = false;
  private onMessage: ((s: Slot, m: FromWorker) => void) | null = null;
  private onDead: ((s: Slot, e: Error) => void) | null = null;
  /** The current run's preparation, sent to every worker after its "init". */
  private prep: (() => TessPrep) | null = null;
  private src: string;
  private cfg: PoolConfig;
  private threads: number;
  private make: () => WorkerLike;

  private constructor(src: string, cfg: PoolConfig, threads: number, make: () => WorkerLike) {
    this.src = src; this.cfg = cfg; this.threads = threads; this.make = make;
  }

  /** A pool of up to `threads` workers on the model. null when workers cannot be created here. */
  static async open(src: string, cfg: PoolConfig, threads: number, createWorker?: () => WorkerLike): Promise<MeshPool | null> {
    const make = createWorker ?? await nodeWorkerFactory();
    if (!make) return null;
    return new MeshPool(src, cfg, threads, make);
  }

  /** Start the workers (once). Each begins reading the model at once. */
  private spawn(): void {
    if (this.spawned || this.closed) return;
    this.spawned = true;
    for (let k = 0; k < this.threads; k++) {
      let w: WorkerLike;
      try { w = this.make(); } catch { break; }
      const s: Slot = { w, prepared: false, ready: false, busy: false, dead: false, batch: null, solid: -1 };
      this.slots.push(s);
      const msg = (m: FromWorker): void => { if (!this.closed) this.receive(s, m); };
      const err = (e: unknown): void => { if (!this.closed) this.died(s, e instanceof Error ? e : new Error(String((e as { message?: string })?.message ?? e))); };
      const anyW = w as unknown as {
        on?: (ev: string, cb: (x: never) => void) => void;
        addEventListener?: (ev: string, cb: (x: never) => void) => void;
      };
      if (typeof anyW.on === "function") {
        anyW.on("message", msg as (x: never) => void);
        anyW.on("error", err as (x: never) => void);
        anyW.on("exit", ((code: number) => { if (code !== 0) err(new Error(`meshStep: worker thread exited with code ${code}`)); }) as (x: never) => void);
      } else if (typeof anyW.addEventListener === "function") {
        anyW.addEventListener("message", ((ev: { data: FromWorker }) => msg(ev.data)) as (x: never) => void);
        anyW.addEventListener("error", err as (x: never) => void);
        anyW.addEventListener("messageerror", (() => err(new Error("meshStep: worker message could not be deserialized"))) as (x: never) => void);
      }
      try { w.postMessage({ type: "init", src: this.src, cfg: this.cfg } satisfies ToWorker); } catch { s.dead = true; }
    }
    if (this.slots.length > 0) activePools.add(this);
    this.sendPrep();
  }

  /** Give the run's preparation to every worker that does not have it yet. */
  private sendPrep(): void {
    if (!this.prep) return;
    let prep: TessPrep | null = null;
    for (const s of this.slots) {
      if (s.dead || s.prepared) continue;
      prep ??= this.prep();
      s.prepared = true;
      try { s.w.postMessage({ type: "prep", prep } satisfies ToWorker); } catch (e) { this.died(s, e instanceof Error ? e : new Error(String(e))); }
    }
  }

  /** Workers that can still take work (all of them, before they are started). */
  private usable(): boolean { return !this.closed && (!this.spawned || this.slots.some((s) => !s.dead)); }
  private alive(): number { return this.slots.reduce((n, s) => n + (s.dead ? 0 : 1), 0); }
  private anyReady(): boolean { return this.slots.some((s) => s.ready && !s.dead); }

  private receive(s: Slot, m: FromWorker): void {
    if (m.type === "ready") s.ready = true;
    // An error before "ready" is a worker that could not read the model: count it as lost (if
    // the model itself is at fault, the calling thread reports that from its own reading of it).
    else if (m.type === "error" && !s.ready) { this.died(s, new Error(m.message)); return; }
    this.onMessage?.(s, m);
  }
  private died(s: Slot, e: Error): void {
    if (s.dead) return;
    s.dead = true; s.ready = false;
    try { s.w.terminate(); } catch { /* already gone */ }
    this.onDead?.(s, e);
  }

  /** Use other options from now on (the unmerged model): running workers read the model again. */
  reinit(cfg: PoolConfig): void {
    this.cfg = cfg;
    for (const s of this.slots) {
      if (s.dead) continue;
      s.ready = false; s.busy = false; s.prepared = false;
      s.w.postMessage({ type: "init", cfg } satisfies ToWorker);
    }
    this.prep = null;
  }

  close(): void {
    if (this.closed) return;
    this.closed = true;
    activePools.delete(this);
    for (const s of this.slots) { try { s.w.terminate(); } catch { /* already gone */ } }
  }

  /**
   * Tessellate `brep` (the model the pool was opened on, as the pool's options make it) with
   * `tess`. Returns what `tessellate(brep, tess)` returns.
   */
  async run(brep: BrepModel, tess: TessOptions, signal?: AbortSignal): Promise<MeshResult> {
    // A long file takes the workers a while to read: have them start on it now, during this
    // thread's preparation, not only once the head start shows they are needed.
    if (this.src.length >= poolTuning.eagerChars) this.spawn();
    // Faces meshed on this thread out of model order must not report to `trace` yet: their
    // mesher names are captured and replayed in model order with everything else.
    let capture: string[] | null = null;
    const userTrace = tess.trace;
    const session = createTessSession(brep, userTrace
      ? { ...tess, trace: (fid, name) => { if (capture) capture.push(name); else userTrace(fid, name); } }
      : tess);
    const solids = brep.solids;
    const faces: BFace[] = [], solidOf: number[] = [], first: number[] = [];
    for (let si = 0; si < solids.length; si++) {
      first.push(faces.length);
      for (const f of solids[si]!.faces) { faces.push(f); solidOf.push(si); }
    }
    const nF = faces.length;
    if (nF === 0) return runSession(session, brep);
    const cost = new Float64Array(nF);
    let costLeft = 0;
    for (let i = 0; i < nF; i++) { cost[i] = session.faceCost(faces[i]!); costLeft += cost[i]!; }
    for (const solid of solids) session.beginSolid(solid);
    // The preparation for the workers (put into plain arrays only when a worker is started),
    // and to those already reading the model.
    {
      let made: TessPrep | null = null;
      this.prep = () => (made ??= session.exportPrep());
      for (const s of this.slots) s.prepared = false;
      if (this.spawned) this.sendPrep();
    }
    // Largest faces first for the workers (a late large face would leave one thread working
    // alone at the end); this thread takes from the cheap end. order[0..tail] is still to do.
    const order = Array.from({ length: nF }, (_, i) => i).sort((a, b) => (cost[b]! - cost[a]!) || (a - b));
    let tail = nF - 1;

    // What is meshed so far, per face and per body.
    const faceV: (Float64Array | null)[] = new Array(nF).fill(null);
    const faceI: (Uint32Array | null)[] = new Array(nF).fill(null);
    const nVerts = new Float64Array(nF);
    const flags = new Uint8Array(nF);
    const extra = new Map<number, FaceExtra>();
    const left = solids.map((s) => s.faces.length);
    const stitchQueue: { solid: number; verts: Float64Array; faceIds: Uint32Array }[] = [];
    const meshes: (SolidMesh | null)[] = new Array(solids.length).fill(null);
    let solidsDone = 0;

    // The warnings so far leave the module-level sink: faces record theirs one by one, and this
    // import may wait for its workers while another one runs on this thread.
    session.parkWarnings();

    const solidComplete = (si: number): void => {
      const f0 = first[si]!, f1 = f0 + solids[si]!.faces.length;
      let tv = 0, ti = 0;
      for (let i = f0; i < f1; i++) {
        if (tv > SOLID_VERTS_LIMIT) throw vertsLimitError(tv); // as the serial loop checks before each face
        tv += faceV[i]!.length; ti += faceI[i]!.length;
      }
      const verts = new Float64Array(tv), faceIds = new Uint32Array(ti);
      tv = 0; ti = 0;
      for (let i = f0; i < f1; i++) {
        verts.set(faceV[i]!, tv); tv += faceV[i]!.length; faceV[i] = null;
        faceIds.set(faceI[i]!, ti); ti += faceI[i]!.length; faceI[i] = null;
      }
      stitchQueue.push({ solid: si, verts, faceIds });
    };
    /** Mesh face `i` on this thread and file it like a worker's result. */
    const meshHere = (i: number): void => {
      const si = solidOf[i]!, solid = solids[si]!;
      session.selectSolid(solid);
      const v: number[] = [], ids: number[] = [];
      const names: string[] | null = userTrace ? [] : null;
      capture = names;
      let o: FaceOutcome;
      try { o = session.recordFace(solid, faces[i]!, v, ids); } finally { capture = null; }
      faceV[i] = new Float64Array(v); faceI[i] = new Uint32Array(ids);
      nVerts[i] = v.length;
      flags[i] = (o.tessellated ? 1 : 0) | (o.degenerate ? 2 : 0);
      if (o.skipped !== null || o.warnings.length > 0 || (names !== null && names.length > 0)) {
        extra.set(i, { i: 0, skipped: o.skipped, warnings: o.warnings, trace: names });
      }
      costLeft -= cost[i]!;
      if (--left[si]! === 0) solidComplete(si);
    };
    const stitchHere = (): void => {
      for (let st = stitchQueue.shift(); st; st = stitchQueue.shift()) {
        meshes[st.solid] = session.stitchSolid(solids[st.solid]!, st.verts, Array.from(st.faceIds));
        solidsDone++;
      }
    };
    const assemble = (): MeshResult => {
      session.unparkWarnings();
      for (let si = 0; si < solids.length; si++) {
        const solid = solids[si]!;
        let before = 0;
        for (let k = 0, i = first[si]!; k < solid.faces.length; k++, i++) {
          const x = extra.get(i);
          const o: FaceOutcome = {
            tessellated: (flags[i]! & 1) !== 0, degenerate: (flags[i]! & 2) !== 0,
            skipped: x ? x.skipped : null, warnings: x ? x.warnings : [], trace: x ? x.trace : null,
          };
          session.replayFace(solid.faces[k]!, o, before);
          before += nVerts[i]!;
        }
        session.appendSolid(solid, meshes[si]!);
        meshes[si] = null;
      }
      return session.finish();
    };
    /** The whole model again on this thread, from a fresh session (after waiting, this thread's
     * shared mesher state may be another import's). */
    const startOver = (): MeshResult => runSession(createTessSession(brep, tess), brep);

    for (let si = 0; si < solids.length; si++) if (left[si] === 0) solidComplete(si); // bodies without faces

    // 1. Head start on this thread, cheapest faces first. A small model ends here, and no
    //    worker is ever started for it.
    const t0 = performance.now();
    while (tail >= 0 && !(this.usable() && performance.now() - t0 >= poolTuning.headMs)) meshHere(order[tail--]!);
    if (tail < 0) { stitchHere(); return assemble(); }

    // 2. Start the workers. They take batches from the large end of the order as soon as they
    //    are ready, while this thread carries on from the cheap end.
    this.spawn();
    let head = 0; // order[head..tail] is not yet given out
    let jobSeq = 0;
    let failed = false, failure: unknown;
    let wake: (() => void) | null = null;
    const notify = (): void => { const w = wake; wake = null; w?.(); };
    const fail = (e: unknown): void => { if (!failed) { failed = true; failure = e; } notify(); };
    const onAbort = (): void => fail(signal!.reason);
    const dispatch = (s: Slot): void => {
      if (s.dead || !s.ready || s.busy) return;
      const st = stitchQueue.shift();
      if (st) {
        s.busy = true; s.batch = null; s.solid = st.solid;
        s.w.postMessage({ type: "stitch", job: jobSeq++, solid: st.solid, verts: st.verts, faceIds: st.faceIds } satisfies ToWorker,
          [st.verts.buffer, st.faceIds.buffer]);
        return;
      }
      if (head > tail) return;
      // A batch of about equal cost: one large face, or many small ones.
      const target = costLeft / (Math.max(1, this.alive()) * BATCHES_PER_WORKER);
      const ids: number[] = [];
      for (let acc = 0; head <= tail && ids.length < BATCH_MAX_FACES && (ids.length === 0 || acc < target);) {
        const i = order[head++]!;
        ids.push(i); acc += cost[i]!;
      }
      for (const i of ids) costLeft -= cost[i]!;
      s.busy = true; s.batch = Int32Array.from(ids); s.solid = -1;
      s.w.postMessage({ type: "faces", job: jobSeq++, faces: s.batch } satisfies ToWorker);
    };
    this.onDead = (s, e) => {
      // A worker lost with work in hand cannot be made up for; one lost idle is only capacity.
      if (s.busy) fail(e); else notify();
    };
    this.onMessage = (s, m) => {
      try {
        if (m.type === "error") {
          const e = new Error(m.message);
          if (m.stack) e.stack = m.stack;
          fail(e);
          return;
        }
        if (m.type === "faces") {
          const ids = s.batch!;
          let vo = 0, io = 0;
          for (let k = 0; k < ids.length; k++) {
            const i = ids[k]!;
            faceV[i] = m.verts.subarray(vo, vo + m.nVerts[k]!); vo += m.nVerts[k]!;
            faceI[i] = m.faceIds.subarray(io, io + m.nIds[k]!); io += m.nIds[k]!;
            nVerts[i] = m.nVerts[k]!;
            flags[i] = m.flags[k]!;
          }
          for (const x of m.extra) extra.set(ids[x.i]!, x);
          s.busy = false;
          for (let k = 0; k < ids.length; k++) {
            session.tick(FACE_UNITS);
            const si = solidOf[ids[k]!]!;
            if (--left[si]! === 0) solidComplete(si);
          }
        } else if (m.type === "stitch") {
          meshes[s.solid] = { positions: m.positions, indices: m.indices, faceOf: m.faceOf, fillIndices: m.fillIndices, fillFaceOf: m.fillFaceOf };
          s.busy = false;
          solidsDone++;
          session.tick((STITCH_UNITS - 1) * solids[s.solid]!.faces.length);
        }
        if (!failed) for (const x of this.slots) dispatch(x);
        notify();
      } catch (e) {
        fail(e);
      }
    };
    try {
      if (signal) {
        signal.throwIfAborted();
        signal.addEventListener("abort", onAbort, { once: true });
      }
      let lastYield = performance.now();
      while (solidsDone < solids.length) {
        if (failed) throw failure;
        for (const s of this.slots) dispatch(s);
        const mine = session.isCurrent(); // else this thread's mesher state is another import's
        // While this thread meshes a face it hands out nothing, so with workers waiting for
        // batches it only takes faces that are small against a batch.
        const small = head <= tail && (!this.anyReady()
          || cost[order[tail]!]! * this.alive() * BATCHES_PER_WORKER * 2 <= costLeft);
        if (small && mine) {
          meshHere(order[tail--]!);
          if (performance.now() - lastYield < 4) continue;
        } else if (!this.usable()) {
          // No worker (left): everything that remains is this thread's.
          if (!mine) return startOver();
          stitchHere();
          continue;
        } else if (head > tail && mine && !this.anyReady()) {
          // All faces are meshed before a worker is up: do not wait for one to stitch.
          stitchHere();
          continue;
        } else if (!this.slots.some((s) => s.busy) && this.anyReady() && stitchQueue.length === 0 && head > tail) {
          throw new Error("meshStep: parallel tessellation stalled"); // cannot happen: nothing left to wait for
        } else {
          await new Promise<void>((resolve) => { wake = resolve; });
          continue;
        }
        await nextTask(); // let the workers' messages in
        lastYield = performance.now();
      }
      if (failed) throw failure;
      return assemble();
    } catch (e) {
      this.close();
      throw e;
    } finally {
      this.onMessage = null; this.onDead = null;
      signal?.removeEventListener("abort", onAbort);
    }
  }
}
