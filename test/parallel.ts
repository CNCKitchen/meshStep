// SPDX-License-Identifier: AGPL-3.0-only
// importStepAsync must return exactly what importStep returns — same vertices, triangles, face
// and solid ids, statistics, warnings and trace — whatever the options, and must fall back to
// the calling thread when workers cannot be started.
import { readFileSync } from "node:fs";
import { Worker } from "node:worker_threads";

import { importStep, importStepAsync, type ImportOptions, type ImportResult, type WorkerLike } from "../src/index.ts";
import { poolTuning } from "../src/mesh/pool.ts";

// The committed fixtures are small enough to be finished by the calling thread's head start:
// without one, the workers get (nearly) every face.
const headMs = poolTuning.headMs;
poolTuning.headMs = 0;

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}`);
}

const same = (a: ArrayLike<number> | undefined, b: ArrayLike<number> | undefined): boolean => {
  if (!a || !b) return a === b;
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (!Object.is(a[i], b[i])) return false;
  return true;
};
const json = (x: unknown): string => JSON.stringify(x, (_k, v) => (v instanceof Map ? [...v] : ArrayBuffer.isView(v) ? Array.from(v as unknown as ArrayLike<number>) : v));

/** Empty string when the two results agree, otherwise the first field that differs. */
function diff(a: ImportResult, b: ImportResult): string {
  if (!same(a.mesh.positions, b.mesh.positions)) return "positions";
  if (!same(a.mesh.indices, b.mesh.indices)) return "indices";
  if (!same(a.faceOfTri, b.faceOfTri)) return "faceOfTri";
  if (!same(a.solidOfTri, b.solidOfTri)) return "solidOfTri";
  if (!same(a.instanceOfTri, b.instanceOfTri)) return "instanceOfTri";
  if (!same(a.normals, b.normals)) return "normals";
  if (!same(a.uv, b.uv)) return "uv";
  for (const k of ["stats", "diagnostics", "openSolids", "merged", "units", "instances", "faces", "faceUV", "measure", "structure", "colors"] as const) {
    if (json(a[k]) !== json(b[k])) return k;
  }
  return "";
}

const fixtures = ["everything", "cube", "cylinder", "sphere"].map((n) => ({ n, src: readFileSync(new URL(`../${n}.step`, import.meta.url), "utf8") }));
const everything = fixtures[0]!.src;

// --- same result as the serial import, per option set
const optionSets: [string, ImportOptions][] = [
  ["default", {}],
  ["coarse", { surfaceDeviation: 0.1, normalDeviation: 30, maxEdge: 5 }],
  ["fine", { surfaceDeviation: 0.002, maxEdge: 0.5 }],
  ["isotropic", { isotropic: true }],
  ["isotropic, free fillets", { isotropic: true, mappedFillets: false, growthRate: 1.3 }],
  ["merge", { mergeTolerance: 0.05 }],
  ["merge + isotropic", { mergeTolerance: 0.2, isotropic: true }],
  ["attributes", { vertexNormals: true, parameterUVs: true, measureGeometry: true }],
  ["remesh", { remesh: true }],
];
for (const { n, src } of fixtures) {
  for (const [label, opts] of optionSets) {
    if (n !== "everything" && label !== "default" && label !== "isotropic") continue;
    const traceA: string[] = [], traceB: string[] = [];
    const a = importStep(src, { ...opts, trace: (f, m) => traceA.push(`${f}:${m}`) });
    const b = await importStepAsync(src, { ...opts, threads: 3, trace: (f, m) => traceB.push(`${f}:${m}`) });
    const d = diff(a, b);
    check(`${n} / ${label}: same result`, d === "", `differs in ${d}`);
    check(`${n} / ${label}: same trace`, traceA.join() === traceB.join());
  }
}

// --- per-face overrides reach the workers
{
  const base = importStep(everything);
  const fid = base.faceOfTri[Math.floor(base.faceOfTri.length / 2)]!;
  const opts: ImportOptions = { faceOverrides: { [fid]: { surfaceDeviation: 0.001, maxEdge: 0.2 } } };
  const a = importStep(everything, opts);
  const b = await importStepAsync(everything, { ...opts, threads: 2 });
  check("face overrides: same result", diff(a, b) === "", diff(a, b));
  check("face overrides: took effect", a.mesh.indices.length > base.mesh.indices.length);
}

// --- progress is monotone and reaches the total; two imports at once do not disturb each other
{
  let last = -1, total = 0, monotone = true, phases = "";
  const p1 = importStepAsync(everything, {
    threads: 2,
    onProgress: (p) => {
      if (!phases.endsWith(p.phase)) phases += (phases ? "," : "") + p.phase;
      if (p.phase !== "tessellate") return;
      if (p.done < last) monotone = false;
      last = p.done; total = p.total;
    },
  });
  const p2 = importStepAsync(everything, { threads: 2, isotropic: true });
  const [r1, r2] = await Promise.all([p1, p2]);
  check("progress: monotone", monotone);
  check("progress: reaches the total", last === total && total > 0, `${last} of ${total}`);
  check("progress: phases in order", phases === "parse,tessellate,finalize", phases);
  check("concurrent imports: first is the serial result", diff(r1, importStep(everything)) === "");
  check("concurrent imports: second is the serial result", diff(r2, importStep(everything, { isotropic: true })) === "");
}

// --- abort
{
  const ctl = new AbortController();
  let ticks = 0;
  let err: unknown = null;
  try {
    await importStepAsync(everything, { threads: 2, signal: ctl.signal, onProgress: () => { if (++ticks === 25) ctl.abort(); } });
  } catch (e) { err = e; }
  check("abort: rejects with AbortError", err instanceof DOMException && err.name === "AbortError", String(err));
  let threw = false;
  try { await importStepAsync(everything, { threads: 2, signal: AbortSignal.abort() }); } catch { threw = true; }
  check("abort: pre-aborted rejects", threw);
}

// --- a file that cannot be read fails the same way
{
  const msg = async (f: () => unknown): Promise<string> => { try { await f(); return "(no error)"; } catch (e) { return String((e as Error).message); } };
  const bad = "this is not a STEP file";
  const a = await msg(() => importStep(bad));
  const b = await msg(() => importStepAsync(bad, { threads: 2 }));
  check("unreadable file: same outcome", a === b, `${a} | ${b}`);
}

// --- fallbacks to the calling thread
{
  const serial = importStep(everything);
  const one = await importStepAsync(everything, { threads: 1 });
  check("threads: 1 runs on this thread", diff(serial, one) === "");
  const noFactory = await importStepAsync(everything, { threads: 3, createWorker: () => { throw new Error("no workers here"); } });
  check("createWorker throws: falls back", diff(serial, noFactory) === "");
  const broken = await importStepAsync(everything, {
    threads: 3,
    createWorker: () => new Worker(new URL("data:text/javascript,throw new Error('worker module failed to load')")),
  });
  check("workers fail to start: falls back", diff(serial, broken) === "");
  let created = 0;
  const own = await importStepAsync(everything, {
    threads: 2,
    createWorker: () => { created++; return new Worker(new URL("../src/mesh/face-worker.ts", import.meta.url)); },
  });
  check("createWorker: used", created === 2 && diff(serial, own) === "", `created ${created}`);

  // Browser-style addEventListener worker emitting messageerror before ready falls back cleanly.
  const makeBrowserMockWorker = (failBusy: boolean): WorkerLike & {
    addEventListener: (ev: string, cb: (x: unknown) => void) => void;
  } => {
    const listeners = new Map<string, ((x: unknown) => void)[]>();
    const emit = (ev: string, data: unknown): void => {
      for (const cb of listeners.get(ev) ?? []) cb(data);
    };
    return {
      addEventListener(ev: string, cb: (x: unknown) => void): void {
        let list = listeners.get(ev);
        if (!list) { list = []; listeners.set(ev, list); }
        list.push(cb);
      },
      postMessage(msg: unknown): void {
        const type = (msg as { type?: string }).type;
        if (type === "prep") {
          queueMicrotask(() => {
            if (failBusy) emit("message", { data: { type: "ready" } });
            else emit("messageerror", {});
          });
        } else if (type === "faces" && failBusy) {
          queueMicrotask(() => emit("messageerror", {}));
        }
      },
      terminate(): void {},
    };
  };

  const msgErrBeforeReady = await importStepAsync(everything, {
    threads: 2,
    createWorker: () => makeBrowserMockWorker(false),
  });
  check("messageerror before ready: falls back", diff(serial, msgErrBeforeReady) === "");

  let busyErr: unknown = null;
  try {
    await importStepAsync(everything, {
      threads: 2,
      createWorker: () => makeBrowserMockWorker(true),
    });
  } catch (e) { busyErr = e; }
  check(
    "messageerror while busy: rejects promptly",
    busyErr instanceof Error && busyErr.message.includes("could not be deserialized"),
    String(busyErr),
  );
}

// --- with the head start, the calling thread finishes a small model itself. The head start is
// stretched well beyond the default so that a slow or busy machine cannot turn this into a race.
{
  poolTuning.headMs = Math.max(headMs, 5000);
  let created = 0;
  for (const { n, src } of fixtures) {
    if (n === "everything") continue; // takes long enough for workers to be worth starting
    for (const opts of [{}, { isotropic: true }] as ImportOptions[]) {
      const traceA: string[] = [], traceB: string[] = [];
      const a = importStep(src, { ...opts, trace: (f, m) => traceA.push(`${f}:${m}`) });
      const b = await importStepAsync(src, {
        ...opts, threads: 3, trace: (f, m) => traceB.push(`${f}:${m}`),
        createWorker: () => { created++; return new Worker(new URL("../src/mesh/face-worker.ts", import.meta.url)); },
      });
      check(`head start, ${n}${opts.isotropic ? " / isotropic" : ""}: same result`, diff(a, b) === "" && traceA.join() === traceB.join(), diff(a, b));
    }
  }
  check("head start: no worker started for small models", created === 0, `created ${created}`);
}

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("parallel: all passed");
process.exit(0);
