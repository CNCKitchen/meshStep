// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — worker script of the parallel import (`meshstep/worker`): answers the pool's
// messages (face-serve.ts). Runs as a Node worker thread and as a browser module worker.
import { serveWorkerMessage } from "./face-serve.ts";
import type { ToWorker } from "./pool.ts";

interface NodePort { on(ev: "message", cb: (m: ToWorker) => void): void; postMessage(m: unknown, transfer?: ArrayBuffer[]): void; }
interface WebScope { onmessage: ((ev: { data: ToWorker }) => void) | null; postMessage(m: unknown, transfer?: ArrayBuffer[]): void; }

if (typeof process !== "undefined" && !!process.versions?.node) {
  // The specifier is a variable so that a browser bundler does not try to resolve it.
  const spec = "node:worker_threads";
  void import(/* @vite-ignore */ /* webpackIgnore: true */ spec).then((wt: { parentPort: NodePort | null }) => {
    const port = wt.parentPort;
    if (!port) return; // imported outside a worker: nothing to serve
    port.on("message", (m) => serveWorkerMessage(m, (r, t) => port.postMessage(r, t)));
  });
} else {
  const scope = globalThis as unknown as WebScope;
  scope.onmessage = (ev) => serveWorkerMessage(ev.data, (r, t) => scope.postMessage(r, t ?? []));
}
