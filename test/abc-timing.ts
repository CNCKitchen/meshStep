// SPDX-License-Identifier: AGPL-3.0-only
// OCC-vs-meshStep timing + correctness sweep over a spread sample. Each model runs gapcheck-one
// in its own child process under a timeout (so a hang can't stall the sweep); limited parallelism
// keeps the per-engine timings honest (not starved). Writes one JSON line per model to stdout.
import { readdirSync, statSync, writeFileSync } from "node:fs";
import { join, dirname, relative } from "node:path";
import { fileURLToPath } from "node:url";
import { spawn } from "node:child_process";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const worker = join(root, "test", "gapcheck-one.ts");
const scanDir = join(root, "abc_0000_step_v00");
const STRIDE = Number(process.argv[2] ?? 80);
const JOBS = Number(process.argv[3] ?? 4);
const TIMEOUT = Number(process.argv[4] ?? 60) * 1000;

const all: string[] = [];
const scan = (d: string): void => { for (const e of readdirSync(d)) { const p = join(d, e); if (statSync(p).isDirectory()) scan(p); else if (/\.ste?p$/i.test(e)) all.push(p); } };
scan(scanDir); all.sort();
const sample = all.filter((_, i) => i % STRIDE === 0);
console.error(`sample: ${sample.length} models (stride ${STRIDE}), ${JOBS} parallel, ${TIMEOUT / 1000}s timeout`);

const runOne = (file: string): Promise<any> => new Promise((resolve) => {
  const child = spawn(process.execPath, [worker, file, '{"samples":2000}'], { cwd: root });
  let out = "";
  child.stdout.on("data", (d) => { out += d; });
  const killer = setTimeout(() => child.kill(), TIMEOUT);
  child.on("close", () => {
    clearTimeout(killer);
    const rel = relative(scanDir, file);
    try {
      const mark = out.lastIndexOf("@@GAPCHECK@@");
      const r = JSON.parse(out.slice(mark + "@@GAPCHECK@@".length));
      const o = r.ours || {}, c = r.occ || {}, dv = r.deviation;
      resolve({ file: rel, status: r.status, oursMs: o.ms ?? null, occMs: c.ms ?? null, oursTris: o.tris ?? null, occTris: c.tris ?? null, devPct: dv ? +(100 * dv.oursToOcc.max / dv.diag).toFixed(2) : null, vol: r.volRatio ? +((r.volRatio - 1) * 100).toFixed(1) : null });
    } catch { resolve({ file: rel, status: "TIMEOUT_OR_CRASH", oursMs: null, occMs: null }); }
  });
});

const results: any[] = [];
let next = 0, done = 0;
await new Promise<void>((resolveAll) => {
  const pump = (): void => {
    if (done === sample.length) { resolveAll(); return; }
    while (next < sample.length && next - done < JOBS) {
      void runOne(sample[next++]!).then((r) => { done++; results.push(r); console.error(`[${done}/${sample.length}] ${r.status} ${r.file} ours=${r.oursMs}ms occ=${r.occMs}ms`); pump(); });
    }
  };
  pump();
});
writeFileSync(join(root, "out/abc-timing.jsonl"), results.map((r) => JSON.stringify(r)).join("\n"));
console.error("wrote out/abc-timing.jsonl");
