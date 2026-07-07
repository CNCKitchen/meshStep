// SPDX-License-Identifier: AGPL-3.0-only
// ABC-dataset batch survey orchestrator.
//
// Walks a folder of STEP files, converts every one with meshStep through a POOL of
// long-lived worker processes (test/abc-worker.ts), and reports how many produced a
// watertight, fully-tessellated mesh and — for those that didn't — the grouped root cause.
//
//   node test/abc-batch.ts [dir] [--jobs N] [--timeout SEC] [--outdir DIR] [--limit N]
//
// Robustness: each worker handles one file at a time under a watchdog. A per-file HANG is
// killed and recorded TIMEOUT; a native CRASH/OOM (worker exits mid-file) is recorded CRASH;
// both respawn a fresh worker so one pathological model never stalls or corrupts the run.
// Every verdict is appended to results.jsonl immediately, so the run is crash-safe and
// resumable — re-running skips files already recorded.
import { readdirSync, statSync, existsSync, readFileSync, mkdirSync, createWriteStream, writeFileSync } from "node:fs";
import { join, dirname, relative, basename } from "node:path";
import { fileURLToPath } from "node:url";
import { spawn, type ChildProcess } from "node:child_process";
import { createInterface } from "node:readline";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const workerPath = join(root, "test", "abc-worker.ts");
const SENT = "@@ABC@@";

// ---------- CLI ----------
const argv = process.argv.slice(2);
const numFlag = (n: string, d: number): number => { const i = argv.indexOf(`--${n}`); return i >= 0 ? Number(argv[i + 1]) : d; };
const strFlag = (n: string, d: string): string => { const i = argv.indexOf(`--${n}`); return i >= 0 ? argv[i + 1]! : d; };
const dirArg = argv.find((a, i) => !a.startsWith("--") && !argv[i - 1]?.startsWith("--")) ?? "abc_0000_step_v00";
const scanDir = join(root, dirArg);
const JOBS = numFlag("jobs", 14);
const TIMEOUT_MS = numFlag("timeout", 90) * 1000;
const LIMIT = numFlag("limit", Infinity);
const outBase = join(root, strFlag("outdir", join("out", `abc-${basename(scanDir)}`)));
mkdirSync(outBase, { recursive: true });
const jsonlPath = join(outBase, "results.jsonl");

// ---------- enumerate ----------
const listArg = strFlag("list", "");
const allFiles: string[] = [];
if (listArg) {
  // --list <file>: newline-delimited paths RELATIVE to scanDir (re-run a targeted subset).
  console.log(`reading file list ${listArg} ...`);
  for (const line of readFileSync(join(root, listArg), "utf8").split("\n")) {
    const rel = line.trim();
    if (rel) allFiles.push(join(scanDir, rel.replace(/\\/g, "/")));
  }
} else {
  console.log(`scanning ${dirArg} ...`);
  const scan = (d: string): void => {
    for (const e of readdirSync(d)) {
      const p = join(d, e);
      if (statSync(p).isDirectory()) scan(p);
      else if (/\.ste?p$/i.test(e)) allFiles.push(p);
    }
  };
  scan(scanDir);
}
allFiles.sort();
const targetFiles = Number.isFinite(LIMIT) ? allFiles.slice(0, LIMIT) : allFiles;
console.log(`found ${allFiles.length} STEP files${Number.isFinite(LIMIT) ? ` (limited to ${targetFiles.length})` : ""}`);

// ---------- resume ----------
interface Rec { file: string; status: string; bucket: string; wallMs?: number; [k: string]: unknown }
const done = new Set<string>();
const results: Rec[] = [];
if (existsSync(jsonlPath)) {
  for (const line of readFileSync(jsonlPath, "utf8").split("\n")) {
    if (!line.trim()) continue;
    try { const r = JSON.parse(line) as Rec; done.add(r.file); results.push(r); } catch { /* skip partial line */ }
  }
  console.log(`resuming: ${done.size} already recorded, skipping them`);
}
const queue = targetFiles.map((f) => relative(scanDir, f)).filter((f) => !done.has(f));
console.log(`queue: ${queue.length} to process, ${JOBS} workers, ${TIMEOUT_MS / 1000}s/model timeout\n`);

const out = createWriteStream(jsonlPath, { flags: "a" });
const record = (rec: Rec): void => { results.push(rec); out.write(JSON.stringify(rec) + "\n"); };

// ---------- progress ----------
const runStart = Date.now();
let processed = 0;
const tally = (): Record<string, number> => {
  const c: Record<string, number> = {};
  for (const r of results) c[r.status] = (c[r.status] ?? 0) + 1;
  return c;
};
const logProgress = (): void => {
  const elapsed = (Date.now() - runStart) / 1000;
  const rate = processed / (elapsed || 1);
  const left = queue.length + busyCount();
  const eta = rate > 0 ? left / rate : 0;
  const c = tally();
  const line = `[${done.size + processed}/${targetFiles.length}] ${rate.toFixed(1)}/s  ETA ${(eta / 60).toFixed(1)}m  | PASS ${c.PASS ?? 0} FAIL ${c.FAIL ?? 0} ERR ${(c.ERR ?? 0) + (c.CRASH ?? 0)} TO ${c.TIMEOUT ?? 0} EMPTY ${c.EMPTY ?? 0}`;
  console.log(line);
};

// ---------- worker pool ----------
interface Worker { id: number; proc: ChildProcess; ready: boolean; file: string | null; startedAt: number; killing: boolean; stderr: string }
const workers: Worker[] = [];
const busyCount = (): number => workers.reduce((n, w) => n + (w.file ? 1 : 0), 0);

function spawnWorker(id: number): Worker {
  const proc = spawn(process.execPath, ["--max-old-space-size=4096", workerPath], { cwd: root });
  const w: Worker = { id, proc, ready: false, file: null, startedAt: 0, killing: false, stderr: "" };
  proc.stderr.on("data", (d: Buffer) => { w.stderr += d; if (w.stderr.length > 4096) w.stderr = w.stderr.slice(-4096); });
  const rl = createInterface({ input: proc.stdout });
  rl.on("line", (line) => {
    const i = line.indexOf(SENT);
    if (i < 0) return; // stray stdout
    let msg: any;
    try { msg = JSON.parse(line.slice(i + SENT.length)); } catch { return; }
    if (msg.ready) { w.ready = true; dispatch(w); return; }
    finishFile(w, msg as Rec);
  });
  proc.on("exit", () => {
    if (w.file && !w.killing) {
      // crashed / OOM mid-file: record the victim, then respawn
      record({ file: w.file, status: "CRASH", bucket: "crash", wallMs: Date.now() - w.startedAt, err: w.stderr.split("\n").filter(Boolean).slice(-4).join(" | ") });
      processed++;
    }
    const idx = workers.indexOf(w);
    if (idx >= 0) workers.splice(idx, 1);
    if (!draining) { const nw = spawnWorker(w.id); workers.push(nw); }
    maybeFinish();
  });
  return w;
}

function dispatch(w: Worker): void {
  if (!w.ready || w.file || draining) return;
  const next = queue.shift();
  if (next === undefined) { maybeFinish(); return; }
  w.file = next;
  w.startedAt = Date.now();
  w.stderr = "";
  w.proc.stdin!.write(join(scanDir, next) + "\n");
}

function finishFile(w: Worker, rec: Rec): void {
  if (!w.file) return;
  rec.file = w.file;
  rec.wallMs = Date.now() - w.startedAt;
  record(rec);
  w.file = null;
  processed++;
  if (processed % 250 === 0) logProgress();
  dispatch(w);
}

// ---------- watchdog: kill+respawn any worker stuck past the timeout ----------
const watchdog = setInterval(() => {
  const now = Date.now();
  for (const w of workers) {
    if (w.file && !w.killing && now - w.startedAt > TIMEOUT_MS) {
      w.killing = true;
      record({ file: w.file, status: "TIMEOUT", bucket: "timeout", wallMs: now - w.startedAt });
      processed++;
      w.file = null;
      w.proc.kill(); // 'exit' handler respawns a fresh worker
    }
  }
}, 2000);

// ---------- shutdown ----------
let draining = false;
let finished = false;
function maybeFinish(): void {
  if (finished || draining) return;
  if (queue.length === 0 && busyCount() === 0) { finished = true; shutdown(); }
}

function shutdown(): void {
  draining = true;
  clearInterval(watchdog);
  for (const w of workers) { try { w.proc.stdin?.end(); w.proc.kill(); } catch { /* ignore */ } }
  out.end(() => { writeReport(); });
}

// ---------- report ----------
function writeReport(): void {
  logProgress();
  const c = tally();
  const total = results.length;
  const pass = c.PASS ?? 0;

  // group buckets
  const bucketCount = new Map<string, number>();
  const bucketFiles = new Map<string, string[]>();
  const producerFail = new Map<string, { fail: number; total: number }>();
  const skippedKinds = new Map<string, number>();
  const seamKinds = new Map<string, number>();
  const errSamples = new Map<string, { msg: string; file: string }[]>();
  for (const r of results) {
    const p = (r as any).producer as string | undefined;
    if (p) { const e = producerFail.get(p) ?? { fail: 0, total: 0 }; e.total++; if (r.status === "FAIL" || r.status === "ERR" || r.status === "EMPTY") e.fail++; producerFail.set(p, e); }
    if (r.status === "PASS") continue;
    bucketCount.set(r.bucket, (bucketCount.get(r.bucket) ?? 0) + 1);
    const bf = bucketFiles.get(r.bucket) ?? []; if (bf.length < 8) { bf.push(r.file); bucketFiles.set(r.bucket, bf); }
    if ((r as any).skipped) for (const [k, n] of Object.entries((r as any).skipped as Record<string, number>)) skippedKinds.set(k, (skippedKinds.get(k) ?? 0) + n);
    if (r.bucket.startsWith("seam-leak:")) { const k = r.bucket.slice("seam-leak:".length); seamKinds.set(k, (seamKinds.get(k) ?? 0) + 1); }
    if (r.status === "ERR") { const s = errSamples.get(r.bucket) ?? []; if (s.length < 6) { s.push({ msg: String((r as any).err ?? "").slice(0, 160), file: r.file }); errSamples.set(r.bucket, s); } }
  }
  const sortedBuckets = [...bucketCount.entries()].sort((a, b) => b[1] - a[1]);

  const pct = (n: number): string => `${((100 * n) / total).toFixed(2)}%`;
  const md: string[] = [];
  md.push(`# ABC dataset conversion survey — \`${basename(scanDir)}\``, "");
  md.push(`meshStep no-remesh import, scale-relative tolerance (chord 5e-4·diag, maxEdge 0.05·diag). ${JOBS} workers, ${TIMEOUT_MS / 1000}s/model timeout.`, "");
  md.push(`## Headline`, "");
  md.push(`- **Models converted:** ${total}`);
  md.push(`- **Meshed properly (watertight + all faces tessellated): ${pass} — ${pct(pass)}**`);
  md.push(`- Not watertight / defective: ${c.FAIL ?? 0} (${pct(c.FAIL ?? 0)})`);
  md.push(`- Import threw / crashed: ${(c.ERR ?? 0) + (c.CRASH ?? 0)} (${pct((c.ERR ?? 0) + (c.CRASH ?? 0))})`);
  md.push(`- Timed out (>${TIMEOUT_MS / 1000}s): ${c.TIMEOUT ?? 0} (${pct(c.TIMEOUT ?? 0)})`);
  md.push(`- Empty (0 triangles): ${c.EMPTY ?? 0} (${pct(c.EMPTY ?? 0)})`, "");

  md.push(`## Failure causes, grouped (most common first)`, "");
  md.push(`| # | cause bucket | count | % of all | example files |`, `|---|---|---|---|---|`);
  sortedBuckets.forEach(([b, n], i) => {
    md.push(`| ${i + 1} | \`${b}\` | ${n} | ${pct(n)} | ${(bucketFiles.get(b) ?? []).slice(0, 3).map((f) => f.split(/[\\/]/)[0]).join(", ")} |`);
  });
  md.push("");

  if (seamKinds.size) {
    md.push(`### Seam-leak breakdown by surface kind (watertight leaks, all faces meshed)`, "");
    md.push(`| surface kind | models |`, `|---|---|`);
    for (const [k, n] of [...seamKinds.entries()].sort((a, b) => b[1] - a[1])) md.push(`| ${k} | ${n} |`);
    md.push("");
  }
  if (skippedKinds.size) {
    md.push(`### Skipped-face kinds (faces meshStep never tessellated -> holes)`, "");
    md.push(`| kind | total faces skipped |`, `|---|---|`);
    for (const [k, n] of [...skippedKinds.entries()].sort((a, b) => b[1] - a[1])) md.push(`| ${k} | ${n} |`);
    md.push("");
  }
  if (errSamples.size) {
    md.push(`### Exception samples`, "");
    for (const [b, s] of errSamples) { md.push(`**\`${b}\`**`); for (const e of s) md.push(`- \`${e.msg}\`  (${e.file.split(/[\\/]/)[0]})`); md.push(""); }
  }

  const worstProducers = [...producerFail.entries()].filter(([p, e]) => p !== "?" && e.total >= 20)
    .map(([p, e]) => ({ p, ...e, rate: e.fail / e.total })).sort((a, b) => b.rate - a.rate).slice(0, 15);
  if (worstProducers.length) {
    md.push(`## Failure rate by originating CAD system (>=20 models)`, "");
    md.push(`| originating_system | models | failed | fail rate |`, `|---|---|---|---|`);
    for (const w of worstProducers) md.push(`| ${w.p} | ${w.total} | ${w.fail} | ${(100 * w.rate).toFixed(1)}% |`);
    md.push("");
  }

  const reportPath = join(outBase, "report.md");
  writeFileSync(reportPath, md.join("\n"));
  writeFileSync(join(outBase, "summary.json"), JSON.stringify({
    total, statusCounts: c, buckets: Object.fromEntries(sortedBuckets),
    seamKinds: Object.fromEntries(seamKinds), skippedKinds: Object.fromEntries(skippedKinds),
    elapsedSec: (Date.now() - runStart) / 1000,
  }, null, 1));

  console.log(`\n=== DONE: ${total} models, ${pass} PASS (${pct(pass)}) ===`);
  console.log(sortedBuckets.slice(0, 12).map(([b, n]) => `  ${String(n).padStart(5)}  ${b}`).join("\n"));
  console.log(`\nreport: ${relative(root, reportPath)}  (+ summary.json, results.jsonl)`);
  process.exit(0);
}

// ---------- go ----------
if (queue.length === 0) { console.log("nothing to do (all files already recorded)"); writeReport(); }
else for (let i = 0; i < Math.min(JOBS, queue.length); i++) workers.push(spawnWorker(i));
