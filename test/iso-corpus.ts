// SPDX-License-Identifier: AGPL-3.0-only
// Default vs isotropic mode over the local STEP corpus: one row per model with triangle count,
// tessellation time, shape quality and the correctness verdict (open / non-manifold edges,
// unmeshed faces, folds, flips) for both modes. This is the merge gate for the isotropic mode:
// it may cost triangles and time, but it must not lose watertightness or add folds/flips.
//
//   node test/iso-corpus.ts [--full] [--jobs n] [--timeout s] [--no-audit] [name ...]
//
// Each model runs test/mesh-quality.ts in its own process (a crash or timeout costs one row).
// Tolerances are the web viewer's auto settings for the model's size. Results: out/iso-corpus.json.
import { readdirSync, existsSync, readFileSync, mkdirSync, writeFileSync, rmSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { spawn } from "node:child_process";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const argv = process.argv.slice(2);
const num = (n: string, d: number): number => { const i = argv.indexOf(`--${n}`); return i >= 0 ? Number(argv[i + 1]) : d; };
const jobs = num("jobs", 4), timeout = num("timeout", 600);
const full = argv.includes("--full"), noAudit = argv.includes("--no-audit");
const named = argv.filter((a, i) => !a.startsWith("--") && !["--jobs", "--timeout"].includes(argv[i - 1] ?? ""));
// Same slow set as test/char.ts (kept out of the default run).
const SLOW = /GoProHandlePod|ETA 6497|Belt Tensioner|wallganizer|Ontos_V2|boomerang-v2|bottle-cage|SMILEY|nist_ctc_0[1-5]|nist_ftc_0[678]_asme1_ap242-e2|nist_stc_0[78]|weldment_asm/;

const models: { key: string; file: string }[] = [];
for (const dir of ["", "sourcemodels", "newtestmodels", "out/pretty"]) {
  const abs = join(root, dir);
  if (!existsSync(abs)) continue;
  for (const f of readdirSync(abs)) {
    if (!/\.(step|stp)$/i.test(f)) continue;
    const key = (dir ? dir + "/" : "") + f.replace(/\.(step|stp)$/i, "");
    if (named.length ? !named.some((q) => key.includes(q)) : !full && SLOW.test(key)) continue;
    models.push({ key, file: join(abs, f) });
  }
}
models.sort((a, b) => a.key.localeCompare(b.key));
const tmp = join(root, "out", "iso-corpus-tmp");
mkdirSync(tmp, { recursive: true });

interface Row { label: string; tris: number; ms: number; minAngP5: number; lt20: number; meanQ: number; stubs: number; open: number; nonManifold: number; warnings: number; untriangulated: number; audit?: { folds: number; flips: number; devRatio: number } }
const results: Record<string, { rows?: Row[]; error?: string }> = {};
let next = 0;
const runOne = (): Promise<void> => new Promise((resolve) => {
  const m = models[next++];
  if (!m) return resolve();
  const json = join(tmp, `${next}.json`);
  const child = spawn(process.execPath, ["--max-old-space-size=8192", join(root, "test", "mesh-quality.ts"), m.file, "--compare", "--runs", "1", "--json", json, ...(noAudit ? ["--no-audit"] : [])], { stdio: ["ignore", "ignore", "pipe"] });
  let err = "";
  child.stderr.on("data", (d) => { err += d; });
  const timer = setTimeout(() => { child.kill("SIGKILL"); err = `timeout ${timeout}s`; }, timeout * 1000);
  child.on("close", (code) => {
    clearTimeout(timer);
    if (code === 0 && existsSync(json)) results[m.key] = { rows: JSON.parse(readFileSync(json, "utf8")) };
    else results[m.key] = { error: err.trim().split("\n").pop()?.slice(0, 120) || `exit ${code}` };
    const r = results[m.key]!;
    const [d, i] = r.rows ?? [];
    const bad = (x?: Row): string => (x ? `${x.open}/${x.nonManifold}/${x.untriangulated}/${x.audit ? `${x.audit.folds}/${x.audit.flips}` : "-/-"}` : "");
    console.log(d && i
      ? `${m.key.slice(0, 44).padEnd(45)}${String(d.tris).padStart(9)}${String(i.tris).padStart(9)}${(i.tris / d.tris).toFixed(2).padStart(6)}${d.ms.toFixed(0).padStart(8)}${i.ms.toFixed(0).padStart(8)}${(i.ms / d.ms).toFixed(2).padStart(6)}${d.meanQ.toFixed(3).padStart(7)}${i.meanQ.toFixed(3).padStart(7)}${(100 * d.lt20).toFixed(1).padStart(7)}${(100 * i.lt20).toFixed(2).padStart(7)}  ${bad(d).padEnd(16)}${bad(i).padEnd(16)}${i.stubs}`
      : `${m.key.slice(0, 44).padEnd(45)}ERROR ${r.error}`);
    resolve(runOne());
  });
});
console.log(`${"model".padEnd(45)}${"tris def".padStart(9)}${"iso".padStart(9)}${"x".padStart(6)}${"ms def".padStart(8)}${"iso".padStart(8)}${"x".padStart(6)}${"Q def".padStart(7)}${"iso".padStart(7)}${"<20 def".padStart(7)}${"iso".padStart(7)}  ${"open/nm/unm/fold/flip def".padEnd(16)} iso            stubs`);
await Promise.all(Array.from({ length: jobs }, runOne));
rmSync(tmp, { recursive: true, force: true });
writeFileSync(join(root, "out", "iso-corpus.json"), JSON.stringify(results, null, 1));

// Verdict: the isotropic mode may never be less correct than the default on any model.
let worse = 0, n = 0, tD = 0, tI = 0, msD = 0, msI = 0;
for (const [key, r] of Object.entries(results)) {
  if (!r.rows) { worse++; continue; }
  const [d, i] = r.rows as [Row, Row];
  n++; tD += d.tris; tI += i.tris; msD += d.ms; msI += i.ms;
  const w: string[] = [];
  if (i.open > d.open) w.push(`open ${d.open}->${i.open}`);
  if (i.nonManifold > d.nonManifold) w.push(`non-manifold ${d.nonManifold}->${i.nonManifold}`);
  if (i.untriangulated > d.untriangulated) w.push(`unmeshed ${d.untriangulated}->${i.untriangulated}`);
  if (i.audit && d.audit && i.audit.folds > d.audit.folds) w.push(`folds ${d.audit.folds}->${i.audit.folds}`);
  if (i.audit && d.audit && i.audit.flips > d.audit.flips) w.push(`flips ${d.audit.flips}->${i.audit.flips}`);
  if (w.length) { worse++; console.log(`WORSE ${key}: ${w.join(", ")}`); }
}
console.log(`\n${n} models: triangles x${(tI / tD).toFixed(2)}, time x${(msI / msD).toFixed(2)}; ${worse} model(s) worse or failed in isotropic mode`);
process.exit(worse ? 1 : 0);
