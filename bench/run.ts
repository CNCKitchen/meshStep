// SPDX-License-Identifier: AGPL-3.0-only
// Benchmark orchestrator: every converter on every model, each run in its own process.
//
//   node bench/run.ts [--corpus char|abc[:N]] [--tier fine|coarse] [--adapters a,b,..]
//                     [--jobs N] [--only text] [--out file.jsonl] [--timeout s]
//
//   --corpus char     the characterization corpus (repo models, sourcemodels/, newtestmodels/)
//   --corpus abc:500  500 models spread evenly over ABC chunk 0002 (abc_0002_step_v00/)
//   --tier fine       chord tolerance 1e-4 x model diagonal, 15 degrees (meshStep viewer's default)
//   --tier coarse     1e-3 x diagonal, 30 degrees (a typical "view it on screen" setting)
//   --jobs N          models run N at a time; the converters of one model always run back to back
//                     in one slot, so they see the same machine load (default 4)
//
// Results are appended to bench/out/<corpus>-<tier>.jsonl (one line per model x converter); a rerun
// skips pairs already there, so an interrupted run continues. Model sizes and the fine reference
// meshes are cached in bench/out/cache/. Summarize with bench/report.ts.
import { spawn } from "node:child_process";
import { existsSync, mkdirSync, readFileSync, readdirSync, appendFileSync, writeFileSync, statSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { createHash } from "node:crypto";
import { ADAPTERS, REFERENCES } from "./adapters.ts";

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, "..");

const argv = process.argv.slice(2);
const flag = (name: string, def?: string): string | undefined => {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 ? argv[i + 1] : def;
};
const corpusArg = flag("corpus", "char")!;
const tier = flag("tier", "fine")!;
const jobs = Number(flag("jobs", "4"));
const only = flag("only");
const timeoutS = Number(flag("timeout", "120"));
const adapterIds = (flag("adapters") ?? Object.keys(ADAPTERS).join(",")).split(",");
for (const id of adapterIds) if (!ADAPTERS[id]) throw new Error(`unknown adapter ${id}`);

export const TIERS: Record<string, { rel: number; angleDeg: number }> = {
  fine: { rel: 1e-4, angleDeg: 15 },
  coarse: { rel: 1e-3, angleDeg: 30 },
};
const T = TIERS[tier];
if (!T) throw new Error(`unknown tier ${tier}`);
/** References are meshed this much finer than the tolerance under test. */
const REF_FACTOR = 5;

// ---------------------------------------------------------------- corpus

interface Model { key: string; path: string }

function charCorpus(): Model[] {
  const out: Model[] = [];
  for (const dir of ["", "sourcemodels", "newtestmodels"]) {
    const abs = join(root, dir);
    if (!existsSync(abs)) continue;
    for (const f of readdirSync(abs).sort()) {
      if (!/\.(step|stp)$/i.test(f)) continue;
      const name = f.replace(/\.(step|stp)$/i, "");
      out.push({ key: dir ? `${dir}/${name}` : name, path: join(abs, f) });
    }
  }
  return out;
}

function abcCorpus(n: number): Model[] {
  const abs = join(root, "abc_0002_step_v00");
  const dirs = readdirSync(abs, { withFileTypes: true }).filter(d => d.isDirectory()).map(d => d.name).sort();
  const step = Math.max(1, Math.floor(dirs.length / n));
  const out: Model[] = [];
  for (let i = 0; i < dirs.length && out.length < n; i += step) {
    const files = readdirSync(join(abs, dirs[i]!)).filter(f => /\.(step|stp)$/i.test(f));
    if (files.length) out.push({ key: `abc/${dirs[i]}`, path: join(abs, dirs[i]!, files[0]!) });
  }
  return out;
}

const [corpusName, corpusN] = corpusArg.split(":");
let models = corpusName === "abc" ? abcCorpus(Number(corpusN ?? 500)) : charCorpus();
if (only) models = models.filter(m => m.key.includes(only));
// the same file can sit in two corpus folders: measure it once
{
  const seen = new Set<string>();
  models = models.filter(m => {
    const h = createHash("sha1").update(readFileSync(m.path)).digest("hex");
    if (seen.has(h)) return false;
    seen.add(h); return true;
  });
}

// ---------------------------------------------------------------- child processes

function child(args: string[], timeoutMs: number): Promise<Record<string, any>> {
  return new Promise(resolve => {
    const p = spawn(process.execPath, ["--max-old-space-size=16384", join(here, "one.ts"), ...args], { stdio: ["ignore", "pipe", "pipe"] });
    let out = "", err = "";
    p.stdout.on("data", d => { out += d; });
    p.stderr.on("data", d => { if (err.length < 4000) err += d; });
    const timer = setTimeout(() => { p.kill("SIGKILL"); resolve({ status: "timeout" }); }, timeoutMs);
    p.on("close", code => {
      clearTimeout(timer);
      const line = out.trim().split("\n").filter(l => l.startsWith("{")).pop();
      if (line) { try { resolve(JSON.parse(line)); return; } catch { /* fall through */ } }
      resolve({ status: "crash", error: `exit ${code}: ${err.trim().split("\n").slice(-3).join(" | ").slice(0, 300)}` });
    });
  });
}

// ---------------------------------------------------------------- caches

const outDir = join(here, "out");
const cacheDir = join(outDir, "cache");
mkdirSync(join(cacheDir, "ref"), { recursive: true });
const diagPath = join(cacheDir, "diag.json");
const diagCache: Record<string, { diag: number; from: string }> = existsSync(diagPath) ? JSON.parse(readFileSync(diagPath, "utf8")) : {};
const fileId = (m: Model): string => {
  const s = statSync(m.path);
  return createHash("sha1").update(`${m.key}|${s.size}|${s.mtimeMs}`).digest("hex").slice(0, 12);
};

async function diagOf(m: Model): Promise<number> {
  const id = fileId(m);
  if (!diagCache[id]) {
    const r = await child(["diag", m.path], 300_000);
    diagCache[id] = { diag: r.diag ?? 0, from: r.from ?? "none" };
    writeFileSync(diagPath, JSON.stringify(diagCache, null, 1));
  }
  return diagCache[id]!.diag;
}

/** Fine reference meshes (cached). A reference that fails is left out. */
async function refsOf(m: Model, diag: number, tol: number): Promise<{ id: string; bin: string }[]> {
  const out: { id: string; bin: string }[] = [];
  for (const alternatives of REFERENCES) {
    for (const id of alternatives) {
      const base = join(cacheDir, "ref", `${fileId(m)}-${tier}-${id}`);
      const bin = `${base}.bin`, status = `${base}.json`;
      if (!existsSync(status)) {
        const params = { deflection: tol / REF_FACTOR, angleDeg: T!.angleDeg, diag };
        const r = await child(["ref", id, m.path, JSON.stringify(params), bin], 600_000);
        writeFileSync(status, JSON.stringify(r));
      }
      const r = JSON.parse(readFileSync(status, "utf8"));
      if (r.status === "ok" && existsSync(bin)) { out.push({ id, bin }); break; }
    }
  }
  return out;
}

// ---------------------------------------------------------------- main loop

const outPath = flag("out") ?? join(outDir, `${corpusName}-${tier}.jsonl`);
const done = new Set<string>();
if (existsSync(outPath)) for (const l of readFileSync(outPath, "utf8").split("\n")) {
  if (!l.trim()) continue;
  try { const r = JSON.parse(l); done.add(`${r.model}|${r.adapter}`); } catch { /* skip */ }
}

let next = 0, finished = 0;
const t0 = Date.now();
async function slot(): Promise<void> {
  while (next < models.length) {
    const mi = next++;
    const m = models[mi]!;
    const todo = adapterIds.filter(id => !done.has(`${m.key}|${id}`));
    if (!todo.length) { finished++; continue; }
    const diag = await diagOf(m);
    const base = { model: m.key, corpus: corpusName, tier, diag, tol: diag * T!.rel, angleDeg: T!.angleDeg };
    if (!(diag > 0)) {
      for (const id of todo) appendFileSync(outPath, JSON.stringify({ ...base, adapter: id, status: "unreadable" }) + "\n");
      finished++; continue;
    }
    const refs = await refsOf(m, diag, base.tol);
    // rotate the order per model so no converter always runs first
    const order = todo.map((_, i) => todo[(i + mi) % todo.length]!);
    for (const id of order) {
      const params = { deflection: base.tol, angleDeg: T!.angleDeg, diag };
      const r = await child(["run", id, m.path, JSON.stringify(params), ...refs.map(x => x.bin)], timeoutS * 1000);
      appendFileSync(outPath, JSON.stringify({ ...base, adapter: id, refs: refs.map(x => x.id), ...r }) + "\n");
    }
    finished++;
    const el = (Date.now() - t0) / 1000;
    console.error(`[${finished}/${models.length}] ${m.key}  (${el.toFixed(0)} s)`);
  }
}
console.error(`${models.length} models x ${adapterIds.length} converters, tier ${tier}, ${jobs} jobs -> ${outPath}`);
await Promise.all(Array.from({ length: jobs }, slot));
console.error(`done in ${((Date.now() - t0) / 1000).toFixed(0)} s`);
