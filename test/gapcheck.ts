// SPDX-License-Identifier: AGPL-3.0-only
// Gap-finder batch runner: convert every STEP file in a folder with meshStep AND
// OpenCASCADE, compare them (global + per-face localized metrics, see gapcheck-one.ts)
// and rank the models that need attention. Each model runs in its own child process
// so a hang / crash / OOM in one scraped file cannot take down the run.
//
//   node test/gapcheck.ts [dir] [--jobs N] [--samples N] [--timeout SECONDS] [--outdir DIR]
//
// dir defaults to sourceModels/. Everything lands in <outdir> (default
// out/gapcheck-<dirname>/): converted/*.stl for review, images/*.png failure
// pictures, gapcheck.json, gapcheck.md and report.html.
import { readdirSync, statSync, writeFileSync, mkdirSync } from "node:fs";
import { join, dirname, relative, basename } from "node:path";
import { fileURLToPath } from "node:url";
import { spawn } from "node:child_process";
import { cpus } from "node:os";
import { DEFAULT_OPTS } from "./gapcheck-one.ts";
import { writeHtmlReport } from "./gapcheck-report.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const worker = join(root, "test", "gapcheck-one.ts");

// ---------- CLI ----------
const argv = process.argv.slice(2);
const flag = (name: string, dflt: number): number => {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 ? Number(argv[i + 1]) : dflt;
};
const strFlag = (name: string, dflt: string): string => {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 ? argv[i + 1]! : dflt;
};
const dir = argv.find((a) => !a.startsWith("--") && argv[argv.indexOf(a) - 1]?.startsWith("--") !== true) ?? "sourceModels";
const jobs = flag("jobs", Math.max(1, Math.min(4, cpus().length - 2)));
const timeoutMs = flag("timeout", 600) * 1000;

const scanDir = join(root, dir);
const outBase = join(root, strFlag("outdir", join("out", `gapcheck-${basename(scanDir)}`)));
mkdirSync(join(outBase, "converted"), { recursive: true });
mkdirSync(join(outBase, "images"), { recursive: true });
const opts = {
  ...DEFAULT_OPTS,
  samples: flag("samples", DEFAULT_OPTS.samples),
  stlDir: join(outBase, "converted"),
  imgDir: join(outBase, "images"),
};
const files: string[] = [];
const scan = (d: string): void => {
  for (const e of readdirSync(d)) {
    const p = join(d, e);
    if (statSync(p).isDirectory()) scan(p);
    else if (/\.ste?p$/i.test(e)) files.push(p);
  }
};
scan(scanDir);
if (files.length === 0) { console.error(`no .step/.stp files under ${scanDir}`); process.exit(1); }
console.log(`gapcheck: ${files.length} models in ${dir}, ${jobs} parallel, ${opts.samples} samples/direction, ${timeoutMs / 1000}s timeout\n`);

// ---------- run one child ----------
interface Rec {
  file: string; status: string; reasons: string[];
  ours?: { tris: number; boundaryEdges: number; nonmanifoldEdges: number; sliverPct: number; ms: number } | null;
  occ?: { tris: number; ms: number } | null;
  deviation?: {
    diag: number;
    oursToOcc: { max: number; p99: number };
    occToOurs: { max: number; p99: number };
    missingAreaFrac: number;
    worstOurFaces: { faceId: number; max: number; bad: number; at: number[] }[];
    missingOccFaces: { face: number; badFrac: number; max: number; at: number[] }[];
  } | null;
  areaRatio?: number; volRatio?: number; scaleRatio?: number;
  analysis?: { cause: string; fix: string }[];
  openEdgeFaces?: { faceId: number; kind: string; openEdges: number }[];
  images?: string[]; stl?: string;
  wallMs: number; stderrTail?: string;
}

const runOne = (file: string): Promise<Rec> => new Promise((resolve) => {
  const t0 = Date.now();
  const child = spawn(process.execPath, [worker, file, JSON.stringify(opts)], { cwd: root });
  let out = "", err = "";
  child.stdout.on("data", (d: Buffer) => { out += d; });
  child.stderr.on("data", (d: Buffer) => { err += d; if (err.length > 8192) err = err.slice(-8192); });
  const killer = setTimeout(() => child.kill(), timeoutMs);
  child.on("close", (code) => {
    clearTimeout(killer);
    const wallMs = Date.now() - t0;
    const stage = err.match(/\[stage\] (\w+)(?![\s\S]*\[stage\])/)?.[1] ?? "?";
    const name = relative(scanDir, file);
    if (wallMs >= timeoutMs) {
      resolve({ file: name, status: "TIMEOUT", reasons: [`no result after ${timeoutMs / 1000}s (stuck in: ${stage})`], wallMs });
      return;
    }
    try {
      const mark = out.lastIndexOf("@@GAPCHECK@@");
      const rec = JSON.parse(mark >= 0 ? out.slice(mark + "@@GAPCHECK@@".length) : out) as Rec;
      rec.file = name;
      rec.wallMs = wallMs;
      resolve(rec);
    } catch {
      const tail = err.split("\n").filter((l) => !l.startsWith("[stage]")).slice(-6).join("\n");
      resolve({
        file: name, status: "CRASH",
        reasons: [`worker exited (code ${code}) during: ${stage}`],
        wallMs, stderrTail: tail,
      });
    }
  });
});

// ---------- pool ----------
const results: Rec[] = [];
let next = 0, done = 0;
await new Promise<void>((resolveAll) => {
  const pump = (): void => {
    if (done === files.length) { resolveAll(); return; }
    while (next < files.length && next - done < jobs) {
      const file = files[next++]!;
      // CRASH gets one retry: Windows child stdout occasionally arrives truncated
      void runOne(file)
        .then((rec) => (rec.status === "CRASH" ? runOne(file) : rec))
        .then((rec) => {
        done++;
        results.push(rec);
        const mark = rec.status === "OK" ? "  " : rec.status === "WARN" ? "! " : "!!";
        console.log(`${mark} [${done}/${files.length}] ${rec.status.padEnd(7)} ${rec.file}${rec.reasons.length ? ` — ${rec.reasons[0]}` : ""}`);
        pump();
      });
    }
  };
  pump();
});

// ---------- rank & report ----------
const RANK: Record<string, number> = { CRASH: 0, OURS_ERR: 0, TIMEOUT: 1, EMPTY: 2, SCALE: 3, FAIL: 4, WARN: 5, NOREF: 6, OK: 7 };
const score = (r: Rec): number => {
  const d = r.deviation;
  if (!d) return 0;
  return Math.max(d.oursToOcc.max / d.diag, d.occToOurs.max / d.diag, d.missingAreaFrac * 10);
};
results.sort((a, b) => (RANK[a.status] ?? 9) - (RANK[b.status] ?? 9) || score(b) - score(a));

const counts = new Map<string, number>();
for (const r of results) counts.set(r.status, (counts.get(r.status) ?? 0) + 1);
const summary = [...counts.entries()].map(([s, n]) => `${n} ${s}`).join(", ");
console.log(`\n=== ${summary} ===\n`);

const pctD = (r: Rec, v?: number): string =>
  v === undefined || !r.deviation ? "-" : `${((v / r.deviation.diag) * 100).toFixed(3)}%`;

// console table for everything that is not OK
const flagged = results.filter((r) => r.status !== "OK");
if (flagged.length) {
  console.log("status   ours>occ  occ>ours  missA%   areaΔ%   volΔ%   model");
  for (const r of flagged) {
    const d = r.deviation;
    console.log(
      r.status.padEnd(8),
      pctD(r, d?.oursToOcc.max).padStart(8),
      pctD(r, d?.occToOurs.max).padStart(9),
      (d ? (d.missingAreaFrac * 100).toFixed(2) : "-").padStart(8),
      (r.areaRatio ? ((r.areaRatio - 1) * 100).toFixed(2) : "-").padStart(8),
      (r.volRatio ? ((r.volRatio - 1) * 100).toFixed(2) : "-").padStart(7),
      ` ${r.file}`,
    );
  }
}

// ---------- markdown + json ----------
writeFileSync(join(outBase, "gapcheck.json"), JSON.stringify({ dir, opts, results }, null, 1));

const md: string[] = [];
md.push(`# gapcheck — \`${dir}\`, ${files.length} models`, "");
md.push(`Settings: surfDev ${opts.surfDevRel}·D (ours) + ${opts.occDeflRel}·D (OCC), ${opts.samples} samples/direction, fail at ${opts.failFactor}× combined tolerance.`, "");
md.push(`**${summary}**`, "");
md.push("| status | model | tris | ours→occ max | occ→ours max | missing area | area Δ | vol Δ | time |");
md.push("|---|---|---|---|---|---|---|---|---|");
for (const r of results) {
  const d = r.deviation;
  md.push(`| ${r.status} | ${r.file} | ${r.ours?.tris ?? "-"} | ${pctD(r, d?.oursToOcc.max)} | ${pctD(r, d?.occToOurs.max)} | ${d ? (d.missingAreaFrac * 100).toFixed(2) + "%" : "-"} | ${r.areaRatio ? ((r.areaRatio - 1) * 100).toFixed(2) + "%" : "-"} | ${r.volRatio ? ((r.volRatio - 1) * 100).toFixed(2) + "%" : "-"} | ${(r.wallMs / 1000).toFixed(1)}s |`);
}
md.push("");
for (const r of results) {
  if (r.status === "OK") continue;
  md.push(`## ${r.status} — ${r.file}`, "");
  for (const reason of r.reasons) md.push(`- ${reason}`);
  const d = r.deviation;
  if (d?.worstOurFaces.some((f) => f.bad > 0)) {
    md.push("", "Worst meshStep faces (ours→OCC):", "", "| STEP face | max dev | bad samples | near |", "|---|---|---|---|");
    for (const f of d.worstOurFaces.filter((f) => f.bad > 0)) {
      md.push(`| #${f.faceId} | ${f.max.toFixed(4)}mm (${pctD(r, f.max)}) | ${f.bad} | [${f.at}] |`);
    }
  }
  if (d?.missingOccFaces.length) {
    md.push("", "OCC faces with no counterpart in our mesh (occ→ours):", "", "| OCC face | bad frac | max dev | near |", "|---|---|---|---|");
    for (const f of d.missingOccFaces) {
      md.push(`| #${f.face} | ${(f.badFrac * 100).toFixed(0)}% | ${f.max.toFixed(4)}mm | [${f.at}] |`);
    }
  }
  if (r.stderrTail) md.push("", "```", r.stderrTail, "```");
  md.push("");
}
writeFileSync(join(outBase, "gapcheck.md"), md.join("\n"));
writeHtmlReport(outBase);
console.log(`\nreports: ${relative(root, outBase)}\\report.html (+ gapcheck.md, gapcheck.json, converted\\, images\\)`);
