// SPDX-License-Identifier: AGPL-3.0-only
// Summarize benchmark runs into bench/RESULTS.md and the README chart docs/images/bench.svg.
//
//   node bench/report.ts [results.jsonl ...]      (default: every bench/out/*-*.jsonl)
import { existsSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { ADAPTERS } from "./adapters.ts";
import { topologyOf } from "./topology.ts";

const here = dirname(fileURLToPath(import.meta.url));
const outDir = join(here, "out");

type Rec = Record<string, any>;

const files = process.argv.slice(2).length
  ? process.argv.slice(2)
  : readdirSync(outDir).filter(f => /^[a-z]+-[a-z]+\.jsonl$/.test(f)).map(f => join(outDir, f));
const size: Record<string, { brotli: number; gzip: number; raw: number }> =
  existsSync(join(outDir, "size.json")) ? JSON.parse(readFileSync(join(outDir, "size.json"), "utf8")) : {};

const ORDER = ["meshstep", "meshstep-iso", "meshstep-par", "meshstep-dev", "occt-import-js", "tx-occt-js", "occt-wasm", "opencascade.js", "brepkit-wasm"];
const META: Record<string, { label: string; engine: string; license: string; sizeId: string }> = {};
for (const id of ORDER) {
  const a = ADAPTERS[id]?.();
  if (!a) continue;
  const license = id.startsWith("meshstep") ? "AGPL-3.0 / commercial" : id === "brepkit-wasm" ? "AGPL-3.0" : "LGPL-2.1 (OCCT)";
  META[id] = { label: a.label, engine: a.engine, license, sizeId: id.startsWith("meshstep") ? "meshstep" : id };
}
const versions: Record<string, string> = {};

const median = (v: number[]): number => { if (!v.length) return NaN; const s = [...v].sort((a, b) => a - b); const m = s.length >> 1; return s.length % 2 ? s[m]! : (s[m - 1]! + s[m]!) / 2; };
const geomean = (v: number[]): number => v.length ? Math.exp(v.reduce((s, x) => s + Math.log(x), 0) / v.length) : NaN;
const pct = (x: number, d = 0) => Number.isFinite(x) ? `${(100 * x).toFixed(d)}%` : "–";
const num = (x: number, d = 0) => Number.isFinite(x) ? x.toLocaleString("en-US", { maximumFractionDigits: d, minimumFractionDigits: d }) : "–";

/** A run that produced a usable mesh of the whole model. */
const converted = (r: Rec) => r.status === "ok" && r.tris > 0 && (r.missing === undefined || r.missing <= 0.01) && (r.diagRatio == null || Math.abs(r.diagRatio - 1) < 0.01);
/** The STEP file itself describes only closed solids (see topology.ts). Watertightness is only
 * asked for on these: no converter can be blamed for the open edges of a surface body. */
const solidInFile = (model: string) => topologyOf(model)?.closed ?? true;

interface Summary {
  id: string; n: number; converted: number; closed: number; solidModels: number; flippedModels: number; missingModels: number; failed: number;
  common: number; coldTotal: number; warmTotal: number; coldGeo: number; warmGeo: number;
  tris: number; trisRel: number; under20: number; under5: number; minAngP5: number; q: number;
  devP99: number; withinTol: number; rss: number;
}

function summarize(recs: Rec[], label: string): { rows: Summary[]; common: string[]; models: string[] } {
  const byAdapter = new Map<string, Map<string, Rec>>();
  for (const r of recs) {
    if (!byAdapter.has(r.adapter)) byAdapter.set(r.adapter, new Map());
    byAdapter.get(r.adapter)!.set(r.model, r);
    if (r.version) versions[r.adapter] = r.version;
  }
  const ids = ORDER.filter(id => byAdapter.has(id));
  const models = [...new Set(recs.map(r => r.model))].sort();
  // the speed / shape comparison runs on the models every converter converted. A converter that
  // converts under half the models would shrink that set to the trivial ones: it is left out of the
  // comparison (its speed and shape columns stay empty) and of the set's definition.
  const convertedShare = (id: string) => models.filter(m => { const r = byAdapter.get(id)!.get(m); return r && converted(r); }).length / models.length;
  const compared = new Set(ids.filter(id => convertedShare(id) >= 0.5));
  const common = models.filter(m => [...compared].every(id => { const r = byAdapter.get(id)!.get(m); return r && converted(r); }));
  const baseId = ids.includes("occt-import-js") ? "occt-import-js" : ids[0]!;
  const rows: Summary[] = [];
  for (const id of ids) {
    const runs = byAdapter.get(id)!;
    const all = models.map(m => runs.get(m)).filter(Boolean) as Rec[];
    const c = compared.has(id) ? common.map(m => runs.get(m)!) : [];
    const b = common.map(m => byAdapter.get(baseId)!.get(m)!);
    const triSum = c.reduce((s, r) => s + r.tris, 0);
    rows.push({
      id, n: all.length,
      converted: all.filter(converted).length,
      closed: all.filter(r => solidInFile(r.model) && converted(r) && r.closed).length,
      solidModels: all.filter(r => solidInFile(r.model)).length,
      flippedModels: all.filter(r => r.status === "ok" && r.flipped > 0.001).length,
      missingModels: all.filter(r => r.status === "ok" && r.missing > 0.01).length,
      failed: all.filter(r => r.status !== "ok").length,
      common: c.length,
      coldTotal: c.reduce((s, r) => s + r.initMs + r.coldMs, 0) / 1000,
      warmTotal: c.reduce((s, r) => s + (r.warmMs ?? r.coldMs), 0) / 1000,
      coldGeo: geomean(c.map((r, i) => (r.initMs + r.coldMs) / (b[i]!.initMs + b[i]!.coldMs))),
      warmGeo: geomean(c.map((r, i) => (r.warmMs ?? r.coldMs) / (b[i]!.warmMs ?? b[i]!.coldMs))),
      tris: c.length ? triSum : NaN,
      trisRel: geomean(c.map((r, i) => r.tris / b[i]!.tris)),
      under20: c.reduce((s, r) => s + r.under20 * r.tris, 0) / triSum,
      under5: c.reduce((s, r) => s + r.under5 * r.tris, 0) / triSum,
      minAngP5: median(c.map(r => r.minAngP5)),
      q: c.reduce((s, r) => s + r.radiusRatio * r.tris, 0) / triSum,
      devP99: median(c.map(r => r.devP99)),
      withinTol: c.filter(r => r.devMax <= 1.25).length / (c.length || 1),
      rss: median(c.map(r => r.peakRssMB)),
    });
  }
  console.error(`${label}: ${models.length} models, ${common.length} converted by all, ${ids.length} converters`);
  return { rows, common, models };
}

// ---------------------------------------------------------------- markdown

function tables(recs: Rec[], title: string): string {
  const { rows, common, models } = summarize(recs, title);
  const tier = recs[0]?.tier, ang = recs[0]?.angleDeg;
  const L = (id: string) => `${META[id]?.label ?? id} ${versions[id] ?? ""}`.trim();
  const out: string[] = [];
  out.push(`### ${title}`, "");
  const solid = models.filter(solidInFile).length;
  out.push(`${models.length} models · chord tolerance ${tier === "fine" ? "1e-4" : "1e-3"} × model diagonal, ${ang}° · ${common.length} models converted by every library (speed, size and shape columns use these)`, "");
  out.push(`**Robustness** (all models; watertight counts the ${solid} models whose STEP file describes only closed solids. The other ${models.length - solid} contain surface bodies or no topology, so they cannot be watertight)`, "");
  out.push("| Converter | Converted | Watertight | Flipped areas | Missing surface | Errors / timeouts |", "|---|--:|--:|--:|--:|--:|");
  for (const r of rows) out.push(`| ${L(r.id)} | ${r.converted} / ${r.n} | ${r.closed} / ${r.solidModels} | ${r.flippedModels} | ${r.missingModels} | ${r.failed} |`);
  out.push("", `**Speed** (${common.length} common models; ratio = geometric mean against occt-import-js, < 1 is faster)`, "");
  out.push("| Converter | First file, fresh page | ratio | Same file again | ratio | Peak memory (median) |", "|---|--:|--:|--:|--:|--:|");
  const skip = (r: Summary, cols: number) => `| ${L(r.id)} | ${Array(cols).fill("–").join(" | ")} |`;
  for (const r of rows) if (!r.common) out.push(skip(r, 5)); else out.push(`| ${L(r.id)} | ${num(r.coldTotal, 1)} s | ×${num(r.coldGeo, 2)} | ${num(r.warmTotal, 1)} s | ×${num(r.warmGeo, 2)} | ${num(r.rss)} MB |`);
  out.push("", `**Mesh** (${common.length} common models)`, "");
  out.push("| Converter | Triangles | ratio | Min angle p5 | < 20° | < 5° | Radius ratio | Deviation p99 / tol | Max deviation ≤ 1.25 tol |", "|---|--:|--:|--:|--:|--:|--:|--:|--:|");
  for (const r of rows) if (!r.common) out.push(skip(r, 8)); else out.push(`| ${L(r.id)} | ${num(r.tris)} | ×${num(r.trisRel, 2)} | ${num(r.minAngP5, 1)}° | ${pct(r.under20, 1)} | ${pct(r.under5, 1)} | ${num(r.q, 2)} | ${num(r.devP99, 2)} | ${pct(r.withinTol)} |`);
  const left = rows.filter(r => !r.common).map(r => L(r.id));
  if (left.length) out.push("", `– ${left.join(", ")}: converts fewer than half of the models, so it is left out of the speed and mesh comparison.`);
  out.push("");
  return out.join("\n");
}

function footprintTable(): string {
  const out = ["### Footprint", "", "What a web page downloads (JavaScript bundled and minified with esbuild, WebAssembly as shipped).", "",
    "| Library | Engine | Download (brotli) | gzip | Uncompressed | License |", "|---|---|--:|--:|--:|---|"];
  for (const id of ORDER) {
    const m = META[id];
    if (!m || (id.startsWith("meshstep") && id !== "meshstep")) continue;
    const s = size[m.sizeId];
    if (!s) continue;
    const mb = (n: number) => n < 1e6 ? `${num(n / 1e3)} KB` : `${num(n / 1e6, 1)} MB`;
    out.push(`| ${id === "meshstep" ? "meshStep" : m.label} ${versions[id] ?? ""} | ${m.engine} | ${mb(s.brotli)} | ${mb(s.gzip)} | ${mb(s.raw)} | ${m.license} |`);
  }
  return out.join("\n") + "\n";
}

function perModel(recs: Rec[]): string {
  const ids = ORDER.filter(id => recs.some(r => r.adapter === id));
  const models = [...new Set(recs.map(r => r.model))].sort();
  const out = ["<details><summary>Per model: time to first mesh (s) / watertight</summary>", "",
    `| Model | Closed in file | ${ids.map(id => META[id]?.label ?? id).join(" | ")} |`, `|---|:-:|${ids.map(() => "--:").join("|")}|`];
  for (const m of models) {
    const cells = ids.map(id => {
      const r = recs.find(x => x.model === m && x.adapter === id);
      if (!r) return "";
      if (r.status !== "ok") return r.status;
      const t = ((r.initMs + r.coldMs) / 1000).toFixed(2);
      const flag = !converted(r) ? " ⚠" : !solidInFile(m) ? "" : r.closed ? " ✓" : " ✗";
      return t + flag;
    });
    out.push(`| ${m.replace(/\|/g, "/")} | ${solidInFile(m) ? "yes" : "**no**"} | ${cells.join(" | ")} |`);
  }
  out.push("", "✓ watertight · ✗ converted but open · no mark: converted, file not closed (not counted) · ⚠ surface missing (> 1%), wrong scale or empty", "", "</details>", "");
  return out.join("\n");
}

// ---------------------------------------------------------------- chart (README)

function chart(rows: Summary[], common: number, solid: number): string {
  const ids = rows.map(r => r.id).filter(id => id !== "meshstep-par");
  const get = (id: string) => rows.find(r => r.id === id)!;
  const panels: { title: string; sub: string; val: (id: string) => number; fmt: (v: number) => string }[] = [
    { title: "Download size", sub: "MB, brotli · lower is better", val: id => (size[META[id]!.sizeId]?.brotli ?? NaN) / 1e6, fmt: v => v < 1 ? v.toFixed(2) : v.toFixed(1) },
    { title: "Time to mesh", sub: `s for ${common} models · lower is better`, val: id => get(id).coldTotal, fmt: v => v.toFixed(0) },
    { title: "Watertight", sub: `of ${solid} solid models · higher better`, val: id => get(id).closed, fmt: v => v.toFixed(0) },
    { title: "Sliver triangles", sub: "% under 20° · lower is better", val: id => 100 * get(id).under20, fmt: v => v.toFixed(0) + "%" },
  ];
  const W = 980, labelW = 200, pad = 16, rowH = 30, top = 64, panelW = (W - labelW - pad) / panels.length;
  const H = top + ids.length * rowH + 14;
  const esc = (s: string) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;");
  const parts: string[] = [];
  parts.push(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}" font-family="-apple-system, Segoe UI, Helvetica, Arial, sans-serif">`);
  parts.push(`<style>
  .bg{fill:#fcfcfb}.t1{fill:#0b0b0b}.t2{fill:#52514e}.grid{stroke:#e4e3df}.hi{fill:#2a78d6}.lo{fill:#b9b8b2}
  @media (prefers-color-scheme: dark){.bg{fill:#1a1a19}.t1{fill:#f0efec}.t2{fill:#c3c2b7}.grid{stroke:#3a3a37}.hi{fill:#3987e5}.lo{fill:#5d5c57}}
  text{font-size:13px}.h{font-size:14px;font-weight:600}.s{font-size:12px}
</style>`);
  parts.push(`<rect class="bg" width="${W}" height="${H}" rx="8"/>`);
  ids.forEach((id, i) => {
    const y = top + i * rowH + rowH / 2;
    const isMs = id.startsWith("meshstep");
    parts.push(`<text class="${isMs ? "t1" : "t2"}" x="${pad}" y="${y + 4}"${isMs ? ' font-weight="600"' : ""}>${esc(META[id]!.label)}</text>`);
  });
  panels.forEach((p, k) => {
    const x0 = labelW + k * panelW, barMax = panelW - 64;
    const vals = ids.map(p.val), max = Math.max(...vals.filter(Number.isFinite));
    parts.push(`<text class="t1 h" x="${x0}" y="24">${esc(p.title)}</text>`);
    parts.push(`<text class="t2 s" x="${x0}" y="42">${esc(p.sub)}</text>`);
    parts.push(`<line class="grid" x1="${x0}" x2="${x0}" y1="${top - 6}" y2="${H - 10}"/>`);
    ids.forEach((id, i) => {
      const v = vals[i]!, y = top + i * rowH + (rowH - 16) / 2;
      const w = Number.isFinite(v) && max > 0 ? Math.max(2, (v / max) * barMax) : 0;
      const cls = id.startsWith("meshstep") ? "hi" : "lo";
      // 4px rounded data end, square at the baseline
      if (w >= 8) parts.push(`<path class="${cls}" d="M${x0} ${y}h${w - 4}a4 4 0 0 1 4 4v8a4 4 0 0 1-4 4h-${w - 4}z"/>`);
      else if (w > 0) parts.push(`<rect class="${cls}" x="${x0}" y="${y}" width="${w}" height="16"/>`);
      parts.push(`<text class="t1 s" x="${x0 + w + 6}" y="${y + 12}">${Number.isFinite(v) ? p.fmt(v) : "–"}</text>`);
    });
  });
  parts.push("</svg>");
  return parts.join("\n");
}

// ---------------------------------------------------------------- main

const sections: string[] = [];
let chartDone = false;
for (const f of files) {
  const recs: Rec[] = readFileSync(f, "utf8").split("\n").filter(Boolean).map(l => JSON.parse(l));
  if (!recs.length) continue;
  const name = f.split("/").pop()!.replace(".jsonl", "");
  const [corpus, tier] = name.split("-");
  const title = corpus === "solo" ? `Speed on an idle machine (one conversion at a time), ${tier} tolerance`
    : `${corpus === "char" ? "Test corpus" : corpus === "abc" ? "ABC dataset sample" : corpus}, ${tier} tolerance`;
  sections.push(tables(recs, title));
  sections.push(perModel(recs));
  if (!chartDone && corpus === "char" && tier === "fine") {
    const { rows, common, models } = summarize(recs, "chart");
    writeFileSync(join(here, "..", "docs", "images", "bench.svg"), chart(rows, common.length, models.filter(solidInFile).length));
    chartDone = true;
  }
}

const md = `# meshStep benchmark: results

Generated by \`node bench/report.ts\` from the runs in \`bench/out/\`. How the numbers are made:
[bench/README.md](README.md).

${footprintTable()}
${sections.join("\n")}`;
writeFileSync(join(here, "RESULTS.md"), md);
console.error(`wrote bench/RESULTS.md${chartDone ? " and docs/images/bench.svg" : ""}`);
