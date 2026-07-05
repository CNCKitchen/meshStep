// SPDX-License-Identifier: AGPL-3.0-only
// Assemble <outdir>/report.html from gapcheck.json + images/ + converted/. If a
// notes.md exists in the outdir (hand-written analysis / next steps) it is rendered
// at the top. Standalone (regenerate after editing notes.md without re-running):
//   node test/gapcheck-report.ts out/gapcheck-newTestModels
import { readFileSync, writeFileSync, existsSync } from "node:fs";
import { join, basename } from "node:path";

interface Rec {
  file: string; status: string; reasons: string[];
  occ?: { tris: number; faces: number; area: number; volume: number; diag: number; ms: number } | null;
  ours?: {
    tris: number; facesTessellated: number; facesTotal: number; area: number; volume: number;
    boundaryEdges: number; nonmanifoldEdges: number; sliverPct: number; skipped: Record<string, number>; ms: number;
  } | null;
  deviation?: {
    diag: number; base: number; warn: number; fail: number;
    oursToOcc: { max: number; p99: number; mean: number };
    occToOurs: { max: number; p99: number; mean: number };
    missingAreaFrac: number; missingArea: number;
    worstOurFaces: { faceId: number; max: number; mean: number; n: number; bad: number; at: number[] }[];
    missingOccFaces: { face: number; max: number; badFrac: number; n: number; at: number[] }[];
  } | null;
  areaRatio?: number; volRatio?: number; scaleRatio?: number;
  analysis?: { cause: string; fix: string }[];
  openEdgeFaces?: { faceId: number; kind: string; openEdges: number }[];
  images?: string[]; stl?: string;
  wallMs: number; stderrTail?: string;
}

const esc = (s: string): string =>
  s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");

/** Tiny markdown subset for notes.md: #/##/### headings, - lists, **bold**, `code`. */
const mdToHtml = (md: string): string => {
  const inline = (s: string): string =>
    esc(s).replace(/\*\*(.+?)\*\*/g, "<strong>$1</strong>").replace(/`([^`]+)`/g, "<code>$1</code>");
  const out: string[] = [];
  let inList = false;
  for (const line of md.split(/\r?\n/)) {
    const h = line.match(/^(#{1,4})\s+(.*)/);
    const li = line.match(/^\s*-\s+(.*)/);
    if (inList && !li) { out.push("</ul>"); inList = false; }
    if (h) out.push(`<h${h[1]!.length + 1}>${inline(h[2]!)}</h${h[1]!.length + 1}>`);
    else if (li) { if (!inList) { out.push("<ul>"); inList = true; } out.push(`<li>${inline(li[1]!)}</li>`); }
    else if (line.trim()) out.push(`<p>${inline(line)}</p>`);
  }
  if (inList) out.push("</ul>");
  return out.join("\n");
};

const STATUS_COLOR: Record<string, string> = {
  OK: "#2e9e5b", WARN: "#d99a1f", FAIL: "#d4453a", NOREF: "#7d8695",
  SCALE: "#b0498f", EMPTY: "#b0498f", CRASH: "#8646c9", TIMEOUT: "#8646c9", OURS_ERR: "#8646c9",
};
const badge = (s: string): string =>
  `<span class="badge" style="background:${STATUS_COLOR[s] ?? "#666"}">${esc(s)}</span>`;

const fmt = (v: number | undefined | null, digits = 2, suffix = ""): string =>
  v === undefined || v === null || !Number.isFinite(v) ? "–" : v.toFixed(digits) + suffix;
const pctOf = (v: number | undefined, diag: number | undefined): string =>
  v === undefined || !diag ? "–" : `${((v / diag) * 100).toFixed(3)}%D`;

export function writeHtmlReport(outDir: string): string {
  const data = JSON.parse(readFileSync(join(outDir, "gapcheck.json"), "utf8")) as
    { dir: string; opts: Record<string, unknown>; results: Rec[] };
  const { results } = data;
  const counts = new Map<string, number>();
  for (const r of results) counts.set(r.status, (counts.get(r.status) ?? 0) + 1);
  const notesPath = join(outDir, "notes.md");
  const notes = existsSync(notesPath) ? mdToHtml(readFileSync(notesPath, "utf8")) : "";

  const anchor = (r: Rec): string => r.file.replace(/[^\w.-]+/g, "_");
  const flagged = results.filter((r) => r.status !== "OK");

  const overviewRows = results.map((r) => {
    const d = r.deviation;
    const wt = r.ours ? (r.ours.boundaryEdges === 0 && r.ours.nonmanifoldEdges === 0 ? "closed" : `B${r.ours.boundaryEdges}/NM${r.ours.nonmanifoldEdges}`) : "–";
    const name = r.status === "OK" ? esc(r.file) : `<a href="#${anchor(r)}">${esc(r.file)}</a>`;
    return `<tr>
      <td>${badge(r.status)}</td><td class="name">${name}</td>
      <td class="num">${r.ours ? r.ours.tris.toLocaleString() : "–"}</td>
      <td>${wt}</td>
      <td class="num">${pctOf(d?.oursToOcc.max, d?.diag)}</td>
      <td class="num">${pctOf(d?.occToOurs.max, d?.diag)}</td>
      <td class="num">${d ? fmt(d.missingAreaFrac * 100, 2, "%") : "–"}</td>
      <td class="num">${r.areaRatio ? fmt((r.areaRatio - 1) * 100, 2, "%") : "–"}</td>
      <td class="num">${r.volRatio ? fmt((r.volRatio - 1) * 100, 2, "%") : "–"}</td>
      <td class="num">${fmt(r.wallMs / 1000, 1, "s")}</td>
      <td>${r.stl ? `<a href="converted/${encodeURIComponent(r.stl)}">stl</a>` : "–"}</td>
    </tr>`;
  }).join("\n");

  const sections = flagged.map((r) => {
    const d = r.deviation;
    const imgs = (r.images ?? []).map((i) =>
      `<figure><a href="images/${encodeURIComponent(i)}"><img src="images/${encodeURIComponent(i)}" loading="lazy"></a>
       <figcaption>${i.endsWith(".zoom.png") ? "zoom on defect region" : i.endsWith(".alt.png") ? "opposite view" : "full view"}</figcaption></figure>`).join("\n");
    const causeRows = (r.analysis ?? []).map((a) =>
      `<tr><td>${esc(a.cause)}</td><td>${esc(a.fix)}</td></tr>`).join("\n");
    const worst = (d?.worstOurFaces ?? []).filter((f) => f.bad > 0).map((f) =>
      `<tr><td>#${f.faceId}</td><td>${fmt(f.max, 4)}mm (${pctOf(f.max, d?.diag)})</td><td>${f.bad}/${f.n}</td><td>[${f.at.join(", ")}]</td></tr>`).join("\n");
    const missing = (d?.missingOccFaces ?? []).map((f) =>
      `<tr><td>#${f.face}</td><td>${(f.badFrac * 100).toFixed(0)}%</td><td>${fmt(f.max, 4)}mm</td><td>[${f.at.join(", ")}]</td></tr>`).join("\n");
    const openFaces = (r.openEdgeFaces ?? []).map((f) =>
      `<tr><td>#${f.faceId}</td><td>${esc(f.kind)}</td><td>${f.openEdges}</td></tr>`).join("\n");
    const o = r.ours;
    return `<section id="${anchor(r)}">
    <h3>${badge(r.status)} ${esc(r.file)} ${r.stl ? `&nbsp;<a class="dl" href="converted/${encodeURIComponent(r.stl)}">download STL</a>` : ""}</h3>
    <ul class="reasons">${r.reasons.map((x) => `<li>${esc(x)}</li>`).join("")}</ul>
    ${causeRows ? `<table class="kv"><thead><tr><th>probable cause</th><th>can we fix it?</th></tr></thead><tbody>${causeRows}</tbody></table>` : ""}
    <div class="figs">${imgs || "<p class='dim'>no picture (conversion produced no mesh)</p>"}</div>
    ${o ? `<table class="kv"><thead><tr><th colspan="2">stats</th></tr></thead><tbody>
      <tr><td>triangles (ours / OCC)</td><td>${o.tris.toLocaleString()} / ${r.occ ? r.occ.tris.toLocaleString() : "–"}</td></tr>
      <tr><td>faces tessellated</td><td>${o.facesTessellated}/${o.facesTotal}${Object.keys(o.skipped).length ? ` (skipped: ${esc(Object.entries(o.skipped).map(([k, v]) => `${k}:${v}`).join(", "))})` : ""}</td></tr>
      <tr><td>watertight</td><td>${o.boundaryEdges} boundary / ${o.nonmanifoldEdges} non-manifold edges</td></tr>
      <tr><td>deviation ours→OCC max / p99</td><td>${fmt(d?.oursToOcc.max, 4)}mm (${pctOf(d?.oursToOcc.max, d?.diag)}) / ${fmt(d?.oursToOcc.p99, 4)}mm</td></tr>
      <tr><td>deviation OCC→ours max / p99</td><td>${fmt(d?.occToOurs.max, 4)}mm (${pctOf(d?.occToOurs.max, d?.diag)}) / ${fmt(d?.occToOurs.p99, 4)}mm</td></tr>
      <tr><td>missing area</td><td>${d ? `${fmt(d.missingArea, 1)}mm² (${fmt(d.missingAreaFrac * 100, 2)}%)` : "–"}</td></tr>
      <tr><td>area / volume vs OCC</td><td>${r.areaRatio ? fmt((r.areaRatio - 1) * 100, 2, "%") : "–"} / ${r.volRatio ? fmt((r.volRatio - 1) * 100, 2, "%") : "–"}</td></tr>
      <tr><td>sliver % / time</td><td>${fmt(o.sliverPct, 1, "%")} / ours ${fmt(o.ms / 1000, 1)}s, OCC ${r.occ ? fmt(r.occ.ms / 1000, 1) : "–"}s</td></tr>
    </tbody></table>` : ""}
    ${worst ? `<table class="kv"><thead><tr><th>deviating STEP face</th><th>max dev</th><th>bad samples</th><th>near</th></tr></thead><tbody>${worst}</tbody></table>` : ""}
    ${missing ? `<table class="kv"><thead><tr><th>missing OCC face</th><th>bad frac</th><th>max dev</th><th>near</th></tr></thead><tbody>${missing}</tbody></table>` : ""}
    ${openFaces ? `<table class="kv"><thead><tr><th>face owning open edges</th><th>surface kind</th><th>open edges</th></tr></thead><tbody>${openFaces}</tbody></table>` : ""}
    ${r.stderrTail ? `<pre>${esc(r.stderrTail)}</pre>` : ""}
    </section>`;
  }).join("\n");

  const tiles = ["FAIL", "WARN", "CRASH", "TIMEOUT", "SCALE", "EMPTY", "OURS_ERR", "NOREF", "OK"]
    .filter((s) => counts.has(s))
    .map((s) => `<div class="tile" style="border-color:${STATUS_COLOR[s]}"><div class="n">${counts.get(s)}</div><div class="l">${s}</div></div>`)
    .join("\n");

  const html = `<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>gapcheck — ${esc(data.dir)}</title>
<style>
  :root { color-scheme: light; }
  * { box-sizing: border-box; }
  body { font: 15px/1.5 system-ui, "Segoe UI", sans-serif; color: #1c2430; margin: 0; background: #f4f5f7; }
  main { max-width: 1180px; margin: 0 auto; padding: 24px 28px 80px; }
  h1 { font-size: 26px; margin: 8px 0 2px; } h2 { margin-top: 40px; } h3 { margin: 0 0 8px; font-size: 18px; }
  .sub { color: #5b6675; margin-bottom: 20px; }
  .tiles { display: flex; gap: 12px; flex-wrap: wrap; margin: 18px 0; }
  .tile { background: #fff; border: 1px solid #ddd; border-top: 4px solid; border-radius: 8px; padding: 10px 22px; text-align: center; }
  .tile .n { font-size: 26px; font-weight: 700; } .tile .l { font-size: 12px; color: #5b6675; letter-spacing: .05em; }
  .badge { color: #fff; font-size: 11px; font-weight: 700; padding: 2px 8px; border-radius: 10px; letter-spacing: .04em; vertical-align: 2px; }
  table { border-collapse: collapse; background: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 1px 2px rgba(20,30,50,.08); }
  .overview { width: 100%; font-size: 13.5px; }
  th, td { padding: 6px 10px; text-align: left; border-bottom: 1px solid #eceef1; }
  th { background: #e9ecf0; font-size: 12px; letter-spacing: .04em; color: #444e5c; }
  td.num { text-align: right; font-variant-numeric: tabular-nums; white-space: nowrap; }
  td.name { max-width: 320px; overflow-wrap: anywhere; }
  tr:hover td { background: #f7f9fb; }
  section { background: #fff; border-radius: 10px; padding: 18px 22px; margin: 26px 0; box-shadow: 0 1px 3px rgba(20,30,50,.1); }
  .reasons li { margin: 2px 0; }
  .kv { margin: 12px 0; font-size: 13.5px; }
  .figs { display: flex; gap: 14px; flex-wrap: wrap; margin: 12px 0; }
  figure { margin: 0; } figcaption { font-size: 12px; color: #5b6675; text-align: center; padding-top: 4px; }
  .figs img { width: 460px; max-width: 100%; border-radius: 6px; display: block; }
  .legend { font-size: 13px; color: #5b6675; margin: 6px 0 0; }
  .legend b { padding: 0 6px; border-radius: 4px; color: #fff; font-weight: 600; }
  .dl { font-size: 13px; }
  .dim { color: #8a93a1; }
  .notes { background: #fff; border-radius: 10px; padding: 6px 22px 14px; box-shadow: 0 1px 3px rgba(20,30,50,.1); }
  pre { background: #f0f2f5; padding: 10px; border-radius: 6px; overflow-x: auto; font-size: 12.5px; }
  code { background: #eef1f4; padding: 1px 5px; border-radius: 4px; font-size: .92em; }
  a { color: #1a66c2; }
</style></head><body><main>
<h1>gapcheck report — <code>${esc(data.dir)}</code></h1>
<div class="sub">${results.length} models · meshStep vs OpenCASCADE (occt-import-js) ·
surfDev ${String(data.opts.surfDevRel)}·D + OCC defl ${String(data.opts.occDeflRel)}·D ·
${String(data.opts.samples)} samples/direction · fail at ${String(data.opts.failFactor)}× combined tolerance</div>
<div class="tiles">${tiles}</div>
<p class="legend">picture legend: <b style="background:#d4453a">red</b> open edges ·
<b style="background:#c400c4">magenta</b> non-manifold edges ·
<b style="background:#e08830">orange</b> faces off the reference surface ·
<b style="background:#3f7fd4">blue</b> OCC faces missing from our mesh</p>
${notes ? `<h2>Analysis &amp; next steps</h2><div class="notes">${notes}</div>` : ""}
<h2>All models</h2>
<div style="overflow-x:auto"><table class="overview"><thead><tr>
<th>status</th><th>model</th><th>tris</th><th>watertight</th><th>ours→OCC</th><th>OCC→ours</th>
<th>miss. area</th><th>area Δ</th><th>vol Δ</th><th>time</th><th>mesh</th>
</tr></thead><tbody>${overviewRows}</tbody></table></div>
<h2>Flagged models</h2>
${sections || "<p>none 🎉</p>"}
</main></body></html>`;

  const outPath = join(outDir, "report.html");
  writeFileSync(outPath, html);
  return outPath;
}

// standalone: regenerate a report (e.g. after editing notes.md)
if (process.argv[1] && basename(process.argv[1]) === "gapcheck-report.ts") {
  const dir = process.argv[2];
  if (!dir) { console.error("usage: node test/gapcheck-report.ts <outdir>"); process.exit(2); }
  console.log(writeHtmlReport(dir));
}
