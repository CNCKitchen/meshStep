// Aggregate out/abc-run7/seamleak-census.jsonl -> mechanism clusters for the rec-④ design.
import { readFileSync } from "node:fs";
const recs = readFileSync(process.env.SEAM_OUT ?? "out/abc-run7/seamleak-census.jsonl", "utf8")
  .split(/\r?\n/).filter(Boolean).map(JSON.parse);
console.log("models:", recs.length, " errors:", recs.filter((r) => r.err).length,
  " now-clean:", recs.filter((r) => !r.err && r.open === 0).length);
for (const r of recs.filter((r) => r.err)) console.log("  ERR", r.file, String(r.err).slice(0, 80));

const leaky = recs.filter((r) => !r.err && r.open > 0);
const sum = (obj, key) => { for (const [k, v] of Object.entries(key ?? {})) obj[k] = (obj[k] ?? 0) + v; };
const totC = {}, totM = {}, totK = {};
for (const r of leaky) { sum(totC, r.perClass); sum(totM, r.perMesher); sum(totK, r.perKind); }
const show = (name, o) => {
  console.log(`\n=== open edges by ${name} (corpus total) ===`);
  for (const [k, v] of Object.entries(o).sort((a, b) => b[1] - a[1])) console.log(`  ${String(v).padStart(7)}  ${k}`);
};
show("class", totC); show("mesher", totM); show("surface kind", totK);

// Per-model DOMINANT signature (mesher × class of the majority of its open edges) — models whose
// leak is one mechanism are coherently fixable.
const domSig = {};
for (const r of leaky) {
  const top = (o) => Object.entries(o ?? {}).sort((a, b) => b[1] - a[1])[0];
  const tc = top(r.perClass), tm = top(r.perMesher);
  const frac = tc[1] / r.open;
  const sig = `${tm[0]} × ${tc[0]}${frac >= 0.8 ? "" : " (mixed)"}`;
  (domSig[sig] ??= []).push(r);
}
console.log("\n=== models by dominant (mesher × class) signature ===");
for (const [sig, rs] of Object.entries(domSig).sort((a, b) => b[1].length - a[1].length)) {
  console.log(`  ${String(rs.length).padStart(5)}  ${sig}`);
  for (const r of rs.slice(0, 3)) {
    const tf = (r.topFaces ?? [])[0];
    console.log(`           e.g. ${r.file.split("\\")[0]} open=${r.open}${tf ? ` top fid=${tf.fid} ${tf.kind} n=${tf.n}` : ""}`);
  }
}
