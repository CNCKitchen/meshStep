// Aggregate out/abc-run4/untri-census.jsonl -> failure clusters for the robust-CDT design.
import { readFileSync } from "node:fs";
const recs = readFileSync("out/abc-run4/untri-census.jsonl", "utf8").split(/\r?\n/).filter(Boolean).map(JSON.parse);
console.log("models:", recs.length, " errors:", recs.filter((r) => r.err).length);
for (const r of recs.filter((r) => r.err)) console.log("  ERR", r.file, String(r.err).slice(0, 80));

const ok = recs.filter((r) => !r.err);
const faces = ok.flatMap((r) => (r.faces ?? []).map((f) => ({ ...f, file: r.file })));
console.log("untriangulated faces total:", faces.length, "across", ok.filter((r) => r.untriFaces > 0).length, "models");
// models whose tessellation now has 0 untri faces (fixed since run4 by the fills? shouldn't happen — fills don't affect skip)
console.log("models with 0 untri faces now:", ok.filter((r) => r.untriFaces === 0).length);

const kindOf = (k) => /complex|spline/i.test(k) ? "B_SPLINE" : k.replace("_SURFACE", "").replace("SURFACE_OF_", "");
// cluster: kind × loop-signature × periodicity
const clusters = new Map();
for (const f of faces) {
  const nl = f.loops.length;
  const sig = nl === 1 ? `1loop(${f.loops[0] <= 4 ? "small" : "big"})` : nl <= 3 ? `${nl}loops` : "many-loops";
  const key = `${kindOf(f.kind)} ${sig} ${f.per || "-"}${f.sole ? " sole" : ""}`;
  const c = clusters.get(key) ?? { n: 0, models: new Set(), ex: [] };
  c.n++; c.models.add(f.file);
  if (c.ex.length < 3) c.ex.push(`${f.file.split(/[\\/]/)[0]}#${f.fid} loops=[${f.loops.join(",")}]`);
  clusters.set(key, c);
}
console.log("\n=== face clusters (kind × loops × periodicity) ===");
for (const [k, c] of [...clusters].sort((a, b) => b[1].models.size - a[1].models.size)) {
  console.log(`${String(c.n).padStart(5)} faces / ${String(c.models.size).padStart(3)} models  ${k}`);
  for (const e of c.ex) console.log(`         e.g. ${e}`);
}

// per-model dominant kind (which cluster "owns" each model's FAIL)
const modKind = new Map();
for (const f of faces) {
  const m = modKind.get(f.file) ?? {};
  const k = kindOf(f.kind) + (f.loops.length > 1 ? "/multi" : "/single");
  m[k] = (m[k] ?? 0) + 1;
  modKind.set(f.file, m);
}
const domCount = {};
for (const [, m] of modKind) {
  const dom = Object.entries(m).sort((a, b) => b[1] - a[1])[0][0];
  domCount[dom] = (domCount[dom] ?? 0) + 1;
}
console.log("\n=== models by dominant untri-face class ===");
for (const [k, v] of Object.entries(domCount).sort((a, b) => b[1] - a[1])) console.log(`  ${String(v).padStart(4)}  ${k}`);
