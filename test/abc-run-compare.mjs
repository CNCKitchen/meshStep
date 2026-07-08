// Compare two abc-batch runs: status totals, per-file transitions, bucket movement, near-miss count.
//   node test/abc-run-compare.mjs out/abc-run2 out/abc-run4
import { readFileSync } from "node:fs";
import { join } from "node:path";

const [dirA, dirB] = [process.argv[2] ?? "out/abc-run2", process.argv[3] ?? "out/abc-run4"];
const load = (d) => readFileSync(join(d, "results.jsonl"), "utf8").split(/\r?\n/).filter(Boolean).map(JSON.parse);
const A = load(dirA), B = load(dirB);
const mA = new Map(A.map((r) => [r.file, r]));

const stat = (rs) => { const s = {}; for (const r of rs) s[r.status] = (s[r.status] ?? 0) + 1; return s; };
console.log(`${dirA}: ${JSON.stringify(stat(A))}`);
console.log(`${dirB}: ${JSON.stringify(stat(B))}`);
const passA = A.filter((r) => r.status === "PASS").length, passB = B.filter((r) => r.status === "PASS").length;
console.log(`PASS: ${passA} (${(passA / A.length * 100).toFixed(2)}%) -> ${passB} (${(passB / B.length * 100).toFixed(2)}%)  net ${passB - passA >= 0 ? "+" : ""}${passB - passA}`);

let fixed = 0, regressed = 0;
const fixBy = {}, regList = [];
for (const r of B) {
  const a = mA.get(r.file);
  if (!a) continue;
  const ap = a.status === "PASS", bp = r.status === "PASS";
  if (!ap && bp) { fixed++; fixBy[a.bucket ?? a.status] = (fixBy[a.bucket ?? a.status] ?? 0) + 1; }
  else if (ap && !bp) { regressed++; regList.push(`${r.file}  [${r.bucket ?? r.status} open=${r.open ?? "?"} nm=${r.nm ?? "?"} ms=${a.ms}]`); }
}
console.log(`\nfixed (nonPASS->PASS): ${fixed}`);
for (const [k, v] of Object.entries(fixBy).sort((x, y) => y[1] - x[1])) console.log(`  ${String(v).padStart(4)}  ${k}`);
console.log(`regressed (PASS->nonPASS): ${regressed}`);
for (const r of regList) console.log(`  REG ${r}`);

const buckets = (rs) => { const s = {}; for (const r of rs) { if (r.status === "PASS") continue; const k = r.bucket?.startsWith("seam-leak") ? "seam-leak" : (r.bucket ?? r.status); s[k] = (s[k] ?? 0) + 1; } return s; };
console.log(`\nnon-PASS classes ${dirA}: ${JSON.stringify(buckets(A))}`);
console.log(`non-PASS classes ${dirB}: ${JSON.stringify(buckets(B))}`);
const near = (rs) => rs.filter((r) => r.status === "FAIL" && (r.open ?? 0) > 0 && (r.open ?? 0) <= 20 && (r.nm ?? 0) === 0 && !r.skipped).length;
console.log(`near-watertight fails (<=20 open, 0 nm, no skipped): ${near(A)} -> ${near(B)}`);
const tri = (rs) => rs.reduce((s, r) => s + (r.tris ?? 0), 0);
console.log(`triangles: ${(tri(A) / 1e6).toFixed(0)}M -> ${(tri(B) / 1e6).toFixed(0)}M`);
const cpu = (rs) => rs.reduce((s, r) => s + (r.ms ?? 0), 0);
console.log(`total worker CPU: ${(cpu(A) / 1000).toFixed(0)}s -> ${(cpu(B) / 1000).toFixed(0)}s`);
