// SPDX-License-Identifier: AGPL-3.0-only
// Post-hoc cross-tabs over out/abc-run/results.jsonl for the survey report.
import { readFileSync, statSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const scanDir = join(root, "abc_0000_step_v00");
const recs = readFileSync(join(root, "out/abc-run/results.jsonl"), "utf8")
  .split("\n").filter(Boolean).map((l) => JSON.parse(l));

const N = recs.length;
const by = (pred: (r: any) => boolean) => recs.filter(pred);

// higher-level classes
const seam = by((r) => r.bucket?.startsWith("seam-leak:"));
const unsupported = by((r) => r.bucket?.startsWith("unsupported-surface:"));
const untri = by((r) => r.bucket === "untriangulated-face");
const timeout = by((r) => r.status === "TIMEOUT");
const nm = by((r) => r.bucket === "nonmanifold");
const empty = by((r) => r.status === "EMPTY");
const errcrash = by((r) => r.status === "ERR" || r.status === "CRASH");

console.log("=== higher-level classes ===");
for (const [name, arr] of [["seam-leak (watertight leak, all faces meshed)", seam], ["unsupported surface (missing evaluator)", unsupported], ["untriangulated (CDT robustness)", untri], ["timeout (perf/hang)", timeout], ["nonmanifold", nm], ["empty", empty], ["error/crash", errcrash]] as const) {
  console.log(`${String(arr.length).padStart(5)}  ${(100 * arr.length / N).toFixed(2).padStart(5)}%  ${name}`);
}

// timeout size split (genuine hang vs slow-large)
console.log("\n=== timeouts by source file size ===");
let bigTO = 0, midTO = 0, smallTO = 0;
for (const r of timeout) {
  try {
    const sz = statSync(join(scanDir, r.file)).size;
    if (sz > 3e6) bigTO++; else if (sz > 5e5) midTO++; else smallTO++;
  } catch { /* skip */ }
}
console.log(`>3MB   (large, likely just slow): ${bigTO}`);
console.log(`0.5-3MB (medium)               : ${midTO}`);
console.log(`<0.5MB (small, genuine hang)   : ${smallTO}`);

// near-miss: open-edge count distribution for seam-leak fails
console.log("\n=== seam-leak open-edge counts (near-miss = few edges) ===");
const buckets = { "1-4": 0, "5-20": 0, "21-100": 0, "101-1000": 0, ">1000": 0 };
for (const r of seam) {
  const o = r.open ?? 0;
  if (o <= 4) buckets["1-4"]++; else if (o <= 20) buckets["5-20"]++; else if (o <= 100) buckets["21-100"]++; else if (o <= 1000) buckets["101-1000"]++; else buckets[">1000"]++;
}
for (const [k, v] of Object.entries(buckets)) console.log(`  open ${k.padEnd(10)}: ${v}`);

// how many FAILs are "near watertight" (<=20 open, 0 nm, 0 skipped) -> high ROI
const nearWT = by((r) => r.status === "FAIL" && (r.open ?? 0) > 0 && (r.open ?? 0) <= 20 && (r.nm ?? 0) === 0 && !r.skipped);
console.log(`\nnear-watertight fails (<=20 open edges, no nm, no skipped): ${nearWT.length}`);

// producer distribution
console.log("\n=== originating_system (top 15) ===");
const prod = new Map<string, { total: number; fail: number }>();
for (const r of recs) {
  const p = r.producer ?? "?";
  const e = prod.get(p) ?? { total: 0, fail: 0 };
  e.total++; if (r.status !== "PASS") e.fail++; prod.set(p, e);
}
for (const [p, e] of [...prod.entries()].sort((a, b) => b[1].total - a[1].total).slice(0, 15))
  console.log(`  ${String(e.total).padStart(5)}  fail ${(100 * e.fail / e.total).toFixed(0).padStart(3)}%  ${p}`);

// timing stats on PASS models
const passMs = by((r) => r.status === "PASS").map((r) => r.ms ?? 0).sort((a, b) => a - b);
const q = (p: number) => passMs[Math.floor(p * passMs.length)] ?? 0;
console.log(`\n=== PASS import time (ms): median ${q(0.5)}  p90 ${q(0.9)}  p99 ${q(0.99)}  max ${passMs[passMs.length - 1]} ===`);

// total triangles produced (scale of output)
let totTris = 0; for (const r of recs) totTris += r.tris ?? 0;
console.log(`total triangles produced across all PASS+FAIL: ${(totTris / 1e6).toFixed(1)}M`);
