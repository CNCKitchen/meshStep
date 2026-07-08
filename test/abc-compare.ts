// SPDX-License-Identifier: AGPL-3.0-only
// New-run higher-level classes, near-miss + timeout split (for the improvement report).
import { readFileSync, statSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const scanDir = join(root, "abc_0000_step_v00");
const B = readFileSync(join(root, "out/abc-run2/results.jsonl"), "utf8").split("\n").filter(Boolean).map((l) => JSON.parse(l));
const N = B.length;
const cls: Record<string, number> = { seam: 0, untri: 0, timeout: 0, nm: 0, empty: 0, errcrash: 0 };
for (const r of B) {
  if (r.bucket?.startsWith("seam-leak:")) cls.seam++;
  else if (r.bucket === "untriangulated-face") cls.untri++;
  else if (r.status === "TIMEOUT") cls.timeout++;
  else if (r.bucket === "nonmanifold") cls.nm++;
  else if (r.status === "EMPTY") cls.empty++;
  else if (r.status === "ERR" || r.status === "CRASH") cls.errcrash++;
}
console.log("=== NEW RUN higher-level non-PASS classes ===");
for (const [k, v] of Object.entries(cls)) console.log("  " + k.padEnd(10), String(v).padStart(4), (100 * v / N).toFixed(2) + "%");
console.log("  TOTAL non-PASS", N - B.filter((r) => r.status === "PASS").length);

const seam = B.filter((r) => r.bucket?.startsWith("seam-leak:"));
const nb: Record<string, number> = { "1-4": 0, "5-20": 0, "21-100": 0, "101-1000": 0, ">1000": 0 };
for (const r of seam) { const o = r.open || 0; if (o <= 4) nb["1-4"]++; else if (o <= 20) nb["5-20"]++; else if (o <= 100) nb["21-100"]++; else if (o <= 1000) nb["101-1000"]++; else nb[">1000"]++; }
console.log("\n=== seam-leak open-edge counts ===", JSON.stringify(nb));
const nearWT = B.filter((r) => r.status === "FAIL" && (r.open || 0) > 0 && (r.open || 0) <= 20 && (r.nm || 0) === 0 && !r.skipped).length;
console.log("near-watertight fails (<=20 open, no nm, no skipped):", nearWT);

const to = B.filter((r) => r.status === "TIMEOUT");
let big = 0, mid = 0, small = 0;
for (const r of to) { try { const sz = statSync(join(scanDir, r.file)).size; if (sz > 3e6) big++; else if (sz > 5e5) mid++; else small++; } catch { /**/ } }
console.log("\n=== timeouts by size: >3MB(slow)=" + big, "0.5-3MB=" + mid, "<0.5MB(hang)=" + small, " total=" + to.length);

// where did the 267 baseline unsupported-SoR models go?
const A = readFileSync(join(root, "out/abc-run/results.jsonl"), "utf8").split("\n").filter(Boolean).map((l) => JSON.parse(l));
const mapB = new Map(B.map((r) => [r.file, r]));
const sorModels = A.filter((r) => r.bucket === "unsupported-surface:SURFACE_OF_REVOLUTION");
const dest: Record<string, number> = {};
for (const r of sorModels) { const nb2 = mapB.get(r.file); const k = nb2 ? (nb2.status === "PASS" ? "PASS" : nb2.bucket || nb2.status) : "?"; dest[k] = (dest[k] || 0) + 1; }
console.log("\n=== where the 267 baseline unsupported-SoR models are NOW ===");
for (const [k, v] of Object.entries(dest).sort((a, b) => b[1] - a[1])) console.log("  " + String(v).padStart(4), k);
