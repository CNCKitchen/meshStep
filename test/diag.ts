// SPDX-License-Identifier: AGPL-3.0-only
// Import-diagnostics check: clean models must report ok with zero counters and zero warnings;
// known-defective models must report ok=false with openEdges matching an independent recount.
// The recount (check-all.ts's edge-incidence audit) is the ground truth diagnostics must agree with.
import { readFileSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");

function recount(m: { positions: Float64Array; indices: Uint32Array }): { open: number; nonman: number } {
  const inc = new Map<number, number>();
  const K = 0x4000000;
  const nt = m.indices.length / 3;
  for (let t = 0; t < nt; t++) for (let e = 0; e < 3; e++) {
    const a = m.indices[t * 3 + e]!, b = m.indices[t * 3 + (e + 1) % 3]!;
    const k = a < b ? a * K + b : b * K + a;
    inc.set(k, (inc.get(k) ?? 0) + 1);
  }
  let open = 0, nonman = 0;
  for (const c of inc.values()) { if (c === 1) open++; else if (c > 2) nonman++; }
  return { open, nonman };
}

let failures = 0;
const check = (label: string, cond: boolean, detail: string): void => {
  if (!cond) { failures++; console.log(`  ASSERT FAILED [${label}]: ${detail}`); }
};

const cases: { file: string; expectOk: boolean }[] = [
  { file: "cube.step", expectOk: true },
  { file: "cylinder.step", expectOk: true },
  { file: "roundedCube.step", expectOk: true },
  // Known residual defect (char baseline 2026-07-07: open=405 at default options) — tangent
  // letter engravings whose trim loops genuinely overlap; must surface as ok=false.
  { file: join("newTestModels", "Ontos_V2.step"), expectOk: false },
];

for (const { file, expectOk } of cases) {
  const sp = join(root, file);
  if (!existsSync(sp)) { console.log(`${file}: (no step, skipped)`); continue; }
  const res = importStep(readFileSync(sp, "utf8"), { remesh: false });
  const d = res.diagnostics;
  const truth = recount(res.mesh);
  console.log(`${file}: ok=${d.ok} open=${d.openEdges} nonman=${d.nonManifoldEdges} dropped=${d.facesDropped} skipped=${d.facesSkipped} warnings=${d.warnings.length}`);
  for (const w of d.warnings.slice(0, 5)) console.log(`  [${w.severity}] ${w.code}${w.faceId !== undefined ? ` fid=${w.faceId}` : ""}: ${w.detail}`);
  // No open-shell bodies in these files, so the diagnostics audit must match the raw recount.
  check(file, res.openSolids.length === 0, `unexpected open-shell solids: ${res.openSolids}`);
  check(file, d.openEdges === truth.open, `openEdges=${d.openEdges} but independent recount=${truth.open}`);
  check(file, d.nonManifoldEdges === truth.nonman, `nonManifoldEdges=${d.nonManifoldEdges} but independent recount=${truth.nonman}`);
  check(file, d.ok === expectOk, `expected ok=${expectOk}`);
  if (expectOk) {
    check(file, d.warnings.length === 0, `expected no warnings, got ${JSON.stringify(d.warnings)}`);
    check(file, d.facesDropped === 0 && d.facesSkipped === 0, `dropped=${d.facesDropped} skipped=${d.facesSkipped}`);
  } else {
    check(file, d.openEdges > 0 || d.warnings.length > 0, "defective model reported neither edge defects nor warnings");
  }
  // ok must be derivable from the exposed fields — a consumer-facing invariant.
  const derived = d.openEdges === 0 && d.nonManifoldEdges === 0 && d.facesDropped === 0 && d.facesSkipped === 0 && d.warnings.length === 0;
  check(file, d.ok === derived, `ok=${d.ok} inconsistent with fields (derived=${derived})`);
}

if (failures > 0) { console.log(`\nFAIL: ${failures} assertion(s)`); process.exit(1); }
console.log("\nPASS");
