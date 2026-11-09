// SPDX-License-Identifier: AGPL-3.0-only
// Per-record STEP parse recovery: one malformed record must not abort the file.
// cube.step is corrupted the way some ABC exporters write degenerate edges —
// `EDGE_CURVE('', #a, #b, #0,   );` (curve #0, empty trailing slot) — plus an unreadable record.
// A cube's edges are straight, so the bridged edge reproduces the clean mesh exactly; the import
// must stay watertight and report the repair as a "step-record-repaired" warning.
import { readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";
import { parseStep } from "../src/step/parser.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
let failures = 0;
const check = (label: string, cond: boolean, detail: string): void => {
  if (!cond) { failures++; console.log(`  ASSERT FAILED [${label}]: ${detail}`); }
};

const clean = readFileSync(join(root, "cube.step"), "utf8");
const broken = clean
  .replace("#59=EDGE_CURVE('',#51,#52,#27,.T.);", "#59=EDGE_CURVE('',#51,#52,#0,   );")
  .replace("DATA;", "DATA;\n#99999=BROKEN_RECORD(,( #1 ;");
check("fixture", broken !== clean && broken.includes("#0,   )") && broken.includes("#99999"), "cube.step layout changed — update the corruption");

// Parser: the edge is repaired (empty slot -> unset), the garbage record skipped, the rest intact.
const model = parseStep(broken);
check("parser", (model.repaired ?? []).includes(59), `repaired = ${JSON.stringify(model.repaired)}`);
check("parser", (model.dropped ?? []).includes(99999), `dropped = ${JSON.stringify(model.dropped)}`);
check("parser", model.entities.size === parseStep(clean).entities.size, "entity count differs from the clean file");
check("parser", parseStep(clean).repaired === undefined && parseStep(clean).dropped === undefined, "clean file reports repairs");

// Import: same mesh as the clean cube, watertight, only the repair warning.
const a = importStep(clean), b = importStep(broken);
check("import", b.mesh.indices.length === a.mesh.indices.length, `tris ${b.mesh.indices.length / 3} vs clean ${a.mesh.indices.length / 3}`);
check("import", b.diagnostics.openEdges === 0 && b.diagnostics.nonManifoldEdges === 0 && b.diagnostics.facesSkipped === 0,
  `open=${b.diagnostics.openEdges} nm=${b.diagnostics.nonManifoldEdges} skipped=${b.diagnostics.facesSkipped}`);
const codes = b.diagnostics.warnings.map((w) => w.code);
check("diagnostics", codes.length === 1 && codes[0] === "step-record-repaired", `warnings = ${JSON.stringify(codes)}`);
check("diagnostics", b.diagnostics.warnings[0]?.severity === "warning", "repair must be warning severity");
check("diagnostics", !b.diagnostics.ok && a.diagnostics.ok, "repaired import must clear ok; clean must not");

if (failures) { console.log(`parse-recovery: ${failures} assertion(s) failed`); process.exit(1); }
console.log("parse-recovery: all passed");
