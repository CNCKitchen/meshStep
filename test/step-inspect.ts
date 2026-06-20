// SPDX-License-Identifier: AGPL-3.0-only
// Parses the test STEP files and prints an inventory of geometry-relevant entity types,
// plus the detected length unit. Proves the tokenizer/parser/units stack.
import { readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { parseStep } from "../src/step/parser.ts";
import { Table } from "../src/step/entities.ts";
import { detectUnits } from "../src/step/units.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const files = [
  "cube.step", "cylinder.step", "cone.step", "sphere.step",
  "cylinderWithHole.step", "roundedCube.step", "splineThing.step", "everything.step",
  "MicHolder_Inserts.step",
];

const RELEVANT =
  /FACE|PLANE|CYLINDR|CONIC|SPHERIC|TOROID|B_SPLINE|EDGE|CLOSED_SHELL|OPEN_SHELL|MANIFOLD|VERTEX|CIRCLE|^LINE$|ELLIPSE|SHELL_BASED/;

for (const name of files) {
  let src: string;
  try {
    src = readFileSync(join(root, name), "utf8");
  } catch {
    console.log(`\n=== ${name} === (missing, skipped)`);
    continue;
  }
  const model = parseStep(src);
  const table = new Table(model);
  const units = detectUnits(table);
  console.log(`\n=== ${name} ===`);
  console.log(`  entities: ${model.entities.size}   unit: ${units.label} (mmPerUnit=${units.mmPerUnit})`);
  const hist = [...table.histogram()].sort((a, b) => b[1] - a[1]);
  const relevant = hist.filter(([t]) => RELEVANT.test(t));
  for (const [t, c] of relevant) console.log(`    ${String(c).padStart(5)}  ${t}`);
}
