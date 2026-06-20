// SPDX-License-Identifier: AGPL-3.0-only
// Validates the STL reader against the reference files in the repo root, and reports
// triangle counts + bounding boxes. This is the baseline for the Hausdorff harness.
import { readFileSync, readdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import { readSTL, isBinarySTL, bboxOfSoup } from "../src/io/stl.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const stls = readdirSync(root).filter((f) => f.toLowerCase().endsWith(".stl")).sort();

console.log(`Found ${stls.length} reference STL(s) in ${root}\n`);
console.log("file".padEnd(22), "fmt".padEnd(7), "tris".padStart(8), "  bbox diagonal (mm)");
console.log("-".repeat(60));

for (const f of stls) {
  const bytes = new Uint8Array(readFileSync(join(root, f)));
  const fmt = isBinarySTL(bytes) ? "binary" : "ascii";
  const soup = readSTL(bytes);
  const bb = bboxOfSoup(soup);
  console.log(
    f.padEnd(22),
    fmt.padEnd(7),
    String(soup.triangleCount).padStart(8),
    "  " + bb.diagonal.toFixed(4),
  );
}
