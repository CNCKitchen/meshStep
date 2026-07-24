// SPDX-License-Identifier: AGPL-3.0-only
// Publish gate: the exported VERSION constant must match package.json. If this fails,
// run `node scripts/sync-version.mjs` (the npm "version" lifecycle does it automatically).
import { readFileSync } from "node:fs";
import { VERSION } from "../src/index.ts";

const pkg = JSON.parse(readFileSync(new URL("../package.json", import.meta.url), "utf8")) as { version: string };
if (VERSION !== pkg.version) {
  console.error(`FAIL: exported VERSION "${VERSION}" != package.json "${pkg.version}" — run node scripts/sync-version.mjs`);
  process.exit(1);
}
console.log(`OK: VERSION ${VERSION} matches package.json`);
