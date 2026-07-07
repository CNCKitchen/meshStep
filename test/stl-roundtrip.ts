// SPDX-License-Identifier: AGPL-3.0-only
// STL reader/writer regression: ASCII parsing (incl. negative exponents — a class of files the
// old regex silently corrupted), binary round-trip, and format detection.
import { readSTL, readAsciiSTL, writeBinarySTL, isBinarySTL } from "../src/io/stl.ts";

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}`);
}

// --- ASCII: every numeric shape STL exporters emit, most importantly negative exponents.
const ascii = `solid test
  facet normal 0 0 1
    outer loop
      vertex 1.234e-05 -2.5E-3 3
      vertex -1 +0.5 .25
      vertex 1e2 -1.5e+2 0.0
    endloop
  endfacet
endsolid test
`;
const soup = readAsciiSTL(ascii);
check("ascii: one triangle parsed", soup.triangleCount === 1, `got ${soup.triangleCount}`);
const want = [1.234e-5, -2.5e-3, 3, -1, 0.5, 0.25, 1e2, -1.5e2, 0];
for (let i = 0; i < 9; i++) {
  check(`ascii: value[${i}] = ${want[i]}`, soup.positions[i] === want[i], `got ${soup.positions[i]}`);
}

// --- ASCII via the auto-detecting entry point (must not classify as binary).
const asciiBytes = new TextEncoder().encode(ascii);
check("ascii: not detected as binary", !isBinarySTL(asciiBytes));
check("ascii: readSTL round-trip", readSTL(asciiBytes).triangleCount === 1);

// --- Binary round-trip: write an indexed mesh, read the soup back, compare at f32 precision.
const positions = new Float64Array([0, 0, 0, 10.5, 0, 0, 0, 7.25, 0, 0, 0, -3.125e-4]);
const indices = new Uint32Array([0, 1, 2, 0, 2, 3]);
const bin = writeBinarySTL({ positions, indices });
check("binary: detected as binary", isBinarySTL(bin));
const back = readSTL(bin);
check("binary: triangle count", back.triangleCount === 2, `got ${back.triangleCount}`);
let maxErr = 0;
for (let t = 0; t < 2; t++) {
  for (let e = 0; e < 3; e++) {
    const vi = indices[t * 3 + e]! * 3;
    for (let k = 0; k < 3; k++) {
      const a = Math.fround(positions[vi + k]!); // writer stores f32
      const b = back.positions[t * 9 + e * 3 + k]!;
      maxErr = Math.max(maxErr, Math.abs(a - b));
    }
  }
}
check("binary: vertices exact at f32", maxErr === 0, `maxErr=${maxErr}`);

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("stl-roundtrip: all passed");
