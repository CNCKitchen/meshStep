// SPDX-License-Identifier: AGPL-3.0-only
// Size-field checks (isotropic mode): the kd-tree query must equal the brute-force definition
// h(x) = clamp(min(hMax, min_i(h_i + g·|x − x_i|))), and the field must be growth-limited.
import { SizeField } from "../src/mesh/size-field.ts";

let seed = 12345;
const rnd = (): number => ((seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 4294967296);
let failed = 0;
const check = (ok: boolean, msg: string): void => { if (!ok) { failed++; console.error("FAIL " + msg); } };

for (const [n, growth, hMin, hMax] of [[0, 1.3, 0.02, 1], [1, 1.3, 0.02, 1], [7, 1.2, 0.05, 2], [500, 1.3, 0.02, 1], [5000, 1.5, 0.01, 3]] as const) {
  const src: number[][] = [];
  const f = new SizeField(growth, hMin, hMax);
  for (let i = 0; i < n; i++) {
    // Clustered sources (edges are lines of sources, not a uniform cloud), sizes over two decades,
    // some above hMax (must be ignored) and some below hMin (must clamp).
    const c = i % 5;
    const s = [c * 3 + rnd() * (c === 0 ? 0.01 : 2), rnd() * 10, c === 1 ? 0 : rnd() * 4, 0.005 * 10 ** (rnd() * 3)];
    src.push(s); f.add(s[0]!, s[1]!, s[2]!, s[3]!);
  }
  f.build();
  const g = growth - 1;
  const brute = (x: number, y: number, z: number): number => {
    let best: number = hMax;
    for (const s of src) best = Math.min(best, Math.max(s[3]!, hMin) + g * Math.hypot(s[0]! - x, s[1]! - y, s[2]! - z));
    return Math.max(hMin, best);
  };
  let worst = 0, lip = 0;
  for (let q = 0; q < 2000; q++) {
    const x = rnd() * 16 - 1, y = rnd() * 12 - 1, z = rnd() * 6 - 1;
    const a = f.at(x, y, z);
    worst = Math.max(worst, Math.abs(a - brute(x, y, z)));
    const dx = rnd() - 0.5, dy = rnd() - 0.5, dz = rnd() - 0.5;
    lip = Math.max(lip, Math.abs(f.at(x + dx, y + dy, z + dz) - a) - g * Math.hypot(dx, dy, dz));
  }
  check(worst < 1e-12, `n=${n}: kd-tree differs from brute force by ${worst}`);
  check(lip < 1e-12, `n=${n}: field grows faster than g by ${lip}`);
  check(f.size <= n, `n=${n}: more sources than added`);
  console.log(`n=${n}: ${f.size} sources kept, max query error ${worst.toExponential(1)}`);
}
// A built field takes further sources and a rebuild: the old sources stay in force.
{
  const f = new SizeField(1.3, 0.01, 1);
  f.add(0, 0, 0, 0.1); f.build();
  const before = f.at(5, 0, 0);
  f.add(5, 0, 0, 0.2); f.build();
  check(Math.abs(f.at(0, 0, 0) - 0.1) < 1e-12, `rebuild: the first source is still there (${f.at(0, 0, 0)})`);
  check(before === 1 && Math.abs(f.at(5, 0, 0) - 0.2) < 1e-12, `rebuild: the added source took effect (${before} -> ${f.at(5, 0, 0)})`);
  check(Math.abs(f.at(1, 0, 0) - 0.4) < 1e-12 && Math.abs(f.at(4, 0, 0) - 0.5) < 1e-12, "rebuild: both sources grade the space between them");
}
if (failed) { console.error(`${failed} check(s) failed`); process.exit(1); }
console.log("size-field: ok");
