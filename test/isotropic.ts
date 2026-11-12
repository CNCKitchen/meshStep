// SPDX-License-Identifier: AGPL-3.0-only
// Isotropic mode regression checks on the tracked fixtures (cube, cylinder, sphere, everything):
// the mode must stay watertight and warning-free like the default, seed every edge evenly, and
// actually deliver near-equilateral triangles. Plus a synthetic check of the per-face optimiser
// on its own. (The size field has its own test, test/size-field.ts; the full default-vs-isotropic
// comparison over the local corpus is test/iso-corpus.ts.)
import { readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";
import { optimizeFace } from "../src/mesh/optimize.ts";
import { constrainedTriangulate } from "../src/mesh/cdt2d.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
let failed = 0;
const check = (ok: boolean, msg: string): void => { console.log(`${ok ? "ok  " : "FAIL"} ${msg}`); if (!ok) failed++; };

/** Smallest angle (degrees) of every triangle, ascending. */
function minAngles(P: ArrayLike<number>, I: ArrayLike<number>): number[] {
  const out: number[] = [];
  for (let t = 0; t < I.length; t += 3) {
    const l = (a: number, b: number): number => Math.hypot(P[a * 3]! - P[b * 3]!, P[a * 3 + 1]! - P[b * 3 + 1]!, P[a * 3 + 2]! - P[b * 3 + 2]!);
    const s = [l(I[t]!, I[t + 1]!), l(I[t + 1]!, I[t + 2]!), l(I[t + 2]!, I[t]!)].sort((x, y) => x - y);
    const c = (s[1]! ** 2 + s[2]! ** 2 - s[0]! ** 2) / (2 * s[1]! * s[2]!); // angle facing the shortest side
    out.push(Math.acos(Math.max(-1, Math.min(1, c))) * 180 / Math.PI);
  }
  return out.sort((x, y) => x - y);
}

// ---- fixtures through the public entry ---------------------------------------------------------
for (const [name, p5Min] of [["cube", 40], ["cylinder", 35], ["sphere", 30], ["everything", 30]] as const) {
  const src = readFileSync(join(root, `${name}.step`), "utf8");
  const def = importStep(src);
  const iso = importStep(src, { isotropic: true, measureGeometry: true });
  check(iso.diagnostics.ok, `${name}: isotropic import is clean (open ${iso.diagnostics.openEdges}, non-manifold ${iso.diagnostics.nonManifoldEdges}, warnings ${iso.diagnostics.warnings.length})`);
  check(iso.stats.facesTessellated === def.stats.facesTessellated, `${name}: same faces meshed as the default (${iso.stats.facesTessellated}/${def.stats.facesTessellated})`);
  const a = minAngles(iso.mesh.positions, iso.mesh.indices);
  const p5 = a[Math.floor(a.length * 0.05)]!;
  check(p5 >= p5Min, `${name}: 5th-percentile smallest angle ${p5.toFixed(1)}° >= ${p5Min}°`);
  // Edge seeding: no stub at an edge end, and neighbouring segments within the growth rate
  // (1.3) plus rounding slack.
  let stubs = 0, worst = 1;
  for (const e of iso.measure?.edges ?? []) {
    const pl = iso.measure!.points, o = e.first * 3;
    const seg: number[] = [];
    for (let i = 0; i + 1 < e.count; i++) seg.push(Math.hypot(pl[o + i * 3 + 3]! - pl[o + i * 3]!, pl[o + i * 3 + 4]! - pl[o + i * 3 + 1]!, pl[o + i * 3 + 5]! - pl[o + i * 3 + 2]!));
    for (let i = 0; i + 1 < seg.length; i++) worst = Math.max(worst, Math.max(seg[i]!, seg[i + 1]!) / Math.min(seg[i]!, seg[i + 1]!));
    if (seg.length > 1 && (seg[0]! < 0.5 * seg[1]! || seg[seg.length - 1]! < 0.5 * seg[seg.length - 2]!)) stubs++;
  }
  check(stubs === 0, `${name}: no edge ends in a stub segment`);
  check(worst <= 1.6, `${name}: neighbouring edge segments differ by at most x${worst.toFixed(2)} (<= 1.6)`);
}

// ---- the optimiser alone -------------------------------------------------------------------------
// A 10 x 6 rectangle on the plane z = 0, boundary seeded at 0.25 on the bottom side, at 1 on
// the top and graded in between on the two sides, with no interior point at all (the coarsest valid start). Size: 0.25 at the fine side growing 0.3
// per unit distance to 1.
{
  const X: number[] = [], Y: number[] = [], P: number[] = [];
  const add = (x: number, y: number): void => { X.push(x); Y.push(y); P.push(x, y, 0); };
  // Side nodes graded the way the edge seeder would: steps growing x1.3 from 0.25 up to 1.
  const ys: number[] = [0];
  for (let h = 0.25; ys[ys.length - 1]! < 6; h = Math.min(1, h * 1.3)) ys.push(ys[ys.length - 1]! + h);
  for (let k = 0; k < ys.length; k++) ys[k] = (ys[k]! * 6) / ys[ys.length - 1]!;
  for (let i = 0; i < 40; i++) add(i * 0.25, 0);          // fine bottom side
  for (let k = 0; k + 1 < ys.length; k++) add(10, ys[k]!);
  for (let i = 0; i < 10; i++) add(10 - i, 6);
  for (let k = ys.length - 1; k > 0; k--) add(0, ys[k]!);
  const nB = X.length;
  // Boundary-only constrained Delaunay triangulation: valid, but nothing like the target mesh.
  const tris = constrainedTriangulate(X.map((x, i) => [x, Y[i]!] as [number, number]), [Array.from({ length: nB }, (_, i) => i)], []);
  const res = optimizeFace({
    x: X, y: Y, p: P, tris, nFrozen: nB,
    isConstrained: (a, b) => Math.abs(a - b) === 1 || Math.abs(a - b) === nB - 1,
    evalAt: (x, y) => [x, y, 0],
    sizeAt: (_x, y) => Math.min(1, 0.25 + 0.3 * y),
  });
  check(res !== null, "optimiser: accepts a boundary-only triangulation");
  if (res) {
    let area = 0, inverted = 0;
    const used = new Set<number>();
    for (const [a, b, c] of res.tris) {
      const ar = (X[b]! - X[a]!) * (Y[c]! - Y[a]!) - (Y[b]! - Y[a]!) * (X[c]! - X[a]!);
      if (!(ar > 0)) inverted++;
      area += ar / 2;
      for (const [u, v] of [[a, b], [b, c], [c, a]] as const) used.add(Math.min(u, v) * 100000 + Math.max(u, v));
    }
    let boundaryKept = 0;
    for (let i = 0; i < nB; i++) { const j = (i + 1) % nB; if (used.has(Math.min(i, j) * 100000 + Math.max(i, j))) boundaryKept++; }
    check(inverted === 0, "optimiser: no inverted triangle");
    check(Math.abs(area - 60) < 1e-9, `optimiser: covers the region exactly (area ${area.toFixed(6)})`);
    check(boundaryKept === nB, `optimiser: every boundary segment kept (${boundaryKept}/${nB})`);
    const I = res.tris.flat();
    const a = minAngles(P, I);
    check(a[0]! > 20 && a[Math.floor(a.length * 0.05)]! > 35, `optimiser: smallest angle ${a[0]!.toFixed(1)}°, 5th percentile ${a[Math.floor(a.length * 0.05)]!.toFixed(1)}° (${res.tris.length} triangles)`);
    // Graded: triangles at the fine side are small, at the far side large.
    let fine = 0, coarse = 0, nf = 0, nc = 0;
    for (const [p, q] of res.tris.flatMap(([a2, b2, c2]) => [[a2, b2], [b2, c2], [c2, a2]] as const)) {
      const ym = (Y[p]! + Y[q]!) / 2, L = Math.hypot(X[p]! - X[q]!, Y[p]! - Y[q]!);
      if (ym < 0.5) { fine += L; nf++; } else if (ym > 4) { coarse += L; nc++; }
    }
    check(fine / nf < 0.45 && coarse / nc > 0.75, `optimiser: graded sizes (mean edge ${(fine / nf).toFixed(2)} at the fine side, ${(coarse / nc).toFixed(2)} far from it)`);
  }
}

if (failed) { console.error(`isotropic: ${failed} check(s) failed`); process.exit(1); }
console.log("isotropic: all passed");
