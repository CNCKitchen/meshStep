// SPDX-License-Identifier: AGPL-3.0-only
// Diagnose the "near-watertight" seam-leak residue (<=20 open edges, no nm): for each
// still-open vertex in the FINAL mesh, measure (a) distance to its nearest open vertex that
// it is NOT edge-connected to (what a zip weld would use), (b) distance to the nearest open
// EDGE it is not incident to (T-junction gap), and (c) the per-vertex cap zipSlivers would
// apply. Relates every distance to the fixed zip tol (0.05) and the model's chordTol so we
// can see which gate blocks the close.
//
//   node test/abc-neargap-diag.ts [N]         # first N near-misses from out/abc-run2/near-misses.json
//   node test/abc-neargap-diag.ts <file.step>  # one specific model (path relative to abc root ok)
import { readFileSync, existsSync } from "node:fs";
import { join } from "node:path";
import { buildBrep, type BrepModel } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";

const ABC = "abc_0000_step_v00";

function brepDiag(brep: BrepModel): number {
  let mnx = Infinity, mny = Infinity, mnz = Infinity, mxx = -Infinity, mxy = -Infinity, mxz = -Infinity;
  for (const e of brep.edges.values()) {
    for (const p of [e.v0, e.v1]) {
      if (p[0]! < mnx) mnx = p[0]!; if (p[1]! < mny) mny = p[1]!; if (p[2]! < mnz) mnz = p[2]!;
      if (p[0]! > mxx) mxx = p[0]!; if (p[1]! > mxy) mxy = p[1]!; if (p[2]! > mxz) mxz = p[2]!;
    }
  }
  return Number.isFinite(mnx) ? Math.hypot(mxx - mnx, mxy - mny, mxz - mnz) : 0;
}

function diagnose(path: string): void {
  const src = readFileSync(path, "utf8");
  const brep = buildBrep(src);
  const diag = brepDiag(brep);
  const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
  const targetEdge = diag > 0 ? 0.05 * diag : 1;
  const res = tessellate(brep, { chordTol, targetEdge, normalDev: 15 * Math.PI / 180 });
  const { positions: P, indices: I } = res.mesh;

  const openSet = new Set(res.openSolids ?? []);
  const KEY = 2 ** 26;
  const ek = (a: number, b: number): number => (a < b ? a * KEY + b : b * KEY + a);
  const use = new Map<number, number>();
  const nt = I.length / 3;
  for (let t = 0; t < nt; t++) {
    if (openSet.has(res.solidOfTri[t]!)) continue;
    for (let e = 0; e < 3; e++) use.set(ek(I[t * 3 + e]!, I[t * 3 + (e + 1) % 3]!), (use.get(ek(I[t * 3 + e]!, I[t * 3 + (e + 1) % 3]!)) ?? 0) + 1);
  }
  const openE: [number, number][] = [];
  const openV = new Set<number>();
  const vCap = new Map<number, number>(); // zipSlivers' cap input: shortest incident OPEN segment
  for (let t = 0; t < nt; t++) {
    if (openSet.has(res.solidOfTri[t]!)) continue;
    for (let e = 0; e < 3; e++) {
      const a = I[t * 3 + e]!, b = I[t * 3 + (e + 1) % 3]!;
      if (use.get(ek(a, b)) !== 1) continue;
      openE.push([a, b]); openV.add(a); openV.add(b);
      const L = Math.hypot(P[a * 3]! - P[b * 3]!, P[a * 3 + 1]! - P[b * 3 + 1]!, P[a * 3 + 2]! - P[b * 3 + 2]!);
      vCap.set(a, Math.min(vCap.get(a) ?? Infinity, L)); vCap.set(b, Math.min(vCap.get(b) ?? Infinity, L));
    }
  }
  // face kinds owning open edges
  const faceKind = new Map<number, string>();
  for (const s of brep.solids) for (const f of s.faces) faceKind.set(f.faceId, f.surfaceKind);

  console.log(`\n=== ${path}`);
  console.log(`  diag=${diag.toFixed(2)} chordTol=${chordTol.toFixed(4)} openEdges=${openE.length} openVerts=${openV.size} tris=${nt}`);
  if (!openE.length) { console.log("  (no open edges — already watertight here?)"); return; }

  // chain the open edges into connected components (by shared vertex)
  const adj = new Map<number, Set<number>>();
  for (const [a, b] of openE) {
    (adj.get(a) ?? adj.set(a, new Set()).get(a)!).add(b);
    (adj.get(b) ?? adj.set(b, new Set()).get(b)!).add(a);
  }
  const compOf = new Map<number, number>();
  let ncomp = 0;
  for (const v of openV) {
    if (compOf.has(v)) continue;
    const stack = [v]; compOf.set(v, ncomp);
    while (stack.length) { const x = stack.pop()!; for (const y of adj.get(x) ?? []) if (!compOf.has(y)) { compOf.set(y, ncomp); stack.push(y); } }
    ncomp++;
  }
  const compSize = new Array(ncomp).fill(0);
  for (const v of openV) compSize[compOf.get(v)!]++;
  // is each component a closed loop? (every vertex has exactly 2 open-edge neighbours)
  const compLoop = new Array(ncomp).fill(true);
  for (const v of openV) if ((adj.get(v)?.size ?? 0) !== 2) compLoop[compOf.get(v)!] = false;
  console.log(`  components: ${ncomp} -> sizes [${compSize.join(",")}] loops [${compLoop.map((l: boolean) => l ? "Y" : "n").join("")}]`);

  const px = (i: number): number => P[i * 3]!, py = (i: number): number => P[i * 3 + 1]!, pz = (i: number): number => P[i * 3 + 2]!;
  const ov = [...openV];
  for (const v of ov) {
    // nearest non-edge-connected open vertex
    let best = -1, bestD = Infinity;
    let bestSame = -1, bestSameD = Infinity; // nearest in the SAME component (structure hint)
    for (const w of ov) {
      if (w === v) continue;
      const d = Math.hypot(px(v) - px(w), py(v) - py(w), pz(v) - pz(w));
      if (!use.has(ek(v, w))) {
        if (d < bestD) { bestD = d; best = w; }
        if (compOf.get(w) === compOf.get(v) && d < bestSameD) { bestSameD = d; bestSame = w; }
      }
    }
    // nearest open edge not incident to v (T-junction distance)
    let bestE = Infinity;
    for (const [a, b] of openE) {
      if (a === v || b === v) continue;
      const ax = px(a), ay = py(a), az = pz(a);
      const ex = px(b) - ax, ey = py(b) - ay, ez = pz(b) - az;
      const l2 = ex * ex + ey * ey + ez * ez;
      if (l2 < 1e-24) continue;
      let t = ((px(v) - ax) * ex + (py(v) - ay) * ey + (pz(v) - az) * ez) / l2;
      t = Math.max(0, Math.min(1, t));
      const d = Math.hypot(px(v) - ax - t * ex, py(v) - ay - t * ey, pz(v) - az - t * ez);
      if (d < bestE) bestE = d;
    }
    const cap = Math.min(0.05, Math.max(0.5 * (vCap.get(v) ?? Infinity), 1e-2));
    const crossComp = best >= 0 && compOf.get(best) !== compOf.get(v);
    console.log(`  v${v} comp=${compOf.get(v)} nnV=${bestD.toFixed(4)}${crossComp ? "x" : "s"} nnE=${bestE.toFixed(4)} cap=${cap.toFixed(4)} minSeg=${(vCap.get(v) ?? Infinity).toFixed(4)}`
      + `  | gap/chordTol=${(bestD / chordTol).toFixed(2)} gap/ztol=${(bestD / 0.05).toFixed(2)}`);
  }
  // which faces own the opens
  const kinds = new Map<string, number>();
  for (let t = 0; t < nt; t++) {
    if (openSet.has(res.solidOfTri[t]!)) continue;
    for (let e = 0; e < 3; e++) {
      const a = I[t * 3 + e]!, b = I[t * 3 + (e + 1) % 3]!;
      if (use.get(ek(a, b)) === 1) { const k = faceKind.get(res.faceOfTri[t]!) ?? "?"; kinds.set(k, (kinds.get(k) ?? 0) + 1); }
    }
  }
  console.log(`  open-edge owner kinds: ${JSON.stringify(Object.fromEntries(kinds))}`);
}

const arg = process.argv[2];
if (arg && /\.step$/i.test(arg)) {
  diagnose(existsSync(arg) ? arg : join(ABC, arg));
} else {
  const n = arg ? parseInt(arg, 10) : 8;
  const near = JSON.parse(readFileSync("out/abc-run2/near-misses.json", "utf8")) as { file: string; open: number; kind: string }[];
  for (const r of near.slice(0, n)) {
    try { diagnose(join(ABC, r.file)); }
    catch (e) { console.log(`\n=== ${r.file}\n  ERROR: ${e instanceof Error ? e.message : e}`); }
  }
}
