// SPDX-License-Identifier: AGPL-3.0-only
// Ring-level statistics over the near-watertight seam-leak residue (out/abc-run2/near-misses.json):
// for each model, tessellate and decompose the final open-edge set into undirected rings
// (every vertex of the component has exactly 2 incident open edges) vs bowtie/other residue,
// and record each ring's vertex count, perimeter, best-fit planar deviation, and scale context
// (chordTol / targetEdge). The aggregate picks the fill gates for the small-hole extension.
//
//   node test/abc-neargap-stats.ts            # orchestrate: pool of 6 children over all 138
//   node test/abc-neargap-stats.ts --child i n # child mode: process near-misses [i, i+n, i+2n, ...)
import { readFileSync, writeFileSync, appendFileSync, existsSync, unlinkSync } from "node:fs";
import { spawn } from "node:child_process";
import { join } from "node:path";
import { buildBrep, type BrepModel } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";

const ABC = "abc_0000_step_v00";
const OUT = "out/abc-run2/neargap-stats.jsonl";

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

function statsOne(path: string): object {
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
  const adj = new Map<number, number[]>();
  let openEdges = 0;
  for (let t = 0; t < nt; t++) {
    if (openSet.has(res.solidOfTri[t]!)) continue;
    for (let e = 0; e < 3; e++) {
      const a = I[t * 3 + e]!, b = I[t * 3 + (e + 1) % 3]!;
      if (use.get(ek(a, b)) !== 1) continue;
      openEdges++;
      (adj.get(a) ?? adj.set(a, []).get(a)!).push(b);
      (adj.get(b) ?? adj.set(b, []).get(b)!).push(a);
    }
  }

  const planarDev = (ring: number[]): number => {
    let nx = 0, ny = 0, nz = 0, cx = 0, cy = 0, cz = 0;
    const n = ring.length;
    for (let i = 0; i < n; i++) {
      const a = ring[i]! * 3, b = ring[(i + 1) % n]! * 3;
      nx += (P[a + 1]! - P[b + 1]!) * (P[a + 2]! + P[b + 2]!);
      ny += (P[a + 2]! - P[b + 2]!) * (P[a]! + P[b]!);
      nz += (P[a]! - P[b]!) * (P[a + 1]! + P[b + 1]!);
      cx += P[a]!; cy += P[a + 1]!; cz += P[a + 2]!;
    }
    const l = Math.hypot(nx, ny, nz);
    if (l < 1e-30) return Infinity;
    nx /= l; ny /= l; nz /= l; cx /= n; cy /= n; cz /= n;
    let mx = 0;
    for (const v of ring) mx = Math.max(mx, Math.abs((P[v * 3]! - cx) * nx + (P[v * 3 + 1]! - cy) * ny + (P[v * 3 + 2]! - cz) * nz));
    return mx;
  };

  // undirected ring walk over degree-2 vertices; anything else is bowtie/other residue
  const rings: { n: number; per: number; dev: number }[] = [];
  let bowtieEdges = 0;
  const visited = new Set<number>();
  for (const [start, nb] of adj) {
    if (visited.has(start)) continue;
    if (nb.length !== 2) continue; // handled below as residue
    const ring: number[] = [start];
    let prev = start, cur = nb[0]!, ok = true;
    for (let g = 0; g < 4096; g++) {
      const link = adj.get(cur);
      if (!link || link.length !== 2) { ok = false; break; }
      if (cur === start) break;
      ring.push(cur);
      const next = link[0]! === prev ? link[1]! : link[0]!;
      prev = cur; cur = next;
    }
    if (!ok || cur !== start) { for (const v of ring) visited.add(v); continue; }
    for (const v of ring) visited.add(v);
    let per = 0;
    for (let i = 0; i < ring.length; i++) {
      const a = ring[i]!, b = ring[(i + 1) % ring.length]!;
      per += Math.hypot(P[a * 3]! - P[b * 3]!, P[a * 3 + 1]! - P[b * 3 + 1]!, P[a * 3 + 2]! - P[b * 3 + 2]!);
    }
    rings.push({ n: ring.length, per, dev: planarDev(ring) });
  }
  for (const [v, nb] of adj) if (nb.length !== 2) bowtieEdges += nb.length;
  bowtieEdges /= 2; // roughly: each such edge counted from both ends only if both ends deg!=2

  const ringEdges = rings.reduce((s, r) => s + r.n, 0);
  return { file: path, diag, chordTol, targetEdge, open: openEdges, ringEdges, otherEdges: openEdges - ringEdges, rings };
}

const argv = process.argv.slice(2);
if (argv[0] === "--child") {
  const i0 = parseInt(argv[1]!, 10), stride = parseInt(argv[2]!, 10);
  const near = JSON.parse(readFileSync("out/abc-run2/near-misses.json", "utf8")) as { file: string }[];
  for (let i = i0; i < near.length; i += stride) {
    const f = near[i]!.file;
    let rec: object;
    try { rec = statsOne(join(ABC, f)); }
    catch (e) { rec = { file: f, err: e instanceof Error ? e.message : String(e) }; }
    appendFileSync(OUT, JSON.stringify(rec) + "\n");
    console.log(`[${i0}] done ${i}: ${f}`);
  }
} else {
  if (existsSync(OUT)) unlinkSync(OUT);
  const N = 6;
  let done = 0;
  for (let k = 0; k < N; k++) {
    const c = spawn(process.execPath, ["test/abc-neargap-stats.ts", "--child", String(k), String(N)], { stdio: "inherit" });
    c.on("exit", () => { if (++done === N) console.log("ALL DONE"); });
  }
}
