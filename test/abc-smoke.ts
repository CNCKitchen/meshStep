// SPDX-License-Identifier: AGPL-3.0-only
// Throwaway smoke test: decomposed import pipeline on a handful of ABC models.
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { orientConsistent } from "../src/mesh/orient.ts";

function watertight(m: { positions: Float64Array; indices: Uint32Array }, openTri?: (t: number) => boolean) {
  const K = 0x40000000;
  const inc = new Map<number, number>();
  const nt = m.indices.length / 3;
  for (let t = 0; t < nt; t++) {
    if (openTri?.(t)) continue;
    for (let e = 0; e < 3; e++) {
      const a = m.indices[t * 3 + e]!, b = m.indices[t * 3 + (e + 1) % 3]!;
      const k = a < b ? a * K + b : b * K + a;
      inc.set(k, (inc.get(k) ?? 0) + 1);
    }
  }
  let open = 0, nm = 0;
  for (const c of inc.values()) { if (c === 1) open++; else if (c > 2) nm++; }
  return { open, nm };
}

function brepDiag(brep: any): number {
  let mnx = Infinity, mny = Infinity, mnz = Infinity, mxx = -Infinity, mxy = -Infinity, mxz = -Infinity;
  for (const e of brep.edges.values()) {
    for (const p of [e.v0, e.v1]) {
      if (p[0] < mnx) mnx = p[0]; if (p[1] < mny) mny = p[1]; if (p[2] < mnz) mnz = p[2];
      if (p[0] > mxx) mxx = p[0]; if (p[1] > mxy) mxy = p[1]; if (p[2] > mxz) mxz = p[2];
    }
  }
  if (!Number.isFinite(mnx)) return 0;
  return Math.hypot(mxx - mnx, mxy - mny, mxz - mnz);
}

for (const path of process.argv.slice(2)) {
  const t0 = Date.now();
  try {
    const src = readFileSync(path, "utf8");
    const brep = buildBrep(src);
    const diag = brepDiag(brep);
    const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
    const targetEdge = diag > 0 ? 0.05 * diag : 1;
    const res = tessellate(brep, { chordTol, targetEdge, normalDev: 15 * Math.PI / 180 });
    orientConsistent(res.mesh, res.solidOfTri);
    const openSet = new Set(res.openSolids ?? []);
    const wt = watertight(res.mesh, openSet.size ? (t) => openSet.has(res.solidOfTri[t]!) : undefined);
    const nt = res.mesh.indices.length / 3;
    const ok = wt.open === 0 && wt.nm === 0 && res.stats.facesTessellated >= res.stats.facesTotal;
    console.log(`${ok ? "PASS" : "FAIL"} ${path.split(/[\\/]/).pop()} diag=${diag.toFixed(1)} tris=${nt} open=${wt.open} nm=${wt.nm} faces=${res.stats.facesTessellated}/${res.stats.facesTotal} skipped=${JSON.stringify(res.stats.skipped)} ${Date.now() - t0}ms`);
  } catch (e) {
    console.log(`ERR  ${path.split(/[\\/]/).pop()}: ${(e as Error).message} ${Date.now() - t0}ms`);
  }
}
