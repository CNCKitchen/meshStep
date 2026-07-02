// SPDX-License-Identifier: AGPL-3.0-only
// Convert every testFiles/*.stp|*.step to a binary STL in stl_out/ for visual inspection.
// Uses the buildBrep+tessellate path (same as test/corpus.ts) so it matches the watertightness
// check and avoids the heavier importStep post-passes. Usage: node test/export-corpus-stl.ts [filter]
import { readFileSync, writeFileSync, readdirSync, mkdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { writeBinarySTL } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const inDir = join(root, "testFiles");
const outDir = join(root, "stl_out");
mkdirSync(outDir, { recursive: true });

const only = process.argv[2];
const files = readdirSync(inDir).filter((x) => /\.(stp|step)$/i.test(x)).filter((x) => !only || x.includes(only)).sort();

// Optional overrides to match a CAD tessellation export (else a part-relative heuristic is used):
//   MESH_CHORD=0.0015 MESH_NORMDEV=10 MESH_MAXEDGE=85 node test/export-corpus-stl.ts OpenVessel
const envChord = process.env.MESH_CHORD ? Number(process.env.MESH_CHORD) : null;
const envNormDev = process.env.MESH_NORMDEV ? Number(process.env.MESH_NORMDEV) : null;
const envMaxEdge = process.env.MESH_MAXEDGE ? Number(process.env.MESH_MAXEDGE) : null;

let ok = 0, fail = 0;
for (const f of files) {
  const base = f.replace(/\.[^.]+$/, "");
  try {
    const src = readFileSync(join(inDir, f), "utf8");
    const brep = buildBrep(src);
    // Size tolerances to the part (same heuristic as corpus.ts) so small and large parts both look right.
    let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const e of brep.edges.values()) for (const v of [e.v0, e.v1]) for (let k = 0; k < 3; k++) { if (v[k]! < lo[k]!) lo[k] = v[k]!; if (v[k]! > hi[k]!) hi[k] = v[k]!; }
    const diag = Number.isFinite(hi[0]!) ? Math.hypot(hi[0]! - lo[0]!, hi[1]! - lo[1]!, hi[2]! - lo[2]!) : 0;
    const maxEdge = envMaxEdge ?? Math.max(0.5, diag / 120);
    const surfaceDev = envChord ?? Math.max(0.01, diag / 4000);
    const normalDev = (envNormDev ?? 15) * Math.PI / 180;
    const t0 = Date.now();
    const res = tessellate(brep, { chordTol: surfaceDev, targetEdge: maxEdge, normalDev });
    const nt = res.mesh.indices.length / 3;
    writeFileSync(join(outDir, `${base}.stl`), writeBinarySTL(res.mesh));
    console.log(`OK   ${f.padEnd(38)} ${String(nt).padStart(8)} tris -> stl_out/${base}.stl (${Date.now() - t0}ms)`);
    ok++;
  } catch (e) {
    console.log(`ERR  ${f.padEnd(38)} ${(e as Error).message.slice(0, 100)}`);
    fail++;
  }
}
console.log(`\n${ok} written, ${fail} errored -> ${outDir}`);
