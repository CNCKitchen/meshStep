// SPDX-License-Identifier: AGPL-3.0-only
// Corpus check over testFiles/*.stp — watertightness + empty-face coverage per surface kind.
// Usage: node test/corpus.ts [nameFilter]
import { readFileSync, readdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";

const dir = join(dirname(fileURLToPath(import.meta.url)), "..", "testFiles");

function meshStats(I: Uint32Array, P: Float64Array | Float32Array) {
  const nt = I.length / 3, und = new Map<string, number>();
  for (let t = 0; t < nt; t++)
    for (let e = 0; e < 3; e++) { const a = I[t*3+e]!, b = I[t*3+(e+1)%3]!; const k = a < b ? `${a}_${b}` : `${b}_${a}`; und.set(k, (und.get(k) ?? 0) + 1); }
  let open = 0, nm = 0; for (const c of und.values()) { if (c === 1) open++; else if (c > 2) nm++; }
  return { nt, open, nm };
}

const emptyByKind = new Map<string, number>();
const totalByKind = new Map<string, number>();
let pass = 0, fail = 0, err = 0;

const only = process.argv[2];
const files = readdirSync(dir).filter(x => /\.(stp|step)$/i.test(x)).filter(x => !only || x.includes(only)).sort();
for (const f of files) {
  const src = readFileSync(join(dir, f), "utf8");
  try {
    const brep = buildBrep(src);
    let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const e of brep.edges.values()) for (const v of [e.v0, e.v1]) for (let k = 0; k < 3; k++) { if (v[k]! < lo[k]!) lo[k] = v[k]!; if (v[k]! > hi[k]!) hi[k] = v[k]!; }
    const diag = Number.isFinite(hi[0]) ? Math.hypot(hi[0]!-lo[0]!, hi[1]!-lo[1]!, hi[2]!-lo[2]!) : 0;
    const maxEdge = Math.max(0.5, diag / 120);
    const surfaceDev = Math.max(0.01, diag / 4000);
    const t0 = Date.now();
    const res = tessellate(brep, { chordTol: surfaceDev, targetEdge: maxEdge, normalDev: 15 * Math.PI / 180 });
    const ms = Date.now() - t0;

    const triPerFace = new Map<number, number>();
    for (let t = 0; t < res.faceOfTri.length; t++) triPerFace.set(res.faceOfTri[t]!, (triPerFace.get(res.faceOfTri[t]!) ?? 0) + 1);
    let emptyFaces = 0, totalFaces = 0;
    for (const solid of brep.solids) for (const face of solid.faces) {
      totalFaces++;
      totalByKind.set(face.surfaceKind, (totalByKind.get(face.surfaceKind) ?? 0) + 1);
      if (!(triPerFace.get(face.faceId)! > 0)) { emptyFaces++; emptyByKind.set(face.surfaceKind, (emptyByKind.get(face.surfaceKind) ?? 0) + 1); }
    }

    const s = meshStats(res.mesh.indices, res.mesh.positions as Float64Array);
    const ok = s.open === 0 && s.nm === 0 && emptyFaces === 0;
    ok ? pass++ : fail++;
    console.log(`${ok ? "PASS" : "FAIL"} ${f.padEnd(36)} tris=${String(s.nt).padStart(7)} open=${String(s.open).padStart(5)} nm=${String(s.nm).padStart(4)} emptyFaces=${emptyFaces}/${totalFaces} ${ms}ms`);
  } catch (e) { err++; console.log(`ERR  ${f.padEnd(36)} ${(e as Error).message.slice(0, 120)}`); }
}

console.log(`\n${pass} pass, ${fail} fail, ${err} error of ${files.length}`);
console.log("--- empty faces by surface kind ---");
for (const [kind, total] of [...totalByKind.entries()].sort((a, b) => (emptyByKind.get(b[0]) ?? 0) - (emptyByKind.get(a[0]) ?? 0))) {
  const empty = emptyByKind.get(kind) ?? 0;
  if (empty > 0) console.log(`GAP ${kind.padEnd(34)} ${empty} / ${total}`);
}
