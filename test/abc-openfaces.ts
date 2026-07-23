// SPDX-License-Identifier: AGPL-3.0-only
// Which faces carry the open edges? Mirrors abc-worker's watertight() but prints a per-face
// breakdown (face id, surface kind, loop shape, open-edge count) for leak diagnosis.
//   node test/abc-openfaces.ts <step-file>
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { estimateBrepSize } from "../src/step/measure.ts";

const path = process.argv[2]!;
const brep = buildBrep(readFileSync(path, "utf8"));
const diag = estimateBrepSize(brep)?.diag ?? 0;
const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
const targetEdge = diag > 0 ? 0.05 * diag : 1;
const res = tessellate(brep, { chordTol, targetEdge, normalDev: (15 * Math.PI) / 180 });
const m = res.mesh;

const K = 0x40000000;
const inc = new Map<number, { n: number; fwd: number; tri: number }>();
const nt = m.indices.length / 3;
const openSet = new Set(res.openSolids ?? []);
for (let t = 0; t < nt; t++) {
  if (openSet.size && openSet.has(res.solidOfTri[t]!)) continue;
  const a = m.indices[t * 3]!, b = m.indices[t * 3 + 1]!, c = m.indices[t * 3 + 2]!;
  for (const [x, y] of [[a, b], [b, c], [c, a]] as const) {
    const k = x < y ? x * K + y : y * K + x;
    const e = inc.get(k);
    if (e) { e.n++; if (x < y) e.fwd++; } else inc.set(k, { n: 1, fwd: x < y ? 1 : 0, tri: t });
  }
}
const perFace = new Map<number, number>();
let open = 0;
for (const e of inc.values()) {
  if (e.n === 2) continue;
  if (e.n === 1) { open++; const f = res.faceOfTri[e.tri]!; perFace.set(f, (perFace.get(f) ?? 0) + 1); }
}
const faceInfo = new Map<number, string>();
for (const s of brep.solids) for (const f of s.faces) {
  faceInfo.set(f.faceId, `${f.surfaceKind} loops=${f.loops.map((l) => l.edges.length).join("/")}`);
}
console.log(`open=${open} tris=${nt}`);
for (const [f, n] of [...perFace].sort((a, b) => b[1] - a[1]).slice(0, 25)) {
  console.log(`  fid=${f}  open=${n}  ${faceInfo.get(f) ?? "?"}`);
}
// Optional: dump the open edges of one face (endpoint coords) to see where they sit.
const dumpFid = Number(process.argv[3]);
if (Number.isFinite(dumpFid)) {
  const P = m.positions;
  for (const e of inc.values()) {
    if (e.n !== 1 || res.faceOfTri[e.tri] !== dumpFid) continue;
    const t = e.tri;
    const a = m.indices[t * 3]!, b = m.indices[t * 3 + 1]!, c = m.indices[t * 3 + 2]!;
    // report the triangle's open edge(s) by re-checking each
    for (const [x, y] of [[a, b], [b, c], [c, a]] as const) {
      const k = x < y ? x * K + y : y * K + x;
      if (inc.get(k)?.n === 1) {
        console.log(`  edge (${P[x * 3]!.toFixed(4)},${P[x * 3 + 1]!.toFixed(4)},${P[x * 3 + 2]!.toFixed(4)}) - (${P[y * 3]!.toFixed(4)},${P[y * 3 + 1]!.toFixed(4)},${P[y * 3 + 2]!.toFixed(4)})`);
      }
    }
  }
}
