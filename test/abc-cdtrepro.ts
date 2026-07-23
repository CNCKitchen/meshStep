// SPDX-License-Identifier: AGPL-3.0-only
// Re-run a dumped gridCDT input (MESHSTEP_DUMPCDT) through constrainedTriangulate in isolation
// and audit the RESULT: edge use-counts among the emitted triangles, restricted to the boundary
// polygon's own edges, plus any edge used more than twice (overlap) or boundary edge used twice.
//   node test/abc-cdtrepro.ts <dump-log-file>
import { readFileSync } from "node:fs";
import { constrainedTriangulate } from "../src/mesh/cdt2d.ts";

const log = readFileSync(process.argv[2]!, "utf8");
const m = log.match(/@@CDTDUMP@@(.*?)@@END@@/s);
if (!m) throw new Error("no CDTDUMP in log");
const d = JSON.parse(m[1]!) as { fid: number; outerIdx: number[]; holeIdx: number[][]; cdtPts: [number, number][]; p3: [number, number, number][] };
console.log(`fid=${d.fid} outer=${d.outerIdx.length} holes=${d.holeIdx.length} pts=${d.cdtPts.length}`);
// pinch positions: vertices appearing more than once in the outer walk
const count = new Map<number, number>();
for (const v of d.outerIdx) count.set(v, (count.get(v) ?? 0) + 1);
for (const [v, c] of count) if (c > 1) console.log(`  pinch vertex ${v} appears ${c}x at positions ${d.outerIdx.map((x, i) => (x === v ? i : -1)).filter((i) => i >= 0).join(",")}  uv=(${d.cdtPts[v]![0].toFixed(4)},${d.cdtPts[v]![1].toFixed(4)})`);
// Boundary points occupy the first block of cdtPts (pushLoop order); an aliased pinch twin
// stays in that block but is referenced by no loop and is NEVER inserted — mirror that.
const nBoundary = d.outerIdx.length + d.holeIdx.reduce((s, h) => s + h.length, 0);
const interior = Array.from({ length: d.cdtPts.length }, (_, i) => i).filter((i) => i >= nBoundary);
const out: { missing: number; rescue?: string } = { missing: 0 };
const tris = constrainedTriangulate(d.cdtPts, [d.outerIdx, ...d.holeIdx], interior, out);
console.log(`tris=${tris.length} missing=${out.missing} rescue=${out.rescue ?? ""}`);
// audit edges
const ek = (a: number, b: number): string => (a < b ? `${a},${b}` : `${b},${a}`);
const use = new Map<string, number>();
for (const [a, b, c] of tris) for (const [x, y] of [[a, b], [b, c], [c, a]] as const) use.set(ek(x, y), (use.get(ek(x, y)) ?? 0) + 1);
const boundary = new Set<string>();
for (const loop of [d.outerIdx, ...d.holeIdx]) for (let i = 0; i < loop.length; i++) boundary.add(ek(loop[i]!, loop[(i + 1) % loop.length]!));
let bad = 0;
for (const [k, n] of use) {
  const isB = boundary.has(k);
  if ((isB && n !== 1) || (!isB && n !== 2)) {
    if (bad++ < 20) console.log(`  edge ${k}: use=${n} boundary=${isB}`);
  }
}
console.log(`defective edges: ${bad}`);
for (const v of [...count].filter(([, c]) => c > 1).map(([v]) => v)) {
  console.log(`triangles at pinch ${v}:`);
  for (const t of tris) if (t.includes(v)) console.log(`  [${t.join(",")}]`);
}
