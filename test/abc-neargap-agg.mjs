// Aggregate out/abc-run2/neargap-stats.jsonl -> gate design data for the small-hole fill.
import { readFileSync } from "node:fs";
const recs = readFileSync("out/abc-run2/neargap-stats.jsonl", "utf8").split(/\r?\n/).filter(Boolean).map(JSON.parse);
console.log("models:", recs.length, " errors:", recs.filter(r => r.err).length);
for (const r of recs.filter(r => r.err)) console.log("  ERR", r.file, r.err.slice(0, 80));

const ok = recs.filter(r => !r.err);
// how much residue is ring-shaped?
const allRing = ok.filter(r => r.otherEdges === 0);
console.log(`all-ring models: ${allRing.length}/${ok.length}; models with non-ring residue: ${ok.length - allRing.length}`);

// ring population
const rings = ok.flatMap(r => r.rings.map(g => ({ ...g, te: r.targetEdge, ct: r.chordTol, file: r.file })));
console.log("total rings:", rings.length);
const q = (a, p) => a.slice().sort((x, y) => x - y)[Math.min(a.length - 1, Math.floor(p * a.length))];
const ns = rings.map(g => g.n);
const perTe = rings.map(g => g.per / g.te);
const devPer = rings.map(g => g.dev / g.per);
console.log("ring nverts:   p50=%s p90=%s p99=%s max=%s", q(ns, .5), q(ns, .9), q(ns, .99), Math.max(...ns));
console.log("per/targetEdge p50=%s p90=%s p99=%s max=%s", q(perTe, .5).toFixed(2), q(perTe, .9).toFixed(2), q(perTe, .99).toFixed(2), Math.max(...perTe).toFixed(2));
console.log("dev/per        p50=%s p90=%s p99=%s max=%s", q(devPer, .5).toFixed(3), q(devPer, .9).toFixed(3), q(devPer, .99).toFixed(3), Math.max(...devPer).toFixed(3));

// candidate gates: nverts<=NV, per<=C*targetEdge (3-rings exempt from per), dev<=S*per
for (const [NV, C, S] of [[24, 2, 0.15], [24, 4, 0.2], [24, 6, 0.25], [32, 8, 0.3], [24, 4, 1]]) {
  let recovered = 0, ringsPassing = 0;
  for (const r of ok) {
    if (r.otherEdges !== 0) continue;
    const pass = r.rings.every(g =>
      (g.n === 3 && (g.per <= 1e9)) ||
      (g.n <= NV && g.per <= C * r.targetEdge && g.dev <= S * g.per));
    if (pass) recovered++;
  }
  for (const g of rings) if ((g.n === 3) || (g.n <= NV && g.per <= C * g.te && g.dev <= S * g.per)) ringsPassing++;
  console.log(`gate NV=${NV} per<=${C}·te dev<=${S}·per  -> rings ${ringsPassing}/${rings.length}, MODELS RECOVERED ${recovered}/${ok.length}`);
}

// what blocks the rest? per-model diagnosis under the middle gate
const NV = 24, C = 4, S = 0.2;
let blockedByOther = 0, blockedByN = 0, blockedByPer = 0, blockedByDev = 0;
for (const r of ok) {
  if (r.otherEdges !== 0) { blockedByOther++; continue; }
  let bn = false, bp = false, bd = false;
  for (const g of r.rings) {
    if (g.n === 3) continue;
    if (g.n > NV) bn = true;
    else if (g.per > C * r.targetEdge) bp = true;
    else if (g.dev > S * g.per) bd = true;
  }
  if (bn) blockedByN++; else if (bp) blockedByPer++; else if (bd) blockedByDev++;
}
console.log(`blocked: non-ring residue ${blockedByOther}, nverts ${blockedByN}, perim ${blockedByPer}, dev ${blockedByDev}`);
// print the models blocked by perimeter with their worst ring
for (const r of ok) {
  if (r.otherEdges !== 0) continue;
  const bad = r.rings.filter(g => g.n > 3 && (g.n > NV || g.per > C * r.targetEdge || g.dev > S * g.per));
  if (bad.length) console.log("  blocked:", r.file.split("\\").pop().slice(0, 40), bad.map(g => `n=${g.n} per/te=${(g.per / r.targetEdge).toFixed(1)} dev/per=${(g.dev / g.per).toFixed(2)}`).join(" | "));
}
