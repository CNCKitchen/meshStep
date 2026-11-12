// SPDX-License-Identifier: AGPL-3.0-only
// Triangle-SHAPE report for one STEP file: the yardstick for the isotropic ("pretty") mesh mode.
// test/mesh-audit.ts answers "is the mesh correct" (folds, flips, deviation); this answers "is it
// nice" (how close to equilateral, how evenly the edges are seeded, how smoothly sizes grow) and
// what it costs (triangles, time). Every comparison between the default and the isotropic mode
// is made with this one tool so the numbers stay comparable.
//
//   node test/mesh-quality.ts <file.step> [--dev mm] [--edge mm] [--normal deg]
//        [--iso] [--growth r] [--min-edge mm]   mesh in the isotropic mode (see TessOptions)
//        [--compare]                            default and isotropic side by side
//        [--runs n]                             time = median of n tessellations (default 3)
//        [--detail]                             per-mesher and per-surface-kind rows
//        [--worst n]                            list the n worst triangles (face, mesher, place)
//        [--no-audit]                           skip the fold/flip/deviation audit (slow on big models)
//        [--stl prefix]                         write <prefix>.<label>.stl
//        [--json file]                          write the raw numbers
//
// Tolerances default to autoTessellation(model size), i.e. what the web viewer picks.
//
// Columns
//   minAng p5/p50   smallest interior angle per triangle, 5th percentile and median (60 = equilateral)
//   <20 / >120      share of triangles with an angle under 20 / over 120 degrees
//   Q               mean radius ratio 2·r_in/r_circ (1 = equilateral, 0 = degenerate)
//   stubs           B-rep edges whose first or last segment is under half its neighbour segment
//   seg ratio       adjacent segments along one B-rep edge: worst and 99th percentile length ratio
//   vtx ratio       longest / shortest mesh edge at one vertex, 95th and 99th percentile: the
//                   mesh-wide measure of how abruptly triangle size changes
import { readFileSync, writeFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate, type TessOptions } from "../src/mesh/tessellate.ts";
import { estimateBrepSize, autoTessellation } from "../src/step/measure.ts";
import { meshDefects } from "../src/mesh/diag.ts";
import { writeBinarySTL } from "../src/io/stl.ts";
import { auditMesh } from "./mesh-audit.ts";

const argv = process.argv.slice(2);
const flag = (n: string): boolean => argv.includes(`--${n}`);
const opt = (n: string): string | undefined => { const i = argv.indexOf(`--${n}`); return i >= 0 ? argv[i + 1] : undefined; };
const valued = new Set(["dev", "edge", "normal", "growth", "min-edge", "runs", "worst", "stl", "json"]);
const file = argv.find((a, i) => !a.startsWith("--") && !(i > 0 && valued.has(argv[i - 1]!.slice(2))));
if (!file) { console.error("usage: node test/mesh-quality.ts <file.step> [--compare] [--iso] ... (see file header)"); process.exit(2); }

const src = readFileSync(file, "utf8");
const pct = (a: number[], p: number): number => (a.length ? a[Math.min(a.length - 1, Math.floor(a.length * p))]! : NaN);

interface Shape { n: number; lt20: number; lt30: number; gt120: number; sumQ: number; angs: number[] }
const newShape = (): Shape => ({ n: 0, lt20: 0, lt30: 0, gt120: 0, sumQ: 0, angs: [] });

interface Report {
  label: string; tris: number; verts: number; ms: number;
  shape: Shape; byMesher: Map<string, Shape>; byKind: Map<string, Shape>;
  edges: number; stubs: number; segMax: number; segP99: number; vtxP95: number; vtxP99: number;
  open: number; nonManifold: number; warnings: number; untriangulated: number;
  audit?: { folds: number; flips: number; devRatio: number; tiltMaxDeg: number; slivers: number };
  worst: string[];
}

function run(label: string, extra: Partial<TessOptions>): Report {
  const brep = buildBrep(src);
  const est = estimateBrepSize(brep);
  const auto = autoTessellation(est?.diag ?? 100);
  const chordTol = Number(opt("dev") ?? auto.surfaceDeviation);
  const targetEdge = Number(opt("edge") ?? auto.maxEdge);
  const normalDev = Number(opt("normal") ?? 15) * Math.PI / 180;
  const base: TessOptions = { chordTol, targetEdge, normalDev, ...extra };
  // Time on fresh B-reps (tessellate caches nothing across calls, but reclassifies open shells).
  const runs = Math.max(1, Number(opt("runs") ?? 3));
  const times: number[] = [];
  for (let i = 0; i < runs; i++) {
    const b = i === 0 ? brep : buildBrep(src);
    const t0 = performance.now();
    tessellate(b, base);
    times.push(performance.now() - t0);
  }
  times.sort((a, b) => a - b);
  const mesherOf = new Map<number, string>();
  const b2 = buildBrep(src);
  const r = tessellate(b2, { ...base, trace: (fid, m) => mesherOf.set(fid, m), collectEdgePolylines: true });
  const P = r.mesh.positions, I = r.mesh.indices, nT = I.length / 3;
  const kindOf = new Map<number, string>();
  for (const s of b2.solids) for (const f of s.faces) kindOf.set(f.faceId, f.surfaceKind.replace(/_SURFACE.*$/, "").replace(/^\(complex\/bspline-surface\)$/, "B_SPLINE(rational)"));

  const shape = newShape(), byMesher = new Map<string, Shape>(), byKind = new Map<string, Shape>();
  const worstTris: { ang: number; t: number }[] = [];
  const nWorst = Number(opt("worst") ?? 0);
  const elen = (a: number, b: number): number => Math.hypot(P[b * 3]! - P[a * 3]!, P[b * 3 + 1]! - P[a * 3 + 1]!, P[b * 3 + 2]! - P[a * 3 + 2]!);
  const vMin = new Float64Array(P.length / 3).fill(Infinity), vMax = new Float64Array(P.length / 3);
  for (let t = 0; t < nT; t++) {
    const a = I[t * 3]!, b = I[t * 3 + 1]!, c = I[t * 3 + 2]!;
    const ab = elen(a, b), bc = elen(b, c), ca = elen(c, a);
    for (const [v, l1, l2] of [[a, ab, ca], [b, ab, bc], [c, bc, ca]] as const) {
      const lo = Math.min(l1, l2), hi = Math.max(l1, l2);
      if (lo < vMin[v]!) vMin[v] = lo; if (hi > vMax[v]!) vMax[v] = hi;
    }
    const s = [ab, bc, ca].sort((x, y) => x - y);
    const sp = (ab + bc + ca) / 2;
    const area = Math.sqrt(Math.max(0, sp * (sp - ab) * (sp - bc) * (sp - ca)));
    const sinMin = s[2]! * s[1]! > 0 ? (2 * area) / (s[2]! * s[1]!) : 0;
    const minA = Math.asin(Math.min(1, sinMin)) * 180 / Math.PI;
    const cosMax = s[0]! * s[1]! > 0 ? (s[0]! ** 2 + s[1]! ** 2 - s[2]! ** 2) / (2 * s[0]! * s[1]!) : -1;
    const maxA = Math.acos(Math.max(-1, Math.min(1, cosMax))) * 180 / Math.PI;
    const q = ab * bc * ca > 0 ? (8 * (sp - ab) * (sp - bc) * (sp - ca)) / (ab * bc * ca) : 0;
    const fid = r.faceOfTri[t]!;
    const m = mesherOf.get(fid) ?? "?", k = kindOf.get(fid) ?? "?";
    for (const acc of [shape, byMesher.get(m) ?? byMesher.set(m, newShape()).get(m)!, byKind.get(k) ?? byKind.set(k, newShape()).get(k)!]) {
      acc.n++; acc.sumQ += q; acc.angs.push(minA);
      if (minA < 20) acc.lt20++; if (minA < 30) acc.lt30++; if (maxA > 120) acc.gt120++;
    }
    if (nWorst) worstTris.push({ ang: minA, t });
  }
  for (const s of [shape, ...byMesher.values(), ...byKind.values()]) s.angs.sort((x, y) => x - y);
  const vr: number[] = [];
  for (let v = 0; v < vMin.length; v++) if (vMin[v]! > 0 && Number.isFinite(vMin[v]!)) vr.push(vMax[v]! / vMin[v]!);
  vr.sort((x, y) => x - y);

  // Seeding of the B-rep edges actually used by a face.
  const used = new Set<number>();
  for (const s of b2.solids) for (const f of s.faces) for (const lp of f.loops) for (const oe of lp.edges) used.add(oe.edgeId);
  let stubs = 0, nEdges = 0;
  const segR: number[] = [];
  for (const id of used) {
    const pl = r.edgePolylines?.get(id);
    if (!pl || pl.length < 2) continue;
    nEdges++;
    const seg: number[] = [];
    for (let i = 0; i + 1 < pl.length; i++) seg.push(Math.hypot(pl[i + 1]![0] - pl[i]![0], pl[i + 1]![1] - pl[i]![1], pl[i + 1]![2] - pl[i]![2]));
    if (seg.length < 2) continue;
    for (let i = 0; i + 1 < seg.length; i++) segR.push(Math.max(seg[i]!, seg[i + 1]!) / Math.max(Math.min(seg[i]!, seg[i + 1]!), 1e-12));
    if (seg[0]! < 0.5 * seg[1]! || seg[seg.length - 1]! < 0.5 * seg[seg.length - 2]!) stubs++;
  }
  segR.sort((x, y) => x - y);

  const defects = meshDefects(r.mesh, r.solidOfTri, r.openSolids);
  const rep: Report = {
    label, tris: nT, verts: P.length / 3, ms: times[times.length >> 1]!, shape, byMesher, byKind,
    edges: nEdges, stubs, segMax: segR[segR.length - 1] ?? 1, segP99: pct(segR, 0.99), vtxP95: pct(vr, 0.95), vtxP99: pct(vr, 0.99),
    open: defects.openEdges, nonManifold: defects.nonManifoldEdges, warnings: r.warnings.length,
    untriangulated: r.stats.facesTotal - r.stats.facesTessellated, worst: [],
  };
  if (!flag("no-audit")) {
    const a = auditMesh({ positions: P, indices: I, faceOfTri: r.faceOfTri, solidOfTri: r.solidOfTri }, b2, { chordTol });
    rep.audit = { folds: a.folds, flips: a.flips, devRatio: a.devRatio, tiltMaxDeg: a.tiltMaxDeg, slivers: a.slivers };
  }
  if (nWorst) {
    worstTris.sort((x, y) => x.ang - y.ang);
    for (const w of worstTris.slice(0, nWorst)) {
      const a = I[w.t * 3]!, fid = r.faceOfTri[w.t]!;
      rep.worst.push(`${w.ang.toFixed(2).padStart(6)}°  face #${fid} ${kindOf.get(fid)}/${mesherOf.get(fid)}  at (${P[a * 3]!.toFixed(3)}, ${P[a * 3 + 1]!.toFixed(3)}, ${P[a * 3 + 2]!.toFixed(3)})`);
    }
  }
  const stl = opt("stl");
  if (stl) writeFileSync(`${stl}.${label}.stl`, writeBinarySTL(r.mesh));
  if (label === labels[0]) console.log(`${file}: size ${est?.diag.toFixed(1)} mm, deviation ${chordTol} mm, max edge ${targetEdge} mm, normal ${(normalDev * 180 / Math.PI).toFixed(0)}°`);
  return rep;
}

const iso: Partial<TessOptions> = {
  isotropic: true,
  ...(opt("growth") ? { growthRate: Number(opt("growth")) } : {}),
  ...(opt("min-edge") ? { minEdge: Number(opt("min-edge")) } : {}),
};
const labels = flag("compare") ? ["default", "isotropic"] : [flag("iso") ? "isotropic" : "default"];
const reports = labels.map((l) => run(l, l === "isotropic" ? iso : {}));

const f1 = (x: number): string => x.toFixed(1), f2 = (x: number): string => x.toFixed(2);
const rows: [string, (r: Report) => string][] = [
  ["triangles", (r) => String(r.tris)],
  ["time, ms (median)", (r) => r.ms.toFixed(0)],
  ["min angle p5, deg", (r) => f1(pct(r.shape.angs, 0.05))],
  ["min angle p50, deg", (r) => f1(pct(r.shape.angs, 0.5))],
  ["min angle worst, deg", (r) => f2(r.shape.angs[0] ?? NaN)],
  ["triangles < 20 deg, %", (r) => f2(100 * r.shape.lt20 / r.shape.n)],
  ["triangles < 30 deg, %", (r) => f2(100 * r.shape.lt30 / r.shape.n)],
  ["triangles > 120 deg, %", (r) => f2(100 * r.shape.gt120 / r.shape.n)],
  ["mean quality Q", (r) => (r.shape.sumQ / r.shape.n).toFixed(3)],
  ["edge stubs / edges", (r) => `${r.stubs} / ${r.edges}`],
  ["seg ratio max / p99", (r) => `${f2(r.segMax)} / ${f2(r.segP99)}`],
  ["vtx ratio p95 / p99", (r) => `${f2(r.vtxP95)} / ${f2(r.vtxP99)}`],
  ["open / non-manifold edges", (r) => `${r.open} / ${r.nonManifold}`],
  ["warnings / faces unmeshed", (r) => `${r.warnings} / ${r.untriangulated}`],
  ["folds / flips", (r) => (r.audit ? `${r.audit.folds} / ${r.audit.flips}` : "-")],
  ["deviation / tolerance", (r) => (r.audit ? f2(r.audit.devRatio) : "-")],
  ["max tilt, deg", (r) => (r.audit ? f1(r.audit.tiltMaxDeg) : "-")],
  ["slivers < 1 deg", (r) => (r.audit ? String(r.audit.slivers) : "-")],
];
const w0 = 28, w = 16;
console.log("".padEnd(w0) + reports.map((r) => r.label.padStart(w)).join(""));
for (const [name, fn] of rows) console.log(name.padEnd(w0) + reports.map((r) => fn(r).padStart(w)).join(""));
if (flag("detail")) {
  for (const r of reports) for (const [title, m] of [["mesher", r.byMesher], ["surface", r.byKind]] as const) {
    console.log(`\n${r.label}: by ${title}`.padEnd(w0 + 2) + "tris".padStart(9) + "p5".padStart(7) + "p50".padStart(7) + "<20%".padStart(8) + ">120%".padStart(8) + "Q".padStart(7));
    for (const [k, s] of [...m].sort((x, y) => y[1].n - x[1].n)) {
      console.log(`  ${k}`.padEnd(w0 + 1) + String(s.n).padStart(9) + f1(pct(s.angs, 0.05)).padStart(7) + f1(pct(s.angs, 0.5)).padStart(7)
        + f2(100 * s.lt20 / s.n).padStart(8) + f2(100 * s.gt120 / s.n).padStart(8) + (s.sumQ / s.n).toFixed(3).padStart(7));
    }
  }
}
for (const r of reports) if (r.worst.length) { console.log(`\n${r.label}: worst triangles`); for (const l of r.worst) console.log("  " + l); }
const json = opt("json");
if (json) {
  writeFileSync(json, JSON.stringify(reports.map((r) => ({
    label: r.label, tris: r.tris, verts: r.verts, ms: r.ms,
    minAngP5: pct(r.shape.angs, 0.05), minAngP50: pct(r.shape.angs, 0.5), lt20: r.shape.lt20 / r.shape.n, gt120: r.shape.gt120 / r.shape.n,
    meanQ: r.shape.sumQ / r.shape.n, stubs: r.stubs, edges: r.edges, segMax: r.segMax, segP99: r.segP99, vtxP95: r.vtxP95, vtxP99: r.vtxP99,
    open: r.open, nonManifold: r.nonManifold, warnings: r.warnings, untriangulated: r.untriangulated, audit: r.audit,
  })), null, 1));
}
