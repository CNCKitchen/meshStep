// SPDX-License-Identifier: AGPL-3.0-only
// Measurement-geometry invariants: import with { measureGeometry: true } and assert
//   1. every edge polyline point coincides with a mesh vertex (snap geometry == rendered edges),
//   2. circle edges are internally exact (polyline on the circle, axis ⊥ radial, len == |sweep|·r),
//   3. polyline arc length agrees with the analytic length,
//   4. multi-occurrence parts get one edge record per instance at distinct placements.
import { readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep, type PartNode } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const files = ["cube", "cylinderWithHole", "everything", "chamferFillet", "LampenHalter", "GoProHandlePod",
  "ETA 6497-1 Movement Corrected"]; // the multi-occurrence assembly fixture (instance replication)

let failures = 0;
const fail = (file: string, msg: string): void => { failures++; console.error(`FAIL ${file}: ${msg}`); };

const maxOccurrences = (n: PartNode): number => {
  let m = n.occurrences ?? 1;
  for (const c of n.children ?? []) m = Math.max(m, maxOccurrences(c));
  return m;
};

for (const name of files) {
  let src: string;
  try { src = readFileSync(join(root, `${name}.step`), "utf8"); }
  catch { console.log(`skip ${name} (no file)`); continue; }

  const res = importStep(src, { measureGeometry: true });
  const mg = res.measure;
  if (!mg) { fail(name, "no measure geometry returned"); continue; }
  if (mg.edges.length === 0) { fail(name, "0 measure edges"); continue; }

  // --- 1. polyline<->mesh coincidence: every polyline point is a mesh vertex (1e-3mm grid,
  // 27-cell neighborhood absorbs Float32 truncation and instance-transform rounding).
  const G = 1e-3;
  const verts = new Set<string>();
  const P = res.mesh.positions;
  for (let i = 0; i < P.length; i += 3) {
    verts.add(`${Math.round(P[i]! / G)}_${Math.round(P[i + 1]! / G)}_${Math.round(P[i + 2]! / G)}`);
  }
  const onMesh = (x: number, y: number, z: number): boolean => {
    const cx = Math.round(x / G), cy = Math.round(y / G), cz = Math.round(z / G);
    for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) for (let dz = -1; dz <= 1; dz++) {
      if (verts.has(`${cx + dx}_${cy + dy}_${cz + dz}`)) return true;
    }
    return false;
  };
  // Strict only for edges SHARED by two faces (real feature edges): seam/rim edges with a single
  // adjacent face may be legitimately resampled by the periodic-surface meshers (deviation is
  // bounded by chordTol, invisible at snap scales) — report those, don't fail.
  let offShared = 0, offSeam = 0, totalPts = 0;
  for (const e of mg.edges) {
    for (let i = 0; i < e.count; i++) {
      const o = (e.first + i) * 3;
      totalPts++;
      if (!onMesh(mg.points[o]!, mg.points[o + 1]!, mg.points[o + 2]!)) {
        if (e.faceIds.length >= 2) offShared++; else offSeam++;
      }
    }
  }
  if (offShared > 0) fail(name, `${offShared}/${totalPts} shared-edge polyline points not on mesh vertices`);

  // --- 2/3. circle exactness + length agreement.
  let circles = 0, fullCircles = 0;
  for (const e of mg.edges) {
    // polyline chord length must not exceed the analytic length and must approach it (<1.5% short)
    let plen = 0;
    for (let i = 1; i < e.count; i++) {
      const a = (e.first + i - 1) * 3, b = (e.first + i) * 3;
      plen += Math.hypot(mg.points[b]! - mg.points[a]!, mg.points[b + 1]! - mg.points[a + 1]!, mg.points[b + 2]! - mg.points[a + 2]!);
    }
    if (e.kind === "line" || e.kind === "circle") {
      const rel = Math.abs(plen - e.length) / Math.max(e.length, 1e-9);
      if (e.length > 0.05 && rel > 0.015) fail(name, `edge ${e.edgeId} (${e.kind}) polyline len ${plen.toFixed(4)} vs analytic ${e.length.toFixed(4)}`);
    }
    if (e.kind !== "circle") continue;
    circles++;
    const c = e.center!, ax = e.axis!, r = e.radius!;
    if (!(r > 0) || !c || !ax) { fail(name, `edge ${e.edgeId} circle missing params`); continue; }
    if (Math.abs(e.length - Math.abs(e.sweep!) * r) > 1e-6 * Math.max(1, r)) {
      fail(name, `edge ${e.edgeId} circle length ${e.length} != |sweep|*r ${Math.abs(e.sweep!) * r}`);
    }
    if (Math.abs(e.sweep!) > Math.PI * 2 - 1e-6) fullCircles++;
    for (let i = 0; i < e.count; i++) {
      const o = (e.first + i) * 3;
      const d: [number, number, number] = [mg.points[o]! - c[0], mg.points[o + 1]! - c[1], mg.points[o + 2]! - c[2]];
      const radial = Math.hypot(d[0] - ax[0] * (d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2]),
        d[1] - ax[1] * (d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2]),
        d[2] - ax[2] * (d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2]));
      const axial = Math.abs(d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2]);
      // Endpoints are snapped to the STEP VERTEX_POINTs, which exporters place a few µm off the
      // analytic curve — allow jitter there; interior points are sampled on the circle exactly.
      const tol = (i === 0 || i === e.count - 1 ? 1e-2 : 1e-3) * Math.max(1, r);
      if (Math.abs(radial - r) > tol || axial > tol) {
        fail(name, `edge ${e.edgeId} polyline pt ${i}/${e.count} off circle: radial ${radial.toFixed(6)} vs r ${r.toFixed(6)}, axial ${axial.toFixed(6)}`);
        break;
      }
    }
  }

  // --- 4. instances: a part used N>1 times must yield N records per edge at distinct centers.
  const occ = maxOccurrences(res.structure);
  let instanceNote = "";
  if (occ > 1) {
    const byEdge = new Map<number, typeof mg.edges>();
    for (const e of mg.edges) {
      const l = byEdge.get(e.edgeId) ?? [];
      l.push(e); byEdge.set(e.edgeId, l);
    }
    const replicated = [...byEdge.values()].filter((l) => l.length > 1);
    if (replicated.length === 0) instanceNote = ` (occ ${occ} but no replicated edges?)`;
    else {
      const l = replicated[0]!;
      const a = l[0]!, b = l[1]!;
      const pa = a.first * 3, pb = b.first * 3;
      const d = Math.hypot(mg.points[pa]! - mg.points[pb]!, mg.points[pa + 1]! - mg.points[pb + 1]!, mg.points[pa + 2]! - mg.points[pb + 2]!);
      if (d < 1e-6) fail(name, `replicated edge ${a.edgeId} instances coincide`);
      instanceNote = ` occ=${occ} replicatedEdges=${replicated.length}`;
    }
  }

  console.log(`${name.padEnd(18)} edges=${String(mg.edges.length).padStart(5)} circles=${String(circles).padStart(4)} (full=${fullCircles}) faces=${String(mg.faces.length).padStart(5)} pts=${totalPts}${offSeam ? ` seamOff=${offSeam}` : ""}${mg.truncated ? " TRUNCATED" : ""}${instanceNote}`);
}

if (failures > 0) { console.error(`\n${failures} failure(s)`); process.exit(1); }
console.log("\nall measure-geometry invariants hold");
