// SPDX-License-Identifier: AGPL-3.0-only
// Per-face tessellation overrides (ImportOptions.faceOverrides): a listed face is meshed with its
// own tolerances while the mesh stays watertight, faces away from it are untouched, ids survive,
// and an empty override set is bit-identical to no option at all.
import { readFileSync } from "node:fs";
import { importStep, type ImportResult } from "../src/index.ts";

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}${detail ? ` (${detail})` : ""}`);
}

const trisOf = (r: ImportResult, fid: number): number[] => {
  const out: number[] = [];
  for (let t = 0; t < r.faceOfTri.length; t++) if (r.faceOfTri[t] === fid) out.push(t);
  return out;
};
const normal = (r: ImportResult, t: number): [number, number, number] => {
  const p = r.mesh.positions, i = r.mesh.indices;
  const a = 3 * i[3 * t]!, b = 3 * i[3 * t + 1]!, c = 3 * i[3 * t + 2]!;
  const u = [p[b]! - p[a]!, p[b + 1]! - p[a + 1]!, p[b + 2]! - p[a + 2]!];
  const v = [p[c]! - p[a]!, p[c + 1]! - p[a + 1]!, p[c + 2]! - p[a + 2]!];
  const n: [number, number, number] = [u[1]! * v[2]! - u[2]! * v[1]!, u[2]! * v[0]! - u[0]! * v[2]!, u[0]! * v[1]! - u[1]! * v[0]!];
  const l = Math.hypot(...n) || 1;
  return [n[0] / l, n[1] / l, n[2] / l];
};
/** Angles (deg) between the normals of every two triangles of `fid` sharing an edge, sorted. */
const turns = (r: ImportResult, fid: number): number[] => {
  const tris = trisOf(r, fid);
  const byEdge = new Map<string, number[]>();
  for (const t of tris) {
    for (let k = 0; k < 3; k++) {
      const a = r.mesh.indices[3 * t + k]!, b = r.mesh.indices[3 * t + ((k + 1) % 3)]!;
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      const l = byEdge.get(key);
      if (l) l.push(t); else byEdge.set(key, [t]);
    }
  }
  const out: number[] = [];
  for (const l of byEdge.values()) {
    if (l.length !== 2) continue;
    const [n0, n1] = [normal(r, l[0]!), normal(r, l[1]!)];
    const c = Math.max(-1, Math.min(1, n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]));
    out.push(Math.acos(c) * 180 / Math.PI);
  }
  return out.sort((x, y) => x - y);
};
const pct = (v: number[], q: number): number => v[Math.min(v.length - 1, Math.floor(q * v.length))] ?? 0;
/** Faces that share no vertex position with `fid` (in result `r`). */
const farFaces = (r: ImportResult, fid: number): number[] => {
  const key = (v: number): string => {
    const p = r.mesh.positions;
    return `${p[3 * v]!.toFixed(6)},${p[3 * v + 1]!.toFixed(6)},${p[3 * v + 2]!.toFixed(6)}`;
  };
  const mine = new Set<string>();
  for (const t of trisOf(r, fid)) for (let k = 0; k < 3; k++) mine.add(key(r.mesh.indices[3 * t + k]!));
  const touching = new Set<number>([fid]);
  for (let t = 0; t < r.faceOfTri.length; t++) {
    for (let k = 0; k < 3; k++) if (mine.has(key(r.mesh.indices[3 * t + k]!))) touching.add(r.faceOfTri[t]!);
  }
  return [...r.faces.keys()].filter((f) => !touching.has(f));
};

// This suite runs in the publish gate (CI), so it may only read the COMMITTED fixtures
// (cube/cylinder/sphere/everything.step) — local corpus models don't exist on the runner.

// --- An empty override set is no override at all: bit-identical output.
{
  const src = readFileSync(new URL("../everything.step", import.meta.url), "utf8");
  const a = importStep(src);
  const b = importStep(src, { faceOverrides: {} });
  const c = importStep(src, { faceOverrides: new Map() });
  const same = (x: ImportResult, y: ImportResult): boolean =>
    x.mesh.positions.length === y.mesh.positions.length && x.mesh.positions.every((v, i) => v === y.mesh.positions[i])
    && x.mesh.indices.every((v, i) => v === y.mesh.indices[i]) && x.faceOfTri.every((v, i) => v === y.faceOfTri[i]);
  check("empty overrides (object) bit-identical", same(a, b));
  check("empty overrides (Map) bit-identical", same(a, c));
}

// --- One curved face refined to 2°: a trimmed cylinder/torus among many faces (everything.step,
// 25 of 34 faces share nothing with it) and a full seamed tube (cylinder.step).
for (const model of ["everything.step", "cylinder.step"]) {
  const src = readFileSync(new URL(`../${model}`, import.meta.url), "utf8");
  const r0 = importStep(src);
  const curved = [...r0.faces.entries()].filter(([, f]) => f.type === "cylinder" || f.type === "torus");
  check(`${model}: has a curved face`, curved.length > 0);
  if (curved.length === 0) continue;
  const [fid] = curved[0]!;
  const r1 = importStep(src, { faceOverrides: { [fid]: { normalDeviation: 2, maxEdge: 0.25 } } });
  check(`${model}: watertight with the override`, r1.diagnostics.ok && r1.diagnostics.openEdges === 0,
    `open ${r1.diagnostics.openEdges}, nonmanifold ${r1.diagnostics.nonManifoldEdges}`);
  const n0 = trisOf(r0, fid).length, n1 = trisOf(r1, fid).length;
  check(`${model}: the face got finer`, n1 > 2 * n0, `${n0} -> ${n1} triangles`);
  // The 99.5th percentile, not the max: a CDT leaves a few boundary slivers (areas ~1e-4 mm²)
  // whose normals are noise, with or without the override.
  const [a0, a1] = [turns(r0, fid), turns(r1, fid)];
  check(`${model}: its facets turn by <= 2 deg (+ slack)`, pct(a1, 0.995) <= 2.5,
    `p99.5 ${pct(a0, 0.995).toFixed(2)} -> ${pct(a1, 0.995).toFixed(2)} deg, max ${a1[a1.length - 1]!.toFixed(2)}`);
  check(`${model}: face ids unchanged`, [...r0.faces.keys()].sort().join() === [...r1.faces.keys()].sort().join());
  const far = farFaces(r0, fid);
  const changed = far.filter((f) => trisOf(r0, f).length !== trisOf(r1, f).length);
  check(`${model}: faces away from it untouched`, changed.length === 0, `${far.length} far faces, changed: ${changed.join(",")}`);
}

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("face-overrides: all passed");
