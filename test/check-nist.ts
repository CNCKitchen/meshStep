// SPDX-License-Identifier: AGPL-3.0-only
// NIST MBE PMI conformance set (CTC/FTC/STC, AP242). Picks maxEdge from each part's bbox so big
// parts stay fast, reports watertightness AND a surface-kind coverage report: which surface entity
// types appear, and which faces emit ZERO triangles (= unhandled/failed surfaces = real gaps).
import { readFileSync, readdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { buildBrep } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const dir = join(root, "sourceModels");

function producer(src: string): string {
  const m = src.match(/originating_system\s*\*?\/?\s*'([^']+)'/i)
    ?? src.match(/'([^']*(?:ONSHAPE|Fusion|Autodesk|SOLIDWORKS|FreeCAD|ST-DEVELOPER|Creo|Pro\/E|NX|CATIA|Inventor|Rhino|HOOPS)[^']*)'/i);
  return m ? m[1]!.slice(0, 28) : "?";
}

function meshStats(I: Uint32Array | number[], P: Float32Array | number[]) {
  const nt = I.length / 3, und = new Map<string, number>();
  let sl = 0;
  for (let t = 0; t < nt; t++) {
    for (let e = 0; e < 3; e++) { const a = I[t * 3 + e]!, b = I[t * 3 + (e + 1) % 3]!; const k = a < b ? `${a}_${b}` : `${b}_${a}`; und.set(k, (und.get(k) ?? 0) + 1); }
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const ang = (p: number, q: number, r: number) => { const u = [P[q]! - P[p]!, P[q + 1]! - P[p + 1]!, P[q + 2]! - P[p + 2]!], v = [P[r]! - P[p]!, P[r + 1]! - P[p + 1]!, P[r + 2]! - P[p + 2]!]; const du = Math.hypot(u[0]!, u[1]!, u[2]!), dv = Math.hypot(v[0]!, v[1]!, v[2]!); return Math.acos(Math.max(-1, Math.min(1, (u[0]! * v[0]! + u[1]! * v[1]! + u[2]! * v[2]!) / (du * dv || 1e-9)))); };
    if (Math.min(ang(a, b, c), ang(b, c, a), ang(c, a, b)) < 20 * Math.PI / 180) sl++;
  }
  let open = 0, nm = 0; for (const c of und.values()) { if (c === 1) open++; else if (c > 2) nm++; }
  return { nt, open, nm, sl: (100 * sl / nt).toFixed(1) };
}

const emptyByKind = new Map<string, number>();   // surfaceKind -> # faces that produced 0 triangles
const totalByKind = new Map<string, number>();   // surfaceKind -> # faces total

const files = readdirSync(dir).filter((x) => x.toLowerCase().startsWith("nist") && (x.toLowerCase().endsWith(".stp") || x.toLowerCase().endsWith(".step"))).sort();
for (const f of files) {
  const src = readFileSync(join(dir, f), "utf8");
  try {
    const brep = buildBrep(src);
    // Bounding box from edge endpoints (cheap; already in mm).
    let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const e of brep.edges.values()) for (const v of [e.v0, e.v1]) for (let k = 0; k < 3; k++) { if (v[k]! < lo[k]!) lo[k] = v[k]!; if (v[k]! > hi[k]!) hi[k] = v[k]!; }
    // Tessellated-only bodies have no B-rep edges (diag = Infinity); resolution is irrelevant there.
    const diag = Number.isFinite(hi[0]) ? Math.hypot(hi[0]! - lo[0]!, hi[1]! - lo[1]!, hi[2]! - lo[2]!) : 0;
    // Pick a coarse-but-faithful resolution from the bbox so big parts don't explode.
    const maxEdge = Math.max(0.5, diag / 120);
    const surfaceDev = Math.max(0.01, diag / 4000);

    const t0 = Date.now();
    const res = tessellate(brep, { chordTol: surfaceDev, targetEdge: maxEdge, normalDev: 15 * Math.PI / 180 });
    const ms = Date.now() - t0;

    // Per-face triangle counts -> which surface kinds yield nothing.
    const triPerFace = new Map<number, number>();
    for (let t = 0; t < res.faceOfTri.length; t++) triPerFace.set(res.faceOfTri[t]!, (triPerFace.get(res.faceOfTri[t]!) ?? 0) + 1);
    for (const solid of brep.solids) for (const face of solid.faces) {
      const kind = face.surfaceKind;
      totalByKind.set(kind, (totalByKind.get(kind) ?? 0) + 1);
      if (!(triPerFace.get(face.faceId)! > 0)) emptyByKind.set(kind, (emptyByKind.get(kind) ?? 0) + 1);
    }

    const s = meshStats(res.mesh.indices, res.mesh.positions);
    const tag = s.open === 0 && s.nm === 0 ? "PASS" : "FAIL";
    console.log(`${tag} ${f.slice(0, 30).padEnd(30)} bbox=${diag.toFixed(0).padStart(4)}mm me=${maxEdge.toFixed(2).padStart(5)} tris=${String(s.nt).padStart(7)} open=${String(s.open).padStart(4)} nm=${String(s.nm).padStart(3)} sl%=${s.sl.padStart(4)} ${String(ms).padStart(5)}ms [${producer(src)}]`);
  } catch (e) { console.log(`ERR  ${f}: ${(e as Error).message}`); }
}

console.log("\n--- surface-kind coverage (empty-faces / total-faces across all NIST parts) ---");
for (const [kind, total] of [...totalByKind.entries()].sort((a, b) => (emptyByKind.get(b[0]) ?? 0) - (emptyByKind.get(a[0]) ?? 0))) {
  const empty = emptyByKind.get(kind) ?? 0;
  console.log(`${empty > 0 ? "GAP " : "ok  "} ${kind.padEnd(34)} empty=${String(empty).padStart(5)} / ${total}`);
}
