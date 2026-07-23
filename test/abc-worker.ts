// SPDX-License-Identifier: AGPL-3.0-only
// Persistent batch worker for the ABC-dataset survey (see abc-batch.ts).
//
// Long-lived process: reads one STEP file path per line on stdin, imports it with
// meshStep (decomposed buildBrep -> tessellate -> orient so the file is parsed once),
// checks watertightness + face coverage, classifies the failure cause, and writes one
// newline-delimited JSON verdict (sentinel-prefixed) per file on stdout. The orchestrator
// owns process isolation: it kills+respawns this worker on a per-file hang (timeout) or a
// native crash / OOM, so a single pathological model can never stall or corrupt the run.
import { readFileSync } from "node:fs";
import { createInterface } from "node:readline";
import { buildBrep, type BrepModel } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { orientConsistent } from "../src/mesh/orient.ts";
import { estimateBrepSize } from "../src/step/measure.ts";

const SENT = "@@ABC@@";
const errMsg = (e: unknown): string => (e instanceof Error ? e.message : String(e));

// --- model scale from the library's geometric size estimate -> scale-relative tolerances, so a
// 500 mm part and a 5 mm part get comparably fine meshes and neither explodes nor coarsens.
// (An edge-ENDPOINT bbox is not enough: closed circles hide their radius, edge-less solids hide
// everything — chunk-1 had 18 false timeouts and one array-overflow crash from exactly that.) ---
const brepDiag = (brep: BrepModel): number => estimateBrepSize(brep)?.diag ?? 0;

interface WT { open: number; nm: number; openEdgesOfFace: Map<number, number> }
function watertight(m: { positions: Float64Array; indices: Uint32Array }, faceOfTri: Uint32Array, skip?: (t: number) => boolean): WT {
  const K = 0x40000000;
  const inc = new Map<number, { n: number; fwd: number; tri: number }>();
  const nt = m.indices.length / 3;
  for (let t = 0; t < nt; t++) {
    if (skip?.(t)) continue;
    const a = m.indices[t * 3]!, b = m.indices[t * 3 + 1]!, c = m.indices[t * 3 + 2]!;
    for (const [x, y] of [[a, b], [b, c], [c, a]] as const) {
      const k = x < y ? x * K + y : y * K + x;
      const e = inc.get(k);
      if (e) { e.n++; if (x < y) e.fwd++; } else inc.set(k, { n: 1, fwd: x < y ? 1 : 0, tri: t });
    }
  }
  let open = 0, nm = 0;
  const openEdgesOfFace = new Map<number, number>();
  for (const e of inc.values()) {
    if (e.n === 2) continue;
    if (e.n === 1) { open++; const f = faceOfTri[e.tri]!; openEdgesOfFace.set(f, (openEdgesOfFace.get(f) ?? 0) + 1); }
    else if (e.n % 2 === 0 && 2 * e.fwd === e.n) continue; // balanced self-touching contact (faithful topology)
    else nm++;
  }
  return { open, nm, openEdgesOfFace };
}

const normKind = (k: string): string => {
  if (!k || k === "?") return "UNKNOWN";
  if (/SPLINE|complex/i.test(k)) return "B_SPLINE_SURFACE";
  return k;
};

function producer(src: string): string {
  const m = src.match(/originating_system\s*[^']*'([^']+)'/i)
    ?? src.match(/'([^']*(?:ONSHAPE|Fusion|Autodesk|SOLIDWORKS|FreeCAD|OpenSCAD|Rhino|Inventor|CATIA|NX|Creo|ST-DEVELOPER|SpaceClaim|Parasolid)[^']*)'/i);
  return m ? m[1]!.slice(0, 48) : "?";
}

interface Rec {
  status: string;         // PASS | FAIL | EMPTY | ERR
  bucket: string;         // grouping key for the report
  phase?: string;         // read | parse | tess (for ERR)
  err?: string;
  open?: number; nm?: number;
  tris?: number; facesTotal?: number; facesTess?: number;
  skipped?: Record<string, number>;
  openKinds?: Record<string, number>;
  domOpenKind?: string;
  diag?: number; solids?: number; producer?: string; ms: number;
}

function processOne(path: string): Rec {
  const t0 = Date.now();
  let src: string;
  try { src = readFileSync(path, "utf8"); }
  catch (e) { return { status: "ERR", bucket: "error:read", phase: "read", err: errMsg(e), ms: Date.now() - t0 }; }

  let brep: BrepModel;
  try { brep = buildBrep(src); }
  catch (e) { return { status: "ERR", bucket: "error:parse", phase: "parse", err: errMsg(e), producer: producer(src), ms: Date.now() - t0 }; }

  const diag = brepDiag(brep);
  const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
  const targetEdge = diag > 0 ? 0.05 * diag : 1;

  let res: ReturnType<typeof tessellate>;
  try { res = tessellate(brep, { chordTol, targetEdge, normalDev: 15 * Math.PI / 180 }); }
  catch (e) { return { status: "ERR", bucket: "error:tess", phase: "tess", err: errMsg(e), diag, solids: brep.solids.length, producer: producer(src), ms: Date.now() - t0 }; }
  try { orientConsistent(res.mesh, res.solidOfTri); } catch { /* orientation is cosmetic for watertightness */ }

  const tris = res.mesh.indices.length / 3;
  const prod = producer(src);
  const skippedTotal = res.stats.facesTotal - res.stats.facesTessellated;
  if (tris === 0) {
    return { status: "EMPTY", bucket: "empty", tris: 0, facesTotal: res.stats.facesTotal, facesTess: res.stats.facesTessellated, skipped: res.stats.skipped, diag, solids: brep.solids.length, producer: prod, ms: Date.now() - t0 };
  }

  const openSet = new Set(res.openSolids ?? []);
  const wt = watertight(res.mesh, res.faceOfTri, openSet.size ? (t) => openSet.has(res.solidOfTri[t]!) : undefined);

  // dominant surface kind carrying the open edges (root-cause attribution)
  let domOpenKind: string | undefined;
  let openKinds: Record<string, number> | undefined;
  if (wt.open > 0) {
    const faceKind = new Map<number, string>();
    for (const s of brep.solids) for (const f of s.faces) faceKind.set(f.faceId, f.surfaceKind);
    const byKind: Record<string, number> = {};
    for (const [f, n] of wt.openEdgesOfFace) { const k = normKind(faceKind.get(f) ?? "?"); byKind[k] = (byKind[k] ?? 0) + n; }
    openKinds = byKind;
    domOpenKind = Object.entries(byKind).sort((a, b) => b[1] - a[1])[0]?.[0];
  }

  const ok = wt.open === 0 && wt.nm === 0 && skippedTotal === 0;
  let bucket = "ok";
  if (!ok) {
    const skippedKinds = Object.keys(res.stats.skipped);
    const unsupported = skippedKinds.filter((k) => k !== "untriangulated");
    if (unsupported.length) bucket = `unsupported-surface:${normKind(unsupported[0]!)}`;
    else if (skippedKinds.includes("untriangulated")) bucket = "untriangulated-face";
    else if (wt.nm > 0 && wt.open === 0) bucket = "nonmanifold";
    else if (wt.open > 0) bucket = `seam-leak:${domOpenKind}`;
    else if (wt.nm > 0) bucket = "nonmanifold";
    else bucket = "other";
  }

  return {
    status: ok ? "PASS" : "FAIL", bucket,
    open: wt.open, nm: wt.nm, tris, facesTotal: res.stats.facesTotal, facesTess: res.stats.facesTessellated,
    skipped: skippedTotal > 0 ? res.stats.skipped : undefined,
    openKinds, domOpenKind, diag, solids: brep.solids.length, producer: prod, ms: Date.now() - t0,
  };
}

const rl = createInterface({ input: process.stdin });
rl.on("line", (line) => {
  const path = line.trim();
  if (!path) return;
  let rec: Rec;
  try { rec = processOne(path); }
  catch (e) { rec = { status: "ERR", bucket: "error:worker", err: errMsg(e), ms: 0 }; }
  process.stdout.write(SENT + JSON.stringify(rec) + "\n");
});
process.stdout.write(SENT + JSON.stringify({ ready: true }) + "\n");
