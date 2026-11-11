// SPDX-License-Identifier: AGPL-3.0-only
// Characterization harness: freezes the exact output of the default import pipeline over the
// local .step corpus so refactors can be verified stage by stage.
//
//   node test/char.ts             compare against test/char-baseline.json (exit 1 on any drift)
//   node test/char.ts --update    regenerate the baseline (run after an INTENDED behavior change)
//   node test/char.ts --full      include the slow models (GoProHandlePod, ~70s)
//   node test/char.ts cube tool   restrict to named models
//
// Three layers of protection, strict to loose:
//  - hash: SHA-256 over positions quantized at 1e-9 mm + indices + faceOfTri + solidOfTri.
//    Pure-move refactors and exact-arithmetic changes must keep it BIT-IDENTICAL.
//  - dispatch: which mesher took how many faces (via TessOptions.trace). Guards the fallback
//    order — a face silently sliding from "band" to "grid" is a behavior change even if it looks
//    fine on one model.
//  - metrics: open/non-manifold edge counts, signed volume, triangle count. The review gate for
//    changes that legitimately alter float ordering (compare these by hand when the hash moves).
//  - audit (test/mesh-audit.ts): mesh QUALITY against the exact B-rep — same-face folds, in-face
//    flips, sampled surface deviation / chordTol, normal tilt, slivers. Drift in any of them is a
//    DIFF; a drift for the worse is tagged WORSE. Folds and flips are also a HARD GATE: any model
//    with either fails the run, even with --update (they are always bugs — a fold stays watertight
//    and on-surface, so nothing else catches it). Known open cases sit in KNOWN_DEFECTS with their
//    exact counts; a count above the entry fails, a count below asks to shrink the entry.
import { readFileSync, writeFileSync, readdirSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { createHash } from "node:crypto";
import { importStep } from "../src/index.ts";
import { buildBrep } from "../src/brep/build.ts";
import { auditMesh } from "./mesh-audit.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const baselinePath = join(root, "test", "char-baseline.json");
// Model discovery: repo root plus the two local (gitignored) corpus folders. Root models keep
// bare keys; folder models are keyed "folder/name" (OpenVessel etc. exist in both folders).
const MODEL_DIRS = ["", "sourcemodels", "newtestmodels"];
const STEP_RE = /\.(step|stp)$/i;
// Models slower than ~8s (measured 2026-11-10 on the reference Mac) stay out of the default
// per-stage check; --full runs them all.
const SLOW = new Set([
  "GoProHandlePod", "ETA 6497-1 Movement Corrected", "Tool-free Z-Axis Belt Tensioner for Voron 2.4",
  "newtestmodels/wallganizer", "newtestmodels/Ontos_V2", "newtestmodels/boomerang-v2",
  "newtestmodels/bottle-cage", "newtestmodels/SMILEY - Shrub X MHP Bolt Action - Release R1.2",
  "sourcemodels/nist_ctc_01_asme1_ap242-e1", "sourcemodels/nist_ctc_02_asme1_ap242-e2",
  "sourcemodels/nist_ctc_03_asme1_ap242-e2", "sourcemodels/nist_ctc_04_asme1_ap242-e1",
  "sourcemodels/nist_ctc_05_asme1_ap242-e1", "sourcemodels/nist_ftc_06_asme1_ap242-e2",
  "sourcemodels/nist_ftc_07_asme1_ap242-e2", "sourcemodels/nist_ftc_08_asme1_ap242-e2",
  "sourcemodels/nist_stc_07_asme1_ap242-e3", "sourcemodels/nist_stc_08_asme1_ap242-e3",
  "sourcemodels/weldment_asm_solid",
]);
// Small models also frozen through the OPTIONAL remesh pass (split/flip/collapse/smooth), so
// remesh.ts refactors are protected too. Kept to fast models — remesh multiplies runtime.
const REMESH = new Set(["cube", "cylinder", "cone", "sphere", "roundedCube", "cylinderWithHole", "LampenHalter", "chamferFillet"]);
// Open quality defects, exact counts (fold/flip). Remove an entry once its fix lands.
const KNOWN_DEFECTS: Record<string, { folds?: number; flips?: number; why: string }> = {
  // B-spline param grid: ONE median metric per direction; where the parametrisation speed varies
  // several-fold (rational fillets), triangles in the fast stretch span >90° of the fillet and face
  // against the surface. A plain 3D-size refinement doubles the triangles without fixing it — needs
  // a varying anisotropic metric (and fillet cross-arcs sampled to normalDeviation).
  "ETA 6497-1 Movement Corrected": { flips: 20, why: "bspline grid under-resolved fillet (face 66554)" },
  "newtestmodels/bottle-cage": { flips: 2, why: "bspline grid under-resolved fillets (25600, 61534)" },
  "newtestmodels/Ontos_V2": { flips: 4, why: "bspline grid under-resolved fillets (91400, 91926, 95636, 96057)" },
  // thinFace ribbon on 0.26mm-wide planar strips: rail pairing crosses (0.03° slivers), and the
  // orientation consensus flips part of the strip.
  "sourcemodels/OpenVessel": { folds: 3, flips: 12, why: "thinFace ribbon on planar strips 5890/5924/5958/5992" },
  "sourcemodels/OpenVessel_fusion360": { flips: 12, why: "thinFace ribbon on planar strips 4982-4985" },
  // Sphere lune whose two meridian edges meet at BOTH poles of the default frame, too wide (151°)
  // for reorientSphere; the pole rows collapse to slivers. A clear-axis frame fixed it but broke the
  // neighbouring lunes (>180° wide, no clearing axis) — needs per-lune frame choice.
  "sourcemodels/nist_ftc_06_asme1_ap242-e2": { folds: 1, why: "sphere lune #3188 gridded in its pole-cornered frame" },
};
const QUANT = 1e9; // 1e-9 mm — same identity the pipeline's own weld/audit quantization implies

interface MeshRecord {
  hash: string;
  verts: number;
  tris: number;
  openEdges: number;
  nonManifoldEdges: number;
  volume: number;
}

interface AuditRecord {
  folds: number;
  crossFace: number;
  flips: number;
  devRatio: number;
  tiltMaxDeg: number;
  slivers: number;
}

interface ModelRecord extends MeshRecord {
  audit?: AuditRecord;
  dispatch: Record<string, number>;
  skipped: Record<string, number>;
  remesh?: MeshRecord;
  /** Import threw — the message is baselined so a model starting/stopping to error is a DIFF. */
  error?: string;
}

function meshRecord(res: ReturnType<typeof importStep>): MeshRecord {
  const P = res.mesh.positions, I = res.mesh.indices;

  // --- hash: quantized positions + connectivity + attribution (byte order: little-endian x64)
  const q = new Float64Array(P.length);
  for (let i = 0; i < P.length; i++) q[i] = Math.round(P[i]! * QUANT);
  const h = createHash("sha256");
  const head = new Uint32Array([P.length / 3, I.length / 3]);
  h.update(new Uint8Array(head.buffer));
  h.update(new Uint8Array(q.buffer));
  h.update(new Uint8Array(I.buffer, I.byteOffset, I.byteLength));
  h.update(new Uint8Array(res.faceOfTri.buffer, res.faceOfTri.byteOffset, res.faceOfTri.byteLength));
  h.update(new Uint8Array(res.solidOfTri.buffer, res.solidOfTri.byteOffset, res.solidOfTri.byteLength));

  // --- watertightness counters + signed volume
  // Edge-use counts over a sorted key array: a Map stops at 2^24 entries, which a 58M-triangle
  // mesh (wallganizer) far exceeds.
  const KEY = 0x4000000; // vertex ids < 2^26; a*KEY+b exact below 2^53
  const keys = new Float64Array(I.length);
  for (let t = 0; t < I.length; t += 3) {
    const a = I[t]!, b = I[t + 1]!, c = I[t + 2]!;
    keys[t] = a < b ? a * KEY + b : b * KEY + a;
    keys[t + 1] = b < c ? b * KEY + c : c * KEY + b;
    keys[t + 2] = c < a ? c * KEY + a : a * KEY + c;
  }
  keys.sort();
  let open = 0, nonManifold = 0;
  for (let i = 0; i < keys.length;) {
    let j = i + 1;
    while (j < keys.length && keys[j] === keys[i]) j++;
    if (j - i === 1) open++; else if (j - i > 2) nonManifold++;
    i = j;
  }
  let vol6 = 0;
  for (let t = 0; t < I.length; t += 3) {
    const a = I[t]! * 3, b = I[t + 1]! * 3, c = I[t + 2]! * 3;
    vol6 += P[a]! * (P[b + 1]! * P[c + 2]! - P[b + 2]! * P[c + 1]!)
          - P[a + 1]! * (P[b]! * P[c + 2]! - P[b + 2]! * P[c]!)
          + P[a + 2]! * (P[b]! * P[c + 1]! - P[b + 1]! * P[c]!);
  }
  return {
    hash: h.digest("hex"),
    verts: P.length / 3,
    tris: I.length / 3,
    openEdges: open,
    nonManifoldEdges: nonManifold,
    volume: Number((vol6 / 6).toFixed(3)),
  };
}

function characterize(src: string, withRemesh: boolean): ModelRecord {
  const dispatch: Record<string, number> = {};
  const res = importStep(src, { trace: (_fid, mesher) => { dispatch[mesher] = (dispatch[mesher] ?? 0) + 1; } });

  // sort dispatch/skipped keys so the baseline JSON is diff-stable
  const sortRec = (o: Record<string, number>): Record<string, number> =>
    Object.fromEntries(Object.entries(o).sort(([x], [y]) => x.localeCompare(y)));
  const a = auditMesh({
    positions: res.mesh.positions, indices: res.mesh.indices, faceOfTri: res.faceOfTri, solidOfTri: res.solidOfTri,
    instanceOfTri: res.instanceOfTri, frames: res.instances.map((i) => i.frame),
  }, buildBrep(src), { chordTol: 0.01 }); // importStep's default surfaceDeviation
  const rec: ModelRecord = {
    ...meshRecord(res),
    dispatch: sortRec(dispatch),
    skipped: sortRec(res.stats.skipped),
    audit: {
      folds: a.folds, crossFace: a.crossFace, flips: a.flips,
      devRatio: Number(a.devRatio.toFixed(2)), tiltMaxDeg: Number(a.tiltMaxDeg.toFixed(1)), slivers: a.slivers,
    },
  };
  if (withRemesh) rec.remesh = meshRecord(importStep(src, { remesh: true }));
  return rec;
}

// --- CLI ------------------------------------------------------------------------------------
const argv = process.argv.slice(2);
const update = argv.includes("--update");
// --resume: with --update, skip models already in the baseline — recovery for a crashed long run
// (a giant model can OOM the process; run via npm run char which raises the heap limit).
const resume = argv.includes("--resume");
const full = argv.includes("--full");
const named = argv.filter((a) => !a.startsWith("--"));

let models: { key: string; file: string }[] = [];
for (const dir of MODEL_DIRS) {
  const abs = dir ? join(root, dir) : root;
  if (!existsSync(abs)) continue;
  for (const f of readdirSync(abs)) {
    if (!STEP_RE.test(f)) continue;
    const name = f.replace(STEP_RE, "");
    models.push({ key: dir ? `${dir}/${name}` : name, file: join(abs, f) });
  }
}
models.sort((a, b) => a.key.localeCompare(b.key));
if (named.length) models = models.filter((m) => named.some((q) => m.key === q || m.key.endsWith(`/${q}`) || m.key === q.replace(STEP_RE, "")));
else if (!full) models = models.filter((m) => !SLOW.has(m.key));

const baseline: Record<string, ModelRecord> = existsSync(baselinePath)
  ? JSON.parse(readFileSync(baselinePath, "utf8")) : {};
const results: Record<string, ModelRecord> = {};
let drift = 0, qualityFail = 0;

const writeBaseline = (): void => {
  const merged = { ...baseline, ...results };
  const sorted = Object.fromEntries(Object.entries(merged).sort(([x], [y]) => x.localeCompare(y)));
  writeFileSync(baselinePath, JSON.stringify(sorted, null, 2) + "\n");
};

for (const { key: m, file } of models) {
  if (update && resume && baseline[m]) { console.log(`skip ${m.padEnd(40)} (already baselined)`); continue; }
  const t0 = Date.now();
  let rec: ModelRecord;
  try {
    rec = characterize(readFileSync(file, "utf8"), REMESH.has(m));
  } catch (e) {
    rec = { hash: "", verts: 0, tris: 0, openEdges: 0, nonManifoldEdges: 0, volume: 0, dispatch: {}, skipped: {}, error: (e as Error).message };
  }
  const ms = Date.now() - t0;
  results[m] = rec;
  if (rec.audit) {
    const known = KNOWN_DEFECTS[m];
    for (const k of ["folds", "flips"] as const) {
      const n = rec.audit[k], allowed = known?.[k] ?? 0;
      if (n > allowed) { qualityFail++; console.log(`QUAL ${m.padEnd(40)} ${k}=${n}${allowed ? ` (known: ${allowed})` : ""} — always a mesher bug, see test/mesh-audit.ts`); }
      else if (n < allowed) console.log(`QUAL ${m.padEnd(40)} ${k}=${n} < known ${allowed}: shrink/remove its KNOWN_DEFECTS entry`);
    }
  }
  const base = baseline[m];
  if (update || !base) {
    const tag = rec.error ? `ERROR ${rec.error.slice(0, 60)}` : `tris=${String(rec.tris).padStart(7)} open=${rec.openEdges} nm=${rec.nonManifoldEdges} vol=${rec.volume}`;
    console.log(`${update ? "base" : "NEW "} ${m.padEnd(40)} ${tag} ${ms}ms`);
    if (update) writeBaseline(); // incremental: a crash (OOM on a giant model) loses one model, not the run
    if (!update && !base) drift++;
    continue;
  }
  const diffs: string[] = [];
  if ((rec.error ?? "") !== (base.error ?? "")) diffs.push(`error "${base.error ?? ""}" -> "${rec.error ?? ""}"`);
  if (rec.hash !== base.hash) diffs.push("hash");
  for (const k of ["verts", "tris", "openEdges", "nonManifoldEdges", "volume"] as const) {
    if (rec[k] !== base[k]) diffs.push(`${k} ${base[k]} -> ${rec[k]}`);
  }
  if (JSON.stringify(rec.dispatch) !== JSON.stringify(base.dispatch)) {
    diffs.push(`dispatch ${JSON.stringify(base.dispatch)} -> ${JSON.stringify(rec.dispatch)}`);
  }
  if (JSON.stringify(rec.skipped) !== JSON.stringify(base.skipped)) {
    diffs.push(`skipped ${JSON.stringify(base.skipped)} -> ${JSON.stringify(rec.skipped)}`);
  }
  if (rec.audit && base.audit) {
    // higher is worse for every audit metric
    for (const k of ["folds", "crossFace", "flips", "devRatio", "tiltMaxDeg", "slivers"] as const) {
      const was = base.audit[k], now = rec.audit[k];
      if (now !== was) diffs.push(`${now > was ? "WORSE " : ""}audit.${k} ${was} -> ${now}`);
    }
  } else if (rec.audit && !base.audit) diffs.push("audit missing from baseline");
  if (base.remesh && rec.remesh) {
    if (rec.remesh.hash !== base.remesh.hash) diffs.push("remesh hash");
    for (const k of ["verts", "tris", "openEdges", "nonManifoldEdges", "volume"] as const) {
      if (rec.remesh[k] !== base.remesh[k]) diffs.push(`remesh.${k} ${base.remesh[k]} -> ${rec.remesh[k]}`);
    }
  }
  if (diffs.length) { drift++; console.log(`DIFF ${m.padEnd(40)} ${diffs.join("; ")}`); }
  else console.log(`ok   ${m.padEnd(40)} tris=${String(rec.tris).padStart(7)} ${ms}ms`);
}

if (qualityFail) {
  if (update) writeBaseline();
  console.error(`${qualityFail} quality defect(s) (fold/flip) — fix them; KNOWN_DEFECTS is only for documented open cases`);
  process.exit(1);
}
if (update) {
  // merge: keep baseline entries for models not run this time (e.g. slow ones skipped)
  writeBaseline();
  console.log(`baseline written: ${Object.keys({ ...baseline, ...results }).length} models -> test/char-baseline.json`);
} else if (drift) {
  console.error(`${drift} model(s) drifted from baseline (or missing). If intended: node test/char.ts --update${full ? " --full" : ""}`);
  process.exit(1);
} else {
  console.log(`char: all ${models.length} models match baseline`);
}
