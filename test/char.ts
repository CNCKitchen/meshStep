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
import { readFileSync, writeFileSync, readdirSync, existsSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { createHash } from "node:crypto";
import { importStep } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const baselinePath = join(root, "test", "char-baseline.json");
const SLOW = new Set(["GoProHandlePod"]);
// Small models also frozen through the OPTIONAL remesh pass (split/flip/collapse/smooth), so
// remesh.ts refactors are protected too. Kept to fast models — remesh multiplies runtime.
const REMESH = new Set(["cube", "cylinder", "cone", "sphere", "roundedCube", "cylinderWithHole", "LampenHalter", "chamferFillet"]);
const QUANT = 1e9; // 1e-9 mm — same identity the pipeline's own weld/audit quantization implies

interface MeshRecord {
  hash: string;
  verts: number;
  tris: number;
  openEdges: number;
  nonManifoldEdges: number;
  volume: number;
}

interface ModelRecord extends MeshRecord {
  dispatch: Record<string, number>;
  skipped: Record<string, number>;
  remesh?: MeshRecord;
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
  const KEY = 0x4000000; // vertex ids < 2^26; a*KEY+b exact below 2^53
  const use = new Map<number, number>();
  for (let t = 0; t < I.length; t += 3) {
    const a = I[t]!, b = I[t + 1]!, c = I[t + 2]!;
    const k0 = a < b ? a * KEY + b : b * KEY + a;
    const k1 = b < c ? b * KEY + c : c * KEY + b;
    const k2 = c < a ? c * KEY + a : a * KEY + c;
    use.set(k0, (use.get(k0) ?? 0) + 1);
    use.set(k1, (use.get(k1) ?? 0) + 1);
    use.set(k2, (use.get(k2) ?? 0) + 1);
  }
  let open = 0, nonManifold = 0;
  for (const n of use.values()) { if (n === 1) open++; else if (n > 2) nonManifold++; }
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
  const rec: ModelRecord = {
    ...meshRecord(res),
    dispatch: sortRec(dispatch),
    skipped: sortRec(res.stats.skipped),
  };
  if (withRemesh) rec.remesh = meshRecord(importStep(src, { remesh: true }));
  return rec;
}

// --- CLI ------------------------------------------------------------------------------------
const argv = process.argv.slice(2);
const update = argv.includes("--update");
const full = argv.includes("--full");
const named = argv.filter((a) => !a.startsWith("--"));

let models = readdirSync(root).filter((f) => f.endsWith(".step")).map((f) => f.slice(0, -5)).sort();
if (named.length) models = models.filter((m) => named.includes(m));
else if (!full) models = models.filter((m) => !SLOW.has(m));

const baseline: Record<string, ModelRecord> = existsSync(baselinePath)
  ? JSON.parse(readFileSync(baselinePath, "utf8")) : {};
const results: Record<string, ModelRecord> = {};
let drift = 0;

for (const m of models) {
  const t0 = Date.now();
  const rec = characterize(readFileSync(join(root, `${m}.step`), "utf8"), REMESH.has(m));
  const ms = Date.now() - t0;
  results[m] = rec;
  const base = baseline[m];
  if (update || !base) {
    console.log(`${update ? "base" : "NEW "} ${m.padEnd(24)} tris=${String(rec.tris).padStart(7)} open=${rec.openEdges} nm=${rec.nonManifoldEdges} vol=${rec.volume} ${ms}ms`);
    if (!update && !base) drift++;
    continue;
  }
  const diffs: string[] = [];
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
  if (base.remesh && rec.remesh) {
    if (rec.remesh.hash !== base.remesh.hash) diffs.push("remesh hash");
    for (const k of ["verts", "tris", "openEdges", "nonManifoldEdges", "volume"] as const) {
      if (rec.remesh[k] !== base.remesh[k]) diffs.push(`remesh.${k} ${base.remesh[k]} -> ${rec.remesh[k]}`);
    }
  }
  if (diffs.length) { drift++; console.log(`DIFF ${m.padEnd(24)} ${diffs.join("; ")}`); }
  else console.log(`ok   ${m.padEnd(24)} tris=${String(rec.tris).padStart(7)} ${ms}ms`);
}

if (update) {
  // merge: keep baseline entries for models not run this time (e.g. slow ones skipped)
  const merged = { ...baseline, ...results };
  const sorted = Object.fromEntries(Object.entries(merged).sort(([x], [y]) => x.localeCompare(y)));
  writeFileSync(baselinePath, JSON.stringify(sorted, null, 2) + "\n");
  console.log(`baseline written: ${Object.keys(sorted).length} models -> test/char-baseline.json`);
} else if (drift) {
  console.error(`${drift} model(s) drifted from baseline (or missing). If intended: node test/char.ts --update${full ? " --full" : ""}`);
  process.exit(1);
} else {
  console.log(`char: all ${models.length} models match baseline`);
}
