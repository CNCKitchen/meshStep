// SPDX-License-Identifier: AGPL-3.0-only
// One converter on one model, in a fresh process (run.ts starts one per pair, so every run begins
// like a freshly opened browser tab: module load, WebAssembly compile, cold JIT).
//
//   node bench/one.ts diag <file.step>
//       model size: bounding-box diagonal in mm, from a coarse OCCT pass (meshStep if OCCT fails)
//   node bench/one.ts ref <adapter> <file.step> <params-json> <out.bin>
//       mesh and save the result (the fine references)
//   node bench/one.ts run <adapter> <file.step> <params-json> [ref.bin ...]
//       time it, then measure the result against the references
//
// Prints one JSON line on stdout. Library chatter goes to stderr.
import { readFileSync, writeFileSync } from "node:fs";
import { ADAPTERS, type MeshOut, type MeshParams } from "./adapters.ts";
import { measure, prepareReference, type Reference } from "./metrics.ts";

// keep stdout clean for the JSON line
const realLog = console.log;
console.log = console.info = console.warn = (...a: unknown[]) => console.error(...a);
const emit = (o: unknown) => { realLog(JSON.stringify(o)); };

const [mode, ...args] = process.argv.slice(2);

function bboxDiag(P: ArrayLike<number>): number {
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < P.length; i += 3) for (let k = 0; k < 3; k++) {
    const v = P[i + k]!; if (v < mn[k]!) mn[k] = v; if (v > mx[k]!) mx[k] = v;
  }
  return Math.hypot(mx[0]! - mn[0]!, mx[1]! - mn[1]!, mx[2]! - mn[2]!);
}

function saveMesh(path: string, m: MeshOut): void {
  const nv = m.positions.length, ni = m.indices.length;
  const buf = Buffer.alloc(8 + nv * 4 + ni * 4);
  buf.writeUInt32LE(nv, 0); buf.writeUInt32LE(ni, 4);
  Buffer.from(new Float32Array(m.positions).buffer).copy(buf, 8);
  Buffer.from(m.indices.buffer, m.indices.byteOffset, ni * 4).copy(buf, 8 + nv * 4);
  writeFileSync(path, buf);
}
function loadMesh(path: string): { positions: Float32Array; indices: Uint32Array } {
  const b = readFileSync(path);
  const nv = b.readUInt32LE(0), ni = b.readUInt32LE(4);
  const ab = b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength);
  return { positions: new Float32Array(ab, 8, nv), indices: new Uint32Array(ab, 8 + nv * 4, ni) };
}

const errText = (e: unknown) => (e instanceof Error ? e.message : String(e)).slice(0, 300);

async function main(): Promise<void> {
  if (mode === "diag") {
    const bytes = new Uint8Array(readFileSync(args[0]!));
    for (const id of ["occt-import-js", "meshstep"]) {
      try {
        const a = ADAPTERS[id]!();
        await a.init();
        // coarse: 1 mm chord, 0.5 rad - only the extent matters
        const m = await a.mesh(bytes, { deflection: 1, angleDeg: 30, diag: 1e4 });
        if (m.indices.length) { emit({ diag: bboxDiag(m.positions), from: id }); return; }
      } catch { /* try the next */ }
    }
    emit({ diag: 0, from: "none" });
    return;
  }

  const [id, file, paramsJson, ...rest] = args;
  const a = ADAPTERS[id!]?.();
  if (!a) throw new Error(`unknown adapter ${id}`);
  const p: MeshParams = JSON.parse(paramsJson!);
  const bytes = new Uint8Array(readFileSync(file!));
  const rec: Record<string, unknown> = { adapter: a.id, version: a.version };

  let t = performance.now();
  try { await a.init(); } catch (e) { emit({ ...rec, status: "error", stage: "init", error: errText(e) }); return; }
  rec.initMs = performance.now() - t;

  let m: MeshOut;
  t = performance.now();
  try { m = await a.mesh(bytes, p); } catch (e) { emit({ ...rec, status: "error", stage: "mesh", error: errText(e) }); return; }
  rec.coldMs = performance.now() - t;
  rec.parts = m.parts; rec.faces = m.faces;

  if (mode === "ref") {
    saveMesh(rest[0]!, m);
    emit({ ...rec, status: m.indices.length ? "ok" : "empty", tris: m.indices.length / 3 });
    return;
  }

  // warm: the same file again in the same page (JIT warm, WASM compiled); best of two
  if ((rec.coldMs as number) < 5000) {
    let best = Infinity;
    for (let k = 0; k < 2; k++) {
      t = performance.now();
      await a.mesh(bytes, p);
      best = Math.min(best, performance.now() - t);
    }
    rec.warmMs = best;
  }
  rec.peakRssMB = process.resourceUsage().maxRSS / 1024;
  if (!m.indices.length) { emit({ ...rec, status: "empty" }); return; }

  const refs: Reference[] = rest.map(f => prepareReference(loadMesh(f), 20000));
  const diag = bboxDiag(m.positions);
  rec.diagRatio = p.diag > 0 ? diag / p.diag : null; // unit / scale bugs
  Object.assign(rec, measure(m, p.diag, p.deflection, refs));
  emit({ ...rec, status: "ok" });
}

main().then(() => process.exit(0), e => { emit({ status: "error", stage: "harness", error: errText(e) }); process.exit(0); });
