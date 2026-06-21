// SPDX-License-Identifier: AGPL-3.0-only
// Watertightness + producer for every model in sourceModels/ (Printables direct downloads).
import { readFileSync, readdirSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { importStep } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const dir = join(root, "sourceModels");
function producer(src: string): string {
  const m = src.match(/originating_system\s*\*?\/?\s*'([^']+)'/i) ?? src.match(/'([^']*(?:ONSHAPE|Fusion|Autodesk|SOLIDWORKS|FreeCAD|ST-DEVELOPER|OpenSCAD|Rhino|Inventor)[^']*)'/i);
  return m ? m[1]!.slice(0, 40) : "?";
}
function stats(m: any) {
  const I = m.indices, P = m.positions, nt = I.length / 3, und = new Map<string, number>();
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
for (const f of readdirSync(dir).filter((x) => x.toLowerCase().endsWith(".step") || x.toLowerCase().endsWith(".stp"))) {
  const src = readFileSync(join(dir, f), "utf8");
  try {
    const s = stats(importStep(src, { remesh: false, surfaceDeviation: 0.002, maxEdge: 1 }).mesh);
    const tag = s.open === 0 && s.nm === 0 ? "PASS" : "FAIL";
    console.log(`${tag} ${f.slice(0, 34).padEnd(34)} tris=${String(s.nt).padStart(8)} open=${String(s.open).padStart(4)} nm=${s.nm} sliver%=${s.sl}  [${producer(src)}]`);
  } catch (e) { console.log(`ERR  ${f}: ${(e as Error).message}`); }
}
