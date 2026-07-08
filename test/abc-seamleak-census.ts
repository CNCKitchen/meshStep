// SPDX-License-Identifier: AGPL-3.0-only
// Census of the seam-leak class (rec ④ scoping): for every ABC model whose bucket is
// seam-leak:*, tessellate with the trace hook and attribute each OPEN edge of the final mesh to
//   - the mesher that produced its owning face (grid / band / ribbon / region / …), and
//   - a geometric class: ON a shared BREP edge polyline (the two faces consumed the same edge
//     differently), ON the owning surface's periodic seam ruling (the face's own wrap didn't
//     weld), or INTERIOR (CDT hole / fold loss inside one face).
// Output clusters pick the sub-class to attack.
//
//   node test/abc-seamleak-census.ts             # orchestrate: 6 children over the list
//   node test/abc-seamleak-census.ts --child i n # child mode
import { readFileSync, appendFileSync, existsSync, unlinkSync } from "node:fs";
import { spawn } from "node:child_process";
import { join } from "node:path";
import { buildBrep, type BrepModel } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { makeSurface } from "../src/geom/surfaces.ts";
import { sampleEdgePolyline } from "../src/geom/curves.ts";
import type { Vec3 } from "../src/geom/vec.ts";

const ABC = "abc_0000_step_v00";
const OUT = process.env.SEAM_OUT ?? "out/abc-run7/seamleak-census.jsonl";
const RESULTS = process.env.SEAM_RESULTS ?? "out/abc-run7/results.jsonl";

function brepDiag(brep: BrepModel): number {
  let mnx = Infinity, mny = Infinity, mnz = Infinity, mxx = -Infinity, mxy = -Infinity, mxz = -Infinity;
  for (const e of brep.edges.values()) {
    for (const p of [e.v0, e.v1]) {
      if (p[0]! < mnx) mnx = p[0]!; if (p[1]! < mny) mny = p[1]!; if (p[2]! < mnz) mnz = p[2]!;
      if (p[0]! > mxx) mxx = p[0]!; if (p[1]! > mxy) mxy = p[1]!; if (p[2]! > mxz) mxz = p[2]!;
    }
  }
  return Number.isFinite(mnx) ? Math.hypot(mxx - mnx, mxy - mny, mxz - mnz) : 0;
}

function censusOne(path: string): object {
  const src = readFileSync(path, "utf8");
  const brep = buildBrep(src);
  const diag = brepDiag(brep);
  const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
  const targetEdge = diag > 0 ? 0.05 * diag : 1;
  const mesherOf = new Map<number, string>();
  const res = tessellate(brep, {
    chordTol, targetEdge, normalDev: 15 * Math.PI / 180,
    trace: (fid, mesher) => mesherOf.set(fid, mesher),
  });
  // Open edges of the final mesh (undirected count == 1).
  const I = res.mesh.indices, P = res.mesh.positions;
  const KEY = 2 ** 26;
  const ek = (a: number, b: number): number => (a < b ? a * KEY + b : b * KEY + a);
  const use = new Map<number, number>();
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
    const k = ek(I[i + e]!, I[i + (e + 1) % 3]!);
    use.set(k, (use.get(k) ?? 0) + 1);
  }
  type Open = { a: number; b: number; fid: number };
  const open: Open[] = [];
  for (let i = 0; i < I.length; i += 3) for (let e = 0; e < 3; e++) {
    const a = I[i + e]!, b = I[i + (e + 1) % 3]!;
    if (use.get(ek(a, b)) === 1) open.push({ a, b, fid: res.faceOfTri[i / 3]! });
  }
  if (!open.length) return { file: path.replace(/^.*abc_0000_step_v00[\\/]/, ""), open: 0 };

  // Spatial hash of BREP edge polylines (coarse resample) for the shared-edge test.
  const cell = Math.max(2 * chordTol, 1e-6);
  const OFFC = 1 << 20, PACK = 1 << 21;
  const ehash = new Map<number, Map<number, [Vec3, Vec3][]>>();
  const addSeg = (p: Vec3, q: Vec3): void => {
    const mx = (p[0] + q[0]) / 2, my = (p[1] + q[1]) / 2, mz = (p[2] + q[2]) / 2;
    const cx = Math.round(mx / cell), k2 = (Math.round(my / cell) + OFFC) * PACK + (Math.round(mz / cell) + OFFC);
    let col = ehash.get(cx); if (!col) { col = new Map(); ehash.set(cx, col); }
    const arr = col.get(k2); if (arr) arr.push([p, q]); else col.set(k2, [[p, q]]);
  };
  for (const e of brep.edges.values()) {
    let poly: Vec3[];
    try { poly = sampleEdgePolyline(brep.table, e.curveId, e.v0, e.v1, e.sameSense, e.scale ?? brep.scale, chordTol, targetEdge, brep.units.radPerAngle, 15 * Math.PI / 180); }
    catch { continue; }
    for (let i = 0; i + 1 < poly.length; i++) addSeg(poly[i]!, poly[i + 1]!);
  }
  const distSeg = (x: number, y: number, z: number, p: Vec3, q: Vec3): number => {
    const ex = q[0] - p[0], ey = q[1] - p[1], ez = q[2] - p[2];
    const l2 = ex * ex + ey * ey + ez * ez;
    let t = l2 > 0 ? ((x - p[0]) * ex + (y - p[1]) * ey + (z - p[2]) * ez) / l2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return Math.hypot(x - p[0] - t * ex, y - p[1] - t * ey, z - p[2] - t * ez);
  };
  const nearBrepEdge = (x: number, y: number, z: number, tol: number): boolean => {
    const cx = Math.round(x / cell), cy = Math.round(y / cell), cz = Math.round(z / cell);
    for (let gx = -1; gx <= 1; gx++) {
      const col = ehash.get(cx + gx); if (!col) continue;
      for (let gy = -1; gy <= 1; gy++) for (let gz = -1; gz <= 1; gz++) {
        for (const [p, q] of col.get((cy + gy + OFFC) * PACK + (cz + gz + OFFC)) ?? []) {
          if (distSeg(x, y, z, p, q) <= tol) return true;
        }
      }
    }
    return false;
  };

  // Surface cache for the seam test.
  const surfOf = new Map<number, { s: unknown; kind: string } | null>();
  const faceInfo = new Map<number, { surfaceId: number; scale: number | undefined; kind: string }>();
  for (const solid of brep.solids) for (const f of solid.faces) faceInfo.set(f.faceId, { surfaceId: f.surfaceId, scale: solid.scale ?? brep.scale, kind: f.surfaceKind });
  const getSurf = (fid: number): { s: { project(p: Vec3): [number, number]; periodicU?: boolean; periodicV?: boolean; uSeam?: number; vSeam?: number; uPeriod?: number; vPeriod?: number } | null; kind: string } | null => {
    if (surfOf.has(fid)) return surfOf.get(fid) as never;
    const fi = faceInfo.get(fid);
    if (!fi) { surfOf.set(fid, null); return null; }
    let s: unknown = null;
    try { s = makeSurface(brep.table, fi.surfaceId, fi.scale, brep.units.radPerAngle); } catch { /* keep null */ }
    const rec = { s, kind: fi.kind };
    surfOf.set(fid, rec as never);
    return rec as never;
  };

  const perMesher: Record<string, number> = {};
  const perClass: Record<string, number> = {};
  const perKind: Record<string, number> = {};
  const perFid = new Map<number, number>();
  const TWO_PI = Math.PI * 2;
  for (const oe of open) {
    const mx = (P[oe.a * 3]! + P[oe.b * 3]!) / 2, my = (P[oe.a * 3 + 1]! + P[oe.b * 3 + 1]!) / 2, mz = (P[oe.a * 3 + 2]! + P[oe.b * 3 + 2]!) / 2;
    const m = mesherOf.get(oe.fid) ?? "?";
    perMesher[m] = (perMesher[m] ?? 0) + 1;
    perFid.set(oe.fid, (perFid.get(oe.fid) ?? 0) + 1);
    const info = getSurf(oe.fid);
    perKind[info?.kind ?? "?"] = (perKind[info?.kind ?? "?"] ?? 0) + 1;
    let cls = "interior";
    if (nearBrepEdge(mx, my, mz, Math.max(2 * chordTol, 1e-6))) cls = "sharedEdge";
    else if (info?.s && (info.s.periodicU || info.s.periodicV)) {
      try {
        const [u, v] = info.s.project([mx, my, mz]);
        const nearSeam = (x: number, seam: number | undefined, per: number | undefined): boolean => {
          const p = per || TWO_PI;
          const d = Math.abs((((x - (seam ?? 0)) % p) + p) % p);
          return Math.min(d, p - d) < 0.02 * p;
        };
        if ((info.s.periodicU && nearSeam(u, info.s.uSeam, info.s.uPeriod))
          || (info.s.periodicV && nearSeam(v, info.s.vSeam, info.s.vPeriod))) cls = "seamRuling";
      } catch { /* interior */ }
    }
    perClass[cls] = (perClass[cls] ?? 0) + 1;
  }
  const topF = [...perFid.entries()].sort((a, b) => b[1] - a[1]).slice(0, 4)
    .map(([fid, n]) => ({ fid, n, mesher: mesherOf.get(fid) ?? "?", kind: getSurf(fid)?.kind ?? "?" }));
  return {
    file: path.replace(/^.*abc_0000_step_v00[\\/]/, ""), diag, open: open.length,
    perMesher, perClass, perKind, topFaces: topF,
  };
}

const argv = process.argv.slice(2);
if (argv[0] === "--child") {
  const i0 = parseInt(argv[1]!, 10), stride = parseInt(argv[2]!, 10);
  const files = readFileSync(RESULTS, "utf8").split(/\r?\n/).filter(Boolean).map((l) => JSON.parse(l))
    .filter((r) => typeof r.bucket === "string" && r.bucket.startsWith("seam-leak")).map((r) => r.file as string);
  for (let i = i0; i < files.length; i += stride) {
    let rec: object;
    try { rec = censusOne(join(ABC, files[i]!)); }
    catch (e) { rec = { file: files[i], err: e instanceof Error ? e.message : String(e) }; }
    appendFileSync(OUT, JSON.stringify(rec) + "\n");
    console.log(`[${i0}] ${i}: done`);
  }
} else {
  if (existsSync(OUT)) unlinkSync(OUT);
  const N = 6;
  let done = 0;
  for (let k = 0; k < N; k++) {
    const c = spawn(process.execPath, ["test/abc-seamleak-census.ts", "--child", String(k), String(N)], { stdio: "inherit" });
    c.on("exit", () => { if (++done === N) console.log("CENSUS DONE"); });
  }
}
