// SPDX-License-Identifier: AGPL-3.0-only
// Mesh QUALITY audit against the exact B-rep — the checks that watertightness and Hausdorff-vs-OCC
// cannot see. Shared by test/char.ts (frozen per model), test/abc-worker.ts (ABC survey verdicts)
// and test/gapcheck-one.ts.
//
//   folds      triangle pairs of the SAME face that cross each other. A face meshed correctly never
//              self-intersects; a fold (part of the face covered 2-3x) stays watertight, on-surface
//              and consistently oriented, so nothing else flags it (Ontos collet nut torus: 7368).
//   flips      triangles whose normal disagrees with their face's majority orientation relative
//              to the surface normal (a twisted/folded patch inside an otherwise correct face).
//   dev        sampled distance from the mesh to the face's TRUE surface (triangle centroid and
//              edge midpoints), as a multiple of the requested chord tolerance. Catches chords that
//              cut through the surface (the unroll seam gouge: 48x) and coarse patches.
//   tilt       max angle between a triangle's normal and the surface normal at its centroid.
//   slivers    triangles with a minimum angle below 1°.
//
// Triangles that bridge two bodies (vertices shared with another solid — sewSolids ribbons between
// coincident sheet rims, reported by the import as a heuristic-fill warning) are left out of the
// flip/tilt checks: they stand across or fold over a ~0.02mm rim gap by construction.
//
// Faces without an analytic/B-spline surface (tessellated input) are skipped for dev/flips/tilt.
// Placed assembly meshes are mapped back to part-local coordinates through their instance frames.
import type { BrepModel } from "../src/brep/build.ts";
import { makeSurface, type Surface } from "../src/geom/surfaces.ts";
import { selfIntersections } from "./self-intersect.ts";

export interface FaceIssue { solid: number; face: number; kind: string; value: number }

export interface MeshAudit {
  /** Same-face crossing triangle pairs (always a mesher bug). */
  folds: number;
  /** Crossing pairs between different faces of the same solid (near-tangent faces can do this at
   * chord scale; large counts are suspicious). */
  crossFace: number;
  /** Triangles oriented against their face's majority (relative to the surface normal). */
  flips: number;
  /** Max sampled mesh-to-surface distance / chordTol. */
  devRatio: number;
  /** Max sampled mesh-to-surface distance in mm. */
  devMax: number;
  /** Max triangle-vs-surface normal angle, degrees. */
  tiltMaxDeg: number;
  /** Triangles with min angle < 1°. */
  slivers: number;
  /** Worst offenders, for drilling in (few per kind). */
  worst: FaceIssue[];
}

export interface AuditMeshInput {
  positions: Float64Array;
  indices: Uint32Array;
  faceOfTri: ArrayLike<number>;
  solidOfTri: ArrayLike<number>;
  /** Assembly placement: per-triangle index into frames (importStep's instanceOfTri/instances). */
  instanceOfTri?: ArrayLike<number>;
  frames?: ({ o: ArrayLike<number>; x: ArrayLike<number>; y: ArrayLike<number>; z: ArrayLike<number> } | null)[];
}

export interface AuditOptions {
  chordTol: number;
  /** Cap on sampled triangles per face (largest half + stride half). Default 1500. */
  samplesPerFace?: number;
  /** Skip the surface-based checks (dev/flips/tilt). */
  topologyOnly?: boolean;
}

export function auditMesh(m: AuditMeshInput, brep: BrepModel, opts: AuditOptions): MeshAudit {
  const P = m.positions, I = m.indices, nT = I.length / 3;
  const worst: FaceIssue[] = [];
  // ---- folds: crossing pairs inside one (solid, face); instance 0 of each solid only ----
  const keep: number[] = [];
  const firstFrame = new Map<number, number>();
  for (let t = 0; t < nT; t++) {
    if (m.instanceOfTri) {
      const s = m.solidOfTri[t]!, inst = m.instanceOfTri[t]!;
      const f0 = firstFrame.get(s);
      if (f0 === undefined) firstFrame.set(s, inst);
      else if (f0 !== inst) continue;
    }
    keep.push(t);
  }
  const keyOf = new Map<string, number>(), back: [number, number][] = [];
  const key = new Uint32Array(keep.length), sub = new Uint32Array(keep.length * 3);
  keep.forEach((t, k) => {
    const s = m.solidOfTri[t]!, f = m.faceOfTri[t]!, ks = `${s}:${f}`;
    let v = keyOf.get(ks); if (v === undefined) { v = back.length; keyOf.set(ks, v); back.push([s, f]); }
    key[k] = v; sub[k * 3] = I[t * 3]!; sub[k * 3 + 1] = I[t * 3 + 1]!; sub[k * 3 + 2] = I[t * 3 + 2]!;
  });
  const si = selfIntersections(P, sub, key);
  let crossFace = 0;
  for (const [k, n] of si.byFace) {
    if (!k.includes("-")) { const [s, f] = back[+k]!; if (worst.filter((w) => w.kind === "fold").length < 5) worst.push({ solid: s, face: f, kind: "fold", value: n }); continue; }
    const [a, b] = k.split("-").map(Number);
    if (back[a!]![0] === back[b!]![0]) crossFace += n;
  }

  // ---- slivers (all triangles) ----
  let slivers = 0;
  const SIN1 = Math.sin(Math.PI / 180);
  for (let t = 0; t < nT; t++) {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const ab = Math.hypot(P[b]! - P[a]!, P[b + 1]! - P[a + 1]!, P[b + 2]! - P[a + 2]!);
    const bc = Math.hypot(P[c]! - P[b]!, P[c + 1]! - P[b + 1]!, P[c + 2]! - P[b + 2]!);
    const ca = Math.hypot(P[a]! - P[c]!, P[a + 1]! - P[c + 1]!, P[a + 2]! - P[c + 2]!);
    const ux = P[b]! - P[a]!, uy = P[b + 1]! - P[a + 1]!, uz = P[b + 2]! - P[a + 2]!;
    const vx = P[c]! - P[a]!, vy = P[c + 1]! - P[a + 1]!, vz = P[c + 2]! - P[a + 2]!;
    const area2 = Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
    // min angle via sin(θ) = 2A / (product of the two sides adjacent to the smallest angle,
    // i.e. the two LONGEST sides)
    const s = [ab, bc, ca].sort((x, y) => x - y);
    if (s[2]! * s[1]! > 0 && area2 / (s[2]! * s[1]!) < SIN1) slivers++;
  }

  const res: MeshAudit = { folds: si.sameFace, crossFace, flips: 0, devRatio: 0, devMax: 0, tiltMaxDeg: 0, slivers, worst };
  if (opts.topologyOnly) return res;

  // ---- surface checks ----
  const faceRec = new Map<string, { surface: Surface | null; sense: number }>();
  for (const solid of brep.solids) {
    const sc = solid.scale ?? brep.scale;
    for (const f of solid.faces) {
      let surface: Surface | null = null;
      try { surface = makeSurface(brep.table, f.surfaceId, sc, brep.units.radPerAngle); } catch { surface = null; }
      faceRec.set(`${solid.id}:${f.faceId}`, { surface, sense: f.sameSense ? 1 : -1 });
    }
  }
  // vertices used by more than one solid -> triangles touching them bridge bodies (sew ribbons)
  // (typed arrays, not Maps: a 20M-triangle assembly has more vertices than a Map can hold)
  const vSolid = new Float64Array(P.length / 3).fill(-1), vSharedArr = new Uint8Array(P.length / 3);
  let anyShared = false;
  for (let t = 0; t < nT; t++) for (let e = 0; e < 3; e++) {
    const v = I[t * 3 + e]!, s = m.solidOfTri[t]!, o = vSolid[v]!;
    if (o === -1) vSolid[v] = s; else if (o !== s) { vSharedArr[v] = 1; anyShared = true; }
  }
  const vShared = { size: anyShared ? 1 : 0, has: (v: number): boolean => vSharedArr[v] === 1 };
  const byFace = new Map<number, number[]>();
  keep.forEach((t, k) => { const f = key[k]!; let a = byFace.get(f); if (!a) byFace.set(f, a = []); a.push(t); });
  const cap = opts.samplesPerFace ?? 1500;
  const local = (t: number, vi: number): [number, number, number] => {
    const x = P[vi * 3]!, y = P[vi * 3 + 1]!, z = P[vi * 3 + 2]!;
    const fr = m.instanceOfTri && m.frames ? m.frames[m.instanceOfTri[t]!] : null;
    if (!fr) return [x, y, z];
    const dx = x - fr.o[0]!, dy = y - fr.o[1]!, dz = z - fr.o[2]!;
    return [dx * fr.x[0]! + dy * fr.x[1]! + dz * fr.x[2]!, dx * fr.y[0]! + dy * fr.y[1]! + dz * fr.y[2]!, dx * fr.z[0]! + dy * fr.z[1]! + dz * fr.z[2]!];
  };
  const triArea = (t: number): number => {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const ux = P[b]! - P[a]!, uy = P[b + 1]! - P[a + 1]!, uz = P[b + 2]! - P[a + 2]!;
    const vx = P[c]! - P[a]!, vy = P[c + 1]! - P[a + 1]!, vz = P[c + 2]! - P[a + 2]!;
    return Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
  };
  const dist = (S: Surface, p: number[]): { d: number; uv: [number, number] } => {
    const uv = S.project(p as [number, number, number]);
    const q = S.evaluate(uv[0], uv[1]);
    return { d: Math.hypot(q[0] - p[0]!, q[1] - p[1]!, q[2] - p[2]!), uv };
  };
  let devFace: FaceIssue | null = null, tiltFace: FaceIssue | null = null;
  for (const [fk, tris] of byFace) {
    const [sid, fid] = back[fk]!;
    const rec = faceRec.get(`${sid}:${fid}`);
    const S = rec?.surface;
    if (!S) continue;
    let sample = tris;
    if (tris.length > cap) {
      const byArea = tris.slice().sort((x, y) => triArea(y) - triArea(x));
      const half = cap >> 1, set = new Set(byArea.slice(0, half));
      const stride = tris.length / half;
      for (let i = 0; i < half; i++) set.add(tris[Math.floor(i * stride)]!);
      sample = [...set];
    }
    let pos = 0, neg = 0, fDev = 0, fTilt = 0;
    for (const t of sample) {
      const bridges = vShared.size > 0 && (vShared.has(I[t * 3]!) || vShared.has(I[t * 3 + 1]!) || vShared.has(I[t * 3 + 2]!));
      const A = local(t, I[t * 3]!), B = local(t, I[t * 3 + 1]!), C = local(t, I[t * 3 + 2]!);
      const cen = [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3, (A[2] + B[2] + C[2]) / 3];
      const c0 = dist(S, cen);
      let d = c0.d;
      for (const [X, Y] of [[A, B], [B, C], [C, A]] as const) {
        d = Math.max(d, dist(S, [(X[0] + Y[0]) / 2, (X[1] + Y[1]) / 2, (X[2] + Y[2]) / 2]).d);
      }
      if (Number.isFinite(d) && d > fDev) fDev = d;
      const ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2], vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
      const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx, nl = Math.hypot(nx, ny, nz);
      const sn = S.normal(c0.uv[0], c0.uv[1]), sl = Math.hypot(sn[0], sn[1], sn[2]);
      if (bridges || !(nl > 0) || !(sl > 0.5)) continue; // sew ribbon, degenerate, or undefined normal (pole/apex)
      // A near-degenerate triangle's normal is float noise (it is counted as a sliver instead):
      // sin(min angle) = 2A / (two longest sides), skip below ~0.06°.
      const e = [Math.hypot(ux, uy, uz), Math.hypot(vx, vy, vz), Math.hypot(C[0] - B[0], C[1] - B[1], C[2] - B[2])].sort((x, y) => x - y);
      if (nl / (e[2]! * e[1]!) < 1e-3) continue;
      const cos = ((nx * sn[0] + ny * sn[1] + nz * sn[2]) / (nl * sl)) * rec.sense;
      // A flip is a triangle pointing clearly AGAINST the face (>120° off); one standing across it
      // (|cos| < 0.5) is not a reversed copy of the face — e.g. a sewSolids ribbon bridging two
      // coincident sheet rims carries a neighbouring face id and stands ~90° to it.
      if (cos > 0.5) pos++; else if (cos < -0.5) neg++;
      const tilt = Math.acos(Math.min(1, Math.abs(cos))) * 180 / Math.PI;
      if (tilt > fTilt) fTilt = tilt;
    }
    res.flips += Math.min(pos, neg);
    if (Math.min(pos, neg) > 0 && worst.filter((w) => w.kind === "flip").length < 5) worst.push({ solid: sid, face: fid, kind: "flip", value: Math.min(pos, neg) });
    if (fDev > res.devMax) { res.devMax = fDev; devFace = { solid: sid, face: fid, kind: "dev", value: fDev / opts.chordTol }; }
    if (fTilt > res.tiltMaxDeg) { res.tiltMaxDeg = fTilt; tiltFace = { solid: sid, face: fid, kind: "tilt", value: fTilt }; }
  }
  res.devRatio = res.devMax / opts.chordTol;
  if (devFace) worst.push(devFace);
  if (tiltFace) worst.push(tiltFace);
  return res;
}
