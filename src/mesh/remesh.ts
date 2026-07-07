// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — isotropic remeshing toward a uniform target edge length.
//
// Operates per CAD face: edges between two different faces (the BREP feature edges) and the
// vertices on them are FROZEN, which (a) keeps sharp edges crisp, (b) keeps the mesh watertight
// for free. Interior vertices are projected onto their analytic surface so refinement also
// improves geometric fidelity. Operations: edge split (coarse->fine), edge flip (de-sliver),
// edge collapse (over-dense->coarse), tangential smoothing.
import type { Vec3 } from "../geom/vec.ts";
import { isBSpline, type Surface } from "../geom/surfaces.ts";
import type { IndexedMesh } from "../io/stl.ts";

const ekey = (a: number, b: number): number => (a < b ? a * 0x4000000 + b : b * 0x4000000 + a);

/**
 * A face is FROZEN (kept as its fine initial tessellation, never split/collapsed/flipped/smoothed)
 * when we can't reliably project onto it: an unknown surface, or a B-spline whose closest-point is a
 * Newton search that can map a smoothed centroid to a wrong (u,v) and spawn degenerate triangles.
 */
function frozenFace(s: State, faceId: number): boolean {
  const surface = s.surf.get(faceId);
  // Plane / cylinder / cone / sphere have a cheap exact projection and remesh cleanly under the
  // quality-guarded passes. A torus and a B-spline are kept as-is: their split over-refines (torus
  // tube radius) or projects a midpoint to a wrong (u,v) on a skewed patch (exploding the count to
  // 2x with MORE slivers), and the guarded collapse/smooth barely move their already-fine, already-
  // curvature-adaptive initial mesh while tripling the runtime. Not worth it.
  return !surface || isBSpline(surface) || surface.kind === "TOROIDAL_SURFACE";
}

interface State {
  pos: number[];           // 3 per vertex
  tris: number[];          // 3 per triangle
  tFace: number[];         // face id per triangle
  surf: Map<number, Surface | null>;
  surfaceDev: number;      // max chord deviation (mm)
  normalDev: number;       // max normal turn across an edge (radians)
  maxEdge: number;         // max edge length (mm)
  maxTris: number;         // triangle budget (stop refining beyond this)
}

/**
 * Curvature-adaptive target edge length for the edge a-b on a face: the smallest of the
 * max-edge cap, the edge that keeps chord deviation within surfaceDev, and the edge that keeps
 * the normal turn within normalDev — so flat faces stay coarse and curved faces refine finely.
 */
function localTargetEdge(s: State, faceId: number, a: number, b: number): number {
  const surface = s.surf.get(faceId);
  if (!surface) return s.maxEdge;
  const mx = (s.pos[a * 3]! + s.pos[b * 3]!) / 2;
  const my = (s.pos[a * 3 + 1]! + s.pos[b * 3 + 1]!) / 2;
  const mz = (s.pos[a * 3 + 2]! + s.pos[b * 3 + 2]!) / 2;
  const [u, v] = surface.project([mx, my, mz]);
  const R = surface.curvatureRadius(u, v);
  if (!Number.isFinite(R)) return s.maxEdge;
  const tDev = Math.sqrt(8 * R * s.surfaceDev); // chord sagitta e^2/(8R) = surfaceDev
  const tNorm = R * s.normalDev;                 // arc turn e/R = normalDev
  return Math.max(s.maxEdge / 20, Math.min(s.maxEdge, tDev, tNorm));
}

/** Project a point onto a face's analytic surface (no-op if surface unknown, e.g. B-spline). */
function projectToFace(s: State, faceId: number, p: Vec3): Vec3 {
  const surface = s.surf.get(faceId);
  if (!surface) return p;
  const [u, v] = surface.project(p);
  return surface.evaluate(u, v);
}

/** Map edge -> incident triangle indices. */
function edgeTriMap(s: State): Map<number, number[]> {
  const m = new Map<number, number[]>();
  const nt = s.tris.length / 3;
  // Unrolled — the tuple-array idiom allocates 4 arrays per triangle, and this map is rebuilt
  // several times per remesh iteration over the whole mesh.
  const add = (u: number, v: number, t: number): void => {
    const k = ekey(u, v);
    const arr = m.get(k);
    if (arr) arr.push(t); else m.set(k, [t]);
  };
  for (let t = 0; t < nt; t++) {
    const a = s.tris[t * 3]!, b = s.tris[t * 3 + 1]!, c = s.tris[t * 3 + 2]!;
    add(a, b, t); add(b, c, t); add(c, a, t);
  }
  return m;
}

/**
 * Per-vertex face id to project onto, or -1 if frozen. A vertex is frozen if it lies on a
 * feature edge (touches >1 CAD face) or is a parametric singularity (very high valence — a cone
 * apex or sphere pole), which must not be smoothed away.
 */
function classifyVertices(s: State): Int32Array {
  const nv = s.pos.length / 3;
  const faceOf = new Int32Array(nv).fill(-2); // -2 unseen, -1 frozen, >=0 single face
  const valence = new Uint32Array(nv);
  const nt = s.tris.length / 3;
  for (let t = 0; t < nt; t++) {
    const f = s.tFace[t]!;
    for (let e = 0; e < 3; e++) {
      const v = s.tris[t * 3 + e]!;
      valence[v]++;
      if (faceOf[v] === -2) faceOf[v] = f;
      else if (faceOf[v] !== f) faceOf[v] = -1;
    }
  }
  // Freeze singular vertices so they aren't smoothed away: high-valence hubs (sphere poles), and
  // curvature singularities (a cone apex, where the radius -> 0) — the latter caught by geometry,
  // not valence, since collapse lowers the apex valence over time and would otherwise unfreeze it.
  for (let v = 0; v < nv; v++) {
    if (valence[v]! > 12) { faceOf[v] = -1; continue; }
    const f = faceOf[v]!;
    if (f < 0) continue;
    if (frozenFace(s, f)) { faceOf[v] = -1; continue; } // keep B-spline / unknown faces as-is
    const surface = s.surf.get(f);
    if (!surface) continue;
    const [u, vv] = surface.project([s.pos[v * 3]!, s.pos[v * 3 + 1]!, s.pos[v * 3 + 2]!]);
    if (surface.curvatureRadius(u, vv) < s.maxEdge / 8) faceOf[v] = -1;
  }
  return faceOf;
}

function dist2(s: State, a: number, b: number): number {
  const dx = s.pos[a * 3]! - s.pos[b * 3]!;
  const dy = s.pos[a * 3 + 1]! - s.pos[b * 3 + 1]!;
  const dz = s.pos[a * 3 + 2]! - s.pos[b * 3 + 2]!;
  return dx * dx + dy * dy + dz * dz;
}

/** Split interior edges longer than 4/3 of their curvature-adaptive target; returns true if changed. */
function splitPass(s: State): boolean {
  if (s.tris.length / 3 >= s.maxTris) return false; // triangle budget reached
  const em = edgeTriMap(s);
  const mid = new Map<number, number>(); // edge key -> new vertex index
  for (const [k, ts] of em) {
    if (ts.length !== 2 || s.tFace[ts[0]!] !== s.tFace[ts[1]!]) continue; // feature/boundary edge
    if (frozenFace(s, s.tFace[ts[0]!]!)) continue;                        // unprojectable / kept-as-is face
    const a = Math.floor(k / 0x4000000), b = k % 0x4000000;
    const high = (4 / 3) * localTargetEdge(s, s.tFace[ts[0]!]!, a, b);
    if (dist2(s, a, b) <= high * high) continue;
    const p: Vec3 = [
      (s.pos[a * 3]! + s.pos[b * 3]!) / 2,
      (s.pos[a * 3 + 1]! + s.pos[b * 3 + 1]!) / 2,
      (s.pos[a * 3 + 2]! + s.pos[b * 3 + 2]!) / 2,
    ];
    const proj = projectToFace(s, s.tFace[ts[0]!]!, p);
    const idx = s.pos.length / 3;
    s.pos.push(proj[0], proj[1], proj[2]);
    mid.set(k, idx);
  }
  if (mid.size === 0) return false;

  const nt = s.tris.length / 3;
  const newTris: number[] = [];
  const newFace: number[] = [];
  const emit = (a: number, b: number, c: number, f: number): void => { newTris.push(a, b, c); newFace.push(f); };
  for (let t = 0; t < nt; t++) {
    const v0 = s.tris[t * 3]!, v1 = s.tris[t * 3 + 1]!, v2 = s.tris[t * 3 + 2]!, f = s.tFace[t]!;
    const m0 = mid.get(ekey(v0, v1)) ?? -1;
    const m1 = mid.get(ekey(v1, v2)) ?? -1;
    const m2 = mid.get(ekey(v2, v0)) ?? -1;
    const cnt = (m0 >= 0 ? 1 : 0) + (m1 >= 0 ? 1 : 0) + (m2 >= 0 ? 1 : 0);
    if (cnt === 0) { emit(v0, v1, v2, f); continue; }
    if (cnt === 3) {
      emit(v0, m0, m2, f); emit(v1, m1, m0, f); emit(v2, m2, m1, f); emit(m0, m1, m2, f); continue;
    }
    if (cnt === 1) {
      if (m0 >= 0) { emit(v0, m0, v2, f); emit(m0, v1, v2, f); }
      else if (m1 >= 0) { emit(v0, v1, m1, f); emit(v0, m1, v2, f); }
      else { emit(v0, v1, m2, f); emit(m2, v1, v2, f); }
      continue;
    }
    // cnt === 2
    if (m0 >= 0 && m1 >= 0) { emit(m0, v1, m1, f); emit(v0, m0, m1, f); emit(v0, m1, v2, f); }
    else if (m1 >= 0 && m2 >= 0) { emit(m1, v2, m2, f); emit(v0, v1, m1, f); emit(v0, m1, m2, f); }
    else { emit(v0, m0, m2, f); emit(m0, v1, v2, f); emit(m0, v2, m2, f); }
  }
  s.tris = newTris;
  s.tFace = newFace;
  return true;
}

const third = (s: State, t: number, a: number, b: number): number => {
  const x = s.tris[t * 3]!, y = s.tris[t * 3 + 1]!, z = s.tris[t * 3 + 2]!;
  return x !== a && x !== b ? x : y !== a && y !== b ? y : z;
};

/** Unit normal of a triangle given by three vertex indices. */
function triNormal(s: State, a: number, b: number, c: number): Vec3 {
  const ux = s.pos[b * 3]! - s.pos[a * 3]!, uy = s.pos[b * 3 + 1]! - s.pos[a * 3 + 1]!, uz = s.pos[b * 3 + 2]! - s.pos[a * 3 + 2]!;
  const vx = s.pos[c * 3]! - s.pos[a * 3]!, vy = s.pos[c * 3 + 1]! - s.pos[a * 3 + 1]!, vz = s.pos[c * 3 + 2]! - s.pos[a * 3 + 2]!;
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const l = Math.hypot(nx, ny, nz) || 1;
  return [nx / l, ny / l, nz / l];
}

/** Smallest interior angle (radians) of the triangle with the given corner positions. */
function minAngleP(A: Vec3, B: Vec3, C: Vec3): number {
  const ang = (i: Vec3, j: Vec3, k: Vec3): number => {
    const ux = j[0] - i[0], uy = j[1] - i[1], uz = j[2] - i[2];
    const vx = k[0] - i[0], vy = k[1] - i[1], vz = k[2] - i[2];
    const d = (ux * vx + uy * vy + uz * vz) / (Math.hypot(ux, uy, uz) * Math.hypot(vx, vy, vz) || 1);
    return Math.acos(Math.max(-1, Math.min(1, d)));
  };
  return Math.min(ang(A, B, C), ang(B, C, A), ang(C, A, B));
}
const vpos = (s: State, i: number): Vec3 => [s.pos[i * 3]!, s.pos[i * 3 + 1]!, s.pos[i * 3 + 2]!];
/** Smallest interior angle (radians) of triangle a,b,c. */
function minAngle(s: State, a: number, b: number, c: number): number {
  return minAngleP(vpos(s, a), vpos(s, b), vpos(s, c));
}

/** Set triangle t to (x,y,z), oriented outward via the analytic surface normal. */
function setTriOriented(s: State, t: number, x: number, y: number, z: number): void {
  const surface = s.surf.get(s.tFace[t]!);
  let keep = true;
  if (surface) {
    const ax = s.pos[x * 3]!, ay = s.pos[x * 3 + 1]!, az = s.pos[x * 3 + 2]!;
    const ux = s.pos[y * 3]! - ax, uy = s.pos[y * 3 + 1]! - ay, uz = s.pos[y * 3 + 2]! - az;
    const vx = s.pos[z * 3]! - ax, vy = s.pos[z * 3 + 1]! - ay, vz = s.pos[z * 3 + 2]! - az;
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const cen: Vec3 = [(ax + s.pos[y * 3]! + s.pos[z * 3]!) / 3, (ay + s.pos[y * 3 + 1]! + s.pos[z * 3 + 1]!) / 3, (az + s.pos[y * 3 + 2]! + s.pos[z * 3 + 2]!) / 3];
    const [u, v] = surface.project(cen);
    const n2 = surface.normal(u, v);
    keep = nx * n2[0] + ny * n2[1] + nz * n2[2] >= 0;
  }
  s.tris[t * 3] = x;
  s.tris[t * 3 + 1] = keep ? y : z;
  s.tris[t * 3 + 2] = keep ? z : y;
}

/** Flip interior edges when it improves the minimum angle (de-slivers), preserving features. */
function flipPass(s: State): void {
  const em = edgeTriMap(s);
  const used = new Uint8Array(s.tris.length / 3);
  const created = new Set<number>(); // edges introduced this pass (avoid making them non-manifold)
  for (const [k, ts] of em) {
    if (ts.length !== 2) continue;
    const ti = ts[0]!, tj = ts[1]!;
    if (used[ti] || used[tj]) continue;
    if (s.tFace[ti] !== s.tFace[tj]) continue; // feature edge — frozen
    if (frozenFace(s, s.tFace[ti]!)) continue; // unprojectable face — keep as-is
    const a = Math.floor(k / 0x4000000), b = k % 0x4000000;
    const c = third(s, ti, a, b), d = third(s, tj, a, b);
    const cd = ekey(c, d);
    if (c === d || em.has(cd) || created.has(cd)) continue;
    // Only flip near-coplanar edges; flipping across curvature folds the surface.
    const n0 = triNormal(s, a, b, c), n1 = triNormal(s, a, b, d);
    if (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2] < 0.92) continue;
    const oldMin = Math.min(minAngle(s, a, b, c), minAngle(s, a, b, d));
    const newMin = Math.min(minAngle(s, a, c, d), minAngle(s, c, b, d));
    if (newMin <= oldMin + 1e-4) continue;
    setTriOriented(s, ti, a, c, d);
    setTriOriented(s, tj, c, b, d);
    used[ti] = 1; used[tj] = 1;
    created.add(cd);
  }
}

/** Compact away dead vertices/triangles, remapping indices. */
function compact(s: State, deadV: Uint8Array, deadT: Uint8Array): void {
  const nv = s.pos.length / 3;
  const remap = new Int32Array(nv).fill(-1);
  const newPos: number[] = [];
  for (let v = 0; v < nv; v++) {
    if (deadV[v]) continue;
    remap[v] = newPos.length / 3;
    newPos.push(s.pos[v * 3]!, s.pos[v * 3 + 1]!, s.pos[v * 3 + 2]!);
  }
  const newTris: number[] = [];
  const newFace: number[] = [];
  const nt = s.tris.length / 3;
  for (let t = 0; t < nt; t++) {
    if (deadT[t]) continue;
    const a = remap[s.tris[t * 3]!]!, b = remap[s.tris[t * 3 + 1]!]!, c = remap[s.tris[t * 3 + 2]!]!;
    if (a < 0 || b < 0 || c < 0 || a === b || b === c || c === a) continue;
    newTris.push(a, b, c);
    newFace.push(s.tFace[t]!);
  }
  s.pos = newPos; s.tris = newTris; s.tFace = newFace;
}

/** Collapse edges shorter than 4/5 of their curvature-adaptive target; merges over-dense regions. */
function collapsePass(s: State): void {
  const highCap2 = (4 / 3 * s.maxEdge) * (4 / 3 * s.maxEdge);
  const faceOf = classifyVertices(s);
  const nv = s.pos.length / 3, nt = s.tris.length / 3;
  const v2t: number[][] = Array.from({ length: nv }, () => []);
  for (let t = 0; t < nt; t++) for (let e = 0; e < 3; e++) v2t[s.tris[t * 3 + e]!]!.push(t);
  const em = edgeTriMap(s);
  const deadV = new Uint8Array(nv), deadT = new Uint8Array(nt), touched = new Uint8Array(nv);

  const neighbors = (v: number): Set<number> => {
    const set = new Set<number>();
    for (const t of v2t[v]!) { if (deadT[t]) continue; for (let e = 0; e < 3; e++) { const w = s.tris[t * 3 + e]!; if (w !== v) set.add(w); } }
    return set;
  };

  for (const [k, ts] of em) {
    if (ts.length !== 2) continue;
    const ti = ts[0]!, tj = ts[1]!;
    if (deadT[ti] || deadT[tj] || s.tFace[ti] !== s.tFace[tj]) continue;
    if (frozenFace(s, s.tFace[ti]!)) continue; // unprojectable face — keep as-is
    const a = Math.floor(k / 0x4000000), b = k % 0x4000000;
    if (deadV[a] || deadV[b] || touched[a] || touched[b]) continue;
    const low = (4 / 5) * localTargetEdge(s, s.tFace[ti]!, a, b);
    if (dist2(s, a, b) >= low * low) continue;
    const fa = faceOf[a]!, fb = faceOf[b]!;
    let kp: number, rm: number, pos: Vec3;
    if (fa >= 0 && fb >= 0) { kp = a; rm = b; pos = projectToFace(s, fa, [(s.pos[a * 3]! + s.pos[b * 3]!) / 2, (s.pos[a * 3 + 1]! + s.pos[b * 3 + 1]!) / 2, (s.pos[a * 3 + 2]! + s.pos[b * 3 + 2]!) / 2]); }
    else if (fa < 0 && fb >= 0) { kp = a; rm = b; pos = [s.pos[a * 3]!, s.pos[a * 3 + 1]!, s.pos[a * 3 + 2]!]; }
    else if (fb < 0 && fa >= 0) { kp = b; rm = a; pos = [s.pos[b * 3]!, s.pos[b * 3 + 1]!, s.pos[b * 3 + 2]!]; }
    else continue;
    // Link condition: a and b must share exactly the two opposite vertices (else non-manifold).
    const na = neighbors(a), nb = neighbors(b);
    let common = 0;
    for (const w of na) if (nb.has(w)) common++;
    if (common !== 2) continue;
    // Validity over every triangle around a or b (both change: rm is redirected to kp, which moves to
    // pos): no triangle may invert, degenerate, or grow a too-long edge — AND the collapse must not
    // create a sliver in a previously-acceptable region (post min angle ≥ pre, unless already < 20°).
    let ok = true, preMin = Infinity, postMin = Infinity;
    const Pw = (i: number): Vec3 => (i === kp || i === rm ? pos : vpos(s, i));
    const around = new Set<number>();
    for (const t of v2t[a]!) if (!deadT[t] && t !== ti && t !== tj) around.add(t);
    for (const t of v2t[b]!) if (!deadT[t] && t !== ti && t !== tj) around.add(t);
    for (const t of around) {
      const v0 = s.tris[t * 3]!, v1 = s.tris[t * 3 + 1]!, v2 = s.tris[t * 3 + 2]!;
      const w0 = v0 === rm ? kp : v0, w1 = v1 === rm ? kp : v1, w2 = v2 === rm ? kp : v2;
      if (w0 === w1 || w1 === w2 || w2 === w0) { ok = false; break; }
      preMin = Math.min(preMin, minAngle(s, v0, v1, v2));
      const old = triNormal(s, v0, v1, v2);
      const A = Pw(w0), B = Pw(w1), C = Pw(w2);
      const ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2], vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
      const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      const nl = Math.hypot(nx, ny, nz);
      if (nl < 1e-12 || (nx * old[0] + ny * old[1] + nz * old[2]) / nl < 0.2) { ok = false; break; }
      const e1 = (B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2 + (B[2] - A[2]) ** 2;
      const e2 = (C[0] - B[0]) ** 2 + (C[1] - B[1]) ** 2 + (C[2] - B[2]) ** 2;
      const e3 = (A[0] - C[0]) ** 2 + (A[1] - C[1]) ** 2 + (A[2] - C[2]) ** 2;
      if (Math.max(e1, e2, e3) > highCap2) { ok = false; break; }
      postMin = Math.min(postMin, minAngleP(A, B, C));
    }
    if (!ok || (postMin < preMin - 1e-3 && postMin < 20 * Math.PI / 180)) continue;
    deadV[rm] = 1; deadT[ti] = 1; deadT[tj] = 1;
    touched[a] = touched[b] = 1;
    for (const w of na) touched[w] = 1;
    for (const w of nb) touched[w] = 1;
    s.pos[kp * 3] = pos[0]; s.pos[kp * 3 + 1] = pos[1]; s.pos[kp * 3 + 2] = pos[2];
    for (const t of v2t[rm]!) { if (deadT[t]) continue; for (let e = 0; e < 3; e++) if (s.tris[t * 3 + e] === rm) s.tris[t * 3 + e] = kp; }
  }
  compact(s, deadV, deadT);
}

/** Tangential (Laplacian) smoothing of interior vertices, re-projected onto their surface. Each move
 * is accepted only if it doesn't lower the vertex's minimum incident angle (so it can't spawn a
 * sliver); otherwise the vertex is left where it was. */
function smoothPass(s: State, faceOf: Int32Array): void {
  const nv = s.pos.length / 3;
  const sum = new Float64Array(nv * 3);
  const cnt = new Uint32Array(nv);
  const nt = s.tris.length / 3;
  const v2t: number[][] = Array.from({ length: nv }, () => []);
  const acc = (v: number, w: number): void => {
    sum[v * 3] += s.pos[w * 3]!; sum[v * 3 + 1] += s.pos[w * 3 + 1]!; sum[v * 3 + 2] += s.pos[w * 3 + 2]!; cnt[v]++;
  };
  for (let t = 0; t < nt; t++) {
    const a = s.tris[t * 3]!, b = s.tris[t * 3 + 1]!, c = s.tris[t * 3 + 2]!;
    acc(a, b); acc(a, c); acc(b, a); acc(b, c); acc(c, a); acc(c, b);
    v2t[a]!.push(t); v2t[b]!.push(t); v2t[c]!.push(t);
  }
  const incidentMin = (v: number, at: Vec3): number => {
    let m = Math.PI;
    for (const t of v2t[v]!) {
      const x = s.tris[t * 3]!, y = s.tris[t * 3 + 1]!, z = s.tris[t * 3 + 2]!;
      m = Math.min(m, minAngleP(x === v ? at : vpos(s, x), y === v ? at : vpos(s, y), z === v ? at : vpos(s, z)));
    }
    return m;
  };
  for (let v = 0; v < nv; v++) {
    const f = faceOf[v]!;
    if (f < 0 || cnt[v] === 0) continue; // frozen feature/corner vertex
    const n = cnt[v]!;
    const target = projectToFace(s, f, [sum[v * 3]! / n, sum[v * 3 + 1]! / n, sum[v * 3 + 2]! / n]);
    if (incidentMin(v, target) < incidentMin(v, vpos(s, v)) - 1e-4) continue; // would worsen -> skip
    s.pos[v * 3] = target[0]; s.pos[v * 3 + 1] = target[1]; s.pos[v * 3 + 2] = target[2];
  }
}

export interface RemeshOptions {
  /** Max chord deviation from the true surface (mm). */
  surfaceDev: number;
  /** Max normal turn across an edge (radians). */
  normalDev: number;
  /** Max edge length (mm). */
  maxEdge: number;
  iterations?: number;
  /** Triangle budget — refinement stops past this (default 800k). */
  maxTris?: number;
}

/** Curvature-adaptive isotropic remesh in place; returns a fresh mesh + per-triangle face id. */
export function remesh(
  mesh: IndexedMesh, faceOfTri: Uint32Array, surf: Map<number, Surface | null>, opts: RemeshOptions,
): { mesh: IndexedMesh; faceOfTri: Uint32Array } {
  const iterations = opts.iterations ?? 8;
  const s: State = {
    pos: Array.from(mesh.positions), tris: Array.from(mesh.indices), tFace: Array.from(faceOfTri), surf,
    surfaceDev: opts.surfaceDev, normalDev: opts.normalDev, maxEdge: opts.maxEdge, maxTris: opts.maxTris ?? 800_000,
  };

  const env = (globalThis as { process?: { env?: Record<string, string | undefined> } }).process?.env ?? {};
  const noCollapse = !!env.NO_COLLAPSE, noFlip = !!env.NO_FLIP, noSmooth = !!env.NO_SMOOTH, noSplit = !!env.NO_SPLIT;
  const dbg = !!env.DBG_REM;
  // Split/collapse oscillate (collapse coarsens, split re-refines), so the loop rarely "converges" by
  // making no edits. But MESH QUALITY (sliver%, min angle) plateaus within ~3 iterations — the rest
  // only coarsens slowly. So also stop once the triangle count stabilises (<2% change), which keeps
  // quality while cutting the iteration count (and runtime) roughly in half on dense parts.
  let iters = 0, prevTris = s.tris.length / 3;
  for (let it = 0; it < iterations; it++) {
    iters++;
    const changed = noSplit ? false : splitPass(s);
    if (!noCollapse) collapsePass(s);
    if (!noFlip) flipPass(s);
    const faceOf = classifyVertices(s);
    if (!noSmooth) smoothPass(s, faceOf);
    if (!noFlip) flipPass(s);
    if (!noSmooth) smoothPass(s, faceOf);
    const nt = s.tris.length / 3;
    if (it > 2 && (!changed || Math.abs(nt - prevTris) < 0.02 * prevTris)) break;
    prevTris = nt;
  }
  if (dbg) console.log(`  remesh iters=${iters}`);
  if (!noFlip) flipPass(s); // final de-sliver

  return {
    mesh: { positions: Float64Array.from(s.pos), indices: Uint32Array.from(s.tris) },
    faceOfTri: Uint32Array.from(s.tFace),
  };
}
