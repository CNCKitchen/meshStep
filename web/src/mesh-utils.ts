// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";

export interface RawMesh {
  positions: Float64Array; // 3 per vertex
  indices: Uint32Array; // 3 per triangle
}

/** Build an indexed BufferGeometry (Float32 positions) from a meshStep IndexedMesh. */
export function buildIndexedGeometry(mesh: RawMesh): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(mesh.positions), 3));
  g.setIndex(new THREE.Uint32BufferAttribute(mesh.indices.slice(), 1));
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

/** Build a non-indexed geometry from a flat triangle-soup (9 floats per tri), e.g. a reference STL. */
export function buildSoupGeometry(positions: Float64Array | Float32Array): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(positions), 3));
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

/** LineSegments-ready edge set: 6 floats per segment, plus the solid (body) id each segment
 * belongs to, so per-part hiding can filter the lines together with the triangles. */
export interface EdgeSet {
  positions: Float32Array;
  count: number;
  solidOfSeg: Uint32Array;
}

/**
 * Extract boundary ("open") edges of an indexed mesh: edges used by exactly one
 * triangle. A watertight mesh has none.
 */
export function boundaryEdges(mesh: RawMesh, solidOfTri: Uint32Array): EdgeSet {
  const idx = mesh.indices;
  const pos = mesh.positions;
  const useCount = new Map<number, number>();
  // Same packing as src/mesh/orient.ts: exact for vertex ids below 2^26 (~67M). The old
  // a*2^32+b key exceeded 2^53 from ~2M vertices up and silently collided.
  const KEY = 0x4000000;
  const key = (a: number, b: number) => (a < b ? a * KEY + b : b * KEY + a);

  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t]!, b = idx[t + 1]!, c = idx[t + 2]!;
    const k0 = key(a, b), k1 = key(b, c), k2 = key(c, a);
    useCount.set(k0, (useCount.get(k0) ?? 0) + 1);
    useCount.set(k1, (useCount.get(k1) ?? 0) + 1);
    useCount.set(k2, (useCount.get(k2) ?? 0) + 1);
  }

  // Second pass: collect the actual vertex pairs for edges used once.
  const out: number[] = [];
  const segSolid: number[] = [];
  const emit = (u: number, v: number, s: number): void => {
    if (useCount.get(key(u, v)) !== 1) return;
    out.push(pos[u * 3]!, pos[u * 3 + 1]!, pos[u * 3 + 2]!);
    out.push(pos[v * 3]!, pos[v * 3 + 1]!, pos[v * 3 + 2]!);
    segSolid.push(s);
  };
  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t]!, b = idx[t + 1]!, c = idx[t + 2]!;
    const s = solidOfTri[t / 3] ?? 0;
    emit(a, b, s); emit(b, c, s); emit(c, a, s);
  }
  return { positions: new Float32Array(out), count: segSolid.length, solidOfSeg: Uint32Array.from(segSolid) };
}

/**
 * Extract CAD surface boundaries: mesh edges that separate two different B-rep faces
 * (`faceOfTri` differs across the edge) plus any open boundary edge (used by one triangle).
 * Because meshStep welds shared B-rep edges once and maps every triangle back to its STEP
 * face, these are exactly the analytic face borders — rims, holes, fillet edges — sampled to
 * the tessellation tolerance. Each segment carries its solid id (bodies are welded
 * independently, so an edge never straddles two solids).
 */
export function featureEdges(mesh: RawMesh, faceOfTri: Uint32Array, solidOfTri: Uint32Array): EdgeSet {
  const idx = mesh.indices;
  const pos = mesh.positions;
  const KEY = 0x4000000; // matches boundaryEdges: exact for vertex ids below ~67M
  const key = (a: number, b: number) => (a < b ? a * KEY + b : b * KEY + a);

  const cnt = new Map<number, number>();
  const firstFace = new Map<number, number>();
  const feature = new Set<number>();
  const consider = (a: number, b: number, f: number): void => {
    const k = key(a, b);
    const n = (cnt.get(k) ?? 0) + 1;
    cnt.set(k, n);
    if (n === 1) firstFace.set(k, f);
    else if (firstFace.get(k) !== f) feature.add(k); // edge straddles two distinct faces
  };
  const tris = idx.length / 3;
  for (let t = 0; t < tris; t++) {
    const f = faceOfTri[t] ?? 0;
    const a = idx[t * 3]!, b = idx[t * 3 + 1]!, c = idx[t * 3 + 2]!;
    consider(a, b, f); consider(b, c, f); consider(c, a, f);
  }

  const out: number[] = [];
  const segSolid: number[] = [];
  const seen = new Set<number>();
  const emit = (u: number, v: number, s: number): void => {
    const k = key(u, v);
    if (seen.has(k)) return;
    if (cnt.get(k) !== 1 && !feature.has(k)) return; // interior-to-a-face edge: skip
    seen.add(k);
    out.push(pos[u * 3]!, pos[u * 3 + 1]!, pos[u * 3 + 2]!);
    out.push(pos[v * 3]!, pos[v * 3 + 1]!, pos[v * 3 + 2]!);
    segSolid.push(s);
  };
  for (let t = 0; t < tris; t++) {
    const a = idx[t * 3]!, b = idx[t * 3 + 1]!, c = idx[t * 3 + 2]!;
    const s = solidOfTri[t] ?? 0;
    emit(a, b, s); emit(b, c, s); emit(c, a, s);
  }
  return { positions: new Float32Array(out), count: segSolid.length, solidOfSeg: Uint32Array.from(segSolid) };
}

/**
 * Extract crease edges: edges whose two adjacent triangles' normals differ by more than
 * `angleDeg`, plus open boundary edges — the mesh-only stand-in for CAD face borders when the
 * model has no B-rep (STL). Degenerate triangles (zero-area) never register a crease, and edges
 * used by 3+ triangles are always emitted (a non-manifold junction is a feature by any measure).
 */
export function creaseEdges(mesh: RawMesh, solidOfTri: Uint32Array, angleDeg: number): EdgeSet {
  const idx = mesh.indices;
  const pos = mesh.positions;
  const tris = idx.length / 3;

  // Per-triangle unit normals; zero-length (degenerate) stays [0,0,0] and can't exceed any
  // threshold against a real normal (dot 0 < cos only for angles < 90° — so treat it explicitly).
  const nrm = new Float32Array(tris * 3);
  const degenerate = new Uint8Array(tris);
  for (let t = 0; t < tris; t++) {
    const a = idx[t * 3]! * 3, b = idx[t * 3 + 1]! * 3, c = idx[t * 3 + 2]! * 3;
    const abx = pos[b]! - pos[a]!, aby = pos[b + 1]! - pos[a + 1]!, abz = pos[b + 2]! - pos[a + 2]!;
    const acx = pos[c]! - pos[a]!, acy = pos[c + 1]! - pos[a + 1]!, acz = pos[c + 2]! - pos[a + 2]!;
    const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
    const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (l < 1e-30) { degenerate[t] = 1; continue; }
    nrm[t * 3] = nx / l; nrm[t * 3 + 1] = ny / l; nrm[t * 3 + 2] = nz / l;
  }

  const KEY = 0x4000000; // matches boundaryEdges/featureEdges: exact for vertex ids below ~67M
  const key = (a: number, b: number) => (a < b ? a * KEY + b : b * KEY + a);
  const cosThresh = Math.cos((angleDeg * Math.PI) / 180);

  const cnt = new Map<number, number>();
  const firstTri = new Map<number, number>();
  const crease = new Set<number>();
  const consider = (a: number, b: number, t: number): void => {
    const k = key(a, b);
    const n = (cnt.get(k) ?? 0) + 1;
    cnt.set(k, n);
    if (n === 1) { firstTri.set(k, t); return; }
    if (n > 2) { crease.add(k); return; } // non-manifold junction
    const o = firstTri.get(k)!;
    if (degenerate[o] || degenerate[t]) return;
    const dot = nrm[o * 3]! * nrm[t * 3]! + nrm[o * 3 + 1]! * nrm[t * 3 + 1]! + nrm[o * 3 + 2]! * nrm[t * 3 + 2]!;
    if (dot < cosThresh) crease.add(k);
  };
  for (let t = 0; t < tris; t++) {
    const a = idx[t * 3]!, b = idx[t * 3 + 1]!, c = idx[t * 3 + 2]!;
    consider(a, b, t); consider(b, c, t); consider(c, a, t);
  }

  const out: number[] = [];
  const segSolid: number[] = [];
  const seen = new Set<number>();
  const emit = (u: number, v: number, s: number): void => {
    const k = key(u, v);
    if (seen.has(k)) return;
    if (cnt.get(k) !== 1 && !crease.has(k)) return; // smooth interior edge: skip
    seen.add(k);
    out.push(pos[u * 3]!, pos[u * 3 + 1]!, pos[u * 3 + 2]!);
    out.push(pos[v * 3]!, pos[v * 3 + 1]!, pos[v * 3 + 2]!);
    segSolid.push(s);
  };
  for (let t = 0; t < tris; t++) {
    const a = idx[t * 3]!, b = idx[t * 3 + 1]!, c = idx[t * 3 + 2]!;
    const s = solidOfTri[t] ?? 0;
    emit(a, b, s); emit(b, c, s); emit(c, a, s);
  }
  return { positions: new Float32Array(out), count: segSolid.length, solidOfSeg: Uint32Array.from(segSolid) };
}

/** Triangle index buffer with every triangle of a hidden solid removed. */
export function filterTriangles(fullIndex: Uint32Array, solidOfTri: Uint32Array, hidden: ReadonlySet<number>): Uint32Array {
  const out = new Uint32Array(fullIndex.length);
  let n = 0;
  for (let t = 0; t < solidOfTri.length; t++) {
    if (hidden.has(solidOfTri[t]!)) continue;
    out[n] = fullIndex[t * 3]!; out[n + 1] = fullIndex[t * 3 + 1]!; out[n + 2] = fullIndex[t * 3 + 2]!;
    n += 3;
  }
  return out.subarray(0, n);
}

/** An EdgeSet minus the segments belonging to the given solids — used to strip open-by-design
 * sheet-body boundaries out of the open-edge (defect) overlay. */
export function dropSolidSegments(edges: EdgeSet, drop: ReadonlySet<number>): EdgeSet {
  if (drop.size === 0) return edges;
  const positions = new Float32Array(edges.positions.length);
  const solidOfSeg = new Uint32Array(edges.count);
  let n = 0;
  for (let s = 0; s < edges.count; s++) {
    if (drop.has(edges.solidOfSeg[s]!)) continue;
    positions.set(edges.positions.subarray(s * 6, s * 6 + 6), n * 6);
    solidOfSeg[n] = edges.solidOfSeg[s]!;
    n++;
  }
  return { positions: positions.subarray(0, n * 6), count: n, solidOfSeg: solidOfSeg.subarray(0, n) };
}

/** Line-segment positions with every segment of a hidden solid removed. */
export function filterSegments(edges: EdgeSet, hidden: ReadonlySet<number>): Float32Array {
  const out = new Float32Array(edges.positions.length);
  let n = 0;
  for (let s = 0; s < edges.count; s++) {
    if (hidden.has(edges.solidOfSeg[s]!)) continue;
    out.set(edges.positions.subarray(s * 6, s * 6 + 6), n);
    n += 6;
  }
  return out.subarray(0, n);
}

/**
 * Per-vertex colors for STEP face colors, splitting welded vertices along color borders.
 * The mesh welds face boundaries, so a border vertex is shared by triangles of different
 * colors — a single per-vertex color would smear a gradient across every border triangle.
 * Vertices whose incident triangles all agree keep their index; a vertex on a color border
 * is duplicated once per additional color, so borders stay crisp. `colorOfTri` holds a
 * palette index per triangle (-1 = unstyled -> defaultRGB). Colors must be in the renderer's
 * working color space (convert sRGB before calling).
 */
export function splitByTriColor(
  mesh: RawMesh,
  colorOfTri: Int32Array,
  palette: ReadonlyArray<readonly [number, number, number]>,
  defaultRGB: readonly [number, number, number],
): { mesh: RawMesh; colors: Float32Array } {
  const nV = mesh.positions.length / 3;
  const nT = colorOfTri.length;
  const indices = new Uint32Array(nT * 3);
  const vertColor = new Int32Array(nV).fill(-2); // -2 = not seen yet
  // (vertex, color) -> duplicated vertex index. Key stays exact below 2^53 (nV in the millions,
  // palette small).
  const K = palette.length + 2;
  const dup = new Map<number, number>();
  const extraPos: number[] = [];
  const extraColor: number[] = [];
  for (let t = 0; t < nT; t++) {
    const c = colorOfTri[t]!;
    for (let e = 0; e < 3; e++) {
      const v = mesh.indices[t * 3 + e]!;
      if (vertColor[v] === -2) vertColor[v] = c;
      if (vertColor[v] === c) { indices[t * 3 + e] = v; continue; }
      const k = v * K + (c + 1);
      let nv = dup.get(k);
      if (nv === undefined) {
        nv = nV + extraColor.length;
        dup.set(k, nv);
        extraPos.push(mesh.positions[v * 3]!, mesh.positions[v * 3 + 1]!, mesh.positions[v * 3 + 2]!);
        extraColor.push(c);
      }
      indices[t * 3 + e] = nv;
    }
  }
  const totalV = nV + extraColor.length;
  const colors = new Float32Array(totalV * 3);
  const setC = (i: number, c: number): void => {
    const rgb = c >= 0 ? palette[c]! : defaultRGB;
    colors[i * 3] = rgb[0]; colors[i * 3 + 1] = rgb[1]; colors[i * 3 + 2] = rgb[2];
  };
  for (let v = 0; v < nV; v++) setC(v, vertColor[v]!);
  for (let i = 0; i < extraColor.length; i++) setC(nV + i, extraColor[i]!);
  const positions = new Float64Array(totalV * 3);
  positions.set(mesh.positions);
  positions.set(extraPos, nV * 3);
  return { mesh: { positions, indices }, colors };
}

/** Triangle count of an indexed mesh. */
export function triCount(mesh: RawMesh): number {
  return mesh.indices.length / 3;
}

/**
 * "Turbo"-style colormap (Google), good for deviation maps. t in [0,1] -> [r,g,b] in [0,1].
 * Polynomial approximation (Anton Mikhailov).
 */
export function turbo(t: number): [number, number, number] {
  const x = Math.min(1, Math.max(0, t));
  const r =
    0.13572138 + x * (4.6153926 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
  const g =
    0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
  const b =
    0.1066733 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
  return [Math.min(1, Math.max(0, r)), Math.min(1, Math.max(0, g)), Math.min(1, Math.max(0, b))];
}

/** Diverging blue-white-red map for SIGNED deviation, d in [-range, +range]. */
export function diverging(d: number, range: number): [number, number, number] {
  if (range <= 0) return [1, 1, 1];
  const t = Math.min(1, Math.max(-1, d / range)); // -1..1
  if (t < 0) {
    const a = 1 + t; // 0 at -range, 1 at 0
    return [a, a, 1]; // blue -> white
  }
  const a = 1 - t; // 1 at 0, 0 at +range
  return [1, a, a]; // white -> red
}
