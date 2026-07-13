// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";
import { EdgeTable } from "../../src/mesh/edge-table.ts";

export interface RawMesh {
  positions: Float64Array; // 3 per vertex
  indices: Uint32Array; // 3 per triangle
}

/** Build an indexed BufferGeometry (Float32 positions) from a meshStep IndexedMesh.
 * `normals` (e.g. from autoSmooth) is used verbatim when given, else averaged vertex normals. */
export function buildIndexedGeometry(mesh: RawMesh, normals?: Float32Array): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(mesh.positions), 3));
  g.setIndex(new THREE.Uint32BufferAttribute(mesh.indices.slice(), 1));
  if (normals) g.setAttribute("normal", new THREE.Float32BufferAttribute(normals, 3));
  else g.computeVertexNormals();
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
  // EdgeTable, not a Map: a large assembly mesh carries more unique edges than a Map's 2^24 cap
  // ("RangeError: Map maximum size exceeded" on a ~20M-triangle conversion).
  const useCount = new EdgeTable(idx.length / 2);

  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t]!, b = idx[t + 1]!, c = idx[t + 2]!;
    useCount.bump(a, b); useCount.bump(b, c); useCount.bump(c, a);
  }

  // Second pass: collect the actual vertex pairs for edges used once.
  const out: number[] = [];
  const segSolid: number[] = [];
  const emit = (u: number, v: number, s: number): void => {
    if (useCount.cnt[useCount.find(u, v)]! !== 1) return;
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
  // EdgeTable (no 2^24 Map cap): cnt = use-count, v0 = first face id, v1 = flag bits.
  const FEATURE = 1, SEEN = 2;
  const et = new EdgeTable(idx.length / 2, 2);
  const consider = (a: number, b: number, f: number): void => {
    const s = et.bump(a, b);
    if (et.cnt[s]! === 1) { et.v0[s] = f; et.v1[s] = 0; } // v1 lane initialises to -1, not 0
    else if (et.v0[s] !== f) et.v1[s] = et.v1[s]! | FEATURE; // edge straddles two distinct faces
  };
  const tris = idx.length / 3;
  for (let t = 0; t < tris; t++) {
    const f = faceOfTri[t] ?? 0;
    const a = idx[t * 3]!, b = idx[t * 3 + 1]!, c = idx[t * 3 + 2]!;
    consider(a, b, f); consider(b, c, f); consider(c, a, f);
  }

  const out: number[] = [];
  const segSolid: number[] = [];
  const emit = (u: number, v: number, s: number): void => {
    const k = et.find(u, v);
    if (et.v1[k]! & SEEN) return;
    if (et.cnt[k]! !== 1 && !(et.v1[k]! & FEATURE)) return; // interior-to-a-face edge: skip
    et.v1[k] = et.v1[k]! | SEEN;
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

  const cosThresh = Math.cos((angleDeg * Math.PI) / 180);

  // EdgeTable (no 2^24 Map cap): cnt = use-count, v0 = first incident triangle, v1 = flag bits.
  const CREASE = 1, SEEN = 2;
  const et = new EdgeTable(idx.length / 2, 2);
  const consider = (a: number, b: number, t: number): void => {
    const s = et.bump(a, b);
    const n = et.cnt[s]!;
    if (n === 1) { et.v0[s] = t; et.v1[s] = 0; return; } // v1 lane initialises to -1, not 0
    if (n > 2) { et.v1[s] = et.v1[s]! | CREASE; return; } // non-manifold junction
    const o = et.v0[s]!;
    if (degenerate[o] || degenerate[t]) return;
    const dot = nrm[o * 3]! * nrm[t * 3]! + nrm[o * 3 + 1]! * nrm[t * 3 + 1]! + nrm[o * 3 + 2]! * nrm[t * 3 + 2]!;
    if (dot < cosThresh) et.v1[s] = et.v1[s]! | CREASE;
  };
  for (let t = 0; t < tris; t++) {
    const a = idx[t * 3]!, b = idx[t * 3 + 1]!, c = idx[t * 3 + 2]!;
    consider(a, b, t); consider(b, c, t); consider(c, a, t);
  }

  const out: number[] = [];
  const segSolid: number[] = [];
  const emit = (u: number, v: number, s: number): void => {
    const k = et.find(u, v);
    if (et.v1[k]! & SEEN) return;
    if (et.cnt[k]! !== 1 && !(et.v1[k]! & CREASE)) return; // smooth interior edge: skip
    et.v1[k] = et.v1[k]! | SEEN;
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
 * Auto-smooth: split welded vertices along crease edges and compute crease-aware vertex
 * normals, so the mesh shades smooth across curvature but keeps sharp edges sharp.
 *
 * A feature (crease) edge uses the same rule as creaseEdges: an interior manifold edge is a
 * crease when its two triangles' normals differ by more than `angleDeg`; boundary edges and
 * non-manifold junctions (3+ triangles) always are. CAD face borders are deliberately NOT a
 * criterion — tangent-continuous borders (fillet blends) must stay smooth, and genuinely sharp
 * borders exceed the angle anyway.
 *
 * Around each vertex, incident triangle corners connected through smooth edges form one
 * smoothing group (union-find); each group becomes one output vertex whose normal is the
 * area-weighted average of its triangles. Triangle order and count are unchanged, so
 * per-triangle attributes (solidOfTri, faceOfTri) stay valid. `src` maps each output vertex
 * to its input vertex for remapping per-vertex attributes (face colors).
 */
export function autoSmooth(mesh: RawMesh, angleDeg: number): { mesh: RawMesh; normals: Float32Array; src: Uint32Array } {
  const idx = mesh.indices;
  const pos = mesh.positions;
  const tris = idx.length / 3;

  // Raw cross products (length = 2·area) for area-weighted averaging; unit normals for the
  // angle test. A degenerate triangle's unit normal stays [0,0,0] and can never pass the
  // cos test, so nothing smooths across it.
  const raw = new Float64Array(tris * 3);
  const unit = new Float64Array(tris * 3);
  for (let t = 0; t < tris; t++) {
    const a = idx[t * 3]! * 3, b = idx[t * 3 + 1]! * 3, c = idx[t * 3 + 2]! * 3;
    const abx = pos[b]! - pos[a]!, aby = pos[b + 1]! - pos[a + 1]!, abz = pos[b + 2]! - pos[a + 2]!;
    const acx = pos[c]! - pos[a]!, acy = pos[c + 1]! - pos[a + 1]!, acz = pos[c + 2]! - pos[a + 2]!;
    const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
    raw[t * 3] = nx; raw[t * 3 + 1] = ny; raw[t * 3 + 2] = nz;
    const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (l < 1e-30) continue;
    unit[t * 3] = nx / l; unit[t * 3 + 1] = ny / l; unit[t * 3 + 2] = nz / l;
  }

  const cosThresh = Math.cos((angleDeg * Math.PI) / 180);

  // Pass 1: edge use-counts + first incident triangle, so only exactly-two-triangle edges
  // can smooth (a 3rd triangle arriving later must not have already merged the first pair).
  // EdgeTable (no 2^24 Map cap): cnt = use-count, v0 = first incident triangle.
  const et = new EdgeTable(idx.length / 2, 1);
  for (let t = 0; t < tris; t++) {
    const a = idx[t * 3]!, b = idx[t * 3 + 1]!, c = idx[t * 3 + 2]!;
    for (const [u, v] of [[a, b], [b, c], [c, a]] as const) {
      const s = et.bump(u, v);
      if (et.cnt[s]! === 1) et.v0[s] = t;
    }
  }

  // Union-find over triangle corners (a corner = one triangle's use of one vertex).
  const parent = new Int32Array(idx.length);
  for (let i = 0; i < parent.length; i++) parent[i] = i;
  const find = (x: number): number => {
    while (parent[x] !== x) { parent[x] = parent[parent[x]!]!; x = parent[x]!; }
    return x;
  };
  const cornerOf = (t: number, v: number): number =>
    idx[t * 3] === v ? t * 3 : idx[t * 3 + 1] === v ? t * 3 + 1 : t * 3 + 2;

  // Pass 2: merge corners across smooth manifold edges.
  for (let t = 0; t < tris; t++) {
    const va = idx[t * 3]!, vb = idx[t * 3 + 1]!, vc = idx[t * 3 + 2]!;
    for (const [a, b] of [[va, vb], [vb, vc], [vc, va]] as const) {
      const s = et.find(a, b);
      if (et.cnt[s]! !== 2) continue;
      const o = et.v0[s]!;
      if (o === t) continue; // this is the edge's first triangle; its partner does the merge
      const dot = unit[o * 3]! * unit[t * 3]! + unit[o * 3 + 1]! * unit[t * 3 + 1]! + unit[o * 3 + 2]! * unit[t * 3 + 2]!;
      if (dot < cosThresh) continue; // crease: keep the corners split
      parent[find(cornerOf(t, a))] = find(cornerOf(o, a));
      parent[find(cornerOf(t, b))] = find(cornerOf(o, b));
    }
  }

  // Pass 3: one output vertex per corner group; accumulate area-weighted normals.
  const outIdx = new Uint32Array(idx.length);
  const groupOfRoot = new Int32Array(idx.length).fill(-1);
  let nOut = 0;
  for (let c = 0; c < idx.length; c++) {
    const r = find(c);
    if (groupOfRoot[r] === -1) groupOfRoot[r] = nOut++;
    outIdx[c] = groupOfRoot[r]!;
  }
  const positions = new Float64Array(nOut * 3);
  const acc = new Float64Array(nOut * 3);
  const src = new Uint32Array(nOut);
  for (let c = 0; c < idx.length; c++) {
    const g = outIdx[c]!, v = idx[c]!, t = (c - (c % 3)) / 3;
    src[g] = v;
    positions[g * 3] = pos[v * 3]!; positions[g * 3 + 1] = pos[v * 3 + 1]!; positions[g * 3 + 2] = pos[v * 3 + 2]!;
    acc[g * 3] += raw[t * 3]!; acc[g * 3 + 1] += raw[t * 3 + 1]!; acc[g * 3 + 2] += raw[t * 3 + 2]!;
  }
  const normals = new Float32Array(nOut * 3);
  for (let g = 0; g < nOut; g++) {
    const x = acc[g * 3]!, y = acc[g * 3 + 1]!, z = acc[g * 3 + 2]!;
    const l = Math.sqrt(x * x + y * y + z * z);
    if (l > 1e-30) { normals[g * 3] = x / l; normals[g * 3 + 1] = y / l; normals[g * 3 + 2] = z / l; }
    else normals[g * 3 + 2] = 1; // group of degenerate triangles only: any unit vector beats NaN
  }
  return { mesh: { positions, indices: outIdx }, normals, src };
}

/**
 * Indexed wireframe: two indices per unique undirected edge of the given triangle index buffer.
 * Replaces THREE.WireframeGeometry, which dedups edges through a Set of STRING hashes (two per
 * edge) — it hits V8's 2^24 Set cap around 8M unique edges ("RangeError: Set maximum size
 * exceeded" on a ~17M-triangle assembly) and duplicates every position. The returned buffer is
 * meant for a LineSegments geometry that SHARES the mesh's position attribute.
 */
export function wireframeIndex(indices: ArrayLike<number>): Uint32Array {
  const et = new EdgeTable(indices.length / 2);
  const out = new Uint32Array(indices.length * 2); // upper bound: 3 unique edges per triangle
  let n = 0;
  for (let t = 0; t + 2 < indices.length; t += 3) {
    for (let e = 0; e < 3; e++) {
      const a = indices[t + e]!, b = indices[t + ((e + 1) % 3)]!;
      const s = et.bump(a, b);
      if (et.cnt[s]! === 1) { out[n] = a; out[n + 1] = b; n += 2; }
    }
  }
  return out.slice(0, n);
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
