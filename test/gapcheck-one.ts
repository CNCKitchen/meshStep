// SPDX-License-Identifier: AGPL-3.0-only
// Gap-finder worker: convert ONE STEP file with meshStep and with OpenCASCADE
// (occt-import-js WASM), compare the two meshes with global AND localized metrics,
// and print a JSON verdict on stdout. Run via gapcheck.ts (which adds process
// isolation + timeouts), or standalone:
//   node test/gapcheck-one.ts path/to/model.step
//
// Metrics (all distances normalized by the OCC bbox diagonal "D"):
//  - ours->OCC sampled deviation, aggregated PER STEP FACE (catches a single
//    distorted patch that a whole-model RMS would average away)
//  - OCC->ours sampled deviation, aggregated PER OCC B-REP FACE (catches faces
//    we dropped or collapsed — invisible to any ours->reference metric)
//  - missing-area estimate (fraction of OCC surface area with no counterpart)
//  - area ratio, signed-volume ratio, bbox scale ratio (unit bugs)
//  - watertightness (boundary / non-manifold edges), sliver %, face coverage
import { readFileSync, writeFileSync } from "node:fs";
import { basename, join } from "node:path";
import { createRequire } from "node:module";
import { importStep, writeBinarySTL } from "../src/index.ts";
import { buildBrep } from "../src/brep/build.ts";
import { buildBVH, closestDist, type TriBVH } from "./closest.ts";
import { renderPNGFile, bboxOf, growBox, type Box3, type EdgeSet, type Layer } from "./draw.ts";

const require = createRequire(import.meta.url);

export interface GapOptions {
  /** meshStep surface deviation as a fraction of the bbox diagonal. */
  surfDevRel: number;
  /** OCC linear deflection as a fraction of the bbox diagonal. */
  occDeflRel: number;
  /** meshStep max edge as a fraction of the bbox diagonal. */
  maxEdgeRel: number;
  /** Area-uniform samples per direction. */
  samples: number;
  /** WARN when a sample deviates more than warnFactor * (surfDev + occDefl). */
  warnFactor: number;
  /** FAIL when a sample deviates more than failFactor * (surfDev + occDefl). */
  failFactor: number;
  /** Write the converted mesh as binary STL into this directory. */
  stlDir?: string;
  /** Render failure pictures (full + defect zoom) into this directory. */
  imgDir?: string;
}
export const DEFAULT_OPTS: GapOptions = {
  surfDevRel: 5e-4, occDeflRel: 5e-4, maxEdgeRel: 0.05,
  samples: 20000, warnFactor: 3, failFactor: 10,
};

// ---------- small mesh helpers ----------

interface Soup { tris: Float64Array; faceOfTri: Uint32Array; nTris: number }

const soupFromIndexed = (m: { positions: Float64Array; indices: Uint32Array }, faceOfTri: Uint32Array): Soup => {
  const nTris = m.indices.length / 3;
  const tris = new Float64Array(nTris * 9);
  for (let i = 0; i < m.indices.length; i++) {
    const v = m.indices[i]! * 3;
    tris[i * 3] = m.positions[v]!;
    tris[i * 3 + 1] = m.positions[v + 1]!;
    tris[i * 3 + 2] = m.positions[v + 2]!;
  }
  return { tris, faceOfTri, nTris };
};

const triArea = (T: Float64Array, o: number): number => {
  const ux = T[o + 3]! - T[o]!, uy = T[o + 4]! - T[o + 1]!, uz = T[o + 5]! - T[o + 2]!;
  const vx = T[o + 6]! - T[o]!, vy = T[o + 7]! - T[o + 1]!, vz = T[o + 8]! - T[o + 2]!;
  const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
  return Math.hypot(cx, cy, cz) / 2;
};

const soupBbox = (T: Float64Array): { diag: number; min: number[]; max: number[] } => {
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < T.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      const v = T[i + k]!;
      if (v < min[k]!) min[k] = v;
      if (v > max[k]!) max[k] = v;
    }
  }
  return { diag: Math.hypot(max[0]! - min[0]!, max[1]! - min[1]!, max[2]! - min[2]!), min, max };
};

const soupVolume = (T: Float64Array): number => {
  let vol = 0;
  for (let o = 0; o < T.length; o += 9) {
    vol += (T[o]! * (T[o + 4]! * T[o + 8]! - T[o + 5]! * T[o + 7]!)
      - T[o + 1]! * (T[o + 3]! * T[o + 8]! - T[o + 5]! * T[o + 6]!)
      + T[o + 2]! * (T[o + 3]! * T[o + 7]! - T[o + 4]! * T[o + 6]!)) / 6;
  }
  return vol;
};

interface Watertight {
  boundary: number;
  nonmanifold: number;
  /** World-space segments (6 doubles each) for rendering, capped at 20k each. */
  boundarySegs: Float64Array;
  nmSegs: Float64Array;
  /** STEP face id -> number of open edges owned by that face's triangles. */
  openEdgesOfFace: Map<number, number>;
}

const watertight = (m: { positions: Float64Array; indices: Uint32Array }, faceOfTri: Uint32Array, skipTri?: (t: number) => boolean): Watertight => {
  const K = 0x40000000;
  const inc = new Map<number, { n: number; fwd: number; a: number; b: number; tri: number }>();
  for (let i = 0; i < m.indices.length; i += 3) {
    if (skipTri?.(i / 3)) continue; // open-shell surface bodies: boundary is open by design
    const a = m.indices[i]!, b = m.indices[i + 1]!, c = m.indices[i + 2]!;
    for (const [x, y] of [[a, b], [b, c], [c, a]] as const) {
      const k = x < y ? x * K + y : y * K + x;
      const e = inc.get(k);
      if (e) { e.n++; if (x < y) e.fwd++; }
      else inc.set(k, { n: 1, fwd: x < y ? 1 : 0, a: x, b: y, tri: i / 3 });
    }
  }
  const CAP = 20000;
  const bSegs: number[] = [], nmSegs: number[] = [];
  const openEdgesOfFace = new Map<number, number>();
  let boundary = 0, nonmanifold = 0;
  const P = m.positions;
  const pushSeg = (out: number[], a: number, b: number): void => {
    if (out.length >= CAP * 6) return;
    out.push(P[a * 3]!, P[a * 3 + 1]!, P[a * 3 + 2]!, P[b * 3]!, P[b * 3 + 1]!, P[b * 3 + 2]!);
  };
  for (const e of inc.values()) {
    if (e.n === 2) continue;
    if (e.n === 1) {
      boundary++;
      pushSeg(bSegs, e.a, e.b);
      const f = faceOfTri[e.tri]!;
      openEdgesOfFace.set(f, (openEdgesOfFace.get(f) ?? 0) + 1);
    } else if (e.n % 2 === 0 && 2 * e.fwd === e.n) {
      // Balanced even-count edge: pairs of consistently-wound manifold sheets welded together —
      // a SELF-TOUCHING solid (two coincident B-rep edges, each carrying 2 faces, along a tangent
      // contact line: Stealthburner). Faithful topology, not a defect. A folded flap traverses the
      // edge twice the SAME way and stays unbalanced -> still flagged below.
      continue;
    } else {
      nonmanifold++;
      pushSeg(nmSegs, e.a, e.b);
    }
  }
  return { boundary, nonmanifold, boundarySegs: Float64Array.from(bSegs), nmSegs: Float64Array.from(nmSegs), openEdgesOfFace };
};

const sliverPct = (S: Soup): number => {
  const T = S.tris;
  let slivers = 0;
  for (let o = 0; o < T.length; o += 9) {
    const ab = Math.hypot(T[o + 3]! - T[o]!, T[o + 4]! - T[o + 1]!, T[o + 5]! - T[o + 2]!);
    const bc = Math.hypot(T[o + 6]! - T[o + 3]!, T[o + 7]! - T[o + 4]!, T[o + 8]! - T[o + 5]!);
    const ca = Math.hypot(T[o]! - T[o + 6]!, T[o + 1]! - T[o + 7]!, T[o + 2]! - T[o + 8]!);
    const ang = (p: number, q: number, r: number): number => {
      const v = (p * p + q * q - r * r) / (2 * p * q || 1e-30);
      return Math.acos(Math.max(-1, Math.min(1, v)));
    };
    if (Math.min(ang(ab, ca, bc), ang(ab, bc, ca), ang(bc, ca, ab)) < 20 * Math.PI / 180) slivers++;
  }
  return S.nTris ? (100 * slivers) / S.nTris : 0;
};

// ---------- sampling ----------

const mulberry32 = (seed: number) => (): number => {
  seed = (seed + 0x6d2b79f5) | 0;
  let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
};

/** Area-uniform samples over a soup; returns points + originating triangle index. */
function areaSamples(S: Soup, count: number, rng: () => number): { pts: Float64Array; tri: Uint32Array } {
  const cum = new Float64Array(S.nTris);
  let acc = 0;
  for (let t = 0; t < S.nTris; t++) { acc += triArea(S.tris, t * 9); cum[t] = acc; }
  const pts = new Float64Array(count * 3);
  const tri = new Uint32Array(count);
  for (let s = 0; s < count; s++) {
    const target = rng() * acc;
    let lo = 0, hi = S.nTris - 1;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (cum[mid]! < target) lo = mid + 1; else hi = mid; }
    const o = lo * 9;
    let u = rng(), v = rng();
    if (u + v > 1) { u = 1 - u; v = 1 - v; }
    tri[s] = lo;
    pts[s * 3] = S.tris[o]! + u * (S.tris[o + 3]! - S.tris[o]!) + v * (S.tris[o + 6]! - S.tris[o]!);
    pts[s * 3 + 1] = S.tris[o + 1]! + u * (S.tris[o + 4]! - S.tris[o + 1]!) + v * (S.tris[o + 7]! - S.tris[o + 1]!);
    pts[s * 3 + 2] = S.tris[o + 2]! + u * (S.tris[o + 5]! - S.tris[o + 2]!) + v * (S.tris[o + 8]! - S.tris[o + 2]!);
  }
  return { pts, tri };
}

/** Up to 3 representative triangles per face (first / largest / last) so every face
 * gets probed even if the area-uniform pass never landed on it (tiny faces). */
function faceRepTris(S: Soup): Map<number, number[]> {
  const rep = new Map<number, number[]>();
  const largestArea = new Map<number, number>();
  for (let t = 0; t < S.nTris; t++) {
    const f = S.faceOfTri[t]!;
    const a = triArea(S.tris, t * 9);
    let r = rep.get(f);
    if (!r) { rep.set(f, [t, t, t]); largestArea.set(f, a); continue; }
    if (a > largestArea.get(f)!) { r[1] = t; largestArea.set(f, a); }
    r[2] = t;
  }
  return rep;
}

// ---------- per-face aggregation ----------

interface FaceAgg { max: number; sum: number; n: number; bad: number; at: [number, number, number] }

function aggregate(
  agg: Map<number, FaceAgg>, face: number, d: number, failThresh: number,
  x: number, y: number, z: number,
): void {
  let a = agg.get(face);
  if (!a) { a = { max: -1, sum: 0, n: 0, bad: 0, at: [0, 0, 0] }; agg.set(face, a); }
  a.sum += d; a.n++;
  if (d > failThresh) a.bad++;
  if (d > a.max) { a.max = d; a.at = [x, y, z]; }
}

/** Measure samples of A against BVH of B; aggregates per source face of A. */
function measure(
  pts: Float64Array, tri: Uint32Array, src: Soup, bvh: TriBVH,
  agg: Map<number, FaceAgg>, failThresh: number,
): Float64Array {
  const n = pts.length / 3;
  const dists = new Float64Array(n);
  for (let s = 0; s < n; s++) {
    const x = pts[s * 3]!, y = pts[s * 3 + 1]!, z = pts[s * 3 + 2]!;
    const d = closestDist(bvh, x, y, z);
    dists[s] = d;
    aggregate(agg, src.faceOfTri[tri[s]!]!, d, failThresh, x, y, z);
  }
  return dists;
}

const percentile = (dists: Float64Array, p: number): number => {
  if (dists.length === 0) return 0;
  const sorted = Float64Array.from(dists).sort();
  return sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))]!;
};

// ---------- OCC reference ----------

interface OccResult { soup: Soup; faceCount: number; shapes: number }

async function occConvert(path: string, deflRel: number): Promise<OccResult | null> {
  const occtimportjs = require("occt-import-js");
  // OCC logs (e.g. "*** ERR StepReaderData ***") default to stdout and would corrupt
  // the JSON result stream — route them to stderr.
  const occt = await occtimportjs({ print: (t: string) => console.error(t), printErr: (t: string) => console.error(t) });
  const buf = new Uint8Array(readFileSync(path));
  const r = occt.ReadStepFile(buf, {
    linearUnit: "millimeter",
    linearDeflectionType: "bounding_box_ratio",
    linearDeflection: deflRel,
    angularDeflection: 0.35,
  });
  if (!r.success || !r.meshes?.length) return null;
  let nTris = 0, nFaces = 0;
  for (const m of r.meshes) { nTris += m.index.array.length / 3; nFaces += m.brep_faces?.length ?? 0; }
  const tris = new Float64Array(nTris * 9);
  const faceOfTri = new Uint32Array(nTris);
  let triOff = 0, faceOff = 0;
  for (const m of r.meshes) {
    const P = m.attributes.position.array, I = m.index.array;
    const mTris = I.length / 3;
    for (let i = 0; i < I.length; i++) {
      const v = I[i] * 3;
      tris[(triOff * 3 + i) * 3] = P[v];
      tris[(triOff * 3 + i) * 3 + 1] = P[v + 1];
      tris[(triOff * 3 + i) * 3 + 2] = P[v + 2];
    }
    const bf = m.brep_faces ?? [];
    for (let f = 0; f < bf.length; f++) {
      for (let t = bf[f].first; t <= bf[f].last; t++) faceOfTri[triOff + t] = faceOff + f;
    }
    triOff += mTris; faceOff += bf.length;
  }
  return { soup: { tris, faceOfTri, nTris }, faceCount: faceOff, shapes: r.meshes.length };
}

// ---------- main ----------

const errMsg = (e: unknown): string => (e instanceof Error ? e.message : String(e));

export async function gapcheckOne(path: string, opts: GapOptions): Promise<Record<string, unknown>> {
  const reasons: string[] = [];
  const rec: Record<string, unknown> = { file: basename(path), status: "OK", reasons };
  const seed = [...basename(path)].reduce((h, c) => (Math.imul(h, 31) + c.charCodeAt(0)) | 0, 7);
  const rng = mulberry32(seed);

  // -- OCC reference --
  console.error("[stage] occ");
  let t0 = Date.now();
  let occ: OccResult | null = null;
  try { occ = await occConvert(path, opts.occDeflRel); }
  catch (e) { reasons.push(`OCC threw: ${errMsg(e)}`); }
  const occMs = Date.now() - t0;
  const occDiag = occ ? soupBbox(occ.soup.tris).diag : 0;
  if (occ) {
    let area = 0;
    for (let t = 0; t < occ.soup.nTris; t++) area += triArea(occ.soup.tris, t * 9);
    rec.occ = { tris: occ.soup.nTris, faces: occ.faceCount, shapes: occ.shapes, area, volume: Math.abs(soupVolume(occ.soup.tris)), diag: occDiag, ms: occMs };
  } else {
    rec.occ = null;
    reasons.push("OCC could not read the file (no reference)");
  }

  // -- ours --
  console.error("[stage] ours");
  t0 = Date.now();
  const src = readFileSync(path, "utf8");
  let res: ReturnType<typeof importStep>;
  try {
    res = importStep(src, occ
      ? { surfaceDeviation: opts.surfDevRel * occDiag, maxEdge: opts.maxEdgeRel * occDiag }
      : {});
  } catch (e) {
    rec.status = "OURS_ERR";
    reasons.push(`importStep threw: ${errMsg(e)}`);
    return rec;
  }
  const oursMs = Date.now() - t0;
  const ours = soupFromIndexed(res.mesh, res.faceOfTri);
  const openSet = new Set(res.openSolids ?? []);
  const wt = watertight(res.mesh, res.faceOfTri, openSet.size ? (t) => openSet.has(res.solidOfTri[t]!) : undefined);
  const outBase = basename(path).replace(/\.[^.]+$/, "").replace(/[^\w.\- ]+/g, "_");
  const analysis: { cause: string; fix: string }[] = [];
  rec.analysis = analysis;
  // STEP face id -> surface kind, parsed lazily (only failures need attribution)
  let kindCache: Map<number, string> | null = null;
  const faceKind = (id: number): string => {
    if (!kindCache) {
      kindCache = new Map();
      try {
        for (const s of buildBrep(src).solids) for (const f of s.faces) kindCache.set(f.faceId, f.surfaceKind);
      } catch { /* attribution stays "?" */ }
    }
    return kindCache.get(id) ?? "?";
  };
  let oursArea = 0;
  for (let t = 0; t < ours.nTris; t++) oursArea += triArea(ours.tris, t * 9);
  const oursBbox = soupBbox(ours.tris);
  rec.ours = {
    tris: ours.nTris, facesTessellated: res.stats.facesTessellated, facesTotal: res.stats.facesTotal,
    area: oursArea, volume: Math.abs(soupVolume(ours.tris)), diag: oursBbox.diag,
    boundaryEdges: wt.boundary, nonmanifoldEdges: wt.nonmanifold, sliverPct: sliverPct(ours),
    skipped: res.stats.skipped, ms: oursMs,
  };
  if (opts.stlDir && ours.nTris > 0) {
    writeFileSync(join(opts.stlDir, `${outBase}.stl`), writeBinarySTL(res.mesh));
    rec.stl = `${outBase}.stl`;
  }

  /** Failure pictures: full view + (if the defect is localized) a zoom onto it.
   * Red = open edges, magenta = non-manifold, orange = deviating faces, blue = OCC
   * faces missing from our mesh. */
  const emitImages = (flaggedFaces: Set<number>, missingTris: Float64Array | null, extraPts: number[]): void => {
    if (!opts.imgDir || ours.nTris === 0) return;
    console.error("[stage] render");
    const W = 1000, H = 800;
    let bodyTris = ours.tris, flagTris = new Float64Array(0);
    if (flaggedFaces.size) {
      let nf = 0;
      for (let t = 0; t < ours.nTris; t++) if (flaggedFaces.has(ours.faceOfTri[t]!)) nf++;
      flagTris = new Float64Array(nf * 9);
      bodyTris = new Float64Array((ours.nTris - nf) * 9);
      let fi = 0, bi = 0;
      for (let t = 0; t < ours.nTris; t++) {
        if (flaggedFaces.has(ours.faceOfTri[t]!)) { flagTris.set(ours.tris.subarray(t * 9, t * 9 + 9), fi); fi += 9; }
        else { bodyTris.set(ours.tris.subarray(t * 9, t * 9 + 9), bi); bi += 9; }
      }
    }
    const layers: Layer[] = [{ tris: bodyTris, rgb: [125, 150, 185] }];
    if (flagTris.length) layers.push({ tris: flagTris, rgb: [255, 150, 50] });
    if (missingTris?.length) layers.push({ tris: missingTris, rgb: [70, 150, 255] });
    const edges: EdgeSet[] = [];
    if (wt.boundarySegs.length) edges.push({ segs: wt.boundarySegs, rgb: [255, 45, 45], thick: 1 });
    if (wt.nmSegs.length) edges.push({ segs: wt.nmSegs, rgb: [255, 0, 255], thick: 1 });
    const images: string[] = [];
    const view: [number, number, number] = [-0.55, -0.65, -0.52];
    renderPNGFile(join(opts.imgDir, `${outBase}.png`), { width: W, height: H, layers, edges, viewDir: view });
    images.push(`${outBase}.png`);
    const defectArrs = [wt.boundarySegs, wt.nmSegs, flagTris, missingTris ?? new Float64Array(0), Float64Array.from(extraPts)]
      .filter((a) => a.length > 0);
    if (defectArrs.length) {
      const db: Box3 = bboxOf(defectArrs);
      const dSize = Math.hypot(db.max[0]! - db.min[0]!, db.max[1]! - db.min[1]!, db.max[2]! - db.min[2]!);
      if (Number.isFinite(dSize) && dSize < oursBbox.diag * 0.7) {
        renderPNGFile(join(opts.imgDir, `${outBase}.zoom.png`),
          { width: W, height: H, layers, edges, viewDir: view, fit: growBox(db, 0.35, oursBbox.diag * 0.06) });
        images.push(`${outBase}.zoom.png`);
      } else {
        renderPNGFile(join(opts.imgDir, `${outBase}.alt.png`),
          { width: W, height: H, layers, edges, viewDir: [0.55, 0.65, 0.52] });
        images.push(`${outBase}.alt.png`);
      }
    }
    rec.images = images;
  };

  if (ours.nTris === 0) { rec.status = "EMPTY"; reasons.push("meshStep produced 0 triangles"); return rec; }
  let oursIssues = 0;
  if (res.stats.facesTessellated < res.stats.facesTotal) {
    oursIssues++;
    const kinds = Object.keys(res.stats.skipped).join(", ") || "unknown";
    reasons.push(`skipped faces: ${res.stats.facesTotal - res.stats.facesTessellated}/${res.stats.facesTotal} (${Object.entries(res.stats.skipped).map(([k, v]) => `${k}:${v}`).join(" ")})`);
    analysis.push({
      cause: `${res.stats.facesTotal - res.stats.facesTessellated} faces were never tessellated (${kinds}) — each leaves a hole with open boundary edges.`,
      fix: kinds.includes("untriangulated")
        ? "CDT failed to realize boundary constraints (degenerate or self-intersecting (u,v) projection). Needs robust B-spline projection (bounded Newton, multi-start seeds) or a 3D fallback triangulator for these faces."
        : `Unsupported entity kind(s): ${kinds}. Implement the missing surface/curve evaluator (same pattern as OFFSET_SURFACE in commit bffeaa8).`,
    });
  }
  if (wt.boundary > 0 || wt.nonmanifold > 0) {
    oursIssues++;
    reasons.push(`not watertight: ${wt.boundary} boundary / ${wt.nonmanifold} non-manifold edges`);
    if (wt.boundary > 0) {
      const top = [...wt.openEdgesOfFace.entries()].sort((a, b) => b[1] - a[1]).slice(0, 5)
        .map(([f, n]) => ({ faceId: f, kind: faceKind(f), openEdges: n }));
      rec.openEdgeFaces = top;
      const byKind = new Map<string, number>();
      for (const [f, n] of wt.openEdgesOfFace) {
        const k = faceKind(f);
        byKind.set(k, (byKind.get(k) ?? 0) + n);
      }
      const kindsSorted = [...byKind.entries()].sort((a, b) => b[1] - a[1]);
      const dom = kindsSorted[0]?.[0] ?? "?";
      const FIX_BY_KIND: Record<string, string> = {
        B_SPLINE_SURFACE: "B-spline param-domain collapse / tangled projection (known cdt2d.ts issue class — see bspline diagnosis). Needs robust projection or 3D fallback.",
        PLANE: "Multi-loop trims on planes: region extraction in the CDT picks the wrong in/out region when constraints go unrealized.",
        CYLINDRICAL_SURFACE: "Boundary sampling mismatch between adjacent faces at a seam (loopParam/seam-projection class).",
      };
      analysis.push({
        cause: `${wt.boundary} open edges owned by: ${kindsSorted.map(([k, n]) => `${k}:${n}`).join(", ")}. Worst faces: ${top.map((t) => `#${t.faceId} (${t.kind}, ${t.openEdges})`).join(", ")}.`,
        fix: FIX_BY_KIND[dom] ?? `Dominant kind ${dom}: check CDT bail reasons and boundary sampling consistency for the listed faces.`,
      });
    }
    if (wt.nonmanifold > 0) {
      analysis.push({
        cause: `${wt.nonmanifold} non-manifold edges — overlapping/folded triangles.`,
        fix: "Usually a self-intersecting (u,v) boundary projection emitting folded triangles (B-spline projection robustness), or duplicate geometry. A bad fill can look worse than a hole — verify visually.",
      });
    }
  }
  if (!occ) {
    rec.status = oursIssues ? "FAIL" : "NOREF";
    analysis.push({
      cause: "OpenCASCADE could not read this file — no reference mesh, deviation metrics skipped.",
      fix: "Not necessarily our bug (OCC parse limitation). Review the picture/STL manually.",
    });
    emitImages(new Set(), null, []);
    return rec;
  }

  // -- scale sanity (unit bugs make distance metrics meaningless) --
  const scaleRatio = oursBbox.diag / occDiag;
  rec.scaleRatio = scaleRatio;
  if (scaleRatio < 0.95 || scaleRatio > 1.05) {
    rec.status = "SCALE";
    reasons.push(`bbox diagonal ratio ours/OCC = ${scaleRatio.toFixed(4)} (unit or placement bug?)`);
    analysis.push({
      cause: "Overall size disagrees with the OCC reference — a unit-conversion or assembly-placement bug, not a tessellation problem.",
      fix: "Check units detection (LENGTH_UNIT/SI prefix) and per-solid assembly transforms for this file.",
    });
    emitImages(new Set(), null, []);
    return rec;
  }

  // -- localized deviation, both directions --
  console.error("[stage] compare");
  t0 = Date.now();
  const base = (opts.surfDevRel + opts.occDeflRel) * occDiag; // combined chordal budget of the two meshes
  const warnT = opts.warnFactor * base;
  const failT = opts.failFactor * base;

  const occBvh = buildBVH(occ.soup.tris);
  const oursBvh = buildBVH(ours.tris);

  const ourAgg = new Map<number, FaceAgg>();
  const occAgg = new Map<number, FaceAgg>();

  const ourSamp = areaSamples(ours, opts.samples, rng);
  const occSamp = areaSamples(occ.soup, opts.samples, rng);
  const dOursToOcc = measure(ourSamp.pts, ourSamp.tri, ours, occBvh, ourAgg, failT);
  const dOccToOurs = measure(occSamp.pts, occSamp.tri, occ.soup, oursBvh, occAgg, failT);

  // guarantee every face on both sides is probed at least once (tiny faces)
  for (const [S, bvh, agg] of [[ours, occBvh, ourAgg], [occ.soup, oursBvh, occAgg]] as const) {
    for (const [face, reps] of faceRepTris(S)) {
      if ((agg.get(face)?.n ?? 0) >= 3) continue;
      for (const t of new Set(reps)) {
        const o = t * 9;
        const x = (S.tris[o]! + S.tris[o + 3]! + S.tris[o + 6]!) / 3;
        const y = (S.tris[o + 1]! + S.tris[o + 4]! + S.tris[o + 7]!) / 3;
        const z = (S.tris[o + 2]! + S.tris[o + 5]! + S.tris[o + 8]!) / 3;
        aggregate(agg, face, closestDist(bvh, x, y, z), failT, x, y, z);
      }
    }
  }

  const occArea = (rec.occ as { area: number }).area;
  const badOcc = dOccToOurs.reduce((n, d) => n + (d > failT ? 1 : 0), 0);
  const missingAreaFrac = badOcc / dOccToOurs.length;

  const worstOurFaces = [...ourAgg.entries()]
    .sort((a, b) => b[1].max - a[1].max).slice(0, 8)
    .map(([faceId, a]) => ({ faceId, max: a.max, mean: a.sum / a.n, n: a.n, bad: a.bad, at: a.at.map((v) => +v.toFixed(3)) }));
  const missingOccFaces = [...occAgg.entries()]
    .filter(([, a]) => a.bad >= Math.max(2, a.n * 0.5))
    .sort((a, b) => b[1].bad / b[1].n - a[1].bad / a[1].n || b[1].max - a[1].max).slice(0, 8)
    .map(([face, a]) => ({ face, max: a.max, badFrac: a.bad / a.n, n: a.n, at: a.at.map((v) => +v.toFixed(3)) }));

  rec.deviation = {
    diag: occDiag, base, warn: warnT, fail: failT,
    oursToOcc: { max: percentile(dOursToOcc, 1), p99: percentile(dOursToOcc, 0.99), mean: dOursToOcc.reduce((a, b) => a + b, 0) / dOursToOcc.length },
    occToOurs: { max: percentile(dOccToOurs, 1), p99: percentile(dOccToOurs, 0.99), mean: dOccToOurs.reduce((a, b) => a + b, 0) / dOccToOurs.length },
    missingAreaFrac, missingArea: missingAreaFrac * occArea,
    worstOurFaces, missingOccFaces,
    ms: Date.now() - t0,
  };

  // -- global sanity ratios --
  const areaRatio = oursArea / occArea;
  const volRatio = (rec.ours as { volume: number }).volume / (rec.occ as { volume: number }).volume;
  rec.areaRatio = areaRatio;
  rec.volRatio = volRatio;

  // -- verdict --
  let level = 0; // 0 ok, 1 warn, 2 fail
  const flag = (lvl: number, msg: string): void => { level = Math.max(level, lvl); reasons.push(msg); };
  const pctD = (d: number): string => `${((d / occDiag) * 100).toFixed(3)}%D`;

  for (const f of worstOurFaces) {
    if (f.bad >= 2) flag(2, `our face #${f.faceId} (${faceKind(f.faceId)}) deviates up to ${f.max.toFixed(3)}mm (${pctD(f.max)}) near [${f.at}]`);
    else if (f.max > warnT) flag(1, `our face #${f.faceId} (${faceKind(f.faceId)}) deviates up to ${f.max.toFixed(3)}mm (${pctD(f.max)}) near [${f.at}]`);
  }
  for (const f of missingOccFaces) {
    flag(2, `OCC face #${f.face}: ${(f.badFrac * 100).toFixed(0)}% of samples >${failT.toFixed(3)}mm from our mesh (missing/collapsed?) near [${f.at}]`);
  }
  if (missingAreaFrac > 0.001) flag(2, `~${(missingAreaFrac * 100).toFixed(2)}% of OCC surface area (${(missingAreaFrac * occArea).toFixed(1)}mm²) has no counterpart in our mesh`);
  if (Math.abs(areaRatio - 1) > 0.02) flag(2, `surface area off by ${((areaRatio - 1) * 100).toFixed(2)}%`);
  else if (Math.abs(areaRatio - 1) > 0.005) flag(1, `surface area off by ${((areaRatio - 1) * 100).toFixed(2)}%`);
  if (Math.abs(volRatio - 1) > 0.02) flag(2, `volume off by ${((volRatio - 1) * 100).toFixed(2)}%`);
  else if (Math.abs(volRatio - 1) > 0.005) flag(1, `volume off by ${((volRatio - 1) * 100).toFixed(2)}%`);
  if (wt.boundary > 0 || wt.nonmanifold > 0) level = 2;
  if (res.stats.facesTessellated < res.stats.facesTotal) level = 2;
  const dev = rec.deviation as { oursToOcc: { max: number } };
  if (level === 0 && dev.oursToOcc.max > warnT) flag(1, `max deviation ${dev.oursToOcc.max.toFixed(3)}mm (${pctD(dev.oursToOcc.max)})`);

  rec.status = level === 2 ? "FAIL" : level === 1 ? "WARN" : "OK";

  // -- cause analysis for the comparison-level findings --
  const flaggedFaces = new Set(worstOurFaces.filter((f) => f.bad >= 2).map((f) => f.faceId));
  if (flaggedFaces.size) {
    analysis.push({
      cause: `Face(s) ${[...flaggedFaces].map((f) => `#${f} (${faceKind(f)})`).join(", ")} lie off the reference surface while the rest of the model matches — a wrongly projected/placed patch, not tessellation noise.`,
      fix: "Compare that face's surface evaluation against OCC locally. Typical culprits: seam/period handling, degenerate parametrization, or a wrong same_sense/offset direction.",
    });
  }
  if (missingOccFaces.length) {
    analysis.push({
      cause: `${missingOccFaces.length} OCC face(s) have no counterpart within ${failT.toFixed(3)}mm — dropped or collapsed on our side (blue in the picture).`,
      fix: "If stats.skipped is 0 the face was emitted but collapsed — thin/degenerate param-domain class. Otherwise implement the missing surface kind.",
    });
  }
  if (Math.abs(volRatio - 1) > 0.02 && dev.oursToOcc.max < warnT && wt.boundary === 0) {
    if (occ.shapes > 1 && areaRatio < 0.99 && missingAreaFrac < 0.005) {
      analysis.push({
        cause: `Our mesh is on-surface everywhere yet has less area/volume than OCC, and OCC found ${occ.shapes} shapes — the STEP likely contains non-solid surface shells (coincident with solid faces) that we skip by design.`,
        fix: "Probably not a defect. Optionally support open shells (SHELL_BASED_SURFACE_MODEL) or compare per-shape in gapcheck.",
      });
    } else {
      analysis.push({
        cause: "Volume differs although every sampled surface matches and the mesh is closed — an internal shell/void is oriented wrong (adds instead of subtracts), which distance sampling cannot see.",
        fix: "Orient shells per connected component and verify inner shells point inward (containment test), instead of one global orientation.",
      });
    }
  }

  if (rec.status !== "OK") {
    let missingTris: Float64Array | null = null;
    if (missingOccFaces.length) {
      const missSet = new Set(missingOccFaces.map((f) => f.face));
      const idx: number[] = [];
      for (let t = 0; t < occ.soup.nTris && idx.length < 60000; t++) if (missSet.has(occ.soup.faceOfTri[t]!)) idx.push(t);
      missingTris = new Float64Array(idx.length * 9);
      idx.forEach((t, i) => missingTris!.set(occ!.soup.tris.subarray(t * 9, t * 9 + 9), i * 9));
    }
    const extraPts = worstOurFaces.filter((f) => f.bad > 0).flatMap((f) => f.at);
    emitImages(flaggedFaces, missingTris, extraPts);
  }
  return rec;
}

// standalone entry
if (process.argv[1] && import.meta.url.endsWith(basename(process.argv[1]))) {
  const path = process.argv[2];
  if (!path) { console.error("usage: node test/gapcheck-one.ts <file.step> [optsJson]"); process.exit(2); }
  const opts = { ...DEFAULT_OPTS, ...(process.argv[3] ? JSON.parse(process.argv[3]) : {}) };
  const rec = await gapcheckOne(path, opts);
  // sentinel lets the orchestrator find the JSON even if anything else leaked to stdout
  process.stdout.write("\n@@GAPCHECK@@" + JSON.stringify(rec, (_k, v) => (typeof v === "number" && !Number.isFinite(v) ? null : v)));
}
