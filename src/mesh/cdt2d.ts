// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — robust 2D constrained Delaunay triangulation (incremental Bowyer-Watson insertion
// with neighbour links, constraint forcing, and region extraction by constraint-parity flood
// fill). Unlike a batch hull triangulation this is robust on collinear boundaries (e.g. a
// cylinder rim, which projects to a straight line of points) and on dense inputs.

type P2 = [number, number];

const orient = (a: P2, b: P2, c: P2): number => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);

/** True if d is strictly inside the circumcircle of CCW triangle a,b,c. */
function inCircle(a: P2, b: P2, c: P2, d: P2): boolean {
  const ax = a[0] - d[0], ay = a[1] - d[1];
  const bx = b[0] - d[0], by = b[1] - d[1];
  const cx = c[0] - d[0], cy = c[1] - d[1];
  const det = (ax * ax + ay * ay) * (bx * cy - cx * by)
    - (bx * bx + by * by) * (ax * cy - cx * ay)
    + (cx * cx + cy * cy) * (ax * by - bx * ay);
  return det > 1e-12;
}

function segCross(p: P2, q: P2, r: P2, s: P2): boolean {
  const d1 = orient(r, s, p), d2 = orient(r, s, q), d3 = orient(p, q, r), d4 = orient(p, q, s);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

interface Tri { v: [number, number, number]; n: [number, number, number]; dead: boolean; }

const ckey = (a: number, b: number): number => (a < b ? a * 0x8000000 + b : b * 0x8000000 + a);

/** In triangle ti, set the neighbour across edge (x,y) to value. */
function setNeighbor(tris: Tri[], ti: number, x: number, y: number, value: number): void {
  if (ti < 0) return;
  const t = tris[ti]!;
  for (let e = 0; e < 3; e++) {
    const a = t.v[e]!, b = t.v[(e + 1) % 3]!;
    if ((a === x && b === y) || (a === y && b === x)) { t.n[e] = value; return; }
  }
}

/** Even-odd point-in-polygon test (ray casting) on a vertex ring. */
function pnpoly(px: number, py: number, poly: P2[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i]!, b = poly[j]!;
    if ((a[1] > py) !== (b[1] > py)) {
      const x = ((b[0] - a[0]) * (py - a[1])) / (b[1] - a[1]) + a[0];
      if (px < x) inside = !inside;
    }
  }
  return inside;
}

function pointInTri(p: P2, a: P2, b: P2, c: P2): boolean {
  const d1 = orient(a, b, p), d2 = orient(b, c, p), d3 = orient(c, a, p);
  return !((d1 < -1e-9 || d2 < -1e-9 || d3 < -1e-9) && (d1 > 1e-9 || d2 > 1e-9 || d3 > 1e-9));
}

function locate(tris: Tri[], pts: P2[], pi: number, hint: number): number {
  const p = pts[pi]!;
  let t = !tris[hint] || tris[hint]!.dead ? tris.findIndex((x) => !x.dead) : hint;
  for (let steps = 0; steps < tris.length * 3 + 8; steps++) {
    const tri = tris[t]!;
    let moved = false;
    for (let e = 0; e < 3; e++) {
      if (orient(pts[tri.v[e]!]!, pts[tri.v[(e + 1) % 3]!]!, p) < -1e-12) {
        const nb = tri.n[e]!;
        if (nb >= 0 && !tris[nb]!.dead) { t = nb; moved = true; break; }
      }
    }
    if (!moved) return t;
  }
  for (let i = 0; i < tris.length; i++) {
    const tri = tris[i]!;
    if (!tri.dead && pointInTri(p, pts[tri.v[0]!]!, pts[tri.v[1]!]!, pts[tri.v[2]!]!)) return i;
  }
  return tris.findIndex((x) => !x.dead);
}

function insertPoint(tris: Tri[], pts: P2[], pi: number, hint: number, free: number[]): number {
  const start = locate(tris, pts, pi, hint);
  // Collect the cavity: triangles whose circumcircle contains p, grown across shared edges.
  const bad: number[] = [];
  const seen = new Set<number>([start]);
  const stack = [start];
  while (stack.length) {
    const t = stack.pop()!;
    bad.push(t);
    const tri = tris[t]!;
    for (let e = 0; e < 3; e++) {
      const nb = tri.n[e]!;
      if (nb < 0 || seen.has(nb)) continue;
      const nt = tris[nb]!;
      if (inCircle(pts[nt.v[0]!]!, pts[nt.v[1]!]!, pts[nt.v[2]!]!, pts[pi]!)) { seen.add(nb); stack.push(nb); }
    }
  }
  // Cavity boundary edges (oriented so the cavity is to the left).
  const bset = new Set(bad);
  const edges: { a: number; b: number; ext: number }[] = [];
  for (const t of bad) {
    const tri = tris[t]!;
    for (let e = 0; e < 3; e++) {
      const nb = tri.n[e]!;
      if (nb >= 0 && bset.has(nb)) continue;
      edges.push({ a: tri.v[e]!, b: tri.v[(e + 1) % 3]!, ext: nb });
    }
  }
  // Recycle cavity slots; build a fan of new triangles (pi,a,b).
  for (const t of bad) { tris[t]!.dead = true; free.push(t); }
  const startMap = new Map<number, number>(), endMap = new Map<number, number>();
  const made: number[] = [];
  for (const { a, b, ext } of edges) {
    const tri: Tri = { v: [pi, a, b], n: [-1, ext, -1], dead: false };
    let idx: number;
    if (free.length) { idx = free.pop()!; tris[idx] = tri; } else { idx = tris.length; tris.push(tri); }
    setNeighbor(tris, ext, a, b, idx);
    startMap.set(a, idx); endMap.set(b, idx);
    made.push(idx);
  }
  for (const idx of made) {
    const t = tris[idx]!;
    t.n[0] = endMap.get(t.v[1]!) ?? -1;   // edge (pi,a) shared with the tri ending at a
    t.n[2] = startMap.get(t.v[2]!) ?? -1; // edge (b,pi) shared with the tri starting at b
  }
  return made[0] ?? start;
}

const edgeExists = (tris: Tri[], v2t: number[][], a: number, b: number): boolean =>
  (v2t[a] ?? []).some((ti) => { const t = tris[ti]; return t && !t.dead && (t.v[0] === b || t.v[1] === b || t.v[2] === b); });

function flipEdge(tris: Tri[], pts: P2[], ti: number, e: number): boolean {
  const t = tris[ti]!;
  const tj = t.n[e]!;
  if (tj < 0) return false;
  const a = t.v[e]!, b = t.v[(e + 1) % 3]!, c = t.v[(e + 2) % 3]!;
  const nt = tris[tj]!;
  let j = -1;
  for (let k = 0; k < 3; k++) if (nt.v[k] === b && nt.v[(k + 1) % 3] === a) { j = k; break; }
  if (j < 0) return false;
  const d = nt.v[(j + 2) % 3]!;
  if (orient(pts[c]!, pts[a]!, pts[d]!) <= 0 || orient(pts[c]!, pts[d]!, pts[b]!) <= 0) return false; // not convex
  const nbc = t.n[(e + 1) % 3]!, nca = t.n[(e + 2) % 3]!, nad = nt.n[(j + 1) % 3]!, ndb = nt.n[(j + 2) % 3]!;
  tris[ti] = { v: [c, a, d], n: [nca, nad, tj], dead: false };
  tris[tj] = { v: [c, d, b], n: [ti, ndb, nbc], dead: false };
  setNeighbor(tris, nad, a, d, ti);
  setNeighbor(tris, nbc, b, c, tj);
  return true;
}

function forceEdge(tris: Tri[], pts: P2[], v2t: number[][], a: number, b: number): void {
  let guard = 0;
  while (!edgeExists(tris, v2t, a, b) && guard++ < 500) {
    let flipped = false;
    for (let ti = 0; ti < tris.length && !flipped; ti++) {
      const t = tris[ti]!;
      if (t.dead) continue;
      for (let e = 0; e < 3; e++) {
        const u = t.v[e]!, w = t.v[(e + 1) % 3]!;
        if (u === a || u === b || w === a || w === b) continue;
        if (segCross(pts[a]!, pts[b]!, pts[u]!, pts[w]!) && flipEdge(tris, pts, ti, e)) {
          // refresh incidence for the two changed triangles
          for (const tt of [ti, t.n[e]!]) if (tt >= 0) for (const vv of tris[tt]!.v) (v2t[vv] ??= []).push(tt);
          flipped = true;
          break;
        }
      }
    }
    if (!flipped) break;
  }
}

/** Ear-clip a simple polygon (vertex-index ring) into triangles, appended to `out`. */
function earClip(pts: P2[], ring: number[], out: [number, number, number][]): boolean {
  const idx = ring.slice();
  if (idx.length < 3) return true;
  let area = 0;
  for (let i = 0; i < idx.length; i++) { const p = pts[idx[i]!]!, q = pts[idx[(i + 1) % idx.length]!]!; area += p[0] * q[1] - q[0] * p[1]; }
  if (area < 0) idx.reverse();
  let guard = 0;
  while (idx.length > 3 && guard++ < 100000) {
    let clipped = false;
    for (let i = 0; i < idx.length; i++) {
      const a = idx[(i + idx.length - 1) % idx.length]!, b = idx[i]!, c = idx[(i + 1) % idx.length]!;
      if (orient(pts[a]!, pts[b]!, pts[c]!) <= 0) continue; // reflex or degenerate — not an ear
      let ok = true;
      for (let j = 0; j < idx.length; j++) {
        const p = idx[j]!;
        if (p === a || p === b || p === c) continue;
        if (pointInTri(pts[p]!, pts[a]!, pts[b]!, pts[c]!)) { ok = false; break; }
      }
      if (!ok) continue;
      out.push([a, b, c]); idx.splice(i, 1); clipped = true; break;
    }
    if (!clipped) return false; // not a simple polygon — bail
  }
  if (idx.length === 3) out.push([idx[0]!, idx[1]!, idx[2]!]);
  return true;
}

/**
 * Watertight fallback when the CDT can't realise a face's boundary constraints: ear-clip the outer
 * ring (every boundary edge guaranteed present => watertight with the neighbour), THEN re-insert the
 * interior points so the patch still follows the surface instead of spanning flat chords across a
 * curved boundary. Each interior point splits its containing triangle into three — the original three
 * edges survive as triangle edges, so neighbours stay matched (no T-junctions for strictly-interior
 * points; points that miss every triangle, e.g. just outside the trimmed region, are skipped).
 * Returns null if the ring isn't a clean simple polygon (caller keeps its other fallback).
 */
function boundaryFillWithInterior(pts: P2[], ring: number[], interior: number[]): [number, number, number][] | null {
  const out: [number, number, number][] = [];
  if (!earClip(pts, ring, out) || out.length === 0) return null;
  for (const pi of interior) {
    const p = pts[pi]!;
    // Split only the triangle that STRICTLY contains pi (a clear margin from every edge). A point on
    // (or hugging) a shared edge would split one side only and leave a T-junction = non-manifold; such
    // points are simply skipped, costing a little interior density, never watertightness.
    let found = -1;
    for (let k = 0; k < out.length; k++) {
      const [a, b, c] = out[k]!, A = pts[a]!, B = pts[b]!, C = pts[c]!;
      const m = 1e-3 * Math.abs(orient(A, B, C));
      if (orient(A, B, p) > m && orient(B, C, p) > m && orient(C, A, p) > m) { found = k; break; }
    }
    if (found < 0) continue;
    const [a, b, c] = out[found]!;
    out[found] = [a, b, pi];
    out.push([b, c, pi], [c, a, pi]);
  }
  return out;
}

/**
 * Fan-triangulate a simple polygon from a single apex vertex (every triangle is apex-edge_i). Works
 * — where ear-clipping fails — on a polygon that is geometrically a triangle with COLLINEAR points
 * along one side: a ruled/curved slice meeting at a singular vertex (a cone apex or a sphere pole)
 * plus a curved rim whose samples project collinear. Ear-clip can only clip the apex ear, leaving a
 * degenerate collinear remainder; a fan from the apex makes every rim segment a triangle edge (hence
 * watertight with the neighbour). Tries each vertex as the star centre and accepts the first whose
 * fan is a valid, non-overlapping, area-exact tiling.
 */
function fanFill(pts: P2[], ring: number[]): [number, number, number][] | null {
  const n = ring.length;
  if (n < 3) return null;
  let polyArea = 0;
  for (let i = 0; i < n; i++) { const p = pts[ring[i]!]!, q = pts[ring[(i + 1) % n]!]!; polyArea += p[0] * q[1] - q[0] * p[1]; }
  polyArea = Math.abs(polyArea) / 2;
  if (polyArea < 1e-12) return null;
  for (let c = 0; c < n; c++) {
    const out: [number, number, number][] = [];
    let sumA = 0, ok = true;
    for (let i = 0; i < n && ok; i++) {
      const a = ring[i]!, b = ring[(i + 1) % n]!;
      if (a === ring[c] || b === ring[c]) continue; // edge incident to the apex
      const A = pts[ring[c]!]!, B = pts[a]!, C = pts[b]!;
      const ar = orient(A, B, C);
      if (ar <= 1e-12) { ok = false; break; } // reflex/degenerate from this centre -> not the kernel
      for (let j = 0; j < n; j++) {
        const vj = ring[j]!;
        if (vj === ring[c]! || vj === a || vj === b) continue;
        if (pointInTri(pts[vj]!, A, B, C)) { ok = false; break; }
      }
      sumA += ar / 2;
      out.push([ring[c]!, a, b]);
    }
    if (ok && Math.abs(sumA - polyArea) < 1e-6 * polyArea) return out;
  }
  return null;
}

/**
 * Robustly enforce constraint edge a-b when flips couldn't: delete every triangle the segment
 * crosses, then ear-clip the two simple sub-polygons that the segment splits the cavity into.
 * Returns false (leaving the triangulation unchanged) if the cavity isn't a clean single loop.
 */
function enforceByRetriangulation(tris: Tri[], pts: P2[], free: number[], a: number, b: number): boolean {
  const PA = pts[a]!, PB = pts[b]!;
  const crossed: number[] = [];
  for (let t = 0; t < tris.length; t++) {
    const T = tris[t]!; if (T.dead) continue;
    const v = T.v;
    for (let e = 0; e < 3; e++) {
      const u = v[e]!, w = v[(e + 1) % 3]!;
      if (u === a || u === b || w === a || w === b) continue;
      if (segCross(PA, PB, pts[u]!, pts[w]!)) { crossed.push(t); break; }
    }
  }
  if (!crossed.length || crossed.length > 64) return false; // huge/empty cavity = degenerate seam — leave it
  // Boundary edges of the cavity = edges of crossed triangles not shared by two crossed triangles.
  const count = new Map<number, number>();
  for (const t of crossed) { const v = tris[t]!.v; for (let e = 0; e < 3; e++) count.set(ckey(v[e]!, v[(e + 1) % 3]!), (count.get(ckey(v[e]!, v[(e + 1) % 3]!)) ?? 0) + 1); }
  const nextOf = new Map<number, number>();
  let edges = 0;
  for (const t of crossed) {
    const v = tris[t]!.v;
    for (let e = 0; e < 3; e++) {
      const u = v[e]!, w = v[(e + 1) % 3]!;
      if ((count.get(ckey(u, w)) ?? 0) === 1) { if (nextOf.has(u)) return false; nextOf.set(u, w); edges++; }
    }
  }
  if (!nextOf.has(a) || !nextOf.has(b)) return false;
  const loop: number[] = [];
  let cur = a;
  for (let g = 0; g <= edges; g++) { loop.push(cur); const nx = nextOf.get(cur); if (nx === undefined) return false; cur = nx; if (cur === a) break; }
  if (cur !== a || loop.length !== edges) return false; // not one simple loop
  const ia = loop.indexOf(a), ib = loop.indexOf(b);
  const path1: number[] = [], path2: number[] = [];
  for (let i = ia; ; i = (i + 1) % loop.length) { path1.push(loop[i]!); if (i === ib) break; }
  for (let i = ib; ; i = (i + 1) % loop.length) { path2.push(loop[i]!); if (i === ia) break; }
  const newTris: [number, number, number][] = [];
  if (!earClip(pts, path1, newTris) || !earClip(pts, path2, newTris)) return false;
  // Area-conservation guard: the fill must tile exactly the deleted cavity. If areas disagree the
  // cavity loop was self-folded (a degenerate periodic seam) — abort rather than corrupt the mesh.
  const triArea = (x: number, y: number, z: number): number => Math.abs(orient(pts[x]!, pts[y]!, pts[z]!));
  let oldA = 0; for (const t of crossed) { const v = tris[t]!.v; oldA += triArea(v[0]!, v[1]!, v[2]!); }
  let newA = 0; for (const [x, y, z] of newTris) newA += triArea(x, y, z);
  if (Math.abs(newA - oldA) > 1e-3 * (oldA + 1e-12)) return false;
  for (const t of crossed) { tris[t]!.dead = true; free.push(t); }
  for (const [x, y, z] of newTris) {
    const tri: Tri = { v: [x, y, z], n: [-1, -1, -1], dead: false };
    if (free.length) { const idx = free.pop()!; tris[idx] = tri; } else tris.push(tri);
  }
  return true;
}

/**
 * Triangulate the region bounded by loops[0] (outer) minus loops[1..] (holes), using all listed
 * points (loop vertices + interior). Returns triangle index triples into `points`.
 */
export function constrainedTriangulate(points: P2[], loops: number[][], interior: number[]): [number, number, number][] {
  const n = points.length;
  if (n < 3) return [];
  let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
  for (const p of points) {
    if (p[0] < minx) minx = p[0]; if (p[1] < miny) miny = p[1];
    if (p[0] > maxx) maxx = p[0]; if (p[1] > maxy) maxy = p[1];
  }
  const dmax = Math.max(maxx - minx, maxy - miny) || 1;
  const cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
  const pts = points.slice();
  // Super-triangle in CCW order (all predicates assume CCW).
  const s0 = pts.length; pts.push([cx - 3 * dmax, cy - dmax]); // bottom-left
  const s1 = pts.length; pts.push([cx + 3 * dmax, cy - dmax]); // bottom-right
  const s2 = pts.length; pts.push([cx, cy + 3 * dmax]);        // top

  const tris: Tri[] = [{ v: [s0, s1, s2], n: [-1, -1, -1], dead: false }];
  const free: number[] = [];
  let hint = 0;
  // Insert boundary loop points first, then interior points.
  for (const loop of loops) for (const pi of loop) hint = insertPoint(tris, pts, pi, hint, free);
  for (const pi of interior) hint = insertPoint(tris, pts, pi, hint, free);

  // Force the constraint (boundary) edges.
  const v2t: number[][] = Array.from({ length: pts.length }, () => []);
  for (let i = 0; i < tris.length; i++) if (!tris[i]!.dead) for (const v of tris[i]!.v) v2t[v]!.push(i);
  const constraints = new Set<number>();
  for (const loop of loops) for (let i = 0; i < loop.length; i++) {
    const a = loop[i]!, b = loop[(i + 1) % loop.length]!;
    constraints.add(ckey(a, b));
    forceEdge(tris, pts, v2t, a, b);
  }
  // Second pass: any constraint the flip method left unrealised (stuck on non-convex quads) is
  // enforced by deleting the crossed triangles and ear-clipping the cavity — guaranteed to succeed.
  const present = new Set<number>();
  for (let t = 0; t < tris.length; t++) { const T = tris[t]!; if (T.dead) continue; for (let e = 0; e < 3; e++) present.add(ckey(T.v[e]!, T.v[(e + 1) % 3]!)); }
  for (const ck of constraints) {
    if (present.has(ck)) continue;
    const a = Math.floor(ck / 0x8000000), b = ck % 0x8000000;
    enforceByRetriangulation(tris, pts, free, a, b);
  }

  // Region extraction: flood fill from a super-triangle (outside), toggling in/out at constraints.
  // Build edge adjacency FRESH from the live triangles — the per-triangle `n` links get mangled by
  // constraint flipping, and trusting them leaves unreachable interior triangles (holes).
  const edgeTris = new Map<number, number[]>();
  for (let i = 0; i < tris.length; i++) {
    const t = tris[i]!;
    if (t.dead) continue;
    for (let e = 0; e < 3; e++) {
      const k = ckey(t.v[e]!, t.v[(e + 1) % 3]!);
      const a = edgeTris.get(k); if (a) a.push(i); else edgeTris.set(k, [i]);
    }
  }
  const inside = new Map<number, boolean>();
  const startT = tris.findIndex((t) => !t.dead && (t.v[0] >= n || t.v[1] >= n || t.v[2] >= n));
  if (startT < 0) return [];
  const q = [startT]; inside.set(startT, false);
  while (q.length) {
    const t = q.shift()!;
    const st = inside.get(t)!;
    const tri = tris[t]!;
    for (let e = 0; e < 3; e++) {
      const isC = constraints.has(ckey(tri.v[e]!, tri.v[(e + 1) % 3]!));
      for (const nb of edgeTris.get(ckey(tri.v[e]!, tri.v[(e + 1) % 3]!)) ?? []) {
        if (nb === t || inside.has(nb)) continue;
        inside.set(nb, isC ? !st : st);
        q.push(nb);
      }
    }
  }

  const flood: [number, number, number][] = [];
  for (let i = 0; i < tris.length; i++) {
    const t = tris[i]!;
    if (t.dead || t.v[0] >= n || t.v[1] >= n || t.v[2] >= n) continue;
    if (inside.get(i) === true) flood.push([t.v[0], t.v[1], t.v[2]]);
  }

  // Count boundary constraints the CDT couldn't realise. When every constraint is present the parity
  // flood fill is exact, so we keep it untouched (the common case for every well-parametrised face).
  let missing = 0;
  for (const loop of loops) for (let i = 0; i < loop.length; i++) {
    if (!edgeTris.has(ckey(loop[i]!, loop[(i + 1) % loop.length]!))) missing++;
  }
  if (missing === 0) return flood;

  // Constraints unrealised — the (u,v) embedding collapsed (a thin-sliver B-spline parameter domain
  // where distinct 3D points coincide / fall collinear, so the CDT triangulated across the boundary
  // and shattered the face internally). For a single loop with no holes, discard the CDT result and
  // re-triangulate from the boundary ring directly (ear-clip + interior re-insertion): this
  // guarantees every boundary edge is a triangle edge — hence watertight with the neighbour, which
  // shares those exact samples — while still following the surface through the interior points.
  // Falls through to the geometric classification if the ring isn't a clean simple polygon.
  if (loops.length === 1 && (loops[0]?.length ?? 0) >= 3) {
    const filled = boundaryFillWithInterior(points, loops[0]!, interior);
    if (filled && filled.length > 0) return filled;
    // Ear-clip failed — the loop is a triangle with a collinear side (a curved slice meeting at a
    // cone apex / sphere pole). Fan from the apex, making every rim segment a triangle edge.
    const fan = fanFill(points, loops[0]!);
    if (fan && fan.length > 0) return fan;
  }

  // Some seam edge is unrealisable — a metric-collapsed boundary on a skewed B-spline patch. The
  // flood fill leaks through that gap and flips a whole connected region to "outside", opening half
  // the face's seam (one unenforced edge => hundreds of open edges). A GEOMETRIC classification
  // (centroid inside outer, outside holes) can't be leaked: a miss costs at most one triangle. But on
  // a boundary that genuinely self-intersects in scaled (u,v) — a closed-v B-spline patch whose seam
  // unwrap tangled the loop — there is NO non-overlapping triangulation of that polygon: keeping
  // either the flood or the geometric fill leaves a knot of double-covered (non-manifold) triangles.
  // That chaos reads worse than a clean hole, so emit NOTHING and let the face be a small gap.
  const outerLoop = loops[0] ?? [];
  let selfCross = false;
  for (let i = 0; i < outerLoop.length && !selfCross; i++) {
    const a0 = points[outerLoop[i]!]!, a1 = points[outerLoop[(i + 1) % outerLoop.length]!]!;
    for (let j = i + 2; j < outerLoop.length; j++) {
      if (i === 0 && j === outerLoop.length - 1) continue; // shares the closing vertex
      const b0 = points[outerLoop[j]!]!, b1 = points[outerLoop[(j + 1) % outerLoop.length]!]!;
      if (segCross(a0, a1, b0, b1)) { selfCross = true; break; }
    }
  }
  if (selfCross) {
    // If the flood fill itself is manifold, keep it — a benign self-cross. Only when it double-covers
    // (an edge shared by >2 triangles, i.e. an unavoidable knot on a tangled closed-v seam) is the
    // chaos worse than a clean hole, so emit nothing for those.
    const fe = new Map<number, number>();
    for (const [a, b, cc] of flood) for (const [x, y] of [[a, b], [b, cc], [cc, a]] as const) fe.set(ckey(x, y), (fe.get(ckey(x, y)) ?? 0) + 1);
    let floodNm = 0; for (const v of fe.values()) if (v > 2) floodNm++;
    return floodNm > 0 ? [] : flood;
  }

  const outerPoly = outerLoop.map((i) => points[i]!);
  const holePolys = loops.slice(1).map((l) => l.map((i) => points[i]!));
  const geom: [number, number, number][] = [];
  for (let i = 0; i < tris.length; i++) {
    const t = tris[i]!;
    if (t.dead || t.v[0] >= n || t.v[1] >= n || t.v[2] >= n) continue;
    const cxp = (points[t.v[0]]![0] + points[t.v[1]]![0] + points[t.v[2]]![0]) / 3;
    const cyp = (points[t.v[0]]![1] + points[t.v[1]]![1] + points[t.v[2]]![1]) / 3;
    if (!pnpoly(cxp, cyp, outerPoly)) continue;
    if (holePolys.some((h) => pnpoly(cxp, cyp, h))) continue;
    geom.push([t.v[0], t.v[1], t.v[2]]);
  }
  return geom;
}
