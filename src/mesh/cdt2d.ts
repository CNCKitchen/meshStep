// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — robust 2D constrained Delaunay triangulation (incremental Bowyer-Watson insertion
// with neighbour links, constraint forcing, and region extraction by constraint-parity flood
// fill). Unlike a batch hull triangulation this is robust on collinear boundaries (e.g. a
// cylinder rim, which projects to a straight line of points) and on dense inputs.

type P2 = [number, number];

/** Diagnostics for the gapcheck harness (MESHSTEP_DEBUG=1); no-op in production/browser. */
const DBG = typeof process !== "undefined" && !!process.env?.MESHSTEP_DEBUG;

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
  const tryFlip = (ti: number, e: number): boolean => {
    const t = tris[ti]!;
    if (!flipEdge(tris, pts, ti, e)) return false;
    // refresh incidence for the two changed triangles
    for (const tt of [ti, t.n[e]!]) if (tt >= 0) for (const vv of tris[tt]!.v) (v2t[vv] ??= []).push(tt);
    return true;
  };
  /** Walk the corridor of triangles the segment a-b crosses (via neighbour links, valid during the
   * flip pass) and flip the first flippable crossing edge. O(corridor) instead of O(all triangles).
   * Returns false when the walk can't proceed (segment through a vertex, broken link) — caller
   * falls back to the exhaustive scan for that iteration. */
  const corridorFlip = (): boolean | null => {
    for (const ti of v2t[a] ?? []) {
      const t = tris[ti];
      if (!t || t.dead) continue;
      const k = t.v.indexOf(a);
      if (k < 0) continue;
      const e = (k + 1) % 3; // edge opposite a
      const u = t.v[e]!, w = t.v[(e + 1) % 3]!;
      if (u === b || w === b) continue;
      if (!segCross(pts[a]!, pts[b]!, pts[u]!, pts[w]!)) continue;
      // found the corridor entrance; walk it, trying to flip each crossing edge
      let cur = ti, edge = e;
      for (let step = 0; step < 2000; step++) {
        if (tryFlip(cur, edge)) return true;
        const nx = tris[cur]!.n[edge]!;
        if (nx < 0) return null;
        const nt = tris[nx]!;
        if (nt.dead) return null;
        // entry edge in nx is (w2,u2) reversed; segment exits through one of the other two edges
        let advanced = false;
        for (let e2 = 0; e2 < 3; e2++) {
          const u2 = nt.v[e2]!, w2 = nt.v[(e2 + 1) % 3]!;
          if (u2 === a || u2 === b || w2 === a || w2 === b) continue;
          const pk = ckey(u2, w2);
          if (pk === ckey(tris[cur]!.v[edge]!, tris[cur]!.v[(edge + 1) % 3]!)) continue; // entry edge
          if (segCross(pts[a]!, pts[b]!, pts[u2]!, pts[w2]!)) { cur = nx; edge = e2; advanced = true; break; }
        }
        if (!advanced) return false; // corridor ends (reached b's fan) with nothing flippable
      }
      return null;
    }
    return null; // no crossing edge incident to a (collinear pass-through) — needs the full scan
  };
  let guard = 0;
  while (!edgeExists(tris, v2t, a, b) && guard++ < 500) {
    const cf = corridorFlip();
    if (cf === true) continue;
    // corridor blocked or unwalkable: exhaustive scan — the old behaviour. Flipping a crossing
    // edge anywhere (even outside the walked corridor) can unblock a non-convex quad, so the
    // corridor is strictly an accelerator, never a reason to give up earlier than the scan did.
    let flipped = false;
    for (let ti = 0; ti < tris.length && !flipped; ti++) {
      const t = tris[ti]!;
      if (t.dead) continue;
      for (let e = 0; e < 3; e++) {
        const u = t.v[e]!, w = t.v[(e + 1) % 3]!;
        if (u === a || u === b || w === a || w === b) continue;
        if (segCross(pts[a]!, pts[b]!, pts[u]!, pts[w]!) && tryFlip(ti, e)) { flipped = true; break; }
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
 * Excise MICRO self-intersections from a boundary ring: projection noise where two rails of a
 * slot/thread converge (run-outs) makes the ring cross itself in tiny zero-area loops, which makes
 * every watertight fill refuse the ring — and a 15k-point dome then vanishes over a dozen bad
 * points. A crossing whose two segments are ≤64 ring positions apart bounds a micro-loop: drop the
 * short arc between them (the neighbour keeps those samples — a few T-junction points at the
 * run-out, instead of the whole face missing). Genuine large-scale tangles (crossings far apart on
 * the ring) are NOT repairable this way — return null so the caller keeps its other fallbacks.
 */
function dropMicroLoops(pts: P2[], ring: number[]): number[] | null {
  let cur = ring.slice();
  const maxDrop = Math.max(16, Math.floor(ring.length * 0.1));
  let dropped = 0;
  const distPS = (p: P2, a: P2, b: P2): number => {
    const ex = b[0] - a[0], ey = b[1] - a[1];
    const l2 = ex * ex + ey * ey;
    let t = l2 > 0 ? ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / l2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return Math.hypot(p[0] - (a[0] + t * ex), p[1] - (a[1] + t * ey));
  };
  for (let pass = 0; pass < 32; pass++) {
    const n = cur.length;
    if (n < 4) return null;
    // spatial hash of segments (short segments -> few cells each)
    let sum = 0;
    for (let i = 0; i < n; i++) {
      const a = pts[cur[i]!]!, b = pts[cur[(i + 1) % n]!]!;
      sum += Math.hypot(b[0] - a[0], b[1] - a[1]);
    }
    const avgSeg = Math.max(sum / n, 1e-12);
    const touchTol = 0.25 * avgSeg; // rails closer than a quarter sample-step = degenerate contact
    const cell = avgSeg * 2;
    const hk = (ix: number, iy: number): number => Math.imul(ix, 73856093) ^ Math.imul(iy, 19349663);
    const grid = new Map<number, number[]>();
    for (let i = 0; i < n; i++) {
      const a = pts[cur[i]!]!, b = pts[cur[(i + 1) % n]!]!;
      const x0 = Math.floor((Math.min(a[0], b[0]) - touchTol) / cell), x1 = Math.floor((Math.max(a[0], b[0]) + touchTol) / cell);
      const y0 = Math.floor((Math.min(a[1], b[1]) - touchTol) / cell), y1 = Math.floor((Math.max(a[1], b[1]) + touchTol) / cell);
      for (let x = x0; x <= x1; x++) for (let y = y0; y <= y1; y++) {
        const k = hk(x, y);
        const arr = grid.get(k); if (arr) arr.push(i); else grid.set(k, [i]);
      }
    }
    // Collect degenerate contacts: proper segment crossings AND rail segments passing within
    // touchTol of each other (interleaved zero-width corridors block every ear without crossing).
    const pairs: [number, number, number][] = []; // [dcyc, i, j]
    for (const arr of grid.values()) {
      for (let ai = 0; ai < arr.length; ai++) for (let bi = ai + 1; bi < arr.length; bi++) {
        let i = arr[ai]!, j = arr[bi]!;
        if (i > j) { const t = i; i = j; j = t; }
        const dIdx = Math.min(j - i, n - (j - i));
        if (dIdx <= 1) continue; // adjacent segments legitimately touch
        const a0 = pts[cur[i]!]!, a1 = pts[cur[(i + 1) % n]!]!;
        const b0 = pts[cur[j]!]!, b1 = pts[cur[(j + 1) % n]!]!;
        const touch = segCross(a0, a1, b0, b1)
          || distPS(b0, a0, a1) < touchTol || distPS(b1, a0, a1) < touchTol
          || distPS(a0, b0, b1) < touchTol || distPS(a1, b0, b1) < touchTol;
        if (touch) pairs.push([dIdx, i, j]);
      }
    }
    if (!pairs.length) {
      if (DBG) console.error(`[cdt]   dropMicroLoops: clean after ${pass} passes, dropped=${dropped}/${ring.length}`);
      return dropped > 0 ? cur : null; // simple now (or was never degenerate)
    }
    // Excise every non-overlapping small-span contact this pass (the short arc between the two
    // segments = the micro-loop / zero-width tail). A long interleaved run-out is hundreds of
    // touch points, so one-at-a-time never converges.
    pairs.sort((a, b) => a[0] - b[0]);
    const drop = new Uint8Array(n);
    let any = false, tangle = false;
    for (const [d, i, j] of pairs) {
      if (d > 96) { tangle = tangle || !any; break; } // spans this large are a genuine tangle
      if (j - i !== d) continue; // wrap-around short side: rare, let a later pass handle it solo
      if (dropped + d > maxDrop) break;
      let clear = drop[i] === 0 && drop[(j + 1) % n] === 0;
      for (let k = i + 1; clear && k <= j; k++) clear = drop[k] === 0;
      if (!clear) continue;
      for (let k = i + 1; k <= j; k++) drop[k] = 1;
      dropped += d;
      any = true;
    }
    if (!any) {
      // nothing excisable: either all contacts are huge (tangle) or budget is spent
      if (DBG) console.error(`[cdt]   dropMicroLoops: ${tangle ? "large-span contact — genuine tangle" : `budget spent (dropped=${dropped})`}, contacts=${pairs.length}`);
      return null;
    }
    cur = cur.filter((_, k) => drop[k] === 0);
  }
  if (DBG) console.error(`[cdt]   dropMicroLoops: pass budget exhausted, dropped=${dropped}`);
  return null;
}

/**
 * Robustly enforce constraint edge a-b when flips couldn't: delete every triangle the segment
 * crosses, then ear-clip the two simple sub-polygons that the segment splits the cavity into.
 * Returns false (leaving the triangulation unchanged) if the cavity isn't a clean single loop.
 */
function enforceByRetriangulation(tris: Tri[], pts: P2[], free: number[], a: number, b: number, constraints: Set<number>): boolean {
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
  if (!crossed.length || crossed.length > 64) {
    if (DBG) console.error(`[cdt]     enforce ${a}-${b}: bail crossed=${crossed.length}${process.env.MESHSTEP_SLICEDBG ? ` A=(${PA[0]},${PA[1]}) B=(${PB[0]},${PB[1]})` : ""}`);
    return false; // huge/empty cavity = degenerate seam — leave it
  }
  // Boundary edges of the cavity = edges of crossed triangles not shared by two crossed triangles.
  const count = new Map<number, number>();
  for (const t of crossed) { const v = tris[t]!.v; for (let e = 0; e < 3; e++) count.set(ckey(v[e]!, v[(e + 1) % 3]!), (count.get(ckey(v[e]!, v[(e + 1) % 3]!)) ?? 0) + 1); }
  // CONSTRAINT PROTECTION: an edge interior to the cavity (shared by two crossed triangles) is
  // deleted with them, and a free ear-clip fill has no obligation to recreate it. When the segment
  // squeezes past a shared boundary vertex, such an interior edge can be a NEIGHBOURING constraint
  // the flip pass already realised — enforcing a-b would silently destroy it (the wio-front/letters
  // open-edge class; both a fixpoint re-enforcement and a chain-split repair of the aftermath
  // REGRESSED — see the note above constrainedTriangulate's second pass). Enforce the invariant at
  // the source: such an edge becomes a MANDATORY DIAGONAL of the fill (the cavity polygon is split
  // at it and each side clipped separately), so a-b and the neighbour are BOTH realised. It cannot
  // cross a-b (an interior edge crossed by the segment is a genuine constraint-vs-constraint
  // conflict) — there, refuse and leave a-b to the face-level rescue.
  const chords: [number, number][] = [];
  for (const [k, c] of count) {
    if (c === 2 && constraints.has(k)) {
      const u = Math.floor(k / 0x8000000), w = k % 0x8000000;
      if (segCross(PA, PB, pts[u]!, pts[w]!)) {
        if (DBG) console.error(`[cdt]     enforce ${a}-${b}: bail crosses realised constraint ${u}-${w}`);
        return false;
      }
      chords.push([u, w]);
    }
  }
  const nextOf = new Map<number, number>();
  let edges = 0;
  for (const t of crossed) {
    const v = tris[t]!.v;
    for (let e = 0; e < 3; e++) {
      const u = v[e]!, w = v[(e + 1) % 3]!;
      if ((count.get(ckey(u, w)) ?? 0) === 1) {
        if (nextOf.has(u)) { if (DBG) console.error(`[cdt]     enforce ${a}-${b}: bail cavity-fork at v${u}`); return false; }
        nextOf.set(u, w); edges++;
      }
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
  // Clip each side with the protected constraints as mandatory diagonals: split the polygon at the
  // chord and clip the two sub-polygons, so the chord edge is guaranteed present in the fill. A
  // chord not in this path (endpoints on the other side of a-b) is simply skipped by the index test.
  const clipWithChords = (path: number[], rem: [number, number][]): boolean => {
    for (let ci = 0; ci < rem.length; ci++) {
      const [u, w] = rem[ci]!;
      const iu = path.indexOf(u), iw = path.indexOf(w);
      if (iu < 0 || iw < 0) continue;
      const [i, j] = iu < iw ? [iu, iw] : [iw, iu];
      if (j - i === 1 || (i === 0 && j === path.length - 1)) continue; // already a polygon side
      const rest = rem.slice(0, ci).concat(rem.slice(ci + 1));
      return clipWithChords(path.slice(i, j + 1), rest)
        && clipWithChords([...path.slice(j), ...path.slice(0, i + 1)], rest);
    }
    return earClip(pts, path, newTris);
  };
  if (!clipWithChords(path1, chords) || !clipWithChords(path2, chords)) return false;
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
export function constrainedTriangulate(points: P2[], loops: number[][], interior: number[], out?: { missing: number }): [number, number, number][] {
  const n = points.length;
  if (out) out.missing = 0;
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
  const dbgT0 = DBG ? Date.now() : 0;
  // Insert boundary loop points first, then interior points.
  for (const loop of loops) for (const pi of loop) hint = insertPoint(tris, pts, pi, hint, free);
  for (const pi of interior) hint = insertPoint(tris, pts, pi, hint, free);
  const dbgT1 = DBG ? Date.now() : 0;

  // Force the constraint (boundary) edges.
  const v2t: number[][] = Array.from({ length: pts.length }, () => []);
  for (let i = 0; i < tris.length; i++) if (!tris[i]!.dead) for (const v of tris[i]!.v) v2t[v]!.push(i);
  const constraints = new Set<number>();
  for (const loop of loops) for (let i = 0; i < loop.length; i++) {
    const a = loop[i]!, b = loop[(i + 1) % loop.length]!;
    constraints.add(ckey(a, b));
    forceEdge(tris, pts, v2t, a, b);
  }
  const dbgT2 = DBG ? Date.now() : 0;
  // Second pass: any constraint the flip method left unrealised (stuck on non-convex quads) is
  // enforced by deleting the crossed triangles and ear-clipping the cavity. (A fixpoint iteration
  // of this pass was tried and REGRESSED: re-forcing hopeless constraints churns the triangulation
  // and trips fold audits on other faces — one pass, like it always was.)
  const present = new Set<number>();
  for (let t = 0; t < tris.length; t++) { const T = tris[t]!; if (T.dead) continue; for (let e = 0; e < 3; e++) present.add(ckey(T.v[e]!, T.v[(e + 1) % 3]!)); }
  // COLLINEAR PASS-THROUGH: a vertex sitting exactly ON a constraint blocks it — no edge can cross
  // a vertex, so the flip pass finds nothing to flip (crossed=0) and the cavity rescue has no
  // cavity. When the triangulation already contains the full chain of edges a→X…→b along the
  // segment, the constraint IS realised by its collinear pieces: swap it for the pieces in the
  // constraints set, so the parity flood treats them as boundary and the missing count stays
  // honest (Pool_Nozzle's cone rim: an out-and-back boundary revisits the rim row and parks a
  // vertex exactly on another rim segment).
  const chainSplit = (ck: number): boolean => {
    const a = Math.floor(ck / 0x8000000), b = ck % 0x8000000;
    const A = pts[a]!, B = pts[b]!;
    const abx = B[0] - A[0], aby = B[1] - A[1];
    const len2 = abx * abx + aby * aby;
    if (len2 < 1e-30) return false;
    // Duplicate-vertex equivalence first: an out-and-back boundary visits the same (u,v) twice
    // with two point indices; the CDT realises the edge between ONE pairing of the duplicates and
    // the constraint references the other. A geometrically-identical realised edge counts.
    const eps2 = 1e-18 * len2;
    const eqOf = (i: number): number[] => {
      const Q = pts[i]!;
      const out = [i];
      for (let v = 0; v < pts.length; v++) {
        if (v === i) continue;
        const P = pts[v]!;
        const dx = P[0] - Q[0], dy = P[1] - Q[1];
        if (dx * dx + dy * dy <= eps2) out.push(v);
      }
      return out;
    };
    const Ea = eqOf(a), Eb = eqOf(b);
    if (Ea.length > 1 || Eb.length > 1) {
      for (const va of Ea) {
        for (const vb of Eb) {
          const k2 = ckey(va, vb);
          if (k2 !== ck && present.has(k2)) {
            if (DBG && process.env.MESHSTEP_SLICEDBG) console.error(`[cdt]     chainSplit ${a}-${b}: realised by duplicate pair ${va}-${vb}`);
            constraints.add(k2);
            constraints.delete(ck);
            return true;
          }
        }
      }
    }
    const mid: { t: number; v: number }[] = [];
    for (let v = 0; v < pts.length; v++) {
      if (v === a || v === b) continue;
      const P = pts[v]!;
      const t = ((P[0] - A[0]) * abx + (P[1] - A[1]) * aby) / len2;
      if (t <= 1e-9 || t >= 1 - 1e-9) continue;
      const dx = P[0] - (A[0] + t * abx), dy = P[1] - (A[1] + t * aby);
      if (dx * dx + dy * dy > eps2) continue;
      mid.push({ t, v });
    }
    if (!mid.length) return false;
    mid.sort((p, q) => p.t - q.t);
    const chain = [a, ...mid.map((m) => m.v), b];
    for (let i = 0; i + 1 < chain.length; i++) {
      if (!present.has(ckey(chain[i]!, chain[i + 1]!))) {
        if (DBG && process.env.MESHSTEP_SLICEDBG) console.error(`[cdt]     chainSplit ${a}-${b}: piece ${chain[i]}-${chain[i + 1]} absent`);
        return false;
      }
    }
    for (let i = 0; i + 1 < chain.length; i++) constraints.add(ckey(chain[i]!, chain[i + 1]!));
    constraints.delete(ck);
    return true;
  };
  let unrealized = 0, chainRealized = 0;
  for (const ck of constraints) {
    if (present.has(ck)) continue;
    unrealized++;
    const a = Math.floor(ck / 0x8000000), b = ck % 0x8000000;
    // Cavity enforcement first — it realises the constraint EXACTLY whenever a cavity exists.
    // Only when it can't (crossed=0: nothing to flip, no cavity to fill) fall back to accepting a
    // geometrically-identical realisation (duplicate pair / collinear chain).
    if (!enforceByRetriangulation(tris, pts, free, a, b, constraints) && chainSplit(ck)) chainRealized++;
  }
  if (DBG && Date.now() - dbgT0 > 500) {
    console.error(`[cdt] SLOW n=${n} constraints=${constraints.size} unrealized=${unrealized}: insert=${dbgT1 - dbgT0}ms force=${dbgT2 - dbgT1}ms enforce=${Date.now() - dbgT2}ms`);
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
  // (over the constraints set, not the raw loops: a chain-split constraint is realised by its
  // collinear pieces and must not count as missing)
  let missing = 0;
  for (const ck of constraints) if (!edgeTris.has(ck)) missing++;
  if (out) out.missing = missing;
  if (DBG && missing > 0) console.error(`[cdt] missing=${missing}/${constraints.size} pts=${n} loops=[${loops.map((l) => l.length).join(",")}] interior=${interior.length} flood=${flood.length}`);
  if (missing === 0) {
    if (chainRealized === 0 || loops.length !== 1) return flood;
    // Constraints realised only by EQUIVALENCE (duplicate pair / collinear chain): the parity flood
    // can still classify the region wrongly around the duplicated vertices — but the single-loop
    // rescue can be wrong the OTHER way (it fills the out-and-back spike as a notch). Neither rule
    // wins at every tolerance (Pool_Nozzle: flood correct at 0.002mm absolute, rescue correct at
    // corpus-relative). Judge by OUTCOME: count coverage defects — a boundary segment covered ≠ 1×,
    // any other edge covered ≠ 2× or once — and keep the cleaner candidate (flood wins ties).
    const raw = new Set<number>();
    for (const loop of loops) for (let i = 0; i < loop.length; i++) raw.add(ckey(loop[i]!, loop[(i + 1) % loop.length]!));
    const defects = (cand: [number, number, number][]): number => {
      const fe = new Map<number, number>();
      for (const [a, b, c] of cand) for (const [x, y] of [[a, b], [b, c], [c, a]] as const) fe.set(ckey(x, y), (fe.get(ckey(x, y)) ?? 0) + 1);
      let bad = 0;
      for (const [k, cnt] of fe) {
        const isB = raw.has(k) || constraints.has(k);
        if (cnt === 1) { if (!isB) bad++; }
        else if (cnt === 2) { if (isB) bad++; }
        else bad += cnt - 2;
      }
      return bad;
    };
    let alt = boundaryFillWithInterior(points, loops[0]!, interior);
    if (!alt || alt.length === 0) {
      const simplified = dropMicroLoops(points, loops[0]!);
      if (simplified) alt = boundaryFillWithInterior(points, simplified, interior);
    }
    if (!alt || alt.length === 0) alt = fanFill(points, loops[0]!);
    // Geometric (centroid-in-polygon) classification is the third candidate — it was the winning
    // path for these rings before equivalence-realisation existed (missing>0 used to fall through
    // to it), and pnpoly is immune to the duplicated vertices that mislead the parity flood.
    const outerPoly0 = loops[0]!.map((i) => points[i]!);
    const geom0: [number, number, number][] = [];
    for (let i = 0; i < tris.length; i++) {
      const t = tris[i]!;
      if (t.dead || t.v[0] >= n || t.v[1] >= n || t.v[2] >= n) continue;
      const cxp = (points[t.v[0]]![0] + points[t.v[1]]![0] + points[t.v[2]]![0]) / 3;
      const cyp = (points[t.v[0]]![1] + points[t.v[1]]![1] + points[t.v[2]]![1]) / 3;
      if (pnpoly(cxp, cyp, outerPoly0)) geom0.push([t.v[0], t.v[1], t.v[2]]);
    }
    let best = flood, dBest = defects(flood), label = "flood";
    for (const [name, cand] of [["rescue", alt], ["geom", geom0]] as const) {
      if (!cand || cand.length === 0) continue;
      const d = defects(cand);
      if (d < dBest) { best = cand; dBest = d; label = name; }
    }
    if (DBG) console.error(`[cdt]   equivalence-realised: flood=${defects(flood)} rescue=${alt ? defects(alt) : "-"} geom=${defects(geom0)} -> ${label}`);
    return best;
  }

  // Constraints unrealised — the (u,v) embedding collapsed (a thin-sliver B-spline parameter domain
  // where distinct 3D points coincide / fall collinear, so the CDT triangulated across the boundary
  // and shattered the face internally). For a single loop with no holes, discard the CDT result and
  // re-triangulate from the boundary ring directly (ear-clip + interior re-insertion): this
  // guarantees every boundary edge is a triangle edge — hence watertight with the neighbour, which
  // shares those exact samples — while still following the surface through the interior points.
  // Falls through to the geometric classification if the ring isn't a clean simple polygon.
  if (loops.length === 1 && (loops[0]?.length ?? 0) >= 3) {
    let filled = boundaryFillWithInterior(points, loops[0]!, interior);
    if (DBG) console.error(`[cdt]   boundaryFillWithInterior: ${filled ? `${filled.length} tris` : "FAILED (ring not simple)"}`);
    if (!filled || filled.length === 0) {
      // Ring not simple — often micro self-crossings at slot/thread run-outs. Excise them and retry.
      const simplified = dropMicroLoops(points, loops[0]!);
      if (simplified) {
        filled = boundaryFillWithInterior(points, simplified, interior);
        if (DBG) console.error(`[cdt]   dropMicroLoops(${loops[0]!.length}->${simplified.length}) + refill: ${filled ? `${filled.length} tris` : "still FAILED"}`);
      }
    }
    if (filled && filled.length > 0) return filled;
    // Ear-clip failed — the loop is a triangle with a collinear side (a curved slice meeting at a
    // cone apex / sphere pole). Fan from the apex, making every rim segment a triangle edge.
    const fan = fanFill(points, loops[0]!);
    if (DBG) console.error(`[cdt]   fanFill: ${fan ? `${fan.length} tris` : "FAILED"}`);
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
  if (DBG) console.error(`[cdt]   selfCross=${selfCross} -> ${selfCross ? "flood-or-nothing" : "geometric classification"}`);
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
