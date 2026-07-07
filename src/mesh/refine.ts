// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — conforming isotropic refinement of a fallback 2D triangulation. The ear-clip fill
// (earcut.ts) is watertight by construction but places NO interior vertices: on a large multi-loop
// plane every triangle is a boundary-to-boundary sliver, visibly coarser (and far worse shaped)
// than the refined grid-CDT that meshes every other face. This pass brings such a fill up to the
// face's interior edge target without touching what makes it watertight — an incremental
// isotropic-remesh loop (split / flip / smooth per round, à la Botsch-Kobbelt, minus collapses):
//   - long INTERIOR edges are split at their midpoint; both incident triangles split together, so
//     the triangulation stays conforming (no T-junctions). Boundary segments are never split —
//     those polylines are shared verbatim with the neighbouring faces. The split length is GRADED:
//     besides the absolute target, an edge much longer than its triangles' shortest edge is split
//     too, so density transitions smoothly away from finely-sampled boundary arcs instead of
//     leaving one ring of needle fans.
//   - Lawson (Delaunay) edge flips restore quality around the inserted vertices; a flip is vetoed
//     when it would invert or over-thin a triangle, or duplicate an existing edge (a duplicate
//     would weld non-manifold in 3D).
//   - inserted vertices (only) are Laplace-smoothed, guarded so no incident triangle inverts or
//     gets thinner than it started.
// Valid on a PLANE (or any affine (u,v)->3D map): 2D coverage/conformity equals 3D coverage.
// Degenerate and near-degenerate (thin) triangles from tangled input are left untouched — they
// vanish (or barely show) in the 3D emit, and refining them would seed vertices micrometres from
// OTHER boundary points (collinear boundary runs, sub-tolerance crescent loops), which the 3D
// weld's 1e-6 quantisation would fuse into non-manifold edges.

/** Undirected edge key for vertex indices < 2^22 (well above any face's vertex count). */
export const K_EDGE = 0x400000;
export const ekey = (a: number, b: number): number => (a < b ? a * K_EDGE + b : b * K_EDGE + a);

/** Twice the signed area of triangle (a,b,c) in the flat [x0,y0,x1,y1,...] point array. */
function area2(pts: number[], a: number, b: number, c: number): number {
  const ax = pts[a * 2]!, ay = pts[a * 2 + 1]!;
  return (pts[b * 2]! - ax) * (pts[c * 2 + 1]! - ay) - (pts[c * 2]! - ax) * (pts[b * 2 + 1]! - ay);
}

/** Refine `tris` (flattened index triples over `pts`) toward `target` edge length in place.
 * `boundary` holds ekey()s of the loop segments that must survive verbatim (never split, never
 * duplicated by a flip). All vertices present on entry are treated as immovable. */
export function refineTriangulation(pts: number[], tris: number[], boundary: Set<number>, target: number): void {
  const nFixed = pts.length / 2;
  const epsA = 1e-9 * target * target; // "positively oriented" threshold for area2, mm² scale
  // Minimum ALTITUDE for a triangle to take part in refinement (1% of the edge target). Ear-clip
  // fans contain ultra-thin slivers whose long edge passes micrometres from an unrelated boundary
  // vertex: splitting such an edge drops the midpoint next to that vertex, and each further round
  // converges geometrically until points collide at the 3D weld's 1e-6 quantum — welding
  // non-manifold. An area threshold cannot see this (long × thin is a "large" area); altitude
  // = area2/longestEdge can.
  const altMin = 0.01 * target;
  // Canonicalize winding CCW so the flip/smooth inversion guards defend a single sign.
  for (let t = 0; t < tris.length; t += 3) {
    if (area2(pts, tris[t]!, tris[t + 1]!, tris[t + 2]!) < 0) {
      const tmp = tris[t + 1]!; tris[t + 1] = tris[t + 2]!; tris[t + 2] = tmp;
    }
  }
  const len2 = (a: number, b: number): number => {
    const dx = pts[a * 2]! - pts[b * 2]!, dy = pts[a * 2 + 1]! - pts[b * 2 + 1]!;
    return dx * dx + dy * dy;
  };
  /** Altitude of triangle at offset t: area2 / longest edge (0 for inverted). */
  const altitude = (t: number): number => {
    const a = tris[t]!, b = tris[t + 1]!, c = tris[t + 2]!;
    const A2 = area2(pts, a, b, c);
    if (A2 <= 0) return 0;
    return A2 / Math.sqrt(Math.max(len2(a, b), len2(b, c), len2(c, a)));
  };
  /** Positively oriented AND at least altMin thick — safe to refine around. */
  const fatEnough = (t: number): boolean => area2(pts, tris[t]!, tris[t + 1]!, tris[t + 2]!) > epsA && altitude(t) > altMin;
  /** Edge key -> offsets of the triangles using it (rebuilt per pass; sizes stay small). */
  const edgeTris = (): Map<number, number[]> => {
    const m = new Map<number, number[]>();
    for (let t = 0; t < tris.length; t += 3) {
      for (let e = 0; e < 3; e++) {
        const k = ekey(tris[t + e]!, tris[t + (e + 1) % 3]!);
        const arr = m.get(k);
        if (arr) arr.push(t); else m.set(k, [t]);
      }
    }
    return m;
  };

  /** One split pass: midpoint-split every over-long interior edge (longest first, one split per
   * triangle per pass). Returns the number of splits. */
  const splitPass = (): number => {
    const em = edgeTris();
    const lim2 = 2.25 * target * target; // absolute: split above 1.5×target
    const floor2 = 0.04 * target * target; // graded splits never go below 0.2×target
    const cand: [number, number, number][] = []; // [len2, a, b]
    for (const [k, ts] of em) {
      if (ts.length !== 2 || boundary.has(k)) continue;
      const a = (k / K_EDGE) | 0, b = k % K_EDGE;
      // both incident triangles must be thick enough that the midpoint lands in open space
      if (!ts.every(fatEnough)) continue;
      const l2 = len2(a, b);
      if (l2 <= floor2) continue;
      let want = l2 > lim2;
      if (!want) {
        // graded criterion: an edge ≥3× its triangles' shortest edge makes a needle — split it
        // even below the absolute target so density blends into finely-sampled boundary arcs.
        // The local edge is CLAMPED from below at 0.25×target: without the clamp a µm micro-
        // feature (sub-tolerance crescent loop, coincident collinear chains — this file's STEP is
        // genuinely degenerate there) cascades its neighbourhood down to the floor and paints a
        // dense "fuzz" carpet around itself.
        let mn2 = Infinity;
        for (const t of ts) {
          for (let e = 0; e < 3; e++) {
            const l = len2(tris[t + e]!, tris[t + (e + 1) % 3]!);
            if (l < mn2) mn2 = l;
          }
        }
        want = l2 > 9 * Math.max(mn2, 0.0625 * target * target);
      }
      if (want) cand.push([l2, a, b]);
    }
    cand.sort((p, q) => q[0] - p[0]);
    const touched = new Set<number>();
    let split = 0;
    for (const [, a, b] of cand) {
      const ts = em.get(ekey(a, b))!;
      if (ts.some((t) => touched.has(t))) continue; // stale this pass — next round catches it
      const m = pts.length / 2;
      pts.push((pts[a * 2]! + pts[b * 2]!) / 2, (pts[a * 2 + 1]! + pts[b * 2 + 1]!) / 2);
      for (const t of ts) {
        // copy the triangle, then substitute m for b in the original and for a in the copy —
        // winding (and thus CCW orientation) is preserved in both halves
        const nt = tris.length;
        tris.push(tris[t]!, tris[t + 1]!, tris[t + 2]!);
        for (let e = 0; e < 3; e++) {
          if (tris[t + e] === b) tris[t + e] = m;
          if (tris[nt + e] === a) tris[nt + e] = m;
        }
        touched.add(t); touched.add(nt);
      }
      split++;
    }
    return split;
  };

  /** Lawson flip sweeps toward Delaunay, up to `maxSweeps`. */
  const flipPasses = (maxSweeps: number): void => {
    for (let sweep = 0; sweep < maxSweeps; sweep++) {
      const em = edgeTris();
      let flips = 0;
      for (const [k, ts] of em) {
        if (ts.length !== 2 || boundary.has(k)) continue;
        const a0 = (k / K_EDGE) | 0, b0 = k % K_EDGE;
        let [t1, t2] = ts as [number, number];
        // the map goes stale after each flip — verify both triangles still hold this edge
        const has = (t: number, x: number): boolean => tris[t] === x || tris[t + 1] === x || tris[t + 2] === x;
        if (!has(t1, a0) || !has(t1, b0) || !has(t2, a0) || !has(t2, b0)) continue;
        // orient so t1 traverses a->b (CCW keeps c strictly left, d strictly right)
        const a = a0, b = b0;
        const traverses = (t: number, x: number, y: number): boolean => {
          for (let e = 0; e < 3; e++) if (tris[t + e] === x && tris[t + (e + 1) % 3] === y) return true;
          return false;
        };
        if (!traverses(t1, a, b)) { const tt = t1; t1 = t2; t2 = tt; }
        if (!traverses(t1, a, b) || !traverses(t2, b, a)) continue; // tangled/degenerate — leave it
        const third = (t: number, x: number, y: number): number => {
          for (let e = 0; e < 3; e++) { const v = tris[t + e]!; if (v !== x && v !== y) return v; }
          return -1;
        };
        const c = third(t1, a, b), d = third(t2, a, b);
        if (c < 0 || d < 0 || c === d) continue;
        const altOld = Math.min(altitude(t1), altitude(t2));
        // THIN-CAP absorption: ear-clipping leaves sub-µm caps along collinear boundary runs (a
        // vertex sits ON the opposite long edge). They can't be split (their neighbourhood is weld
        // territory), but the flip reroutes the fat side exactly through the collinear vertex —
        // no new coordinates, one thin triangle absorbed per flip. Otherwise: plain Delaunay.
        const thinCap = altOld <= altMin;
        if (!thinCap) {
          if (area2(pts, a, b, c) <= epsA || area2(pts, b, a, d) <= epsA) continue; // degenerate flap
          // Delaunay: flip only when d is strictly inside the circumcircle of CCW (a,b,c)
          const dx = pts[d * 2]!, dy = pts[d * 2 + 1]!;
          const adx = pts[a * 2]! - dx, ady = pts[a * 2 + 1]! - dy;
          const bdx = pts[b * 2]! - dx, bdy = pts[b * 2 + 1]! - dy;
          const cdx = pts[c * 2]! - dx, cdy = pts[c * 2 + 1]! - dy;
          const det =
            (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy) -
            (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady) +
            (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
          if (det <= 0) continue;
        }
        // validity: convex quad (both new triangles CCW) and the new edge must not already exist
        if (area2(pts, a, d, c) <= epsA || area2(pts, d, b, c) <= epsA) continue;
        const nk = ekey(c, d);
        if (boundary.has(nk) || em.has(nk)) continue;
        tris[t1] = a; tris[t1 + 1] = d; tris[t1 + 2] = c;
        tris[t2] = d; tris[t2 + 1] = b; tris[t2 + 2] = c;
        // a thin-cap flip must land in healthy territory; a Delaunay flip must never trade a
        // healthy pair for a thinner one near a degenerate zone — undo if worse
        const altNew = Math.min(altitude(t1), altitude(t2));
        if (thinCap ? altNew <= altMin : altNew < Math.min(altOld, altMin)) {
          tris[t1] = a; tris[t1 + 1] = b; tris[t1 + 2] = c;
          tris[t2] = b; tris[t2 + 1] = a; tris[t2 + 2] = d;
          continue;
        }
        em.set(nk, [t1, t2]); // block a duplicate flip onto (c,d) within this sweep
        flips++;
      }
      if (flips === 0) break;
    }
  };

  /** Laplace-smooth the inserted vertices; a move is undone if any incident triangle inverts or
   * ends up thinner than both altMin and what it was before the move. */
  const smoothPasses = (passes: number): void => {
    for (let pass = 0; pass < passes; pass++) {
      const nbr = new Map<number, Set<number>>();
      const inc = new Map<number, number[]>();
      for (let t = 0; t < tris.length; t += 3) {
        for (let e = 0; e < 3; e++) {
          const v = tris[t + e]!;
          if (v < nFixed) continue;
          let s = nbr.get(v);
          if (!s) { s = new Set(); nbr.set(v, s); inc.set(v, []); }
          s.add(tris[t + (e + 1) % 3]!); s.add(tris[t + (e + 2) % 3]!);
          inc.get(v)!.push(t);
        }
      }
      for (const [v, s] of nbr) {
        const its = inc.get(v)!;
        let altBefore = Infinity;
        for (const t of its) { const al = altitude(t); if (al < altBefore) altBefore = al; }
        let sx = 0, sy = 0;
        for (const n of s) { sx += pts[n * 2]!; sy += pts[n * 2 + 1]!; }
        const ox = pts[v * 2]!, oy = pts[v * 2 + 1]!;
        pts[v * 2] = sx / s.size; pts[v * 2 + 1] = sy / s.size;
        for (const t of its) {
          const A2 = area2(pts, tris[t]!, tris[t + 1]!, tris[t + 2]!);
          if (A2 <= epsA || altitude(t) < Math.min(altBefore, altMin)) {
            pts[v * 2] = ox; pts[v * 2 + 1] = oy;
            break;
          }
        }
      }
    }
  };

  // Incremental remesh loop: refine, re-triangulate, relax — each round's flips (which also
  // absorb thin caps) open room for the next round's splits, so only stop after two consecutive
  // rounds without a split.
  let dry = 0;
  for (let round = 0; round < 64 && dry < 2; round++) {
    dry = splitPass() === 0 ? dry + 1 : 0;
    flipPasses(4);
    smoothPasses(3);
    if (pts.length / 2 > 250_000) break; // runaway backstop, far beyond any real face
  }
  flipPasses(8);
  smoothPasses(4);
  flipPasses(4);
  smoothPasses(3);
}
