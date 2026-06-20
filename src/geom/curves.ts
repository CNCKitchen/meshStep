// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — edge-curve sampling into 3D polylines (Line, Circle; B-spline curve falls back
// to a straight chord for now and is upgraded in M3b). Endpoints are snapped to the exact
// STEP vertices so that edges sharing a vertex weld watertight.
import type { Vec3 } from "./vec.ts";
import { add, dot, scale, sub, dist, lerp } from "./vec.ts";
import { readPlacement, readPoint } from "./placement.ts";
import { Table, num, ref, refList, numList } from "../step/entities.ts";

const TWO_PI = Math.PI * 2;

/** Evaluate a (possibly RATIONAL) B-spline curve at parameter u via de Boor in homogeneous space.
 * Non-rational curves pass weights all 1 and reduce to the standard algorithm. */
function deBoor(degree: number, cps: Vec3[], knots: number[], u: number, weights?: number[]): Vec3 {
  const n = cps.length - 1;
  let k = degree;
  while (k < n && knots[k + 1]! <= u) k++;
  // Homogeneous control points [x·w, y·w, z·w, w]; lerp in 4D, then project back to 3D.
  const d: [number, number, number, number][] = [];
  for (let j = 0; j <= degree; j++) {
    const idx = k - degree + j, p = cps[idx]!, w = weights ? weights[idx]! : 1;
    d[j] = [p[0] * w, p[1] * w, p[2] * w, w];
  }
  for (let r = 1; r <= degree; r++) {
    for (let j = degree; j >= r; j--) {
      const i = k - degree + j;
      const lo = knots[i]!, hi = knots[i + degree - r + 1]!;
      const a = hi > lo ? (u - lo) / (hi - lo) : 0;
      const p = d[j - 1]!, q = d[j]!;
      d[j] = [p[0] + (q[0] - p[0]) * a, p[1] + (q[1] - p[1]) * a, p[2] + (q[2] - p[2]) * a, p[3] + (q[3] - p[3]) * a];
    }
  }
  const r = d[degree]!, w = r[3] || 1;
  return [r[0] / w, r[1] / w, r[2] / w];
}

/** Extract B-spline curve data from a simple B_SPLINE_CURVE_WITH_KNOTS entity OR a complex instance
 * combining B_SPLINE_CURVE + B_SPLINE_CURVE_WITH_KNOTS [+ RATIONAL_B_SPLINE_CURVE] (a rational/NURBS
 * curve — e.g. a conic from a chamfer ∩ cylinder). Complex sub-records have no leading name param, so
 * the field offsets differ. Returns null if the entity isn't a B-spline curve. */
function bsplineData(t: Table, curveId: number, s: number): { degree: number; cps: Vec3[]; knots: number[]; weights: number[]; u0: number; u1: number } | null {
  let degree: number, cpsRefs: number[], mults: number[], knotVals: number[], weights: number[];
  if (!t.isComplex(curveId) && t.typeOf(curveId) === "B_SPLINE_CURVE_WITH_KNOTS") {
    const r = t.record(curveId); // (name, degree, cps, form, closed, self_int, mults, knots, spec)
    degree = num(r.params[1]!); cpsRefs = refList(r.params[2]!);
    mults = numList(r.params[6]!); knotVals = numList(r.params[7]!);
    weights = cpsRefs.map(() => 1);
  } else {
    const bc = t.sub(curveId, "B_SPLINE_CURVE"), bk = t.sub(curveId, "B_SPLINE_CURVE_WITH_KNOTS");
    if (!bc || !bk) return null;
    degree = num(bc.params[0]!); cpsRefs = refList(bc.params[1]!); // (degree, cps, form, closed, self_int)
    mults = numList(bk.params[0]!); knotVals = numList(bk.params[1]!); // (mults, knots, spec)
    const rat = t.sub(curveId, "RATIONAL_B_SPLINE_CURVE");
    weights = rat ? numList(rat.params[0]!) : cpsRefs.map(() => 1); // (weights)
  }
  const cps = cpsRefs.map((id) => readPoint(t, id, s));
  const knots: number[] = [];
  for (let i = 0; i < knotVals.length; i++) for (let j = 0; j < mults[i]!; j++) knots.push(knotVals[i]!);
  return { degree, cps, knots, weights, u0: knots[degree]!, u1: knots[cps.length]! };
}

function arcSegments(radius: number, sweepAbs: number, chordTol: number): number {
  const r = Math.max(radius, 1e-9);
  const dTheta = 2 * Math.acos(Math.max(0, Math.min(1, 1 - chordTol / r)));
  const seg = dTheta > 1e-6 ? Math.ceil(sweepAbs / dTheta) : 1;
  return Math.max(1, Math.min(4000, seg));
}

/**
 * Sample the curve of an EDGE_CURVE from vertex v0 to v1.
 * `sameSense` is the EDGE_CURVE flag (edge direction agrees with the curve's parametrisation).
 * Returns points from v0 to v1 inclusive (≥ 2 points).
 */
export function sampleEdgePolyline(
  t: Table, curveId: number, v0: Vec3, v1: Vec3, sameSense: boolean,
  s: number, chordTol: number, maxSegLen: number,
): Vec3[] {
  const kind = t.typeOf(curveId); // undefined for complex entities (surface/seam curves)

  if (kind === "CIRCLE") {
    const rec = t.record(curveId);
    const f = readPlacement(t, ref(rec.params[1]!), s);
    const R = num(rec.params[2]!) * s;
    const ang = (p: Vec3): number => {
      const d = sub(p, f.o);
      return Math.atan2(dot(d, f.y), dot(d, f.x));
    };
    const t0 = ang(v0);
    const t1 = ang(v1);
    const full = dist(v0, v1) < Math.max(1e-7, chordTol * 1e-3);
    let sweep: number;
    if (full) {
      sweep = sameSense ? TWO_PI : -TWO_PI;
    } else if (sameSense) {
      let d = t1 - t0;
      while (d <= 0) d += TWO_PI;
      while (d > TWO_PI) d -= TWO_PI;
      sweep = d;
    } else {
      let d = t1 - t0;
      while (d >= 0) d -= TWO_PI;
      while (d < -TWO_PI) d += TWO_PI;
      sweep = d;
    }
    const arcLen = Math.abs(sweep) * R;
    const n = Math.max(arcSegments(R, Math.abs(sweep), chordTol), Math.ceil(arcLen / Math.max(maxSegLen, 1e-9)));
    const pts: Vec3[] = [];
    for (let i = 0; i <= n; i++) {
      const a = t0 + (sweep * i) / n;
      pts.push(add(f.o, add(scale(f.x, Math.cos(a) * R), scale(f.y, Math.sin(a) * R))));
    }
    pts[0] = v0;
    pts[pts.length - 1] = v1;
    return pts;
  }

  if (kind === "ELLIPSE") {
    // Cylinder/cone ∩ plane = an ellipse arc (e.g. a fillet running into a chamfer). Sampled with the
    // ELLIPTIC eccentric angle t: p(t) = o + a·cos t·x + b·sin t·y. Without this it falls through to
    // the straight-line case below and the arc is drawn as a single secant chord (visible flat band).
    const rec = t.record(curveId);
    const f = readPlacement(t, ref(rec.params[1]!), s);
    const a = num(rec.params[2]!) * s, b = num(rec.params[3]!) * s;
    const at = (p: Vec3): number => { const d = sub(p, f.o); return Math.atan2(dot(d, f.y) / b, dot(d, f.x) / a); };
    const t0 = at(v0), t1 = at(v1);
    const full = dist(v0, v1) < Math.max(1e-7, chordTol * 1e-3);
    let sweep: number;
    if (full) sweep = sameSense ? TWO_PI : -TWO_PI;
    else if (sameSense) { let d = t1 - t0; while (d <= 0) d += TWO_PI; while (d > TWO_PI) d -= TWO_PI; sweep = d; }
    else { let d = t1 - t0; while (d >= 0) d -= TWO_PI; while (d < -TWO_PI) d += TWO_PI; sweep = d; }
    // Tightest curvature radius (minSemi²/maxSemi) bounds the chord deviation everywhere; arc length
    // ~ mean-radius·sweep bounds the segment length. Uniform-in-t sampling is fine for both.
    const rMin = Math.min(a, b) ** 2 / Math.max(a, b, 1e-9);
    const n = Math.max(arcSegments(rMin, Math.abs(sweep), chordTol), Math.ceil((Math.abs(sweep) * (a + b) / 2) / Math.max(maxSegLen, 1e-9)));
    const pts: Vec3[] = [];
    for (let i = 0; i <= n; i++) {
      const ta = t0 + (sweep * i) / n;
      pts.push(add(f.o, add(scale(f.x, Math.cos(ta) * a), scale(f.y, Math.sin(ta) * b))));
    }
    pts[0] = v0;
    pts[pts.length - 1] = v1;
    return pts;
  }

  // B-spline curve — simple (non-rational) OR a complex/rational instance (e.g. a conic from a
  // chamfer ∩ cylinder). Without this a complex curve falls through to the straight chord below and
  // the surrounding triangles snap to that secant instead of following the arc.
  const bs = bsplineData(t, curveId, s);
  if (bs) {
    const { degree, cps, knots, weights, u0, u1 } = bs;
    let clen = 0;
    for (let i = 1; i < cps.length; i++) clen += dist(cps[i - 1]!, cps[i]!);
    const n = Math.max(2, Math.ceil(clen / Math.max(maxSegLen, 1e-9)));
    let pts: Vec3[] = [];
    for (let i = 0; i <= n; i++) pts.push(deBoor(degree, cps, knots, u0 + ((u1 - u0) * i) / n, weights));
    if (dist(pts[0]!, v0) > dist(pts[0]!, v1)) pts = pts.reverse(); // orient v0 -> v1
    pts[0] = v0;
    pts[pts.length - 1] = v1;
    return pts;
  }

  // LINE and anything else: straight, subdivided to the target edge length. Done in the shared
  // edge table so both adjacent faces get identical points (watertight).
  const n = Math.max(1, Math.ceil(dist(v0, v1) / Math.max(maxSegLen, 1e-9)));
  const pts: Vec3[] = [];
  for (let i = 0; i <= n; i++) pts.push(lerp(v0, v1, i / n));
  pts[0] = v0;
  pts[pts.length - 1] = v1;
  return pts;
}
