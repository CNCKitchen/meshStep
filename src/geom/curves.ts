// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — edge-curve sampling into 3D polylines (Line, Circle; B-spline curve falls back
// to a straight chord for now and is upgraded in M3b). Endpoints are snapped to the exact
// STEP vertices so that edges sharing a vertex weld watertight.
import type { Vec3 } from "./vec.ts";
import { add, dot, scale, sub, dist, lerp } from "./vec.ts";
import { readPlacement, readPoint, readDirection, type Frame } from "./placement.ts";
import { Table, num, ref, refList, numList } from "../step/entities.ts";
import type { Param } from "../step/parser.ts";

const TWO_PI = Math.PI * 2;

// ---- Evaluable curve objects -----------------------------------------------------------------
// The polyline sampler below (sampleEdgePolyline) special-cases each curve type for edge meshing.
// Swept surfaces (extrusion/revolution) and trimmed/composite boundary curves additionally need a
// curve they can EVALUATE at parameters — this interface is that seam.

export interface Curve3 {
  kind: string;
  /** Parameter domain. */
  t0: number;
  t1: number;
  /** True if evaluate(t0) === evaluate(t1) (full circle/ellipse, closed B-spline). */
  closed: boolean;
  evaluate(t: number): Vec3;
  /** Parameter of the closest point (exact for conics/lines, iterative for B-splines). */
  project(p: Vec3): number;
  /** Smallest radius of curvature over the curve; Infinity for a line. Drives sampling density. */
  minRadius(): number;
}

class LineCurve implements Curve3 {
  kind = "LINE";
  o: Vec3; d: Vec3; // d includes the STEP vector magnitude: p(t) = o + t·d
  t0 = 0; t1 = 1; closed = false;
  constructor(o: Vec3, d: Vec3) { this.o = o; this.d = d; }
  evaluate(t: number): Vec3 { return add(this.o, scale(this.d, t)); }
  project(p: Vec3): number { const l2 = dot(this.d, this.d); return l2 > 0 ? dot(sub(p, this.o), this.d) / l2 : 0; }
  minRadius(): number { return Infinity; }
}

class CircleCurve implements Curve3 {
  kind = "CIRCLE";
  f: Frame; r: number;
  t0 = 0; t1 = TWO_PI; closed = true;
  constructor(f: Frame, r: number) { this.f = f; this.r = r; }
  evaluate(t: number): Vec3 { return add(this.f.o, add(scale(this.f.x, Math.cos(t) * this.r), scale(this.f.y, Math.sin(t) * this.r))); }
  project(p: Vec3): number { const d = sub(p, this.f.o); return Math.atan2(dot(d, this.f.y), dot(d, this.f.x)); }
  minRadius(): number { return this.r; }
}

class EllipseCurve implements Curve3 {
  kind = "ELLIPSE";
  f: Frame; a: number; b: number;
  t0 = 0; t1 = TWO_PI; closed = true;
  constructor(f: Frame, a: number, b: number) { this.f = f; this.a = a; this.b = b; }
  evaluate(t: number): Vec3 { return add(this.f.o, add(scale(this.f.x, Math.cos(t) * this.a), scale(this.f.y, Math.sin(t) * this.b))); }
  project(p: Vec3): number { const d = sub(p, this.f.o); return Math.atan2(dot(d, this.f.y) / this.b, dot(d, this.f.x) / this.a); }
  minRadius(): number { return Math.min(this.a, this.b) ** 2 / Math.max(this.a, this.b, 1e-9); }
}

class BSplineCurve3 implements Curve3 {
  kind = "B_SPLINE_CURVE";
  degree: number; cps: Vec3[]; knots: number[]; weights: number[];
  t0: number; t1: number; closed: boolean;
  private samples: { t: number; p: Vec3 }[] = [];
  constructor(degree: number, cps: Vec3[], knots: number[], weights: number[], t0: number, t1: number) {
    this.degree = degree; this.cps = cps; this.knots = knots; this.weights = weights;
    this.t0 = t0; this.t1 = t1;
    this.closed = dist(this.evaluate(t0), this.evaluate(t1)) < 1e-6;
    const N = Math.max(16, cps.length * 4);
    for (let i = 0; i <= N; i++) { const t = t0 + ((t1 - t0) * i) / N; this.samples.push({ t, p: this.evaluate(t) }); }
  }
  evaluate(t: number): Vec3 {
    const tc = Math.min(Math.max(t, this.t0), this.t1);
    return deBoor(this.degree, this.cps, this.knots, tc, this.weights);
  }
  project(p: Vec3): number {
    let bt = this.t0, bd = Infinity;
    for (const s of this.samples) { const d = dist(s.p, p); if (d < bd) { bd = d; bt = s.t; } }
    // refine by ternary-style local search around the best sample
    let step = (this.t1 - this.t0) / (this.samples.length - 1);
    for (let it = 0; it < 20 && step > 1e-9 * (this.t1 - this.t0); it++) {
      const cand = [bt - step / 2, bt + step / 2];
      for (const t of cand) {
        const tc = Math.min(Math.max(t, this.t0), this.t1);
        const d = dist(this.evaluate(tc), p);
        if (d < bd) { bd = d; bt = tc; }
      }
      step /= 2;
    }
    return bt;
  }
  minRadius(): number {
    // Turn-angle over chord-length between consecutive samples bounds the curvature radius.
    let minR = Infinity;
    for (let i = 1; i + 1 < this.samples.length; i++) {
      const a = this.samples[i - 1]!.p, b = this.samples[i]!.p, c = this.samples[i + 1]!.p;
      const u = sub(b, a), v = sub(c, b);
      const lu = Math.hypot(u[0], u[1], u[2]), lv = Math.hypot(v[0], v[1], v[2]);
      if (lu < 1e-12 || lv < 1e-12) continue;
      const ang = Math.acos(Math.max(-1, Math.min(1, dot(u, v) / (lu * lv))));
      if (ang > 1e-5) minR = Math.min(minR, ((lu + lv) / 2) / ang);
    }
    return minR;
  }
}

/** A basis curve restricted to [ta, tb], reparametrised to t∈[0,1] running trim1 -> trim2. */
class TrimmedCurve3 implements Curve3 {
  kind = "TRIMMED_CURVE";
  base: Curve3; ta: number; tb: number;
  t0 = 0; t1 = 1; closed: boolean;
  constructor(base: Curve3, ta: number, tb: number) {
    this.base = base; this.ta = ta; this.tb = tb;
    this.closed = Math.abs(Math.abs(tb - ta) - (base.t1 - base.t0)) < 1e-9 && base.closed;
  }
  evaluate(t: number): Vec3 { return this.base.evaluate(this.ta + (this.tb - this.ta) * t); }
  project(p: Vec3): number {
    let t = this.base.project(p);
    if (this.base.closed) { // choose the branch inside [ta,tb]
      const period = this.base.t1 - this.base.t0;
      const lo = Math.min(this.ta, this.tb);
      while (t < lo) t += period;
      while (t > lo + period) t -= period;
    }
    const u = (t - this.ta) / ((this.tb - this.ta) || 1e-12);
    return Math.min(1, Math.max(0, u));
  }
  minRadius(): number { return this.base.minRadius(); }
}

/**
 * Construct an evaluable Curve3 from a STEP curve entity (LINE, CIRCLE, ELLIPSE, B-spline simple or
 * complex/rational, TRIMMED_CURVE). `aRad` converts file plane-angle units to radians (trim values
 * on conics are given in the file's angle unit). Returns null for unsupported curve types.
 */
export function makeCurve(t: Table, id: number, s: number, aRad = 1): Curve3 | null {
  const kind = t.typeOf(id);
  // SURFACE_CURVE / SEAM_CURVE / INTERSECTION_CURVE(name, curve_3d#, (pcurves), rep) carry their
  // true geometry in the referenced 3D curve; the pcurves are parameter-space companions.
  if (kind === "SURFACE_CURVE" || kind === "SEAM_CURVE" || kind === "INTERSECTION_CURVE") {
    return makeCurve(t, ref(t.record(id).params[1]!), s, aRad);
  }
  if (kind === "LINE") {
    const r = t.record(id); // LINE(name, pnt#, vector#)
    const o = readPoint(t, ref(r.params[1]!), s);
    const vec = t.record(ref(r.params[2]!)); // VECTOR(name, dir#, magnitude)
    const dir = readDirection(t, ref(vec.params[1]!));
    return new LineCurve(o, scale(dir, num(vec.params[2]!) * s));
  }
  if (kind === "CIRCLE") {
    const r = t.record(id);
    return new CircleCurve(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s);
  }
  if (kind === "ELLIPSE") {
    const r = t.record(id);
    return new EllipseCurve(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s, num(r.params[3]!) * s);
  }
  if (kind === "TRIMMED_CURVE") {
    // TRIMMED_CURVE(name, basis#, (trim1), (trim2), senseAgreement, masterRepresentation)
    const r = t.record(id);
    const base = makeCurve(t, ref(r.params[1]!), s, aRad);
    if (!base) return null;
    const conic = base.kind === "CIRCLE" || base.kind === "ELLIPSE";
    const trim = (p: Param): number | null => {
      if (p.k !== "list") return null;
      // Prefer a cartesian point trim; else PARAMETER_VALUE / bare number (angle-unit scaled on conics).
      for (const q of p.v) if (q.k === "ref") return base.project(readPoint(t, q.v, s));
      for (const q of p.v) {
        if (q.k === "num") return conic ? q.v * aRad : q.v;
        if (q.k === "typed" && q.params[0]?.k === "num") return conic ? q.params[0].v * aRad : q.params[0].v;
      }
      return null;
    };
    let ta = trim(r.params[2]!), tb = trim(r.params[3]!);
    if (ta === null || tb === null) return null;
    const sense = r.params[4]?.k === "enum" ? r.params[4].v === "T" : true;
    if (base.closed) {
      const period = base.t1 - base.t0;
      // On a closed basis run trim1 -> trim2 the way `sense` dictates (wrapping across the seam).
      if (sense && tb <= ta) tb += period;
      if (!sense && tb >= ta) tb -= period;
    }
    return sense || base.closed ? new TrimmedCurve3(base, ta, tb) : new TrimmedCurve3(base, tb, ta);
  }
  const bs = bsplineData(t, id, s);
  if (bs) return new BSplineCurve3(bs.degree, bs.cps, bs.knots, bs.weights, bs.u0, bs.u1);
  return null;
}

/**
 * Adaptively sample a Curve3 over [ta, tb] (defaults to its whole domain): start uniform, then
 * bisect any segment whose midpoint sags more than chordTol off the chord or that is longer than
 * maxSegLen. Returns ≥ 2 points, capped to stay bounded on pathological input.
 */
export function sampleCurve(c: Curve3, chordTol: number, maxSegLen: number, ta = c.t0, tb = c.t1): Vec3[] {
  const ts: number[] = [];
  const n0 = c.closed || c.kind === "TRIMMED_CURVE" ? 8 : 2;
  for (let i = 0; i <= n0; i++) ts.push(ta + ((tb - ta) * i) / n0);
  const pts = ts.map((t) => c.evaluate(t));
  const maxPts = 4000;
  for (let i = 0; i + 1 < ts.length && ts.length < maxPts; ) {
    const tm = (ts[i]! + ts[i + 1]!) / 2;
    const pm = c.evaluate(tm);
    const a = pts[i]!, b = pts[i + 1]!;
    const chord = dist(a, b);
    const mid = lerp(a, b, 0.5);
    const sag = dist(pm, mid);
    if ((sag > chordTol || chord > maxSegLen) && dist(a, pm) > 1e-9 && dist(pm, b) > 1e-9) {
      ts.splice(i + 1, 0, tm); pts.splice(i + 1, 0, pm);
    } else i++;
  }
  return pts;
}

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
  s: number, chordTol: number, maxSegLen: number, aRad = 1,
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

  // Any other curve type makeCurve understands (TRIMMED_CURVE, SURFACE_CURVE/SEAM_CURVE wrapping a
  // conic, ...): sample it adaptively instead of collapsing to a straight secant chord.
  if (kind !== "LINE") {
    const c = makeCurve(t, curveId, s, aRad);
    if (c) {
      let pts = sampleCurve(c, chordTol, maxSegLen);
      const ringClosed = pts.length > 3 && dist(pts[0]!, pts[pts.length - 1]!) < Math.max(1e-9, chordTol * 1e-3);
      if (ringClosed) {
        // Closed curve (full circle/intersection ring): its seam start is unrelated to the edge's
        // vertices, so blind endpoint-snapping would fold the polyline back on itself and destroy
        // its winding. Orient by sameSense, then cut the ring at the samples nearest v0/v1 (for a
        // full-period edge v0==v1: rotate so the cycle starts at the vertex).
        const ring = pts.slice(0, -1);
        if (!sameSense) ring.reverse();
        const nearest = (q: Vec3): number => {
          let bi = 0, bd = Infinity;
          for (let i = 0; i < ring.length; i++) { const d = dist(ring[i]!, q); if (d < bd) { bd = d; bi = i; } }
          return bi;
        };
        const i0 = nearest(v0);
        if (dist(v0, v1) < Math.max(1e-7, chordTol * 1e-3)) {
          pts = [...ring.slice(i0), ...ring.slice(0, i0), v1];
          pts[0] = v0;
          return pts;
        }
        const i1 = nearest(v1);
        if (i0 !== i1) { // partial arc from v0 forward (in curve direction) to v1
          const arc: Vec3[] = [];
          for (let i = i0; ; i = (i + 1) % ring.length) { arc.push(ring[i]!); if (i === i1) break; }
          arc[0] = v0; arc[arc.length - 1] = v1;
          if (arc.length >= 2) return arc;
        }
      } else if (pts.length >= 2) {
        if (dist(pts[0]!, v0) > dist(pts[0]!, v1)) pts = pts.slice().reverse();
        pts[0] = v0;
        pts[pts.length - 1] = v1;
        return pts;
      }
    }
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
