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

// ---- Edge-curve classification (measurement metadata) ------------------------------------------

/** Analytic identity of one EDGE_CURVE, for measurement tools. Params are in mm (scaled). */
export interface EdgeCurveInfo {
  kind: "line" | "circle" | "ellipse" | "other";
  /** Circle/ellipse center and plane normal. */
  center?: Vec3;
  axis?: Vec3;
  /** Circle radius / ellipse semi-axes. */
  radius?: number;
  radius2?: number;
  /** Unit direction for a line edge. */
  dir?: Vec3;
  /** Signed traversed arc angle v0 -> v1 (rad); ±2π for a full circle. */
  sweep?: number;
  /** Exact analytic length where closed-form (line, circular arc); undefined otherwise. */
  length?: number;
}

/**
 * Classify the curve of an EDGE_CURVE from vertex v0 to v1 and extract its analytic parameters.
 * Mirrors `sampleEdgePolyline`'s dispatch (unwraps SURFACE_CURVE/SEAM_CURVE/INTERSECTION_CURVE and
 * TRIMMED_CURVE to the underlying conic/line) and its arc-sweep convention, so the reported
 * radius/center/sweep describe exactly the geometry the mesh boundary was sampled from.
 */
export function analyzeEdgeCurve(t: Table, curveId: number, v0: Vec3, v1: Vec3, sameSense: boolean, s: number): EdgeCurveInfo {
  const kind = t.typeOf(curveId);
  if (kind === "SURFACE_CURVE" || kind === "SEAM_CURVE" || kind === "INTERSECTION_CURVE" || kind === "TRIMMED_CURVE") {
    return analyzeEdgeCurve(t, ref(t.record(curveId).params[1]!), v0, v1, sameSense, s);
  }
  // Sweep of a conic edge: same full-circle gate (default-tolerance cap) and sense-directed
  // normalization as sampleEdgePolyline.
  const conicSweep = (t0: number, t1: number): number => {
    if (dist(v0, v1) < 1e-5) return sameSense ? TWO_PI : -TWO_PI;
    let d = t1 - t0;
    if (sameSense) { while (d <= 0) d += TWO_PI; while (d > TWO_PI) d -= TWO_PI; }
    else { while (d >= 0) d -= TWO_PI; while (d < -TWO_PI) d += TWO_PI; }
    return d;
  };
  if (kind === "CIRCLE") {
    const rec = t.record(curveId);
    const f = readPlacement(t, ref(rec.params[1]!), s);
    const R = num(rec.params[2]!) * s;
    const ang = (p: Vec3): number => { const d = sub(p, f.o); return Math.atan2(dot(d, f.y), dot(d, f.x)); };
    const sweep = conicSweep(ang(v0), ang(v1));
    return { kind: "circle", center: f.o, axis: f.z, radius: R, sweep, length: Math.abs(sweep) * R };
  }
  if (kind === "ELLIPSE") {
    const rec = t.record(curveId);
    const f = readPlacement(t, ref(rec.params[1]!), s);
    const a = num(rec.params[2]!) * s, b = num(rec.params[3]!) * s;
    const at = (p: Vec3): number => { const d = sub(p, f.o); return Math.atan2(dot(d, f.y) / b, dot(d, f.x) / a); };
    return { kind: "ellipse", center: f.o, axis: f.z, radius: a, radius2: b, sweep: conicSweep(at(v0), at(v1)) };
  }
  if (kind === "LINE") {
    const rec = t.record(curveId); // LINE(name, pnt#, vector#)
    const vec = t.record(ref(rec.params[2]!));
    const dir = readDirection(t, ref(vec.params[1]!));
    return { kind: "line", dir, length: dist(v0, v1) };
  }
  return { kind: "other" };
}

/**
 * Adaptively sample a Curve3 over [ta, tb] (defaults to its whole domain): start uniform, then
 * bisect any segment whose midpoint sags more than chordTol off the chord or that is longer than
 * maxSegLen. Returns ≥ 2 points, capped to stay bounded on pathological input.
 */
export function sampleCurve(c: Curve3, chordTol: number, maxSegLen: number, ta = c.t0, tb = c.t1, normalDev = Math.PI): Vec3[] {
  const ts: number[] = [];
  const n0 = c.closed || c.kind === "TRIMMED_CURVE" ? 8 : 2;
  for (let i = 0; i <= n0; i++) ts.push(ta + ((tb - ta) * i) / n0);
  const pts = ts.map((t) => c.evaluate(t));
  const maxPts = 4000;
  const cosDev = Math.cos(Math.min(normalDev, Math.PI));
  for (let i = 0; i + 1 < ts.length && ts.length < maxPts; ) {
    const tm = (ts[i]! + ts[i + 1]!) / 2;
    const pm = c.evaluate(tm);
    const a = pts[i]!, b = pts[i + 1]!;
    const chord = dist(a, b);
    const mid = lerp(a, b, 0.5);
    const sag = dist(pm, mid);
    // Angular criterion: the two half-chords turn by ~half the segment's arc angle, so a turn
    // above normalDev means the segment spans more than 2·normalDev of arc — split it.
    const la = dist(a, pm), lb = dist(pm, b);
    const turnDot = la > 1e-9 && lb > 1e-9
      ? ((pm[0] - a[0]) * (b[0] - pm[0]) + (pm[1] - a[1]) * (b[1] - pm[1]) + (pm[2] - a[2]) * (b[2] - pm[2])) / (la * lb)
      : 1;
    if ((sag > chordTol || chord > maxSegLen || turnDot < cosDev) && la > 1e-9 && lb > 1e-9) {
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

function arcSegments(radius: number, sweepAbs: number, chordTol: number, normalDev = Math.PI): number {
  const r = Math.max(radius, 1e-9);
  // Chord-sag limit AND the angular limit: adjacent facet normals differ by the segment's full arc
  // angle, so a normal deviation of normalDev allows at most 2·normalDev of arc per segment.
  const dChord = 2 * Math.acos(Math.max(0, Math.min(1, 1 - chordTol / r)));
  const dTheta = Math.min(dChord, 2 * normalDev);
  const seg = dTheta > 1e-6 ? Math.ceil(sweepAbs / dTheta) : 1;
  return Math.max(1, Math.min(4000, seg));
}

/**
 * Douglas–Peucker polyline simplification that also honours a maximum segment length: a span whose
 * points all sit within ε of its chord still gets split at its farthest point while the chord is
 * longer than maxSeg, so downstream consumers keep their ~targetEdge boundary spacing. Endpoints
 * are always kept.
 */
function dpSimplify(pts: Vec3[], eps: number, maxSeg: number): Vec3[] {
  if (pts.length <= 2) return pts;
  const keep = new Uint8Array(pts.length);
  keep[0] = keep[pts.length - 1] = 1;
  const stack: [number, number][] = [[0, pts.length - 1]];
  while (stack.length) {
    const [a, b] = stack.pop()!;
    if (b - a < 2) continue;
    const pa = pts[a]!, pb = pts[b]!;
    const ex = pb[0] - pa[0], ey = pb[1] - pa[1], ez = pb[2] - pa[2];
    const l2 = ex * ex + ey * ey + ez * ez;
    let imax = -1, dmax = -1;
    for (let i = a + 1; i < b; i++) {
      const p = pts[i]!;
      let w = l2 > 0 ? ((p[0] - pa[0]) * ex + (p[1] - pa[1]) * ey + (p[2] - pa[2]) * ez) / l2 : 0;
      w = w < 0 ? 0 : w > 1 ? 1 : w;
      const dx = p[0] - pa[0] - w * ex, dy = p[1] - pa[1] - w * ey, dz = p[2] - pa[2] - w * ez;
      const d = dx * dx + dy * dy + dz * dz;
      if (d > dmax) { dmax = d; imax = i; }
    }
    if (imax < 0) continue;
    if (Math.sqrt(dmax) > eps || Math.sqrt(l2) > maxSeg) {
      keep[imax] = 1;
      stack.push([a, imax], [imax, b]);
    }
  }
  const out: Vec3[] = [];
  for (let i = 0; i < pts.length; i++) if (keep[i]) out.push(pts[i]!);
  return out;
}

/**
 * Collapse ISOLATED short segments left by knot seeding / bisection parity: a polyline vertex whose
 * incident segment is far shorter than its neighbours' and whose removal deviates less than the
 * FULL chord tolerance is sampling noise, not geometry. DP alone keeps such vertices (its ε is
 * chordTol/2, and on a curved edge each point "earns" that much), but downstream the CDT's
 * boundary-graded size field reads the short segment's length as a local feature size and crowds a
 * starburst of micro-triangles around the spot — on an edge that shades perfectly smooth.
 * Endpoints are always kept, so shared-edge welding is unaffected.
 */
function mergeShortSegs(pts: Vec3[], chordTol: number, maxSeg: number): Vec3[] {
  const devDrop = (a: Vec3, p: Vec3, b: Vec3): number => {
    const ex = b[0] - a[0], ey = b[1] - a[1], ez = b[2] - a[2];
    const l2 = ex * ex + ey * ey + ez * ez;
    let w = l2 > 0 ? ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey + (p[2] - a[2]) * ez) / l2 : 0;
    w = w < 0 ? 0 : w > 1 ? 1 : w;
    return Math.hypot(p[0] - a[0] - w * ex, p[1] - a[1] - w * ey, p[2] - a[2] - w * ez);
  };
  for (let pass = 0; pass < 8 && pts.length > 2; pass++) {
    let changed = false;
    const out: Vec3[] = [pts[0]!];
    for (let i = 1; i < pts.length - 1; i++) {
      const prev = out[out.length - 1]!, cur = pts[i]!, next = pts[i + 1]!;
      const lIn = dist(prev, cur), lOut = dist(cur, next);
      const short = Math.min(lIn, lOut), long = Math.max(lIn, lOut);
      if (short < 0.5 * long
        && devDrop(prev, cur, next) <= chordTol
        && dist(prev, next) <= maxSeg) {
        // Drop cur, and keep its successor unconditionally this pass: back-to-back drops would
        // stack deviations that were each only checked against the pre-drop neighbours.
        changed = true;
        if (i + 1 < pts.length - 1) { out.push(next); i++; }
        continue;
      }
      out.push(cur);
    }
    out.push(pts[pts.length - 1]!);
    pts = out;
    if (!changed) break;
  }
  return pts;
}

/**
 * Cut an OPEN sampled polyline down to the sub-arc between an edge's vertices. Some exporters share
 * ONE curve between several EDGE_CURVEs (Shapr3D: a rim's 0.3mm closing sliver and its 5mm dip are
 * arcs of the same open B-spline), so a whole-domain polyline with its endpoints snapped to v0/v1
 * RETRACES the neighbouring edges and teleports back — the CDT then faithfully double-covers that
 * region. Locate both vertices by nearest-segment projection and keep only the span between them,
 * reversed when the edge runs against the curve's parametrisation. Returns v0..v1 inclusive.
 */
function cutOpenPolyline(pts: Vec3[], v0: Vec3, v1: Vec3): Vec3[] | null {
  const pos = (q: Vec3): number => {
    let best = 0, bd = Infinity;
    for (let i = 0; i + 1 < pts.length; i++) {
      const a = pts[i]!, b = pts[i + 1]!;
      const ex = b[0] - a[0], ey = b[1] - a[1], ez = b[2] - a[2];
      const l2 = ex * ex + ey * ey + ez * ez;
      let w = l2 > 0 ? ((q[0] - a[0]) * ex + (q[1] - a[1]) * ey + (q[2] - a[2]) * ez) / l2 : 0;
      w = w < 0 ? 0 : w > 1 ? 1 : w;
      const dx = q[0] - a[0] - w * ex, dy = q[1] - a[1] - w * ey, dz = q[2] - a[2] - w * ez;
      const d = dx * dx + dy * dy + dz * dz;
      if (d < bd) { bd = d; best = i + w; }
    }
    return best;
  };
  const p0 = pos(v0), p1 = pos(v1);
  const lo = Math.min(p0, p1), hi = Math.max(p0, p1);
  const out: Vec3[] = [p0 <= p1 ? v0 : v1];
  for (let i = Math.floor(lo) + 1; i <= Math.floor(hi); i++) {
    if (i > lo && i < hi && dist(pts[i]!, out[out.length - 1]!) > 1e-9) out.push(pts[i]!);
  }
  const tail = p0 <= p1 ? v1 : v0;
  if (dist(tail, out[out.length - 1]!) > 1e-9) out.push(tail);
  else out[out.length - 1] = tail;
  if (p0 > p1) out.reverse();
  return out.length >= 2 ? out : null;
}

/**
 * Sample the curve of an EDGE_CURVE from vertex v0 to v1.
 * `sameSense` is the EDGE_CURVE flag (edge direction agrees with the curve's parametrisation).
 * Returns points from v0 to v1 inclusive (≥ 2 points).
 */
export function sampleEdgePolyline(
  t: Table, curveId: number, v0: Vec3, v1: Vec3, sameSense: boolean,
  s: number, chordTol: number, maxSegLen: number, aRad = 1, normalDev = Math.PI,
): Vec3[] {
  const kind = t.typeOf(curveId); // undefined for complex entities (surface/seam curves)

  // SURFACE_CURVE / SEAM_CURVE / INTERSECTION_CURVE(name, curve_3d#, (pcurves), rep) wrap their true
  // geometry in the referenced 3D curve — dispatch on THAT so wrapped conics reach the arc samplers
  // and wrapped LINEs the straight chord. Falling through to the generic path below is wrong for a
  // wrapped LINE: LineCurve's domain is the anchor segment [o, o+d], unrelated to this edge's span,
  // so endpoint snapping strands the interior samples there (legacy AP203 exporters wrap EVERY edge
  // in INTERSECTION_CURVE — model-spanning sliver triangles on 2827056.stp).
  if (kind === "SURFACE_CURVE" || kind === "SEAM_CURVE" || kind === "INTERSECTION_CURVE") {
    return sampleEdgePolyline(t, ref(t.record(curveId).params[1]!), v0, v1, sameSense, s, chordTol, maxSegLen, aRad, normalDev);
  }

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
    // Closed-rim detection. The chordTol term is CAPPED at its default-tolerance value (0.01mm
    // deviation -> 1e-5): a coarse conversion must not reinterpret a jitter-split micro arc
    // (endpoints ~2e-5 apart) as a full circle and walk the whole rim into one face's boundary.
    const full = dist(v0, v1) < Math.max(1e-7, Math.min(1e-5, chordTol * 1e-3));
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
    const n = Math.max(arcSegments(R, Math.abs(sweep), chordTol, normalDev), Math.ceil(arcLen / Math.max(maxSegLen, 1e-9)));
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
    const full = dist(v0, v1) < Math.max(1e-7, Math.min(1e-5, chordTol * 1e-3)); // capped like CIRCLE above
    let sweep: number;
    if (full) sweep = sameSense ? TWO_PI : -TWO_PI;
    else if (sameSense) { let d = t1 - t0; while (d <= 0) d += TWO_PI; while (d > TWO_PI) d -= TWO_PI; sweep = d; }
    else { let d = t1 - t0; while (d >= 0) d -= TWO_PI; while (d < -TWO_PI) d += TWO_PI; sweep = d; }
    // Tightest curvature radius (minSemi²/maxSemi) bounds the chord deviation everywhere; arc length
    // ~ mean-radius·sweep bounds the segment length. Uniform-in-t sampling is fine for both.
    // The normal turns at up to max/min radians per eccentric-angle radian (fastest at the tight
    // ends), so shrink the angular tolerance by min/max to bound the turn per uniform-in-t step.
    const rMin = Math.min(a, b) ** 2 / Math.max(a, b, 1e-9);
    const devT = normalDev * Math.min(a, b) / Math.max(a, b, 1e-9);
    const n = Math.max(arcSegments(rMin, Math.abs(sweep), chordTol, devT), Math.ceil((Math.abs(sweep) * (a + b) / 2) / Math.max(maxSegLen, 1e-9)));
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
  // A CLOSED ring B-spline (a full circle exported as one periodic curve) is excluded: an edge may
  // cover only an ARC of it (e.g. two half-rim edges sharing one ring curve, as Shapr3D emits), and
  // sampling the whole knot domain would hand that edge the ENTIRE ring with the endpoints snapped
  // to its vertices — a boundary that walks the whole rim and teleports back. The generic sampler
  // below cuts the ring at the edge's vertices instead.
  const bs = bsplineData(t, curveId, s);
  if (bs && dist(deBoor(bs.degree, bs.cps, bs.knots, bs.u0, bs.weights), deBoor(bs.degree, bs.cps, bs.knots, bs.u1, bs.weights)) >= 1e-6) {
    const { degree, cps, knots, weights, u0, u1 } = bs;
    let clen = 0;
    for (let i = 1; i < cps.length; i++) clen += dist(cps[i - 1]!, cps[i]!);
    const n = Math.max(2, Math.ceil(clen / Math.max(maxSegLen, 1e-9)));
    // Seed uniform-in-u plus the distinct interior knots (a kink at a full-multiplicity knot can
    // sit exactly between probe midpoints and hide), then bisect any span whose midpoint sags off
    // its chord by more than chordTol. The control polygon bounds LENGTH but says nothing about
    // curvature — the old length-only count left small intersection arcs (OpenVessel's r=1.5mm
    // porthole rims through the B-spline hull) as ~10-segment polygons no matter how fine a
    // deviation the caller requested.
    const us: number[] = [];
    for (let i = 0; i <= n; i++) us.push(u0 + ((u1 - u0) * i) / n);
    const span = (u1 - u0) || 1e-12;
    for (const k of knots) if (k > u0 + 1e-9 * span && k < u1 - 1e-9 * span) us.push(k);
    us.sort((a, b) => a - b);
    const ts: number[] = [];
    for (const u of us) if (ts.length === 0 || u - ts[ts.length - 1]! > 1e-12 * span) ts.push(u);
    let pts: Vec3[] = ts.map((u) => deBoor(degree, cps, knots, u, weights));
    const maxPts = 4000;
    const cosDev = Math.cos(Math.min(normalDev, Math.PI));
    // Sampling floor: never place samples closer than this in 3D. Exporter artifacts (parametric
    // speed collapse, sub-µm curvature spikes on hull blend rails — OpenVessel's stem) otherwise
    // pile points into a micro-cluster; the CDT then fans every ~targetEdge interior vertex into
    // it, which looks far worse than the ≤ floor/4 chord deviation the merge leaves at the spot.
    const minSeg = Math.min(maxSegLen / 4, Math.max(4 * chordTol, 1e-4));
    for (let i = 0; i + 1 < ts.length && ts.length < maxPts; ) {
      const um = (ts[i]! + ts[i + 1]!) / 2;
      const pm = deBoor(degree, cps, knots, um, weights);
      const a = pts[i]!, b = pts[i + 1]!;
      const sag = dist(pm, lerp(a, b, 0.5));
      const la = dist(a, pm), lb = dist(pm, b);
      const turnDot = la > 1e-9 && lb > 1e-9
        ? ((pm[0] - a[0]) * (b[0] - pm[0]) + (pm[1] - a[1]) * (b[1] - pm[1]) + (pm[2] - a[2]) * (b[2] - pm[2])) / (la * lb)
        : 1;
      if ((sag > chordTol || dist(a, b) > maxSegLen || turnDot < cosDev) && la > minSeg && lb > minSeg) {
        ts.splice(i + 1, 0, um); pts.splice(i + 1, 0, pm);
      } else i++;
    }
    // Douglas–Peucker sweep (ε = chordTol/2, maxSegLen preserved): exporter noise leaves runs of
    // floor-length segments around each spike (and parametric speed collapse bunches the uniform
    // seeds the same way); DP collapses every such trail to the one vertex that actually deviates,
    // so the CDT's boundary-graded size field doesn't read the trail as a fine feature and crowd
    // micro-triangles around it. Points a smooth arc needs stay: dropping one of them would leave
    // ~4·chordTol of sag, well over ε.
    pts = dpSimplify(pts, chordTol / 2, maxSegLen);
    pts = mergeShortSegs(pts, chordTol, maxSegLen);
    // FULL-CYCLE edge on a nearly-closed curve: the edge's own vertices coincide (one vertex, a
    // ring boundary) but the curve's seam gap — exporter noise, µm scale — exceeds the 1e-6
    // closed-ring gate above, so a v0..v1 cut would find both cut points at the same place and
    // collapse the whole ring to a 2-point polyline (ABC 00014671: an 18-dome family whose apex
    // rings all sampled to a single point, faces dropped). The edge demonstrably covers the
    // whole curve — return the full ring rotated to start at the vertex.
    if (dist(v0, v1) < Math.max(1e-7, Math.min(1e-5, chordTol * 1e-3)) && pts.length > 3) {
      let lo0 = Infinity, lo1 = Infinity, lo2 = Infinity, hi0 = -Infinity, hi1 = -Infinity, hi2 = -Infinity;
      for (const p of pts) {
        if (p[0] < lo0) lo0 = p[0]; if (p[0] > hi0) hi0 = p[0];
        if (p[1] < lo1) lo1 = p[1]; if (p[1] > hi1) hi1 = p[1];
        if (p[2] < lo2) lo2 = p[2]; if (p[2] > hi2) hi2 = p[2];
      }
      const extent = Math.hypot(hi0 - lo0, hi1 - lo1, hi2 - lo2);
      if (dist(pts[0]!, pts[pts.length - 1]!) < 0.05 * extent) {
        const ring = pts.slice(0, -1);
        if (!sameSense) ring.reverse();
        let i0 = 0, bd = Infinity;
        for (let i = 0; i < ring.length; i++) { const d = dist(ring[i]!, v0); if (d < bd) { bd = d; i0 = i; } }
        const out = [...ring.slice(i0), ...ring.slice(0, i0), v1];
        out[0] = v0;
        return out;
      }
    }
    // The edge may cover only PART of the curve (exporters share one curve between edges) — cut the
    // whole-domain polyline to the v0..v1 span instead of blindly snapping the domain endpoints.
    const cut = cutOpenPolyline(pts, v0, v1);
    if (cut) return cut;
  }

  // Any other curve type makeCurve understands (TRIMMED_CURVE, SURFACE_CURVE/SEAM_CURVE wrapping a
  // conic, ...): sample it adaptively instead of collapsing to a straight secant chord.
  if (kind !== "LINE") {
    const c = makeCurve(t, curveId, s, aRad);
    if (c) {
      let pts = mergeShortSegs(sampleCurve(c, chordTol, maxSegLen, c.t0, c.t1, normalDev), chordTol, maxSegLen);
      // Nearly-closed counts as closed when the EDGE itself is a full cycle (its vertices
      // coincide): a µm seam gap from exporter noise must not push a ring boundary into the
      // open-curve cut below, which would collapse it to nothing (see the B-spline block above).
      let ringClosed = pts.length > 3 && dist(pts[0]!, pts[pts.length - 1]!) < Math.max(1e-9, chordTol * 1e-3);
      if (!ringClosed && pts.length > 3 && dist(v0, v1) < Math.max(1e-7, Math.min(1e-5, chordTol * 1e-3))) {
        let lo0 = Infinity, lo1 = Infinity, lo2 = Infinity, hi0 = -Infinity, hi1 = -Infinity, hi2 = -Infinity;
        for (const p of pts) {
          if (p[0] < lo0) lo0 = p[0]; if (p[0] > hi0) hi0 = p[0];
          if (p[1] < lo1) lo1 = p[1]; if (p[1] > hi1) hi1 = p[1];
          if (p[2] < lo2) lo2 = p[2]; if (p[2] > hi2) hi2 = p[2];
        }
        ringClosed = dist(pts[0]!, pts[pts.length - 1]!) < 0.05 * Math.hypot(hi0 - lo0, hi1 - lo1, hi2 - lo2);
      }
      if (ringClosed) {
        // Closed curve (full circle/intersection ring): its seam start is unrelated to the edge's
        // vertices, so blind endpoint-snapping would fold the polyline back on itself and destroy
        // its winding. Orient by sameSense, then cut the ring at the vertices' nearest-SEGMENT
        // projections (for a full-period edge v0==v1: rotate so the cycle starts at the vertex).
        // Cutting at nearest SAMPLES is not enough: an edge covering an arc SHORTER than the ring's
        // sample spacing (three edges partition a rim, one is a 0.3mm sliver) can land its two cut
        // samples in the wrong order, and the forward walk then returns the entire ring MINUS the
        // sliver — a boundary that retraces its neighbour edge and teleports back, which the CDT
        // faithfully double-covers (OpenVessel's counterbore rims).
        const ring = pts.slice(0, -1);
        if (!sameSense) ring.reverse();
        const nearestSeg = (q: Vec3): { seg: number; t: number } => {
          let bs = 0, bt = 0, bd = Infinity;
          for (let i = 0; i < ring.length; i++) {
            const a = ring[i]!, b = ring[(i + 1) % ring.length]!;
            const ex = b[0] - a[0], ey = b[1] - a[1], ez = b[2] - a[2];
            const l2 = ex * ex + ey * ey + ez * ez;
            let w = l2 > 0 ? ((q[0] - a[0]) * ex + (q[1] - a[1]) * ey + (q[2] - a[2]) * ez) / l2 : 0;
            w = w < 0 ? 0 : w > 1 ? 1 : w;
            const dx = q[0] - a[0] - w * ex, dy = q[1] - a[1] - w * ey, dz = q[2] - a[2] - w * ez;
            const d = dx * dx + dy * dy + dz * dz;
            if (d < bd) { bd = d; bs = i; bt = w; }
          }
          return { seg: bs, t: bt };
        };
        if (dist(v0, v1) < Math.max(1e-7, chordTol * 1e-3)) {
          const i0 = nearestSeg(v0).seg;
          pts = [...ring.slice(i0), ...ring.slice(0, i0), v1];
          pts[0] = v0;
          return pts;
        }
        const s0 = nearestSeg(v0), s1 = nearestSeg(v1);
        const arc: Vec3[] = [v0];
        if (!(s0.seg === s1.seg && s0.t <= s1.t)) {
          // v1 is not ahead of v0 within the same segment: walk forward sample by sample, from the
          // sample after v0's projection up to the start sample of v1's segment (inclusive).
          for (let i = (s0.seg + 1) % ring.length; ; i = (i + 1) % ring.length) {
            const p = ring[i]!;
            if (dist(p, arc[arc.length - 1]!) > 1e-9) arc.push(p);
            if (i === s1.seg) break;
          }
        }
        if (dist(v1, arc[arc.length - 1]!) > 1e-9) arc.push(v1);
        else arc[arc.length - 1] = v1;
        if (arc.length >= 2) return arc;
      } else if (pts.length >= 2) {
        // Same shared-curve hazard as the B-spline path above: keep only the v0..v1 span.
        const cut = cutOpenPolyline(pts, v0, v1);
        if (cut) return cut;
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
