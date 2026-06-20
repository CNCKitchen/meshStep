// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — analytic surfaces with parametric evaluate / project / normal.
// Parametrisations follow ISO 10303-42. (No constructor parameter-properties: must stay
// erasable TypeScript so Node's native type-stripping can run these files.)
import type { Vec3 } from "./vec.ts";
import { add, cross, dot, normalize, scale, sub } from "./vec.ts";
import { readPlacement, readPoint, type Frame } from "./placement.ts";
import { Table, num, ref, list, numList } from "../step/entities.ts";

export interface Surface {
  kind: string;
  /** True if the surface wraps in u — cylinder, cone, sphere, closed B-spline. */
  periodicU: boolean;
  /** True if the surface also wraps in v — torus (tube angle), closed B-spline. */
  periodicV?: boolean;
  /** Parameter period in u / v (2π for the analytic surfaces; v1-v0 for a closed B-spline). */
  uPeriod?: number;
  vPeriod?: number;
  evaluate(u: number, v: number): Vec3;
  /** Inverse-map p to (u,v). Optional (hu,hv) seeds an iterative solver so a boundary projects
   *  continuously across a seam (used by the B-spline surface; ignored by the analytic ones). */
  project(p: Vec3, hu?: number, hv?: number): [number, number];
  normal(u: number, v: number): Vec3;
  /** Smallest principal radius of curvature at (u,v); Infinity for flat. Drives adaptive refinement. */
  curvatureRadius(u: number, v: number): number;
}

class Plane implements Surface {
  kind = "PLANE";
  periodicU = false;
  f: Frame;
  constructor(f: Frame) { this.f = f; }
  evaluate(u: number, v: number): Vec3 {
    return add(this.f.o, add(scale(this.f.x, u), scale(this.f.y, v)));
  }
  project(p: Vec3): [number, number] {
    const d = sub(p, this.f.o);
    return [dot(d, this.f.x), dot(d, this.f.y)];
  }
  normal(): Vec3 { return this.f.z; }
  curvatureRadius(): number { return Infinity; }
}

class Cylinder implements Surface {
  kind = "CYLINDRICAL_SURFACE";
  periodicU = true;
  f: Frame;
  r: number;
  constructor(f: Frame, r: number) { this.f = f; this.r = r; }
  evaluate(u: number, v: number): Vec3 {
    const radial = add(scale(this.f.x, Math.cos(u) * this.r), scale(this.f.y, Math.sin(u) * this.r));
    return add(this.f.o, add(radial, scale(this.f.z, v)));
  }
  project(p: Vec3): [number, number] {
    const d = sub(p, this.f.o);
    return [Math.atan2(dot(d, this.f.y), dot(d, this.f.x)), dot(d, this.f.z)];
  }
  normal(u: number): Vec3 {
    return add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
  }
  curvatureRadius(): number { return this.r; } // tight in u, flat in v
}

class Cone implements Surface {
  kind = "CONICAL_SURFACE";
  periodicU = true;
  f: Frame;
  r: number;
  sin: number;
  cos: number;
  constructor(f: Frame, r: number, semiAngle: number) {
    this.f = f; this.r = r;
    this.sin = Math.sin(semiAngle);
    this.cos = Math.cos(semiAngle);
  }
  evaluate(u: number, v: number): Vec3 {
    const rad = this.r + v * this.sin;
    const radial = add(scale(this.f.x, Math.cos(u) * rad), scale(this.f.y, Math.sin(u) * rad));
    return add(this.f.o, add(radial, scale(this.f.z, v * this.cos)));
  }
  project(p: Vec3): [number, number] {
    const d = sub(p, this.f.o);
    const v = this.cos !== 0 ? dot(d, this.f.z) / this.cos : 0;
    return [Math.atan2(dot(d, this.f.y), dot(d, this.f.x)), v];
  }
  normal(u: number): Vec3 {
    const radial = add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
    return normalize(sub(scale(radial, this.cos), scale(this.f.z, this.sin)));
  }
  curvatureRadius(_u: number, v: number): number {
    return Math.max(1e-3, Math.abs(this.r + v * this.sin) / Math.max(this.cos, 1e-3));
  }
}

class Sphere implements Surface {
  kind = "SPHERICAL_SURFACE";
  periodicU = true;
  f: Frame;
  r: number;
  constructor(f: Frame, r: number) { this.f = f; this.r = r; }
  evaluate(u: number, v: number): Vec3 {
    const ring = add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
    const dir = add(scale(ring, Math.cos(v)), scale(this.f.z, Math.sin(v)));
    return add(this.f.o, scale(dir, this.r));
  }
  project(p: Vec3): [number, number] {
    const d = sub(p, this.f.o);
    const v = Math.asin(Math.max(-1, Math.min(1, dot(d, this.f.z) / this.r)));
    return [Math.atan2(dot(d, this.f.y), dot(d, this.f.x)), v];
  }
  normal(u: number, v: number): Vec3 {
    return normalize(sub(this.evaluate(u, v), this.f.o));
  }
  curvatureRadius(): number { return this.r; }
}

class Torus implements Surface {
  kind = "TOROIDAL_SURFACE";
  periodicU = true;
  periodicV = true;
  f: Frame;
  rMaj: number;
  rMin: number;
  constructor(f: Frame, rMaj: number, rMin: number) { this.f = f; this.rMaj = rMaj; this.rMin = rMin; }
  evaluate(u: number, v: number): Vec3 {
    const ring = add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
    const radial = scale(ring, this.rMaj + this.rMin * Math.cos(v));
    return add(this.f.o, add(radial, scale(this.f.z, this.rMin * Math.sin(v))));
  }
  project(p: Vec3): [number, number] {
    const d = sub(p, this.f.o);
    const lx = dot(d, this.f.x), ly = dot(d, this.f.y), lz = dot(d, this.f.z);
    return [Math.atan2(ly, lx), Math.atan2(lz, Math.hypot(lx, ly) - this.rMaj)];
  }
  normal(u: number, v: number): Vec3 {
    const ring = add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
    return normalize(add(scale(ring, Math.cos(v)), scale(this.f.z, Math.sin(v))));
  }
  curvatureRadius(): number { return this.rMin; } // tube radius dominates
}

// ---- B-spline (NURBS) surface --------------------------------------------------------------
// Tensor-product de Boor on a single control row of homogeneous (wx,wy,wz,w) 4-vectors.
function deBoorH(degree: number, ctrl: number[][], knots: number[], u: number): number[] {
  const n = ctrl.length - 1;
  let k = degree;
  while (k < n && knots[k + 1]! <= u) k++;
  const d: number[][] = [];
  for (let j = 0; j <= degree; j++) d[j] = ctrl[k - degree + j]!.slice();
  for (let r = 1; r <= degree; r++) {
    for (let j = degree; j >= r; j--) {
      const i = k - degree + j;
      const lo = knots[i]!, hi = knots[i + degree - r + 1]!;
      const a = hi > lo ? (u - lo) / (hi - lo) : 0;
      const p = d[j - 1]!, q = d[j]!;
      d[j] = [p[0]! + (q[0]! - p[0]!) * a, p[1]! + (q[1]! - p[1]!) * a, p[2]! + (q[2]! - p[2]!) * a, p[3]! + (q[3]! - p[3]!) * a];
    }
  }
  return d[degree]!;
}

function expandKnots(mults: number[], vals: number[]): number[] {
  const k: number[] = [];
  for (let i = 0; i < vals.length; i++) for (let j = 0; j < (mults[i] ?? 0); j++) k.push(vals[i]!);
  return k;
}

class BSplineSurface implements Surface {
  kind = "B_SPLINE_SURFACE";
  periodicU = false;
  periodicV = false;
  uPeriod = 0; vPeriod = 0;
  uDeg: number; vDeg: number;
  cps: number[][][]; // [iu][iv] -> homogeneous (wx,wy,wz,w)
  uKnots: number[]; vKnots: number[];
  u0: number; u1: number; v0: number; v1: number;
  // projection lookup grid
  gU: number[] = []; gV: number[] = []; gP: Vec3[][] = [];
  rc = Infinity;
  closedU = false; closedV = false; // patch wraps onto itself in u / v
  constructor(uDeg: number, vDeg: number, cps: number[][][], uKnots: number[], vKnots: number[]) {
    this.uDeg = uDeg; this.vDeg = vDeg; this.cps = cps; this.uKnots = uKnots; this.vKnots = vKnots;
    const nu = cps.length - 1, nv = cps[0]!.length - 1;
    this.u0 = uKnots[uDeg]!; this.u1 = uKnots[nu + 1]!;
    this.v0 = vKnots[vDeg]!; this.v1 = vKnots[nv + 1]!;
    const um = (this.u0 + this.u1) / 2, vm = (this.v0 + this.v1) / 2;
    const gap = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    this.closedU = gap(this.evaluateRaw(this.u0, vm), this.evaluateRaw(this.u1, vm)) < 1e-4;
    this.closedV = gap(this.evaluateRaw(um, this.v0), this.evaluateRaw(um, this.v1)) < 1e-4;
    // A closed patch is periodic in that direction with period = its parameter span. Trimmed faces
    // then project their boundary continuously across the seam (hint-seeded) so the param-space
    // polygon stays simple and the CDT can enforce it — otherwise the seam tangles -> cracks.
    this.periodicU = this.closedU; this.periodicV = this.closedV;
    this.uPeriod = this.u1 - this.u0; this.vPeriod = this.v1 - this.v0;
    this.buildGrid();
  }
  private buildGrid(): void {
    const GU = 24, GV = 24;
    for (let i = 0; i <= GU; i++) this.gU.push(this.u0 + ((this.u1 - this.u0) * i) / GU);
    for (let j = 0; j <= GV; j++) this.gV.push(this.v0 + ((this.v1 - this.v0) * j) / GV);
    for (let i = 0; i <= GU; i++) {
      const row: Vec3[] = [];
      for (let j = 0; j <= GV; j++) row.push(this.evaluate(this.gU[i]!, this.gV[j]!));
      this.gP.push(row);
    }
    // Rough min curvature radius from normal turn across the grid (drives adaptive refinement).
    let minR = Infinity;
    for (let i = 0; i < GU; i++) for (let j = 0; j < GV; j++) {
      const a = this.gP[i]![j]!, b = this.gP[i + 1]![j]!, c = this.gP[i]![j + 1]!;
      const n1 = this.normalAt(this.gU[i]!, this.gV[j]!), n2 = this.normalAt(this.gU[i + 1]!, this.gV[j]!), n3 = this.normalAt(this.gU[i]!, this.gV[j + 1]!);
      const du = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
      const dv = Math.hypot(c[0] - a[0], c[1] - a[1], c[2] - a[2]);
      const tu = Math.acos(Math.max(-1, Math.min(1, dot(n1, n2))));
      const tv = Math.acos(Math.max(-1, Math.min(1, dot(n1, n3))));
      if (tu > 1e-4 && du > 1e-9) minR = Math.min(minR, du / tu);
      if (tv > 1e-4 && dv > 1e-9) minR = Math.min(minR, dv / tv);
    }
    this.rc = Number.isFinite(minR) ? Math.max(0.05, minR) : Infinity;
  }
  private evaluateRaw(u: number, v: number): Vec3 {
    const temp: number[][] = [];
    for (let i = 0; i < this.cps.length; i++) temp.push(deBoorH(this.vDeg, this.cps[i]!, this.vKnots, v));
    const h = deBoorH(this.uDeg, temp, this.uKnots, u);
    const w = h[3]! || 1;
    return [h[0]! / w, h[1]! / w, h[2]! / w];
  }
  evaluate(u: number, v: number): Vec3 {
    // Wrap a seam-unwrapped coordinate back into the patch domain for a closed direction (same point
    // by periodicity); de Boor extrapolates nonsense outside the knot span otherwise.
    if (this.closedU && this.uPeriod > 0) u = this.u0 + (((u - this.u0) % this.uPeriod) + this.uPeriod) % this.uPeriod;
    if (this.closedV && this.vPeriod > 0) v = this.v0 + (((v - this.v0) % this.vPeriod) + this.vPeriod) % this.vPeriod;
    return this.evaluateRaw(u, v);
  }
  private normalAt(u: number, v: number): Vec3 {
    const e = 1e-4 * Math.max(1, this.u1 - this.u0);
    const ev = 1e-4 * Math.max(1, this.v1 - this.v0);
    const cu = Math.min(Math.max(u, this.u0 + e), this.u1 - e);
    const cv = Math.min(Math.max(v, this.v0 + ev), this.v1 - ev);
    const pu1 = this.evaluate(cu + e, cv), pu2 = this.evaluate(cu - e, cv);
    const pv1 = this.evaluate(cu, cv + ev), pv2 = this.evaluate(cu, cv - ev);
    const du: Vec3 = [pu1[0] - pu2[0], pu1[1] - pu2[1], pu1[2] - pu2[2]];
    const dv: Vec3 = [pv1[0] - pv2[0], pv1[1] - pv2[1], pv1[2] - pv2[2]];
    const n = cross(du, dv);
    const len = Math.hypot(n[0], n[1], n[2]);
    return len > 1e-12 ? [n[0] / len, n[1] / len, n[2] / len] : [0, 0, 1];
  }
  normal(u: number, v: number): Vec3 { return this.normalAt(u, v); }
  curvatureRadius(): number { return this.rc; }
  project(p: Vec3, hu?: number, hv?: number): [number, number] {
    // Seed from the caller's hint (continuous boundary projection) or the nearest grid node, then a
    // few Gauss-Newton steps minimising |S(u,v)-p|². In a CLOSED direction the coordinate is NOT
    // clamped — it follows p across the seam (evaluate() wraps it back), so a loop crossing the seam
    // stays monotone in param instead of snapping between v0 and v1.
    let u: number, v: number;
    if (hu !== undefined && hv !== undefined) { u = hu; v = hv; }
    else {
      let bu = this.u0, bv = this.v0, bd = Infinity;
      for (let i = 0; i < this.gP.length; i++) for (let j = 0; j < this.gP[i]!.length; j++) {
        const q = this.gP[i]![j]!;
        const d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2;
        if (d < bd) { bd = d; bu = this.gU[i]!; bv = this.gV[j]!; }
      }
      u = bu; v = bv;
    }
    const eu = 1e-5 * (this.u1 - this.u0), ev = 1e-5 * (this.v1 - this.v0);
    const clampU = (x: number): number => this.closedU ? x : Math.min(Math.max(x, this.u0), this.u1);
    const clampV = (x: number): number => this.closedV ? x : Math.min(Math.max(x, this.v0), this.v1);
    for (let it = 0; it < 12; it++) {
      const S = this.evaluate(u, v);
      const r: Vec3 = [S[0] - p[0], S[1] - p[1], S[2] - p[2]];
      // Central difference; in an OPEN direction keep the samples inside the knot span (de Boor
      // extrapolates nonsense past it), in a CLOSED direction roam freely (evaluate wraps the seam).
      const up = this.closedU ? u + eu : Math.min(u + eu, this.u1), um = this.closedU ? u - eu : Math.max(u - eu, this.u0);
      const vp = this.closedV ? v + ev : Math.min(v + ev, this.v1), vm = this.closedV ? v - ev : Math.max(v - ev, this.v0);
      const Su0 = this.evaluate(up, v), Su1 = this.evaluate(um, v);
      const Sv0 = this.evaluate(u, vp), Sv1 = this.evaluate(u, vm);
      const idu = 1 / ((up - um) || 1e-12), idv = 1 / ((vp - vm) || 1e-12);
      const Su: Vec3 = [(Su0[0] - Su1[0]) * idu, (Su0[1] - Su1[1]) * idu, (Su0[2] - Su1[2]) * idu];
      const Sv: Vec3 = [(Sv0[0] - Sv1[0]) * idv, (Sv0[1] - Sv1[1]) * idv, (Sv0[2] - Sv1[2]) * idv];
      const a = dot(Su, Su), b = dot(Su, Sv), c = dot(Sv, Sv);
      const g1 = dot(Su, r), g2 = dot(Sv, r);
      const det = a * c - b * b;
      if (Math.abs(det) < 1e-18) break;
      const du = -(c * g1 - b * g2) / det, dv = -(a * g2 - b * g1) / det;
      u = clampU(u + du); v = clampV(v + dv);
      if (Math.abs(du) < eu && Math.abs(dv) < ev) break;
    }
    return [u, v];
  }
}

/** Read a control-point grid + degrees/knots from simple or complex (rational) B-spline surface. */
function makeBSplineSurface(t: Table, id: number, s: number): BSplineSurface | null {
  const base = t.sub(id, "B_SPLINE_SURFACE") ?? t.sub(id, "B_SPLINE_SURFACE_WITH_KNOTS");
  const wk = t.sub(id, "B_SPLINE_SURFACE_WITH_KNOTS");
  if (!base || !wk) return null;
  // Simple form: all fields in one record. Complex form: degrees+cps in B_SPLINE_SURFACE,
  // mults+knots in B_SPLINE_SURFACE_WITH_KNOTS (which then starts at u_mult), weights in RATIONAL_*.
  const simple = base === wk;
  const off = simple ? 1 : 0; // simple form has a leading name param; complex partial does not
  const uDeg = num(base.params[off]!), vDeg = num(base.params[off + 1]!);
  const cpRefs = list(base.params[off + 2]!).map((row) => list(row).map(ref)); // [iu][iv] -> point id
  const km = simple ? 8 : 0; // index where u_multiplicities starts
  const uMult = numList(wk.params[km]!), vMult = numList(wk.params[km + 1]!);
  const uVals = numList(wk.params[km + 2]!), vVals = numList(wk.params[km + 3]!);
  const rat = t.sub(id, "RATIONAL_B_SPLINE_SURFACE");
  const weights = rat ? list(rat.params[0]!).map((row) => numList(row)) : null;
  const cps: number[][][] = cpRefs.map((row, iu) => row.map((pid, iv) => {
    const pt = readPoint(t, pid, s);
    const w = weights ? weights[iu]![iv]! : 1;
    return [pt[0] * w, pt[1] * w, pt[2] * w, w];
  }));
  return new BSplineSurface(uDeg, vDeg, cps, expandKnots(uMult, uVals), expandKnots(vMult, vVals));
}

/** Construct a Surface from a STEP surface entity; returns null for unsupported. */
export function makeSurface(t: Table, id: number, s: number): Surface | null {
  const kind = t.typeOf(id);
  if (!kind) return makeBSplineSurface(t, id, s); // complex (rational B-spline) surface
  const r = t.record(id);
  switch (kind) {
    case "B_SPLINE_SURFACE_WITH_KNOTS":
      return makeBSplineSurface(t, id, s);
    case "PLANE":
      return new Plane(readPlacement(t, ref(r.params[1]!), s));
    case "CYLINDRICAL_SURFACE":
      return new Cylinder(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s);
    case "CONICAL_SURFACE":
      return new Cone(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s, num(r.params[3]!));
    case "SPHERICAL_SURFACE":
      return new Sphere(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s);
    case "TOROIDAL_SURFACE":
      return new Torus(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s, num(r.params[3]!) * s);
    default:
      return null; // swept / b-spline — phase 2
  }
}

export { Sphere, BSplineSurface };
export const isSphere = (s: Surface): s is Sphere => s.kind === "SPHERICAL_SURFACE";
export const isBSpline = (s: Surface): s is BSplineSurface => s.kind === "B_SPLINE_SURFACE";
