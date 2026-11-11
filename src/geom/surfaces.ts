// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — analytic surfaces with parametric evaluate / project / normal.
// Parametrisations follow ISO 10303-42. (No constructor parameter-properties: must stay
// erasable TypeScript so Node's native type-stripping can run these files.)
import type { Vec3 } from "./vec.ts";
import { add, cross, dot, normalize, scale, sub } from "./vec.ts";
import { readPlacement, readPoint, readDirection, type Frame } from "./placement.ts";
import { Table, num, ref, list, numList } from "../step/entities.ts";
import { makeCurve, CurveRadiusTable, type Curve3 } from "./curves.ts";

export interface Surface {
  kind: string;
  /** True if the surface wraps in u — cylinder, cone, sphere, closed B-spline. */
  periodicU: boolean;
  /** True if the surface also wraps in v — torus (tube angle), closed B-spline. */
  periodicV?: boolean;
  /** Parameter period in u / v (2π for the analytic surfaces; v1-v0 for a closed B-spline). */
  uPeriod?: number;
  vPeriod?: number;
  /** Parameter value of the periodic branch cut ("seam") in u / v: where project() wraps — ±π for
   *  the analytic atan2 surfaces, the knot-domain start for a closed B-spline / extrusion. The
   *  seam-aware boundary projector needs it to spot edges that lie ON the seam (ambiguous side). */
  uSeam?: number;
  vSeam?: number;
  /** Bounded parameter domain per direction, when the surface has one (B-spline knot span, SoR
   *  profile span). Absent on unbounded analytic directions (plane axes, cylinder/cone height). */
  uDomain?: [number, number];
  vDomain?: [number, number];
  evaluate(u: number, v: number): Vec3;
  /** Inverse-map p to (u,v). Optional (hu,hv) seeds an iterative solver so a boundary projects
   *  continuously across a seam (used by the B-spline surface; ignored by the analytic ones). */
  project(p: Vec3, hu?: number, hv?: number): [number, number];
  normal(u: number, v: number): Vec3;
  /** Smallest principal radius of curvature at (u,v); Infinity for flat. Drives adaptive refinement. */
  curvatureRadius(u: number, v: number): number;
  /** SWEPT surfaces only (extrusion, revolution, offsets of them): their curvatureRadius() follows
   *  the LOCAL radius of a profile curve, which varies across a face. This is the PROFILE's
   *  tightest radius over the parameter box (profile curvature only — the other, always-local terms
   *  such as a revolution's parallel circle are not included), for whole-face density decisions. */
  spanRadius?(u0: number, u1: number, v0: number, v1: number): number;
  /** With spanRadius: the parameter direction (0 = u, 1 = v) along which the profile curves. */
  profileAxis?: 0 | 1;
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
  uSeam = Math.PI;
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
  uSeam = Math.PI;
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
  uSeam = Math.PI;
  f: Frame;
  r: number;
  constructor(f: Frame, r: number) { this.f = f; this.r = r; }
  evaluate(u: number, v: number): Vec3 {
    const ring = add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
    const dir = add(scale(ring, Math.cos(v)), scale(this.f.z, Math.sin(v)));
    return add(this.f.o, scale(dir, this.r));
  }
  project(p: Vec3, hu?: number): [number, number] {
    const d = sub(p, this.f.o);
    // Latitude against the point's OWN distance from the centre (not the nominal radius): that
    // makes this the true nearest-point projection, so a query slightly off the sphere (a triangle
    // centroid, a tolerance-loose edge sample) doesn't land at a shifted latitude. Identical for
    // on-surface points.
    const L = Math.max(1e-12, Math.hypot(d[0], d[1], d[2]));
    const v = Math.asin(Math.max(-1, Math.min(1, dot(d, this.f.z) / L)));
    const px = dot(d, this.f.x), py = dot(d, this.f.y);
    // At a pole the longitude is undefined — atan2 of fp noise lands anywhere on the period, and a
    // boundary sample AT the pole (a ball's seam meridians meet there) then lifts a wild diagonal
    // the CDT cannot enforce. Keep the caller's continuity hint instead: any u is equally valid at
    // the pole, and the hint is the loop-consistent one.
    if (hu !== undefined && Math.hypot(px, py) < 1e-9 * L) return [hu, v];
    return [Math.atan2(py, px), v];
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
  uSeam = Math.PI;
  vSeam = Math.PI;
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

/**
 * DEGENERATE_TOROIDAL_SURFACE: a torus with minor radius > major radius, so the full revolution
 * self-intersects where the tube crosses the axis (radial coordinate ρ(v) = R + r·cos v changes
 * sign at v = ±arccos(−R/r)). The surface splits into two valid lobes and the STEP select_outer
 * flag names which one the face uses:
 *   apple (outer, .T.): v ∈ (−vc, +vc), ρ(v) > 0 — the big outer barrel;
 *   lemon (inner, .F.): v ∈ (vc, 2π−vc), ρ(v) < 0 — the small lens hugging the axis, whose points
 *     physically sit at angle u+π with radius |ρ(v)|.
 * The plain-Torus placeholder mis-projected lemon boundaries (atan2 returns the MIRRORED angle and
 * the outer-branch v), so interior points landed on the wrong sheet 1.5mm off (wio-tracker's
 * fillet caps). Here project() inverts the correct lobe and v is an OPEN direction bounded at the
 * axis poles, so the mesher can never sample across the self-intersection.
 */
class DegenerateTorus implements Surface {
  kind = "DEGENERATE_TOROIDAL_SURFACE";
  periodicU = true;
  uSeam = Math.PI;
  f: Frame;
  rMaj: number;
  rMin: number;
  outer: boolean;
  vc: number;
  v0: number;
  v1: number;
  constructor(f: Frame, rMaj: number, rMin: number, outer: boolean) {
    this.f = f; this.rMaj = rMaj; this.rMin = rMin; this.outer = outer;
    this.vc = Math.acos(Math.max(-1, Math.min(1, -rMaj / rMin)));
    if (outer) { this.v0 = -this.vc; this.v1 = this.vc; }
    else { this.v0 = this.vc; this.v1 = 2 * Math.PI - this.vc; }
  }
  evaluate(u: number, v: number): Vec3 {
    v = Math.min(Math.max(v, this.v0), this.v1); // clamp at the axis poles — beyond is the other lobe
    const ring = add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
    const radial = scale(ring, this.rMaj + this.rMin * Math.cos(v));
    return add(this.f.o, add(radial, scale(this.f.z, this.rMin * Math.sin(v))));
  }
  project(p: Vec3): [number, number] {
    const d = sub(p, this.f.o);
    const lx = dot(d, this.f.x), ly = dot(d, this.f.y), lz = dot(d, this.f.z);
    const rho = Math.hypot(lx, ly);
    if (this.outer) return [Math.atan2(ly, lx), Math.atan2(lz, rho - this.rMaj)];
    // Lemon: the point sits at the mirrored angle (ρ(v) < 0), and v lives around π — shift atan2's
    // (−π, π] branch onto (0, 2π) so the valid band (vc, 2π−vc) is a single continuous interval.
    const u = Math.atan2(-ly, -lx);
    let v = Math.atan2(lz, -rho - this.rMaj);
    if (v < 0) v += 2 * Math.PI;
    return [u, v];
  }
  normal(u: number, v: number): Vec3 {
    // Su × Sv = ρ(v)·r·(ring·cos v + z·sin v): the analytic normal carries the SIGN of ρ(v), which
    // is negative on the whole lemon lobe — the unsigned torus formula emits lemon faces inverted
    // (18 same-direction shared edges and a 4% volume deficit on wio-tracker's fillet caps).
    const s = this.outer ? 1 : -1;
    const ring = add(scale(this.f.x, Math.cos(u)), scale(this.f.y, Math.sin(u)));
    return normalize(add(scale(ring, s * Math.cos(v)), scale(this.f.z, s * Math.sin(v))));
  }
  curvatureRadius(u: number, v: number): number {
    // Principal radii: the tube (rMin) and the ring |ρ(v)/cos v|, which SHRINKS toward the axis
    // poles — same class as a cone apex, so the ring term must be local or pole caps mesh coarse.
    const rho = Math.abs(this.rMaj + this.rMin * Math.cos(v));
    const c = Math.abs(Math.cos(v));
    return Math.max(1e-3, Math.min(this.rMin, c > 1e-9 ? rho / c : this.rMin));
  }
}

/**
 * Signed principal curvatures (k1, k2) of s at (u,v), positive where the surface bends TOWARD
 * s.normal(u,v): central differences of evaluate() at a METRIC step (the same physical step in u
 * and v whatever the parametrisation's units), then the shape operator's eigenvalues from the
 * first and second fundamental forms. null where the parametrisation degenerates (a pole).
 */
function principalCurvatures(s: Surface, u: number, v: number): [number, number] | null {
  const p = s.evaluate(u, v);
  const L = 1e-5 * (1 + Math.hypot(p[0], p[1], p[2])); // metric FD step (mm)
  const speed = (du: number, dv: number): number => {
    const a = s.evaluate(u + du, v + dv), b = s.evaluate(u - du, v - dv);
    return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) / (2 * Math.max(Math.abs(du), Math.abs(dv)));
  };
  const e0 = 1e-7 * (1 + Math.abs(u)), f0 = 1e-7 * (1 + Math.abs(v));
  const su = speed(e0, 0), sv = speed(0, f0);
  if (!(su > 1e-12) || !(sv > 1e-12)) return null;
  const hu = L / su, hv = L / sv;
  const pu1 = s.evaluate(u + hu, v), pu0 = s.evaluate(u - hu, v);
  const pv1 = s.evaluate(u, v + hv), pv0 = s.evaluate(u, v - hv);
  const a = s.evaluate(u + hu, v + hv), b = s.evaluate(u + hu, v - hv);
  const c = s.evaluate(u - hu, v + hv), d = s.evaluate(u - hu, v - hv);
  const Su: Vec3 = [(pu1[0] - pu0[0]) / (2 * hu), (pu1[1] - pu0[1]) / (2 * hu), (pu1[2] - pu0[2]) / (2 * hu)];
  const Sv: Vec3 = [(pv1[0] - pv0[0]) / (2 * hv), (pv1[1] - pv0[1]) / (2 * hv), (pv1[2] - pv0[2]) / (2 * hv)];
  const Suu: Vec3 = [(pu1[0] + pu0[0] - 2 * p[0]) / (hu * hu), (pu1[1] + pu0[1] - 2 * p[1]) / (hu * hu), (pu1[2] + pu0[2] - 2 * p[2]) / (hu * hu)];
  const Svv: Vec3 = [(pv1[0] + pv0[0] - 2 * p[0]) / (hv * hv), (pv1[1] + pv0[1] - 2 * p[1]) / (hv * hv), (pv1[2] + pv0[2] - 2 * p[2]) / (hv * hv)];
  const k4 = 4 * hu * hv;
  const Suv: Vec3 = [(a[0] - b[0] - c[0] + d[0]) / k4, (a[1] - b[1] - c[1] + d[1]) / k4, (a[2] - b[2] - c[2] + d[2]) / k4];
  const E = dot(Su, Su), F = dot(Su, Sv), G = dot(Sv, Sv);
  const det = E * G - F * F;
  if (!(det > 1e-18 * E * G)) return null;
  const n = s.normal(u, v);
  const e = dot(Suu, n), f = dot(Suv, n), g = dot(Svv, n);
  const H = (e * G - 2 * f * F + g * E) / (2 * det), K = (e * g - f * f) / det;
  const disc = Math.sqrt(Math.max(0, H * H - K));
  return [H + disc, H - disc];
}

/** OFFSET_SURFACE: the basis surface pushed `dist` mm along its (unit) normal. Shares the basis
 * parametrisation, so the boundary projects through the basis solver; the normal is unchanged and the
 * point is shifted. project(p) seeds from the basis foot point (p sits one offset out along the normal —
 * exact for a plane) and refines on the offset surface itself. */
class OffsetSurface implements Surface {
  kind = "OFFSET_SURFACE";
  base: Surface; dist: number;
  periodicU: boolean; periodicV?: boolean; uPeriod?: number; vPeriod?: number; uSeam?: number; vSeam?: number;
  uDomain?: [number, number]; vDomain?: [number, number];
  /** Consecutive wrong-branch rescans in project() that found nothing closer (cost cap). */
  private fruitlessScans = 0;
  constructor(base: Surface, dist: number) {
    this.base = base; this.dist = dist;
    this.periodicU = base.periodicU; this.periodicV = base.periodicV;
    this.uPeriod = base.uPeriod; this.vPeriod = base.vPeriod;
    this.uSeam = base.uSeam; this.vSeam = base.vSeam;
    this.uDomain = base.uDomain; this.vDomain = base.vDomain;
    // Offset of a swept surface: the profile radius over the span moves by |dist| one way or the
    // other depending on the side — take the tighter (floored like the unsigned fallback).
    const bs = base.spanRadius?.bind(base);
    if (bs) {
      this.profileAxis = base.profileAxis;
      this.spanRadius = (u0: number, u1: number, v0: number, v1: number): number => {
        const r = bs(u0, u1, v0, v1);
        return Number.isFinite(r) ? Math.max(1e-3, r - Math.abs(this.dist), 0.1 * r) : Infinity;
      };
    }
  }
  spanRadius?: (u0: number, u1: number, v0: number, v1: number) => number;
  profileAxis?: 0 | 1;
  evaluate(u: number, v: number): Vec3 {
    const p = this.base.evaluate(u, v), n = this.base.normal(u, v);
    return [p[0] + n[0] * this.dist, p[1] + n[1] * this.dist, p[2] + n[2] * this.dist];
  }
  project(p: Vec3, hu?: number, hv?: number): [number, number] {
    let [u, v] = this.base.project(p, hu, hv);
    // The basis foot point is only a SEED: p lies |dist| off the basis, and where the offset runs
    // toward a centre of curvature it approaches the focal distance, the basis distance field goes
    // flat and its nearest-point solve parks at a noisy parameter (a densely sampled boundary then
    // zigzags in (u,v) and self-crosses — the CDT drops constraints). Gauss-Newton on THIS surface,
    // S(u,v) + dist·n(u,v) = p, is well-conditioned there; keep it only if the residual improves.
    const clampU = (x: number): number => !this.periodicU && this.uDomain ? Math.min(Math.max(x, this.uDomain[0]), this.uDomain[1]) : x;
    const clampV = (x: number): number => !this.periodicV && this.vDomain ? Math.min(Math.max(x, this.vDomain[0]), this.vDomain[1]) : x;
    const res2 = (q: Vec3): number => (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2;
    const polish = (u: number, v: number): [number, number, number] => {
      let S = this.evaluate(u, v), r2 = res2(S);
      for (let it = 0; it < 8 && r2 > 1e-18; it++) {
        const eu = 1e-7 * (1 + Math.abs(u)), ev = 1e-7 * (1 + Math.abs(v));
        const a1 = this.evaluate(u + eu, v), a0 = this.evaluate(u - eu, v);
        const b1 = this.evaluate(u, v + ev), b0 = this.evaluate(u, v - ev);
        const Su: Vec3 = [(a1[0] - a0[0]) / (2 * eu), (a1[1] - a0[1]) / (2 * eu), (a1[2] - a0[2]) / (2 * eu)];
        const Sv: Vec3 = [(b1[0] - b0[0]) / (2 * ev), (b1[1] - b0[1]) / (2 * ev), (b1[2] - b0[2]) / (2 * ev)];
        const r: Vec3 = [S[0] - p[0], S[1] - p[1], S[2] - p[2]];
        const A = dot(Su, Su), B = dot(Su, Sv), C = dot(Sv, Sv), det = A * C - B * B;
        if (!(Math.abs(det) > 1e-24 * (A * C + 1e-300))) break;
        const g1 = dot(Su, r), g2 = dot(Sv, r);
        const nu = clampU(u - (C * g1 - B * g2) / det), nv = clampV(v - (A * g2 - B * g1) / det);
        const S2 = this.evaluate(nu, nv), r2n = res2(S2);
        if (!(r2n < r2)) break;
        const moved = Math.abs(nu - u) + Math.abs(nv - v);
        u = nu; v = nv; S = S2; r2 = r2n;
        if (moved < 1e-12 * (1 + Math.abs(u) + Math.abs(v))) break;
      }
      return [u, v, r2];
    };
    let r2: number;
    [u, v, r2] = polish(u, v);
    // WRONG-BRANCH RESEED. When |dist| exceeds the basis's local radius of curvature, the basis
    // nearest point of an offset-surface point can sit on ANOTHER stretch of the basis entirely
    // (ABC 00056073: a -6.35 mm offset of an extruded profile — every point of one rail landed
    // 11 mm off, all lifting to one (u,v) corner, and the face was dropped). Gauss-Newton cannot
    // walk back from there. Only when the polished residual is still large: scan the parameter
    // box on THIS surface for better seeds, polish the best few, and keep a strictly closer
    // answer. The trigger is a miss well beyond what a legitimate off-surface query leaves:
    // project() is routinely asked about points slightly OFF the surface (triangle centroids
    // within chord tolerance), whose misses never improve. Measured: those stay ≤0.0132·|dist|
    // (ABC 00040917, 43k queries — scanning them made that B-spline offset 6× slower), while
    // real wrong branches start at 0.035·|dist| (00046089) and reach 1.74·|dist| (00056073).
    // A rescan only ever keeps a strictly closer point, so a misplaced trigger costs time, not
    // correctness — and that cost is capped: after 64 consecutive fruitless rescans this surface
    // stops rescanning.
    const tol = 0.02 * Math.abs(this.dist) + 1e-6;
    if (r2 > tol * tol && this.fruitlessScans < 64) {
      const curve = (this.base as { curve?: { t0: number; t1: number } }).curve;
      const rangeOf = (dir: 0 | 1): [number, number] | null => {
        const per = dir === 0 ? this.periodicU : this.periodicV;
        const P = dir === 0 ? this.uPeriod : this.vPeriod;
        const seam = (dir === 0 ? this.uSeam : this.vSeam) ?? 0;
        if (per && P) return [seam, seam + P];
        const dom = dir === 0 ? this.uDomain : this.vDomain;
        if (dom) return dom;
        if (dir === 0 && curve && this.base.kind === "SURFACE_OF_LINEAR_EXTRUSION") return [curve.t0, curve.t1];
        return null;
      };
      const ru = rangeOf(0), rv = rangeOf(1);
      if (ru || rv) {
        const nU = ru ? 64 : 1, nV = rv ? 16 : 1;
        const seeds: [number, number, number][] = [];
        for (let i = 0; i < nU; i++) for (let j = 0; j < nV; j++) {
          const su = ru ? ru[0] + ((ru[1] - ru[0]) * (i + 0.5)) / nU : u;
          const sv = rv ? rv[0] + ((rv[1] - rv[0]) * (j + 0.5)) / nV : v;
          seeds.push([su, sv, res2(this.evaluate(su, sv))]);
        }
        seeds.sort((a, b) => a[2] - b[2]);
        const before = r2;
        for (const [su, sv] of seeds.slice(0, 4)) {
          const [pu, pv, pr2] = polish(su, sv);
          if (pr2 < r2) { u = pu; v = pv; r2 = pr2; }
        }
        this.fruitlessScans = r2 < 0.25 * before ? 0 : this.fruitlessScans + 1;
      }
    }
    return [u, v];
  }
  normal(u: number, v: number): Vec3 { return this.base.normal(u, v); }
  /** Offsetting by d along the base normal n moves every principal radius by -d, SIGNED: a
   *  direction bending toward n (centre of curvature on the +n side, R > 0) tightens to R - d, one
   *  bending away (R < 0) widens to |R| + d. The base's curvatureRadius() is unsigned, and the old
   *  r + d clamped every offset toward a concave side to 1 um (gridCDT saturation). Signs come
   *  from the base's second fundamental form; the base's own (cell-smoothed, conservative)
   *  magnitude is kept for its tightest direction. */
  curvatureRadius(u: number, v: number): number {
    const rb = this.base.curvatureRadius(u, v);
    if (!Number.isFinite(rb)) return Infinity; // flat base (plane): the offset is flat too
    const k = principalCurvatures(this.base, u, v);
    if (!k) return Math.max(1e-3, rb - Math.abs(this.dist), 0.1 * rb); // sign unknown: conservative
    // Signed radii, the dominant one replaced by the base's magnitude when that is tighter.
    const big = Math.abs(k[0]) >= Math.abs(k[1]) ? 0 : 1;
    let r = Infinity;
    for (let i = 0; i < 2; i++) {
      const ki = k[i]!;
      let R = Math.abs(ki) > 1e-12 ? 1 / ki : Infinity;
      if (i === big && Math.abs(R) > rb) R = ki >= 0 ? rb : -rb;
      if (Number.isFinite(R)) r = Math.min(r, Math.abs(R - this.dist));
    }
    return Number.isFinite(r) ? Math.max(1e-3, r) : Infinity;
  }
}

/** SURFACE_OF_LINEAR_EXTRUSION: the basis curve swept along a vector, S(u,v) = C(u) + v·axis.
 * u is the curve parameter, v the (unitless) multiple of the extrusion vector. */
class ExtrusionSurface implements Surface {
  kind = "SURFACE_OF_LINEAR_EXTRUSION";
  curve: Curve3;
  axis: Vec3;        // full extrusion vector (direction · magnitude, mm)
  axisUnit: Vec3;
  axisLen: number;
  periodicU: boolean;
  uPeriod?: number;
  uSeam?: number;
  private radii: CurveRadiusTable;
  profileAxis = 0 as const;
  constructor(curve: Curve3, axis: Vec3) {
    this.curve = curve; this.axis = axis;
    this.axisLen = Math.max(1e-12, Math.hypot(axis[0], axis[1], axis[2]));
    this.axisUnit = [axis[0] / this.axisLen, axis[1] / this.axisLen, axis[2] / this.axisLen];
    this.periodicU = curve.closed;
    if (curve.closed) { this.uPeriod = curve.t1 - curve.t0; this.uSeam = curve.t0; }
    this.radii = new CurveRadiusTable(curve, this.axisUnit);
  }
  /** A closed profile wraps u into [t0, t1): the unroll and the seam-spanning grids chart u past t1
   *  (or before t0), and a B-spline basis curve clamps there. In range, u passes through untouched
   *  (the modulo would perturb its last bits). */
  private wrapU(u: number): number {
    if (!this.periodicU || !this.uPeriod || (u >= this.curve.t0 && u < this.curve.t1)) return u;
    return this.curve.t0 + (((u - this.curve.t0) % this.uPeriod) + this.uPeriod) % this.uPeriod;
  }
  evaluate(u: number, v: number): Vec3 {
    if (this.periodicU && this.uPeriod) {
      u = this.curve.t0 + (((u - this.curve.t0) % this.uPeriod) + this.uPeriod) % this.uPeriod;
    }
    const p = this.curve.evaluate(u);
    return [p[0] + this.axis[0] * v, p[1] + this.axis[1] * v, p[2] + this.axis[2] * v];
  }
  project(p: Vec3): [number, number] {
    // Alternate the two 1D solves; converges immediately when the axis is normal to the curve
    // plane (the usual case) and in a few rounds for oblique sweeps.
    let u = this.curve.project(p);
    let v = 0;
    for (let it = 0; it < 3; it++) {
      const c = this.curve.evaluate(u);
      v = ((p[0] - c[0]) * this.axisUnit[0] + (p[1] - c[1]) * this.axisUnit[1] + (p[2] - c[2]) * this.axisUnit[2]) / this.axisLen;
      const q: Vec3 = [p[0] - this.axis[0] * v, p[1] - this.axis[1] * v, p[2] - this.axis[2] * v];
      const u2 = this.curve.project(q);
      if (Math.abs(u2 - u) < 1e-10) { u = u2; break; }
      u = u2;
    }
    return [u, v];
  }
  normal(u: number, v: number): Vec3 {
    // Wrap each probe on its own, like evaluate: unwrapped, a closed B-spline profile clamps both
    // probes to t1 past the seam, the tangent vanishes and the normal fell back to +Z. The unroll
    // orients every triangle by this normal, so the strip beyond u = t1 came out reversed (ABC
    // 00021718 #172: 281 flipped slivers along the seam side).
    const e = 1e-5 * Math.max(1, this.curve.t1 - this.curve.t0);
    const a = this.curve.evaluate(this.wrapU(u - e)), b = this.curve.evaluate(this.wrapU(u + e));
    const tang: Vec3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const n = cross(tang, this.axis);
    const len = Math.hypot(n[0], n[1], n[2]);
    return len > 1e-12 ? [n[0] / len, n[1] / len, n[2] / len] : [0, 0, 1];
  }
  /** Cross-section curvature of the profile AT u (the rulings are straight): local, so a sharp
   *  corner elsewhere on the basis curve no longer sizes this face; u wraps on a closed profile —
   *  a face spanning the seam is charted with u unwrapped past t1 / before t0. */
  curvatureRadius(u: number): number {
    const r = this.radii.at(u);
    return Number.isFinite(r) ? Math.max(1e-3, r) : Infinity;
  }
  spanRadius(u0: number, u1: number): number {
    const r = this.radii.minOver(u0, u1);
    return Number.isFinite(r) ? Math.max(1e-3, r) : Infinity;
  }
}

/** SURFACE_OF_REVOLUTION: the swept (profile) curve C(v) rotated about an axis (point A, unit
 * direction D). S(u,v) = A + Rodrigues(C(v)−A, D, u): u is the revolution angle (periodic, 2π,
 * seam at ±π like the other analytic surfaces), v the profile-curve parameter. At u=0 the surface
 * IS the profile, so cylinder/cone/sphere/torus are all special cases of this one construction.
 * Inversion matches on (axial, radial) coordinates — both preserved by the rotation — so it stays
 * correct even when the profile touches or crosses the axis (a sphere-like pole), where a
 * fixed-meridian angle would be ambiguous. */
class RevolutionSurface implements Surface {
  kind = "SURFACE_OF_REVOLUTION";
  periodicU = true;
  uSeam = Math.PI;
  uPeriod = 2 * Math.PI;
  periodicV = false;
  vPeriod?: number;
  vSeam?: number;
  curve: Curve3;
  A: Vec3;
  D: Vec3;
  t0: number;
  t1: number;
  // profile samples in (axial, radial) space for the v-inversion seed
  private sv: number[] = [];
  private sa: number[] = [];
  private sr: number[] = [];
  vDomain: [number, number];
  private meridian: CurveRadiusTable;
  profileAxis = 1 as const;
  constructor(curve: Curve3, A: Vec3, D: Vec3) {
    this.curve = curve; this.A = A; this.D = normalize(D);
    this.meridian = new CurveRadiusTable(curve);
    this.t0 = curve.t0; this.t1 = curve.t1;
    this.vDomain = [this.t0, this.t1];
    if (curve.closed) { this.periodicV = true; this.vPeriod = this.t1 - this.t0; this.vSeam = this.t0; }
    const N = 128;
    for (let i = 0; i <= N; i++) {
      const v = this.t0 + ((this.t1 - this.t0) * i) / N;
      const [a, rho] = this.axialRadial(curve.evaluate(v));
      this.sv.push(v); this.sa.push(a); this.sr.push(rho);
    }
  }
  /** Decompose a point's offset from the axis into (distance along D, distance perpendicular). */
  private axialRadial(p: Vec3): [number, number] {
    const w = sub(p, this.A);
    const a = dot(w, this.D);
    const rad = sub(w, scale(this.D, a));
    return [a, Math.hypot(rad[0], rad[1], rad[2])];
  }
  /** Rotate a point about the axis by angle u (cu=cos u, su=sin u). */
  private rotPoint(p: Vec3, cu: number, su: number): Vec3 {
    const w = sub(p, this.A);
    const dw = dot(this.D, w);
    const cxw = cross(this.D, w);
    const k = dw * (1 - cu);
    return [
      this.A[0] + w[0] * cu + cxw[0] * su + this.D[0] * k,
      this.A[1] + w[1] * cu + cxw[1] * su + this.D[1] * k,
      this.A[2] + w[2] * cu + cxw[2] * su + this.D[2] * k,
    ];
  }
  /** Rotate a free vector (direction) about the axis by angle u. */
  private rotVec(w: Vec3, cu: number, su: number): Vec3 {
    const dw = dot(this.D, w);
    const cxw = cross(this.D, w);
    const k = dw * (1 - cu);
    return [w[0] * cu + cxw[0] * su + this.D[0] * k, w[1] * cu + cxw[1] * su + this.D[1] * k, w[2] * cu + cxw[2] * su + this.D[2] * k];
  }
  /** A closed profile wraps v into [t0, t1), as ExtrusionSurface wraps u (in range: untouched). */
  private wrapV(v: number): number {
    if (!this.periodicV || !this.vPeriod || (v >= this.t0 && v < this.t1)) return v;
    return this.t0 + (((v - this.t0) % this.vPeriod) + this.vPeriod) % this.vPeriod;
  }
  private tangent(v: number): Vec3 {
    const e = 1e-5 * Math.max(1, this.t1 - this.t0);
    // A closed profile has no ends: wrap the probes across the seam instead of clamping them (a
    // clamp at t1 made the tangent vanish past the seam and the normal fell back to the axis).
    const a = this.curve.evaluate(this.periodicV ? this.wrapV(v - e) : Math.max(this.t0, v - e));
    const b = this.curve.evaluate(this.periodicV ? this.wrapV(v + e) : Math.min(this.t1, v + e));
    return sub(b, a);
  }
  evaluate(u: number, v: number): Vec3 {
    if (this.periodicV && this.vPeriod) v = this.t0 + (((v - this.t0) % this.vPeriod) + this.vPeriod) % this.vPeriod;
    return this.rotPoint(this.curve.evaluate(v), Math.cos(u), Math.sin(u));
  }
  project(p: Vec3): [number, number] {
    const [ap, rp] = this.axialRadial(p);
    // v: profile parameter whose (axial, radial) matches the query — seeded from the sample table,
    // then golden-section refined on the same 2D objective.
    let bi = 0, bf = Infinity;
    for (let i = 0; i < this.sv.length; i++) {
      const da = this.sa[i]! - ap, dr = this.sr[i]! - rp, f = da * da + dr * dr;
      if (f < bf) { bf = f; bi = i; }
    }
    const cost = (v: number): number => { const [a, rho] = this.axialRadial(this.curve.evaluate(v)); const da = a - ap, dr = rho - rp; return da * da + dr * dr; };
    let lo = this.sv[Math.max(0, bi - 1)]!, hi = this.sv[Math.min(this.sv.length - 1, bi + 1)]!;
    const g = 0.6180339887;
    let c1 = hi - g * (hi - lo), c2 = lo + g * (hi - lo);
    let f1 = cost(c1), f2 = cost(c2);
    for (let it = 0; it < 24 && hi - lo > 1e-9 * (this.t1 - this.t0 || 1); it++) {
      if (f1 < f2) { hi = c2; c2 = c1; f2 = f1; c1 = hi - g * (hi - lo); f1 = cost(c1); }
      else { lo = c1; c1 = c2; f1 = f2; c2 = lo + g * (hi - lo); f2 = cost(c2); }
    }
    const v = (lo + hi) / 2;
    // u: signed rotation about D from the profile point's radial direction to the query's.
    const w = sub(p, this.A);
    const dw = dot(this.D, w);
    const radP = sub(w, scale(this.D, dw));
    const wc = sub(this.curve.evaluate(v), this.A);
    const radC = sub(wc, scale(this.D, dot(this.D, wc)));
    const nP = Math.hypot(radP[0], radP[1], radP[2]), nC = Math.hypot(radC[0], radC[1], radC[2]);
    if (nP < 1e-9 || nC < 1e-9) return [0, v]; // on the axis (pole): angle is arbitrary
    const cosA = dot(radC, radP) / (nC * nP);
    const sinA = dot(cross(radC, radP), this.D) / (nC * nP);
    return [Math.atan2(sinA, cosA), v];
  }
  normal(u: number, v: number): Vec3 {
    v = this.wrapV(v);
    const cu = Math.cos(u), su = Math.sin(u);
    const S = this.rotPoint(this.curve.evaluate(v), cu, su);
    const Su = cross(this.D, sub(S, this.A));        // parallel-circle tangent
    const Sv = this.rotVec(this.tangent(v), cu, su); // meridian tangent
    let n = cross(Su, Sv);
    let len = Math.hypot(n[0], n[1], n[2]);
    if (len < 1e-14) { n = cross(Sv, this.D); len = Math.hypot(n[0], n[1], n[2]); } // on axis: fall back
    return len > 1e-14 ? [n[0] / len, n[1] / len, n[2] / len] : this.D;
  }
  curvatureRadius(_u: number, v: number): number {
    const [, rho] = this.axialRadial(this.curve.evaluate(v)); // parallel-circle radius (tight near axis)
    const mr = this.meridian.at(v);                           // LOCAL meridian radius of curvature
    let r = rho > 1e-6 ? rho : Infinity;
    if (Number.isFinite(mr)) r = Math.min(r, mr);
    return Number.isFinite(r) ? Math.max(1e-3, r) : Infinity;
  }
  spanRadius(_u0: number, _u1: number, v0: number, v1: number): number {
    const r = this.meridian.minOver(v0, v1); // the parallel-circle term stays local (curvatureRadius)
    return Number.isFinite(r) ? Math.max(1e-3, r) : Infinity;
  }
}

// ---- B-spline (NURBS) surface --------------------------------------------------------------
// Tensor-product de Boor on homogeneous (wx,wy,wz,w) 4-vectors, flat-buffer edition: the old
// array-of-arrays version allocated ~12 arrays per call and evaluateRaw is the hottest geometry
// path in the pipeline. The triangle runs in place over module-level scratch (single-threaded).
function findSpan(degree: number, nCtrl: number, knots: number[], u: number): number {
  const nmax = nCtrl - 1;
  let k = degree;
  while (k < nmax && knots[k + 1]! <= u) k++;
  return k;
}

/** In-place de Boor triangle around knot span k. On entry dst[j*4..j*4+3] holds the control
 * 4-vectors of span rows k-degree..k; the result lands at dst[degree*4..]. Descending-j updates
 * read d[j-1] before it is touched this pass — identical arithmetic order to the old version. */
function deBoorTri(degree: number, knots: number[], u: number, k: number, dst: Float64Array): void {
  for (let r = 1; r <= degree; r++) {
    for (let j = degree; j >= r; j--) {
      const i = k - degree + j;
      const lo = knots[i]!, hi = knots[i + degree - r + 1]!;
      const a = hi > lo ? (u - lo) / (hi - lo) : 0;
      const pj = (j - 1) * 4, qj = j * 4;
      dst[qj] = dst[pj]! + (dst[qj]! - dst[pj]!) * a;
      dst[qj + 1] = dst[pj + 1]! + (dst[qj + 1]! - dst[pj + 1]!) * a;
      dst[qj + 2] = dst[pj + 2]! + (dst[qj + 2]! - dst[pj + 2]!) * a;
      dst[qj + 3] = dst[pj + 3]! + (dst[qj + 3]! - dst[pj + 3]!) * a;
    }
  }
}

// De Boor scratch (grown on demand): the v-direction triangle and the row of u-direction inputs.
let dbTri = new Float64Array(4 * 8);
let dbRow = new Float64Array(4 * 8);

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
  uSeam = 0; vSeam = 0;
  uDeg: number; vDeg: number;
  // Control net flattened to [iu * nvCps*4 + iv*4 + c] (homogeneous wx,wy,wz,w) — the nested
  // number[][][] costs ~4x the memory and defeats de Boor's cache locality.
  cpsF: Float64Array; nuCps: number; nvCps: number;
  uKnots: number[]; vKnots: number[];
  u0: number; u1: number; v0: number; v1: number;
  uDomain: [number, number]; vDomain: [number, number];
  // projection lookup grid: gPF[(i * gV.length + j) * 3 + c] = evaluate(gU[i], gV[j])
  gU: number[] = []; gV: number[] = []; gPF = new Float64Array(0);
  rc = Infinity;
  rcGrid: number[][] = []; // per grid CELL local min normal-turn radius (same estimate as rc)
  closedU = false; closedV = false; // patch wraps onto itself in u / v
  constructor(uDeg: number, vDeg: number, cps: number[][][], uKnots: number[], vKnots: number[]) {
    this.uDeg = uDeg; this.vDeg = vDeg; this.uKnots = uKnots; this.vKnots = vKnots;
    const nu = cps.length - 1, nv = cps[0]!.length - 1;
    this.nuCps = nu + 1; this.nvCps = nv + 1;
    this.cpsF = new Float64Array(this.nuCps * this.nvCps * 4);
    for (let i = 0; i <= nu; i++) for (let j = 0; j <= nv; j++) {
      const s = cps[i]![j]!, o = (i * this.nvCps + j) * 4;
      this.cpsF[o] = s[0]!; this.cpsF[o + 1] = s[1]!; this.cpsF[o + 2] = s[2]!; this.cpsF[o + 3] = s[3]!;
    }
    this.u0 = uKnots[uDeg]!; this.u1 = uKnots[nu + 1]!;
    this.v0 = vKnots[vDeg]!; this.v1 = vKnots[nv + 1]!;
    this.uDomain = [this.u0, this.u1]; this.vDomain = [this.v0, this.v1];
    const um = (this.u0 + this.u1) / 2, vm = (this.v0 + this.v1) / 2;
    const gap = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    this.closedU = gap(this.evaluateRaw(this.u0, vm), this.evaluateRaw(this.u1, vm)) < 1e-4;
    this.closedV = gap(this.evaluateRaw(um, this.v0), this.evaluateRaw(um, this.v1)) < 1e-4;
    // A closed patch is periodic in that direction with period = its parameter span. Trimmed faces
    // then project their boundary continuously across the seam (hint-seeded) so the param-space
    // polygon stays simple and the CDT can enforce it — otherwise the seam tangles -> cracks.
    this.periodicU = this.closedU; this.periodicV = this.closedV;
    this.uPeriod = this.u1 - this.u0; this.vPeriod = this.v1 - this.v0;
    this.uSeam = this.u0; this.vSeam = this.v0; // knot-domain start = where a closed patch wraps
    this.buildGrid();
  }
  private buildGrid(): void {
    // Seed-grid resolution follows the surface's metric aspect. A fixed 24×24 grid on a
    // 0.4mm × 2400mm helical strip (StingStopp_4000 groove bottom) puts seeds 100mm apart along
    // the strip; Gauss-Newton cannot cross that basin, so boundary projection lands millimetres
    // off (accepted as "best effort"), the pcurve tangles, and the CDT shatters the face.
    // Allocate cells proportional to per-direction arc length within a fixed budget.
    const umid = (this.u0 + this.u1) / 2, vmid = (this.v0 + this.v1) / 2;
    const arcLen = (c: 0 | 1): number => {
      let len = 0;
      let prev: Vec3 | null = null;
      for (let i = 0; i <= 16; i++) {
        const t = i / 16;
        const p = c === 0
          ? this.evaluateRaw(this.u0 + (this.u1 - this.u0) * t, vmid)
          : this.evaluateRaw(umid, this.v0 + (this.v1 - this.v0) * t);
        if (prev) len += Math.hypot(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]);
        prev = p;
      }
      return Math.max(len, 1e-9);
    };
    const uLen = arcLen(0), vLen = arcLen(1);
    const BUDGET = 2048;
    const GU = Math.max(4, Math.min(1024, Math.round(Math.sqrt((BUDGET * uLen) / vLen))));
    const GV = Math.max(4, Math.min(1024, Math.round(Math.sqrt((BUDGET * vLen) / uLen))));
    for (let i = 0; i <= GU; i++) this.gU.push(this.u0 + ((this.u1 - this.u0) * i) / GU);
    for (let j = 0; j <= GV; j++) this.gV.push(this.v0 + ((this.v1 - this.v0) * j) / GV);
    this.gPF = new Float64Array((GU + 1) * (GV + 1) * 3);
    for (let i = 0; i <= GU; i++) {
      for (let j = 0; j <= GV; j++) {
        const p = this.evaluate(this.gU[i]!, this.gV[j]!), o = (i * (GV + 1) + j) * 3;
        this.gPF[o] = p[0]; this.gPF[o + 1] = p[1]; this.gPF[o + 2] = p[2];
      }
    }
    // Rough min curvature radius from normal turn across a FIXED 24×24 lattice (drives adaptive
    // refinement), kept PER CELL: one degenerate spot (a lens patch pinching to a needle tip) must
    // not drag the whole surface's target down — a global minimum made gridCDT mesh a benign R≈2mm
    // face at the needle-tip's 0.1mm "radius" everywhere (OpenVessel's bow lens). Deliberately
    // DECOUPLED from the adaptive seed grid above: finer curvature cells would silently densify
    // meshing ("everything" +50% triangles when this followed the 2048-cell seed budget).
    const RU = 24, RV = 24;
    const rU = (i: number): number => this.u0 + ((this.u1 - this.u0) * i) / RU;
    const rV = (j: number): number => this.v0 + ((this.v1 - this.v0) * j) / RV;
    let minR = Infinity;
    for (let i = 0; i < RU; i++) {
      const row: number[] = [];
      for (let j = 0; j < RV; j++) {
        const a = this.evaluate(rU(i), rV(j)), b = this.evaluate(rU(i + 1), rV(j)), c = this.evaluate(rU(i), rV(j + 1));
        const n1 = this.normalAt(rU(i), rV(j)), n2 = this.normalAt(rU(i + 1), rV(j)), n3 = this.normalAt(rU(i), rV(j + 1));
        const du = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
        const dv = Math.hypot(c[0] - a[0], c[1] - a[1], c[2] - a[2]);
        const tu = Math.acos(Math.max(-1, Math.min(1, dot(n1, n2))));
        const tv = Math.acos(Math.max(-1, Math.min(1, dot(n1, n3))));
        let r = Infinity;
        if (tu > 1e-4 && du > 1e-9) r = Math.min(r, du / tu);
        if (tv > 1e-4 && dv > 1e-9) r = Math.min(r, dv / tv);
        row.push(r);
        minR = Math.min(minR, r);
      }
      this.rcGrid.push(row);
    }
    this.rc = Number.isFinite(minR) ? Math.max(0.05, minR) : Infinity;
  }
  private evaluateRaw(u: number, v: number): Vec3 {
    // The u-direction combination only reads the uDeg+1 rows around u's knot span — the v-direction
    // de Boor runs only for those rows (rows/(uDeg+1)× fewer than the naive full sweep). Each row's
    // triangle runs in place in dbTri; the per-row results accumulate in dbRow for the u-direction
    // pass. Same spans, same arithmetic order: bit-identical to the array-of-arrays version.
    const uDeg = this.uDeg, vDeg = this.vDeg;
    const ku = findSpan(uDeg, this.nuCps, this.uKnots, u);
    const kv = findSpan(vDeg, this.nvCps, this.vKnots, v);
    if (dbRow.length < (uDeg + 1) * 4) dbRow = new Float64Array((uDeg + 1) * 4);
    if (dbTri.length < (vDeg + 1) * 4) dbTri = new Float64Array((vDeg + 1) * 4);
    const stride = this.nvCps * 4;
    const lo = ku - uDeg, vlo = kv - vDeg;
    const nvals = (vDeg + 1) * 4;
    for (let j = 0; j <= uDeg; j++) {
      const rowOff = (lo + j) * stride + vlo * 4;
      for (let c = 0; c < nvals; c++) dbTri[c] = this.cpsF[rowOff + c]!;
      deBoorTri(vDeg, this.vKnots, v, kv, dbTri);
      const res = vDeg * 4, o = j * 4;
      dbRow[o] = dbTri[res]!; dbRow[o + 1] = dbTri[res + 1]!; dbRow[o + 2] = dbTri[res + 2]!; dbRow[o + 3] = dbTri[res + 3]!;
    }
    deBoorTri(uDeg, this.uKnots, u, ku, dbRow);
    const r = uDeg * 4;
    const w = dbRow[r + 3]! || 1;
    return [dbRow[r]! / w, dbRow[r + 1]! / w, dbRow[r + 2]! / w];
  }
  evaluate(u: number, v: number): Vec3 {
    // Wrap a seam-unwrapped coordinate back into the patch domain for a closed direction (same point
    // by periodicity); de Boor extrapolates nonsense outside the knot span otherwise.
    if (this.closedU && this.uPeriod > 0) u = this.u0 + (((u - this.u0) % this.uPeriod) + this.uPeriod) % this.uPeriod;
    if (this.closedV && this.vPeriod > 0) v = this.v0 + (((v - this.v0) % this.vPeriod) + this.vPeriod) % this.vPeriod;
    return this.evaluateRaw(u, v);
  }
  private normalAt(u: number, v: number): Vec3 {
    // Wrap a seam-unwrapped coordinate exactly like evaluate() does BEFORE clamping: a caller in
    // an unwrapped chart (gridCDT's uv centroids) would otherwise clamp to the far knot end and
    // get the normal of the wrong side of the surface (GoProHandlePod: 62% signed-volume loss
    // from systematically mis-oriented seam-adjacent triangles).
    if (this.closedU && this.uPeriod > 0) u = this.u0 + (((u - this.u0) % this.uPeriod) + this.uPeriod) % this.uPeriod;
    if (this.closedV && this.vPeriod > 0) v = this.v0 + (((v - this.v0) % this.vPeriod) + this.vPeriod) % this.vPeriod;
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
  curvatureRadius(u: number, v: number): number {
    // Local lookup: min over the query's cell and its 8 neighbours (one cell of smoothing keeps the
    // size field from jumping across cell borders). Falls back to the global rc off-grid.
    const nu = this.rcGrid.length;
    if (!nu) return this.rc;
    const nv = this.rcGrid[0]!.length;
    const ci = Math.min(nu - 1, Math.max(0, Math.floor(((u - this.u0) / Math.max(this.u1 - this.u0, 1e-30)) * nu)));
    const cj = Math.min(nv - 1, Math.max(0, Math.floor(((v - this.v0) / Math.max(this.v1 - this.v0, 1e-30)) * nv)));
    let r = Infinity;
    for (let i = Math.max(0, ci - 1); i <= Math.min(nu - 1, ci + 1); i++) {
      for (let j = Math.max(0, cj - 1); j <= Math.min(nv - 1, cj + 1); j++) r = Math.min(r, this.rcGrid[i]![j]!);
    }
    return Number.isFinite(r) ? Math.max(0.05, r) : Infinity;
  }
  project(p: Vec3, hu?: number, hv?: number): [number, number] {
    // Seed from the caller's hint (continuous boundary projection) or the nearest grid node, then a
    // few Gauss-Newton steps minimising |S(u,v)-p|². In a CLOSED direction the coordinate is NOT
    // clamped — it follows p across the seam (evaluate() wraps it back), so a loop crossing the seam
    // stays monotone in param instead of snapping between v0 and v1.
    let bd2 = Infinity;
    const scan = (): [number, number] => {
      let bu = this.u0, bv = this.v0;
      const nv = this.gV.length, g = this.gPF;
      for (let i = 0; i < this.gU.length; i++) for (let j = 0; j < nv; j++) {
        const o = (i * nv + j) * 3;
        const d = (g[o]! - p[0]) ** 2 + (g[o + 1]! - p[1]) ** 2 + (g[o + 2]! - p[2]) ** 2;
        if (d < bd2) { bd2 = d; bu = this.gU[i]!; bv = this.gV[j]!; }
      }
      return [bu, bv];
    };
    let u: number, v: number;
    if (hu !== undefined && hv !== undefined) { u = hu; v = hv; }
    else [u, v] = scan();
    [u, v] = this.newton(p, u, v);
    // RESIDUAL-VERIFIED MULTI-START. On a patch with a COLLAPSED row (S(u,0) is one 3D point for
    // every u) the Jacobian is singular along the row, and a Newton seeded there — the hint chain
    // walked onto the row, or the grid-nearest node IS a row node (they are all equidistant) —
    // parks at an arbitrary u with a millimetre residual while the true preimage sits elsewhere
    // (Stealthburner fillet patches). "r beats bd" cannot detect this: the nearest node IS the
    // collapse point, so r ≈ bd looks healthy. The discriminator is MULTIMODALITY: gather the
    // grid nodes within 2× the best distance and cluster them in (u,v) — the stuck case shows
    // several distant clusters (the whole degenerate row is equidistant), a healthy off-surface
    // query (triangle centroid: the hot orientation path) shows exactly one and skips the retries.
    const e0 = this.evaluate(u, v);
    let r2 = (e0[0] - p[0]) ** 2 + (e0[1] - p[1]) ** 2 + (e0[2] - p[2]) ** 2;
    if (r2 > 1e-6) {
      if (bd2 === Infinity) scan();
      if (r2 > 0.25 * bd2) {
        const span = Math.max(this.u1 - this.u0, this.v1 - this.v0);
        const lim = 4 * bd2 + 1e-12; // nodes within 2× the best grid distance
        const near: { d: number; cu: number; cv: number }[] = [];
        const nv = this.gV.length, g = this.gPF;
        for (let i = 0; i < this.gU.length; i++) {
          for (let j = 0; j < nv; j++) {
            const o = (i * nv + j) * 3;
            const d = (g[o]! - p[0]) ** 2 + (g[o + 1]! - p[1]) ** 2 + (g[o + 2]! - p[2]) ** 2;
            if (d <= lim) near.push({ d, cu: this.gU[i]!, cv: this.gV[j]! });
          }
        }
        near.sort((a, b) => a.d - b.d);
        const cand: [number, number][] = [];
        for (const { cu, cv } of near) {
          if (cand.some(([x, y]) => Math.hypot(x - cu, y - cv) < span / 16)) continue;
          cand.push([cu, cv]);
          if (cand.length >= 8) break;
        }
        if (cand.length >= 2) {
          for (const [cu, cv] of cand) {
            const [nu, nv] = this.newton(p, cu, cv);
            const e = this.evaluate(nu, nv);
            const r = (e[0] - p[0]) ** 2 + (e[1] - p[1]) ** 2 + (e[2] - p[2]) ** 2;
            if (r < r2) { r2 = r; u = nu; v = nv; }
          }
        }
      }
    }
    return [u, v];
  }
  private newton(p: Vec3, u: number, v: number): [number, number] {
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
      let du: number, dv: number;
      if (Math.abs(det) >= 1e-18) {
        du = -(c * g1 - b * g2) / det; dv = -(a * g2 - b * g1) / det;
      } else if (c > 1e-30 && c >= a) {
        // Singular Jacobian — a COLLAPSED row (Su = 0 where S(u,v0) is one point for every u).
        // Breaking here strands the solver ON the row at an arbitrary u; instead descend along the
        // one non-degenerate direction, which walks off the row and restores a full Jacobian for
        // the next iteration.
        du = 0; dv = -g2 / c;
      } else if (a > 1e-30) {
        du = -g1 / a; dv = 0;
      } else break;
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
export function makeSurface(t: Table, id: number, s: number, aRad = 1): Surface | null {
  const kind = t.typeOf(id);
  if (!kind) return makeBSplineSurface(t, id, s); // complex (rational B-spline) surface
  const r = t.record(id);
  switch (kind) {
    case "SURFACE_OF_LINEAR_EXTRUSION": {
      // (name, swept_curve#, extrusion_axis#) — the axis is a VECTOR(name, direction#, magnitude).
      const curve = makeCurve(t, ref(r.params[1]!), s, aRad);
      if (!curve) return null;
      const vec = t.record(ref(r.params[2]!));
      const dir = readDirection(t, ref(vec.params[1]!));
      const mag = num(vec.params[2]!) * s;
      return new ExtrusionSurface(curve, [dir[0] * mag, dir[1] * mag, dir[2] * mag]);
    }
    case "SURFACE_OF_REVOLUTION": {
      // (name, swept_curve#, axis_position#) — axis_position is an AXIS1_PLACEMENT(name, location, axis?).
      const curve = makeCurve(t, ref(r.params[1]!), s, aRad);
      if (!curve) return null;
      const ax = t.record(ref(r.params[2]!));
      const A = readPoint(t, ref(ax.params[1]!), s);
      const D = ax.params[2] && ax.params[2].k === "ref" ? readDirection(t, ref(ax.params[2])) : [0, 0, 1] as Vec3;
      return new RevolutionSurface(curve, A, D);
    }
    case "B_SPLINE_SURFACE_WITH_KNOTS":
      return makeBSplineSurface(t, id, s);
    case "PLANE":
      return new Plane(readPlacement(t, ref(r.params[1]!), s));
    case "CYLINDRICAL_SURFACE":
      return new Cylinder(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s);
    case "CONICAL_SURFACE":
      // params: (name, placement, radius, semi_angle). The semi-angle is a PLANE ANGLE — scale it by
      // the file's angle unit (aRad); degree-unit files (e.g. inch/ASME parts) otherwise get a cone
      // half-angle ~57x too large, bulging every chamfer/taper far off its true boundary.
      return new Cone(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s, num(r.params[3]!) * aRad);
    case "SPHERICAL_SURFACE":
      return new Sphere(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s);
    case "TOROIDAL_SURFACE":
      return new Torus(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s, num(r.params[3]!) * s);
    case "DEGENERATE_TOROIDAL_SURFACE":
      // (name, placement, major, minor, select_outer) — see DegenerateTorus for the lobe handling.
      return new DegenerateTorus(readPlacement(t, ref(r.params[1]!), s), num(r.params[2]!) * s, num(r.params[3]!) * s,
        r.params[4]?.k === "enum" ? r.params[4].v === "T" : true);
    case "OFFSET_SURFACE": {
      // The basis is read with the file's angle unit too (a degree-unit cone basis otherwise gets
      // a semi-angle 57x too large, like the plain CONICAL_SURFACE case above).
      const base = makeSurface(t, ref(r.params[1]!), s, aRad);
      return base ? new OffsetSurface(base, num(r.params[2]!) * s) : null;
    }
    default:
      return null; // unsupported surface kind
  }
}

export { Sphere, BSplineSurface };
export const isSphere = (s: Surface): s is Sphere => s.kind === "SPHERICAL_SURFACE";
export const isBSpline = (s: Surface): s is BSplineSurface => s.kind === "B_SPLINE_SURFACE";

/** Analytic identity of a face surface, for measurement tools. Params in mm / radians. */
export interface SurfaceInfo {
  kind: string; // STEP kind: PLANE | CYLINDRICAL_SURFACE | CONICAL_SURFACE | ... ("" for complex)
  origin?: Vec3;
  /** Plane normal / cylinder-cone-revolution axis. */
  axis?: Vec3;
  radius?: number;
  semiAngle?: number;
}

/**
 * Extract the analytic parameters of a face surface (placement origin, axis, radius) without
 * building an evaluable Surface. Mirrors `makeSurface`'s record layout for the fixed quadrics;
 * swept/B-spline/complex surfaces report only their kind.
 */
export function analyzeSurface(t: Table, id: number, s: number, aRad = 1): SurfaceInfo {
  const kind = t.typeOf(id);
  if (!kind) return { kind: "" };
  const r = t.record(id);
  switch (kind) {
    case "PLANE": {
      const f = readPlacement(t, ref(r.params[1]!), s);
      return { kind, origin: f.o, axis: f.z };
    }
    case "CYLINDRICAL_SURFACE": {
      const f = readPlacement(t, ref(r.params[1]!), s);
      return { kind, origin: f.o, axis: f.z, radius: num(r.params[2]!) * s };
    }
    case "CONICAL_SURFACE": {
      const f = readPlacement(t, ref(r.params[1]!), s);
      return { kind, origin: f.o, axis: f.z, radius: num(r.params[2]!) * s, semiAngle: num(r.params[3]!) * aRad };
    }
    case "SPHERICAL_SURFACE": {
      const f = readPlacement(t, ref(r.params[1]!), s);
      return { kind, origin: f.o, radius: num(r.params[2]!) * s };
    }
    case "TOROIDAL_SURFACE":
    case "DEGENERATE_TOROIDAL_SURFACE": {
      const f = readPlacement(t, ref(r.params[1]!), s);
      return { kind, origin: f.o, axis: f.z, radius: num(r.params[2]!) * s };
    }
    default:
      return { kind };
  }
}
