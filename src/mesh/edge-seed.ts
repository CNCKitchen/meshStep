// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — edge seeding for the isotropic mode: where the mesh nodes sit on every B-rep edge.
//
// The default pipeline gives each edge ONE maximum segment length and lets the per-curve samplers
// place points (uniform angle steps on conics, bisection on splines, a cut at the edge's
// vertices). That leaves 2:1 length jumps wherever a bisection stopped one level apart, a short
// stub where the cut lands just beside a sample, and an abrupt size change wherever a finely
// sampled edge meets a coarse one — and every one of those shows up as sliver triangles on BOTH
// faces of the edge, where no face mesher can repair them (the edge samples are shared, fixed).
//
// Here the nodes are placed the way an FE pre-processor seeds an edge:
//
//   1. GEOMETRY. The existing samplers trace the edge 16x finer than the chord tolerance. That
//      dense polyline G is only a ruler: it gives arc length and positions, and inherits all the
//      samplers' handling of shared curves, closed rings and vertex cuts.
//   2. INTRINSIC SIZE h0 at every point of G: the smallest of the max edge length, the size the
//      curve's own curvature needs, and the size the curvature of each adjacent FACE needs AT
//      THAT POINT (the default mode reads each face at (u,v) = (0,0)).
//   3. SIZE FIELD. All edges of a body contribute their (point, h0) as sources of one
//      growth-limited field (size-field.ts), so a fine edge pulls its neighbours' sizes down
//      smoothly, across vertices and across a narrow face alike.
//   4. SEEDING. With h(s) = min(h0, field) along the edge, the edge gets N = ∫ ds/h segments
//      (rounded) and its nodes sit at EQUAL steps of that integral: evenly spaced where h is
//      constant, geometrically graded where it changes, and never a stub — the rounding is
//      spread over the whole edge instead of landing on the last segment.
//
// The chord tolerance is a hard requirement on top: N never drops below what the curve's own
// curvature needs, and every segment is verified against G afterwards (see `seedSpan`).
//
// One sampling per edge, used verbatim by both faces: the watertightness invariant is untouched.
import type { Vec3 } from "../geom/vec.ts";
import type { BrepModel, BEdge } from "../brep/build.ts";
import type { Surface } from "../geom/surfaces.ts";
import { sampleEdgePolyline, analyzeEdgeCurve } from "../geom/curves.ts";
import { SizeField } from "./size-field.ts";

/** One face's tolerances (mm, mm, radians) — FaceTol in tessellate.ts. */
export interface SeedTol { chordTol: number; targetEdge: number; normalDev: number }

/** G is traced this many times finer than the chord tolerance, so a node interpolated on it is
 * off the true curve by at most 1/16 of the tolerance (circles are snapped back exactly). */
const DENSE = 16;
/** A direction change above this at one point of G is a corner of the curve, not curvature: it
 * always carries a node (a chord across it would cut the corner whatever the node count). */
const KINK = 30 * Math.PI / 180;

interface EdgeGeom {
  G: Vec3[];
  /** Cumulative arc length along G. */
  S: Float64Array;
  /** Intrinsic size at each point of G, clamped to [hMin, hMax]. */
  h0: Float64Array;
  /** 1 / (largest segment the curve's own chord and angle tolerances allow) at each point. */
  invTol: Float64Array;
  /** Indices into G that must carry a node: both ends plus every kink. */
  breaks: number[];
  circle: { c: Vec3; n: Vec3; r: number } | null;
  hMin: number;
  chordTol: number;
}

const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const mix = (a: Vec3, b: Vec3, t: number): Vec3 => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];

/** Largest edge length a surface of curvature radius R allows under the tolerances — the same
 * chord (sagitta) and angle rules as the default mode's `targetForRadius`, with both terms
 * floored at hMin (in the default mode only edge sampling floors the angle term). */
export function sizeForRadius(R: number, tol: SeedTol, hMin: number): number {
  if (!Number.isFinite(R)) return tol.targetEdge;
  return Math.max(hMin, Math.min(tol.targetEdge, Math.sqrt(8 * R * tol.chordTol), R * tol.normalDev));
}

export class EdgeSeeder {
  private brep: BrepModel;
  private faceSurf: Map<number, Surface | null>;
  private tolOf: (fid: number) => SeedTol;
  private global: SeedTol;
  private growth: number;
  private hMinOpt: number | undefined;
  /** Faces bounded by each edge, and the body the edge belongs to. */
  private facesOf = new Map<number, number[]>();
  private solidOf = new Map<number, number>();
  /** Fewest segments an edge may get so its loop keeps at least three distinct nodes. */
  private nMin = new Map<number, number>();
  private geom = new Map<number, EdgeGeom | null>();
  /** TWIN edges: distinct EDGE_CURVEs that trace the same curve between the same two points.
   * Exporters that do not share topology give each face its own copy of a common edge, and the
   * two faces only meet because both copies are sampled identically. Seeding depends on the
   * neighbourhood (each copy sees only its own face), so twins are seeded ONCE, on the lowest id
   * of the group with the smallest intrinsic size of all members, and the others copy its nodes.
   * Value: the group's canonical edge and whether this member runs the other way. */
  private twin = new Map<number, { of: number; reversed: boolean }>();
  /** Canonical twin edge -> the bodies of its other members. Each body has its own size field,
   * and a twin group that spans bodies (a sheet model: one body per face) must honour the fine
   * features of every one of them, so its seeding reads all those fields. */
  private twinSolids = new Map<number, number[]>();
  private seeded = new Map<number, Vec3[]>();
  /** Edges that must be seeded UNIFORMLY with (at least) this many segments: the cross sides of
   * mapped faces. Their node count is fixed by the opposite side and their nodes must sit at the
   * same relative positions as its nodes, so the rulings between them stay straight. */
  private uniform = new Map<number, number>();
  /** Edges whose nodes are PINNED at given fractions of their arc length (v0 -> v1, strictly
   * between 0 and 1): the second cross side of a mapped face, whose nodes must sit at the same
   * values of the face's curved parameter as the first side's. */
  private pinned = new Map<number, number[]>();
  /** One field per body: bodies are meshed and welded independently. */
  readonly fields = new Map<number, SizeField>();

  constructor(
    brep: BrepModel, faceSurf: Map<number, Surface | null>, tolOf: (fid: number) => SeedTol,
    global: SeedTol, growthRate: number, minEdge: number | undefined,
  ) {
    this.brep = brep; this.faceSurf = faceSurf; this.tolOf = tolOf; this.global = global;
    this.growth = growthRate; this.hMinOpt = minEdge;
    for (const solid of brep.solids) for (const face of solid.faces) for (const lp of face.loops) {
      for (const oe of lp.edges) {
        const fs = this.facesOf.get(oe.edgeId) ?? this.facesOf.set(oe.edgeId, []).get(oe.edgeId)!;
        if (!fs.includes(face.faceId)) fs.push(face.faceId);
        this.solidOf.set(oe.edgeId, solid.id);
        // A loop of one edge is a closed ring and needs three segments to enclose anything; in a
        // loop of two edges each needs two, or the loop collapses to a pair of coincident chords.
        const need = lp.edges.length === 1 ? 3 : lp.edges.length === 2 ? 2 : 1;
        if (need > (this.nMin.get(oe.edgeId) ?? 1)) this.nMin.set(oe.edgeId, need);
      }
    }
  }

  /** Smallest size the field may ask for: the option, or the default mode's sampling floor. */
  private hMinFor(tol: SeedTol): number {
    return this.hMinOpt !== undefined && this.hMinOpt > 0 ? Math.min(this.hMinOpt, tol.targetEdge) : Math.min(tol.targetEdge / 40, 30 * tol.chordTol);
  }

  /** Finest tolerances of the faces an edge bounds (the globals for an edge no face uses). */
  private edgeTol(id: number): SeedTol {
    const fs = this.facesOf.get(id);
    if (!fs || fs.length === 0) return this.global;
    let c = Infinity, t = Infinity, n = Infinity;
    for (const f of fs) { const ft = this.tolOf(f); c = Math.min(c, ft.chordTol); t = Math.min(t, ft.targetEdge); n = Math.min(n, ft.normalDev); }
    return { chordTol: c, targetEdge: t, normalDev: n };
  }

  /** Trace one edge and measure its intrinsic size. `f` > 1 divides every tolerance and size by
   * f (the micro-face densify pre-pass asks for an edge at 4x / 16x / 64x density). */
  private measure(id: number, f = 1): EdgeGeom | null {
    const e = this.brep.edges.get(id) as BEdge;
    const base = this.edgeTol(id);
    const tol: SeedTol = { chordTol: base.chordTol / f, targetEdge: base.targetEdge / f, normalDev: base.normalDev };
    const hMin = this.hMinFor(base) / f, hMax = tol.targetEdge;
    const sc = e.scale ?? this.brep.scale;
    const raw = sampleEdgePolyline(this.brep.table, e.curveId, e.v0, e.v1, e.sameSense, sc,
      tol.chordTol / DENSE, hMax / 2, this.brep.units.radPerAngle, tol.normalDev / 4);
    // Drop repeated points (a zero-length step has no direction); the ends always stay.
    const G: Vec3[] = [raw[0]!];
    for (let i = 1; i < raw.length; i++) {
      if (d3(raw[i]!, G[G.length - 1]!) > 1e-12) G.push(raw[i]!);
      else if (i === raw.length - 1) G[G.length - 1] = raw[i]!;
    }
    const n = G.length;
    if (n < 2) return null;
    const S = new Float64Array(n);
    for (let i = 1; i < n; i++) S[i] = S[i - 1]! + d3(G[i - 1]!, G[i]!);
    if (!(S[n - 1]! > 1e-12)) return null;

    // A circle keeps its exact radius and has no corners; everything else is measured from G.
    let circle: EdgeGeom["circle"] = null;
    try {
      const info = analyzeEdgeCurve(this.brep.table, e.curveId, e.v0, e.v1, e.sameSense, sc);
      if (info.kind === "circle" && info.center && info.axis && info.radius! > 0) circle = { c: info.center, n: info.axis, r: info.radius! };
    } catch { /* unreadable curve record: G is still valid, treat as a general curve */ }
    // Point of G at arc length s (used for the lagged curvature and corner tests below).
    const at = (s: number): Vec3 => {
      let lo = 0, hi = n - 1;
      while (hi - lo > 1) { const m = (lo + hi) >> 1; if (S[m]! <= s) lo = m; else hi = m; }
      const l = S[hi]! - S[lo]!;
      return l > 0 ? mix(G[lo]!, G[hi]!, Math.min(1, Math.max(0, (s - S[lo]!) / l))) : G[lo]!;
    };
    /** Turn angle and circumradius of the three points a, b, c. */
    const bend = (a: Vec3, b: Vec3, c: Vec3): [number, number] => {
      const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
      const vx = c[0] - b[0], vy = c[1] - b[1], vz = c[2] - b[2];
      const lu = Math.hypot(ux, uy, uz), lv = Math.hypot(vx, vy, vz);
      const cr = Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
      const turn = Math.atan2(cr, ux * vx + uy * vy + uz * vz);
      // circumradius = |a-c| / (2 sin(turn)), with sin(turn) = cr / (lu·lv)
      return [turn, cr > 1e-14 * lu * lv ? (d3(a, c) * lu * lv) / (2 * cr) : Infinity];
    };
    const L = S[n - 1]!;
    const breaks: number[] = [0];
    const R = new Float64Array(n).fill(Infinity);
    // The curve's own tolerance step for a radius: the chord's sagitta and twice the normal
    // deviation, the same rule as the default arc sampler.
    const stepFor = (r: number): number => {
      if (!Number.isFinite(r)) return Infinity;
      const dChord = 2 * Math.acos(Math.max(0, Math.min(1, 1 - tol.chordTol / Math.max(r, 1e-12))));
      return r * Math.min(dChord, 2 * tol.normalDev);
    };
    if (circle) { R.fill(circle.r); breaks.push(n - 1); }
    else {
      // CORNERS. A corner of the curve (a spline with a tangent break, a polyline) always carries
      // a node; it is not curvature and must not pull the size down around it. It is judged
      // between two chords of length hMin, the smallest step the seeding ever takes: a bend
      // tighter than that cannot be resolved by nodes anyway, while exporter noise — a sub-micron
      // spike on a blend rail turns the polyline by tens of degrees over a micro-segment —
      // disappears at that scale. One node per corner: the local maximum of the turn.
      const lagC = hMin;
      const turnAt = new Float64Array(n);
      if (L > 4 * lagC) {
        for (let i = 1; i < n - 1; i++) {
          if (S[i]! < lagC || L - S[i]! < lagC) continue;
          turnAt[i] = bend(at(S[i]! - lagC), G[i]!, at(S[i]! + lagC))[0];
        }
        const limit = Math.max(KINK, 2 * tol.normalDev);
        for (let i = 1; i < n - 1; i++) {
          if (turnAt[i]! <= limit) continue;
          let isMax = true;
          for (let j = i - 1; j > 0 && S[i]! - S[j]! <= lagC && isMax; j--) if (turnAt[j]! > turnAt[i]!) isMax = false;
          for (let j = i + 1; j < n - 1 && S[j]! - S[i]! <= lagC && isMax; j++) if (turnAt[j]! >= turnAt[i]!) isMax = false;
          if (isMax) breaks.push(i);
        }
      }
      breaks.push(n - 1);
      // CURVATURE, per smooth piece, from a three-point circle through the point and the curve
      // one LAG to either side. The lag starts at hMin/2 and grows to half the step the radius
      // found so far allows, until the two agree: the radius is then measured over the very
      // chord it is going to permit. Read over consecutive points of G instead, it would be the
      // curvature of the exporter's noise.
      for (let b = 0; b + 1 < breaks.length; b++) {
        const sLo = S[breaks[b]!]!, sHi = S[breaks[b + 1]!]!;
        for (let i = breaks[b]! + 1; i < breaks[b + 1]!; i++) {
          let lag = hMin / 2, r = Infinity;
          for (let it = 0; it < 6; it++) {
            const sa = Math.max(sLo, S[i]! - lag), sb = Math.min(sHi, S[i]! + lag);
            if (!(S[i]! - sa > 1e-12) || !(sb - S[i]! > 1e-12)) break;
            r = bend(at(sa), G[i]!, at(sb))[1];
            const next = Math.min(hMax, stepFor(r)) / 2;
            if (!(next > 1.5 * lag)) break;
            lag = next;
          }
          R[i] = r;
        }
      }
      // Ends and corners have no two-sided neighbourhood of their own: take the radius of the
      // neighbour inside the same smooth piece (a corner's two sides may differ, the tighter
      // one is the safe choice).
      const isBreak = new Uint8Array(n);
      for (const b of breaks) isBreak[b] = 1;
      for (const b of breaks) {
        let r = Infinity;
        if (b + 1 < n && !isBreak[b + 1]) r = Math.min(r, R[b + 1]!);
        if (b - 1 >= 0 && !isBreak[b - 1]) r = Math.min(r, R[b - 1]!);
        R[b] = r;
      }
    }
    const invTol = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const step = stepFor(R[i]!);
      invTol[i] = Number.isFinite(step) && step > 0 ? 1 / step : 0;
    }

    // Intrinsic size: curve tolerance, adjacent faces' curvature at the point, max edge.
    const h0 = new Float64Array(n);
    for (let i = 0; i < n; i++) h0[i] = invTol[i]! > 0 ? Math.min(hMax, 1 / invTol[i]!) : hMax;
    for (const fid of this.facesOf.get(id) ?? []) {
      const surface = this.faceSurf.get(fid);
      if (!surface || surface.kind === "PLANE") continue;
      const ft0 = this.tolOf(fid);
      const ft: SeedTol = { chordTol: ft0.chordTol / f, targetEdge: ft0.targetEdge / f, normalDev: ft0.normalDev };
      if (surface.kind === "CYLINDRICAL_SURFACE" || surface.kind === "SPHERICAL_SURFACE") {
        const h = sizeForRadius(surface.curvatureRadius(0, 0), ft, hMin);
        for (let i = 0; i < n; i++) if (h < h0[i]!) h0[i] = h;
        continue;
      }
      // Curvature varies over the face: read it at stations along the edge, chaining each
      // projection from the previous one, and bisect where the size changes quickly.
      let ss: number[] = [], hs: number[] = [];
      let hu: number | undefined, hv: number | undefined;
      const sizeAt = (s: number): number => {
        try {
          const q = surface.project(at(s), hu, hv);
          hu = q[0]; hv = q[1];
          return sizeForRadius(surface.curvatureRadius(q[0], q[1]), ft, hMin);
        } catch { return ft.targetEdge; }
      };
      for (let k = 0; k <= 8; k++) { ss.push((L * k) / 8); hs.push(sizeAt((L * k) / 8)); }
      for (let round = 0; round < 3; round++) {
        const s2: number[] = [ss[0]!], h2: number[] = [hs[0]!];
        let added = false;
        for (let k = 1; k < ss.length; k++) {
          const lo = Math.min(hs[k - 1]!, hs[k]!);
          if (Math.abs(hs[k]! - hs[k - 1]!) > 0.15 * lo && ss[k]! - ss[k - 1]! > Math.max(hMin, 0.5 * lo)) {
            const sm = (ss[k - 1]! + ss[k]!) / 2;
            s2.push(sm); h2.push(sizeAt(sm)); added = true;
          }
          s2.push(ss[k]!); h2.push(hs[k]!);
        }
        ss = s2; hs = h2;
        if (!added) break;
      }
      let k = 1;
      for (let i = 0; i < n; i++) {
        while (k < ss.length - 1 && ss[k]! < S[i]!) k++;
        const w = ss[k]! > ss[k - 1]! ? Math.min(1, Math.max(0, (S[i]! - ss[k - 1]!) / (ss[k]! - ss[k - 1]!))) : 0;
        const h = hs[k - 1]! + (hs[k]! - hs[k - 1]!) * w;
        if (h < h0[i]!) h0[i] = h;
      }
    }
    // A piece between two forced nodes cannot hold a segment longer than itself, so its own
    // length caps its size: the neighbours then grade toward a short edge instead of meeting
    // it with a full-size segment. Clamp to [hMin, hMax] last.
    for (let b = 0; b + 1 < breaks.length; b++) {
      const len = S[breaks[b + 1]!]! - S[breaks[b]!]!;
      for (let i = breaks[b]!; i <= breaks[b + 1]!; i++) if (len < h0[i]!) h0[i] = len;
    }
    for (let i = 0; i < n; i++) h0[i] = Math.max(hMin, Math.min(hMax, h0[i]!));
    return { G, S, h0, invTol, breaks, circle, hMin, chordTol: tol.chordTol };
  }

  /** Point of G at arc length s. */
  private static pointAt(g: EdgeGeom, s: number): Vec3 {
    const { G, S } = g;
    let lo = 0, hi = G.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (S[m]! <= s) lo = m; else hi = m; }
    const l = S[hi]! - S[lo]!;
    return l > 0 ? mix(G[lo]!, G[hi]!, Math.min(1, Math.max(0, (s - S[lo]!) / l))) : G[lo]!;
  }

  /** Group twin edges and give each group's canonical edge the smallest intrinsic size any
   * member asks for. Twins are recognised by their end points, their length and three points
   * along the curve (all at the weld scale), never by curve records: two exporters' copies of
   * one edge rarely share an entity. */
  private pairTwins(): void {
    const q = (x: number): number => Math.round(x * 1e4);
    const pk = (p: Vec3): string => `${q(p[0])},${q(p[1])},${q(p[2])}`;
    const groups = new Map<string, number[]>();
    for (const [id, g] of this.geom) {
      if (!g) continue;
      const L = g.S[g.G.length - 1]!;
      const a = pk(g.G[0]!), b = pk(g.G[g.G.length - 1]!);
      const q1 = pk(EdgeSeeder.pointAt(g, L / 4)), q3 = pk(EdgeSeeder.pointAt(g, (3 * L) / 4));
      // Direction-independent key: sorted ends, the midpoint, the sorted quarter points.
      const key = `${a < b ? a + "|" + b : b + "|" + a}|${pk(EdgeSeeder.pointAt(g, L / 2))}|${q1 < q3 ? q1 + "|" + q3 : q3 + "|" + q1}|${q(L)}`;
      (groups.get(key) ?? groups.set(key, []).get(key)!).push(id);
    }
    for (const ids of groups.values()) {
      if (ids.length < 2) continue;
      ids.sort((x, y) => x - y);
      const canon = this.geom.get(ids[0]!)!;
      const L = canon.S[canon.G.length - 1]!;
      const c1 = EdgeSeeder.pointAt(canon, L / 4);
      for (let k = 1; k < ids.length; k++) {
        const g = this.geom.get(ids[k]!)!;
        const Lg = g.S[g.G.length - 1]!;
        const reversed = d3(EdgeSeeder.pointAt(g, Lg / 4), c1) > d3(EdgeSeeder.pointAt(g, (3 * Lg) / 4), c1);
        this.twin.set(ids[k]!, { of: ids[0]!, reversed });
        const so = this.solidOf.get(ids[k]!);
        if (so !== undefined && so !== this.solidOf.get(ids[0]!)) {
          const list = this.twinSolids.get(ids[0]!) ?? this.twinSolids.set(ids[0]!, []).get(ids[0]!)!;
          if (!list.includes(so)) list.push(so);
        }
        this.nMin.set(ids[0]!, Math.max(this.nMin.get(ids[0]!) ?? 1, this.nMin.get(ids[k]!) ?? 1));
        // Pointwise minimum of the two intrinsic sizes (and tolerance steps), read by arc length.
        for (let i = 0; i < canon.G.length; i++) {
          const s = reversed ? Lg - (canon.S[i]! * Lg) / L : (canon.S[i]! * Lg) / L;
          let lo = 0, hi = g.G.length - 1;
          while (hi - lo > 1) { const m = (lo + hi) >> 1; if (g.S[m]! <= s) lo = m; else hi = m; }
          const l = g.S[hi]! - g.S[lo]!;
          const w = l > 0 ? Math.min(1, Math.max(0, (s - g.S[lo]!) / l)) : 0;
          const h = g.h0[lo]! + (g.h0[hi]! - g.h0[lo]!) * w, it = g.invTol[lo]! + (g.invTol[hi]! - g.invTol[lo]!) * w;
          if (h < canon.h0[i]!) canon.h0[i] = h;
          if (it > canon.invTol[i]!) canon.invTol[i] = it;
        }
        canon.hMin = Math.min(canon.hMin, g.hMin);
        canon.chordTol = Math.min(canon.chordTol, g.chordTol);
      }
    }
  }

  /** Measure every edge and build each body's size field. Call once, before `seed`. */
  prepare(): void {
    for (const solid of this.brep.solids) for (const face of solid.faces) for (const lp of face.loops) for (const oe of lp.edges) {
      if (!this.geom.has(oe.edgeId) && this.brep.edges.has(oe.edgeId)) this.geom.set(oe.edgeId, this.measure(oe.edgeId));
    }
    this.pairTwins();
    for (const solid of this.brep.solids) {
      let hMin = Infinity, hMax = 0;
      const ids = new Set<number>();
      for (const face of solid.faces) {
        const ft = this.tolOf(face.faceId);
        hMin = Math.min(hMin, this.hMinFor(ft)); hMax = Math.max(hMax, ft.targetEdge);
        for (const lp of face.loops) for (const oe of lp.edges) ids.add(oe.edgeId);
      }
      if (ids.size === 0) continue;
      const field = new SizeField(this.growth, hMin, hMax);
      for (const id of ids) {
        if (!this.brep.edges.has(id)) continue;
        // A twin contributes through its canonical edge (which already carries the group minimum).
        const g = this.geom.get(this.twin.get(id)?.of ?? id);
        if (!g) continue;
        // Sources every half local size along the edge: between two of them the field can
        // overshoot the intrinsic size by at most g·h/4, and the seeding below also reads the
        // edge's own intrinsic size directly.
        const { G, S, h0 } = g;
        let next = 0;
        for (let i = 0; i + 1 < G.length; i++) {
          const l = S[i + 1]! - S[i]!;
          while (next <= S[i + 1]!) {
            const w = l > 0 ? (next - S[i]!) / l : 0;
            const h = h0[i]! + (h0[i + 1]! - h0[i]!) * w;
            const p = mix(G[i]!, G[i + 1]!, w);
            field.add(p[0], p[1], p[2], h);
            next += Math.max(0.5 * h, 1e-9);
          }
        }
        const last = G.length - 1;
        field.add(G[last]![0], G[last]![1], G[last]![2], h0[last]!);
      }
      field.build();
      this.fields.set(solid.id, field);
    }
  }

  /** Seed edge `id` uniformly with at least `n` segments (a cross side of a mapped face).
   * Returns true if that changes the edge's seeding; the caller then re-reads every edge, since
   * twins copy the edge that changed. */
  requireUniform(id: number, n: number): boolean {
    const canon = this.twin.get(id)?.of ?? id;
    if ((this.uniform.get(canon) ?? 0) >= n) return false;
    this.uniform.set(canon, n);
    this.seeded.delete(canon);
    return true;
  }

  /** True once the edge's node positions are fixed by a mapped face (as its first cross side,
   * whose uniform nodes the second side copied, or as that second side). */
  isPinned(id: number): boolean { const c = this.twin.get(id)?.of ?? id; return this.pinned.has(c) || this.locked.has(c); }
  /** Mark an edge as the reference side of a mapped face: its current nodes may not move. */
  lock(id: number): void { this.locked.add(this.twin.get(id)?.of ?? id); }
  private locked = new Set<number>();
  /** The dense trace of the edge, v0 -> v1, with cumulative arc length (null: degenerate edge). */
  trace(id: number): { G: Vec3[]; S: Float64Array } | null {
    const tw = this.twin.get(id);
    const g = this.geom.get(tw?.of ?? id);
    if (!g) return null;
    if (!tw?.reversed) return { G: g.G, S: g.S };
    const n = g.G.length, L = g.S[n - 1]!;
    const S = new Float64Array(n);
    for (let i = 0; i < n; i++) S[i] = L - g.S[n - 1 - i]!;
    return { G: g.G.slice().reverse(), S };
  }
  /** Fix the interior nodes of edge `id` at the given arc-length fractions (ascending, v0 -> v1). */
  pin(id: number, fractions: number[]): void {
    const tw = this.twin.get(id);
    const canon = tw?.of ?? id;
    this.pinned.set(canon, tw?.reversed ? fractions.map((x) => 1 - x).reverse() : fractions.slice());
    this.seeded.delete(canon);
  }

  /** The nodes of edge `id`, v0 to v1 inclusive. `f` > 1 asks for f times the density. */
  seed(id: number, f = 1): Vec3[] {
    const tw = this.twin.get(id);
    if (tw) {
      // Copy the canonical edge's nodes; only the two ends are this edge's own vertices (equal to
      // the canonical ones at the weld scale, but they must be THIS edge's exact points).
      const base = this.seed(tw.of, f);
      const e = this.brep.edges.get(id) as BEdge;
      const out = tw.reversed ? base.slice().reverse() : base.slice();
      out[0] = e.v0; out[out.length - 1] = e.v1;
      return out;
    }
    if (f === 1) {
      const hit = this.seeded.get(id);
      if (hit) return hit;
      const out = this.seedOwn(id, 1);
      this.seeded.set(id, out);
      return out;
    }
    return this.seedOwn(id, f);
  }

  private seedOwn(id: number, f: number): Vec3[] {
    const e = this.brep.edges.get(id) as BEdge;
    const solid = this.solidOf.get(id);
    const field = solid !== undefined ? this.fields.get(solid) : undefined;
    if (!field) {
      // An edge no face uses: nothing meshes against it, keep the plain sampling.
      const t = this.global;
      return sampleEdgePolyline(this.brep.table, e.curveId, e.v0, e.v1, e.sameSense, e.scale ?? this.brep.scale, t.chordTol / f, t.targetEdge / f, this.brep.units.radPerAngle, t.normalDev);
    }
    const g = f === 1 ? this.geom.get(id) : this.measure(id, f);
    if (!g) return [e.v0, e.v1];
    // Size along the edge: this body's field, and for a twin group spanning bodies theirs too.
    const others = (this.twinSolids.get(id) ?? []).map((so) => this.fields.get(so)).filter((x): x is SizeField => !!x);
    const sizeAt = others.length === 0
      ? (x: number, y: number, z: number): number => field.at(x, y, z)
      : (x: number, y: number, z: number): number => { let h = field.at(x, y, z); for (const o of others) h = Math.min(h, o.at(x, y, z)); return h; };
    const out: Vec3[] = [g.G[0]!];
    const single = g.breaks.length === 2;
    const uni = f === 1 ? this.uniform.get(id) : undefined;
    const pins = f === 1 ? this.pinned.get(id) : undefined;
    if (pins) {
      const L = g.S[g.G.length - 1]!;
      for (const x of pins) out.push(EdgeSeeder.pointAt(g, x * L));
      out.push(g.G[g.G.length - 1]!);
    } else if (uni !== undefined) {
      // Uniform: one constant size over the whole edge (corners included, a cross side has
      // none worth a node of its own), so equal steps of the integral are equal steps of length.
      seedSpan(g, () => Infinity, 1, 0, g.G.length - 1, uni, 0, out, g.S[g.G.length - 1]! / uni);
    } else for (let b = 0; b + 1 < g.breaks.length; b++) {
      seedSpan(g, sizeAt, f, g.breaks[b]!, g.breaks[b + 1]!, single ? (this.nMin.get(id) ?? 1) : 1, 0, out);
    }
    if (g.circle) {
      // Interpolated nodes sit on G's chords, a hair inside the circle: put them back on it.
      const { c, n, r } = g.circle;
      for (let i = 1; i < out.length - 1; i++) {
        const p = out[i]!;
        let dx = p[0] - c[0], dy = p[1] - c[1], dz = p[2] - c[2];
        const along = dx * n[0] + dy * n[1] + dz * n[2];
        dx -= along * n[0]; dy -= along * n[1]; dz -= along * n[2];
        const l = Math.hypot(dx, dy, dz);
        if (l > 1e-12) out[i] = [c[0] + (dx * r) / l + along * n[0], c[1] + (dy * r) / l + along * n[1], c[2] + (dz * r) / l + along * n[2]];
      }
    }
    if (out.length < 2) return [e.v0, e.v1]; // never hand a face less than a segment
    out[0] = e.v0; out[out.length - 1] = e.v1;
    return out;
  }
}

/** Distance of p from segment a-b. */
function segDist(p: Vec3, a: Vec3, b: Vec3): number {
  const ex = b[0] - a[0], ey = b[1] - a[1], ez = b[2] - a[2];
  const l2 = ex * ex + ey * ey + ez * ez;
  let w = l2 > 0 ? ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey + (p[2] - a[2]) * ez) / l2 : 0;
  w = w < 0 ? 0 : w > 1 ? 1 : w;
  return Math.hypot(p[0] - a[0] - w * ex, p[1] - a[1] - w * ey, p[2] - a[2] - w * ez);
}

/**
 * Seed the smooth piece G[i0..i1] and append its nodes after i0 (through i1 inclusive) to `out`.
 *
 * Along the piece, h(s) = min(intrinsic size, field), and A(s) = ∫ ds/h counts how many
 * segments of the requested size fit up to s. The piece gets N segments and node k sits where
 * A = k·A_total/N, so every segment spans the same share of the integral.
 *
 * N is the larger of two counts. The SIZE count rounds A_total, biased slightly up so a segment
 * is never more than ~1.3x its requested size. The TOLERANCE count is the ceiling of the same
 * integral over the curve's own chord/angle step, the hard requirement. Because the tolerance
 * integral runs over an estimated curvature, the result is then CHECKED against G: if any
 * segment's chord is further than the chord tolerance from the points of G it spans, the piece
 * is re-seeded with more segments, and if that does not converge it is cut at the worst point
 * (a corner the kink test missed) and both halves are seeded on their own.
 */
function seedSpan(
  g: EdgeGeom, field: (x: number, y: number, z: number) => number, f: number, i0: number, i1: number, nMin: number, depth: number, out: Vec3[],
  uniformSize?: number,
): void {
  const { G, S, h0, invTol } = g;
  const uniform = uniformSize !== undefined;
  // Stations: the points of G, bisected until no step is longer than half the size at either of
  // its ends. A straight edge has few points of G, yet the field can fall to a fraction of the
  // edge's own size within one of them (a fine neighbour passing close by). The field grows at
  // most g per unit length, so once a step is that short the size cannot dip unseen inside it.
  const ss: number[] = [], ps: Vec3[] = [], A: number[] = [], AT: number[] = [];
  const sizeAt = uniform
    ? (): number => uniformSize!
    : (p: Vec3, hIntr: number): number => Math.max(g.hMin, Math.min(hIntr, field(p[0], p[1], p[2]) / f));
  let prevInv = 0, prevInvT = 0;
  const push = (s: number, p: Vec3, h: number, invT: number): void => {
    const inv = 1 / h;
    if (ss.length === 0) { A.push(0); AT.push(0); }
    else {
      const ds = s - ss[ss.length - 1]!;
      A.push(A[A.length - 1]! + 0.5 * (inv + prevInv) * ds);
      AT.push(AT[AT.length - 1]! + 0.5 * (invT + prevInvT) * ds);
    }
    ss.push(s); ps.push(p); prevInv = inv; prevInvT = invT;
  };
  // Emit the stations strictly inside (a, b); the caller emits a and b themselves.
  const between = (
    sa: number, pa: Vec3, ha: number, ia: number, ta: number,
    sb: number, pb: Vec3, hb: number, ib: number, tb: number, level: number,
  ): void => {
    if (level >= 10 || sb - sa <= 0.5 * Math.min(ha, hb)) return;
    const sm = (sa + sb) / 2, pm = mix(pa, pb, 0.5), im = (ia + ib) / 2, tm = (ta + tb) / 2;
    const hm = sizeAt(pm, im);
    between(sa, pa, ha, ia, ta, sm, pm, hm, im, tm, level + 1);
    push(sm, pm, hm, tm);
    between(sm, pm, hm, im, tm, sb, pb, hb, ib, tb, level + 1);
  };
  let hPrev = sizeAt(G[i0]!, h0[i0]!);
  push(S[i0]!, G[i0]!, hPrev, invTol[i0]!);
  for (let i = i0; i < i1; i++) {
    const hNext = sizeAt(G[i + 1]!, h0[i + 1]!);
    between(S[i]!, G[i]!, hPrev, h0[i]!, invTol[i]!, S[i + 1]!, G[i + 1]!, hNext, h0[i + 1]!, invTol[i + 1]!, 0);
    push(S[i + 1]!, G[i + 1]!, hNext, invTol[i + 1]!);
    hPrev = hNext;
  }
  const total = A[A.length - 1]!;
  // A uniform edge takes exactly the count it was given (the opposite side has the same); only
  // the deviation check below may still raise it.
  let N = uniform ? Math.max(nMin, 1) : Math.max(nMin, Math.ceil(AT[AT.length - 1]! - 1e-6), Math.ceil(total - 0.3), 1);

  const place = (n: number): Vec3[] => {
    const nodes: Vec3[] = [];
    let j = 1;
    for (let k = 1; k < n; k++) {
      const target = (total * k) / n;
      while (j < A.length - 1 && A[j]! < target) j++;
      const da = A[j]! - A[j - 1]!;
      const w = da > 0 ? (target - A[j - 1]!) / da : 0;
      nodes.push(mix(ps[j - 1]!, ps[j]!, w));
    }
    return nodes;
  };
  // Worst deviation of the node chords from G, and where (index into G) it occurs.
  const worst = (nodes: Vec3[]): { dev: number; at: number } => {
    let dev = 0, at = -1, k = 0;
    // Node k lies between G points; walk G once, switching chords as its arc length passes a node.
    const sNode: number[] = [];
    { let j = 1; for (let m = 1; m <= nodes.length; m++) { const target = (total * m) / (nodes.length + 1); while (j < A.length - 1 && A[j]! < target) j++; const da = A[j]! - A[j - 1]!; sNode.push(ss[j - 1]! + (da > 0 ? (target - A[j - 1]!) / da : 0) * (ss[j]! - ss[j - 1]!)); } }
    for (let i = i0 + 1; i < i1; i++) {
      while (k < sNode.length && sNode[k]! < S[i]!) k++;
      const a = k === 0 ? G[i0]! : nodes[k - 1]!, b = k === nodes.length ? G[i1]! : nodes[k]!;
      const d = segDist(G[i]!, a, b);
      if (d > dev) { dev = d; at = i; }
    }
    return { dev, at };
  };
  let nodes = place(N);
  if (i1 - i0 > 1) {
    let w = worst(nodes);
    for (let tries = 0; tries < 3 && w.dev > g.chordTol; tries++) {
      N = Math.max(N + 1, Math.ceil(N * 1.2));
      nodes = place(N);
      w = worst(nodes);
    }
    if (!uniform && w.dev > g.chordTol && w.at > i0 && w.at < i1 && depth < 8) {
      seedSpan(g, field, f, i0, w.at, 1, depth + 1, out);
      seedSpan(g, field, f, w.at, i1, 1, depth + 1, out);
      return;
    }
  }
  for (const p of nodes) out.push(p);
  out.push(G[i1]!);
}
