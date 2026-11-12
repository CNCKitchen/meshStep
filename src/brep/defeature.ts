// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — geometry clean-up before meshing: edges shorter than a tolerance collapse to a point.
//
// A model often carries steps and offsets far smaller than anything the mesh is meant to show:
// two faces that should meet in one corner meet 0.05 mm apart, and the B-rep bridges the gap
// with an edge of that length. Every such edge must carry a mesh segment, so the mesh holds a
// 0.05 mm triangle there and (in the isotropic mode) grades the whole neighbourhood toward it.
// Merging the two end vertices removes the edge: its neighbours end in one shared point and the
// mesh passes the spot at the size of its surroundings. The surface moves by at most the
// tolerance there, which is the user's call to make: it is a separate, opt-in tolerance, not
// part of the surface deviation.
import type { Vec3 } from "../geom/vec.ts";
import { ref } from "../step/entities.ts";
import { sampleEdgePolyline } from "../geom/curves.ts";
import { makeSurface, type Surface } from "../geom/surfaces.ts";
import type { BrepModel, BFace, BSolid } from "./build.ts";

export interface MergeReport {
  /** Edges removed: collapsed to a point, or replaced by the edge on the other side of a sliver. */
  edges: number;
  /** Vertices moved onto another one. */
  vertices: number;
  /** Faces removed: specks that lost every edge, and slivers between two edges within tolerance. */
  faces: number;
  /** Largest distance the boundary of a face moved, mm. */
  maxMove: number;
}

const d3 = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/**
 * Collapse every edge shorter than `tol` (arc length, mm) in place: its end vertices become one
 * point and the edge leaves its loops. Vertices joined by a chain of short edges merge together
 * as long as the whole group stays within `tol` across; a longer chain is real geometry (a small
 * arc split into pieces) and is merged only in part. A face left between two edges that run
 * within `tol` of each other (the riser of a step) is removed and its neighbours share one edge.
 *
 * What is NOT merged: a short edge that is the whole thickness of a wall or the whole width of
 * a gap (the faces at its two ends look away from or at each other), and an edge whose merge
 * would pull a face beyond the end of its surface. Closed edges (a tiny hole) are left alone.
 */
export function mergeShortEdges(brep: BrepModel, tol: number): MergeReport {
  const report: MergeReport = { edges: 0, vertices: 0, faces: 0, maxMove: 0 };
  if (!(tol > 0)) return report;
  const { table, edges } = brep;

  // Edges in use, with the ids of their end vertices.
  const used = new Set<number>();
  for (const solid of brep.solids) for (const face of solid.faces) for (const lp of face.loops) for (const oe of lp.edges) used.add(oe.edgeId);
  const vids = new Map<number, [number, number]>();
  const vpos = new Map<number, Vec3>();
  for (const id of used) {
    const e = edges.get(id);
    if (!e || table.typeOf(id) !== "EDGE_CURVE") continue;
    const rec = table.record(id);
    if (rec.params[1]?.k !== "ref" || rec.params[2]?.k !== "ref") continue;
    const va = ref(rec.params[1]!), vb = ref(rec.params[2]!);
    vids.set(id, [va, vb]);
    vpos.set(va, e.v0); vpos.set(vb, e.v1);
  }

  const parent = new Map<number, number>();
  const find = (x: number): number => {
    let r = x;
    while ((parent.get(r) ?? r) !== r) r = parent.get(r)!;
    while ((parent.get(x) ?? x) !== x) { const nx = parent.get(x)!; parent.set(x, r); x = nx; }
    return r;
  };

  // Vertex records at the same coordinates are one corner and must move together (the mesh weld
  // would join them anyway; moving only one would tear the junction open).
  {
    const cell = 2e-6;
    const hash = new Map<string, number[]>();
    for (const [vid, p] of vpos) {
      const kx = Math.round(p[0] / cell), ky = Math.round(p[1] / cell), kz = Math.round(p[2] / cell);
      for (let ix = kx - 1; ix <= kx + 1; ix++) for (let iy = ky - 1; iy <= ky + 1; iy++) for (let iz = kz - 1; iz <= kz + 1; iz++) {
        for (const o of hash.get(`${ix},${iy},${iz}`) ?? []) {
          if (d3(p, vpos.get(o)!) <= cell) { const ra = find(vid), rb = find(o); if (ra !== rb) parent.set(Math.max(ra, rb), Math.min(ra, rb)); }
        }
      }
      const k = `${kx},${ky},${kz}`;
      (hash.get(k) ?? hash.set(k, []).get(k)!).push(vid);
    }
  }
  /** Corner (coincidence class) of a vertex record; fixed from here on. */
  const corner = new Map<number, number>();
  for (const vid of vpos.keys()) corner.set(vid, find(vid));
  const cpos = (c: number): Vec3 => vpos.get(c)!;

  // Arc length of the edges whose chord is short enough to qualify.
  const length = new Map<number, number>();
  for (const [id, [va, vb]] of vids) {
    const ca = corner.get(va)!, cb = corner.get(vb)!;
    if (ca === cb) continue; // closed edge: a rim, however small
    const e = edges.get(id)!;
    if (d3(e.v0, e.v1) >= tol) continue;
    let L = Infinity;
    try {
      const pts = sampleEdgePolyline(table, e.curveId, e.v0, e.v1, e.sameSense, e.scale ?? brep.scale, tol / 50, tol / 4, brep.units.radPerAngle);
      L = 0;
      for (let i = 1; i < pts.length; i++) L += d3(pts[i - 1]!, pts[i]!);
    } catch { /* unreadable curve: leave the edge alone */ }
    length.set(id, L);
  }
  const short = [...length].filter(([, L]) => L < tol).sort((a, b) => a[1] - b[1] || a[0] - b[0]);
  if (short.length === 0) return report;

  // Group corners along short edges, shortest first, while the group stays within tol across.
  const group = new Map<number, number>();           // corner -> group root
  const members = new Map<number, number[]>();       // group root -> corners
  const gfind = (c: number): number => group.get(c) ?? c;
  for (const [id] of short) {
    const [va, vb] = vids.get(id)!;
    const ga = gfind(corner.get(va)!), gb = gfind(corner.get(vb)!);
    if (ga === gb) continue;
    const ma = members.get(ga) ?? [ga], mb = members.get(gb) ?? [gb];
    let ok = true;
    for (const a of ma) { for (const b of mb) if (d3(cpos(a), cpos(b)) >= tol) { ok = false; break; } if (!ok) break; }
    if (!ok) continue;
    const root = Math.min(ga, gb), all = [...ma, ...mb];
    members.set(root, all); members.delete(Math.max(ga, gb));
    for (const c of all) group.set(c, root);
  }

  // Edges that collapse: both ends in one group and no longer than a few tolerances (a short
  // edge, or the third side of a speck). A LONGER edge with both ends in one group would turn
  // into a closed curve, which the samplers cannot be trusted with: such a group is left alone.
  const gone = new Set<number>();
  const blocked = new Set<number>();
  for (const [id, [va, vb]] of vids) {
    const ca = corner.get(va)!, cb = corner.get(vb)!;
    if (ca === cb) continue;
    const g = gfind(ca);
    if (g !== gfind(cb) || !members.has(g)) continue;
    if ((length.get(id) ?? Infinity) < 4 * tol) gone.add(id);
    else blocked.add(g);
  }
  for (const id of [...gone]) if (blocked.has(gfind(corner.get(vids.get(id)![0])!))) gone.delete(id);

  // The point each group collapses to. A corner can only take the group over if every face that
  // has to follow can reach it: the faces around the OTHER corners get a boundary point up to
  // tol away from where their own corner was. That is harmless when the point still lies over
  // the face's surface (a step: the point sits tol above it), and breaks the face when it lies
  // beyond the surface's end (an untrimmed spline patch stops at the old corner; the nearest
  // surface point is then somewhere on its border and the boundary folds onto itself). Also,
  // no surviving edge may lose more than half its chord to the move. Among the corners that
  // pass, the one most surviving edges end in wins (ties: the lowest vertex id), so as few
  // edges as possible are bent. A group no corner can take over stays as it is.
  const degree = new Map<number, number>();
  const edgesAt = new Map<number, number[]>();          // corner -> surviving edges ending there
  for (const [id, [va, vb]] of vids) {
    if (gone.has(id)) continue;
    for (const v of [va, vb]) {
      const c = corner.get(v)!;
      degree.set(c, (degree.get(c) ?? 0) + 1);
      (edgesAt.get(c) ?? edgesAt.set(c, []).get(c)!).push(id);
    }
  }
  const facesAt = new Map<number, Set<number>>();       // corner -> faces with a boundary edge there
  const faceRec = new Map<number, { face: BFace; scale: number }>();
  for (const solid of brep.solids) for (const face of solid.faces) {
    faceRec.set(face.faceId, { face, scale: solid.scale ?? brep.scale });
    for (const lp of face.loops) for (const oe of lp.edges) {
      const vv = vids.get(oe.edgeId);
      if (!vv) continue;
      for (const v of vv) { const c = corner.get(v)!; (facesAt.get(c) ?? facesAt.set(c, new Set()).get(c)!).add(face.faceId); }
    }
  }
  const surfaces = new Map<number, Surface | null>();
  const surfaceOf = (fid: number): Surface | null => {
    if (!surfaces.has(fid)) {
      const fr = faceRec.get(fid)!;
      let sf: Surface | null = null;
      try { sf = makeSurface(table, fr.face.surfaceId, fr.scale, brep.units.radPerAngle); } catch { /* unreadable surface */ }
      surfaces.set(fid, sf);
    }
    return surfaces.get(fid)!;
  };
  /** True when face `fid`, which has a corner at `from`, can take a boundary point at `p`. */
  const reaches = (fid: number, from: Vec3, p: Vec3): boolean => {
    const sf = surfaceOf(fid);
    if (!sf) return false;
    if (sf.kind === "PLANE") return true;
    try {
      const h = sf.project(from);
      const [u, v] = sf.project(p, h[0], h[1]);
      const q = sf.evaluate(u, v);
      const r: Vec3 = [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
      const rl = Math.hypot(r[0], r[1], r[2]);
      if (rl < 1e-6) return true;
      // Over the surface, the nearest point's offset runs along the normal; beyond its end,
      // the offset leans along the surface.
      const n = sf.normal(u, v);
      const along = r[0] * n[0] + r[1] * n[1] + r[2] * n[2];
      const lean = Math.sqrt(Math.max(0, rl * rl - along * along));
      return lean <= 0.25 * rl;
    } catch { return false; }
  };
  /** Outward normal of face `fid` at (the surface point nearest to) `p`. */
  const outward = (fid: number, p: Vec3): Vec3 | null => {
    const sf = surfaceOf(fid);
    if (!sf) return null;
    try {
      const [u, v] = sf.project(p);
      const n = sf.normal(u, v);
      return faceRec.get(fid)!.face.sameSense ? n : [-n[0], -n[1], -n[2]];
    } catch { return null; }
  };
  /** True when merging corners `a` and `b` would press two faces that look away from each other
   * (the two sides of a wall) or at each other (the two sides of a gap) together: the edge
   * between them is then the wall's or the gap's whole thickness, not a step in a surface. */
  const pinches = (a: number, b: number): boolean => {
    const fa = facesAt.get(a), fb = facesAt.get(b);
    if (!fa || !fb) return false;
    for (const f1 of fa) {
      if (fb.has(f1)) continue;
      const n1 = outward(f1, cpos(a));
      if (!n1) return true;
      for (const f2 of fb) {
        if (fa.has(f2)) continue;
        const n2 = outward(f2, cpos(b));
        if (!n2 || n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2] < -0.5) return true;
      }
    }
    return false;
  };
  const target = new Map<number, Vec3>();              // corner -> new position
  for (const [g, ms] of members) {
    if (blocked.has(g)) continue;
    let pinch = false;
    for (let i = 0; i < ms.length && !pinch; i++) for (let j = i + 1; j < ms.length && !pinch; j++) pinch = pinches(ms[i]!, ms[j]!);
    if (pinch) { blocked.add(g); continue; }
    let best = -1;
    for (const c of ms) {
      const p = cpos(c);
      let ok = true;
      for (const m of ms) {
        if (m === c || !ok) continue;
        for (const fid of facesAt.get(m) ?? []) if (!facesAt.get(c)?.has(fid) && !reaches(fid, cpos(m), p)) { ok = false; break; }
        for (const id of edgesAt.get(m) ?? []) {
          if (!ok) break;
          const [va, vb] = vids.get(id)!;
          const other = corner.get(va)! === m ? corner.get(vb)! : corner.get(va)!;
          if (gfind(other) === g) continue; // both ends move to the same point: handled above
          if (d3(p, cpos(other)) < 0.5 * d3(cpos(m), cpos(other))) ok = false;
        }
      }
      if (!ok) continue;
      const dc = degree.get(c) ?? 0, db = best < 0 ? -1 : degree.get(best) ?? 0;
      if (dc > db || (dc === db && c < best)) best = c;
    }
    if (best < 0) { blocked.add(g); continue; }
    const p = cpos(best);
    for (const c of ms) {
      if (c === best) continue;
      target.set(c, p);
      report.vertices++;
      report.maxMove = Math.max(report.maxMove, d3(cpos(c), p));
    }
  }
  for (const id of [...gone]) if (blocked.has(gfind(corner.get(vids.get(id)![0])!))) gone.delete(id);
  if (target.size === 0) return report;

  for (const [id, [va, vb]] of vids) {
    if (gone.has(id)) continue;
    const e = edges.get(id)!;
    const pa = target.get(corner.get(va)!), pb = target.get(corner.get(vb)!);
    if (pa) e.v0 = [pa[0], pa[1], pa[2]];
    if (pb) e.v1 = [pb[0], pb[1], pb[2]];
  }
  report.edges = gone.size;

  // Take the collapsed edges out of their loops; a loop or face left with nothing was a speck.
  const slivers: { solid: BSolid; face: BFace }[] = [];
  for (const solid of brep.solids) {
    const dead = new Set<BFace>();
    for (const face of solid.faces) {
      let touched = false;
      for (const loop of face.loops) {
        if (!loop.edges.some((oe) => gone.has(oe.edgeId))) continue;
        loop.edges = loop.edges.filter((oe) => !gone.has(oe.edgeId));
        touched = true;
      }
      if (!touched) continue;
      face.loops = face.loops.filter((lp) => lp.edges.length > 0);
      if (face.loops.length === 0) dead.add(face);
    }
    if (dead.size > 0) { solid.faces = solid.faces.filter((f) => !dead.has(f)); report.faces += dead.size; }
    for (const face of solid.faces) if (!dead.has(face) && face.loops.length === 1 && face.loops[0]!.edges.length === 2) slivers.push({ solid, face });
  }

  // SLIVERS. A face that lost its short ends is left between two long edges that now share both
  // end points: the riser of a step. When the two edges run within tol of each other all the
  // way, the face goes too and its two neighbours share ONE of the edges, so the step is gone
  // from the mesh altogether. (Left in, the face would be meshed as a strip of needles between
  // two edges seeded independently.) The neighbour that changes edges must be able to reach the
  // new one, as above; if neither edge can stand in for the other, the face stays.
  const polyline = (id: number): Vec3[] | null => {
    const e = edges.get(id)!;
    try {
      const chord = d3(e.v0, e.v1);
      return sampleEdgePolyline(table, e.curveId, e.v0, e.v1, e.sameSense, e.scale ?? brep.scale, tol / 20, Math.max(tol / 2, chord / 128), brep.units.radPerAngle);
    } catch { return null; }
  };
  /** Nearest point to `p` on polyline `pl`, and its distance. */
  const nearest = (p: Vec3, pl: Vec3[]): [Vec3, number] => {
    let best: Vec3 = pl[0]!, bd = Infinity;
    for (let i = 0; i + 1 < pl.length; i++) {
      const a = pl[i]!, b = pl[i + 1]!;
      const ex = b[0] - a[0], ey = b[1] - a[1], ez = b[2] - a[2];
      const l2 = ex * ex + ey * ey + ez * ez;
      let w = l2 > 0 ? ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey + (p[2] - a[2]) * ez) / l2 : 0;
      w = w < 0 ? 0 : w > 1 ? 1 : w;
      const q: Vec3 = [a[0] + w * ex, a[1] + w * ey, a[2] + w * ez];
      const d = d3(p, q);
      if (d < bd) { bd = d; best = q; }
    }
    return [best, bd];
  };
  const replaced = new Map<number, { by: number; flip: boolean }>();
  const busy = new Set<number>();                      // edges already part of a replacement
  for (const { solid, face } of slivers) {
    const o1 = face.loops[0]!.edges[0]!, o2 = face.loops[0]!.edges[1]!;
    if (o1.edgeId === o2.edgeId || busy.has(o1.edgeId) || busy.has(o2.edgeId)) continue;
    if (!vids.has(o1.edgeId) || !vids.has(o2.edgeId)) continue;
    const p1 = polyline(o1.edgeId), p2 = polyline(o2.edgeId);
    if (!p1 || !p2) continue;
    let far = 0;
    for (const p of p1) far = Math.max(far, nearest(p, p2)[1]);
    for (const p of p2) far = Math.max(far, nearest(p, p1)[1]);
    if (far >= tol) continue;
    // Faces (other than the sliver) on each edge.
    const users = (id: number): BFace[] => solid.faces.filter((f) => f !== face && f.loops.some((lp) => lp.edges.some((oe) => oe.edgeId === id)));
    const u1 = users(o1.edgeId), u2 = users(o2.edgeId);
    if (u1.some((f) => u2.includes(f))) continue;       // one face on both sides: a slit, not a step
    /** Can the faces on edge `drop` follow edge `keep` (polyline `pk`) instead? */
    const follows = (us: BFace[], pd: Vec3[], pk: Vec3[]): boolean => {
      for (const f of us) for (const k of [1, 2, 3]) {
        const p = pk[Math.floor((pk.length - 1) * k / 4)]!;
        if (!reaches(f.faceId, nearest(p, pd)[0], p)) return false;
      }
      return true;
    };
    let keep = o1, drop = o2;
    if (!follows(u2, p2, p1)) {
      if (!follows(u1, p1, p2)) continue;
      keep = o2; drop = o1;
    }
    // The loop runs along one edge and back along the other, so the two run the same way
    // exactly when their orientation flags differ.
    replaced.set(drop.edgeId, { by: keep.edgeId, flip: o1.orient === o2.orient });
    busy.add(o1.edgeId); busy.add(o2.edgeId);
    solid.faces = solid.faces.filter((f) => f !== face);
    report.faces++; report.edges++;
    report.maxMove = Math.max(report.maxMove, far);
  }
  if (replaced.size > 0) {
    for (const solid of brep.solids) for (const face of solid.faces) for (const loop of face.loops) {
      loop.edges = loop.edges.map((oe) => {
        const r = replaced.get(oe.edgeId);
        return r ? { ...oe, edgeId: r.by, orient: r.flip ? !oe.orient : oe.orient } : oe;
      });
    }
  }
  return report;
}
