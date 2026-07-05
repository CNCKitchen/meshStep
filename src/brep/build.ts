// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — build a topological BREP model from the parsed STEP entity table.
// Exposes faces -> loops -> oriented edges, plus a shared edge table (sampled once each so
// the two faces meeting at an edge get identical points => watertight seams).
import type { Vec3 } from "../geom/vec.ts";
import { Table, ref, refList, enumOf } from "../step/entities.ts";
import { parseStep } from "../step/parser.ts";
import { detectUnits, contextLengthScales, type Units } from "../step/units.ts";
import { readPoint, readPlacement, type Frame } from "../geom/placement.ts";
import { makeCurve } from "../geom/curves.ts";

export interface BEdge {
  v0: Vec3;
  v1: Vec3;
  curveId: number;
  sameSense: boolean;
  /** mm per unit for this edge's curve geometry, when its solid's representation context differs
   * from the file-global unit. Samplers must use this over the global scale. */
  scale?: number;
}

export interface OrientedEdge {
  edgeId: number;
  /** True if the loop traverses the edge in its v0->v1 direction. */
  orient: boolean;
}

export interface BLoop {
  outer: boolean;
  edges: OrientedEdge[];
}

export interface BFace {
  faceId: number;
  surfaceId: number;
  surfaceKind: string;
  /** ADVANCED_FACE.same_sense: true if the face normal agrees with the surface normal. */
  sameSense: boolean;
  loops: BLoop[];
}

export interface BSolid {
  id: number;
  faces: BFace[];
  /** World placement from the STEP assembly tree (identity for a single part). Applied to the final
   * mesh AFTER tessellation/remesh, since the analytic surfaces stay in each part's local frame.
   * Equal to instances[0] when the part occurs in the assembly. */
  transform?: Frame;
  /** ALL world placements of this part (one per assembly occurrence). A part used N times in the
   * assembly is meshed once and replicated at each frame; absent = single occurrence at identity. */
  instances?: Frame[];
  /** mm per length unit of this solid's own representation context, when it differs from the
   * file-global unit (Inventor mixes plain-METRE part reps into a millimetre assembly). Geometry
   * readers must use this over the global scale for the solid's points/curves/surfaces. */
  scale?: number;
}

// ---- Rigid transform (a Frame is columns x,y,z + origin o): world = o + x·p₀ + y·p₁ + z·p₂ ----
const IDENT: Frame = { o: [0, 0, 0], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
const rot = (f: Frame, d: Vec3): Vec3 => [
  f.x[0] * d[0] + f.y[0] * d[1] + f.z[0] * d[2],
  f.x[1] * d[0] + f.y[1] * d[1] + f.z[1] * d[2],
  f.x[2] * d[0] + f.y[2] * d[1] + f.z[2] * d[2],
];
const applyF = (f: Frame, p: Vec3): Vec3 => { const r = rot(f, p); return [r[0] + f.o[0], r[1] + f.o[1], r[2] + f.o[2]]; };
const composeF = (a: Frame, b: Frame): Frame => ({ o: applyF(a, b.o), x: rot(a, b.x), y: rot(a, b.y), z: rot(a, b.z) });
const invF = (f: Frame): Frame => {
  const t: Frame = { o: [0, 0, 0], x: [f.x[0], f.y[0], f.z[0]], y: [f.x[1], f.y[1], f.z[1]], z: [f.x[2], f.y[2], f.z[2]] };
  const o = rot(t, f.o); t.o = [-o[0], -o[1], -o[2]]; return t;
};

/**
 * Resolve each solid's world placement(s) and unit scale from the STEP assembly graph: parts live
 * in their own SHAPE_REPRESENTATION and are positioned by
 * REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION chains (each an ITEM_DEFINED_TRANSFORMATION
 * mapping the child rep's frame into its parent's). A part USED N TIMES has N such relationships
 * with the same child rep — every path from its geometry rep to the assembly root is one world
 * placement, so a solid yields a LIST of instance frames (wallganizer: 258 occurrences of 25
 * bodies; keeping only one parent per child dropped 233 of them and picked arbitrary survivors).
 * Each representation may also declare its OWN length unit (Inventor mixes plain-METRE part reps
 * into a millimetre file): a rep's geometry AND the ITEM_DEFINED_TRANSFORMATION placement that
 * lives in it are read at that rep's scale, so composed translations come out in millimetres.
 * A single-part file has no relationships and every solid stays a single identity instance.
 */
function assemblyInfo(t: Table, s: number): {
  instances: Map<number, Frame[]>;
  solidScale: Map<number, number>;
} {
  // geometry rep (ABSR / SHAPE_REPRESENTATION / ...) -> the solid bodies it contains
  const repOfSolid = new Map<number, number>();
  const repCtx = new Map<number, number>();
  for (const ty of ["ADVANCED_BREP_SHAPE_REPRESENTATION", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION"]) {
    for (const [repId, rep] of t.byType(ty)) {
      if (rep.params[2]?.k === "ref") repCtx.set(repId, ref(rep.params[2]!));
      if (rep.params[1]?.k !== "list") continue;
      for (const item of refList(rep.params[1]!)) {
        const ty2 = t.typeOf(item);
        if (ty2 === "MANIFOLD_SOLID_BREP" || ty2 === "BREP_WITH_VOIDS") repOfSolid.set(item, repId);
      }
    }
  }
  const ctxScale = contextLengthScales(t);
  const scaleOfRep = (rep: number): number => {
    const c = repCtx.get(rep);
    return (c !== undefined ? ctxScale.get(c) : undefined) ?? s;
  };

  // child rep -> [{ parent rep, transform mapping child frame -> parent frame }] (one per occurrence)
  const parents = new Map<number, { rep: number; xf: Frame }[]>();
  for (const [id, rrwt] of t.byType("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")) {
    const rr = t.sub(id, "REPRESENTATION_RELATIONSHIP");
    if (!rr || rr.params[2]?.k !== "ref" || rr.params[3]?.k !== "ref") continue;
    const child = ref(rr.params[2]!), par = ref(rr.params[3]!);
    const idt = t.record(ref(rrwt.params[0]!)); // ITEM_DEFINED_TRANSFORMATION(name, desc, item1, item2)
    // item1 is a placement in the CHILD rep, item2 in the PARENT rep — each in its rep's own units.
    const p1 = readPlacement(t, ref(idt.params[2]!), scaleOfRep(child));
    const p2 = readPlacement(t, ref(idt.params[3]!), scaleOfRep(par));
    const arr = parents.get(child) ?? [];
    arr.push({ rep: par, xf: composeF(p2, invF(p1)) }); // maps item1 frame -> item2 frame
    parents.set(child, arr);
  }
  // identity SHAPE_REPRESENTATION_RELATIONSHIP links a placeholder rep to the geometry rep (ABSR)
  const equiv = new Map<number, number>();
  for (const [id, srr] of t.byType("SHAPE_REPRESENTATION_RELATIONSHIP")) {
    if (t.isComplex(id) || srr.params[2]?.k !== "ref" || srr.params[3]?.k !== "ref") continue;
    const a = ref(srr.params[2]!), b = ref(srr.params[3]!);
    equiv.set(a, b); equiv.set(b, a);
  }
  const resolve = (rep: number): number => (parents.has(rep) ? rep : (equiv.get(rep) ?? rep));

  // Every root path of a rep is one world placement. Memoised DFS over the (acyclic) parent links;
  // a cycle or an explosion of paths (malformed graph) degrades to identity-only rather than hanging.
  const memo = new Map<number, Frame[]>();
  const onPath = new Set<number>();
  const MAX_INSTANCES = 4096;
  const worldsOf = (rep0: number): Frame[] => {
    const rep = resolve(rep0);
    const got = memo.get(rep);
    if (got) return got;
    const links = parents.get(rep);
    if (!links || onPath.has(rep)) return [IDENT];
    onPath.add(rep);
    const out: Frame[] = [];
    for (const p of links) {
      for (const w of worldsOf(p.rep)) {
        out.push(composeF(w, p.xf));
        if (out.length >= MAX_INSTANCES) break;
      }
      if (out.length >= MAX_INSTANCES) break;
    }
    onPath.delete(rep);
    memo.set(rep, out.length ? out : [IDENT]);
    return memo.get(rep)!;
  };

  const instances = new Map<number, Frame[]>();
  const solidScale = new Map<number, number>();
  for (const [solidId, geomRep] of repOfSolid) {
    const worlds = worldsOf(geomRep);
    if (worlds.length > 1 || worlds[0] !== IDENT) instances.set(solidId, worlds);
    const sc = scaleOfRep(geomRep);
    if (sc !== s) solidScale.set(solidId, sc);
  }
  return { instances, solidScale };
}

export interface BrepModel {
  solids: BSolid[];
  edges: Map<number, BEdge>;
  units: Units;
  table: Table;
  scale: number;
}

const vertexPoint = (t: Table, id: number, s: number): Vec3 =>
  readPoint(t, ref(t.record(id).params[1]!), s); // VERTEX_POINT(name, point#)

export function buildBrep(src: string): BrepModel {
  const table = new Table(parseStep(src));
  const units = detectUnits(table);
  const s = units.mmPerUnit;

  // Shared edge table: every EDGE_CURVE sampled by id, used by both adjacent faces.
  const edges = new Map<number, BEdge>();
  for (const [id, ec] of table.byType("EDGE_CURVE")) {
    edges.set(id, {
      v0: vertexPoint(table, ref(ec.params[1]!), s),
      v1: vertexPoint(table, ref(ec.params[2]!), s),
      curveId: ref(ec.params[3]!),
      sameSense: enumOf(ec.params[4]!) === "T",
    });
  }

  const readLoop = (loopId: number): OrientedEdge[] => {
    const loop = table.record(loopId);
    // VERTEX_LOOP(name, vertex#) is a degenerate single-point loop (a cone apex or sphere pole):
    // it has no edges, so it constrains nothing for meshing — the pole point is already carried by
    // the adjacent edges. POLY_LOOP (a raw point polygon) is likewise not edge-based. Skip both.
    if (loop.type !== "EDGE_LOOP") return [];
    return refList(loop.params[1]!).map((oeId) => {
      const oe = table.record(oeId); // ORIENTED_EDGE(name, *, *, edge#, orient)
      return { edgeId: ref(oe.params[3]!), orient: enumOf(oe.params[4]!) === "T" };
    });
  };

  // Resolve a shell ref to its faces. ORIENTED_CLOSED_SHELL('',*,base,orient) wraps a base shell;
  // orient=.F. reverses every face (e.g. a void's walls point into the cavity). `flip` propagates.
  const resolveShellFaces = (shellId: number, flip: boolean): { fid: number; flip: boolean }[] => {
    const rec = table.record(shellId);
    if (rec.type === "ORIENTED_CLOSED_SHELL") {
      return resolveShellFaces(ref(rec.params[2]!), flip !== (enumOf(rec.params[3]!) !== "T"));
    }
    return refList(rec.params[1]!).map((fid) => ({ fid, flip }));
  };
  const buildFace = (fid: number, flip: boolean): BFace => {
    let f = table.record(fid); // ADVANCED_FACE(name, (bound#...), surface#, sameSense)
    // ORIENTED_FACE(name, *, face#, orient) wraps a base face; orient=.F. reverses it.
    while (f.type === "ORIENTED_FACE") {
      flip = flip !== (enumOf(f.params[3]!) !== "T");
      fid = ref(f.params[2]!);
      f = table.record(fid);
    }
    const surfaceId = ref(f.params[2]!);
    const loops: BLoop[] = [];
    for (const bId of refList(f.params[1]!)) {
      const b = table.record(bId); // FACE_OUTER_BOUND | FACE_BOUND(name, loop#, orient)
      const edges = readLoop(ref(b.params[1]!));
      if (edges.length > 0) loops.push({ outer: b.type === "FACE_OUTER_BOUND", edges });
    }
    return {
      faceId: fid, surfaceId, sameSense: (enumOf(f.params[3]!) === "T") !== flip,
      surfaceKind: table.typeOf(surfaceId) ?? "(complex/bspline-surface)", loops,
    };
  };

  const solids: BSolid[] = [];
  const addSolid = (sid: number, shellIds: number[]): void => {
    const faces: BFace[] = [];
    // A malformed face (unexpected record layout from an exotic kernel) must not kill the whole
    // file — drop it and let the tessellator report the gap via stats.
    for (const shellId of shellIds) for (const sf of resolveShellFaces(shellId, false)) {
      try { faces.push(buildFace(sf.fid, sf.flip)); } catch { /* skipped face */ }
    }
    if (faces.length > 0) solids.push({ id: sid, faces });
  };
  // MANIFOLD_SOLID_BREP(name, outer_shell); BREP_WITH_VOIDS(name, outer_shell, (void_shells)).
  for (const [sid, msb] of table.byType("MANIFOLD_SOLID_BREP")) addSolid(sid, [ref(msb.params[1]!)]);
  for (const [sid, bwv] of table.byType("BREP_WITH_VOIDS")) addSolid(sid, [ref(bwv.params[1]!), ...refList(bwv.params[2]!)]);

  // Shell-based surface models: SHELL_BASED_SURFACE_MODEL(name, (shell#...)) — open/closed shells of
  // ADVANCED_FACEs without a solid wrapper. Same face machinery, one synthetic solid per model.
  if (solids.length === 0) {
    for (const [sid, sbsm] of table.byType("SHELL_BASED_SURFACE_MODEL")) addSolid(sid, refList(sbsm.params[1]!));
  }

  // AP203-era bounded-surface models (GEOMETRIC_SET): CURVE_BOUNDED_SURFACE(name, basis#,
  // (boundary#...), implicit_outer) has no EDGE_CURVE topology; each boundary is a composite curve
  // of trimmed segments. Synthesize one edge per segment so the shared pipeline (sample once ->
  // param-grid tessellation) applies unchanged. Only consulted when the file has no solid B-rep.
  if (solids.length === 0) {
    const faces: BFace[] = [];
    for (const [cbsId, cbs] of table.byType("CURVE_BOUNDED_SURFACE")) {
      try {
        const surfaceId = ref(cbs.params[1]!);
        const loops: BLoop[] = [];
        for (const bId of refList(cbs.params[2]!)) {
          const b = table.record(bId); // (OUTER_)BOUNDARY_CURVE(name, (segment#...), self_intersect)
          const oedges: OrientedEdge[] = [];
          for (const segId of refList(b.params[1]!)) {
            const seg = table.record(segId); // COMPOSITE_CURVE_SEGMENT(transition, same_sense, curve#)
            const curveId = ref(seg.params[2]!);
            const c = makeCurve(table, curveId, s, units.radPerAngle);
            if (!c) continue;
            const sameSense = enumOf(seg.params[1]!) === "T";
            const a = c.evaluate(c.t0), z = c.evaluate(c.t1);
            edges.set(segId, { v0: sameSense ? a : z, v1: sameSense ? z : a, curveId, sameSense });
            oedges.push({ edgeId: segId, orient: true });
          }
          if (oedges.length > 0) loops.push({ outer: b.type === "OUTER_BOUNDARY_CURVE", edges: oedges });
        }
        if (loops.length > 0) {
          faces.push({ faceId: cbsId, surfaceId, sameSense: true, surfaceKind: table.typeOf(surfaceId) ?? "(complex)", loops });
        }
      } catch { /* skip malformed surface */ }
    }
    if (faces.length > 0) solids.push({ id: 0, faces });
  }

  // Position each part by its STEP assembly placement(s) (identity for a single-part file) and
  // pick up per-representation unit scales (mixed-unit assemblies).
  const { instances, solidScale } = assemblyInfo(table, s);
  for (const solid of solids) {
    const inst = instances.get(solid.id);
    if (inst) { solid.instances = inst; solid.transform = inst[0]; }
    const sc = solidScale.get(solid.id);
    if (sc !== undefined && sc !== s) {
      solid.scale = sc;
      // The shared edge table was read at the global scale — re-read this solid's edge endpoints at
      // its own scale and tag the edges so samplers scale their curves the same way. (An EDGE_CURVE
      // belongs to exactly one shell/solid, so per-solid rescaling cannot conflict.)
      for (const face of solid.faces) for (const lp of face.loops) for (const oe of lp.edges) {
        const e = edges.get(oe.edgeId);
        if (!e || e.scale === sc) continue;
        e.v0 = vertexPoint(table, ref(table.record(oe.edgeId).params[1]!), sc);
        e.v1 = vertexPoint(table, ref(table.record(oe.edgeId).params[2]!), sc);
        e.scale = sc;
      }
    }
  }

  return { solids, edges, units, table, scale: s };
}
