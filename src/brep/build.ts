// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — build a topological BREP model from the parsed STEP entity table.
// Exposes faces -> loops -> oriented edges, plus a shared edge table (sampled once each so
// the two faces meeting at an edge get identical points => watertight seams).
import type { Vec3 } from "../geom/vec.ts";
import { Table, ref, refList, enumOf } from "../step/entities.ts";
import { parseStep } from "../step/parser.ts";
import { detectUnits, type Units } from "../step/units.ts";
import { readPoint, readPlacement, type Frame } from "../geom/placement.ts";

export interface BEdge {
  v0: Vec3;
  v1: Vec3;
  curveId: number;
  sameSense: boolean;
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
   * mesh AFTER tessellation/remesh, since the analytic surfaces stay in each part's local frame. */
  transform?: Frame;
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
 * Resolve each MANIFOLD_SOLID_BREP's world placement from the STEP assembly graph: parts live in
 * their own SHAPE_REPRESENTATION and are positioned by REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION
 * chains (each an ITEM_DEFINED_TRANSFORMATION mapping the child rep's frame into its parent's). We
 * compose those transforms from each part up to the assembly root. Without this every part renders at
 * its own local origin (the assembly comes in disassembled). A single-part file has no such relations
 * and every solid stays identity.
 */
function assemblyTransforms(t: Table, s: number): Map<number, Frame> {
  // child rep -> { parent rep, transform mapping child frame -> parent frame }
  const parent = new Map<number, { rep: number; xf: Frame }>();
  for (const [id, rrwt] of t.byType("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")) {
    const rr = t.sub(id, "REPRESENTATION_RELATIONSHIP");
    if (!rr || rr.params[2]?.k !== "ref" || rr.params[3]?.k !== "ref") continue;
    const child = ref(rr.params[2]!), par = ref(rr.params[3]!);
    const idt = t.record(ref(rrwt.params[0]!)); // ITEM_DEFINED_TRANSFORMATION(name, desc, item1, item2)
    const p1 = readPlacement(t, ref(idt.params[2]!), s), p2 = readPlacement(t, ref(idt.params[3]!), s);
    parent.set(child, { rep: par, xf: composeF(p2, invF(p1)) }); // maps item1 frame -> item2 frame
  }
  // identity SHAPE_REPRESENTATION_RELATIONSHIP links a placeholder rep to the geometry rep (ABSR)
  const equiv = new Map<number, number>();
  for (const [id, srr] of t.byType("SHAPE_REPRESENTATION_RELATIONSHIP")) {
    if (t.isComplex(id) || srr.params[2]?.k !== "ref" || srr.params[3]?.k !== "ref") continue;
    const a = ref(srr.params[2]!), b = ref(srr.params[3]!);
    equiv.set(a, b); equiv.set(b, a);
  }
  // geometry rep (ABSR / SHAPE_REPRESENTATION) -> the MANIFOLD_SOLID_BREPs it contains
  const repOfSolid = new Map<number, number>();
  for (const ty of ["ADVANCED_BREP_SHAPE_REPRESENTATION", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION"]) {
    for (const [repId, rep] of t.byType(ty)) {
      if (rep.params[1]?.k !== "list") continue;
      for (const item of refList(rep.params[1]!)) if (t.typeOf(item) === "MANIFOLD_SOLID_BREP") repOfSolid.set(item, repId);
    }
  }
  const out = new Map<number, Frame>();
  for (const [solidId, geomRep] of repOfSolid) {
    let cur = parent.has(geomRep) ? geomRep : (equiv.get(geomRep) ?? geomRep);
    let world = IDENT, guard = 0; const seen = new Set<number>();
    while (parent.has(cur) && !seen.has(cur) && guard++ < 64) {
      seen.add(cur);
      const p = parent.get(cur)!;
      world = composeF(p.xf, world);
      cur = parent.has(p.rep) ? p.rep : (equiv.get(p.rep) ?? p.rep);
    }
    if (world !== IDENT) out.set(solidId, world);
  }
  return out;
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
    const f = table.record(fid); // ADVANCED_FACE(name, (bound#...), surface#, sameSense)
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
    for (const shellId of shellIds) for (const sf of resolveShellFaces(shellId, false)) faces.push(buildFace(sf.fid, sf.flip));
    if (faces.length > 0) solids.push({ id: sid, faces });
  };
  // MANIFOLD_SOLID_BREP(name, outer_shell); BREP_WITH_VOIDS(name, outer_shell, (void_shells)).
  for (const [sid, msb] of table.byType("MANIFOLD_SOLID_BREP")) addSolid(sid, [ref(msb.params[1]!)]);
  for (const [sid, bwv] of table.byType("BREP_WITH_VOIDS")) addSolid(sid, [ref(bwv.params[1]!), ...refList(bwv.params[2]!)]);

  // Position each part by its STEP assembly placement (identity for a single-part file).
  const xforms = assemblyTransforms(table, s);
  for (const solid of solids) { const xf = xforms.get(solid.id); if (xf) solid.transform = xf; }

  return { solids, edges, units, table, scale: s };
}
