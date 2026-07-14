// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — measurement geometry: per-edge analytic identity (line/circle/ellipse params) plus
// the exact boundary polylines the mesh was built from, instance-placed like the final mesh.
// This is what lets a viewer snap to true CAD edges and report exact hole diameters — the
// polylines come from tessellate's own edge sampling, so they are coincident with rendered
// feature edges; the analytic params come straight from the STEP records.
import type { Vec3 } from "../geom/vec.ts";
import type { Frame } from "../geom/placement.ts";
import type { BrepModel } from "./build.ts";
import { analyzeEdgeCurve } from "../geom/curves.ts";
import { analyzeSurface } from "../geom/surfaces.ts";

export interface MeasureEdge {
  edgeId: number;
  solidId: number;
  /** Assembly occurrence index (0-based) this record is placed for — matches SolidInstance.instance. */
  instance: number;
  kind: "line" | "circle" | "ellipse" | "other";
  /** Edge length in mm — analytic where closed-form (line, circular arc), else polyline sum. */
  length: number;
  /** Circle/ellipse center and plane normal (instance-placed mm / unit). */
  center?: [number, number, number];
  /** Circle/ellipse plane normal; unit direction for a line edge. */
  axis?: [number, number, number];
  /** Circle radius / ellipse semi-major. */
  radius?: number;
  /** Ellipse semi-minor. */
  radius2?: number;
  /** Signed traversed arc angle (rad); |sweep| ≈ 2π means a full circle. */
  sweep?: number;
  /** Adjacent B-rep face ids (1-2, same ids as faceOfTri). */
  faceIds: number[];
  /** Slice into MeasureGeometry.points: floats [first*3, (first+count)*3). */
  first: number;
  count: number;
}

export interface MeasureFace {
  faceId: number;
  solidId: number;
  /** Assembly occurrence index (0-based) this record is placed for — matches SolidInstance.instance. */
  instance: number;
  /** STEP surface kind (PLANE | CYLINDRICAL_SURFACE | ...); "" for complex/rational surfaces. */
  kind: string;
  origin?: [number, number, number];
  /** Plane normal / cylinder-cone axis (instance-placed unit vector). */
  axis?: [number, number, number];
  radius?: number;
  semiAngle?: number;
  /** ADVANCED_FACE.same_sense resolved to a sign: face normal = normalSign * surface normal. */
  normalSign: 1 | -1;
}

export interface MeasureGeometry {
  /** All edge polylines packed, 3 floats per point, instance-placed mm. Transferable. */
  points: Float32Array;
  edges: MeasureEdge[];
  faces: MeasureFace[];
  /** True when the polyline guard rail decimated freeform edges (pathological assemblies). */
  truncated?: boolean;
}

/** Edge-instance count above which freeform ("other") polylines are decimated to bound payload. */
const MAX_EDGE_INSTANCES = 250_000;
/** Max polyline points kept per freeform edge once the guard rail is active. */
const DECIMATED_POINTS = 8;

const IDENT: Frame = { o: [0, 0, 0], x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
const rot = (f: Frame, d: Vec3): Vec3 => [
  f.x[0] * d[0] + f.y[0] * d[1] + f.z[0] * d[2],
  f.x[1] * d[0] + f.y[1] * d[1] + f.z[1] * d[2],
  f.x[2] * d[0] + f.y[2] * d[1] + f.z[2] * d[2],
];
const applyF = (f: Frame, p: Vec3): Vec3 => { const r = rot(f, p); return [r[0] + f.o[0], r[1] + f.o[1], r[2] + f.o[2]]; };

const polylineLength = (pts: Vec3[]): number => {
  let l = 0;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]!, b = pts[i]!;
    l += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  }
  return l;
};

/** Stride-decimate a polyline to at most n points, always keeping both endpoints. */
const decimate = (pts: Vec3[], n: number): Vec3[] => {
  if (pts.length <= n) return pts;
  const out: Vec3[] = [];
  for (let i = 0; i < n; i++) out.push(pts[Math.round((i * (pts.length - 1)) / (n - 1))]!);
  return out;
};

/**
 * Collect measurement geometry for every edge and face of the model, replicated per assembly
 * occurrence with the same instance frames the mesh is placed by (`solidXf` in importStep) —
 * so a hole used ×N measures at each placed position, and solidIds match solidOfTri.
 * `edgePolylines` must be tessellate's own `edgePolylines` output (part-local mm).
 */
export function collectMeasureGeometry(
  brep: BrepModel,
  edgePolylines: Map<number, Vec3[]>,
  instances: Map<number, Frame[]>,
): MeasureGeometry {
  const edges: MeasureEdge[] = [];
  const faces: MeasureFace[] = [];

  // Pass 1: per-solid edge->face adjacency and raw (untransformed) edge records.
  interface RawEdge {
    edgeId: number; solidId: number; frames: Frame[];
    info: ReturnType<typeof analyzeEdgeCurve>;
    faceIds: number[]; polyline: Vec3[]; length: number;
  }
  const raw: RawEdge[] = [];
  let edgeInstances = 0;
  for (const solid of brep.solids) {
    const frames = instances.get(solid.id) ?? [IDENT];
    const scale = solid.scale ?? brep.scale;
    const edgeFaces = new Map<number, number[]>();
    for (const face of solid.faces) {
      const surfInfo = analyzeSurface(brep.table, face.surfaceId, scale, brep.units.radPerAngle);
      for (const [k, f] of frames.entries()) {
        faces.push({
          faceId: face.faceId, solidId: solid.id, instance: k, kind: surfInfo.kind,
          origin: surfInfo.origin ? applyF(f, surfInfo.origin) as [number, number, number] : undefined,
          axis: surfInfo.axis ? rot(f, surfInfo.axis) as [number, number, number] : undefined,
          radius: surfInfo.radius, semiAngle: surfInfo.semiAngle,
          normalSign: face.sameSense ? 1 : -1,
        });
      }
      for (const lp of face.loops) for (const oe of lp.edges) {
        const list = edgeFaces.get(oe.edgeId);
        if (list) { if (!list.includes(face.faceId)) list.push(face.faceId); }
        else edgeFaces.set(oe.edgeId, [face.faceId]);
      }
    }
    for (const [edgeId, faceIds] of edgeFaces) {
      const e = brep.edges.get(edgeId);
      const polyline = edgePolylines.get(edgeId);
      if (!e || !polyline || polyline.length < 2) continue; // healed-away micro-edge
      const info = analyzeEdgeCurve(brep.table, e.curveId, e.v0, e.v1, e.sameSense, e.scale ?? scale);
      raw.push({
        edgeId, solidId: solid.id, frames, info, faceIds, polyline,
        length: info.length ?? polylineLength(polyline),
      });
      edgeInstances += frames.length;
    }
  }

  // Guard rail: pathological assemblies decimate freeform polylines (analytic edges keep exact
  // params regardless, so snapping accuracy only degrades where no closed form exists anyway).
  const truncated = edgeInstances > MAX_EDGE_INSTANCES;
  if (truncated) {
    for (const r of raw) if (r.info.kind === "other") r.polyline = decimate(r.polyline, DECIMATED_POINTS);
  }

  // Pass 2: pack instance-placed polylines and emit final edge records.
  let totalPts = 0;
  for (const r of raw) totalPts += r.polyline.length * r.frames.length;
  const points = new Float32Array(totalPts * 3);
  let at = 0;
  for (const r of raw) {
    for (const [k, f] of r.frames.entries()) {
      const first = at;
      for (const p of r.polyline) {
        const w = applyF(f, p);
        points[at * 3] = w[0]; points[at * 3 + 1] = w[1]; points[at * 3 + 2] = w[2];
        at++;
      }
      edges.push({
        edgeId: r.edgeId, solidId: r.solidId, instance: k, kind: r.info.kind, length: r.length,
        center: r.info.center ? applyF(f, r.info.center) as [number, number, number] : undefined,
        axis: r.info.axis ? rot(f, r.info.axis) as [number, number, number]
          : r.info.dir ? rot(f, r.info.dir) as [number, number, number] : undefined,
        radius: r.info.radius, radius2: r.info.radius2, sweep: r.info.sweep,
        faceIds: r.faceIds, first, count: r.polyline.length,
      });
    }
  }

  return truncated ? { points, edges, faces, truncated } : { points, edges, faces };
}
