// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — fast model-size estimate and size-adaptive tessellation defaults.
// Tessellation tolerances are absolute (mm), so one fixed default cannot fit both a 5 mm clip
// and a 3 m assembly. The estimate builds the B-rep topology (no tessellation) and measures the
// bounding box of every solid's EDGE VERTICES — points that lie ON the model — at each solid's
// own unit scale and through its assembly instance placements. A raw CARTESIAN_POINT scan is NOT
// safe here: STEP files carry unbounded construction geometry (an infinite LINE's origin point
// may sit anywhere along it — CATIA parks "Line Origine" points kilometers out), which inflated
// a 77 mm board to a 500 m estimate. Circle/ellipse edges additionally contribute their full
// circle box, so a lone cylinder's diameter isn't missed when its rim is one closed edge.
import { buildBrep } from "../brep/build.ts";
import { ref, num } from "./entities.ts";
import { readPlacement, type Frame } from "../geom/placement.ts";
import type { Vec3 } from "../geom/vec.ts";

export interface SizeEstimate {
  /** Approximate axis-aligned bbox in mm: [minX,minY,minZ, maxX,maxY,maxZ]. */
  bbox: [number, number, number, number, number, number];
  /** Bbox diagonal, mm — the "model size" the auto defaults scale with. */
  diag: number;
  /** Detected length-unit label (e.g. "mm", "inch"). */
  units: string;
}

class Box {
  min: [number, number, number] = [Infinity, Infinity, Infinity];
  max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  add(x: number, y: number, z: number): void {
    if (x < this.min[0]) this.min[0] = x; if (x > this.max[0]) this.max[0] = x;
    if (y < this.min[1]) this.min[1] = y; if (y > this.max[1]) this.max[1] = y;
    if (z < this.min[2]) this.min[2] = z; if (z > this.max[2]) this.max[2] = z;
  }
  get empty(): boolean { return this.min[0] === Infinity; }
}

const applyFrame = (f: Frame, x: number, y: number, z: number): Vec3 => [
  f.o[0] + f.x[0] * x + f.y[0] * y + f.z[0] * z,
  f.o[1] + f.x[1] * x + f.y[1] * y + f.z[1] * z,
  f.o[2] + f.x[2] * x + f.y[2] * y + f.z[2] * z,
];

/** Estimate a STEP model's size without tessellating (B-rep topology only). Null when the file
 * yields no measurable topology (e.g. pure AP242 tessellated geometry) or does not parse. */
export function estimateStepSize(src: string): SizeEstimate | null {
  try {
    const brep = buildBrep(src);
    const table = brep.table;
    const world = new Box();
    for (const solid of brep.solids) {
      const local = new Box();
      const seen = new Set<number>();
      for (const face of solid.faces) for (const lp of face.loops) for (const oe of lp.edges) {
        if (seen.has(oe.edgeId)) continue;
        seen.add(oe.edgeId);
        const e = brep.edges.get(oe.edgeId);
        if (!e) continue;
        local.add(e.v0[0], e.v0[1], e.v0[2]);
        local.add(e.v1[0], e.v1[1], e.v1[2]);
        // A closed rim (circle/ellipse whose two vertices coincide) contributes only a single
        // point above — include the whole circle's box so a cylinder's diameter registers.
        // Partial arcs keep endpoints only: their two vertices already span the arc, and the
        // full-circle box would double a quarter-round's extent. Wrapped kinds unwrap first.
        if (e.v0[0] !== e.v1[0] || e.v0[1] !== e.v1[1] || e.v0[2] !== e.v1[2]) continue;
        try {
          let cid = e.curveId, kind = table.typeOf(cid);
          while (kind === "SURFACE_CURVE" || kind === "SEAM_CURVE" || kind === "INTERSECTION_CURVE") {
            cid = ref(table.record(cid).params[1]!);
            kind = table.typeOf(cid);
          }
          if (kind === "CIRCLE" || kind === "ELLIPSE") {
            const sc = e.scale ?? brep.scale;
            const rec = table.record(cid);
            const f = readPlacement(table, ref(rec.params[1]!), sc);
            const a = num(rec.params[2]!) * sc;
            const r = kind === "ELLIPSE" ? Math.max(a, num(rec.params[3]!) * sc) : a;
            local.add(f.o[0] - r, f.o[1] - r, f.o[2] - r);
            local.add(f.o[0] + r, f.o[1] + r, f.o[2] + r);
          }
        } catch { /* malformed curve — endpoints already counted */ }
      }
      if (local.empty) continue;
      const frames = solid.instances ?? (solid.transform ? [solid.transform] : null);
      if (!frames) {
        world.add(local.min[0], local.min[1], local.min[2]);
        world.add(local.max[0], local.max[1], local.max[2]);
        continue;
      }
      // Transform the 8 local bbox corners per instance — bounds the transformed geometry.
      for (const f of frames) for (let c = 0; c < 8; c++) {
        const p = applyFrame(f,
          (c & 1 ? local.max : local.min)[0],
          (c & 2 ? local.max : local.min)[1],
          (c & 4 ? local.max : local.min)[2]);
        world.add(p[0], p[1], p[2]);
      }
    }
    if (world.empty) return null;
    const diag = Math.hypot(world.max[0] - world.min[0], world.max[1] - world.min[1], world.max[2] - world.min[2]);
    return {
      bbox: [world.min[0], world.min[1], world.min[2], world.max[0], world.max[1], world.max[2]],
      diag,
      units: brep.units.label,
    };
  } catch {
    return null;
  }
}

/** Round to the nearest "nice" 1/2/5 × 10^k value, so auto defaults read like hand-picked ones. */
const nice = (v: number): number => {
  const e = Math.floor(Math.log10(v));
  const m = v / 10 ** e;
  const n = m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10;
  return n * 10 ** e;
};
const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v));

/**
 * Size-adaptive tessellation defaults, anchored so a ~100 mm part gets the library defaults
 * (0.01 mm surface deviation, 1 mm max edge). Max edge scales linearly with model size, but
 * surface deviation scales linearly below that anchor but only doubles per decade above it
 * (100 mm → 0.01, 1000 mm → 0.02): chord error is an absolute surface-quality budget, so
 * letting it grow 10× on metre-scale parts visibly facets them. Clamps keep tiny models from
 * demanding sub-micron chords.
 */
export function autoTessellation(diagMm: number): { surfaceDeviation: number; maxEdge: number } {
  const dev = diagMm <= 100 ? diagMm * 1e-4 : 0.01 * 2 ** Math.log10(diagMm / 100);
  return {
    surfaceDeviation: nice(clamp(dev, 0.001, 0.1)),
    maxEdge: nice(clamp(diagMm * 1e-2, 0.1, 100)),
  };
}
