// SPDX-License-Identifier: AGPL-3.0-only
// Exploded-view offsets. Two ideas make this smarter than "push everything away from the COG":
//
//  1. HIERARCHICAL grouping — the assembly tree (PartNode) drives the motion: at each branching
//     level, sibling subtrees separate as rigid groups away from their parent's centroid, and a
//     group's internals get their own (later-easing) separation inside it. At a low slider value
//     the top-level sub-assemblies drift apart while their internals barely loosen; at 1.0
//     everything is apart but each part still sits NEAR its group, so ownership stays readable.
//     Flat trees (STL / 3MF / single-part STEP) degrade to a plain radial explosion naturally.
//
//  2. MATE-AXIS directions — a part whose cylindrical faces are coaxial with another part's
//     (pin-in-hole: screw/boss, bearing/shaft, rod/bushing) explodes ALONG that axis instead of
//     radially, so fasteners visibly back out of their holes. Inferred from the analytic face
//     data the measure payload already carries; parts without an unambiguous mate fall back to
//     the radial rule, so the inference can never do worse than the default.
//
// All geometry statistics come from the placed mesh (per-instance, via instanceOfTri), so a part
// used ×N explodes as N independent occurrences.
import type { MeasureGeometry, PartNode, SolidInstance } from "../../src/index.ts";
import type { RawMesh } from "./mesh-utils.ts";

/** How the parts move apart:
 *  - hierarchical — assembly-tree grouped, mate-axis-aware (the smart default).
 *  - radial — classic scale-about-COG, ignores hierarchy and mates.
 *  - axis — stack-up along one direction (auto = dominant mate axis), like a layered drawing.
 *  - peel — outermost parts fly away first; the slider is "how deep have I disassembled".
 *  - layout — parts travel to a flat grid in front of the assembly (workbench inventory);
 *    the axis picks the plane's normal ("up"), z = lay flat on the ground plane. */
export type ExplodeStyle = "hierarchical" | "radial" | "axis" | "peel" | "layout";
export type ExplodeAxis = "auto" | "x" | "y" | "z";

export interface ExplodeInfo {
  /** Per-triangle instance index (synthesized from solidOfTri for STL/3MF). */
  instanceOfTri: Uint32Array;
  /** Placed occurrences that actually carry geometry — below 2 there is nothing to explode. */
  leafCount: number;
  /** World offset per instance at explode factor f in [0,1], 3 floats per instance. */
  offsetsAt(f: number, style?: ExplodeStyle, axis?: ExplodeAxis): Float64Array;
}

/** Overall explosion strength: at factor 1 a group's distance from its parent centroid grows
 * by this multiple (classic scale-about-centroid explode, so concentric nesting stays nested). */
const K = 1.75;

interface XNode {
  /** Instance index for a leaf (one placed body occurrence); -1 for a group. */
  inst: number;
  children: XNode[];
  /** Area-weighted surface centroid (x, y, z) and total area weight. */
  c: [number, number, number];
  w: number;
  /** Bounding box (union of children) and its half diagonal. */
  bb: [number, number, number, number, number, number];
  r: number;
}

interface Cyl {
  inst: number;
  /** Canonical unit axis (sign-flipped so the largest component is positive). */
  a: [number, number, number];
  o: [number, number, number];
  r: number;
  /** Extent of the face's triangles along `a`, relative to `o`. */
  tmin: number;
  tmax: number;
}

const EMPTY_BB: XNode["bb"] = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];

function groupOf(children: XNode[]): XNode {
  const bb: XNode["bb"] = [...EMPTY_BB];
  let w = 0, cx = 0, cy = 0, cz = 0;
  for (const k of children) {
    w += k.w;
    cx += k.c[0] * k.w; cy += k.c[1] * k.w; cz += k.c[2] * k.w;
    for (let i = 0; i < 3; i++) {
      if (k.bb[i]! < bb[i]!) bb[i] = k.bb[i]!;
      if (k.bb[i + 3]! > bb[i + 3]!) bb[i + 3] = k.bb[i + 3]!;
    }
  }
  const c: XNode["c"] = w > 0 ? [cx / w, cy / w, cz / w]
    : [(bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2];
  const r = Math.hypot(bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]) / 2 || 0;
  return { inst: -1, children, c, w, bb, r };
}

/** Deterministic well-spread unit direction for the k-th concentric sibling (golden spiral). */
function spreadDir(k: number): [number, number, number] {
  const z = 1 - (2 * ((k % 16) + 0.5)) / 16;
  const s = Math.sqrt(Math.max(0, 1 - z * z));
  const phi = k * 2.399963229728653; // golden angle
  return [s * Math.cos(phi), s * Math.sin(phi), z];
}

export function buildExplode(args: {
  structure: PartNode;
  mesh: RawMesh;
  solidOfTri: Uint32Array;
  faceOfTri: Uint32Array;
  instances: SolidInstance[] | null;
  instanceOfTri: Uint32Array | null;
  measure: MeasureGeometry | null;
}): ExplodeInfo {
  const { mesh, solidOfTri, faceOfTri, measure, structure } = args;
  // STL/3MF ship no instance table: every body is one occurrence and the ids are dense
  // (0..bodies-1), so solidOfTri doubles as the instance mapping unchanged.
  let instances = args.instances;
  let instanceOfTri = args.instanceOfTri;
  if (!instances || !instanceOfTri) {
    let maxS = -1;
    for (let t = 0; t < solidOfTri.length; t++) if (solidOfTri[t]! > maxS) maxS = solidOfTri[t]!;
    instances = Array.from({ length: maxS + 1 }, (_, i) => ({ solidId: i, instance: 0, frame: null }));
    instanceOfTri = solidOfTri;
  }
  const nI = instances.length;

  // ---- per-instance statistics: area-weighted centroid + bbox (one pass over the mesh) ----
  const accW = new Float64Array(nI);
  const accC = new Float64Array(nI * 3);
  const bb = new Float64Array(nI * 6);
  for (let i = 0; i < nI; i++) {
    bb[i * 6] = bb[i * 6 + 1] = bb[i * 6 + 2] = Infinity;
    bb[i * 6 + 3] = bb[i * 6 + 4] = bb[i * 6 + 5] = -Infinity;
  }
  const pos = mesh.positions, idx = mesh.indices;
  const nT = instanceOfTri.length;
  for (let t = 0; t < nT; t++) {
    const i = instanceOfTri[t]!;
    const a = idx[t * 3]! * 3, b = idx[t * 3 + 1]! * 3, c = idx[t * 3 + 2]! * 3;
    const ax = pos[a]!, ay = pos[a + 1]!, az = pos[a + 2]!;
    const bx = pos[b]!, by = pos[b + 1]!, bz = pos[b + 2]!;
    const cx = pos[c]!, cy = pos[c + 1]!, cz = pos[c + 2]!;
    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const vx = cx - ax, vy = cy - ay, vz = cz - az;
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const w = Math.sqrt(nx * nx + ny * ny + nz * nz); // 2 × area — only relative weight matters
    accW[i]! += w;
    accC[i * 3] += w * (ax + bx + cx) / 3;
    accC[i * 3 + 1] += w * (ay + by + cy) / 3;
    accC[i * 3 + 2] += w * (az + bz + cz) / 3;
    const o = i * 6;
    if (ax < bb[o]!) bb[o] = ax; if (ax > bb[o + 3]!) bb[o + 3] = ax;
    if (ay < bb[o + 1]!) bb[o + 1] = ay; if (ay > bb[o + 4]!) bb[o + 4] = ay;
    if (az < bb[o + 2]!) bb[o + 2] = az; if (az > bb[o + 5]!) bb[o + 5] = az;
    if (bx < bb[o]!) bb[o] = bx; if (bx > bb[o + 3]!) bb[o + 3] = bx;
    if (by < bb[o + 1]!) bb[o + 1] = by; if (by > bb[o + 4]!) bb[o + 4] = by;
    if (bz < bb[o + 2]!) bb[o + 2] = bz; if (bz > bb[o + 5]!) bb[o + 5] = bz;
    if (cx < bb[o]!) bb[o] = cx; if (cx > bb[o + 3]!) bb[o + 3] = cx;
    if (cy < bb[o + 1]!) bb[o + 1] = cy; if (cy > bb[o + 4]!) bb[o + 4] = cy;
    if (cz < bb[o + 2]!) bb[o + 2] = cz; if (cz > bb[o + 5]!) bb[o + 5] = cz;
  }

  const leafOf = (i: number): XNode | null => {
    if (accW[i]! <= 0 && bb[i * 6]! === Infinity) return null; // no triangles (skipped solid)
    const o = i * 6;
    const box: XNode["bb"] = [bb[o]!, bb[o + 1]!, bb[o + 2]!, bb[o + 3]!, bb[o + 4]!, bb[o + 5]!];
    const w = accW[i]!;
    const c: XNode["c"] = w > 0
      ? [accC[i * 3]! / w, accC[i * 3 + 1]! / w, accC[i * 3 + 2]! / w]
      : [(box[0] + box[3]) / 2, (box[1] + box[4]) / 2, (box[2] + box[5]) / 2];
    const r = Math.hypot(box[3] - box[0], box[4] - box[1], box[5] - box[2]) / 2 || 0;
    return { inst: i, children: [], c, w, bb: box, r };
  };

  // ---- explosion tree mirroring the part tree; bodies expand to their placed instances ----
  const instIdxOfSolid = new Map<number, number[]>();
  instances.forEach((si, i) => {
    const list = instIdxOfSolid.get(si.solidId);
    if (list) list.push(i); else instIdxOfSolid.set(si.solidId, [i]);
  });
  const covered = new Set<number>();
  const bodyNode = (solidId: number): XNode | null => {
    const leaves: XNode[] = [];
    for (const i of instIdxOfSolid.get(solidId) ?? []) {
      const leaf = leafOf(i);
      if (leaf) { leaves.push(leaf); covered.add(i); }
    }
    if (leaves.length === 0) return null;
    return leaves.length === 1 ? leaves[0]! : groupOf(leaves);
  };
  // Single-child chains collapse (wrapper products carry no geometry of their own), so tree
  // depth below counts only BRANCHING levels — the level-gain staggering isn't wasted on them.
  const partToX = (n: PartNode): XNode | null => {
    const kids: XNode[] = [];
    for (const child of n.children) { const x = partToX(child); if (x) kids.push(x); }
    for (const b of n.bodies) { const x = bodyNode(b.id); if (x) kids.push(x); }
    if (kids.length === 0) return null;
    return kids.length === 1 ? kids[0]! : groupOf(kids);
  };
  let root = partToX(structure);
  // Bodies present in the mesh but absent from the part tree still deserve to move.
  const strays: XNode[] = [];
  for (let i = 0; i < nI; i++) {
    if (covered.has(i)) continue;
    const leaf = leafOf(i);
    if (leaf) strays.push(leaf);
  }
  if (strays.length > 0) root = groupOf(root ? [root, ...strays] : strays);
  const leaves: XNode[] = [];
  const gather = (n: XNode): void => {
    if (n.inst >= 0) { leaves.push(n); return; }
    for (const k of n.children) gather(k);
  };
  if (root) gather(root);
  const leafCount = leaves.length;

  // Global frame shared by the flat styles: area-weighted centroid, bbox, half diagonal.
  const C: [number, number, number] = root ? root.c : [0, 0, 0];
  const GB = root ? root.bb : EMPTY_BB;
  const R = root ? Math.max(root.r, 1e-9) : 1;

  // ---- mate-axis inference: coaxial cylinder pairs across different instances ----
  const mateAxis = new Float64Array(nI * 3); // zero vector = no unambiguous mate
  if (measure && leafCount > 1) inferMateAxes(measure, instances, instanceOfTri, faceOfTri, mesh, mateAxis);

  const hierarchicalAt = (f: number): Float64Array => {
    const out = new Float64Array(nI * 3);
    if (!root || f <= 0) return out;
    const fac = Math.min(1, f);
    const walk = (node: XNode, parent: XNode | null, siblingIdx: number, depth: number, ix: number, iy: number, iz: number): void => {
      let ox = ix, oy = iy, oz = iz;
      if (parent) {
        const gain = K * Math.pow(fac, depth);
        let dx = node.c[0] - parent.c[0], dy = node.c[1] - parent.c[1], dz = node.c[2] - parent.c[2];
        const ax3 = node.inst * 3;
        if (node.inst >= 0 && (mateAxis[ax3] !== 0 || mateAxis[ax3 + 1] !== 0 || mateAxis[ax3 + 2] !== 0)) {
          // Pin-in-hole part: slide along its mate axis, signed away from the group centroid.
          const ax = mateAxis[ax3]!, ay = mateAxis[ax3 + 1]!, az = mateAxis[ax3 + 2]!;
          const along = dx * ax + dy * ay + dz * az;
          const sign = along >= 0 ? 1 : -1;
          const mag = gain * Math.max(Math.abs(along), node.r);
          ox += ax * sign * mag; oy += ay * sign * mag; oz += az * sign * mag;
        } else {
          const len = Math.hypot(dx, dy, dz);
          if (len < 1e-6 * Math.max(parent.r, 1e-9)) {
            // Concentric sibling — the centroid delta carries no direction; spread deterministically.
            const [sx, sy, sz] = spreadDir(siblingIdx);
            const mag = gain * Math.max(node.r, parent.r * 0.4);
            ox += sx * mag; oy += sy * mag; oz += sz * mag;
          } else {
            ox += dx * gain; oy += dy * gain; oz += dz * gain;
          }
        }
      }
      if (node.inst >= 0) {
        out[node.inst * 3] = ox; out[node.inst * 3 + 1] = oy; out[node.inst * 3 + 2] = oz;
        return;
      }
      const childDepth = node.children.length > 1 ? depth + 1 : depth;
      node.children.forEach((k, j) => walk(k, node, j, parent ? childDepth : depth, ox, oy, oz));
    };
    walk(root, null, 0, 1, 0, 0, 0);
    return out;
  };

  // ---- radial: classic scale-about-COG (every part's distance to the centroid grows) ----
  const radialAt = (f: number): Float64Array => {
    const out = new Float64Array(nI * 3);
    if (f <= 0) return out;
    leaves.forEach((n, k) => {
      const dx = n.c[0] - C[0], dy = n.c[1] - C[1], dz = n.c[2] - C[2];
      const g = K * Math.min(1, f);
      const o = n.inst * 3;
      if (Math.hypot(dx, dy, dz) < 1e-6 * R) {
        const [sx, sy, sz] = spreadDir(k);
        const mag = g * 0.3 * R;
        out[o] = sx * mag; out[o + 1] = sy * mag; out[o + 2] = sz * mag;
      } else {
        out[o] = dx * g; out[o + 1] = dy * g; out[o + 2] = dz * g;
      }
    });
    return out;
  };

  // ---- axis: stack-up along one direction, proportional to each part's station along it ----
  const resolveAxis = (axis: ExplodeAxis): [number, number, number] => {
    if (axis === "x") return [1, 0, 0];
    if (axis === "y") return [0, 1, 0];
    if (axis === "z") return [0, 0, 1];
    // auto: the coordinate axis the mate axes agree on most (screws point along the assembly
    // direction); with no mates, the axis the part centroids spread along most.
    const score = [0, 0, 0];
    let anyMate = false;
    for (let i = 0; i < nI; i++) {
      const ax = mateAxis[i * 3]!, ay = mateAxis[i * 3 + 1]!, az = mateAxis[i * 3 + 2]!;
      if (ax === 0 && ay === 0 && az === 0) continue;
      anyMate = true;
      score[0] += Math.abs(ax); score[1] += Math.abs(ay); score[2] += Math.abs(az);
    }
    if (!anyMate) {
      const mean = [0, 0, 0];
      for (const n of leaves) { mean[0] += n.c[0]; mean[1] += n.c[1]; mean[2] += n.c[2]; }
      for (let d = 0; d < 3; d++) mean[d] = mean[d]! / Math.max(1, leaves.length);
      for (const n of leaves) for (let d = 0; d < 3; d++) score[d] += (n.c[d]! - mean[d]!) ** 2;
    }
    const best = score[0] >= score[1] && score[0] >= score[2] ? 0 : score[1] >= score[2] ? 1 : 2;
    return best === 0 ? [1, 0, 0] : best === 1 ? [0, 1, 0] : [0, 0, 1];
  };
  const axisAt = (f: number, axis: ExplodeAxis): Float64Array => {
    if (f <= 0) return new Float64Array(nI * 3);
    const a = resolveAxis(axis);
    let pMin = Infinity, pMax = -Infinity;
    for (const n of leaves) {
      const p = n.c[0] * a[0] + n.c[1] * a[1] + n.c[2] * a[2];
      if (p < pMin) pMin = p;
      if (p > pMax) pMax = p;
    }
    const span = pMax - pMin;
    if (span < 1e-6) return radialAt(f); // everything at one station — nothing to stack
    const mid = (pMin + pMax) / 2;
    const out = new Float64Array(nI * 3);
    // Endpoints travel ~2.2 half-diagonals at f=1 — clears even a flat pancake assembly whose
    // stacking span (a few mm) is tiny next to its width.
    const g = Math.min(1, f) * 2.2 * R;
    for (const n of leaves) {
      const p = n.c[0] * a[0] + n.c[1] * a[1] + n.c[2] * a[2];
      const mag = ((p - mid) / (span / 2)) * g;
      const o = n.inst * 3;
      out[o] = a[0] * mag; out[o + 1] = a[1] * mag; out[o + 2] = a[2] * mag;
    }
    return out;
  };

  // ---- peel: outer parts leave first, the slider walks inward layer by layer ----
  let peelRank: Float64Array | null = null; // per-leaf rank in [0,1]: 0 = outermost
  const peelAt = (f: number): Float64Array => {
    const out = new Float64Array(nI * 3);
    if (f <= 0 || leaves.length < 2) return out;
    if (!peelRank) {
      // Enclosure-depth proxy: distance of the centroid to the nearest global bbox wall —
      // shallow = outer shell, deep = core. Rank-based so odd shapes can't skew the pacing.
      const depth = leaves.map((n, k) => ({
        k,
        d: Math.min(
          n.c[0] - GB[0], GB[3] - n.c[0],
          n.c[1] - GB[1], GB[4] - n.c[1],
          n.c[2] - GB[2], GB[5] - n.c[2],
        ),
      }));
      depth.sort((a, b) => a.d - b.d);
      peelRank = new Float64Array(leaves.length);
      depth.forEach((e, order) => { peelRank![e.k] = order / (depth.length - 1); });
    }
    const fac = Math.min(1, f);
    leaves.forEach((n, k) => {
      const rank = peelRank![k]!;
      const start = 0.7 * rank; // outermost moves immediately; the core waits until f ~ 0.7
      const local = Math.min(1, Math.max(0, (fac - start) / (1 - start)));
      if (local <= 0) return;
      const ease = local * local * (3 - 2 * local);
      const mag = (1.2 + 1.6 * (1 - rank)) * R * ease; // outer parts also end up farther out
      let dx = n.c[0] - C[0], dy = n.c[1] - C[1], dz = n.c[2] - C[2];
      const len = Math.hypot(dx, dy, dz);
      if (len < 1e-6 * R) [dx, dy, dz] = spreadDir(k);
      else { dx /= len; dy /= len; dz /= len; }
      const o = n.inst * 3;
      out[o] = dx * mag; out[o + 1] = dy * mag; out[o + 2] = dz * mag;
    });
    return out;
  };

  // ---- layout: shelf-pack every part onto a flat grid in front of the assembly ----
  // The axis is the plane's NORMAL ("up"): z lays parts on the ground (XY) plane in front of
  // the assembly (-Y side), x/y pick the other two coordinate planes with the same convention
  // (grid along the first remaining axis, rows advancing toward negative on the second).
  let layoutTarget: Float64Array | null = null; // per-leaf target centroid (x, y, z)
  let layoutNormal = -1; // axis the cached targets were built for
  const layoutAt = (f: number, axis: ExplodeAxis): Float64Array => {
    const out = new Float64Array(nI * 3);
    if (f <= 0 || leaves.length === 0) return out;
    const nAx = axis === "x" ? 0 : axis === "y" ? 1 : 2; // auto behaves as z (ground plane)
    const uAx = nAx === 0 ? 1 : 0;
    const vAx = nAx === 2 ? 1 : 2;
    if (!layoutTarget || layoutNormal !== nAx) {
      layoutNormal = nAx;
      layoutTarget = new Float64Array(leaves.length * 3);
      const pad = 0.06 * R;
      // Big parts first, rows capped near the assembly's width (or the grid's own square).
      const order = leaves.map((n, k) => ({
        k,
        w: n.bb[uAx + 3]! - n.bb[uAx]! + pad,
        d: n.bb[vAx + 3]! - n.bb[vAx]! + pad,
      }));
      let cellArea = 0;
      for (const it of order) cellArea += it.w * it.d;
      const W = Math.max(GB[uAx + 3]! - GB[uAx]!, Math.sqrt(cellArea) * 1.25);
      order.sort((a, b) => Math.max(b.w, b.d) - Math.max(a.w, a.d));
      const x0 = GB[uAx]!;
      let cx = x0, rowY = GB[vAx]! - pad * 3, rowDepth = 0;
      for (const it of order) {
        if (cx > x0 && cx + it.w > x0 + W) { cx = x0; rowY -= rowDepth; rowDepth = 0; }
        const n = leaves[it.k]!;
        // Cell places the part's bbox; the centroid target keeps its offset inside that bbox,
        // and the part drops onto the plane (normal coordinate = global min).
        layoutTarget[it.k * 3 + uAx] = cx + (n.c[uAx]! - n.bb[uAx]!);
        layoutTarget[it.k * 3 + vAx] = rowY - it.d + (n.c[vAx]! - n.bb[vAx]!);
        layoutTarget[it.k * 3 + nAx] = GB[nAx]! + (n.c[nAx]! - n.bb[nAx]!);
        cx += it.w;
        if (it.d > rowDepth) rowDepth = it.d;
      }
    }
    const fac = Math.min(1, f);
    leaves.forEach((n, k) => {
      const o = n.inst * 3;
      out[o] = (layoutTarget![k * 3]! - n.c[0]) * fac;
      out[o + 1] = (layoutTarget![k * 3 + 1]! - n.c[1]) * fac;
      out[o + 2] = (layoutTarget![k * 3 + 2]! - n.c[2]) * fac;
    });
    return out;
  };

  const offsetsAt = (f: number, style: ExplodeStyle = "hierarchical", axis: ExplodeAxis = "auto"): Float64Array => {
    switch (style) {
      case "radial": return radialAt(f);
      case "axis": return axisAt(f, axis);
      case "peel": return peelAt(f);
      case "layout": return layoutAt(f, axis);
      default: return hierarchicalAt(f);
    }
  };

  return { instanceOfTri, leafCount, offsetsAt };
}

/** Fill `mateAxis` (3 floats per instance, zero = none) with each instance's dominant mating
 * direction: cylindrical faces of DIFFERENT instances that are coaxial (parallel axes, near-zero
 * axis offset, similar radii, overlapping axial extent) mark a pin-in-hole mate; an instance
 * whose mates cluster around one direction gets that unit axis. */
function inferMateAxes(
  measure: MeasureGeometry,
  instances: SolidInstance[],
  instanceOfTri: Uint32Array,
  faceOfTri: Uint32Array,
  mesh: RawMesh,
  mateAxis: Float64Array,
): void {
  // Cylinder records per faceId, indexed by occurrence (measure replicates faces per instance).
  const cylByFace = new Map<number, (Cyl | undefined)[]>();
  const instIdx = new Map<number, number>(); // solidId * 2^20 + occurrence -> instance index
  instances.forEach((si, i) => instIdx.set(si.solidId * 1048576 + si.instance, i));
  let count = 0;
  for (const f of measure.faces) {
    if (f.kind !== "CYLINDRICAL_SURFACE" || !f.axis || !f.origin || !(f.radius! > 0)) continue;
    const inst = instIdx.get(f.solidId * 1048576 + f.instance);
    if (inst === undefined) continue;
    // Canonical sign so opposite-sense cylinders still land in the same direction bucket.
    let [ax, ay, az] = f.axis;
    const m = Math.abs(ax) >= Math.abs(ay) && Math.abs(ax) >= Math.abs(az) ? ax : Math.abs(ay) >= Math.abs(az) ? ay : az;
    if (m < 0) { ax = -ax; ay = -ay; az = -az; }
    const l = Math.hypot(ax, ay, az);
    if (l < 1e-9) continue;
    const rec: Cyl = {
      inst, a: [ax / l, ay / l, az / l], o: [...f.origin], r: f.radius!,
      tmin: Infinity, tmax: -Infinity,
    };
    let list = cylByFace.get(f.faceId);
    if (!list) { list = []; cylByFace.set(f.faceId, list); }
    list[f.instance] = rec;
    if (++count > 60_000) return; // pathological model — skip inference, radial fallback everywhere
  }
  if (count === 0) return;

  // Axial extent of each cylinder face from its own (instance-placed) triangles.
  const pos = mesh.positions, idx = mesh.indices;
  for (let t = 0; t < instanceOfTri.length; t++) {
    const list = cylByFace.get(faceOfTri[t]!);
    if (!list) continue;
    const rec = list[instances[instanceOfTri[t]!]!.instance];
    if (!rec || rec.inst !== instanceOfTri[t]!) continue;
    for (let e = 0; e < 3; e++) {
      const v = idx[t * 3 + e]! * 3;
      const s = (pos[v]! - rec.o[0]) * rec.a[0] + (pos[v + 1]! - rec.o[1]) * rec.a[1] + (pos[v + 2]! - rec.o[2]) * rec.a[2];
      if (s < rec.tmin) rec.tmin = s;
      if (s > rec.tmax) rec.tmax = s;
    }
  }

  // Bucket by quantized canonical direction; pair-match within a bucket.
  const buckets = new Map<string, Cyl[]>();
  for (const list of cylByFace.values()) {
    for (const rec of list) {
      if (!rec || rec.tmin === Infinity) continue; // face never tessellated
      const key = `${Math.round(rec.a[0] * 50)},${Math.round(rec.a[1] * 50)},${Math.round(rec.a[2] * 50)}`;
      const arr = buckets.get(key);
      if (arr) arr.push(rec); else buckets.set(key, [rec]);
    }
  }
  const votes = new Map<number, { a: [number, number, number]; w: number }[]>();
  for (const arr of buckets.values()) {
    if (arr.length < 2 || arr.length > 512) continue; // 512+ coaxial candidates: degenerate, skip
    for (let i = 0; i < arr.length; i++) {
      for (let j = i + 1; j < arr.length; j++) {
        const A = arr[i]!, B = arr[j]!;
        if (A.inst === B.inst) continue;
        if (A.a[0] * B.a[0] + A.a[1] * B.a[1] + A.a[2] * B.a[2] < 0.9995) continue;
        const rMax = Math.max(A.r, B.r);
        if (Math.abs(A.r - B.r) > Math.max(1.0, 0.35 * rMax)) continue;
        // Distance between the two axis lines (they're parallel).
        const dx = B.o[0] - A.o[0], dy = B.o[1] - A.o[1], dz = B.o[2] - A.o[2];
        const along = dx * A.a[0] + dy * A.a[1] + dz * A.a[2];
        const px = dx - along * A.a[0], py = dy - along * A.a[1], pz = dz - along * A.a[2];
        if (Math.hypot(px, py, pz) > Math.max(0.3, 0.1 * rMax)) continue;
        // Axial overlap in A's parameterization.
        const bMin = B.tmin + along, bMax = B.tmax + along;
        const overlap = Math.min(A.tmax, bMax) - Math.max(A.tmin, bMin);
        const minLen = Math.min(A.tmax - A.tmin, B.tmax - B.tmin);
        if (overlap < Math.max(0.05, 0.1 * minLen)) continue;
        const w = overlap * Math.min(A.r, B.r); // contact-area proxy
        for (const inst of [A.inst, B.inst]) {
          const list = votes.get(inst);
          const vote = { a: A.a, w };
          if (list) list.push(vote); else votes.set(inst, [vote]);
        }
      }
    }
  }

  // Dominant direction per instance: the heaviest near-parallel cluster must carry >= 70% of
  // all mate weight, else the part mates on divergent axes and radial is the honest answer.
  for (const [inst, list] of votes) {
    let total = 0;
    for (const v of list) total += v.w;
    let bestW = 0;
    let best: [number, number, number] | null = null;
    for (const seed of list) {
      let w = 0, sx = 0, sy = 0, sz = 0;
      for (const v of list) {
        const d = seed.a[0] * v.a[0] + seed.a[1] * v.a[1] + seed.a[2] * v.a[2];
        if (Math.abs(d) < 0.98) continue;
        const s = d >= 0 ? 1 : -1;
        w += v.w;
        sx += s * v.a[0] * v.w; sy += s * v.a[1] * v.w; sz += s * v.a[2] * v.w;
      }
      if (w > bestW) {
        const l = Math.hypot(sx, sy, sz);
        if (l > 1e-9) { bestW = w; best = [sx / l, sy / l, sz / l]; }
      }
    }
    if (best && bestW >= 0.7 * total) {
      mateAxis[inst * 3] = best[0]; mateAxis[inst * 3 + 1] = best[1]; mateAxis[inst * 3 + 2] = best[2];
    }
  }
}
