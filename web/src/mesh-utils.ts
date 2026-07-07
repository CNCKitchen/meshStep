// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";

export interface RawMesh {
  positions: Float64Array; // 3 per vertex
  indices: Uint32Array; // 3 per triangle
}

/** Build an indexed BufferGeometry (Float32 positions) from a meshStep IndexedMesh. */
export function buildIndexedGeometry(mesh: RawMesh): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(mesh.positions), 3));
  g.setIndex(new THREE.Uint32BufferAttribute(mesh.indices.slice(), 1));
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

/** Build a non-indexed geometry from a flat triangle-soup (9 floats per tri), e.g. a reference STL. */
export function buildSoupGeometry(positions: Float64Array | Float32Array): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(positions), 3));
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

/**
 * Extract boundary ("open") edges of an indexed mesh: edges used by exactly one
 * triangle. A watertight mesh has none. Returns a LineSegments-ready position array.
 */
export function boundaryEdges(mesh: RawMesh): { positions: Float32Array; count: number } {
  const idx = mesh.indices;
  const pos = mesh.positions;
  const useCount = new Map<number, number>();
  // Same packing as src/mesh/orient.ts: exact for vertex ids below 2^26 (~67M). The old
  // a*2^32+b key exceeded 2^53 from ~2M vertices up and silently collided.
  const KEY = 0x4000000;
  const key = (a: number, b: number) => (a < b ? a * KEY + b : b * KEY + a);

  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t]!, b = idx[t + 1]!, c = idx[t + 2]!;
    const k0 = key(a, b), k1 = key(b, c), k2 = key(c, a);
    useCount.set(k0, (useCount.get(k0) ?? 0) + 1);
    useCount.set(k1, (useCount.get(k1) ?? 0) + 1);
    useCount.set(k2, (useCount.get(k2) ?? 0) + 1);
  }

  // Second pass: collect the actual vertex pairs for edges used once.
  const out: number[] = [];
  const emit = (u: number, v: number): void => {
    if (useCount.get(key(u, v)) !== 1) return;
    out.push(pos[u * 3]!, pos[u * 3 + 1]!, pos[u * 3 + 2]!);
    out.push(pos[v * 3]!, pos[v * 3 + 1]!, pos[v * 3 + 2]!);
  };
  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t]!, b = idx[t + 1]!, c = idx[t + 2]!;
    emit(a, b); emit(b, c); emit(c, a);
  }
  return { positions: new Float32Array(out), count: out.length / 6 };
}

/** Triangle count of an indexed mesh. */
export function triCount(mesh: RawMesh): number {
  return mesh.indices.length / 3;
}

/**
 * "Turbo"-style colormap (Google), good for deviation maps. t in [0,1] -> [r,g,b] in [0,1].
 * Polynomial approximation (Anton Mikhailov).
 */
export function turbo(t: number): [number, number, number] {
  const x = Math.min(1, Math.max(0, t));
  const r =
    0.13572138 + x * (4.6153926 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
  const g =
    0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
  const b =
    0.1066733 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
  return [Math.min(1, Math.max(0, r)), Math.min(1, Math.max(0, g)), Math.min(1, Math.max(0, b))];
}

/** Diverging blue-white-red map for SIGNED deviation, d in [-range, +range]. */
export function diverging(d: number, range: number): [number, number, number] {
  if (range <= 0) return [1, 1, 1];
  const t = Math.min(1, Math.max(-1, d / range)); // -1..1
  if (t < 0) {
    const a = 1 + t; // 0 at -range, 1 at 0
    return [a, a, 1]; // blue -> white
  }
  const a = 1 - t; // 1 at 0, 0 at +range
  return [1, a, a]; // white -> red
}
