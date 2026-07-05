// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";
import { MeshBVH } from "three-mesh-bvh";
import { diverging } from "./mesh-utils.ts";

export interface DeviationResult {
  /** Signed distance per vertex (mm). + = generated surface lies outside the reference. */
  signed: Float32Array;
  /** Per-vertex RGB colors packed for a BufferGeometry "color" attribute. */
  colors: Float32Array;
  maxAbs: number;
  rms: number;
  mean: number;
}

/** A reference mesh wrapped with a BVH for fast closest-point queries. */
export class ReferenceSurface {
  readonly geometry: THREE.BufferGeometry;
  private bvh: MeshBVH;

  constructor(geometry: THREE.BufferGeometry) {
    this.geometry = geometry;
    this.bvh = new MeshBVH(geometry);
  }

  dispose(): void {
    this.geometry.dispose();
  }

  /**
   * For every vertex of `target`, compute the signed distance to this reference
   * surface and a diverging color. `range` is the color clamp (mm); pass null to
   * auto-scale to the max abs deviation found.
   */
  deviationFor(target: THREE.BufferGeometry, range: number | null): DeviationResult {
    const posAttr = target.getAttribute("position");
    const n = posAttr.count;
    const signed = new Float32Array(n);

    const p = new THREE.Vector3();
    const hit = { point: new THREE.Vector3(), distance: 0, faceIndex: -1 };

    const refPos = this.geometry.getAttribute("position");
    const refIndex = this.geometry.getIndex();
    const va = new THREE.Vector3(), vb = new THREE.Vector3(), vc = new THREE.Vector3();
    const e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), faceN = new THREE.Vector3(), dir = new THREE.Vector3();

    let maxAbs = 0;
    let sumSq = 0;
    let sum = 0;

    for (let i = 0; i < n; i++) {
      p.set(posAttr.getX(i), posAttr.getY(i), posAttr.getZ(i));
      this.bvh.closestPointToPoint(p, hit);
      const dist = hit.distance;

      // Sign via the reference face normal at the closest triangle.
      const f = hit.faceIndex;
      let i0: number, i1: number, i2: number;
      if (refIndex) {
        i0 = refIndex.getX(f * 3);
        i1 = refIndex.getX(f * 3 + 1);
        i2 = refIndex.getX(f * 3 + 2);
      } else {
        i0 = f * 3; i1 = f * 3 + 1; i2 = f * 3 + 2;
      }
      va.set(refPos.getX(i0), refPos.getY(i0), refPos.getZ(i0));
      vb.set(refPos.getX(i1), refPos.getY(i1), refPos.getZ(i1));
      vc.set(refPos.getX(i2), refPos.getY(i2), refPos.getZ(i2));
      e1.subVectors(vb, va);
      e2.subVectors(vc, va);
      faceN.crossVectors(e1, e2);
      dir.subVectors(p, hit.point);
      const sign = faceN.dot(dir) >= 0 ? 1 : -1;
      const s = dist * sign;

      signed[i] = s;
      const a = Math.abs(s);
      if (a > maxAbs) maxAbs = a;
      sumSq += s * s;
      sum += s;
    }

    const clamp = range ?? (maxAbs > 0 ? maxAbs : 1);
    const colors = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      const [r, g, b] = diverging(signed[i]!, clamp);
      colors[i * 3] = r;
      colors[i * 3 + 1] = g;
      colors[i * 3 + 2] = b;
    }

    return { signed, colors, maxAbs, rms: Math.sqrt(sumSq / n), mean: sum / n };
  }
}
