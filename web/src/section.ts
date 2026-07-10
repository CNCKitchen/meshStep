// SPDX-License-Identifier: AGPL-3.0-only

// Solid section view, ported from smartInfillGenerator's section-plane rig:
// one clipping plane cuts the model, and a stencil-buffer cap (the three.js
// clipping_stencil technique) fills the cut cross-section so the part reads
// as a solid instead of a hollow shell. The plane is posed with a combined
// gizmo — a proxy Object3D carrying a translucent quad, one TransformControls
// that translates along the plane normal, and one that rotates about the two
// in-plane axes (spinning about the normal is a no-op and stays hidden).

import * as THREE from "three";
import { TransformControls } from "three/addons/controls/TransformControls.js";

/** Cut-face colors of the capped section view — matte tones clearly distinct
 *  from the part gray (CAD convention: the section face reads as "cut
 *  material"). A single body gets the clay (same color as
 *  smartInfillGenerator's caps); an assembly cycles the palette so adjacent
 *  bodies' cross-sections read as different parts. */
const CAP_PALETTE = [
  0xbe7b4d, // clay
  0x5b8db8, // steel blue
  0x7aa25c, // moss
  0xb05c5c, // brick
  0x8d6fc0, // violet
  0xc9a83c, // ochre
  0x4fb3a5, // teal
  0xb0699e, // magenta
];
const PLANE_COLOR = 0x2e6fd0;

/** What the section rig needs from the viewer. */
export interface SectionHost {
  scene: THREE.Scene;
  camera(): THREE.Camera;
  domElement(): HTMLElement;
  /** A handle drag started/ended: gate the orbit controls while dragging. */
  onDraggingChanged(dragging: boolean): void;
  /** Model bbox diagonal (sizes the quad + cap), safe fallback pre-model. */
  bboxDiag(): number;
  /** Model bbox center (initial plane position), orbit target pre-model. */
  partCenter(): THREE.Vector3;
}

export class SectionController {
  /** The clipping plane (three.js convention: kept side is n·p + c ≥ 0).
   *  Mutated in place, so materials holding it in clippingPlanes track it. */
  readonly plane = new THREE.Plane(new THREE.Vector3(-1, 0, 0), 0);

  /** Pose carrier for the plane: local +Z is the plane normal. */
  private proxy = new THREE.Object3D();
  private translate: TransformControls | null = null;
  private rotate: TransformControls | null = null;
  private quad: THREE.Group | null = null;
  private quadDisposables: { dispose(): void }[] = [];

  // Stencil caps: per color bucket, two colorless meshes count back/front
  // faces of the clipped solid into the stencil buffer and a plane-aligned
  // quad fills where != 0 (clearing the stencil for the next bucket).
  private capObjects: THREE.Object3D[] = [];
  private capQuads: THREE.Mesh[] = [];
  /** Bucket index geometries — position attribute shared with the solid, own
   *  index; setHiddenSolids swaps the indices without touching the vertices. */
  private bucketGeos: THREE.BufferGeometry[] = [];
  private capDisposables: { dispose(): void }[] = [];
  private geometry: THREE.BufferGeometry | null = null;
  private fullIndex: Uint32Array | null = null;
  private solidOfTri: Uint32Array | null = null;
  /** Palette slot per solid id (stable under hiding). Empty = single bucket. */
  private slotOfSolid = new Map<number, number>();
  private slotCount = 1;
  private hiddenSolids: ReadonlySet<number> = new Set();

  private on = false;
  private capsAllowed = true; // host gate: hidden while transparent / edges-only

  constructor(private readonly host: SectionHost) {}

  get enabled(): boolean {
    return this.on;
  }

  /** A gizmo handle is hovered or dragged — the viewer's own left-drag orbit
   *  must yield so grabbing an arrow doesn't also spin the camera. */
  busy(): boolean {
    const t = this.translate;
    const r = this.rotate;
    return this.on && !!t && !!r && (t.dragging || r.dragging || t.axis !== null || r.axis !== null);
  }

  setEnabled(on: boolean): void {
    this.on = on;
    if (on) this.ensure();
    this.updateVisibility();
  }

  /** The viewer swapped persp/ortho — TransformControls raycast the camera. */
  setCamera(camera: THREE.Camera): void {
    if (this.translate) this.translate.camera = camera;
    if (this.rotate) this.rotate.camera = camera;
  }

  /** New (or cleared) solid geometry to cap. `fullIndex` is the unfiltered
   *  triangle index and `solidOfTri` each triangle's body id (same order) —
   *  they assign every body a stable palette slot so an assembly's cut faces
   *  read as distinct parts, and let per-part hiding drop cap triangles. */
  setGeometry(
    geometry: THREE.BufferGeometry | null,
    fullIndex: Uint32Array | null = null,
    solidOfTri: Uint32Array | null = null
  ): void {
    this.geometry = geometry;
    this.fullIndex = fullIndex;
    this.solidOfTri = solidOfTri;
    this.hiddenSolids = new Set();
    // Palette slots by body rank (ascending id): stable while parts are
    // hidden/shown, and adjacent ids — usually adjacent parts — differ.
    this.slotOfSolid.clear();
    if (solidOfTri) {
      const ids = [...new Set(solidOfTri)].sort((a, b) => a - b);
      ids.forEach((id, rank) => this.slotOfSolid.set(id, rank % CAP_PALETTE.length));
    }
    this.slotCount = this.slotOfSolid.size ? Math.min(this.slotOfSolid.size, CAP_PALETTE.length) : 1;
    if (this.translate) this.rebuildCaps();
  }

  /** Per-part hiding: rebuild the bucket indices without the hidden bodies
   *  (index swap only — the shared vertex buffer stays untouched). */
  setHiddenSolids(hidden: ReadonlySet<number>): void {
    this.hiddenSolids = new Set(hidden);
    if (this.bucketGeos.length) this.refillBucketIndices();
  }

  /** A new model arrived: re-center the plane through it, refit the quad. */
  refit(): void {
    if (!this.translate) return;
    this.proxy.position.copy(this.host.partCenter());
    this.buildQuad();
    this.sync();
  }

  flip(): void {
    if (!this.translate) return;
    this.proxy.rotateX(Math.PI); // local +Z (= plane normal) flips
    this.sync();
  }

  setAxis(axis: "x" | "y" | "z"): void {
    if (!this.translate) return;
    this.proxy.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 0, 1),
      this.normalTowardCut(axis)
    );
    this.sync();
  }

  /** Solid caps only make sense on an opaque, visible solid — the host turns
   *  them off for the transparent and edges-only views. */
  setCapsAllowed(v: boolean): void {
    this.capsAllowed = v;
    this.updateVisibility();
  }

  /** Axis-aligned section normal SIGNED so the camera sits on the clipped
   *  side — the cut always opens toward the viewer instead of hiding on the
   *  part's far side. No `axis`: the dominant axis of the view direction
   *  (initial plane on activation). Clipping keeps n·p + c ≥ 0, so "camera
   *  clipped" means the normal points WITH the view direction. */
  private normalTowardCut(axis?: "x" | "y" | "z"): THREE.Vector3 {
    const dir = this.host.partCenter().sub(this.host.camera().position);
    const a =
      axis ??
      (Math.abs(dir.x) >= Math.abs(dir.y) && Math.abs(dir.x) >= Math.abs(dir.z)
        ? "x"
        : Math.abs(dir.y) >= Math.abs(dir.z)
          ? "y"
          : "z");
    const n = new THREE.Vector3(a === "x" ? 1 : 0, a === "y" ? 1 : 0, a === "z" ? 1 : 0);
    if (n.dot(dir) < 0) n.negate();
    return n;
  }

  /** Hover arbitration between the two overlapping TransformControls: each
   *  raycasts its own fat invisible pickers independently, and the rotation
   *  rings pass right THROUGH the arrow-tip region — so aiming at the arrow
   *  routinely started a ROTATE. The arrow wins: whenever the translate
   *  control hovers its axis, the rotate control is disabled (highlight
   *  cleared) so its pointerdown is a no-op. Registered AFTER the controls'
   *  own listeners, so both hover states are fresh when it runs. */
  private arbitrateHover = () => {
    const t = this.translate;
    const r = this.rotate;
    if (!t || !r || t.dragging || r.dragging) return;
    const wantRotate = this.on && t.axis === null;
    if (r.enabled !== wantRotate) {
      r.enabled = wantRotate;
      if (!wantRotate) r.axis = null; // drop the ring highlight under the arrow
    }
  };

  /** Lazily create the proxy + controls + quad on first enable. */
  private ensure(): void {
    if (this.translate) return;
    // Through the part's center, normal along the dominant view axis, opening
    // the cut TOWARD the camera (the near half is the clipped one).
    this.proxy.position.copy(this.host.partCenter());
    this.proxy.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), this.normalTowardCut());
    this.host.scene.add(this.proxy);

    const make = (mode: "translate" | "rotate", size: number, tune: (tc: TransformControls) => void) => {
      const tc = new TransformControls(this.host.camera(), this.host.domElement() as HTMLCanvasElement);
      tc.setMode(mode);
      tc.setSpace("local");
      tc.setSize(size);
      tune(tc);
      tc.addEventListener("dragging-changed", (e: { value?: unknown }) => {
        this.host.onDraggingChanged(!!e.value);
      });
      tc.addEventListener("objectChange", () => this.sync());
      tc.attach(this.proxy);
      this.host.scene.add(tc.getHelper());
      return tc;
    };
    // The plane cuts everything, so tangential motion is meaningless — only
    // the normal arrow translates; two rings rotate.
    this.translate = make("translate", 0.75, (tc) => {
      tc.showX = false;
      tc.showY = false;
    });
    this.rotate = make("rotate", 1.05, (tc) => {
      tc.showZ = false;
    });
    // After the controls' own pointermove listeners → their hover state is
    // current when the arbiter decides (see arbitrateHover).
    this.host.domElement().addEventListener("pointermove", this.arbitrateHover);
    this.buildQuad();
    this.rebuildCaps();
    this.sync();
  }

  /** (Re)build the translucent plane rectangle, child of the proxy so it is
   *  always centered on the gizmo. */
  private buildQuad(): void {
    if (this.quad) {
      this.proxy.remove(this.quad);
      for (const d of this.quadDisposables) d.dispose();
      this.quadDisposables = [];
    }
    const s = this.host.bboxDiag() * 1.15;
    const group = new THREE.Group();
    const quadGeo = new THREE.PlaneGeometry(s, s);
    const quadMat = new THREE.MeshBasicMaterial({
      color: PLANE_COLOR,
      transparent: true,
      opacity: 0.08,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    const edgeGeo = new THREE.EdgesGeometry(quadGeo);
    const edgeMat = new THREE.LineBasicMaterial({ color: PLANE_COLOR, transparent: true, opacity: 0.7 });
    this.quadDisposables.push(quadGeo, quadMat, edgeGeo, edgeMat);
    group.add(new THREE.Mesh(quadGeo, quadMat));
    group.add(new THREE.LineSegments(edgeGeo, edgeMat));
    this.quad = group;
    this.proxy.add(group);
  }

  /** Triangle indices per palette slot, hidden bodies dropped. A model with
   *  no body info is one bucket holding the full index. */
  private bucketIndices(): Uint32Array[] {
    const full = this.fullIndex ?? ((this.geometry?.getIndex()?.array as Uint32Array) || null);
    if (!full) return [];
    const sot = this.solidOfTri;
    if (!sot || !this.slotOfSolid.size) return [full];
    const counts: number[] = new Array(this.slotCount).fill(0);
    for (let t = 0; t < sot.length; t++) {
      if (!this.hiddenSolids.has(sot[t]!)) counts[this.slotOfSolid.get(sot[t]!)!]++;
    }
    const out = counts.map((c) => new Uint32Array(c * 3));
    const at: number[] = new Array(this.slotCount).fill(0);
    for (let t = 0; t < sot.length; t++) {
      const id = sot[t]!;
      if (this.hiddenSolids.has(id)) continue;
      const s = this.slotOfSolid.get(id)!;
      const o = out[s]!;
      o[at[s]!] = full[t * 3]!;
      o[at[s]! + 1] = full[t * 3 + 1]!;
      o[at[s]! + 2] = full[t * 3 + 2]!;
      at[s]! += 3;
    }
    return out;
  }

  private refillBucketIndices(): void {
    const indices = this.bucketIndices();
    this.bucketGeos.forEach((g, i) => {
      g.setIndex(new THREE.BufferAttribute(indices[i] ?? new Uint32Array(0), 1));
    });
  }

  /** Stencil-buffer caps (three.js clipping_stencil technique), one group per
   *  palette slot: back faces of that slot's clipped bodies increment, front
   *  faces decrement; a plane quad in the slot's color drawn where
   *  stencil != 0 fills the cut, then clears the stencil for the next slot.
   *  Bodies never overlap in space, so the slots' cut regions are disjoint
   *  and the coplanar quads cannot fight. */
  private rebuildCaps(): void {
    for (const o of this.capObjects) this.host.scene.remove(o);
    for (const d of this.capDisposables) d.dispose();
    this.capObjects = [];
    this.capQuads = [];
    this.bucketGeos = [];
    this.capDisposables = [];
    if (!this.geometry) return;

    const position = this.geometry.getAttribute("position");
    const capSize = this.host.bboxDiag() * 4;
    this.bucketIndices().forEach((idx, slot) => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", position);
      geo.setIndex(new THREE.BufferAttribute(idx, 1));
      geo.boundingSphere = this.geometry!.boundingSphere;
      this.bucketGeos.push(geo);
      this.capDisposables.push(geo);

      const stencilBase = () => {
        const m = new THREE.MeshBasicMaterial();
        m.depthWrite = false;
        m.depthTest = false;
        m.colorWrite = false;
        m.stencilWrite = true;
        m.stencilFunc = THREE.AlwaysStencilFunc;
        m.clippingPlanes = [this.plane];
        this.capDisposables.push(m);
        return m;
      };
      const backMat = stencilBase();
      backMat.side = THREE.BackSide;
      backMat.stencilFail = THREE.IncrementWrapStencilOp;
      backMat.stencilZFail = THREE.IncrementWrapStencilOp;
      backMat.stencilZPass = THREE.IncrementWrapStencilOp;
      const frontMat = stencilBase();
      frontMat.side = THREE.FrontSide;
      frontMat.stencilFail = THREE.DecrementWrapStencilOp;
      frontMat.stencilZFail = THREE.DecrementWrapStencilOp;
      frontMat.stencilZPass = THREE.DecrementWrapStencilOp;
      const back = new THREE.Mesh(geo, backMat);
      const front = new THREE.Mesh(geo, frontMat);
      // Slots run strictly in order (counters, then that slot's cap) so each
      // cap sees exactly its own stencil count. All before highlight (5) /
      // open edges (10).
      const order = 1 + slot * 0.02;
      back.renderOrder = order;
      front.renderOrder = order;

      const capGeo = new THREE.PlaneGeometry(capSize, capSize);
      const capMat = new THREE.MeshStandardMaterial({
        color: CAP_PALETTE[slot % CAP_PALETTE.length],
        metalness: 0.05,
        roughness: 0.8,
        // The cut is looked at from the REMOVED side — the quad backfaces the
        // viewer there, so it must render double-sided or it culls away.
        side: THREE.DoubleSide,
        stencilWrite: true,
        stencilRef: 0,
        stencilFunc: THREE.NotEqualStencilFunc,
        stencilFail: THREE.ReplaceStencilOp,
        stencilZFail: THREE.ReplaceStencilOp,
        stencilZPass: THREE.ReplaceStencilOp,
      });
      this.capDisposables.push(capGeo, capMat);
      const cap = new THREE.Mesh(capGeo, capMat);
      cap.renderOrder = order + 0.01;
      cap.onAfterRender = (renderer) => renderer.clearStencil();
      cap.position.copy(this.proxy.position);
      cap.quaternion.copy(this.proxy.quaternion);
      this.capQuads.push(cap);
      this.capObjects.push(back, front, cap);
    });
    for (const o of this.capObjects) this.host.scene.add(o);
    this.updateVisibility();
  }

  /** Re-derive the plane from the proxy pose and keep the cap quad on it.
   *  Fired on every handle drag (objectChange) and programmatic moves. */
  private sync(): void {
    const n = new THREE.Vector3(0, 0, 1).applyQuaternion(this.proxy.quaternion);
    this.plane.setFromNormalAndCoplanarPoint(n, this.proxy.position);
    for (const cap of this.capQuads) {
      cap.position.copy(this.proxy.position);
      cap.quaternion.copy(this.proxy.quaternion);
    }
  }

  private updateVisibility(): void {
    this.proxy.visible = this.on;
    for (const tc of [this.translate, this.rotate]) {
      if (tc) {
        tc.enabled = this.on;
        tc.getHelper().visible = this.on;
      }
    }
    const caps = this.on && this.capsAllowed;
    for (const o of this.capObjects) o.visible = caps;
  }
}
