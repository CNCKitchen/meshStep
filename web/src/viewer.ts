// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ViewHelper } from "three/addons/helpers/ViewHelper.js";
import { CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";
import { MeshBVH } from "three-mesh-bvh";
import { filterTriangles, filterSegments, type EdgeSet } from "./mesh-utils.ts";
import { SectionController } from "./section.ts";
import { MeasureController, type MeasureMode } from "./measure.ts";
import type { MeasureGeometry } from "../../src/index.ts";

interface Content {
  group: THREE.Group;
  solid: THREE.Mesh | null;
  wire: THREE.LineSegments | null;
  edges: THREE.LineSegments | null;
  feature: THREE.LineSegments | null;
  reference: THREE.Mesh | null;
  highlight: THREE.Mesh | null;
}

export const BASE_COLOR = 0x6f8fb0;
const REF_COLOR = 0x33dd88;
const EDGE_COLOR = 0xff3b30;
const FEATURE_COLOR = 0x0b0e12;
const HIGHLIGHT_COLOR = 0xffa62b;

type CameraView = "top" | "bottom" | "front" | "behind" | "left" | "right" | "iso";

/** Keyboard camera presets (Ctrl+0 = isometric, like a browser zoom reset). */
const VIEW_KEYS: Record<string, CameraView> = {
  "0": "iso",
  "1": "top",
  "2": "bottom",
  "3": "front",
  "4": "behind",
  "5": "left",
  "6": "right",
};

/** Isometric view direction (offset from the part toward the camera), Z-up. */
const ISO_DIR = new THREE.Vector3(0.7, -0.8, 0.55);

/**
 * A single orbit-controlled viewport showing the tessellated mesh, with optional
 * wireframe, open-edge highlight, and a reference-STL overlay / deviation colors.
 */
export class Viewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private persp: THREE.PerspectiveCamera;
  private ortho: THREE.OrthographicCamera;
  private camera: THREE.PerspectiveCamera | THREE.OrthographicCamera;
  private controls: OrbitControls;
  private content: Content;
  private container: HTMLElement;

  // bumpMesh-style navigation (same scheme as smartInfillGenerator): left-drag
  // orbits around the surface point under the cursor (free over the poles),
  // the wheel zooms toward the cursor, and right-drag pans in screen space.
  // OrbitControls keeps only damping + right-drag pan; rotation and zoom are
  // handled manually below.
  private pivotMarker: THREE.Mesh;
  private orbitPivot: THREE.Vector3 | null = null; // active drag pivot
  private lastOrbitPivot: THREE.Vector3 | null = null; // fallback between drags
  private orbitStart: { x: number; y: number } | null = null;
  private orbitLast: { x: number; y: number } | null = null;
  private orbiting = false;
  private raycaster = new THREE.Raycaster();
  private pointerNdc = new THREE.Vector2();
  private _oq1 = new THREE.Quaternion();
  private _oq2 = new THREE.Quaternion();
  private _oRight = new THREE.Vector3();
  private _oTmp = new THREE.Vector3();
  private _oTmp2 = new THREE.Vector3();
  private _oDir = new THREE.Vector3();
  private _oUp = new THREE.Vector3();

  // toggle state
  private orthoOn = false;
  private showWire = false;
  private showEdges = false;
  private showFeature = true; // signature CAD face-border look, on by default
  private showReference = false;
  private deviationOn = false;
  private showColors = true; // STEP face colors, on by default when the file has them
  private transparentOn = false;
  private solidVisible = true; // false => CAD-edges-only view

  // Per-vertex color sources for the solid mesh; applyVisibility picks which one is active
  // (deviation analysis wins over the model's own face colors while its toggle is on).
  private faceColors: Float32Array | null = null;
  private devColors: Float32Array | null = null;

  // Per-part visibility: the full (unfiltered) triangle index plus per-triangle / per-segment
  // solid ids, so hiding a part rebuilds the drawn subsets without touching the source data.
  private solidOfTri: Uint32Array | null = null;
  private fullIndex: Uint32Array | null = null;
  private boundarySet: EdgeSet | null = null;
  private featureSet: EdgeSet | null = null;
  private hiddenSolids = new Set<number>();
  private wireDirty = false; // wireframe geometry lags hidden-part changes until it is shown
  // Per-triangle solid ids matching the CURRENT (possibly part-filtered) index, so a raycast
  // faceIndex maps straight to the body under the cursor.
  private drawnSolidOfTri: Uint32Array | null = null;
  private rmbStart: { x: number; y: number } | null = null; // right-button down position

  /** Right-click (no drag) hook: body id under the cursor (null over empty space) + client x/y. */
  onContextMenu: ((solidId: number | null, x: number, y: number) => void) | null = null;

  // Section view: one clipping plane + stencil caps + plane gizmo (section.ts).
  private section: SectionController;
  // Axes triad in the lower-right corner showing the world orientation.
  private viewHelper: ViewHelper;

  // Measurement mode: snapping, markers and committed dimensions (measure.ts). Labels render as
  // DOM elements through a CSS2DRenderer overlay pass (theme-aware, never section-clipped).
  private measure: MeasureController;
  private labelRenderer: CSS2DRenderer;
  // Lazy BVH over the FULL (unfiltered) triangle index for measurement raycasts: built on first
  // use (multi-million-tri builds would jank every conversion otherwise), survives part hiding
  // (hits are filtered by solidOfTri instead), invalidated when the model changes.
  private pickBvh: MeshBVH | null = null;
  private pickGeo: THREE.BufferGeometry | null = null;

  /** Measure mode was exited internally (ESC) — sync the sidebar toggle. */
  onMeasureExit: (() => void) | null = null;

  constructor(container: HTMLElement) {
    this.container = container;
    // stencil: required for the filled section caps (default off since r163).
    this.renderer = new THREE.WebGLRenderer({ antialias: true, stencil: true });
    this.renderer.localClippingEnabled = true;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    // The axes triad renders as a second pass into the same frame.
    this.renderer.autoClear = false;
    container.appendChild(this.renderer.domElement);

    this.scene.background = new THREE.Color(0x14181d);

    // Engineering convention: Z up (STEP models are typically Z-up).
    this.persp = new THREE.PerspectiveCamera(45, 1, 0.01, 1e6);
    this.persp.up.set(0, 0, 1);
    this.persp.position.set(70, -80, 55);
    this.ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 1e6);
    this.ortho.up.set(0, 0, 1);
    this.ortho.position.set(70, -80, 55);
    this.camera = this.persp;

    this.controls = this.makeControls(new THREE.Vector3());
    this.viewHelper = this.makeViewHelper();

    // Lights (Z-up: key from above-front, fill from behind-below).
    const hemi = new THREE.HemisphereLight(0xffffff, 0x404048, 1.0);
    hemi.position.set(0, 0, 1);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(1, -1.2, 1.8);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.7);
    fill.position.set(-1.5, 1, -0.5);
    this.scene.add(fill);

    // Small red sphere marking the live orbit centre (drawn over the part).
    this.pivotMarker = new THREE.Mesh(
      new THREE.SphereGeometry(1, 16, 10),
      new THREE.MeshBasicMaterial({ color: 0xff2222, depthTest: false })
    );
    this.pivotMarker.renderOrder = 10;
    this.pivotMarker.visible = false;
    this.scene.add(this.pivotMarker);

    const group = new THREE.Group();
    this.scene.add(group);
    this.content = { group, solid: null, wire: null, edges: null, feature: null, reference: null, highlight: null };

    this.section = new SectionController({
      scene: this.scene,
      camera: () => this.camera,
      domElement: () => this.renderer.domElement,
      // this.controls is reassigned on projection swaps — resolve it late.
      onDraggingChanged: (dragging) => { this.controls.enabled = !dragging; },
      bboxDiag: () => this.contentDiag(),
      partCenter: () => this.contentCenter(),
    });

    // DOM-label overlay pass (measurement readouts). pointer-events: none so orbit/wheel/
    // section-gizmo interactions pass straight through to the canvas.
    this.labelRenderer = new CSS2DRenderer();
    const lr = this.labelRenderer.domElement;
    lr.style.position = "absolute";
    lr.style.inset = "0";
    lr.style.pointerEvents = "none";
    container.appendChild(lr);

    this.measure = new MeasureController({
      scene: this.scene,
      camera: () => this.camera,
      domElement: () => this.renderer.domElement,
      raycastSurface: (x, y) => this.raycastSurface(x, y),
      sectionPlane: () => (this.section.enabled ? this.section.plane : null),
      hiddenSolids: () => this.hiddenSolids,
      busy: () => this.section.busy(),
      onEnabledChanged: () => this.onMeasureExit?.(),
      bboxDiag: () => this.contentDiag(),
    });

    this.installNavigation(this.renderer.domElement);

    window.addEventListener("resize", () => this.resize());
    this.resize();
    this.animate();
  }

  private resize(): void {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.renderer.setSize(w, h, true);
    this.labelRenderer.setSize(w, h);
    this.persp.aspect = w / h;
    this.persp.updateProjectionMatrix();
    this.setOrthoFrustum(this.ortho.top - this.ortho.bottom); // preserve current world height
    this.ortho.updateProjectionMatrix();
  }

  /** Build an OrbitControls bound to the active camera, aimed at `target`. */
  private makeControls(target: THREE.Vector3): OrbitControls {
    const c = new OrbitControls(this.camera, this.renderer.domElement);
    c.enableDamping = true;
    c.dampingFactor = 0.08;
    // Right-drag pans in the screen plane (not along the ground).
    c.screenSpacePanning = true;
    // Rotation + zoom are manual (pivot-on-cursor orbit, cursor-centric
    // zoom — see installNavigation). OrbitControls keeps damping + R-drag pan.
    c.enableRotate = false;
    c.enableZoom = false;
    // Full polar range: the manual orbit (onOrbitMove) already clamps its own
    // pitch to poleEps, so the reconstruction never lands on the ±Z pole
    // during a drag. The only on-pole placements are the top/bottom presets
    // (setCameraView), which set up = +Y so lookAt stays well-defined.
    c.minPolarAngle = 0;
    c.maxPolarAngle = Math.PI;
    c.target.copy(target);
    c.update();
    return c;
  }

  /** Size the orthographic frustum to a given world height, keeping the viewport aspect. */
  private setOrthoFrustum(worldHeight: number): void {
    const aspect = this.container.clientWidth / Math.max(1, this.container.clientHeight);
    const halfH = Math.max(worldHeight, 1e-3) / 2;
    this.ortho.top = halfH;
    this.ortho.bottom = -halfH;
    this.ortho.left = -halfH * aspect;
    this.ortho.right = halfH * aspect;
  }

  /** Switch between perspective and orthographic projection, preserving the view. */
  setProjection(ortho: boolean): void {
    if (ortho === this.orthoOn) return;
    const target = this.controls.target.clone();
    const pos = this.camera.position.clone();
    const dist = pos.distanceTo(target);
    this.orthoOn = ortho;
    if (ortho) {
      // Match the perspective view's apparent size at the target distance.
      this.setOrthoFrustum(2 * dist * Math.tan((this.persp.fov * Math.PI) / 360));
      this.ortho.position.copy(pos);
      this.ortho.up.copy(this.persp.up); // free orbit may have rolled the view
      this.ortho.zoom = 1;
      this.ortho.updateProjectionMatrix();
      this.camera = this.ortho;
    } else {
      this.persp.position.copy(pos);
      this.persp.up.copy(this.ortho.up);
      this.persp.updateProjectionMatrix();
      this.camera = this.persp;
    }
    this.controls.dispose();
    this.controls = this.makeControls(target);
    this.section.setCamera(this.camera); // the gizmo raycasts the active camera
    // ViewHelper captures its camera in a closure — rebuild for the new one.
    this.viewHelper.dispose();
    this.viewHelper = this.makeViewHelper();
  }

  /** Corner axes triad bound to the active camera. */
  private makeViewHelper(): ViewHelper {
    const h = new ViewHelper(this.camera, this.renderer.domElement);
    h.setLabels("X", "Y", "Z");
    return h;
  }

  /** Model bbox center in world units (orbit target before a model loads). */
  private contentCenter(): THREE.Vector3 {
    const s = this.content.solid;
    if (!s) return this.controls.target.clone();
    return new THREE.Box3().expandByObject(s).getCenter(new THREE.Vector3());
  }

  /** Model bbox diagonal (sizes the section quad/cap; safe pre-model fallback). */
  private contentDiag(): number {
    const s = this.content.solid;
    if (!s) return 100;
    return new THREE.Box3().expandByObject(s).getSize(new THREE.Vector3()).length() || 100;
  }

  // ---------- navigation (orbit / move / zoom) ----------
  // Same camera routine as smartInfillGenerator (bumpMesh-style): left-drag
  // orbits around the surface point under the cursor with no polar clamping,
  // the wheel zooms toward the cursor, and right-drag pans in screen space.

  private installNavigation(canvas: HTMLCanvasElement): void {
    canvas.addEventListener("pointerdown", (ev) => {
      if (ev.button === 0) this.beginOrbit(ev);
      else if (ev.button === 2) this.rmbStart = { x: ev.clientX, y: ev.clientY };
    });
    // Right-drag pans (OrbitControls) — the context menu only opens on a clean right CLICK.
    canvas.addEventListener("contextmenu", (ev) => {
      ev.preventDefault();
      const s = this.rmbStart;
      this.rmbStart = null;
      if (s && Math.hypot(ev.clientX - s.x, ev.clientY - s.y) > 4) return;
      this.onContextMenu?.(this.pickSolid(ev.clientX, ev.clientY), ev.clientX, ev.clientY);
    });
    // Move + release on document so a drag that leaves the canvas still tracks.
    document.addEventListener("pointermove", this.onOrbitMove);
    document.addEventListener("pointerup", this.onOrbitUp);
    document.addEventListener("keydown", this.onViewKey);
    canvas.addEventListener("wheel", this.onWheel, { passive: false });
  }

  /** Meshes the orbit ray can land on: the model itself (even in the
   *  edges-only view, where the solid is hidden but still the part) and the
   *  reference overlay while it is shown. */
  private orbitTargets(): THREE.Object3D[] {
    const list: THREE.Object3D[] = [];
    if (this.content.solid) list.push(this.content.solid);
    if (this.content.reference?.visible) list.push(this.content.reference);
    return list;
  }

  /** Nearest surface point under the cursor, or null if the ray misses. */
  private pickPoint(ev: PointerEvent): THREE.Vector3 | null {
    const targets = this.orbitTargets();
    if (!targets.length) return null;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointerNdc.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointerNdc.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);
    const hits = this.raycaster.intersectObjects(targets, false);
    // Section view: the raycaster still hits clipped-away (invisible) surface —
    // pivot on the first hit on the KEPT side (n·p + c ≥ 0) instead.
    if (this.section.enabled) {
      const kept = hits.find((h) => this.section.plane.distanceToPoint(h.point) > -1e-6);
      return kept ? kept.point.clone() : null;
    }
    return hits.length ? hits[0].point.clone() : null;
  }

  /** Body (solid) id under the given client position, or null over empty space / no model.
   *  Hidden parts can't be hit (their triangles are filtered out of the drawn index); in the
   *  section view, hits on the clipped-away side are skipped (same rule as the orbit pivot). */
  private pickSolid(clientX: number, clientY: number): number | null {
    const c = this.content;
    if (!c.solid || !this.drawnSolidOfTri) return null;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointerNdc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
    this.pointerNdc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);
    const hits = this.raycaster.intersectObject(c.solid, false);
    const hit = this.section.enabled
      ? hits.find((h) => this.section.plane.distanceToPoint(h.point) > -1e-6)
      : hits[0];
    if (!hit || hit.faceIndex == null) return null;
    return this.drawnSolidOfTri[hit.faceIndex] ?? null;
  }

  /** Lazy full-index BVH for measurement raycasts. `indirect: true` is essential: the default
   *  build REORDERS the index in place, which would scramble the triangle -> solidOfTri mapping
   *  (and the drawn mesh, since the position/index buffers are shared with the visible solid). */
  private ensurePickBvh(): MeshBVH | null {
    if (this.pickBvh) return this.pickBvh;
    const c = this.content;
    if (!c.solid || !this.fullIndex) return null;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", c.solid.geometry.getAttribute("position"));
    geo.setIndex(new THREE.BufferAttribute(this.fullIndex, 1));
    this.pickGeo = geo;
    this.pickBvh = new MeshBVH(geo, { indirect: true });
    return this.pickBvh;
  }

  /** Nearest VISIBLE surface point under the client coords for the measure tool: BVH raycast on
   *  the full mesh, skipping hidden parts and (in section view) hits on the clipped-away side. */
  private raycastSurface(clientX: number, clientY: number): THREE.Vector3 | null {
    const bvh = this.ensurePickBvh();
    if (!bvh) return null;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointerNdc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
    this.pointerNdc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);
    const hits = bvh.raycast(this.raycaster.ray, THREE.DoubleSide);
    hits.sort((a, b) => a.distance - b.distance);
    for (const h of hits) {
      if (h.faceIndex != null && this.solidOfTri && this.hiddenSolids.has(this.solidOfTri[h.faceIndex]!)) continue;
      if (this.section.enabled && this.section.plane.distanceToPoint(h.point) < -1e-6) continue;
      return h.point.clone();
    }
    return null;
  }

  private beginOrbit(ev: PointerEvent): void {
    // Grabbing a section-gizmo handle must not also spin the camera.
    if (this.section.busy()) return;
    // Orbit about the surface under the cursor; fall back to the last pivot,
    // then to the controls target, so a drag off the part still rotates.
    const pivot = this.pickPoint(ev) ?? this.lastOrbitPivot ?? this.controls.target.clone();
    this.orbitPivot = pivot.clone();
    this.lastOrbitPivot = pivot.clone();
    this.orbitStart = { x: ev.clientX, y: ev.clientY };
    this.orbitLast = { x: ev.clientX, y: ev.clientY };
    this.orbiting = false; // promoted once the drag passes the threshold
  }

  private showPivotMarker(): void {
    const m = this.pivotMarker;
    if (!this.orbitPivot) return;
    m.position.copy(this.orbitPivot);
    // ~1.5% of the visible (half-)frustum height at the pivot: same apparent
    // size at any zoom, in either projection.
    const halfH = this.orthoOn
      ? this.ortho.top / this.ortho.zoom
      : this.persp.position.distanceTo(this.orbitPivot) * Math.tan((this.persp.fov * Math.PI) / 360);
    m.scale.setScalar(halfH * 0.015);
    m.visible = true;
  }

  private onOrbitMove = (ev: PointerEvent): void => {
    if (!this.orbitPivot || !this.orbitLast) return;
    if (!this.orbiting) {
      const moved = Math.hypot(ev.clientX - this.orbitStart!.x, ev.clientY - this.orbitStart!.y);
      if (moved < 3) return; // tolerate a click without flashing the marker
      this.orbiting = true;
      this.showPivotMarker();
    }
    const dx = ev.clientX - this.orbitLast.x;
    const dy = ev.clientY - this.orbitLast.y;
    this.orbitLast = { x: ev.clientX, y: ev.clientY };
    if (dx === 0 && dy === 0) return;

    const pivot = this.orbitPivot;
    const rotSpeed = 0.005;
    // Free trackball orbit: yaw about the camera's own up axis, pitch about
    // its right axis — both screen-relative, so there is no polar clamp and
    // no special pole. camera.up follows the same rotation; OrbitControls'
    // per-frame `lookAt(target)` then reproduces this orientation exactly at
    // ANY tilt (up is never parallel to the view direction).
    this.camera.updateMatrixWorld();
    this._oRight.setFromMatrixColumn(this.camera.matrixWorld, 0).normalize(); // camera right
    this._oUp.setFromMatrixColumn(this.camera.matrixWorld, 1).normalize(); // camera up

    this._oq1.setFromAxisAngle(this._oUp, -dx * rotSpeed);
    this._oq2.setFromAxisAngle(this._oRight, -dy * rotSpeed);
    this._oq1.premultiply(this._oq2);

    // Swing both the camera and the orbit target around the pivot so
    // OrbitControls (which owns damping + pan) stays consistent.
    this._oTmp.copy(this.camera.position).sub(pivot).applyQuaternion(this._oq1);
    this.camera.position.copy(pivot).add(this._oTmp);
    this._oTmp2.copy(this.controls.target).sub(pivot).applyQuaternion(this._oq1);
    this.controls.target.copy(pivot).add(this._oTmp2);
    this.camera.up.applyQuaternion(this._oq1);
    this.camera.quaternion.premultiply(this._oq1);
    this.camera.updateMatrixWorld();
  };

  private onOrbitUp = (): void => {
    if (!this.orbitPivot) return;
    this.orbitPivot = null;
    this.orbitStart = null;
    this.orbitLast = null;
    this.orbiting = false; // the free orbit keeps its orientation — no re-leveling
    this.pivotMarker.visible = false;
  };

  /** Cursor-centric zoom: keep the world point under the cursor pinned. */
  private onWheel = (ev: WheelEvent): void => {
    ev.preventDefault();
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndcX = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    const ndcY = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
    const factor = ev.deltaY > 0 ? 1 / 1.1 : 1.1;
    if (this.orthoOn) {
      // Scale the frustum, then shift so the cursor's world point re-projects
      // to the same screen position.
      this._oTmp.set(ndcX, ndcY, 0).unproject(this.ortho);
      this.ortho.zoom = Math.max(0.05, Math.min(200, this.ortho.zoom * factor));
      this.ortho.updateProjectionMatrix();
      this._oTmp2.set(ndcX, ndcY, 0).unproject(this.ortho);
      this._oTmp.sub(this._oTmp2);
      this.ortho.position.add(this._oTmp);
      this.controls.target.add(this._oTmp);
    } else {
      // Perspective: scale camera + target about the world point under the
      // cursor at the target-plane depth — its projection stays pinned while
      // everything grows/shrinks by `factor`.
      const cam = this.persp;
      cam.updateMatrixWorld();
      const fwd = cam.getWorldDirection(this._oDir);
      this._oTmp.set(ndcX, ndcY, 0.5).unproject(cam).sub(cam.position).normalize();
      const denom = this._oTmp.dot(fwd);
      const planeDist = this._oTmp2.copy(this.controls.target).sub(cam.position).dot(fwd);
      if (denom < 1e-6 || planeDist < 1e-9) return;
      const p = this._oTmp.multiplyScalar(planeDist / denom).add(cam.position);
      const s = 1 / factor;
      // Don't dolly into the near plane.
      if (factor > 1 && cam.position.distanceTo(p) * s < cam.near * 2) return;
      cam.position.sub(p).multiplyScalar(s).add(p);
      this.controls.target.sub(p).multiplyScalar(s).add(p);
    }
    this.controls.update();
  };

  private onViewKey = (ev: KeyboardEvent): void => {
    if (ev.altKey || ev.shiftKey) return;
    // Don't hijack keys while typing in a field.
    const t = ev.target as HTMLElement | null;
    if (t && (t.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName))) return;
    const mod = ev.ctrlKey || ev.metaKey;
    if (!mod && (ev.key === "f" || ev.key === "F")) {
      ev.preventDefault();
      this.fit();
      return;
    }
    const view = VIEW_KEYS[ev.key];
    if (!view || (!mod && ev.key === "0")) return;
    ev.preventDefault();
    this.setCameraView(view);
  };

  /** Bounding box of the VISIBLE parts (hidden solids excluded), so view framing ignores
   *  what isn't shown. Falls back to the whole model when everything is hidden. */
  private visibleBox(): THREE.Box3 | null {
    const s = this.content.solid;
    if (!s) return null;
    if (!this.hiddenSolids.size || !this.solidOfTri || !this.fullIndex) {
      return new THREE.Box3().expandByObject(s);
    }
    const pos = s.geometry.getAttribute("position") as THREE.BufferAttribute;
    const box = new THREE.Box3();
    const sot = this.solidOfTri, idx = this.fullIndex;
    for (let t = 0; t < sot.length; t++) {
      if (this.hiddenSolids.has(sot[t]!)) continue;
      for (let e = 0; e < 3; e++) {
        const v = idx[t * 3 + e]!;
        this._oTmp.set(pos.getX(v), pos.getY(v), pos.getZ(v));
        box.expandByPoint(this._oTmp);
      }
    }
    return box.isEmpty() ? new THREE.Box3().expandByObject(s) : box;
  }

  /** Snap to an axis (or isometric) view, framing the visible parts. */
  setCameraView(view: CameraView): void {
    const box = this.visibleBox();
    if (!box) return;
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z) * 0.5 || 1;

    // dir: offset from the part toward the camera. up: which way is up on
    // screen (top/bottom look along ±Z, where up = +Z would be degenerate).
    let dir: THREE.Vector3;
    const up = new THREE.Vector3(0, 0, 1);
    switch (view) {
      case "top": dir = new THREE.Vector3(0, 0, 1); up.set(0, 1, 0); break;
      case "bottom": dir = new THREE.Vector3(0, 0, -1); up.set(0, 1, 0); break;
      case "front": dir = new THREE.Vector3(0, -1, 0); break;
      case "behind": dir = new THREE.Vector3(0, 1, 0); break;
      case "left": dir = new THREE.Vector3(-1, 0, 0); break;
      case "right": dir = new THREE.Vector3(1, 0, 0); break;
      default: dir = ISO_DIR.clone().normalize(); break;
    }

    const dist = (radius / Math.sin((this.persp.fov * Math.PI) / 360)) * 1.4;
    const pos = center.clone().addScaledVector(dir, dist);

    this.persp.up.copy(up);
    this.persp.position.copy(pos);
    this.persp.near = dist / 100;
    this.persp.far = dist * 100;
    this.persp.updateProjectionMatrix();

    this.ortho.up.copy(up);
    this.ortho.position.copy(pos);
    this.ortho.near = 0.01;
    this.ortho.far = dist * 100;
    this.ortho.zoom = 1;
    this.setOrthoFrustum(radius * 2.4);
    this.ortho.updateProjectionMatrix();

    this.controls.target.copy(center);
    this.controls.update();
    // Re-pivot the next orbit drag on the part centre, not a stale surface hit.
    this.lastOrbitPivot = center.clone();
  }

  private animate = (): void => {
    requestAnimationFrame(this.animate);
    this.controls.update();
    this.measure.update(); // process queued hover + keep markers a constant apparent size
    this.renderer.clear(); // autoClear is off for the axes-triad second pass
    this.renderer.render(this.scene, this.camera);
    this.viewHelper.render(this.renderer);
    this.labelRenderer.render(this.scene, this.camera);
  };

  /** Push/remove the section clipping plane on every content material. */
  private refreshClipping(): void {
    const planes = this.section.enabled ? [this.section.plane] : null;
    const c = this.content;
    const mats = [c.solid, c.wire, c.edges, c.feature, c.highlight, c.reference]
      .map((o) => o?.material as THREE.Material | undefined);
    for (const m of mats) {
      if (!m) continue;
      const had = (m.clippingPlanes?.length ?? 0) > 0;
      if (had !== !!planes) {
        m.clippingPlanes = planes;
        m.needsUpdate = true;
      }
    }
  }

  private applyVisibility(): void {
    this.refreshClipping();
    // Solid caps only where an OPAQUE solid is being cut (transparent /
    // edges-only views look inside anyway — a filled cut face would lie).
    this.section.setCapsAllowed(this.solidVisible && !this.transparentOn);
    const c = this.content;
    if (c.wire) c.wire.visible = this.showWire;
    if (c.edges) c.edges.visible = this.showEdges;
    if (c.feature) c.feature.visible = this.showFeature;
    if (c.reference) c.reference.visible = this.showReference;
    if (c.solid) {
      c.solid.visible = this.solidVisible;
      const m = c.solid.material as THREE.MeshStandardMaterial;
      const active = this.deviationOn && this.devColors ? this.devColors
        : this.showColors ? this.faceColors : null;
      const geo = c.solid.geometry;
      const cur = geo.getAttribute("color") as THREE.BufferAttribute | undefined;
      if (!active) {
        if (cur) geo.deleteAttribute("color");
      } else if (!cur || cur.array !== active) {
        geo.setAttribute("color", new THREE.Float32BufferAttribute(active, 3));
      }
      m.vertexColors = !!active;
      m.color.set(m.vertexColors ? 0xffffff : BASE_COLOR);
      m.transparent = this.transparentOn;
      m.opacity = this.transparentOn ? 0.4 : 1;
      m.depthWrite = !this.transparentOn; // let edges/back faces show through when translucent
      m.needsUpdate = true;
    }
  }

  /** Replace the solid + wireframe + open-edge + surface-boundary geometry. `faceColors` is the
   * per-vertex STEP face-color attribute for this geometry (null = uncolored model); `solidOfTri`
   * gives each triangle's body id (same order as the geometry's index) for per-part hiding. */
  setMesh(geometry: THREE.BufferGeometry, boundary: EdgeSet, feature: EdgeSet, faceColors: Float32Array | null = null, solidOfTri: Uint32Array | null = null): void {
    const c = this.content;
    this.disposeMeshes(c);
    this.faceColors = faceColors;
    this.devColors = null; // stale deviation colors would mismatch the new vertex count
    this.lastOrbitPivot = null; // a pivot on the old model would be off-surface
    this.disposePickBvh(); // built from the old model's buffers
    this.measure.setData(null); // old measurements/labels would float over the new model
    this.solidOfTri = solidOfTri;
    this.drawnSolidOfTri = solidOfTri;
    this.fullIndex = (geometry.getIndex()?.array as Uint32Array) ?? null;
    this.boundarySet = boundary;
    this.featureSet = feature;
    this.hiddenSolids.clear();
    this.wireDirty = false;

    const mat = new THREE.MeshStandardMaterial({
      color: BASE_COLOR,
      metalness: 0.0,
      roughness: 0.85,
      side: THREE.DoubleSide,
      flatShading: true,
      // Push faces slightly back so coplanar feature lines win the depth test (crisp hidden-line look).
      polygonOffset: true,
      polygonOffsetFactor: 1,
      polygonOffsetUnits: 1,
    });
    c.solid = new THREE.Mesh(geometry, mat);
    c.group.add(c.solid);

    const wireMat = new THREE.LineBasicMaterial({ color: 0x0c0f12, transparent: true, opacity: 0.55 });
    c.wire = new THREE.LineSegments(new THREE.WireframeGeometry(geometry), wireMat);
    c.wire.visible = this.showWire;
    c.group.add(c.wire);

    // Surface boundaries (CAD face borders): depth-tested so hidden edges stay hidden.
    const featGeo = new THREE.BufferGeometry();
    featGeo.setAttribute("position", new THREE.Float32BufferAttribute(feature.positions, 3));
    const featMat = new THREE.LineBasicMaterial({ color: FEATURE_COLOR });
    c.feature = new THREE.LineSegments(featGeo, featMat);
    c.feature.visible = this.showFeature;
    c.group.add(c.feature);

    // Open (defect) edges: drawn over everything in red so leaks are never hidden.
    const edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute("position", new THREE.Float32BufferAttribute(boundary.positions, 3));
    const edgeMat = new THREE.LineBasicMaterial({ color: EDGE_COLOR, depthTest: false });
    c.edges = new THREE.LineSegments(edgeGeo, edgeMat);
    c.edges.renderOrder = 10;
    c.edges.visible = this.showEdges;
    c.group.add(c.edges);

    // Part-highlight overlay: shares the solid's position buffer, starts with an empty index;
    // setHighlightSolids swaps the index in and out (cheap — no vertex re-upload per hover).
    const hlGeo = new THREE.BufferGeometry();
    hlGeo.setAttribute("position", geometry.getAttribute("position"));
    hlGeo.setIndex(new THREE.BufferAttribute(new Uint32Array(0), 1));
    hlGeo.boundingSphere = geometry.boundingSphere;
    const hlMat = new THREE.MeshBasicMaterial({
      color: HIGHLIGHT_COLOR,
      transparent: true,
      opacity: 0.45,
      side: THREE.DoubleSide,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
    });
    c.highlight = new THREE.Mesh(hlGeo, hlMat);
    c.highlight.renderOrder = 5; // over the solid + feature lines, under the red defect edges
    c.highlight.visible = false;
    c.group.add(c.highlight);

    // Section caps track the display geometry; body ids give each part's cut
    // face its own color. Re-center the plane through the new model.
    this.section.setGeometry(geometry, this.fullIndex, solidOfTri);
    this.section.refit();

    this.applyVisibility();
  }

  /** Set (or clear) the reference overlay. */
  setReference(geometry: THREE.BufferGeometry | null): void {
    const c = this.content;
    if (c.reference) {
      c.group.remove(c.reference);
      (c.reference.material as THREE.Material).dispose();
      c.reference = null;
    }
    if (geometry) {
      const mat = new THREE.MeshStandardMaterial({
        color: REF_COLOR,
        transparent: true,
        opacity: 0.28,
        metalness: 0.0,
        roughness: 1.0,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      c.reference = new THREE.Mesh(geometry, mat);
      c.reference.visible = this.showReference;
      c.group.add(c.reference);
    }
    this.refreshClipping(); // the fresh reference material needs the section plane too
  }

  /** Apply (or clear) per-vertex deviation colors on the solid mesh. */
  setDeviationColors(colors: Float32Array | null): void {
    this.devColors = colors;
    this.applyVisibility();
  }

  // ---------- per-part visibility / highlight ----------

  /** Hide the given solid (body) ids: triangles, face borders and open-edge lines of those
   * parts are dropped from the drawn subsets; the source buffers stay intact. */
  setHiddenSolids(hidden: Iterable<number>): void {
    const c = this.content;
    if (!c.solid || !this.solidOfTri || !this.fullIndex) return;
    this.hiddenSolids = new Set(hidden);
    const geo = c.solid.geometry;
    const index = this.hiddenSolids.size
      ? filterTriangles(this.fullIndex, this.solidOfTri, this.hiddenSolids)
      : this.fullIndex;
    geo.setIndex(new THREE.BufferAttribute(index, 1));
    // Keep the drawn-triangle -> solid map aligned with the filtered index (picking).
    if (this.hiddenSolids.size) {
      const sot = this.solidOfTri;
      const kept = new Uint32Array(sot.length);
      let n = 0;
      for (let t = 0; t < sot.length; t++) if (!this.hiddenSolids.has(sot[t]!)) kept[n++] = sot[t]!;
      this.drawnSolidOfTri = kept.subarray(0, n);
    } else {
      this.drawnSolidOfTri = this.solidOfTri;
    }
    if (c.feature && this.featureSet) {
      const p = this.hiddenSolids.size ? filterSegments(this.featureSet, this.hiddenSolids) : this.featureSet.positions;
      c.feature.geometry.setAttribute("position", new THREE.Float32BufferAttribute(p, 3));
    }
    if (c.edges && this.boundarySet) {
      const p = this.hiddenSolids.size ? filterSegments(this.boundarySet, this.hiddenSolids) : this.boundarySet.positions;
      c.edges.geometry.setAttribute("position", new THREE.Float32BufferAttribute(p, 3));
    }
    // The wireframe is derived from the (now changed) index — rebuilding it on a 2M-tri model
    // is the expensive part, so defer until it is actually shown.
    this.wireDirty = true;
    if (this.showWire) this.rebuildWire();
    this.section.setHiddenSolids(this.hiddenSolids); // caps drop the hidden bodies too
  }

  /** Highlight the given solid ids (hover feedback), or clear with null. Hidden parts highlight
   * too — a translucent ghost that shows where a hidden part sits. The overlay mesh persists
   * per model (it shares the solid's position buffer); only its small index is swapped here. */
  setHighlightSolids(ids: ReadonlySet<number> | null): void {
    const c = this.content;
    if (!c.highlight || !this.solidOfTri || !this.fullIndex) return;
    if (!ids || ids.size === 0) {
      c.highlight.visible = false;
      return;
    }
    const sot = this.solidOfTri, full = this.fullIndex;
    const out = new Uint32Array(full.length);
    let n = 0;
    for (let t = 0; t < sot.length; t++) {
      if (!ids.has(sot[t]!)) continue;
      out[n] = full[t * 3]!; out[n + 1] = full[t * 3 + 1]!; out[n + 2] = full[t * 3 + 2]!;
      n += 3;
    }
    c.highlight.geometry.setIndex(new THREE.BufferAttribute(out.slice(0, n), 1));
    c.highlight.visible = n > 0;
  }

  private rebuildWire(): void {
    const c = this.content;
    if (!c.wire || !c.solid) return;
    c.wire.geometry.dispose();
    c.wire.geometry = new THREE.WireframeGeometry(c.solid.geometry);
    this.wireDirty = false;
  }

  setWireframe(v: boolean): void {
    this.showWire = v;
    if (v && this.wireDirty) this.rebuildWire();
    this.applyVisibility();
  }
  /** Show/hide the model's own STEP face colors. */
  setShowColors(v: boolean): void { this.showColors = v; this.applyVisibility(); }
  setOpenEdges(v: boolean): void { this.showEdges = v; this.applyVisibility(); }
  setFeatureEdges(v: boolean): void { this.showFeature = v; this.applyVisibility(); }
  setTransparent(v: boolean): void { this.transparentOn = v; this.applyVisibility(); }
  /** false => hide the shaded surfaces (CAD-edges-only line view). */
  setSurfacesVisible(v: boolean): void { this.solidVisible = v; this.applyVisibility(); }

  // ---------- section view ----------

  /** Toggle the solid section view (clipping plane + filled cut face + gizmo). */
  setSection(v: boolean): void { this.section.setEnabled(v); this.applyVisibility(); }
  /** Snap the section plane perpendicular to a world axis, cut opening toward the camera. */
  setSectionAxis(axis: "x" | "y" | "z"): void { this.section.setAxis(axis); }
  /** Swap which half of the model is kept. */
  flipSection(): void { this.section.flip(); }

  // ---------- measurement ----------

  /** Toggle measure mode (snap-assisted distance / edge dimensions). */
  setMeasure(v: boolean): void { this.measure.setEnabled(v); }
  setMeasureMode(m: MeasureMode): void { this.measure.setMode(m); }
  /** Swap the model's measurement payload (null = no B-rep / clear everything). */
  setMeasureData(d: MeasureGeometry | null): void { this.measure.setData(d); }
  clearMeasurements(): void { this.measure.clearAll(); }

  /** Match the 3D background to the UI theme. */
  setTheme(mode: "light" | "dark"): void {
    this.scene.background = new THREE.Color(mode === "light" ? 0xe9edf2 : 0x14181d);
    this.measure.setTheme(mode);
  }
  setReferenceVisible(v: boolean): void { this.showReference = v; this.applyVisibility(); }
  setDeviation(v: boolean): void { this.deviationOn = v; this.applyVisibility(); }

  /** Frame both cameras to the visible content (hidden parts don't count). */
  fit(): void {
    const box = this.visibleBox();
    if (!box) return;
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z) * 0.5 || 1;
    const dist = (radius / Math.sin((this.persp.fov * Math.PI) / 360)) * 1.4;
    const dir = ISO_DIR.clone().normalize();
    const pos = center.clone().addScaledVector(dir, dist);

    // Fit re-frames from the iso direction — also re-level a free-orbit roll.
    this.persp.up.set(0, 0, 1);
    this.ortho.up.set(0, 0, 1);
    this.persp.position.copy(pos);
    this.persp.near = dist / 100;
    this.persp.far = dist * 100;
    this.persp.updateProjectionMatrix();

    this.ortho.position.copy(pos);
    this.ortho.near = 0.01;
    this.ortho.far = dist * 100;
    this.ortho.zoom = 1;
    this.setOrthoFrustum(radius * 2.4); // enclose the model with a small margin
    this.ortho.updateProjectionMatrix();

    this.controls.target.copy(center);
    this.controls.update();
    // Re-pivot the next orbit drag on the part centre, not a stale surface hit.
    this.lastOrbitPivot = center.clone();
  }

  /** Frame the given solid (body) ids, keeping the current view direction. A part that occurs
   *  several times in the assembly shares one solid id — the frame encloses all instances. */
  fitSolids(ids: ReadonlySet<number>): void {
    const c = this.content;
    if (!c.solid || !this.solidOfTri || !this.fullIndex || ids.size === 0) return;
    const pos = c.solid.geometry.getAttribute("position") as THREE.BufferAttribute;
    const box = new THREE.Box3();
    const sot = this.solidOfTri, idx = this.fullIndex;
    for (let t = 0; t < sot.length; t++) {
      if (!ids.has(sot[t]!)) continue;
      for (let e = 0; e < 3; e++) {
        const v = idx[t * 3 + e]!;
        this._oTmp.set(pos.getX(v), pos.getY(v), pos.getZ(v));
        box.expandByPoint(this._oTmp);
      }
    }
    if (box.isEmpty()) return;
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z) * 0.5 || 1;
    const dir = this.camera.position.clone().sub(this.controls.target);
    if (dir.lengthSq() < 1e-12) dir.copy(ISO_DIR);
    dir.normalize();
    const dist = (radius / Math.sin((this.persp.fov * Math.PI) / 360)) * 1.4;
    const p = center.clone().addScaledVector(dir, dist);

    this.persp.position.copy(p);
    this.persp.near = dist / 100;
    this.persp.far = dist * 100;
    this.persp.updateProjectionMatrix();

    this.ortho.position.copy(p);
    this.ortho.near = 0.01;
    this.ortho.far = dist * 100;
    this.ortho.zoom = 1;
    this.setOrthoFrustum(radius * 2.4);
    this.ortho.updateProjectionMatrix();

    this.controls.target.copy(center);
    this.controls.update();
    this.lastOrbitPivot = center.clone();
  }

  private disposePickBvh(): void {
    // The shadow geometry shares the solid's position/index buffers — dispose() would destroy
    // the GPU buffers of the visible mesh, so only drop the references.
    this.pickBvh = null;
    this.pickGeo = null;
  }

  private disposeMeshes(c: Content): void {
    if (c.solid) {
      c.group.remove(c.solid);
      c.solid.geometry.dispose();
      (c.solid.material as THREE.Material).dispose();
      c.solid = null;
    }
    if (c.wire) {
      c.group.remove(c.wire);
      c.wire.geometry.dispose();
      (c.wire.material as THREE.Material).dispose();
      c.wire = null;
    }
    if (c.edges) {
      c.group.remove(c.edges);
      c.edges.geometry.dispose();
      (c.edges.material as THREE.Material).dispose();
      c.edges = null;
    }
    if (c.feature) {
      c.group.remove(c.feature);
      c.feature.geometry.dispose();
      (c.feature.material as THREE.Material).dispose();
      c.feature = null;
    }
    if (c.highlight) {
      c.group.remove(c.highlight);
      c.highlight.geometry.dispose();
      (c.highlight.material as THREE.Material).dispose();
      c.highlight = null;
    }
  }
}
