// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ViewHelper } from "three/addons/helpers/ViewHelper.js";
import { CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";
import { MeshBVH } from "three-mesh-bvh";
import { filterTriangles, filterSegments, wireframeIndex, type EdgeSet } from "./mesh-utils.ts";
import { SectionController } from "./section.ts";
import { MeasureController, type MeasureMode } from "./measure.ts";
import { SCHEMES, MMB, RMB, type ControlScheme, type NavAction } from "./nav-schemes.ts";
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

export type CameraView = "top" | "bottom" | "front" | "behind" | "left" | "right" | "iso";

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
  private keyLight!: THREE.DirectionalLight;
  private fillLight!: THREE.DirectionalLight;
  private controls: OrbitControls;
  private content: Content;
  private container: HTMLElement;

  // Scheme-driven navigation: the active ControlScheme maps pointer chords
  // (buttons bitmask + modifiers) to orbit / pan / drag-zoom. Orbit spins
  // around the surface point under the cursor (free over the poles), the
  // wheel zooms toward the cursor, and pan translates in screen space — all
  // handled manually below; OrbitControls keeps only the target + lookAt.
  private scheme: ControlScheme = SCHEMES[0]!;
  private navAction: NavAction | null = null; // gesture in progress
  private navMask = 0; // pressed-button chord (buttons bitmask) of the active gesture
  private navLast: { x: number; y: number } | null = null; // pan/zoom drag anchor
  private zoomAnchor: { x: number; y: number } | null = null; // drag-zoom zooms toward this screen point
  private chordDown: { t: number; x: number; y: number } | null = null; // CATIA tick-zoom detection
  private catiaZoomLatch = false; // middle-drag means zoom (not pan) until the chord ends
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
  private smoothOn = true; // smooth (baked crease-aware normals) vs flat facet shading

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

  // Content bounding sphere for the per-frame near/far update (updateClipPlanes);
  // recomputed lazily after the model or reference overlay changes.
  private clipSphere = new THREE.Sphere();
  private clipSphereDirty = true;

  // Section view: one clipping plane + stencil caps + plane gizmo (section.ts).
  private section: SectionController;
  // Axes triad in the lower-right corner showing the world orientation.
  private viewHelper: ViewHelper;

  // Exploded view: per-instance world offsets applied on the CPU into the (shared) position
  // buffer — the wireframe, highlight overlay and pick geometry share that buffer, so they move
  // for free; the feature/open-edge line sets carry per-segment instance ids and are rebuilt.
  private explodeData: { instanceOfTri: Uint32Array; offsetsAt: (f: number) => Float64Array } | null = null;
  private explodeFactor = 0; // currently applied factor
  private explodePending: number | null = null; // coalesced slider input; applied once per frame
  private explodeTarget: number | null = null; // eased-animation target (mode enter/exit)
  private explodeBase: Float32Array | null = null; // pristine solid positions (exact restore at 0)
  private instanceOfVertex: Uint32Array | null = null; // display vertex -> instance (lazy)
  private explodedOffsets: Float64Array | null = null; // current offsets (null = collapsed)
  private explodeBlendFrom: Float64Array | null = null; // style-switch: offsets to blend away from
  private explodeBlendT = 1; // 1 = no blend in progress
  private pickBvhDirty = false; // positions moved since the BVH was built — refit before use

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

    // Lights: soft world-fixed ambient dome + a camera-following key/fill pair
    // ("headlight" rig, like CAD viewers) so the part is lit from wherever you look —
    // no permanently dark underside. Directions are re-derived from the camera each frame.
    const hemi = new THREE.HemisphereLight(0xffffff, 0x606068, 0.9);
    hemi.position.set(0, 0, 1);
    this.scene.add(hemi);
    this.keyLight = new THREE.DirectionalLight(0xffffff, 1.6);
    this.scene.add(this.keyLight);
    this.fillLight = new THREE.DirectionalLight(0xffffff, 0.7);
    this.scene.add(this.fillLight);
    this.updateLights();

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
    // Rotation, pan and zoom are all manual (scheme-mapped buttons, pivot-on-
    // cursor orbit, cursor-centric zoom — see installNavigation). OrbitControls
    // only owns the target and the per-frame lookAt.
    c.enableRotate = false;
    c.enableZoom = false;
    c.enablePan = false;
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
    // Drop the grey negative-axis dots — only the three labelled axes carry information here.
    h.traverse((o) => {
      if (typeof o.userData.type === "string" && o.userData.type.startsWith("neg")) o.visible = false;
    });
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

  // ---------- navigation (orbit / pan / zoom) ----------
  // Orbit rotates around the surface point under the cursor with no polar
  // clamping, the wheel zooms toward the cursor, and pan translates in screen
  // space. WHICH buttons do what comes from the active ControlScheme
  // (nav-schemes.ts): the chord (buttons bitmask + modifiers) is re-resolved
  // at every press/release, so chorded schemes (CATIA/FreeCAD middle-then-
  // second-button) and mid-gesture switches work without special cases.

  /** Switch the pointer-button control scheme (dropdown in the bottom bar). */
  setNavScheme(scheme: ControlScheme): void {
    this.scheme = scheme;
    // Cancel any in-flight gesture so stale state can't leak across schemes.
    this.endOrbitDrag();
    this.navAction = null;
    this.navMask = 0;
    this.navLast = null;
    this.zoomAnchor = null;
    this.chordDown = null;
    this.catiaZoomLatch = false;
  }

  private installNavigation(canvas: HTMLCanvasElement): void {
    canvas.addEventListener("pointerdown", (ev) => this.syncNavButtons(ev));
    // Middle-click autoscroll (Windows) would swallow every middle-drag scheme.
    canvas.addEventListener("mousedown", (ev) => {
      if (ev.button === 1) ev.preventDefault();
    });
    // The context menu only opens on a clean right CLICK — a right-DRAG is
    // navigation in most schemes (pan here, orbit in Onshape/Tinkercad/Rhino).
    canvas.addEventListener("contextmenu", (ev) => {
      ev.preventDefault();
      const s = this.rmbStart;
      this.rmbStart = null;
      if (ev.buttons & 7) return; // right released mid-chord (CATIA/NX gestures)
      if (s && Math.hypot(ev.clientX - s.x, ev.clientY - s.y) > 4) return;
      this.onContextMenu?.(this.pickSolid(ev.clientX, ev.clientY), ev.clientX, ev.clientY);
    });
    // Move + release on document so a drag that leaves the canvas still tracks.
    document.addEventListener("pointermove", this.onNavMove);
    document.addEventListener("pointerup", (ev) => {
      if (this.navMask) this.syncNavButtons(ev);
    });
    document.addEventListener("keydown", this.onViewKey);
    canvas.addEventListener("wheel", this.onWheel, { passive: false });
  }

  /** Track the pressed-button chord and re-resolve the gesture when it changes.
   *  Spec quirk this exists for: pressing/releasing a SECOND button while one
   *  is already down arrives as pointermove — NOT pointerdown/pointerup — so
   *  every pointer event funnels through here (via onNavMove for the chorded
   *  case) before being routed. */
  private syncNavButtons(ev: PointerEvent): void {
    const mask = ev.buttons & 7;
    if (mask === this.navMask) return;
    const prev = this.navMask;
    this.navMask = mask;
    // rmbStart must catch chorded right presses too (they skip pointerdown).
    if (mask & RMB && !(prev & RMB)) this.rmbStart = { x: ev.clientX, y: ev.clientY };
    // CATIA tick-zoom: a second button quickly CLICKED (not held) while middle
    // stays down flips the rest of the middle-drag from pan to zoom.
    if (this.scheme.catiaZoomTick && prev & MMB && mask & MMB) {
      if (mask & ~MMB & ~prev) {
        this.chordDown = { t: performance.now(), x: ev.clientX, y: ev.clientY };
      } else if (prev & ~MMB & ~mask && this.chordDown) {
        const d = this.chordDown;
        if (performance.now() - d.t < 300 && Math.hypot(ev.clientX - d.x, ev.clientY - d.y) < 5) {
          this.catiaZoomLatch = true;
        }
        this.chordDown = null;
      }
    } else {
      this.chordDown = null;
    }
    this.updateNavAction(ev);
  }

  /** Re-resolve the drag action from the current button chord + modifiers.
   *  Called at every button press/release so chords switch mid-gesture. */
  private updateNavAction(ev: PointerEvent): void {
    const mask = ev.buttons & 7;
    if (!(mask & MMB)) this.catiaZoomLatch = false; // chord over
    let action: NavAction | null = null;
    if (this.catiaZoomLatch && mask === MMB) {
      action = "zoom";
    } else if (mask) {
      const b = this.scheme.bindings.find(
        (b) =>
          b.buttons === mask && !!b.shift === ev.shiftKey && !!b.ctrl === ev.ctrlKey && !!b.alt === ev.altKey
      );
      action = b?.action ?? null;
    }
    if (action === this.navAction) return;
    this.endOrbitDrag();
    this.navAction = null;
    this.navLast = null;
    this.zoomAnchor = null;
    if (!action || this.section.busy()) return; // gizmo drags must not also move the camera
    this.navAction = action;
    if (action === "orbit") {
      this.beginOrbit(ev);
    } else {
      this.navLast = { x: ev.clientX, y: ev.clientY };
      if (action === "zoom") this.zoomAnchor = { x: ev.clientX, y: ev.clientY };
    }
  }

  private onNavMove = (ev: PointerEvent): void => {
    // Chorded button changes (second button pressed/released mid-drag) arrive
    // as pointermove — catch them by the mask changing under an active gesture.
    if (this.navMask && (ev.buttons & 7) !== this.navMask) {
      this.syncNavButtons(ev);
      return;
    }
    if (this.navAction === "orbit") this.onOrbitMove(ev);
    else if (this.navAction === "pan") this.onPanMove(ev);
    else if (this.navAction === "zoom") this.onZoomMove(ev);
  };

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
    if (this.pickBvh) {
      // Exploded-view offsets moved the (shared) positions under the tree — refit the bounds
      // on demand (cheaper than a rebuild; bounds get looser but stay correct).
      if (this.pickBvhDirty) {
        this.pickBvh.refit();
        this.pickBvhDirty = false;
      }
      return this.pickBvh;
    }
    const c = this.content;
    if (!c.solid || !this.fullIndex) return null;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", c.solid.geometry.getAttribute("position"));
    geo.setIndex(new THREE.BufferAttribute(this.fullIndex, 1));
    this.pickGeo = geo;
    this.pickBvh = new MeshBVH(geo, { indirect: true });
    this.pickBvhDirty = false;
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

  /** All body ids whose surface the ray through (clientX, clientY) crosses, nearest first,
   *  deduped. Cast against the FULL mesh, so bodies occluded by others — and hidden ones —
   *  are found too (the context menu uses this to retarget an action onto an interior part
   *  without hiding the externals first). Section view: clipped-away hits don't count. */
  pickSolidsThrough(clientX: number, clientY: number): number[] {
    const bvh = this.ensurePickBvh();
    if (!bvh || !this.solidOfTri) return [];
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointerNdc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
    this.pointerNdc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointerNdc, this.camera);
    const hits = bvh.raycast(this.raycaster.ray, THREE.DoubleSide);
    hits.sort((a, b) => a.distance - b.distance);
    const seen = new Set<number>();
    const out: number[] = [];
    for (const h of hits) {
      if (h.faceIndex == null) continue;
      if (this.section.enabled && this.section.plane.distanceToPoint(h.point) < -1e-6) continue;
      const id = this.solidOfTri[h.faceIndex]!;
      if (!seen.has(id)) {
        seen.add(id);
        out.push(id);
      }
    }
    return out;
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

  private endOrbitDrag(): void {
    if (!this.orbitPivot) return;
    this.orbitPivot = null;
    this.orbitStart = null;
    this.orbitLast = null;
    this.orbiting = false; // the free orbit keeps its orientation — no re-leveling
    this.pivotMarker.visible = false;
  }

  /** Screen-space pan: translate camera + target along the view plane so the
   *  model follows the cursor 1:1 (world units per pixel at the target depth). */
  private onPanMove(ev: PointerEvent): void {
    if (!this.navLast) return;
    const dx = ev.clientX - this.navLast.x;
    const dy = ev.clientY - this.navLast.y;
    this.navLast = { x: ev.clientX, y: ev.clientY };
    if (dx === 0 && dy === 0) return;
    const h = Math.max(1, this.renderer.domElement.clientHeight);
    const worldPerPx = this.orthoOn
      ? (this.ortho.top - this.ortho.bottom) / this.ortho.zoom / h
      : (2 * this.persp.position.distanceTo(this.controls.target) * Math.tan((this.persp.fov * Math.PI) / 360)) / h;
    this.camera.updateMatrixWorld();
    this._oRight.setFromMatrixColumn(this.camera.matrixWorld, 0).normalize();
    this._oUp.setFromMatrixColumn(this.camera.matrixWorld, 1).normalize();
    this._oTmp.copy(this._oRight).multiplyScalar(-dx * worldPerPx).addScaledVector(this._oUp, dy * worldPerPx);
    this.camera.position.add(this._oTmp);
    this.controls.target.add(this._oTmp);
    this.controls.update();
  }

  /** Drag-zoom (Shift+MMB in SolidWorks, CATIA chord, ...): drag up = zoom in,
   *  zooming about the point where the gesture started. */
  private onZoomMove(ev: PointerEvent): void {
    if (!this.navLast || !this.zoomAnchor) return;
    const dy = ev.clientY - this.navLast.y;
    this.navLast = { x: ev.clientX, y: ev.clientY };
    if (dy === 0) return;
    this.zoomAt(Math.exp(-dy * 0.005), this.zoomAnchor.x, this.zoomAnchor.y);
  }

  private onWheel = (ev: WheelEvent): void => {
    ev.preventDefault();
    // SolidWorks/Autodesk/NX muscle memory: their schemes zoom OUT on scroll up.
    const zoomIn = this.scheme.wheelZoomsOut ? ev.deltaY > 0 : ev.deltaY < 0;
    this.zoomAt(zoomIn ? 1.1 : 1 / 1.1, ev.clientX, ev.clientY);
  };

  /** Cursor-centric zoom: keep the world point under (clientX, clientY) pinned. */
  private zoomAt(factor: number, clientX: number, clientY: number): void {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndcX = ((clientX - rect.left) / rect.width) * 2 - 1;
    const ndcY = -((clientY - rect.top) / rect.height) * 2 + 1;
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
      // Don't dolly below the dynamic near-plane floor (r/1000, see
      // updateClipPlanes). Checking against cam.near itself would deadlock:
      // far from the part, near tracks the camera distance (0.8x), so every
      // zoom-in would be swallowed.
      const rClip = Math.max(this.clipSphere.radius, 1e-3);
      if (factor > 1 && cam.position.distanceTo(p) * s < rClip / 500) return;
      cam.position.sub(p).multiplyScalar(s).add(p);
      this.controls.target.sub(p).multiplyScalar(s).add(p);
    }
    this.controls.update();
  }

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
    this.persp.updateProjectionMatrix();

    this.ortho.up.copy(up);
    this.ortho.position.copy(pos);
    this.ortho.zoom = 1;
    this.setOrthoFrustum(radius * 2.4);
    this.ortho.updateProjectionMatrix();

    this.controls.target.copy(center);
    this.controls.update();
    // Re-pivot the next orbit drag on the part centre, not a stale surface hit.
    this.lastOrbitPivot = center.clone();
  }

  /** Re-aim the key/fill lights from the current camera orientation (headlight rig).
   *  Offsets are in camera space (+X right, +Y up, +Z toward the viewer); only the
   *  direction matters for DirectionalLight, so magnitudes are arbitrary. */
  private updateLights(): void {
    this.keyLight.position.set(0.5, 0.7, 1.5).applyQuaternion(this.camera.quaternion);
    this.fillLight.position.set(-1, -0.6, 0.4).applyQuaternion(this.camera.quaternion);
  }

  /** Re-derive near/far from the camera's distance to the model every frame.
   *  The pivot-on-cursor orbit and cursor-centric zoom move the camera freely,
   *  so planes frozen at fit() time end up slicing the part once the camera
   *  gets close (near-plane clipping while orbiting). */
  private updateClipPlanes(): void {
    if (this.clipSphereDirty) {
      const box = new THREE.Box3().expandByObject(this.content.group);
      if (box.isEmpty()) return; // pre-model: keep the constructor defaults
      box.getBoundingSphere(this.clipSphere);
      this.clipSphereDirty = false;
    }
    const r = Math.max(this.clipSphere.radius, 1e-3);
    const dist = this.camera.position.distanceTo(this.clipSphere.center);
    // 4r of headroom past the sphere keeps helpers (section quad, pivot marker)
    // inside the frustum; far-plane slack costs no depth precision — near does.
    if (this.orthoOn) {
      // Ortho projections allow near < 0: bracket the whole model regardless
      // of where pan/zoom left the camera (it may sit inside the part).
      this.ortho.near = dist - 4 * r;
      this.ortho.far = dist + 4 * r;
      this.ortho.updateProjectionMatrix();
    } else {
      // The camera can't be closer to geometry than dist - r, so 0.8x that is a
      // safe near plane; inside the sphere fall back to r/1000 (~0.1mm on a
      // 100mm part) so close-up detail still renders without z-fighting.
      this.persp.near = Math.max((dist - r) * 0.8, r / 1000);
      this.persp.far = dist + 4 * r;
      this.persp.updateProjectionMatrix();
    }
  }

  private animate = (): void => {
    requestAnimationFrame(this.animate);
    // Exploded view: cross-blend after a style switch, ease toward an animation target, then
    // apply at most one factor per frame (slider input events coalesce into explodePending).
    if (this.explodeBlendT < 1) {
      this.explodeBlendT = Math.min(1, this.explodeBlendT + 0.1);
      this.applyExplode(this.explodePending ?? this.explodeFactor, true);
      this.explodePending = null;
      if (this.explodeBlendT >= 1) this.explodeBlendFrom = null;
    }
    if (this.explodeTarget !== null) {
      const d = this.explodeTarget - this.explodeFactor;
      if (Math.abs(d) < 0.004) {
        this.explodePending = this.explodeTarget;
        this.explodeTarget = null;
      } else {
        this.explodePending = this.explodeFactor + d * 0.18;
      }
    }
    if (this.explodePending !== null) {
      const f = this.explodePending;
      this.explodePending = null;
      this.applyExplode(f);
    }
    this.controls.update();
    this.updateClipPlanes();
    this.updateLights();
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
      const m = c.solid.material as THREE.MeshPhongMaterial;
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
      m.flatShading = !this.smoothOn;
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
    this.clipSphereDirty = true;
    this.fullIndex = (geometry.getIndex()?.array as Uint32Array) ?? null;
    this.boundarySet = boundary;
    this.featureSet = feature;
    this.hiddenSolids.clear();
    // Explode state is per-model: the base-position copy and vertex->instance map reference the
    // old buffers. setExplode (called after setMesh) provides the new model's data.
    this.explodeData = null;
    this.explodeBase = null;
    this.instanceOfVertex = null;
    this.explodedOffsets = null;
    this.explodeFactor = 0;
    this.explodePending = null;
    this.explodeTarget = null;

    // Blinn-Phong, not PBR: cheaper per fragment AND gives the glossy CAD-style
    // specular highlight (Onshape look) that makes curvature readable.
    const mat = new THREE.MeshPhongMaterial({
      color: BASE_COLOR,
      specular: 0x777777,
      shininess: 40,
      side: THREE.DoubleSide,
      flatShading: !this.smoothOn,
      // Push faces slightly back so coplanar feature lines win the depth test (crisp hidden-line look).
      polygonOffset: true,
      polygonOffsetFactor: 1,
      polygonOffsetUnits: 1,
    });
    c.solid = new THREE.Mesh(geometry, mat);
    c.group.add(c.solid);

    // Wireframe as an index buffer over the mesh's own positions — THREE.WireframeGeometry
    // dies on V8's 2^24 Set cap past ~8M unique edges and duplicates every vertex. Built
    // lazily: it starts hidden, and a ~25M-edge index is real time and memory.
    const wireMat = new THREE.LineBasicMaterial({ color: 0x0c0f12, transparent: true, opacity: 0.55 });
    const wireGeo = new THREE.BufferGeometry();
    wireGeo.setAttribute("position", geometry.getAttribute("position"));
    wireGeo.setIndex(new THREE.BufferAttribute(new Uint32Array(0), 1));
    wireGeo.boundingSphere = geometry.boundingSphere;
    c.wire = new THREE.LineSegments(wireGeo, wireMat);
    c.wire.visible = this.showWire;
    c.group.add(c.wire);
    this.wireDirty = true;
    if (this.showWire) this.rebuildWire(); // the toggle persists across loads

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
    this.clipSphereDirty = true; // the overlay may extend past the model's bounds
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
    this.refreshEdgeOverlays();
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
    // Swap only the index (positions are shared with the solid mesh — disposing the geometry
    // would tear down the shared GPU buffer). Derives from the solid's CURRENT index, so
    // per-part hiding is honored.
    const idx = c.solid.geometry.getIndex()!.array as Uint32Array;
    c.wire.geometry.setIndex(new THREE.BufferAttribute(wireframeIndex(idx), 1));
    this.wireDirty = false;
  }

  /** Replace the surface-boundary line set without touching the rest of the model — used when
   * the STL crease-angle threshold changes. Honors the current per-part hiding. */
  setFeatureEdgeSet(feature: EdgeSet): void {
    this.featureSet = feature;
    this.refreshEdgeOverlays();
  }

  // ---------- exploded view ----------

  /** Positions of a line EdgeSet with the current explode offsets and hidden-part filter applied. */
  private displayedSegPositions(set: EdgeSet): Float32Array {
    let s = set;
    const off = this.explodedOffsets;
    if (off && set.instanceOfSeg) {
      const p = set.positions.slice();
      const inst = set.instanceOfSeg;
      for (let i = 0; i < set.count; i++) {
        const o = inst[i]! * 3, b = i * 6;
        p[b] += off[o]!; p[b + 1] += off[o + 1]!; p[b + 2] += off[o + 2]!;
        p[b + 3] += off[o]!; p[b + 4] += off[o + 1]!; p[b + 5] += off[o + 2]!;
      }
      s = { ...set, positions: p };
    }
    return this.hiddenSolids.size ? filterSegments(s, this.hiddenSolids) : s.positions;
  }

  /** Re-push the feature / open-edge line positions (hidden filter + explode offsets). */
  private refreshEdgeOverlays(): void {
    const c = this.content;
    if (c.feature && this.featureSet) {
      c.feature.geometry.setAttribute("position", new THREE.Float32BufferAttribute(this.displayedSegPositions(this.featureSet), 3));
      c.feature.geometry.boundingSphere = null; // stale bounds would frustum-cull moved lines
    }
    if (c.edges && this.boundarySet) {
      c.edges.geometry.setAttribute("position", new THREE.Float32BufferAttribute(this.displayedSegPositions(this.boundarySet), 3));
      c.edges.geometry.boundingSphere = null;
    }
  }

  /** Provide the model's explode data (per-triangle instance ids + offsets provider), or null to
   * disable. Call after setMesh; heavy derived state (base-position copy, vertex->instance map)
   * is built lazily on the first nonzero factor, so non-exploding sessions pay nothing. */
  setExplode(data: { instanceOfTri: Uint32Array; offsetsAt: (f: number) => Float64Array } | null): void {
    this.explodeData = data;
    this.explodeBase = null;
    this.instanceOfVertex = null;
    this.explodedOffsets = null;
    this.explodeFactor = 0;
    this.explodePending = null;
    this.explodeTarget = null;
    this.explodeBlendFrom = null;
    this.explodeBlendT = 1;
  }

  /** The offsets provider changed meaning (explode STYLE switch) while possibly exploded:
   * cross-blend from the currently applied offsets to the new style's over a few frames. */
  restyleExplode(): void {
    if (!this.explodeData || !this.explodedOffsets) return; // collapsed — next explode just uses the new style
    this.explodeBlendFrom = this.explodedOffsets;
    this.explodeBlendT = 0;
  }

  /** Set the explode factor (0 = assembled .. 1 = fully exploded). `animate` eases there over a
   * few frames — used when entering/leaving the explode mode; slider drags pass false and apply
   * synchronously, so a caller reading positions right after (e.g. deviation) sees the result. */
  setExplodeFactor(f: number, animate = false): void {
    if (!this.explodeData) return;
    const v = Math.min(1, Math.max(0, f));
    if (animate) {
      this.explodeTarget = v;
    } else {
      this.explodeTarget = null;
      this.explodePending = null;
      this.applyExplode(v);
    }
  }

  /** Apply per-instance offsets into the shared position buffer (and the line overlays). The
   * wireframe, highlight overlay and pick geometry share that buffer, so they follow for free;
   * the pick BVH is refit lazily on next use. Factor 0 restores the pristine copy exactly. */
  private applyExplode(f: number, force = false): void {
    const c = this.content;
    const data = this.explodeData;
    if (!c.solid || !data || !this.fullIndex) return;
    if (!force && f === this.explodeFactor && (f > 0) === !!this.explodedOffsets) return;
    const attr = c.solid.geometry.getAttribute("position") as THREE.BufferAttribute;
    const arr = attr.array as Float32Array;
    if (!this.explodeBase) this.explodeBase = arr.slice();
    if (!this.instanceOfVertex) {
      // Instances are separate welded components, so no display vertex straddles two instances —
      // a per-corner sweep over the full index assigns every vertex unambiguously.
      const inst = new Uint32Array(arr.length / 3);
      const it = data.instanceOfTri, idx = this.fullIndex;
      for (let t = 0; t < it.length; t++) {
        inst[idx[t * 3]!] = it[t]!;
        inst[idx[t * 3 + 1]!] = it[t]!;
        inst[idx[t * 3 + 2]!] = it[t]!;
      }
      this.instanceOfVertex = inst;
    }
    this.explodeFactor = f;
    const base = this.explodeBase;
    if (f <= 0) {
      arr.set(base);
      this.explodedOffsets = null;
    } else {
      const off = data.offsetsAt(f);
      // Style switch mid-explode: ease from the old style's offsets into the new one's.
      if (this.explodeBlendFrom && this.explodeBlendT < 1 && this.explodeBlendFrom.length === off.length) {
        const t = this.explodeBlendT;
        const s = t * t * (3 - 2 * t);
        const from = this.explodeBlendFrom;
        for (let i = 0; i < off.length; i++) off[i] = from[i]! * (1 - s) + off[i]! * s;
      }
      const inst = this.instanceOfVertex;
      for (let v = 0; v < inst.length; v++) {
        const o = inst[v]! * 3, p = v * 3;
        arr[p] = base[p]! + off[o]!;
        arr[p + 1] = base[p + 1]! + off[o + 1]!;
        arr[p + 2] = base[p + 2]! + off[o + 2]!;
      }
      this.explodedOffsets = off;
    }
    attr.needsUpdate = true;
    c.solid.geometry.computeBoundingBox(); // visibleBox/fit read it via expandByObject
    c.solid.geometry.computeBoundingSphere(); // shared (same object) with wire + highlight
    this.refreshEdgeOverlays();
    this.pickBvhDirty = true;
    this.clipSphereDirty = true;
  }

  setWireframe(v: boolean): void {
    this.showWire = v;
    if (v && this.wireDirty) this.rebuildWire();
    this.applyVisibility();
  }
  /** Show/hide the model's own STEP face colors. */
  setShowColors(v: boolean): void { this.showColors = v; this.applyVisibility(); }
  /** Smooth vs flat shading. The crease-aware normals are baked into the geometry by the
   * loader (autoSmooth), so this only flips the material flag — no geometry work. */
  setSmoothShading(v: boolean): void { this.smoothOn = v; this.applyVisibility(); }
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
    // (near/far are owned by updateClipPlanes, re-derived every frame.)
    this.persp.up.set(0, 0, 1);
    this.ortho.up.set(0, 0, 1);
    this.persp.position.copy(pos);
    this.persp.updateProjectionMatrix();

    this.ortho.position.copy(pos);
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
    this.persp.updateProjectionMatrix();

    this.ortho.position.copy(p);
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
