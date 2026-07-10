// SPDX-License-Identifier: AGPL-3.0-only
// Interactive measurement: point-to-point distance with smart snapping (CAD vertices, arc/circle
// centers, edge midpoints, on-edge points, free surface points) and edge measurements (length,
// exact hole Ø / arc radius from the STEP records). Snap geometry comes from the worker's
// MeasureGeometry payload, whose polylines are the exact samplings the mesh was built from.
import * as THREE from "three";
import { CSS2DObject } from "three/addons/renderers/CSS2DRenderer.js";
import type { MeasureGeometry, MeasureEdge } from "../../src/index.ts";

export type MeasureMode = "distance" | "edge";

export interface MeasureHost {
  scene: THREE.Scene;
  camera(): THREE.PerspectiveCamera | THREE.OrthographicCamera;
  domElement(): HTMLCanvasElement;
  /** Nearest VISIBLE surface point under the client coords (hidden parts + section respected). */
  raycastSurface(clientX: number, clientY: number): THREE.Vector3 | null;
  /** Active section plane, or null when the section view is off. */
  sectionPlane(): THREE.Plane | null;
  hiddenSolids(): ReadonlySet<number>;
  /** True while another pointer interaction owns the gesture (section-gizmo drag). */
  busy(): boolean;
  /** Mode was exited internally (ESC) — sync the sidebar checkbox. */
  onEnabledChanged(on: boolean): void;
  /** Model bbox diagonal in mm (occlusion slack / dash sizing). */
  bboxDiag(): number;
}

type SnapKind = "vertex" | "center" | "midpoint" | "edge" | "surface";

interface Snap {
  kind: SnapKind;
  point: THREE.Vector3;
  /** Index into data.edges when the snap came from an edge (center/midpoint/on-edge). */
  edgeIdx: number | null;
}

interface Measurement {
  objects: THREE.Object3D[];
  labels: CSS2DObject[];
}

/** Screen-px snap radii (doubled for touch). Priority = object order below. */
const SNAP_PX = { vertex: 10, center: 10, midpoint: 8, edge: 6 };
const SNAP_COLORS: Record<SnapKind, number> = {
  vertex: 0xffa62b, center: 0x4da3ff, midpoint: 0x3ddc84, edge: 0xffffff, surface: 0x9aa5b1,
};
const LINE_COLOR = { light: 0xb45309, dark: 0xffb020 };

const fmtMm = (v: number): string => {
  const a = Math.abs(v);
  const d = a >= 100 ? 2 : a >= 1 ? 3 : 4;
  return `${v.toFixed(d)} mm`;
};
const fmtDeg = (rad: number): string => `${((rad * 180) / Math.PI).toFixed(1)}°`;

export class MeasureController {
  private host: MeasureHost;
  private _enabled = false;
  private mode: MeasureMode = "distance";
  private data: MeasureGeometry | null = null;
  private theme: "light" | "dark" = "dark";

  // Snap caches (rebuilt in setData). Point candidates are packed xyz triples.
  private verts = new Float32Array(0);
  private vertSolid = new Uint32Array(0);
  private centers = new Float32Array(0);
  private centerSolid = new Uint32Array(0);
  private centerEdge = new Uint32Array(0);
  private mids = new Float32Array(0);
  private midSolid = new Uint32Array(0);
  private midEdge = new Uint32Array(0);
  // Per-edge polyline bounding spheres (cull before the segment scan).
  private sphC = new Float32Array(0);
  private sphR = new Float32Array(0);

  // Scene graph: everything lives in one group; materials get NO clipping planes, so
  // measurement graphics are never cut by the section view.
  private group = new THREE.Group();
  private hoverMarker: THREE.Mesh;
  private firstMarker: THREE.Mesh;
  private edgeHover: THREE.Line;
  private rubber: THREE.Line;
  private rubberLabel: CSS2DObject;
  private lineMat: THREE.LineBasicMaterial;
  private rubberMat: THREE.LineDashedMaterial;
  private edgeMat: THREE.LineBasicMaterial;
  private measurements: Measurement[] = [];
  /** Frustum-scaled marker meshes: [mesh, relative size]. */
  private scalables: [THREE.Mesh, number][] = [];

  private firstPoint: Snap | null = null;
  private downAt: { x: number; y: number } | null = null;
  /** Latest pointer position, processed once per frame in update() (rAF throttle). */
  private pendingMove: { x: number; y: number; touch: boolean } | null = null;

  private raycaster = new THREE.Raycaster();
  private _ndc = new THREE.Vector2();
  private _v = new THREE.Vector3();
  private _a = new THREE.Vector3();
  private _b = new THREE.Vector3();
  private _seg = new THREE.Vector3();
  // Dedicated slot for the ray-to-segment closest point: pxOf() clobbers _v/_a (project() turns
  // _v into NDC coords near the origin), so the candidate must never live in those.
  private _segPt = new THREE.Vector3();

  constructor(host: MeasureHost) {
    this.host = host;
    this.group.name = "measure";
    host.scene.add(this.group);

    const markerGeo = new THREE.SphereGeometry(1, 16, 10);
    this.hoverMarker = new THREE.Mesh(markerGeo, new THREE.MeshBasicMaterial({ color: SNAP_COLORS.surface, depthTest: false }));
    this.hoverMarker.renderOrder = 11;
    this.hoverMarker.visible = false;
    this.group.add(this.hoverMarker);
    this.scalables.push([this.hoverMarker, 0.012]);

    this.firstMarker = new THREE.Mesh(markerGeo, new THREE.MeshBasicMaterial({ color: SNAP_COLORS.vertex, depthTest: false }));
    this.firstMarker.renderOrder = 11;
    this.firstMarker.visible = false;
    this.group.add(this.firstMarker);
    this.scalables.push([this.firstMarker, 0.01]);

    this.lineMat = new THREE.LineBasicMaterial({ color: LINE_COLOR.dark, depthTest: false });
    this.rubberMat = new THREE.LineDashedMaterial({ color: LINE_COLOR.dark, depthTest: false, dashSize: 1, gapSize: 0.6, transparent: true, opacity: 0.9 });
    this.edgeMat = new THREE.LineBasicMaterial({ color: SNAP_COLORS.center, depthTest: false });

    const rubberGeo = new THREE.BufferGeometry();
    rubberGeo.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(6), 3));
    this.rubber = new THREE.Line(rubberGeo, this.rubberMat);
    this.rubber.renderOrder = 9;
    this.rubber.visible = false;
    this.rubber.frustumCulled = false;
    this.group.add(this.rubber);
    this.rubberLabel = this.makeLabel("");
    this.rubberLabel.visible = false;
    this.group.add(this.rubberLabel);

    this.edgeHover = new THREE.Line(new THREE.BufferGeometry(), this.edgeMat);
    this.edgeHover.renderOrder = 9;
    this.edgeHover.visible = false;
    this.edgeHover.frustumCulled = false;
    this.group.add(this.edgeHover);
  }

  get enabled(): boolean { return this._enabled; }

  setEnabled(on: boolean): void {
    if (on === this._enabled) return;
    this._enabled = on;
    const canvas = this.host.domElement();
    canvas.parentElement?.classList.toggle("measuring", on);
    if (on) {
      canvas.addEventListener("pointerdown", this.onDown);
      document.addEventListener("pointermove", this.onMove);
      document.addEventListener("pointerup", this.onUp);
      document.addEventListener("keydown", this.onKey);
    } else {
      canvas.removeEventListener("pointerdown", this.onDown);
      document.removeEventListener("pointermove", this.onMove);
      document.removeEventListener("pointerup", this.onUp);
      document.removeEventListener("keydown", this.onKey);
      this.cancelPending();
    }
  }

  setMode(m: MeasureMode): void {
    this.mode = m;
    this.cancelPending();
  }

  /** Swap the model's measurement payload; clears every measurement and snap cache. */
  setData(d: MeasureGeometry | null): void {
    this.data = d;
    this.clearAll();
    this.cancelPending();
    this.buildCaches();
    const diag = this.host.bboxDiag();
    this.rubberMat.dashSize = diag * 0.012;
    this.rubberMat.gapSize = diag * 0.008;
  }

  clearAll(): void {
    for (const m of this.measurements) {
      for (const o of m.objects) {
        this.group.remove(o);
        (o as THREE.Mesh).geometry?.dispose?.();
      }
      for (const l of m.labels) {
        this.group.remove(l);
        l.element.remove();
      }
    }
    // Committed endpoint markers were appended after the two built-ins — drop them.
    this.scalables.length = 2;
    this.measurements = [];
  }

  setTheme(mode: "light" | "dark"): void {
    this.theme = mode;
    const c = LINE_COLOR[mode];
    this.lineMat.color.set(c);
    this.rubberMat.color.set(c);
  }

  /** Per-frame: process the queued pointer move and keep markers at constant apparent size. */
  update(): void {
    if (this.pendingMove && this._enabled) {
      const { x, y, touch } = this.pendingMove;
      this.pendingMove = null;
      this.hover(x, y, touch);
    }
    if (!this.scalables.length) return;
    const cam = this.host.camera();
    for (const [mesh, k] of this.scalables) {
      if (!mesh.visible) continue;
      const halfH = (cam as THREE.OrthographicCamera).isOrthographicCamera
        ? ((cam as THREE.OrthographicCamera).top - (cam as THREE.OrthographicCamera).bottom) / 2 / (cam as THREE.OrthographicCamera).zoom
        : cam.position.distanceTo(mesh.position) * Math.tan(((cam as THREE.PerspectiveCamera).fov * Math.PI) / 360);
      mesh.scale.setScalar(Math.max(halfH * k, 1e-6));
    }
  }

  dispose(): void {
    this.setEnabled(false);
    this.clearAll();
    this.host.scene.remove(this.group);
  }

  // ---------- pointer handling ----------
  // Clicks resolve on pointerup with <3px total movement (the orbit promotes at 3px), so
  // left-drag still orbits exactly as before and a clean click measures.

  private onDown = (ev: PointerEvent): void => {
    if (ev.button !== 0 || this.host.busy()) { this.downAt = null; return; }
    this.downAt = { x: ev.clientX, y: ev.clientY };
  };

  private onMove = (ev: PointerEvent): void => {
    this.pendingMove = { x: ev.clientX, y: ev.clientY, touch: ev.pointerType === "touch" };
  };

  private onUp = (ev: PointerEvent): void => {
    const d = this.downAt;
    this.downAt = null;
    if (!d || ev.button !== 0) return;
    if (Math.hypot(ev.clientX - d.x, ev.clientY - d.y) >= 3) return; // it was an orbit drag
    if (this.host.busy()) return;
    const snap = this.query(ev.clientX, ev.clientY, ev.pointerType === "touch");
    if (!snap) return;
    if (this.mode === "distance") this.clickDistance(snap);
    else if (snap.kind === "edge" || snap.kind === "center" || snap.kind === "midpoint") this.commitEdge(snap);
  };

  private onKey = (ev: KeyboardEvent): void => {
    if (ev.key !== "Escape") return;
    const t = ev.target as HTMLElement | null;
    if (t && (t.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName))) return;
    ev.preventDefault();
    if (this.firstPoint) { this.cancelPending(); return; } // first ESC: drop the in-progress point
    this.setEnabled(false); // second ESC: leave the mode
    this.host.onEnabledChanged(false);
  };

  private cancelPending(): void {
    this.firstPoint = null;
    this.firstMarker.visible = false;
    this.rubber.visible = false;
    this.rubberLabel.visible = false;
    this.hoverMarker.visible = false;
    this.edgeHover.visible = false;
  }

  // ---------- hover / snapping ----------

  private hover(x: number, y: number, touch: boolean): void {
    const snap = this.query(x, y, touch);
    if (!snap) {
      this.hoverMarker.visible = false;
      this.edgeHover.visible = false;
      if (this.firstPoint) { this.rubber.visible = false; this.rubberLabel.visible = false; }
      return;
    }
    (this.hoverMarker.material as THREE.MeshBasicMaterial).color.set(SNAP_COLORS[snap.kind]);
    this.hoverMarker.position.copy(snap.point);
    this.hoverMarker.visible = true;

    // Edge mode: highlight the whole hovered edge polyline.
    if (this.mode === "edge" && snap.edgeIdx !== null) this.showEdgeHover(snap.edgeIdx);
    else this.edgeHover.visible = false;

    // Distance mode with a first point placed: rubber-band + live readout.
    if (this.mode === "distance" && this.firstPoint) {
      const a = this.firstPoint.point, b = snap.point;
      const attr = this.rubber.geometry.getAttribute("position") as THREE.BufferAttribute;
      attr.setXYZ(0, a.x, a.y, a.z);
      attr.setXYZ(1, b.x, b.y, b.z);
      attr.needsUpdate = true;
      this.rubber.geometry.computeBoundingSphere();
      this.rubber.computeLineDistances();
      this.rubber.visible = true;
      this.setLabel(this.rubberLabel, fmtMm(a.distanceTo(b)), a.clone().add(b).multiplyScalar(0.5));
      this.rubberLabel.visible = true;
    }
  }

  /**
   * Snap query: project candidates to screen px and pick by priority
   * vertex > center > midpoint > on-edge > free surface point. Candidates on hidden parts, on
   * the clipped-away section side, or occluded behind the visible surface are rejected.
   */
  private query(clientX: number, clientY: number, touch: boolean): Snap | null {
    const canvas = this.host.domElement();
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return null;
    const cam = this.host.camera();
    cam.updateMatrixWorld();
    this._ndc.set(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(this._ndc, cam);
    const ray = this.raycaster.ray;

    const surface = this.host.raycastSurface(clientX, clientY);
    // Occlusion budget: candidates deeper than the surface hit (plus a slack that absorbs the
    // chord tolerance and silhouette grazing) can't be snapped through the model.
    const slack = Math.max(0.1, this.host.bboxDiag() * 5e-3);
    const maxT = surface ? this._v.copy(surface).sub(ray.origin).dot(ray.direction) + slack : Infinity;
    const hidden = this.host.hiddenSolids();
    const plane = this.host.sectionPlane();
    const mul = touch ? 2 : 1;

    // Screen distance (px) of a world point to the pointer; Infinity when occluded or clipped.
    const pxOf = (x: number, y: number, z: number): number => {
      this._v.set(x, y, z);
      if (plane && plane.distanceToPoint(this._v) < -1e-6) return Infinity;
      const t = this._a.copy(this._v).sub(ray.origin).dot(ray.direction);
      if (t < 0 || t > maxT) return Infinity;
      this._v.project(cam);
      if (this._v.z > 1) return Infinity;
      const px = rect.left + ((this._v.x + 1) / 2) * rect.width;
      const py = rect.top + ((1 - this._v.y) / 2) * rect.height;
      return Math.hypot(px - clientX, py - clientY);
    };

    // Nearest candidate index within `pts` by screen distance (hidden parts skipped).
    const scanPoints = (pts: Float32Array, solids: Uint32Array): { d: number; i: number } => {
      let d = Infinity, bi = -1;
      for (let i = 0; i < solids.length; i++) {
        if (hidden.has(solids[i]!)) continue;
        const dd = pxOf(pts[i * 3]!, pts[i * 3 + 1]!, pts[i * 3 + 2]!);
        if (dd < d) { d = dd; bi = i; }
      }
      return { d, i: bi };
    };
    const at = (pts: Float32Array, i: number): THREE.Vector3 => new THREE.Vector3(pts[i * 3]!, pts[i * 3 + 1]!, pts[i * 3 + 2]!);

    if (this.mode === "distance" && this.data) {
      const v = scanPoints(this.verts, this.vertSolid);
      if (v.d <= SNAP_PX.vertex * mul) return { kind: "vertex", point: at(this.verts, v.i), edgeIdx: null };
      const c = scanPoints(this.centers, this.centerSolid);
      if (c.d <= SNAP_PX.center * mul) return { kind: "center", point: at(this.centers, c.i), edgeIdx: this.centerEdge[c.i]! };
      const m = scanPoints(this.mids, this.midSolid);
      if (m.d <= SNAP_PX.midpoint * mul) return { kind: "midpoint", point: at(this.mids, m.i), edgeIdx: this.midEdge[m.i]! };
    }

    // On-edge: cull edges by polyline bounding sphere vs the ray, then scan segments.
    const onEdge = this.nearestEdgePoint(ray, pxOf, maxT);
    const edgeLimit = (this.mode === "edge" ? SNAP_PX.vertex : SNAP_PX.edge) * mul;
    if (onEdge && onEdge.d <= edgeLimit) return { kind: "edge", point: onEdge.point, edgeIdx: onEdge.edgeIdx };

    if (this.mode === "edge") {
      // Centers still make sense as edge picks (they identify the circle).
      if (this.data) {
        const c = scanPoints(this.centers, this.centerSolid);
        if (c.d <= SNAP_PX.center * mul) return { kind: "center", point: at(this.centers, c.i), edgeIdx: this.centerEdge[c.i]! };
      }
      return null;
    }
    return surface ? { kind: "surface", point: surface.clone(), edgeIdx: null } : null;
  }

  private nearestEdgePoint(
    ray: THREE.Ray, pxOf: (x: number, y: number, z: number) => number, maxT: number,
  ): { d: number; point: THREE.Vector3; edgeIdx: number } | null {
    const d = this.data;
    if (!d) return null;
    const hidden = this.host.hiddenSolids();
    const pts = d.points;
    let bestD = Infinity, bestIdx = -1;
    const bestPt = new THREE.Vector3();
    // World-space cull radius: a generous px band converted at the sphere's depth.
    for (let ei = 0; ei < d.edges.length; ei++) {
      const e = d.edges[ei]!;
      if (hidden.has(e.solidId)) continue;
      const sc = this._seg.set(this.sphC[ei * 3]!, this.sphC[ei * 3 + 1]!, this.sphC[ei * 3 + 2]!);
      const t = this._a.copy(sc).sub(ray.origin).dot(ray.direction);
      if (t - this.sphR[ei]! > maxT) continue;
      const worldTol = this.worldPerPx(Math.max(t, 0)) * 24; // ~24px cull band
      if (ray.distanceSqToPoint(sc) > (this.sphR[ei]! + worldTol) ** 2) continue;
      for (let i = 1; i < e.count; i++) {
        const o0 = (e.first + i - 1) * 3, o1 = (e.first + i) * 3;
        this._a.set(pts[o0]!, pts[o0 + 1]!, pts[o0 + 2]!);
        this._b.set(pts[o1]!, pts[o1 + 1]!, pts[o1 + 2]!);
        ray.distanceSqToSegment(this._a, this._b, undefined, this._segPt);
        const dd = pxOf(this._segPt.x, this._segPt.y, this._segPt.z); // clobbers _v/_a, not _segPt
        if (dd < bestD) { bestD = dd; bestIdx = ei; bestPt.copy(this._segPt); }
      }
    }
    if (bestIdx === -1) return null;
    // Analytic refinement: project the picked point onto the exact circle, not its chords.
    const e = this.data!.edges[bestIdx]!;
    if (e.kind === "circle" && e.center && e.axis && e.radius) {
      const c = new THREE.Vector3(...e.center), ax = new THREE.Vector3(...e.axis);
      const rad = bestPt.clone().sub(c).addScaledVector(ax, -bestPt.clone().sub(c).dot(ax));
      if (rad.lengthSq() > 1e-12) bestPt.copy(c).addScaledVector(rad.normalize(), e.radius);
    }
    return { d: bestD, point: bestPt, edgeIdx: bestIdx };
  }

  /** World units per screen pixel at view depth t (both projections). */
  private worldPerPx(t: number): number {
    const cam = this.host.camera();
    const h = this.host.domElement().getBoundingClientRect().height || 1;
    if ((cam as THREE.OrthographicCamera).isOrthographicCamera) {
      const oc = cam as THREE.OrthographicCamera;
      return (oc.top - oc.bottom) / oc.zoom / h;
    }
    return (2 * Math.max(t, 1e-3) * Math.tan(((cam as THREE.PerspectiveCamera).fov * Math.PI) / 360)) / h;
  }

  // ---------- committing measurements ----------

  private clickDistance(snap: Snap): void {
    if (!this.firstPoint) {
      this.firstPoint = snap;
      (this.firstMarker.material as THREE.MeshBasicMaterial).color.set(SNAP_COLORS[snap.kind]);
      this.firstMarker.position.copy(snap.point);
      this.firstMarker.visible = true;
      return;
    }
    const a = this.firstPoint.point, b = snap.point;
    this.firstPoint = null;
    this.firstMarker.visible = false;
    this.rubber.visible = false;
    this.rubberLabel.visible = false;
    if (a.distanceTo(b) < 1e-9) return;

    const geo = new THREE.BufferGeometry().setFromPoints([a, b]);
    const line = new THREE.Line(geo, this.lineMat);
    line.renderOrder = 9;
    line.frustumCulled = false;
    const dotA = this.makeDot(a), dotB = this.makeDot(b);
    const label = this.makeLabel(fmtMm(a.distanceTo(b)), a.clone().add(b).multiplyScalar(0.5));
    label.element.title = `ΔX ${fmtMm(Math.abs(b.x - a.x))}  ΔY ${fmtMm(Math.abs(b.y - a.y))}  ΔZ ${fmtMm(Math.abs(b.z - a.z))}`;
    this.group.add(line, dotA, dotB, label);
    this.measurements.push({ objects: [line, dotA, dotB], labels: [label] });
  }

  private commitEdge(snap: Snap): void {
    const d = this.data;
    if (!d || snap.edgeIdx === null) return;
    const e = d.edges[snap.edgeIdx]!;
    const objects: THREE.Object3D[] = [];

    const poly = new THREE.Line(this.edgePolylineGeometry(e), this.lineMat);
    poly.renderOrder = 9;
    poly.frustumCulled = false;
    objects.push(poly);

    // Label anchor: arcs/circles read best at their CENTER (with a center dot to match);
    // straight and freeform edges at the middle of the curve.
    let anchor: THREE.Vector3;
    if ((e.kind === "circle" || e.kind === "ellipse") && e.center) {
      anchor = new THREE.Vector3(...e.center);
      objects.push(this.makeDot(anchor, SNAP_COLORS.center));
    } else {
      anchor = this.edgeMidpoint(e);
    }
    const label = this.makeLabel(this.edgeLabel(e), anchor);
    for (const o of objects) this.group.add(o);
    this.group.add(label);
    this.measurements.push({ objects, labels: [label] });
  }

  /** Point at half arc length along an edge's polyline. */
  private edgeMidpoint(e: MeasureEdge): THREE.Vector3 {
    const pts = this.data!.points;
    let total = 0;
    for (let i = 1; i < e.count; i++) {
      const a = (e.first + i - 1) * 3, b = (e.first + i) * 3;
      total += Math.hypot(pts[b]! - pts[a]!, pts[b + 1]! - pts[a + 1]!, pts[b + 2]! - pts[a + 2]!);
    }
    let acc = 0;
    const half = total / 2;
    for (let i = 1; i < e.count; i++) {
      const a = (e.first + i - 1) * 3, b = (e.first + i) * 3;
      const l = Math.hypot(pts[b]! - pts[a]!, pts[b + 1]! - pts[a + 1]!, pts[b + 2]! - pts[a + 2]!);
      if (acc + l >= half && l > 0) {
        const f = (half - acc) / l;
        return new THREE.Vector3(
          pts[a]! + (pts[b]! - pts[a]!) * f,
          pts[a + 1]! + (pts[b + 1]! - pts[a + 1]!) * f,
          pts[a + 2]! + (pts[b + 2]! - pts[a + 2]!) * f,
        );
      }
      acc += l;
    }
    const o = e.first * 3;
    return new THREE.Vector3(pts[o]!, pts[o + 1]!, pts[o + 2]!);
  }

  private edgeLabel(e: MeasureEdge): string {
    if (e.kind === "circle") {
      const full = Math.abs(e.sweep ?? 0) > Math.PI * 2 - 1e-3;
      return full ? `Ø ${fmtMm((e.radius ?? 0) * 2)}` : `R ${fmtMm(e.radius ?? 0)} · ${fmtDeg(Math.abs(e.sweep ?? 0))}`;
    }
    if (e.kind === "ellipse") return `⌀ ${fmtMm((e.radius ?? 0) * 2)} × ${fmtMm((e.radius2 ?? 0) * 2)}`;
    if (e.kind === "line") return `L ${fmtMm(e.length)}`;
    return `L ≈ ${fmtMm(e.length)}`;
  }

  // ---------- scene helpers ----------

  private makeDot(p: THREE.Vector3, color?: number): THREE.Mesh {
    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(1, 12, 8),
      new THREE.MeshBasicMaterial({ color: color ?? LINE_COLOR[this.theme], depthTest: false }),
    );
    dot.renderOrder = 11;
    dot.position.copy(p);
    this.scalables.push([dot, 0.007]);
    return dot;
  }

  private makeLabel(text: string, pos?: THREE.Vector3): CSS2DObject {
    const div = document.createElement("div");
    div.className = "measure-label";
    div.textContent = text;
    const label = new CSS2DObject(div);
    if (pos) label.position.copy(pos);
    return label;
  }

  private setLabel(label: CSS2DObject, text: string, pos: THREE.Vector3): void {
    label.element.textContent = text;
    label.position.copy(pos);
  }

  private showEdgeHover(edgeIdx: number): void {
    const e = this.data?.edges[edgeIdx];
    if (!e) return;
    this.edgeHover.geometry.dispose();
    this.edgeHover.geometry = this.edgePolylineGeometry(e);
    this.edgeHover.visible = true;
  }

  private edgePolylineGeometry(e: MeasureEdge): THREE.BufferGeometry {
    const pts = this.data!.points;
    const arr = new Float32Array(e.count * 3);
    arr.set(pts.subarray(e.first * 3, (e.first + e.count) * 3));
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(arr, 3));
    return geo;
  }

  // ---------- snap caches ----------

  private buildCaches(): void {
    const d = this.data;
    if (!d) {
      this.verts = new Float32Array(0); this.vertSolid = new Uint32Array(0);
      this.centers = new Float32Array(0); this.centerSolid = new Uint32Array(0); this.centerEdge = new Uint32Array(0);
      this.mids = new Float32Array(0); this.midSolid = new Uint32Array(0); this.midEdge = new Uint32Array(0);
      this.sphC = new Float32Array(0); this.sphR = new Float32Array(0);
      return;
    }
    const pts = d.points;
    const nE = d.edges.length;

    // CAD vertices: deduped polyline endpoints (many edges share each vertex).
    const seen = new Map<string, number>();
    const vx: number[] = [], vs: number[] = [];
    const addVert = (o: number, solid: number): void => {
      const k = `${Math.round(pts[o]! * 1e4)}_${Math.round(pts[o + 1]! * 1e4)}_${Math.round(pts[o + 2]! * 1e4)}`;
      if (seen.has(k)) return;
      seen.set(k, vx.length / 3);
      vx.push(pts[o]!, pts[o + 1]!, pts[o + 2]!);
      vs.push(solid);
    };

    const cx: number[] = [], cs: number[] = [], ce: number[] = [];
    const mx: number[] = [], ms: number[] = [], me: number[] = [];
    this.sphC = new Float32Array(nE * 3);
    this.sphR = new Float32Array(nE);

    for (let ei = 0; ei < nE; ei++) {
      const e = d.edges[ei]!;
      addVert(e.first * 3, e.solidId);
      addVert((e.first + e.count - 1) * 3, e.solidId);
      if ((e.kind === "circle" || e.kind === "ellipse") && e.center) {
        cx.push(e.center[0], e.center[1], e.center[2]);
        cs.push(e.solidId);
        ce.push(ei);
      }
      // Midpoint snap candidate at half arc length along the polyline.
      if (e.count >= 2) {
        const m = this.edgeMidpoint(e);
        mx.push(m.x, m.y, m.z);
        ms.push(e.solidId);
        me.push(ei);
      }
      // Bounding sphere: box center + max distance to it.
      let lx = Infinity, ly = Infinity, lz = Infinity, hx = -Infinity, hy = -Infinity, hz = -Infinity;
      for (let i = 0; i < e.count; i++) {
        const o = (e.first + i) * 3;
        const x = pts[o]!, y = pts[o + 1]!, z = pts[o + 2]!;
        if (x < lx) lx = x; if (x > hx) hx = x;
        if (y < ly) ly = y; if (y > hy) hy = y;
        if (z < lz) lz = z; if (z > hz) hz = z;
      }
      const ccx = (lx + hx) / 2, ccy = (ly + hy) / 2, ccz = (lz + hz) / 2;
      this.sphC[ei * 3] = ccx; this.sphC[ei * 3 + 1] = ccy; this.sphC[ei * 3 + 2] = ccz;
      this.sphR[ei] = Math.hypot(hx - ccx, hy - ccy, hz - ccz);
    }

    this.verts = Float32Array.from(vx); this.vertSolid = Uint32Array.from(vs);
    this.centers = Float32Array.from(cx); this.centerSolid = Uint32Array.from(cs); this.centerEdge = Uint32Array.from(ce);
    this.mids = Float32Array.from(mx); this.midSolid = Uint32Array.from(ms); this.midEdge = Uint32Array.from(me);
  }
}
