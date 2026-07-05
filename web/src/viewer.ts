// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export type Side = "left" | "right";

interface SideContent {
  group: THREE.Group;
  solid: THREE.Mesh | null;
  wire: THREE.LineSegments | null;
  edges: THREE.LineSegments | null;
  reference: THREE.Mesh | null;
}

const BASE_COLOR = 0x6f8fb0;
const REF_COLOR = 0x33dd88;
const EDGE_COLOR = 0xff3b30;

/**
 * Two synced viewports rendered into a single canvas via scissor regions.
 * A single shared camera + OrbitControls guarantees the views stay locked.
 */
export class DualViewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private sides: Record<Side, SideContent>;
  private container: HTMLElement;

  // toggle state
  private showWire = false;
  private showEdges = false;
  private showReference = false;
  private deviationOn = false;

  constructor(container: HTMLElement) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setScissorTest(true);
    container.appendChild(this.renderer.domElement);

    this.scene.background = new THREE.Color(0x14181d);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1e6);
    this.camera.position.set(60, 45, 80);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;

    // Lights (shared scene).
    const hemi = new THREE.HemisphereLight(0xffffff, 0x404048, 1.0);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(1, 1.5, 1);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.7);
    fill.position.set(-1, -0.5, -1);
    this.scene.add(fill);

    const mk = (): SideContent => {
      const group = new THREE.Group();
      this.scene.add(group);
      return { group, solid: null, wire: null, edges: null, reference: null };
    };
    this.sides = { left: mk(), right: mk() };

    const grid = new THREE.GridHelper(200, 20, 0x2a3038, 0x20252b);
    (grid.material as THREE.Material).depthWrite = false;
    this.scene.add(grid);

    window.addEventListener("resize", () => this.resize());
    this.resize();
    this.animate();
  }

  private resize(): void {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.renderer.setSize(w, h, true);
    this.camera.aspect = w / 2 / h;
    this.camera.updateProjectionMatrix();
  }

  private animate = (): void => {
    requestAnimationFrame(this.animate);
    this.controls.update();
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    const halfW = Math.floor(w / 2);

    // Left viewport: show only left group.
    this.setGroupVisible("left", true);
    this.setGroupVisible("right", false);
    this.renderer.setViewport(0, 0, halfW, h);
    this.renderer.setScissor(0, 0, halfW, h);
    this.renderer.render(this.scene, this.camera);

    // Right viewport: show only right group.
    this.setGroupVisible("left", false);
    this.setGroupVisible("right", true);
    this.renderer.setViewport(halfW, 0, w - halfW, h);
    this.renderer.setScissor(halfW, 0, w - halfW, h);
    this.renderer.render(this.scene, this.camera);
  };

  private setGroupVisible(side: Side, v: boolean): void {
    this.sides[side].group.visible = v;
  }

  private applyVisibility(): void {
    for (const side of ["left", "right"] as Side[]) {
      const c = this.sides[side];
      if (c.wire) c.wire.visible = this.showWire;
      if (c.edges) c.edges.visible = this.showEdges;
      if (c.reference) c.reference.visible = this.showReference;
      if (c.solid) {
        const m = c.solid.material as THREE.MeshStandardMaterial;
        m.vertexColors = this.deviationOn && !!c.solid.geometry.getAttribute("color");
        m.color.set(m.vertexColors ? 0xffffff : BASE_COLOR);
        m.needsUpdate = true;
      }
    }
  }

  /** Replace the solid + wireframe + open-edge geometry for one side. */
  setSide(side: Side, geometry: THREE.BufferGeometry, boundaryPositions: Float32Array): void {
    const c = this.sides[side];
    this.disposeSideMeshes(c);

    const mat = new THREE.MeshStandardMaterial({
      color: BASE_COLOR,
      metalness: 0.0,
      roughness: 0.85,
      side: THREE.DoubleSide,
      flatShading: true,
    });
    c.solid = new THREE.Mesh(geometry, mat);
    c.group.add(c.solid);

    const wireMat = new THREE.LineBasicMaterial({ color: 0x0c0f12, transparent: true, opacity: 0.55 });
    c.wire = new THREE.LineSegments(new THREE.WireframeGeometry(geometry), wireMat);
    c.wire.visible = this.showWire;
    c.group.add(c.wire);

    const edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute("position", new THREE.Float32BufferAttribute(boundaryPositions, 3));
    const edgeMat = new THREE.LineBasicMaterial({ color: EDGE_COLOR, depthTest: false });
    c.edges = new THREE.LineSegments(edgeGeo, edgeMat);
    c.edges.renderOrder = 10;
    c.edges.visible = this.showEdges;
    c.group.add(c.edges);

    this.applyVisibility();
  }

  /** Set (or clear) the reference overlay; shown in both viewports. */
  setReference(geometry: THREE.BufferGeometry | null): void {
    for (const side of ["left", "right"] as Side[]) {
      const c = this.sides[side];
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
        // Both sides share the same geometry instance.
        c.reference = new THREE.Mesh(geometry, mat);
        c.reference.visible = this.showReference;
        c.group.add(c.reference);
      }
    }
  }

  /** Apply (or clear) per-vertex deviation colors for one side's solid mesh. */
  setDeviationColors(side: Side, colors: Float32Array | null): void {
    const c = this.sides[side];
    if (!c.solid) return;
    const geo = c.solid.geometry;
    if (colors) {
      geo.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    } else {
      geo.deleteAttribute("color");
    }
    this.applyVisibility();
  }

  setWireframe(v: boolean): void { this.showWire = v; this.applyVisibility(); }
  setOpenEdges(v: boolean): void { this.showEdges = v; this.applyVisibility(); }
  setReferenceVisible(v: boolean): void { this.showReference = v; this.applyVisibility(); }
  setDeviation(v: boolean): void { this.deviationOn = v; this.applyVisibility(); }

  /** Frame the camera to the combined content. */
  fit(): void {
    const box = new THREE.Box3();
    let has = false;
    for (const side of ["left", "right"] as Side[]) {
      const s = this.sides[side].solid;
      if (s) { box.expandByObject(s); has = true; }
    }
    if (!has) return;
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z) * 0.5 || 1;
    const dist = radius / Math.sin((this.camera.fov * Math.PI) / 360) * 1.4;
    const dir = new THREE.Vector3(0.7, 0.5, 1).normalize();
    this.camera.position.copy(center).addScaledVector(dir, dist);
    this.camera.near = dist / 100;
    this.camera.far = dist * 100;
    this.camera.updateProjectionMatrix();
    this.controls.target.copy(center);
    this.controls.update();
  }

  private disposeSideMeshes(c: SideContent): void {
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
  }
}
