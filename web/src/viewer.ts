// SPDX-License-Identifier: AGPL-3.0-only
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

interface Content {
  group: THREE.Group;
  solid: THREE.Mesh | null;
  wire: THREE.LineSegments | null;
  edges: THREE.LineSegments | null;
  feature: THREE.LineSegments | null;
  reference: THREE.Mesh | null;
}

const BASE_COLOR = 0x6f8fb0;
const REF_COLOR = 0x33dd88;
const EDGE_COLOR = 0xff3b30;
const FEATURE_COLOR = 0x0b0e12;

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

  // toggle state
  private orthoOn = false;
  private showWire = false;
  private showEdges = false;
  private showFeature = true; // signature CAD face-border look, on by default
  private showReference = false;
  private deviationOn = false;
  private transparentOn = false;
  private solidVisible = true; // false => CAD-edges-only view

  constructor(container: HTMLElement) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);

    this.scene.background = new THREE.Color(0x14181d);

    this.persp = new THREE.PerspectiveCamera(45, 1, 0.01, 1e6);
    this.persp.position.set(60, 45, 80);
    this.ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 1e6);
    this.ortho.position.set(60, 45, 80);
    this.camera = this.persp;

    this.controls = this.makeControls(new THREE.Vector3());

    // Lights.
    const hemi = new THREE.HemisphereLight(0xffffff, 0x404048, 1.0);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(1, 1.5, 1);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.7);
    fill.position.set(-1, -0.5, -1);
    this.scene.add(fill);

    const group = new THREE.Group();
    this.scene.add(group);
    this.content = { group, solid: null, wire: null, edges: null, feature: null, reference: null };

    window.addEventListener("resize", () => this.resize());
    this.resize();
    this.animate();
  }

  private resize(): void {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.renderer.setSize(w, h, true);
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
    // Free orbit: full vertical range (view from directly above or below), no azimuth limit.
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
      this.ortho.zoom = 1;
      this.ortho.updateProjectionMatrix();
      this.camera = this.ortho;
    } else {
      this.persp.position.copy(pos);
      this.persp.updateProjectionMatrix();
      this.camera = this.persp;
    }
    this.controls.dispose();
    this.controls = this.makeControls(target);
  }

  private animate = (): void => {
    requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  private applyVisibility(): void {
    const c = this.content;
    if (c.wire) c.wire.visible = this.showWire;
    if (c.edges) c.edges.visible = this.showEdges;
    if (c.feature) c.feature.visible = this.showFeature;
    if (c.reference) c.reference.visible = this.showReference;
    if (c.solid) {
      c.solid.visible = this.solidVisible;
      const m = c.solid.material as THREE.MeshStandardMaterial;
      m.vertexColors = this.deviationOn && !!c.solid.geometry.getAttribute("color");
      m.color.set(m.vertexColors ? 0xffffff : BASE_COLOR);
      m.transparent = this.transparentOn;
      m.opacity = this.transparentOn ? 0.4 : 1;
      m.depthWrite = !this.transparentOn; // let edges/back faces show through when translucent
      m.needsUpdate = true;
    }
  }

  /** Replace the solid + wireframe + open-edge + surface-boundary geometry. */
  setMesh(geometry: THREE.BufferGeometry, boundaryPositions: Float32Array, featurePositions: Float32Array): void {
    const c = this.content;
    this.disposeMeshes(c);

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
    featGeo.setAttribute("position", new THREE.Float32BufferAttribute(featurePositions, 3));
    const featMat = new THREE.LineBasicMaterial({ color: FEATURE_COLOR });
    c.feature = new THREE.LineSegments(featGeo, featMat);
    c.feature.visible = this.showFeature;
    c.group.add(c.feature);

    // Open (defect) edges: drawn over everything in red so leaks are never hidden.
    const edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute("position", new THREE.Float32BufferAttribute(boundaryPositions, 3));
    const edgeMat = new THREE.LineBasicMaterial({ color: EDGE_COLOR, depthTest: false });
    c.edges = new THREE.LineSegments(edgeGeo, edgeMat);
    c.edges.renderOrder = 10;
    c.edges.visible = this.showEdges;
    c.group.add(c.edges);

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
  }

  /** Apply (or clear) per-vertex deviation colors on the solid mesh. */
  setDeviationColors(colors: Float32Array | null): void {
    const c = this.content;
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
  setFeatureEdges(v: boolean): void { this.showFeature = v; this.applyVisibility(); }
  setTransparent(v: boolean): void { this.transparentOn = v; this.applyVisibility(); }
  /** false => hide the shaded surfaces (CAD-edges-only line view). */
  setSurfacesVisible(v: boolean): void { this.solidVisible = v; this.applyVisibility(); }

  /** Match the 3D background to the UI theme. */
  setTheme(mode: "light" | "dark"): void {
    this.scene.background = new THREE.Color(mode === "light" ? 0xe9edf2 : 0x14181d);
  }
  setReferenceVisible(v: boolean): void { this.showReference = v; this.applyVisibility(); }
  setDeviation(v: boolean): void { this.deviationOn = v; this.applyVisibility(); }

  /** Frame both cameras to the content. */
  fit(): void {
    const s = this.content.solid;
    if (!s) return;
    const box = new THREE.Box3().expandByObject(s);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z) * 0.5 || 1;
    const dist = (radius / Math.sin((this.persp.fov * Math.PI) / 360)) * 1.4;
    const dir = new THREE.Vector3(0.7, 0.5, 1).normalize();
    const pos = center.clone().addScaledVector(dir, dist);

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
  }
}
