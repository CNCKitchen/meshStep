// SPDX-License-Identifier: AGPL-3.0-only
// SpaceMouse support through the 3Dconnexion software's local navigation server (3DxNLServer)
// (3DxNLServer, the route Onshape uses). No WebHID permission is needed, it works in any
// browser, and the 3Dconnexion settings (speed, object / camera mode, buttons such as Fit)
// apply.
//
// Protocol: WAMP v1 over wss://127.51.68.120:<port> (subprotocol "wamp"; the port comes from
// https://127.51.68.120:8181/3dconnexion/nlproxy). The server asks this page for the camera,
// the model box and hit points ("self:read" calls inside its events) and writes the new camera
// back ("self:update"); the 3Dconnexion software does the navigation. Matrices are column-major
// camera-to-world. Needs the 3DxWare helper running and the 3Dconnexion CA trusted.
import * as THREE from "three";

/** What the SpaceMouse needs from the viewer. The camera is resolved late (it is swapped on
 *  perspective <-> orthographic changes). */
export interface SpaceMouseHost {
  camera(): THREE.PerspectiveCamera | THREE.OrthographicCamera;
  isOrtho(): boolean;
  /** Camera apparent size is driven by ortho.zoom; fov is ignored there. */
  setFov(fovDeg: number): void;
  /** Ortho only: world height the view should show (zoom from the 3Dconnexion software). */
  setOrthoHeight(h: number): void;
  /** Orbit target (kept on the view axis so mouse orbit/pan continue sensibly). */
  target(): THREE.Vector3;
  setTarget(p: THREE.Vector3): void;
  /** World-space box of the visible model, or null before a model is loaded. */
  modelBox(): THREE.Box3 | null;
  /** First surface hit along a ray, or null. */
  hit(ray: THREE.Ray): THREE.Vector3 | null;
  /** Camera moved by the 3Dconnexion software: sync derived state (up vector etc. is handled here). */
  onCameraWritten(): void;
}

const NL = "127.51.68.120";

type Handler = { res: (v: any) => void; rej: (e: Error) => void };

export class SpaceMouse {
  /** Human-readable connection status, for the UI. */
  status = "";
  onStatus: ((s: string, connected: boolean) => void) | null = null;

  private ws: WebSocket | null = null;
  private ctrl: string | null = null;
  private calls = new Map<string, Handler>();
  private n = 0;
  private ptr: THREE.Vector2 | null = null; // pointer in NDC
  private hitFrom = new THREE.Vector3();
  private hitDir = new THREE.Vector3(0, 0, -1);
  private pivot = new THREE.Vector3();
  private pivotSet = false;
  private ray = new THREE.Ray();

  constructor(private host: SpaceMouseHost, pointerEl: HTMLElement) {
    pointerEl.addEventListener("pointermove", (e) => {
      const r = pointerEl.getBoundingClientRect();
      this.ptr = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    });
    pointerEl.addEventListener("pointerleave", () => (this.ptr = null));
    const f = (focus: boolean) => () => void this.update({ focus });
    window.addEventListener("focus", f(true));
    window.addEventListener("blur", f(false));
  }

  get connected(): boolean {
    return this.ctrl !== null;
  }

  private setStatus(s: string): void {
    this.status = s;
    this.onStatus?.(s, this.connected);
  }

  private send(a: unknown[]): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(a));
  }

  private call(proc: string, ...args: unknown[]): Promise<any> {
    const id = "c" + ++this.n;
    return new Promise((res, rej) => {
      this.calls.set(id, { res, rej });
      this.send([2, id, proc, ...args]);
    });
  }

  private update(o: Record<string, unknown>): Promise<void> {
    return this.ctrl ? this.call("3dx_rpc:update", this.ctrl, o).then(() => {}, () => {}) : Promise.resolve();
  }

  /** Call once per rendered frame: the page drives the frame clock (timingSource 1). */
  frame(): void {
    if (this.ctrl) this.send([2, "f" + ++this.n, "3dx_rpc:update", this.ctrl, { frame: { time: performance.now() } }]);
  }

  disconnect(): void {
    const ws = this.ws;
    this.ws = null;
    this.ctrl = null;
    ws?.close();
  }

  async connect(): Promise<void> {
    this.disconnect();
    this.setStatus("connecting…");
    let port: number;
    try {
      port = (await (await fetch(`https://${NL}:8181/3dconnexion/nlproxy`)).json()).port;
    } catch {
      this.setStatus("3Dconnexion software not found (is it running and its certificate trusted?)");
      return;
    }
    let ws: WebSocket;
    try {
      ws = new WebSocket(`wss://${NL}:${port}`, "wamp");
    } catch {
      this.setStatus("cannot open the WebSocket to the 3Dconnexion software");
      return;
    }
    this.ws = ws;
    ws.onmessage = (e) => void this.onMessage(ws, JSON.parse(e.data as string));
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      this.ctrl = null;
      this.setStatus("disconnected from the 3Dconnexion software");
    };
    ws.onerror = () => this.setStatus("cannot open the WebSocket to the 3Dconnexion software (certificate not trusted?)");
  }

  private async onMessage(ws: WebSocket, m: any[]): Promise<void> {
    if (m[0] === 0) {
      try {
        this.send([1, "3dx_rpc", `wss://${NL}/3dconnexion#`]);
        this.send([1, "3dconnexion", `wss://${NL}/3dconnexion`]);
        this.send([1, "self", "meshstep://local"]);
        const cx = (await this.call("3dx_rpc:create", "3dconnexion:3dmouse", "0.8.1")).connexion;
        const inst = (
          await this.call("3dx_rpc:create", "3dconnexion:3dcontroller", cx, { version: 0.8, name: "meshStep", rowMajorOrder: false })
        ).instance;
        this.ctrl = "3dconnexion:3dcontroller/" + inst;
        this.send([5, this.ctrl]);
        await this.update({ focus: document.hasFocus() });
        await this.update({ frame: { timingSource: 1 } });
        this.setStatus("connected through the 3Dconnexion software");
      } catch (err) {
        this.setStatus("3Dconnexion software refused: " + (err as Error).message);
      }
    } else if (m[0] === 3 || m[0] === 4) {
      const c = this.calls.get(m[1]);
      this.calls.delete(m[1]);
      if (c) (m[0] === 3 ? c.res(m[2]) : c.rej(new Error(`${m[2]} ${m[3]}`)));
    } else if (m[0] === 8 && this.ws === ws) {
      this.onEvent(m[2]);
    }
  }

  // ---- properties the 3Dconnexion software reads ----
  private frustum(): [number, number, number, number, number, number] {
    const c = this.host.camera();
    if (c instanceof THREE.OrthographicCamera) {
      const z = c.zoom;
      return [c.left / z, c.right / z, c.bottom / z, c.top / z, c.near, c.far];
    }
    const n = c.near;
    const t = n * Math.tan((c.fov * Math.PI) / 360);
    const r = t * c.aspect;
    return [-r, r, -t, t, n, c.far];
  }

  private read(prop: string): unknown {
    const h = this.host;
    const c = h.camera();
    switch (prop) {
      case "view.affine":
        c.updateMatrixWorld();
        return c.matrixWorld.toArray();
      case "view.perspective":
        return !h.isOrtho();
      case "view.fov":
        return c instanceof THREE.PerspectiveCamera ? (c.fov * Math.PI) / 180 : 0;
      case "view.frustum":
        return this.frustum();
      case "view.extents": {
        const f = this.frustum(); // [l, r, b, t, n, f] -> [l, b, -far, r, t, -near]
        return [f[0], f[2], -f[5], f[1], f[3], -f[4]];
      }
      case "view.target":
        return h.target().toArray();
      case "view.rotatable":
        return true;
      case "view.constructionPlane":
        return null;
      case "view.focusDistance":
        return c.position.distanceTo(h.target());
      case "model.extents": {
        const b = h.modelBox();
        return b ? [b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z] : [-1, -1, -1, 1, 1, 1];
      }
      case "model.floorPlane":
        return null;
      case "model.unitsToMeters":
        return 0.001; // STEP is tessellated in millimetres
      case "pivot.position":
        return (this.pivotSet ? this.pivot : h.target()).toArray();
      case "selection.empty":
        return true;
      case "selection.extents":
      case "selection.affine":
        return null;
      case "pointer.position":
        return this.ptr ? new THREE.Vector3(this.ptr.x, this.ptr.y, -1).unproject(c).toArray() : null;
      case "coordinateSystem": // Z up
        return [1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1];
      case "views.front": // looking along +y
        return [1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1];
      case "frame.timingSource":
        return 1;
      case "frame.time":
        return performance.now();
      case "hit.lookat": {
        this.ray.set(this.hitFrom, this.hitDir);
        return h.hit(this.ray)?.toArray() ?? null;
      }
    }
    throw new Error(prop + " unknown property");
  }

  /** After a move, put the mouse orbit centre on the new view axis at the depth of the pivot it reports;
   *  the camera itself stays exactly as the 3Dconnexion software left it. */
  private retarget(): void {
    const c = this.host.camera();
    const d = new THREE.Vector3(0, 0, -1).applyQuaternion(c.quaternion);
    const from = this.pivotSet ? this.pivot : this.host.target();
    const t = Math.max(1e-3, from.clone().sub(c.position).dot(d));
    this.host.setTarget(c.position.clone().addScaledVector(d, t));
    this.host.onCameraWritten();
  }

  private write(prop: string, v: any): void {
    const h = this.host;
    const c = h.camera();
    switch (prop) {
      case "motion":
        if (!v) this.retarget();
        return;
      case "view.affine": {
        const m = new THREE.Matrix4().fromArray(v);
        c.position.setFromMatrixPosition(m);
        c.quaternion.setFromRotationMatrix(m);
        c.up.set(0, 1, 0).applyQuaternion(c.quaternion);
        this.retarget();
        return;
      }
      case "view.fov":
        if (!h.isOrtho()) h.setFov((v * 180) / Math.PI);
        return;
      case "view.extents":
        // Orthographic zoom arrives as new view extents [l, b, -far, r, t, -near].
        if (h.isOrtho() && Array.isArray(v) && v[4] > v[1]) h.setOrthoHeight(v[4] - v[1]);
        return;
      case "view.target":
      case "pivot.position":
        this.pivot.fromArray(v);
        this.pivotSet = true;
        return;
      case "hit.lookfrom":
        this.hitFrom.fromArray(v);
        return;
      case "hit.direction":
        this.hitDir.fromArray(v).normalize();
        return;
      default:
        return; // pivot.visible, hit.aperture, transaction, selection.affine, commands…, settings.changed
    }
  }

  private onEvent(d: any[]): void {
    if (!Array.isArray(d) || d[0] !== 2) return;
    const id = d[1];
    const proc = d[2];
    const prop = d[4];
    try {
      if (proc === "self:read") {
        const r = this.read(prop);
        this.send([3, id, r === undefined ? null : r]);
      } else if (proc === "self:update") {
        this.write(prop, d[5]);
        this.send([3, id, null]);
      } else {
        this.send([4, id, proc + "#generic", prop + " unknown property"]);
      }
    } catch (e) {
      this.send([4, id, proc + "#generic", String(e)]);
    }
  }
}
