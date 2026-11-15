// SPDX-License-Identifier: AGPL-3.0-only
// One adapter per STEP -> mesh converter. Every adapter does the same job: read the STEP bytes,
// mesh at the given chordal deviation (mm) and angular deviation (degrees), and hand back ONE
// triangle mesh in millimetres and world coordinates (assembly placements applied), exactly as the
// library delivers it (no welding, no clean-up: the metrics do that the same way for everyone).
//
// All of these run in the browser; the benchmark runs them in Node because Node is the same V8 and
// the same WebAssembly engine as Chrome, and lets every run sit in its own process.
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const require = createRequire(import.meta.url);
const here = dirname(fileURLToPath(import.meta.url));
const pkgVersion = (name: string): string =>
  JSON.parse(readFileSync(join(here, "node_modules", name, "package.json"), "utf8")).version;

export interface MeshParams {
  /** Chordal (linear) deviation, mm. */
  deflection: number;
  /** Angular deviation, degrees. */
  angleDeg: number;
  /** Bounding-box diagonal of the model, mm (meshStep derives its max edge from it). */
  diag: number;
}

export interface MeshOut {
  positions: Float32Array | Float64Array;
  indices: Uint32Array;
  /** Bodies / shapes the library reported (for information only). */
  parts: number;
  /** Faces the library reported meshing (for information only). */
  faces?: number;
  /** Body (solid / part occurrence) of every triangle: watertightness is judged per body, since
   *  touching bodies of an assembly legitimately share edges. */
  bodyOfTri: Uint32Array;
}

export interface Adapter {
  id: string;
  label: string;
  version: string;
  /** What the library is built on. */
  engine: string;
  init(): Promise<void>;
  mesh(bytes: Uint8Array, p: MeshParams): Promise<MeshOut> | MeshOut;
}

/** Concatenate several meshes into one. */
/** Concatenate several meshes into one; each part becomes one body. */
function concat(parts: { positions: ArrayLike<number>; indices: ArrayLike<number> }[]): { positions: Float64Array; indices: Uint32Array; bodyOfTri: Uint32Array } {
  let nv = 0, ni = 0;
  for (const m of parts) { nv += m.positions.length; ni += m.indices.length; }
  const positions = new Float64Array(nv), indices = new Uint32Array(ni), bodyOfTri = new Uint32Array(ni / 3);
  let vo = 0, io = 0;
  parts.forEach((m, b) => {
    positions.set(m.positions as ArrayLike<number>, vo);
    const base = vo / 3;
    for (let i = 0; i < m.indices.length; i++) indices[io + i] = m.indices[i]! + base;
    bodyOfTri.fill(b, io / 3, (io + m.indices.length) / 3);
    vo += m.positions.length; io += m.indices.length;
  });
  return { positions, indices, bodyOfTri };
}

/** Column-major 4x4 (three.js / Babylon order) times a point. */
function applyMat(m: number[], P: ArrayLike<number>): Float64Array {
  const out = new Float64Array(P.length);
  for (let i = 0; i < P.length; i += 3) {
    const x = P[i]!, y = P[i + 1]!, z = P[i + 2]!;
    out[i] = m[0]! * x + m[4]! * y + m[8]! * z + m[12]!;
    out[i + 1] = m[1]! * x + m[5]! * y + m[9]! * z + m[13]!;
    out[i + 2] = m[2]! * x + m[6]! * y + m[10]! * z + m[14]!;
  }
  return out;
}
const mul4 = (a: number[], b: number[]): number[] => {
  const r = new Array<number>(16).fill(0);
  for (let c = 0; c < 4; c++) for (let rr = 0; rr < 4; rr++)
    for (let k = 0; k < 4; k++) r[c * 4 + rr]! += a[k * 4 + rr]! * b[c * 4 + k]!;
  return r;
};
const IDENT = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

// ---------------------------------------------------------------- meshStep

type MeshStepMod = typeof import("../src/index.ts");
function meshStepAdapter(id: string, label: string, opts: (p: MeshParams) => Record<string, unknown>, parallel = false): Adapter {
  let ms: MeshStepMod;
  return {
    id, label, engine: "pure TypeScript",
    version: JSON.parse(readFileSync(join(here, "..", "package.json"), "utf8")).version,
    async init() { ms = await import("../src/index.ts"); },
    async mesh(bytes, p) {
      const text = new TextDecoder("latin1").decode(bytes);
      const o = { surfaceDeviation: p.deflection, normalDeviation: p.angleDeg, ...opts(p) };
      // parallel: faces meshed on worker threads (Web Workers in the browser), same output
      const r = parallel ? await ms.importStepAsync(text, o) : ms.importStep(text, o);
      return { positions: r.mesh.positions, indices: r.mesh.indices, parts: r.instances.length, faces: r.faces.size, bodyOfTri: r.instanceOfTri };
    },
  };
}

// ---------------------------------------------------------------- occt-import-js

function occtImportJs(): Adapter {
  let occt: any;
  return {
    id: "occt-import-js", label: "occt-import-js", engine: "OCCT 7.8 (WASM)",
    version: pkgVersion("occt-import-js"),
    async init() {
      const f = require("occt-import-js");
      occt = await f({ print: () => {}, printErr: () => {} });
    },
    mesh(bytes, p) {
      const r = occt.ReadStepFile(bytes, {
        linearUnit: "millimeter",
        linearDeflectionType: "absolute_value",
        linearDeflection: p.deflection,
        angularDeflection: p.angleDeg * Math.PI / 180,
      });
      if (!r.success) throw new Error("ReadStepFile failed");
      // meshes are already in world coordinates (the node tree only groups them)
      const parts = (r.meshes ?? []).map((m: any) => ({ positions: m.attributes.position.array, indices: m.index.array }));
      let faces = 0; for (const m of r.meshes ?? []) faces += m.brep_faces?.length ?? 0;
      return { ...concat(parts), parts: parts.length, faces };
    },
  };
}

// ---------------------------------------------------------------- @tx-code/occt-js

function txOcctJs(): Adapter {
  let occt: any;
  return {
    id: "tx-occt-js", label: "@tx-code/occt-js", engine: "OCCT 7.9.3 (WASM)",
    version: pkgVersion("@tx-code/occt-js"),
    async init() {
      const dir = join(here, "node_modules", "@tx-code", "occt-js", "dist");
      const mod = await import(pathToFileURL(join(dir, "occt-js.js")).href);
      occt = await mod.default({ wasmBinary: readFileSync(join(dir, "occt-js.wasm")), print: () => {}, printErr: () => {} });
    },
    mesh(bytes, p) {
      const r = occt.ReadStepFile(bytes, {
        linearUnit: "millimeter",
        linearDeflectionType: "absolute_value",
        linearDeflection: p.deflection,
        angularDeflection: p.angleDeg * Math.PI / 180,
        readNames: false, readColors: false,
      });
      if (!r.success) throw new Error(r.error ?? "ReadStepFile failed");
      // geometries are instanced through the node tree: walk it and place every occurrence
      const parts: { positions: Float64Array; indices: Uint32Array }[] = [];
      let faces = 0;
      const walk = (n: any, parent: number[]): void => {
        const m = n.transform?.length === 16 ? mul4(parent, n.transform) : parent;
        for (const gi of n.meshes ?? []) {
          const g = r.geometries[gi];
          parts.push({ positions: applyMat(m, g.positions), indices: g.indices });
          faces += g.faces?.length ?? 0;
        }
        for (const c of n.children ?? []) walk(c, m);
      };
      for (const n of r.rootNodes ?? []) walk(n, IDENT);
      return { ...concat(parts), parts: parts.length, faces };
    },
  };
}

// ---------------------------------------------------------------- occt-wasm

function occtWasm(): Adapter {
  let kernel: any;
  return {
    id: "occt-wasm", label: "occt-wasm", engine: "OCCT 8 (WASM)",
    version: pkgVersion("occt-wasm"),
    async init() {
      const { OcctKernel } = await import("occt-wasm");
      kernel = await OcctKernel.init();
    },
    mesh(bytes, p) {
      const shape = kernel.importStep(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
      const opt = { linearDeflection: p.deflection, angularDeflection: p.angleDeg * Math.PI / 180 };
      // one tessellation per solid (to know the body of each triangle); the whole shape at once
      // when it has loose faces outside any solid
      const solids = kernel.getSubShapes(shape, "solid");
      const nFaces = kernel.getSubShapes(shape, "face").length;
      let inSolids = 0;
      for (const s of solids) inSolids += kernel.getSubShapes(s, "face").length;
      const shapes = solids.length && inSolids === nFaces ? solids : [shape];
      const parts = shapes.map((s: unknown) => { const m = kernel.tessellate(s, opt); return { positions: m.positions, indices: m.indices }; });
      const out = { ...concat(parts), parts: parts.length, faces: nFaces };
      kernel.release?.(shape);
      return out;
    },
  };
}

// ---------------------------------------------------------------- opencascade.js

function opencascadeJs(): Adapter {
  let oc: any;
  return {
    id: "opencascade.js", label: "opencascade.js", engine: "OCCT 7.5 (WASM)",
    version: pkgVersion("opencascade.js"),
    async init() {
      const dir = join(here, "node_modules", "opencascade.js", "dist");
      // the Emscripten glue is an ES module that still expects CommonJS globals under Node
      Object.assign(globalThis, { __dirname: dir, __filename: join(dir, "opencascade.wasm.js"), require });
      const mod = await import(pathToFileURL(join(dir, "opencascade.wasm.js")).href);
      oc = await new mod.default({ wasmBinary: readFileSync(join(dir, "opencascade.wasm.wasm")), print: () => {}, printErr: () => {} });
    },
    mesh(bytes, p) {
      oc.FS.writeFile("/model.step", bytes);
      const reader = new oc.STEPControl_Reader_1();
      // (an absolute path fails to open in this build; the working directory is "/")
      if (reader.ReadFile("model.step").value !== oc.IFSelect_ReturnStatus.IFSelect_RetDone.value) throw new Error("ReadFile failed");
      reader.TransferRoots();
      const shape = reader.OneShape();
      new oc.BRepMesh_IncrementalMesh_2(shape, p.deflection, false, p.angleDeg * Math.PI / 180, false);
      const pos: number[] = [], idx: number[] = [], body: number[] = [];
      let faces = 0, nBody = 0;
      // the faces of every solid, then the loose faces outside any solid as one more body
      const groups: { root: any; avoid: any }[] = [];
      const S = oc.TopAbs_ShapeEnum;
      for (const ex = new oc.TopExp_Explorer_2(shape, S.TopAbs_SOLID, S.TopAbs_SHAPE); ex.More(); ex.Next()) groups.push({ root: ex.Current(), avoid: S.TopAbs_SHAPE });
      groups.push({ root: shape, avoid: S.TopAbs_SOLID });
      for (const g of groups) {
      const bodyId = nBody++;
      const exp = new oc.TopExp_Explorer_2(g.root, S.TopAbs_FACE, g.avoid);
      for (; exp.More(); exp.Next()) {
        const face = oc.TopoDS.Face_1(exp.Current());
        const loc = new oc.TopLoc_Location_1();
        const h = oc.BRep_Tool.Triangulation(face, loc);
        if (h.IsNull()) continue;
        faces++;
        const tri = h.get();
        const trsf = loc.Transformation();
        const base = pos.length / 3;
        const nodes = tri.Nodes();
        for (let i = 1; i <= tri.NbNodes(); i++) {
          const q = nodes.Value(i).Transformed(trsf);
          pos.push(q.X(), q.Y(), q.Z());
        }
        const rev = face.Orientation_1() === oc.TopAbs_Orientation.TopAbs_REVERSED;
        const tris = tri.Triangles();
        for (let i = 1; i <= tri.NbTriangles(); i++) {
          const t = tris.Value(i);
          const a = t.Value(1) - 1, b = t.Value(2) - 1, c = t.Value(3) - 1;
          if (rev) idx.push(base + a, base + c, base + b); else idx.push(base + a, base + b, base + c);
          body.push(bodyId);
        }
      }
      }
      oc.FS.unlink("/model.step");
      return { positions: new Float64Array(pos), indices: new Uint32Array(idx), parts: nBody - 1, faces, bodyOfTri: new Uint32Array(body) };
    },
  };
}

// ---------------------------------------------------------------- brepkit-wasm

function brepkit(): Adapter {
  let k: any;
  return {
    id: "brepkit-wasm", label: "brepkit-wasm", engine: "brepkit (Rust, WASM)",
    version: pkgVersion("brepkit-wasm"),
    async init() {
      const { BrepKernel } = require("brepkit-wasm");
      k = new BrepKernel();
    },
    mesh(bytes, p) {
      const solids: Uint32Array = k.importStep(bytes);
      const parts = [];
      for (const s of solids) {
        const m = k.tessellateSolid(s, p.deflection, p.angleDeg * Math.PI / 180);
        parts.push({ positions: m.positions.slice(), indices: m.indices.slice() });
        m.free?.();
      }
      return { ...concat(parts), parts: parts.length };
    },
  };
}

// ---------------------------------------------------------------- registry

export const ADAPTERS: Record<string, () => Adapter> = {
  // meshStep as the web viewer runs it: max edge 1% of the model diagonal (even triangles)
  "meshstep": () => meshStepAdapter("meshstep", "meshStep", p => ({ maxEdge: p.diag * 1e-2 })),
  // meshStep in its FE-style isotropic mode
  "meshstep-iso": () => meshStepAdapter("meshstep-iso", "meshStep isotropic", p => ({ maxEdge: p.diag * 1e-2, isotropic: true })),
  // meshStep driven by deviation only, like OCCT (no max edge): the like-for-like speed row
  "meshstep-dev": () => meshStepAdapter("meshstep-dev", "meshStep (deviation only)", p => ({ maxEdge: p.diag })),
  // the viewer's settings again, through importStepAsync (worker threads, library default count)
  "meshstep-par": () => meshStepAdapter("meshstep-par", "meshStep (workers)", p => ({ maxEdge: p.diag * 1e-2 }), true),
  "occt-import-js": occtImportJs,
  "tx-occt-js": txOcctJs,
  "occt-wasm": occtWasm,
  "opencascade.js": opencascadeJs,
  "brepkit-wasm": brepkit,
};

/** The fine reference meshes the accuracy metrics compare against: one from meshStep, one from
 *  OCCT (the second OCCT build steps in when the first fails on a model). */
export const REFERENCES: string[][] = [["meshstep-dev"], ["occt-import-js", "tx-occt-js"]];
