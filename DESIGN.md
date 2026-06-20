# meshStep — Design

A standalone **STEP → uniform-isotropic mesh** importer, pure TypeScript, zero runtime
dependencies. Built to feed two CNC Kitchen tools:

- **bumpmesh** (`stlTexturizer`, JS/Three.js) — surface-displacement texturing; wants dense,
  even, isotropic meshes so displacement reads smoothly.
- **infeall** (`smartInfillGenerator`, Rust+WASM) — winding-number voxelization + infill; wants a
  watertight solid plus **CAD face topology** to seed segmentation.

License: **AGPL-3.0-only** (matching both consumer tools), dual-licensable commercially since the
copyright is held in-house.

---

## 1. Locked decisions (from the grilling session)

| Dimension | Decision | Rationale |
|---|---|---|
| Deliverable | In-browser library, embeddable in bumpmesh + infeall | Users load STEP directly in the tools, not an external converter |
| Engine | **From-scratch pure TypeScript**, zero runtime deps | User choice; standalone, no toolchain friction, works directly in bumpmesh's JS. Note: this is a *parallel* path to infeall's Rust/truck importer, not a replacement for it |
| Mesh goal | **Uniform isotropic** — ~constant edge length, near-equilateral, low slivers | Even displacement (bumpmesh) + clean voxelization (infeall) |
| Topology | Watertight; multi-body kept as **disjoint welded components** | Both tools assume sealed surfaces; infeall needs per-body identity |
| Scope v1 | **Analytic surfaces only**: plane, cylinder, cone, sphere, torus | Covers 6/8 test files; the parameter-space re-tessellation is exactly infeall's documented DESIGN §9 fix |
| B-splines | **Skipped in v1** — detected and errored cleanly | NURBS surface/curve parsing + evaluation is the one hard part; deferred to phase 2 |
| Controls | Refinement preset (Low/Med/High/Custom) + Surface Deviation, Normal Deviation, Max Edge Length, Aspect Ratio | Reproduce Fusion's dialog, remapped to isotropic semantics (see §4) |
| Budget | Interactive — meshing in a Web Worker, hard triangle cap | Embedded web tool must not freeze the tab |
| Acceptance | Symmetric **Hausdorff** vs reference STL + **spiral-edge %** ≈ 0 + watertight + sliver metrics | Automatable over the test pairs |
| Output | Binary/ASCII **STL** (mm) + **3MF** for true multi-body; plus in-memory mesh + `faceOfTri` + per-face analytic metadata + boundary polylines | STL/3MF for the tools; metadata for infeall segmentation |

## 2. Why parameter-space analytic tessellation (the core idea)

This mirrors the fix documented in infeall's `DESIGN.md §9`. A generic BREP tessellator
(truck's `robust_triangulation`) can **twist trimmed periodic faces** — e.g. a
cylinder-with-cutout came out with 13% of edges spiralling up to 177°, vs 0% in the CAD's own
STL. The robust fix is to mesh each *analytic* face in its **natural (u,v) parameter space**,
where the surface is flat and trivially triangulated, then map back to 3D via the exact
analytic surface equation. No twisting is possible because we control the parameter grid.

The big advantage over remeshing an STL: we get **exact sharp-feature edges for free** from the
STEP `EDGE_CURVE` set, so the isotropic remesher keeps real edges crisp (a cube stays a cube)
while uniformly filling faces.

## 3. Pipeline

```
STEP text
  → tokenizer (ISO-10303-21 lexer)
  → parser (entity instance graph, #id resolution)
  → units (→ millimetres)
  → BREP builder      Solid ▸ Shell ▸ Face { analytic surface, boundary loops of edges }
  → tessellate        per face, in (u,v); SHARED edges sampled once → watertight seams
  → isotropic remesh  split / collapse / flip / tangential-smooth; feature edges from BREP edges
  → outputs           STL · 3MF · { positions, indices, faceOfTri, faces[], boundaryPolylines }
+ acceptance harness  Hausdorff & spiral-edge % vs reference *.stl
```

## 4. Control semantics (uniform-isotropic remap of Fusion's dialog)

Fusion's controls assume *curvature-adaptive* meshing. Under uniform isotropic meshing they remap:

- **Maximum Edge Length** → the **target edge length** (the dominant knob). In the screenshot
  Fusion's value `34.64102 mm` for a 20 mm cube is exactly the bbox diagonal `20·√3`, i.e. "no
  cap"; we instead treat it as the real target.
- **Surface Deviation** → max chord error; caps edge length in high-curvature regions so curved
  faces get finer than the global target where needed.
- **Normal Deviation** → max angle between adjacent face normals; a second curvature cap.
- **Aspect Ratio** → the remesher's quality ceiling. Uniform output wants ~1.0–1.5; Fusion's
  `21.5` is just a loose upper bound. We drive extra collapse/flip passes on triangles above it.
- **Refinement preset** scales the above relative to the part's bbox diagonal `D` (see `presets.ts`).

## 5. Module layout

```
src/
  geom/    vec, surfaces (plane/cylinder/cone/sphere/torus eval+normal+project), curves (line/circle/ellipse)
  step/    tokenizer, parser, units, entities
  brep/    types, build
  mesh/    mesh (indexed + half-edge ops), tessellate, remesh, quality
  io/      stl, threemf, zip (minimal stored zip, zero-dep)
  presets, index (public API)
test/      hausdorff, acceptance, inspect
bin/       cli (step → stl/3mf for offline testing)
worker/    meshWorker (Web Worker wrapper)
demo/      index.html (the slider panel + Three.js viewer)
```

## 6. Test files (in repo root)

| File | Surfaces | v1 |
|---|---|---|
| cube | planes | ✅ |
| cylinder | plane + cylinder | ✅ |
| cone | plane + cone | ✅ |
| sphere | sphere | ✅ |
| cylinderWithHole | plane + cylinder (periodic, the twist case) | ✅ |
| roundedCube | plane + cylinder + sphere + torus (fillets) | ✅ |
| splineThing | B-spline surfaces | ⛔ phase 2 |
| everything | all of the above, multi-body | partial (analytic bodies) |

## 7. Runtime / tooling

Library is **erasable TypeScript** (types only, no enums/namespaces/decorators) so Node's native
type-stripping runs it directly — `node test/acceptance.ts`, no build, no install. Shipping to the
tools is a later esbuild/tsc bundle step. Relative imports use explicit `.ts` extensions.
