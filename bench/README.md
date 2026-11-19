# Benchmark: STEP → mesh in the browser

This benchmark compares meshStep with the other libraries that convert STEP files to triangle meshes inside a web page. Each library meshes the same files at the same tolerance, and the output of each is measured the same way.

Results are in [RESULTS.md](RESULTS.md).

## Contenders

| Library | What it is |
|---|---|
| **meshStep** | Pure TypeScript, no WebAssembly. It runs three ways here: as the web viewer runs it (max edge 1% of the model size), in isotropic mode, and with deviation only (no max edge). The last one gives about OCCT's triangle count, so the speed comparison is like for like. |
| [occt-import-js](https://github.com/kovacsv/occt-import-js) | OpenCASCADE compiled to WASM. Powers [3dviewer.net](https://3dviewer.net). This is the de facto standard, and every ratio is measured against it. |
| [@tx-code/occt-js](https://www.npmjs.com/package/@tx-code/occt-js) | Newer OCCT 7.9.3 build with an XDE scene graph. |
| [occt-wasm](https://github.com/andymai/occt-wasm) | OCCT 8 built with SIMD, with a TypeScript API. |
| [opencascade.js](https://github.com/donalffons/opencascade.js) | Full OCCT 7.5 kernel bindings. We mesh with `BRepMesh_IncrementalMesh` ourselves. |
| [brepkit-wasm](https://github.com/andymai/brepkit) | Independent B-rep kernel in Rust, compiled to WASM. It is the only non-OCCT competitor. |

Not included:
- truck: it has no npm package.
- Commercial SDKs.
- Server-side converters: the file leaves the browser, so they solve a different problem.

## KPIs

| KPI | Definition | Why it matters |
|---|---|---|
| **Converted** | The run produced a mesh, at the right scale, with no more than 1% of the true surface missing. | Did the user get their part at all? |
| **Watertight** | Closed and manifold per body: no open edges, no non-manifold edges, consistent orientation. Coincident vertices are merged within 1e-6 × model diagonal first, a pass any consumer can do. Bodies are never merged into each other, because touching parts of an assembly may share edges. Counted only over the models whose STEP file describes closed solids (see below). | Slicing, booleans, voxelizing and FE all need it. |
| **Flipped areas** | Models where more than 0.1% of the mesh area faces against the surface it lies on. | Inverted patches render dark and break inside/outside tests. |
| **Missing surface** | Models where more than 1% of the true surface is farther than 5 × tolerance from the mesh. | Detects faces the mesher silently skipped. |
| **Time, first file** | Module load, WebAssembly compile and the first conversion, in a fresh process. | What a user waits for after opening the page and dropping a file. |
| **Time, same file again** | Best of two more conversions in the same process. JIT is warm and WASM is compiled. | Throughput once the page is warm. |
| **Peak memory** | Peak resident set size of the process (median). | WASM heaps never shrink, and phones have little memory. |
| **Download size** | JavaScript bundled and minified with esbuild, plus WebAssembly as shipped, compressed with brotli. | Page weight and time to first interaction. |
| **Triangles** | Triangle count; the ratio is against occt-import-js. | Cost downstream (memory, GPU, slicing). |
| **Triangle shape** | Smallest angle of each triangle: 5th percentile, share under 20° and under 5°, and mean radius ratio (1 = equilateral). | Needles and slivers hurt shading, displacement and FE, and are a sign of a fragile mesh. |
| **Deviation** | Area-uniform samples on the mesh (20,000 per model), measured to the true surface, as a multiple of the requested tolerance: 99th percentile, and the share of models whose maximum stays within 1.25 × tolerance. | Does the library keep the accuracy it was asked for? |

### Which models each KPI covers

- **Robustness columns:** every model, except watertight. A converter can only be asked for a watertight mesh where the file describes one. [topology.ts](topology.ts) reads that from the STEP file's B-rep topology, without any converter. A model counts as closed when every shell of its solids and surface models uses each edge exactly twice, in opposite directions. AP242 tessellated bodies are checked the same way on their triangles. Files with surface bodies (`OPEN_SHELL`), bare trimmed surfaces without topology, or free or non-manifold edges in a shell drop out of the watertight count. RESULTS.md marks them in the per-model table. Run `node topology.ts <file.step>` to see the verdict for one file.
- **Speed, triangle and shape columns:** only the models that every library converted. Otherwise a library that fails on the hard models would look fast.

## Fairness rules

**Same tolerance for everyone.** Each model gets these settings, scaled to its bounding-box diagonal:
- Fine tier: chord tolerance 1e-4 × diagonal and 15° angle. This is what the meshStep viewer picks.
- Coarse tier (`--tier coarse`): 1e-3 × diagonal and 30°, a typical setting for viewing on screen.

**Output used as delivered.** No welding, healing or re-orientation inside the adapters. Assembly placements are applied.

**The "true surface" comes from two independent references.** Each model is also meshed at 1/5 of the tolerance by meshStep and by OCCT (occt-import-js, with @tx-code/occt-js as the fallback). Distances are taken to the nearer of the two. So:
- A library is not punished where the two kernels read the geometry differently.
- The reference error is at most 0.2 × tolerance. That is why 1.25 × tolerance counts as "within tolerance".
- When OCCT cannot read a model, only the meshStep reference is left. Deviation for that model is then measured against meshStep. RESULTS.md lists the references used per run in the raw data.

**One process per run.** Every conversion starts in a fresh `node` process, like a fresh browser tab:
- Each run has a timeout of 120 s.
- The libraries of one model run back to back in the same slot, so they see the same machine load. Their order rotates per model.

**Same engine as the browser.** Node is the same V8 and WebAssembly engine as Chrome, so the numbers transfer. Absolute times depend on the machine. The ratios are the part to compare.

## Running it

```sh
cd bench
npm install                               # the competitors live only here, not in meshStep's dependencies
node size.ts                              # download sizes -> out/size.json
node run.ts --corpus char --jobs 4        # the test corpus -> out/char-fine.jsonl (resumable)
node run.ts --corpus abc:300              # 300 ABC-dataset models (needs abc_0002_step_v00/)
# idle-machine speed: one conversion at a time, incl. meshStep on worker threads (it needs the cores)
node run.ts --jobs 1 --adapters meshstep,meshstep-par,occt-import-js --out out/solo-fine.jsonl
node report.ts                            # -> RESULTS.md and ../docs/images/bench.svg
```

`node one.ts run <adapter> <file.step> '{"deflection":0.01,"angleDeg":15,"diag":100}'` runs a single conversion and prints its JSON.

## Known limits

- **Peak memory** is the whole process. That includes Node itself, about 40 MB, and garbage that V8 has not collected yet. Read it as a rough size class.
- **Thin or touching geometry:** where a model has walls thinner than the tolerance, or touching bodies, deviation and flip tests use the nearest reference triangle. That triangle may belong to the other side. Flips are only counted when no reference triangle within the same distance agrees with the triangle's orientation.
- **OCCT decides the model's extent.** The model size (and with it the tolerance) comes from a coarse occt-import-js pass, or from meshStep when OCCT cannot read the file. A run counts as "wrong scale" when its extent differs from that by more than 1%. That catches unit and assembly-placement bugs. But where the readers disagree about the assembly, OCCT's XDE reading wins. An example is `jack_in_the_box`: its parts are placed by a frame stored in each part's own representation. meshStep and the XDE-based readers apply that frame, but opencascade.js's plain `STEPControl_Reader` does not, so its parts land in the wrong place.
- **The "missing" test takes the more lenient reference.** A library that drops the same part as one of the references is not flagged by it. The extent check above is the backstop for that case.
- **Few libraries can read some models.** A model only one or two libraries can read drops out of the speed and shape columns entirely. Its failures still show in the robustness table.
