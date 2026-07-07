# meshStep

Pure-TypeScript **STEP → mesh** importer. Reads ISO-10303-21 B-rep geometry (AP203 / AP214 /
AP242) and produces **watertight, low-sliver, process-grade triangle meshes** — meshes you can
displace, voxelize, offset, slice, or simulate, not just look at.

Zero runtime dependencies: no WASM, no native code, no build step. The whole library is ~6,500
lines of TypeScript (~110 KB minified, ~42 KB gzipped) that run as-is in the browser, in a Web
Worker, and in Node ≥ 22. Built to feed
[bumpmesh](https://github.com/CNCKitchen/stlTexturizer) (displacement texturing) and
[infeall](https://github.com/CNCKitchen/smartInfillGenerator) (smart infill), where mesh defects
aren't cosmetic — they break the algorithm.

## Why another STEP importer?

Because every existing route from "STEP file" to "triangles" optimizes for **rendering**, and
rendering tolerates broken meshes.

Whatever converts your STEP file today — a CAD program's STL export, an
[OpenCASCADE](https://dev.opencascade.org/)-based converter, an online tool — almost certainly
tessellates each B-rep face independently, with curvature-adaptive spacing tuned for screen-space
smoothness. That looks right in a viewer, but structurally it's a per-face triangle soup:
duplicated vertices along every face border, hairline cracks between faces, extreme sliver
triangles on cylinders and fillets, and occasionally faces missing outright (the
[Better STEP](https://arxiv.org/abs/2506.05417) dataset paper measured OpenCASCADE's mesher
leaving faces unmeshed in ~1.5% of ABC-dataset models and ~9% of their assembly set). Algorithms
that *consume* the mesh — displacement mapping, winding-number voxelization, Boolean/offset
operations, slicing, FEM — choke on exactly those defects.

meshStep starts from the opposite end: **the mesh is the product.**

- **Watertight per body, by construction.** Shared B-rep edges are sampled once and welded, so
  every body imports as a closed, consistently outward-oriented 2-manifold — verified as 0 open
  edges / 0 non-manifold edges across the curated test corpus (see [Status](#status) for honest
  numbers on wilder corpora). Multi-body parts and assemblies stay separate welded components.
- **No seam twist, no sliver storm.** Every face is triangulated in its natural (u,v) parameter
  space with a robust constrained-Delaunay pass; periodic faces (cylinder/cone/sphere/torus
  seams) are unwrapped flat first, so the spiral "twist" that plagues generic B-rep tessellators
  cannot occur. Fillets get ruling-aligned anisotropy and normal-smoothing diagonal flips for
  clean shading.
- **CAD topology survives into the mesh.** `faceOfTri` / `solidOfTri` map every triangle back to
  its STEP face and body, and feature edges are exact from the B-rep `EDGE_CURVE` set — a cube
  stays a cube. Downstream tools can segment and mask by real CAD faces instead of re-detecting
  features from a triangle soup.
- **The true surface stays available.** The optional isotropic remesh (`remesh: true`)
  splits/collapses/flips/smooths toward uniform, near-equilateral triangles and reprojects every
  vertex onto the exact analytic/NURBS surface (feature edges frozen) — quality passes converge
  to the CAD geometry, not to a frozen first tessellation.
- **Cross-validated against OpenCASCADE, not just eyeballed.** The `gapcheck` harness converts
  every corpus model with both meshStep and OCCT and compares global + per-face deviation:
  NIST test models, real-world Printables downloads, and a ~1,800-part Voron-assembly sweep.

## How it compares

| Route | In browser | Footprint | Mesh output | Watertight welded | Face→tri topology | License |
|---|---|---|---|---|---|---|
| **meshStep** | ✅ pure TS | ~42 KB gzip | uniform, low-sliver, seam-safe | ✅ per welded body | ✅ | AGPL-3.0 ([commercial](COMMERCIAL.md)) |
| [occt-import-js](https://github.com/kovacsv/occt-import-js) | ✅ WASM | ~8 MB WASM | curvature-adaptive, per face | ❌ | ✅ face ranges | LGPL-2.1 (OCCT) |
| [opencascade.js](https://github.com/donalffons/opencascade.js) | ✅ WASM | larger (custom builds) | same OCCT mesher | ❌ | manual | LGPL-2.1 (OCCT) |
| [cascadio](https://github.com/trimesh/cascadio) / pythonOCC / FreeCAD | ❌ Python/desktop | native wheels/app | same OCCT mesher | ❌ | varies | LGPL (bundles OCCT) |
| [gmsh](https://gmsh.info) | ❌ desktop/Python | native | FE-quality, isotropic | ✅ (when it succeeds) | ⚠️ | GPL-2.0+ |
| [truck](https://github.com/ricosjp/truck) / [foxtrot](https://github.com/Formlabs/foxtrot) (Rust) | ✅ via WASM | MB-scale WASM | generic; twist on periodic faces | ⚠️ | ❌ | Apache-2.0 / MIT |
| STL export from your CAD app | ❌ manual step | — | curvature-adaptive, sliver-heavy | usually | ❌ | proprietary app |
| Commercial SDKs (CAD Exchanger, HOOPS) | server/native | licensed | good | ✅ | ✅ | proprietary, per-seat/app |

License fine print: the OCCT-based routes statically bundle an LGPL-2.1 kernel — workable, but
with linking/relinking obligations that get murky in WASM; gmsh's GPL rules out closed-source
embedding entirely. meshStep is AGPL-3.0 for open-source use, and because the copyright is held
by a single owner with zero third-party code bundled, a clean commercial exception for
closed-source products is available directly — see [COMMERCIAL.md](COMMERCIAL.md).

Honest notes on the alternatives:

- **[occt-import-js](https://github.com/kovacsv/occt-import-js)** is excellent for what it's
  built for — viewing (it powers [Online 3D Viewer](https://3dviewer.net)), and it reads IGES and
  BREP too. Its output is three.js-style geometry tessellated per face by OCCT's
  curvature-adaptive `BRepMesh`; welding, watertightness, and triangle quality are simply not its
  goals. meshStep uses it as the *reference implementation* in its validation harness.
- **[opencascade.js](https://github.com/donalffons/opencascade.js)** gives you the full OCCT
  kernel in the browser — booleans, filleting, sewing, everything — at the cost of a large custom
  WASM build and its toolchain. If you need a modeling kernel in JS, that's the tool; if you only
  need STEP → clean mesh, it's a lot of freight.
- **[gmsh](https://gmsh.info)** is the serious open-source option for *quality* meshes from STEP
  — but it's a desktop/Python FEA tool, imports geometry through OCCT (so inherits its failure
  modes), and doesn't run in a browser tab.
- **[truck](https://github.com/ricosjp/truck)** is a real Rust B-rep kernel (it powers
  [Loft's browser STEP viewer](https://lofttools.com/blog/open-step-file-in-browser/)), and
  infeall's own importer builds on it. meshStep exists partly because truck's generic
  `robust_triangulation` can twist trimmed periodic faces — on a cylinder-with-cutout test, 13%
  of edges spiralled by up to 177°. [foxtrot](https://github.com/Formlabs/foxtrot) is a fast but
  abandoned proof-of-concept with incomplete NURBS support.
- **Commercial SDKs** ([CAD Exchanger](https://cadexchanger.com),
  [HOOPS Exchange](https://www.techsoft3d.com/products/hoops/exchange/)) are robust, fast, and
  read every format — as native/server-side libraries with per-seat/per-app licensing.

**When meshStep is the wrong tool:** you need formats beyond STEP (IGES, JT, Parasolid), colors /
PMI / metadata, a modeling kernel (booleans, sewing, healing of dirty geometry), or maximum
robustness against pathological enterprise CAD exports. Use an OCCT-based stack or a commercial
SDK for those.

## Status

Coverage:

- **Surfaces:** plane, cylinder, cone, sphere, torus (incl. degenerate), **B-spline / NURBS**
  (rational and complex-form), surface of linear extrusion, surface of revolution, offset surfaces.
- **Curves:** lines, circles, ellipses, B-spline curves (incl. intersection edges of drilled
  holes).
- **Structure:** multi-body parts, assemblies with full product-structure instancing,
  per-representation units, `BREP_WITH_VOIDS`; AP242 tessellated-geometry bodies pass through
  as-is.

Robustness is tracked over four corpora of increasing wildness, cross-validated against
OpenCASCADE output with the `gapcheck` harness (per-face deviation + watertightness). Where it
stands (July 2026):

| Corpus | Result |
|---|---|
| 13 curated repo models (analytic + NURBS + assemblies) | all watertight (0 open, 0 non-manifold), <2% slivers, within ~0.5% of reference |
| 81-model characterization corpus (real Printables downloads + NIST AP242 test cases) | 70 fully clean · 9 with localized open-edge leaks (mostly NIST models) · 2 import errors |
| Voron-family assemblies, 1,820 parts | 1,764 OK · 22 WARN · 33 FAIL · 1 timeout vs OCC cross-check at tight tolerance |
| [ABC dataset](https://deep-geometry.github.io/abc-dataset/) chunk 0000, 1,454 wild CAD files | **142 fully clean (~10%)** — the honesty benchmark, see limitations below |

### Current limitations

meshStep is not (yet) an industrial-strength importer — it's tuned for the kind of parts people
actually 3D-print, and the numbers above are deliberately honest about the rest:

- **Wild CAD is hard.** On the ABC research corpus (uncurated files from many CAD systems, full
  of degenerate, microscopic, and exotic geometry) most models currently come out with defects:
  seam leaks on periodic surfaces, faces the CDT refuses to triangulate, or non-manifold spots.
  The curated corpora show the same failure classes at far lower rates; both shrink with every
  release, but if your files look like ABC, OpenCASCADE-based tools are more robust today.
- **Pathological models can be slow.** Pure TypeScript is fast enough for interactive use on
  print-scale parts, but ~19% of ABC models blew a 120 s budget at tight tolerances — native
  OCCT is faster on huge or degenerate inputs.
- **No geometry healing.** meshStep trusts the STEP file: gaps, self-intersections, or broken
  topology in the source B-rep are not repaired, only reported.
- Remaining features: 3MF export, the controls/preset UI, and residual CDT edge cases on
  pathological multi-loop trims (planar and spherical). Pipeline details in
  [DESIGN.md](DESIGN.md).

## Quick start (dev)

Requires Node ≥ 22 (native TypeScript type-stripping — no build step, no dependencies).

```bash
node test/convert.ts        # import every test STEP, export STL to out/, report quality vs reference
node test/check-all.ts      # watertightness check over the repo test models
```

```ts
import { importStep, writeBinarySTL } from "./src/index.ts";
const result = importStep(stepText, { surfaceDeviation: 0.01, maxEdge: 1.0 });
// result.mesh (positions/indices), result.faceOfTri, result.solidOfTri
if (!result.diagnostics.ok) console.warn(result.diagnostics); // see "Import diagnostics"
writeFileSync("out.stl", writeBinarySTL(result.mesh));
```

Options mirror Fusion's mesh-export dialog: `surfaceDeviation` (mm), `normalDeviation` (deg),
`maxEdge` (mm), plus `remesh: true` for uniform-isotropic output (default off — the raw
tessellation is watertight and shades cleaner).

## Import diagnostics

Every import returns a `diagnostics` verdict so an application can tell a trustworthy conversion
from a suspect one — and tell the user when they'd be better served exporting a mesh directly
from their CAD package:

```ts
const { diagnostics: d } = importStep(stepText);
if (!d.ok) {
  const broken = d.openEdges > 0 || d.nonManifoldEdges > 0
    || d.warnings.some((w) => w.severity === "error");
  showBanner(broken
    ? "This STEP file imported with defects (missing or leaking geometry). " +
      "Consider exporting an STL/3MF directly from your CAD software instead."
    : "Some faces needed heuristic repairs — please inspect the result.");
}
```

- `ok` — strict: `true` only when the mesh is closed and manifold, no faces were dropped or
  skipped, and no heuristic repair paths fired.
- `openEdges` / `nonManifoldEdges` — final edge-defect audit of the returned mesh (open-by-design
  surface bodies from `openSolids` excluded). Non-zero means cracks/holes or bad welds.
- `facesDropped` / `facesSkipped` — geometry that is *missing*: malformed face records dropped
  while reading the B-rep, and faces the tessellator could not mesh.
- `warnings` — structured findings with `code`, `severity`, the STEP `faceId`, and a
  human-readable `detail`. Severity `"error"` means geometry is missing; `"warning"` means a
  rescue path (earcut hole-bridge fill, degenerate-boundary CDT rescue, fold surgery,
  self-intersecting trim loops) reconstructed a region heuristically — the result is usually
  fine, but it was not derived from clean topology.

`meshDefects(mesh, solidOfTri?, openSolids?)` is exported separately for re-auditing a mesh after
downstream processing.

## Web verification studio

A browser UI for visually checking output lives in [web/](web/README.md): upload a STEP, tune
the settings, and inspect the result with deviation coloring against a reference STL (3D-scan
style), wireframe, and open-edge highlighting. `cd web && npm install && npm run dev`.

## License

AGPL-3.0-only. Copyright held by CNC Kitchen; commercial licenses for closed-source use are
available — see [COMMERCIAL.md](COMMERCIAL.md). The library ships with zero third-party code, so
the whole artifact can be licensed directly.

Contributions are welcome under the CLA and dependency-license policy in
[CONTRIBUTING.md](CONTRIBUTING.md) (enforced in CI).
