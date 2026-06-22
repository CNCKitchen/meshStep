# meshStep · verification studio

A browser UI for visually verifying meshStep output. Upload a STEP file, tune the
mesh settings, and inspect the result in two **synced** 3D views — the raw
tessellation on the left, the isotropically remeshed result on the right.

## Run

```bash
cd web
npm install
npm run dev        # http://localhost:5173
```

`npm run build` produces a static bundle in `web/dist/`.

The app imports the meshStep library directly from `../src` (pure TypeScript,
zero runtime deps) and runs the conversion in a Web Worker so the UI never
freezes.

## Features

- **Upload a STEP** file and convert it with the same options as the library:
  surface deviation, normal deviation, max edge length, remesh iterations.
- **Two synced views** — left: tessellated; right: tessellated + remeshed.
  One shared camera, so orbit/zoom stays locked between them.
- **Reference STL** upload (e.g. the matching `*.stl` next to each `*.step` in
  the repo root) with a toggleable translucent **overlay**.
- **Deviation analysis** — colors each generated mesh by signed distance to the
  reference surface (blue = inside, red = outside), just like a 3D-scan
  deviation report. Computed with a BVH (`three-mesh-bvh`) for true
  point-to-surface distance. Color range auto-scales or can be set manually.
  Per-view max/RMS deviation is shown in the corner.
- **Wireframe** overlay on both views to inspect the mesh.
- **Open-edge highlighting** — boundary edges (used by a single triangle) drawn
  in red. A watertight mesh shows none; the corner readout reports the count.

## Quick test

Load `cube.step` (or `cylinderWithHole.step`, `sphere.step`, `tool.step`, …)
from the repo root, convert, then load the matching `.stl` as the reference and
turn on **Color by deviation**.
