# meshStep · verification studio

A browser UI for visually verifying meshStep output. Upload a STEP file, tune the
mesh settings, and inspect the raw tessellation in a 3D view.

## Run

```bash
cd web
npm install
npm run dev        # http://localhost:5173
```

`npm run build` produces a static bundle in `web/dist/`.

## Deploy (public STEP viewer)

The build uses `base: "./"` (relative asset URLs), so the same `dist/` works unchanged at an apex
domain, a GitHub Pages project path, or a subpath like `cnckitchen.com/stepview`.

- **`.github/workflows/deploy-viewer.yml`** builds `web/` and publishes to GitHub Pages on every push
  to `main` (enable Pages → "GitHub Actions" in repo settings first). Out of the box it serves at
  `cnckitchen.github.io/meshStep/`.
- **Subdomain (`stepview.cnckitchen.com`)** — easiest custom domain: add `web/public/CNAME` containing
  `stepview.cnckitchen.com`, point a DNS `CNAME` record at `cnckitchen.github.io`, and update the
  `canonical`/`og:url` in `index.html` + the URLs in `public/robots.txt` / `public/sitemap.xml`.
- **Subpath (`cnckitchen.com/stepview`)** — best for SEO (full domain authority), but GitHub Pages
  can't serve a subpath of a domain it doesn't control. Either drop `dist/` under `/stepview` on the
  host that already serves `cnckitchen.com`, or reverse-proxy `/stepview/*` to the Pages site. The
  canonical/OG/sitemap URLs are already set to this path.

SEO surface lives in `index.html` (`<title>`, meta description, canonical, Open Graph, and a
`WebApplication` JSON-LD block) plus `public/robots.txt` and `public/sitemap.xml`.

The app imports the meshStep library directly from `../src` (pure TypeScript,
zero runtime deps) and runs the conversion in a Web Worker so the UI never
freezes.

## Features

- **Upload a STEP** file and convert it with the same options as the library:
  surface deviation, normal deviation, max edge length. Surface deviation and
  max edge auto-scale to the model's size on load (a fast pre-pass estimates
  the bounding box before converting); edit either field and your values win.
- **Upload an STL** (binary or ASCII) through the same picker — no Convert step;
  the mesh is welded and displayed directly with the full toolset (section view,
  measuring, open-edge highlighting, watertight check, deviation vs. a reference
  STL). STL carries no CAD faces, so the boundary overlay becomes **feature
  edges by crease angle**: edges whose adjacent triangles' normals differ by
  more than an adjustable threshold (default 30°). Disconnected shells are
  detected and listed as parts (Shell 1, Shell 2, …), so per-shell hiding and
  defect attribution still work.
- **Upload a 3MF** — objects are welded per shell and displayed with their 3MF
  colors (per-triangle color groups) and a per-object parts list. "Surface" and
  "support" objects are open by design and excluded from the watertight audit.
- **Tessellated view** — orbit/zoom the generated mesh, with a views menu for
  standard camera angles (top/front/…, Ctrl+0 = isometric) and a segmented
  render-style control: shaded, transparent, wireframe, edges.
- **Section view** — cut the model with a clipping plane: X/Y/Z axis buttons
  plus a draggable gizmo to position it interactively.
- **Measure mode** — click vertices/edges/faces to measure. STEP imports carry
  exact analytic identity from the B-rep (true circle centers/radii/axes, not
  values fitted from triangles); point-to-point distance works on every format.
- **Model colors** — the STEP file's face/body colors (`STYLED_ITEM`), rendered
  by default when present, with a show/hide toggle. Colors stay crisp across
  face borders (shared border vertices are split per color). While **Color by
  deviation** is on, the analysis coloring takes precedence.
- **Parts tree** — for multi-body/assembly files, a collapsible component tree
  from the STEP product structure (product names, ×N occurrence counts, body
  counts). Hover a row to highlight the part in orange; uncheck to hide it
  (triangles and edge lines both). A part used N times in the assembly is one
  row — hiding it hides all instances. Right-clicking a part in the viewport
  opens a context menu with the same actions.
- **Reference STL** upload (e.g. the matching `*.stl` next to each `*.step` in
  the repo root) with a toggleable translucent **overlay**.
- **Deviation analysis** — colors the generated mesh by signed distance to the
  reference surface (blue = inside, red = outside), just like a 3D-scan
  deviation report. Computed with a BVH (`three-mesh-bvh`) for true
  point-to-surface distance. Color range auto-scales or can be set manually.
  Max/RMS deviation is shown in the corner.
- **Open-edge highlighting** — boundary edges (used by a single triangle) drawn
  in red. A watertight mesh shows none; the corner readout reports the count.
  Sheet (surface) bodies are open **by design** — their boundary edges are
  excluded from the red overlay and the counters, so a sheet-metal model reads
  "Clean · N sheet bodies" instead of thousands of false open edges. Sheet
  bodies are tagged in the parts tree.

## Quick test

Load `cube.step` (or `cylinderWithHole.step`, `sphere.step`, `tool.step`, …)
from the repo root, convert, then load the matching `.stl` as the reference and
turn on **Color by deviation**.
