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
  surface deviation, normal deviation, max edge length.
- **Tessellated view** — orbit/zoom the generated mesh.
- **Reference STL** upload (e.g. the matching `*.stl` next to each `*.step` in
  the repo root) with a toggleable translucent **overlay**.
- **Deviation analysis** — colors the generated mesh by signed distance to the
  reference surface (blue = inside, red = outside), just like a 3D-scan
  deviation report. Computed with a BVH (`three-mesh-bvh`) for true
  point-to-surface distance. Color range auto-scales or can be set manually.
  Max/RMS deviation is shown in the corner.
- **Wireframe** overlay to inspect the mesh.
- **Open-edge highlighting** — boundary edges (used by a single triangle) drawn
  in red. A watertight mesh shows none; the corner readout reports the count.

## Quick test

Load `cube.step` (or `cylinderWithHole.step`, `sphere.step`, `tool.step`, …)
from the repo root, convert, then load the matching `.stl` as the reference and
turn on **Color by deviation**.
