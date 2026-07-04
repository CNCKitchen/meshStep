# meshStep: robust seam-aware boundary projection + tolerant CDT

## Mission
Replace meshStep's naive 3D→(u,v) boundary projection (`loopParam`) with a robust, seam-aware
per-edge pcurve builder + wire assembly, and harden the CDT against slightly imperfect boundaries —
so Shapr3D-style STEP files with tangled periodic parametrizations mesh watertight, the way OCCT's
BRepMesh does for PrusaSlicer/Bambu. Work happens on branch `seam-aware-projection` (already created
off main @ 4486d5c, clean tree). Work autonomously; commit in small measured steps; take all the
time and tokens you need.

## Hard constraints (non-negotiable)
1. **General geometry only.** No per-model, per-face-id, or per-filename special cases. The STEP
   import must work for ANY geometry.
2. **Clean-room.** meshStep will be released AGPL-3.0 with copyright retained. Do NOT read or port
   OCCT (or any other kernel) source. Implement from this brief and standard published algorithms
   (covering-space lifting, constrained Delaunay, snap rounding). Algorithms are free; their source
   expression is not.
3. **Zero runtime dependencies.** TypeScript only. Match the existing code style (dense,
   intent-explaining comments).
4. **Watertightness invariant.** Every EDGE_CURVE is sampled ONCE (`sampleEdgePolyline`); the shared
   3D polyline must be used VERBATIM by both adjacent faces. Never move or resample boundary 3D
   points per-face — only their 2D (u,v) images may be recomputed.
5. **No corpus regressions.** All 18 currently-passing files must stay open=0 nm=0 emptyFaces=0.

## Where things stand (baseline, measured 2026-07-03)
`node test/corpus.ts` (~4–5 min full): **18 pass / 13 fail of 31**. Key lines:
- `OpenVessel.step` FAIL open=425 nm=33 empty=3/324 — **PRIMARY TARGET** (Shapr3D/HOOPS export)
- `OpenVessel_fusion360.step` FAIL open=6 nm=0 — same geometry via Fusion re-export (near-clean reference)
- `roundedCube.step` PASS (38.5k tris) and `GARMIN_MOUNT_v15.step` PASS (20.2k tris) — small
  user-supplied smoke models with rounded-corner/periodic faces. Keep them perfect; use them for
  fast iteration (each runs in <1 s). Matching CAD reference STLs sit next to them in testFiles/.
- nist_stc/ftc seam+pole cases FAIL small (7–260 open) — should improve or hold; log the numbers.

## The defect (measured evidence)
- Neither OpenVessel STEP contains pcurves (0 PCURVE / SURFACE_CURVE entities) — the 2D boundary
  must be COMPUTED. OCCT computes it robustly; meshStep does not yet.
- `loopParam` (src/mesh/tessellate.ts:70) flattens the whole loop into one 3D point list, projects
  greedily (each point hint-seeded by the previous result), then unwraps ±period jumps. Failure
  mode: on periodic surfaces, a loop containing a SEAM EDGE (same edgeId appearing TWICE in one
  loop — must be traversed once at v=0 and once at v=vPeriod) gets both traversals projected to the
  SAME side → the 2D loop self-intersects → the CDT filters most triangles → cracks / empty faces.
- Concrete: OpenVessel face #2025 (periodic-v B-spline counterbore rim), loop edges
  [1679, 1772, 1863, 1679, 1918, 1969, 2021] — #1679 is the seam edge. Naive projection
  self-intersects 34× and spans ~2 v-periods. The Fusion twin of this face projects to a clean
  simple polygon and meshes perfectly.
- ~9 pathological faces account for most of OpenVessel's 425 open edges. Open-edge attribution by
  adjacent face kind: B_SPLINE 168, OFFSET_SURFACE 89, SURFACE_OF_LINEAR_EXTRUSION 75, CYL 48,
  PLANE 26, DEGENERATE_TOROIDAL 18 (cracks pair across kinds — fixing the tangled faces heals
  their neighbors' open edges too).

## API cheat sheet
- `Surface` (src/geom/surfaces.ts): `evaluate(u,v):Vec3` · `project(p:Vec3, hintU?, hintV?):[u,v]`
  (analytic surfaces are exact and ignore hints; B-spline runs Newton from the hint or a coarse
  seed grid) · `normal(u,v)` · `periodicU/periodicV:boolean` · `uPeriod/vPeriod` ·
  `curvatureRadius(u,v)` · B-splines expose the knot domain `u0/u1/v0/v1`.
- `BrepModel` (src/brep/build.ts): `solids[].faces[]` = {faceId, surfaceId, surfaceKind, sameSense,
  loops: BLoop[]}; `BLoop` = {outer:boolean, edges: {edgeId, orient}[]}; `brep.edges`:
  Map<id, {curveId, v0, v1, sameSense}>.
- Meshing path (src/mesh/tessellate.ts): `tessellate()` dispatch → `tessellateParamGrid` →
  `loopParam` (THE TARGET) → `gridCDT` → `constrainedTriangulate` (src/mesh/cdt2d.ts). Dispatch
  order matters (thin/ribbon, sphere, cone, revolution-band, periodic-unroll run around the param
  grid) — keep it.
- p3/p2 pairing: `gridCDT` maps triangles back to 3D through the loop's `p3` array. Any new
  projector must return `p2` aligned 1:1 with `p3` (the shared edge polylines, per-edge order
  flipped by `oe.orient`). Loop winding matters only for region classification (`pointInPoly`);
  final triangle orientation is handled downstream by `emitTri` via the surface normal.

## Design (implement from this, not from any kernel's source)
**Phase A — per-edge pcurves.** Project each edge's shared polyline to (u,v) independently,
hint-chained within the edge only; keep the existing large-residual → stateless-reproject fallback.
Record per edge: the p2 array plus flags hugs-u-seam / hugs-v-seam (all points within a metric
tolerance of that periodic seam). Use metric tolerances (measure mm via `evaluate`, or scale by
local uScale/vScale as `gridCDT` does) — never blind parameter epsilons.

**Phase B — wire assembly (lift to the universal cover).** Walk the loop's edges in order. Place
edge k's pcurve shifted by integer period multiples (each periodic axis independently) chosen to
minimize the 2D gap to edge k−1's endpoint. Seam-hugging pcurves get BOTH representatives (v≈0 and
v≈vPeriod) as candidates; choose by continuity. A seam edge traversed twice then lands one period
apart automatically. Validate closure (last end ≈ first start): drift ≈ 0 → trimmed patch, proceed;
drift ≈ ±1 period → the loop winds the surface — route to the existing band/unroll meshers (detect,
don't force a broken patch).

**Phase C — validate + repair.** Simplicity check: count 2D segment self-intersections (O(n²) with
early-out is fine at boundary density; loops are typically a few hundred points). If tangled:
bounded search over the seam-representative choices from Phase B (few edges have candidates → few
combinations); score by (self-intersection count, then plausible signed area). Orientation: outer
loop winds positive, holes negative, for region classification — fix by reversing p2 AND p3
together (they stay paired; array order doesn't alter 3D geometry).

**Phase D — tolerant CDT (only if A–C leave residual tangles, e.g. genuinely folded surfaces).**
Preprocess the 2D boundary: snap near-coincident vertices (metric tolerance), split constraint
segments at true pairwise intersections, drop duplicate segments; triangulate the resulting
arrangement so a slightly-imperfect boundary still yields a region-classifiable triangulation
instead of mass triangle filtering. Gate it behind the Phase-C simplicity check so clean loops keep
the existing fast path bit-for-bit.

Recommended order: A+B first (they fix the seam class outright) → run corpus → C → run corpus →
only then decide whether D is still needed. Commit each phase separately with the measured corpus
numbers in the commit message.

## Already tried — do NOT retry (details in memory: meshstep-remaining-gaps.md)
- Analytic re-fit (sphere/plane) of near-analytic B-splines → made OpenVessel WORSE (168→190
  B-spline open edges), even for a face fitting a sphere to 0.011 mm.
- Global seam-offset heuristics on the flattened whole-loop walk (offset 2nd seam traversal by
  +period; rotate loop to start at the seam) → self-intersections 34→22, never 0. Phase B's
  per-edge assembly is the principled version of this idea — the whole-loop variants are dead.
- Ribbon multi-rail chaining (≥3 rails) → spurious caps across holes, nm 33→84.
- Best-fit-plane trimming fallback → fires on nothing (loops are doubled or 3D-tangled).
- A pole-split in loopParam → regressed stc_08, non-manifold tris on ctc_02.

## Verification workflow
- Fast: `node test/corpus.ts roundedCube` · `GARMIN` · `OpenVessel` (name filter; OpenVessel ~30 s).
- Full: `node test/corpus.ts` — PASS requires open=0 nm=0 emptyFaces=0 per file.
- Visual: `node test/render.ts "../testFiles/OpenVessel.step" <az> <el> out/x.png` (open edges
  drawn red, non-manifold magenta; create out/ first). CAD reference STLs: roundedCube.stl,
  GARMIN_MOUNT_v15.stl, OpenVessel.stl in testFiles/.
- Probe scripts from the prior session (self-intersection counts per strategy, per-face renders)
  live in the old scratchpad:
  `C:\Users\stefa\AppData\Local\Temp\claude\c--Users-stefa-Desktop-Coding-meshStep\34d5aac0-b96f-4579-9963-a2f675d1468b\scratchpad\`
  (seamtest.ts, loopprobe.ts, facerender.ts, cmp2.ts) — reusable, or rewrite in your own scratchpad.
- Performance guard: full-corpus time should stay in the same ballpark (~5 min); prefilter any new
  O(n²) checks (bbox / early-out).

## Success criteria (in order)
1. Zero regressions on the 18 passing files.
2. OpenVessel.step: open 425 → below 50 (stretch: ~0, OCCT-level), nm 33 → down, empty 3 → 0.
3. OpenVessel_fusion360.step: 6 → 0 if reachable.
4. NIST seam/pole files improve or hold; report the numbers honestly either way.
If an approach measures worse after a fair attempt, revert it and say so — never keep speculative
code that doesn't measurably help.
