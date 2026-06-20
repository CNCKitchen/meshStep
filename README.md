# meshStep

Pure-TypeScript **STEP → uniform-isotropic mesh** importer. Reads ISO-10303-21 (STEP) B-rep
geometry and produces watertight, near-uniform, low-sliver triangle meshes — the opposite of the
curvature-adaptive, sliver-prone output you get from a CAD program's STL export.

Built to feed [bumpmesh](https://github.com/CNCKitchen/stlTexturizer) (displacement texturing) and
[infeall](https://github.com/CNCKitchen/smartInfillGenerator) (smart infill). Runs in the browser
(no WASM, no native deps) and in Node.

> **v1 scope:** analytic surfaces (plane, cylinder, cone, sphere, torus). B-spline/NURBS faces are
> detected and reported as unsupported (phase 2). See [DESIGN.md](DESIGN.md).

## Status

Analytic geometry is **done and validated**: plane, cylinder, cone, sphere, tori-free fillets,
drilled holes (incl. B-spline-curve intersection edges), and multi-body parts all import as
**watertight, outward-facing, ~uniform 1 mm, <2%-sliver** meshes within ~0.5% of the reference
geometry. The cylinder "twist" that disables STEP import elsewhere cannot occur (tessellation is
done in flat parameter space).

Pipeline: STEP parse → BREP → parameter-space tessellation (shared-edge sampling, constrained
Delaunay, seam unwrap) → isotropic remesh (split/collapse/flip/smooth + projection onto the
analytic surface, with CAD feature edges frozen). See [DESIGN.md](DESIGN.md).

**Remaining:** 3MF export + the controls/preset UI (M5/M6), and B-spline *surfaces*
(`splineThing`, one body of `everything`) in phase 2.

## Quick start (dev)

Requires Node ≥ 22 (uses native TypeScript type-stripping — no build step, no dependencies).

```bash
node test/convert.ts        # import every test STEP, export STL to out/, report quality vs reference
```

```ts
import { importStep, writeBinarySTL } from "./src/index.ts";
const result = importStep(stepText, { remesh: true, targetEdge: 1.0 });
// result.mesh (positions/indices), result.faceOfTri, result.solidOfTri
writeFileSync("out.stl", writeBinarySTL(result.mesh));
```

## License

AGPL-3.0-only. Copyright held by CNC Kitchen; available for commercial licensing.
