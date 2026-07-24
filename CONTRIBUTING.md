# Contributing

Thanks for your interest! Two ground rules keep this project sustainable —
please read them before opening a PR.

## 1. Contributor License Agreement (CLA)

This project is dual-licensed: open source under AGPL-3.0-only for everyone,
with commercial exceptions sold to companies that want to embed it in
proprietary software (see [COMMERCIAL.md](COMMERCIAL.md)). That model only
works if the project owner holds the licensing rights to the whole codebase.

Therefore every contribution requires agreeing to the CLA: you keep the
copyright to your contribution, but you grant the project owner a perpetual,
irrevocable, worldwide right to license your contribution under any terms,
including proprietary ones. By submitting a pull request you confirm:

> I have the right to submit this contribution, and I grant Stefan Hermann
> (CNC Kitchen) a perpetual, worldwide, non-exclusive, irrevocable,
> royalty-free license to use, modify, sublicense, and relicense my
> contribution under licenses of his choosing, including the AGPL-3.0-only
> and commercial licenses.

We're transparent about why: without this, a single outside patch would
legally block the commercial-exception model that funds development.

## 2. Dependency license policy (CRITICAL — checked on every PR)

The dual-licensing model collapses if the codebase ever contains third-party
copyleft code we don't own: we cannot sell a commercial exception covering
someone else's (A)GPL code. **Every new dependency — npm package or vendored
snippet — must be license-vetted before it lands.**

meshStep's core rule is stricter than an allowlist: **the library
(`src/`) has zero runtime dependencies, and stays that way.** That is both a
product feature (no WASM, no supply chain) and the license firewall — nothing
ships to commercial licensees except code this project owns. Anything vendored
into `src/` must be original work contributed under the CLA above; do not
paste third-party code, however small (Stack Overflow snippets are CC BY-SA —
do not paste them).

For the tooling around the library:

- **`web/` verification studio** — permissive-licensed npm packages are
  allowed: MIT, Apache-2.0, BSD-2/3-Clause, ISC, 0BSD, Zlib, CC0-1.0,
  Unicode-3.0, and MPL-2.0 (file-level copyleft — acceptable for commercial
  licensees). Currently: `three`, `three-mesh-bvh` (both MIT).
- **`test/` harness** — dev-only dependencies never ship, but keep them
  permissive where possible. Exception: `occt-import-js` (MIT wrapper around
  LGPL-2.1 OpenCASCADE) is used **only** as the reference implementation in
  the validation harness (`test/gapcheck*`). It must never be imported from
  `src/` or `web/src/`, and never become a runtime dependency.

**Never**, anywhere in the repo, regardless of how useful:

- GPL (any version), LGPL in the core (everything here bundles statically),
  AGPL code we don't own, SSPL, BSL/FSL, "non-commercial" (CC-BY-NC etc.),
  JSON license, unlicensed/no-license code, and copy-pasted code of unknown
  origin.

If a copyleft component is ever genuinely needed, the options are: isolate it
behind a process/network boundary as an optional component, buy a commercial
license for it, or write our own. Ask first.

Checking: CI enforces this on every PR and push via
[.github/workflows/license-check.yml](.github/workflows/license-check.yml) —
the root `package.json` must have no runtime dependencies, and the `web/`
dependency tree must match the allowlist
(`npx license-checker --onlyAllow "..."`).

## Practicalities

- Requires Node ≥ 22.18 (native type-stripping — no build step for dev).
- `npm run typecheck` (tsc strict) and `node test/convert.ts` must stay green;
  `node test/check-all.ts` must report every model watertight (0 open,
  0 non-manifold). Treat tolerance changes in the acceptance numbers as red
  flags.
- Corpus validation: `node test/gapcheck.ts <dir>` cross-checks against
  OpenCASCADE — run it when touching tessellation code.
- Web studio: `cd web && npm run build` (tsc strict + vite).
- Test CAD models (`*.step` / `*.stl` / `*.3mf`) are deliberately **not
  committed** — they include third-party downloads (Printables, Voron) whose
  licenses don't belong in this repo. Keep them local; don't add exceptions
  to `.gitignore` for them. The one carve-out: the four small **own-made**
  fixtures the publish test gate needs in CI (`cube/cylinder/sphere/
  everything.step`, exported from the maintainer's own CAD designs) are
  committed via explicit `.gitignore` exceptions. Anything added to that list
  must be original work covered by the CLA — never a downloaded model.
- New source files carry the SPDX header:
  `// SPDX-License-Identifier: AGPL-3.0-only`

## Releasing (maintainers)

Publishing to npm is automated by
[.github/workflows/publish.yml](.github/workflows/publish.yml): pushing a
version tag builds, runs the full test gate (`prepublishOnly`), and publishes
via npm [trusted publishing](https://docs.npmjs.com/trusted-publishers)
(OIDC — no tokens stored anywhere). The whole ritual is:

```sh
npm version patch   # or minor / major — bumps package.json, commits, tags vX.Y.Z
git push --follow-tags
```

Notes:

- `npm version` requires a clean working tree — commit or stash first.
- The workflow refuses a tag that doesn't match `package.json` (don't tag by
  hand), and green-skips if that version is already on the registry, so
  re-tagging and re-runs are harmless.
- Version semantics: patch = fixes, minor = backward-compatible features,
  major = breaking API changes. While on `0.x`, treat minor as "may break".
- Manual fallback (`npm publish` from the repo root) works but needs
  `npm login` + 2FA; the tag/workflow route is preferred so npm always ships
  exactly what's on GitHub.
