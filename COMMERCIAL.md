# Commercial licensing

meshStep is open source under the **GNU AGPL-3.0-only** (see
[LICENSE](LICENSE)). In short: you may use, modify, self-host, and
redistribute it freely — but if you distribute it, or offer it (modified or
not) to users over a network, you must make the complete corresponding source
of your version available under the same license.

If those terms don't work for your product — typically because you want to:

- embed the STEP importer in **closed-source software** (a slicer, CAD tool,
  mesh-processing pipeline, web service, printer software), or
- ship a modified version **without publishing your changes**,

then a **commercial license exception** is available. The copyright to this
codebase is held by a single owner (enforced via the contributor agreement,
see [CONTRIBUTING.md](CONTRIBUTING.md)), and the library has **zero runtime
dependencies** — no third-party code is bundled — so proprietary-use licenses
can be granted directly and simply, covering the entire shipped artifact.

Contact: **stefan@cnckitchen.com**

Notes:

- **Your output is yours.** Meshes produced with meshStep (STL or otherwise)
  are not covered by the AGPL — the license governs the software, not the
  files you convert with it.
- The validation harness compares against
  [occt-import-js](https://github.com/kovacsv/occt-import-js) (which wraps
  LGPL OpenCASCADE) as a *development-time reference only*; it is not part of
  the library and is not included in any licensed artifact.
- The project name and logo are trademarks and are **not** licensed under the
  AGPL. Forks must use a different name.
