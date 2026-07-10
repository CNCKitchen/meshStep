---
name: verify
description: Build, launch, and drive meshStep's surfaces (library + web viewer) to verify changes end-to-end.
---

# Verifying meshStep changes

## Library surface (src/)

Node ≥ 22 runs the TypeScript sources directly. Exercise `importStep` through the public entry:

```js
// from any directory — use a file:// specifier, relative paths break outside the repo
const m = await import("file:///C:/Users/stefa/Desktop/Coding/meshStep/src/index.ts");
```

`models/pixel-pump-two-mainboard.step` is the big real-world fixture (~45 s import, 2M tris);
the occt-import-js test files (`node_modules/occt-import-js/test/testfiles/*/*.step`) are small
(~2 s) and all carry STEP colors. For an uncolored fixture, `sed '/STYLED_ITEM/d'` a copy.

## Web viewer surface (web/)

```bash
cd web && npm run dev        # NOTE: check the port in the output — often 5180, not 5173
```

Drive it headless with puppeteer-core + system Chrome (Edge headless fails to launch here):

```js
const browser = await puppeteer.launch({
  executablePath: "C:/Program Files/Google/Chrome/Application/chrome.exe",
  headless: "new",
  args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--no-sandbox",
         "--user-data-dir=<some temp dir>",   // required, launch fails without a clean profile
         "--window-size=1500,950"],
});
```

Flow: `(await page.$("#stepFile")).uploadFile(path)` → wait `#convertBtn` enabled → click it →
wait until `#status` starts with "Done" (pixel-pump: ~70 s in-browser) → screenshot. Toggles are
checkboxes `#tColors #tFeature #tTransparent #tEdgesOnly #tWire #tEdges #tRef #tDev`; reference
STL goes in `#refFile`. Dismiss the sponsor/CTA overlays first or they cover the viewport:

```js
await page.evaluate(() => {
  localStorage.setItem("meshstep.sponsor.dismissed", "1");
  localStorage.setItem("meshstep.cta.dismissed", "1");
});
await page.reload({ waitUntil: "networkidle0" });
```

Gotcha: the "Export STL" button uses a blob-URL anchor click — `Page.setDownloadBehavior` does
not capture it. Generate reference STLs via the library (`writeBinarySTL`) instead.
