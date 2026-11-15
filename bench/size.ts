// SPDX-License-Identifier: AGPL-3.0-only
// Footprint KPI: what a web page has to download to convert STEP in the browser with each library.
// JavaScript is bundled and minified with esbuild (browser target), WebAssembly counted as shipped;
// sizes raw, gzip (level 9) and brotli (quality 11), as a CDN would serve them.
//
//   node bench/size.ts            prints a table and writes bench/out/size.json
import { buildSync } from "esbuild";
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { gzipSync, brotliCompressSync, constants } from "node:zlib";

const here = dirname(fileURLToPath(import.meta.url));
const nm = (p: string) => join(here, "node_modules", p);

interface Lib { id: string; js: string[]; wasm: string[] }
const LIBS: Lib[] = [
  // the library itself; the worker script (meshstep/worker) is the same code again, not counted twice
  { id: "meshstep", js: [join(here, "..", "src", "index.ts")], wasm: [] },
  { id: "occt-import-js", js: [nm("occt-import-js/dist/occt-import-js.js")], wasm: [nm("occt-import-js/dist/occt-import-js.wasm")] },
  { id: "tx-occt-js", js: [nm("@tx-code/occt-js/dist/occt-js.js")], wasm: [nm("@tx-code/occt-js/dist/occt-js.wasm")] },
  { id: "occt-wasm", js: [nm("occt-wasm/dist/index.js"), nm("occt-wasm/dist/occt-wasm.js")], wasm: [nm("occt-wasm/dist/occt-wasm.wasm")] },
  { id: "opencascade.js", js: [nm("opencascade.js/dist/opencascade.wasm.js")], wasm: [nm("opencascade.js/dist/opencascade.wasm.wasm")] },
  { id: "brepkit-wasm", js: [nm("brepkit-wasm/brepkit_wasm.js")], wasm: [nm("brepkit-wasm/brepkit_wasm_bg.wasm")] },
];

const gz = (b: Uint8Array) => gzipSync(b, { level: 9 }).length;
const br = (b: Uint8Array) => brotliCompressSync(b, { params: { [constants.BROTLI_PARAM_QUALITY]: 11, [constants.BROTLI_PARAM_SIZE_HINT]: b.length } }).length;

const rows: Record<string, { jsRaw: number; wasmRaw: number; raw: number; gzip: number; brotli: number }> = {};
for (const lib of LIBS) {
  const parts: Uint8Array[] = [];
  for (const entry of lib.js) {
    const r = buildSync({
      entryPoints: [entry], bundle: true, minify: true, write: false, format: "esm", platform: "browser",
      target: "es2022", logLevel: "silent",
      // Node-only branches of the Emscripten glue / meshStep's worker pool stay out of a browser build
      external: ["node:*", "fs", "path", "crypto", "url", "module", "worker_threads", "os", "child_process", "ws", "*.wasm", "./occt-wasm.js"],
    });
    parts.push(r.outputFiles[0]!.contents);
  }
  const js = Buffer.concat(parts);
  const wasm = Buffer.concat(lib.wasm.map(f => readFileSync(f)));
  rows[lib.id] = {
    jsRaw: js.length, wasmRaw: wasm.length, raw: js.length + wasm.length,
    gzip: gz(js) + (wasm.length ? gz(wasm) : 0),
    brotli: br(js) + (wasm.length ? br(wasm) : 0),
  };
}
mkdirSync(join(here, "out"), { recursive: true });
writeFileSync(join(here, "out", "size.json"), JSON.stringify(rows, null, 1));
const mb = (n: number) => (n / 1e6).toFixed(2).padStart(7) + " MB";
console.log("library            raw          gzip         brotli");
for (const [id, r] of Object.entries(rows)) console.log(`${id.padEnd(16)} ${mb(r.raw)}  ${mb(r.gzip)}  ${mb(r.brotli)}`);
