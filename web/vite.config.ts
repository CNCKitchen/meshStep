// SPDX-License-Identifier: AGPL-3.0-only
import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";

// The meshStep library lives one level up in ../src and is imported directly
// (it's pure TS with .ts import specifiers, which esbuild/Vite resolve fine).
const repoRoot = fileURLToPath(new URL("..", import.meta.url));

export default defineConfig({
  root: fileURLToPath(new URL(".", import.meta.url)),
  server: {
    port: 5180,
    fs: {
      // allow importing the library source that sits outside web/
      allow: [repoRoot],
    },
  },
  worker: {
    format: "es",
  },
  build: {
    target: "es2022",
    outDir: "dist",
    emptyOutDir: true,
  },
});
