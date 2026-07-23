// SPDX-License-Identifier: AGPL-3.0-only
// AbortSignal regression: an import whose signal aborts mid-tessellation must throw the signal's
// AbortError promptly (work-unit granularity), and a non-aborted signal must not disturb the run.
import { readFileSync } from "node:fs";
import { importStep } from "../src/index.ts";

let failures = 0;
function check(name: string, cond: boolean, detail = ""): void {
  if (!cond) { failures++; console.error(`FAIL ${name}${detail ? ` — ${detail}` : ""}`); }
  else console.log(`ok   ${name}`);
}

const src = readFileSync(new URL("../tool.step", import.meta.url), "utf8");

// --- abort after N progress ticks → AbortError, and no further progress observed.
{
  const ctl = new AbortController();
  let ticks = 0, ticksAfterAbort = 0;
  let err: unknown = null;
  try {
    importStep(src, {
      signal: ctl.signal,
      onProgress: () => {
        if (ctl.signal.aborted) ticksAfterAbort++;
        if (++ticks === 25) ctl.abort();
      },
    });
  } catch (e) { err = e; }
  check("abort: throws", err !== null);
  check("abort: AbortError", err instanceof DOMException && err.name === "AbortError",
    String(err));
  check("abort: prompt (≤1 tick after)", ticksAfterAbort <= 1, `${ticksAfterAbort} ticks after abort`);
}

// --- pre-aborted signal → throws before any work.
{
  let ticks = 0;
  let threw = false;
  try { importStep(src, { signal: AbortSignal.abort(), onProgress: () => { ticks++; } }); }
  catch { threw = true; }
  check("abort: pre-aborted throws immediately", threw && ticks === 0, `ticks=${ticks}`);
}

// --- live (never-aborted) signal → import completes normally.
{
  const r = importStep(src, { signal: new AbortController().signal });
  check("abort: live signal completes", r.mesh.indices.length > 0 && r.diagnostics.ok);
}

if (failures > 0) { console.error(`${failures} failure(s)`); process.exit(1); }
console.log("abort: all passed");
