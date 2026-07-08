// Re-run a specific list of ABC models through the abc-worker and report status transitions.
//   node test/abc-rerun-list.mjs out/abc-run2/near-misses.json out/abc-run2/near-rerun.jsonl
import { readFileSync, writeFileSync } from "node:fs";
import { spawn } from "node:child_process";
import { join } from "node:path";

const ABC = "abc_0000_step_v00";
const SENT = "@@ABC@@";
const listPath = process.argv[2] ?? "out/abc-run2/near-misses.json";
const outPath = process.argv[3] ?? "out/abc-run2/near-rerun.jsonl";
const near = JSON.parse(readFileSync(listPath, "utf8"));
const files = near.map((r) => r.file);

const N = 6;
const queues = Array.from({ length: N }, () => []);
files.forEach((f, i) => queues[i % N].push(f));

const results = new Map();
let doneWorkers = 0;
for (let k = 0; k < N; k++) {
  const w = spawn(process.execPath, ["test/abc-worker.ts"], { stdio: ["pipe", "pipe", "inherit"] });
  const q = queues[k];
  let qi = -1; // -1 until ready line
  let buf = "";
  w.stdout.on("data", (d) => {
    buf += d.toString();
    let nl;
    while ((nl = buf.indexOf("\n")) >= 0) {
      const line = buf.slice(0, nl).trim();
      buf = buf.slice(nl + 1);
      if (!line.startsWith(SENT)) continue;
      const rec = JSON.parse(line.slice(SENT.length));
      if (qi >= 0) {
        rec.file = q[qi];
        results.set(q[qi], rec);
        console.log(`[${results.size}/${files.length}] ${rec.status} open=${rec.open ?? "-"} ${q[qi]}`);
      }
      qi++;
      if (qi < q.length) w.stdin.write(join(ABC, q[qi]) + "\n");
      else { w.stdin.end(); }
    }
  });
  w.on("exit", () => {
    if (++doneWorkers === N) {
      const recs = files.map((f) => results.get(f) ?? { file: f, status: "MISSING" });
      writeFileSync(outPath, recs.map((r) => JSON.stringify(r)).join("\n") + "\n");
      const byStatus = {};
      for (const r of recs) byStatus[r.status] = (byStatus[r.status] ?? 0) + 1;
      console.log("\n=== transitions (all were FAIL near-miss before):", JSON.stringify(byStatus));
      const stillFail = recs.filter((r) => r.status !== "PASS");
      const buckets = {};
      for (const r of stillFail) buckets[r.bucket ?? r.status] = (buckets[r.bucket ?? r.status] ?? 0) + 1;
      console.log("still not PASS:", stillFail.length, JSON.stringify(buckets));
      for (const r of stillFail) console.log("  ", r.status, r.bucket, "open=" + (r.open ?? "?"), "nm=" + (r.nm ?? "?"), r.file);
    }
  });
}
