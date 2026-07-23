// SPDX-License-Identifier: AGPL-3.0-only
// For each edge of face <fid>, list the other faces that reference the same edge.
//   node test/abc-edgemates.ts <step-file> <fid>
import { readFileSync } from "node:fs";
import { buildBrep } from "../src/brep/build.ts";

const brep = buildBrep(readFileSync(process.argv[2]!, "utf8"));
const target = Number(process.argv[3]);
const users = new Map<number, { fid: number; kind: string; loops: string }[]>();
for (const s of brep.solids) for (const f of s.faces) {
  for (const lp of f.loops) for (const oe of lp.edges) {
    const arr = users.get(oe.edgeId) ?? users.set(oe.edgeId, []).get(oe.edgeId)!;
    arr.push({ fid: f.faceId, kind: f.surfaceKind, loops: f.loops.map((l) => l.edges.length).join("/") });
  }
}
for (const s of brep.solids) for (const f of s.faces) {
  if (f.faceId !== target) continue;
  for (let li = 0; li < f.loops.length; li++) {
    for (const oe of f.loops[li]!.edges) {
      const mates = (users.get(oe.edgeId) ?? []).filter((u) => u.fid !== target);
      console.log(`loop ${li} edge ${oe.edgeId}: mates = ${mates.map((m) => `fid=${m.fid} ${m.kind} loops=${m.loops}`).join("; ") || "NONE"}`);
    }
  }
}
