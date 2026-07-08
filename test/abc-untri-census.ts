// SPDX-License-Identifier: AGPL-3.0-only
// Census of the untriangulated-face class (rec ② scoping): for every ABC model whose run4 bucket
// is untriangulated-face, tessellate with the trace hook and record each face that no mesher
// could realize — surface kind, loop structure (edges per loop), surface periodicity/closedness,
// and whether the face is its solid's sole face. Output clusters pick the sub-class to attack.
//
//   node test/abc-untri-census.ts            # orchestrate: 6 children over the untri list
//   node test/abc-untri-census.ts --child i n # child mode
import { readFileSync, appendFileSync, existsSync, unlinkSync } from "node:fs";
import { spawn } from "node:child_process";
import { join } from "node:path";
import { buildBrep, type BrepModel } from "../src/brep/build.ts";
import { tessellate } from "../src/mesh/tessellate.ts";
import { makeSurface } from "../src/geom/surfaces.ts";

const ABC = "abc_0000_step_v00";
const OUT = process.env.UNTRI_OUT ?? "out/abc-run4/untri-census.jsonl";
const RESULTS = process.env.UNTRI_RESULTS ?? "out/abc-run4/results.jsonl";

function brepDiag(brep: BrepModel): number {
  let mnx = Infinity, mny = Infinity, mnz = Infinity, mxx = -Infinity, mxy = -Infinity, mxz = -Infinity;
  for (const e of brep.edges.values()) {
    for (const p of [e.v0, e.v1]) {
      if (p[0]! < mnx) mnx = p[0]!; if (p[1]! < mny) mny = p[1]!; if (p[2]! < mnz) mnz = p[2]!;
      if (p[0]! > mxx) mxx = p[0]!; if (p[1]! > mxy) mxy = p[1]!; if (p[2]! > mxz) mxz = p[2]!;
    }
  }
  return Number.isFinite(mnx) ? Math.hypot(mxx - mnx, mxy - mny, mxz - mnz) : 0;
}

function censusOne(path: string): object {
  const src = readFileSync(path, "utf8");
  const brep = buildBrep(src);
  const diag = brepDiag(brep);
  const chordTol = diag > 0 ? 5e-4 * diag : 0.01;
  const targetEdge = diag > 0 ? 0.05 * diag : 1;
  const untri = new Set<number>();
  const res = tessellate(brep, {
    chordTol, targetEdge, normalDev: 15 * Math.PI / 180,
    trace: (fid, mesher) => { if (mesher === "untriangulated") untri.add(fid); },
  });
  const faces: object[] = [];
  for (const solid of brep.solids) {
    for (const f of solid.faces) {
      if (!untri.has(f.faceId)) continue;
      let per = "";
      try {
        const s = makeSurface(brep.table, f.surfaceId, solid.scale ?? brep.scale, brep.units.radPerAngle) as
          { periodicU?: boolean; periodicV?: boolean; closedU?: boolean; closedV?: boolean } | null;
        if (s) per = `${s.periodicU ? "Pu" : ""}${s.periodicV ? "Pv" : ""}${s.closedU ? "Cu" : ""}${s.closedV ? "Cv" : ""}`;
      } catch { per = "?"; }
      faces.push({
        fid: f.faceId,
        kind: f.surfaceKind,
        loops: f.loops.map((l) => l.edges.length),
        outer: f.loops.findIndex((l) => l.outer),
        sole: solid.faces.length === 1,
        per,
      });
    }
  }
  const tris = res.mesh.indices.length / 3;
  return { file: path.replace(/^.*abc_0000_step_v00[\\/]/, ""), diag, facesTotal: res.stats.facesTotal, untriFaces: faces.length, tris, faces };
}

const argv = process.argv.slice(2);
if (argv[0] === "--child") {
  const i0 = parseInt(argv[1]!, 10), stride = parseInt(argv[2]!, 10);
  const files = readFileSync(RESULTS, "utf8").split(/\r?\n/).filter(Boolean).map((l) => JSON.parse(l))
    .filter((r) => r.bucket === "untriangulated-face").map((r) => r.file as string);
  for (let i = i0; i < files.length; i += stride) {
    let rec: object;
    try { rec = censusOne(join(ABC, files[i]!)); }
    catch (e) { rec = { file: files[i], err: e instanceof Error ? e.message : String(e) }; }
    appendFileSync(OUT, JSON.stringify(rec) + "\n");
    console.log(`[${i0}] ${i}: done`);
  }
} else {
  if (existsSync(OUT)) unlinkSync(OUT);
  const N = 6;
  let done = 0;
  for (let k = 0; k < N; k++) {
    const c = spawn(process.execPath, ["test/abc-untri-census.ts", "--child", String(k), String(N)], { stdio: "inherit" });
    c.on("exit", () => { if (++done === N) console.log("CENSUS DONE"); });
  }
}
