// SPDX-License-Identifier: AGPL-3.0-only
// Cut ONE named body out of a STEP assembly into a standalone single-part STEP file.
//
//   node test/extract-part.ts <assembly.step> "<body name>" [out.step]
//
// The body's entity closure (every instance it references, transitively) is copied VERBATIM —
// raw instance text with its original #ids — so the geometry is bit-identical to the assembly's
// (re-serialising parsed numbers would lose the REAL/INTEGER distinction and could round). Only
// the wrapper is new: one shape representation holding just this body, a minimal product chain
// naming it, and the presentation styles that target it. The part comes out in its own local
// frame (no assembly placement), which is the frame meshStep meshes it in anyway.
import { readFileSync, writeFileSync } from "node:fs";
import { basename } from "node:path";
import { parseStep, type Entity, type Param } from "../src/step/parser.ts";
import { Table } from "../src/step/entities.ts";

const [file, name, outArg] = process.argv.slice(2);
if (!file || !name) {
  console.error('usage: node test/extract-part.ts <assembly.step> "<body name>" [out.step]');
  process.exit(2);
}
const src = readFileSync(file, "utf8");
const table = new Table(parseStep(src));

/** Raw text of every DATA instance, keyed by id. A `;` ends an instance only outside a string
 * literal ('' is an escaped quote and toggles twice, so plain toggling is correct) and outside a
 * comment. */
function rawInstances(text: string): Map<number, string> {
  const d0 = text.indexOf("DATA;") + "DATA;".length;
  const d1 = text.indexOf("ENDSEC;", d0);
  const out = new Map<number, string>();
  let start = d0, inStr = false;
  for (let i = d0; i < d1; i++) {
    const c = text.charCodeAt(i);
    if (inStr) { if (c === 39) inStr = false; continue; }
    if (c === 39) { inStr = true; continue; }
    if (c === 47 && text.charCodeAt(i + 1) === 42) { const e = text.indexOf("*/", i + 2); i = e < 0 ? d1 : e + 1; continue; }
    if (c !== 59) continue;
    const inst = text.slice(start, i + 1).trim();
    start = i + 1;
    const m = /^#(\d+)\s*=/.exec(inst);
    if (m) out.set(Number(m[1]), inst);
  }
  return out;
}

const refsOf = (e: Entity): number[] => {
  const out: number[] = [];
  const walk = (p: Param): void => {
    if (p.k === "ref") out.push(p.v);
    else if (p.k === "list") p.v.forEach(walk);
    else if (p.k === "typed") p.params.forEach(walk);
  };
  for (const r of "complex" in e ? e.complex : [e]) r.params.forEach(walk);
  return out;
};
const closure = (roots: number[], into = new Set<number>()): Set<number> => {
  const stack = [...roots];
  while (stack.length) {
    const id = stack.pop()!;
    if (into.has(id) || !table.model.entities.has(id)) continue;
    into.add(id);
    for (const r of refsOf(table.get(id))) stack.push(r);
  }
  return into;
};

// The body, and the representation that carries it (for its unit/uncertainty context).
const SOLID_TYPES = ["MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS", "SHELL_BASED_SURFACE_MODEL"];
const solids: number[] = [];
for (const ty of SOLID_TYPES) for (const [id, rec] of table.byType(ty)) {
  if (rec.params[0]?.k === "str" && rec.params[0].v === name) solids.push(id);
}
if (solids.length === 0) { console.error(`no body named "${name}" in ${file}`); process.exit(1); }
if (solids.length > 1) console.error(`note: ${solids.length} bodies are named "${name}" — all are extracted`);
let ctx = -1, repType = "ADVANCED_BREP_SHAPE_REPRESENTATION";
for (const ty of ["ADVANCED_BREP_SHAPE_REPRESENTATION", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION"]) {
  for (const [, rep] of table.byType(ty)) {
    if (ctx >= 0 || rep.params[1]?.k !== "list" || rep.params[2]?.k !== "ref") continue;
    if (rep.params[1].v.some((p) => p.k === "ref" && solids.includes(p.v))) { ctx = rep.params[2].v; repType = ty; }
  }
}
if (ctx < 0) { console.error("the body is not listed in any shape representation"); process.exit(1); }

const keep = closure([...solids, ctx]);
// Presentation styles aimed at anything inside the body (the solid itself, a shell, a face).
const styled: number[] = [];
for (const ty of ["STYLED_ITEM", "OVER_RIDING_STYLED_ITEM"]) for (const [id, rec] of table.byType(ty)) {
  if (rec.params[2]?.k === "ref" && keep.has(rec.params[2].v)) styled.push(id);
}
closure(styled, keep);

const raw = rawInstances(src);
let next = 0;
for (const id of table.model.entities.keys()) if (id > next) next = id;
const fresh = (): number => ++next;
const q = (s: string): string => `'${s.replace(/'/g, "''")}'`;
const lines: string[] = [];
for (const id of [...keep].sort((a, b) => a - b)) {
  const t = raw.get(id);
  if (!t) { console.error(`instance #${id} has no raw text — file not extractable`); process.exit(1); }
  lines.push(t);
}
const rep = fresh(), app = fresh(), apd = fresh(), pctx = fresh(), prod = fresh(), pdf = fresh(), pdc = fresh();
const pd = fresh(), pds = fresh(), sdr = fresh(), cat = fresh();
const schema = /FILE_SCHEMA\s*\(\s*\(([\s\S]*?)\)\s*\)\s*;/.exec(src)?.[1]?.trim() ?? "'AUTOMOTIVE_DESIGN'";
lines.push(
  `#${rep}=${repType}('',(${solids.map((s) => `#${s}`).join(",")}),#${ctx});`,
  `#${app}=APPLICATION_CONTEXT('managed model based 3d engineering');`,
  `#${apd}=APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering',2014,#${app});`,
  `#${pctx}=PRODUCT_CONTEXT('',#${app},'mechanical');`,
  `#${prod}=PRODUCT(${q(name)},${q(name)},'',(#${pctx}));`,
  `#${pdf}=PRODUCT_DEFINITION_FORMATION('','',#${prod});`,
  `#${pdc}=PRODUCT_DEFINITION_CONTEXT('part definition',#${app},'design');`,
  `#${pd}=PRODUCT_DEFINITION('design','',#${pdf},#${pdc});`,
  `#${pds}=PRODUCT_DEFINITION_SHAPE('','',#${pd});`,
  `#${sdr}=SHAPE_DEFINITION_REPRESENTATION(#${pds},#${rep});`,
  `#${cat}=PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#${prod}));`,
);
if (styled.length) {
  lines.push(`#${fresh()}=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(${styled.map((s) => `#${s}`).join(",")}),#${ctx});`);
}
const out = outArg ?? `${name}.step`;
writeFileSync(out, [
  "ISO-10303-21;", "HEADER;",
  `FILE_DESCRIPTION((${q(`${name}, extracted from ${basename(file)} by meshStep test/extract-part.ts`)}),'2;1');`,
  `FILE_NAME(${q(basename(out))},'${new Date().toISOString().slice(0, 19)}',(''),(''),'meshStep extract-part','meshStep','');`,
  `FILE_SCHEMA((${schema}));`,
  "ENDSEC;", "DATA;", ...lines, "ENDSEC;", "END-ISO-10303-21;", "",
].join("\n"));
console.log(`${out}: ${solids.length} body, ${keep.size} instances (${styled.length} styles) of ${table.model.entities.size}`);
