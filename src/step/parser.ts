// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — STEP Part 21 parser: token stream -> entity instance table.
import { tokenize, extractDataSection, type Token, type TokKind } from "./tokenizer.ts";

export type Param =
  | { k: "num"; v: number }
  | { k: "str"; v: string }
  | { k: "ref"; v: number }
  | { k: "enum"; v: string }
  | { k: "none" }              // $  (unset)
  | { k: "derived" }           // *  (derived in supertype)
  | { k: "list"; v: Param[] }
  | { k: "typed"; type: string; params: Param[] }; // e.g. LENGTH_MEASURE(0.01)

export interface EntityRecord { type: string; params: Param[]; }
/** A simple instance, or a "complex" instance: an AND-combination of partial records. */
export type Entity = EntityRecord | { complex: EntityRecord[] };

export interface StepModel {
  entities: Map<number, Entity>;
  /** Instances that were malformed and could not be read at all — skipped, not in `entities`. */
  dropped?: number[];
  /** Instances read only by tolerating an EMPTY parameter slot (`#1=EDGE_CURVE('',#2,#3,#0,);`
   *  — a trailing comma with nothing after it), which is read as unset (`$`). */
  repaired?: number[];
}

export function parseStep(src: string): StepModel {
  const toks = tokenize(extractDataSection(src));
  let p = 0;
  let lenient = false; // empty parameter slots read as `$` (per-entity recovery retry only)
  const peek = (): Token => toks[p]!;
  const expect = (k: TokKind): Token => {
    const t = toks[p++]!;
    if (t.kind !== k) {
      throw new Error(`STEP parse: expected ${k}, got ${t.kind} '${t.text}' @${t.pos}`);
    }
    return t;
  };

  const parseValue = (): Param => {
    const t = peek();
    switch (t.kind) {
      case "str": p++; return { k: "str", v: t.text };
      case "ref": p++; return { k: "ref", v: Number(t.text) };
      case "num": p++; return { k: "num", v: Number(t.text) };
      case "enum": p++; return { k: "enum", v: t.text };
      case "dollar": p++; return { k: "none" };
      case "star": p++; return { k: "derived" };
      case "lparen": {
        p++;
        const v = parseParamList();
        expect("rparen");
        return { k: "list", v };
      }
      case "kw": {
        const type = t.text; p++;
        expect("lparen");
        const params = parseParamList();
        expect("rparen");
        return { k: "typed", type, params };
      }
      default:
        throw new Error(`STEP parse: unexpected ${t.kind} '${t.text}' @${t.pos}`);
    }
  };

  function parseParamList(): Param[] {
    const params: Param[] = [];
    if (peek().kind === "rparen") return params;
    for (;;) {
      const k = peek().kind;
      params.push(lenient && (k === "comma" || k === "rparen") ? { k: "none" } : parseValue());
      if (peek().kind === "comma") { p++; continue; }
      break;
    }
    return params;
  }

  const parseRecord = (): EntityRecord => {
    const type = expect("kw").text;
    expect("lparen");
    const params = parseParamList();
    expect("rparen");
    return { type, params };
  };

  const parseInstance = (): Entity => {
    let entity: Entity;
    if (peek().kind === "lparen") {
      p++; // complex entity: ( REC REC ... )
      const recs: EntityRecord[] = [];
      while (peek().kind === "kw") recs.push(parseRecord());
      expect("rparen");
      entity = { complex: recs };
    } else {
      entity = parseRecord();
    }
    expect("semi");
    return entity;
  };

  // Recovery is PER INSTANCE: one malformed record must not abort the whole file (ABC exports
  // carry `EDGE_CURVE('', #a, #b, #0,   );` — an empty slot after a trailing comma). A failed
  // instance is re-read once with empty slots taken as `$`; if that fails too it is skipped up to
  // its terminating `;` and recorded as dropped. Consumers of a dropped or broken instance see a
  // missing entity and degrade locally (the B-rep builder tolerates a missing edge curve).
  const entities = new Map<number, Entity>();
  const dropped: number[] = [], repaired: number[] = [];
  while (peek().kind !== "eof") {
    if (peek().kind !== "ref") { p++; continue; } // tolerate stray tokens
    const id = Number(expect("ref").text);
    if (peek().kind !== "eq") continue; // a bare reference outside any instance — stray token
    p++;
    const start = p;
    try {
      entities.set(id, parseInstance());
      continue;
    } catch { /* fall through to recovery */ }
    p = start; lenient = true;
    try { entities.set(id, parseInstance()); repaired.push(id); }
    catch {
      // Skip to this instance's terminator. `;` only ever appears as a token at instance level
      // (inside strings it is part of the str token), so the next semi token ends it.
      p = start;
      while (peek().kind !== "semi" && peek().kind !== "eof") p++;
      if (peek().kind === "semi") p++;
      dropped.push(id);
    }
    lenient = false;
  }

  const model: StepModel = { entities };
  if (dropped.length) model.dropped = dropped;
  if (repaired.length) model.repaired = repaired;
  return model;
}

// re-export for callers that only want the token kind type
export type { TokKind } from "./tokenizer.ts";
