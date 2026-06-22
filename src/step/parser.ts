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
}

export function parseStep(src: string): StepModel {
  const toks = tokenize(extractDataSection(src));
  let p = 0;
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
      params.push(parseValue());
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

  const entities = new Map<number, Entity>();
  while (peek().kind !== "eof") {
    if (peek().kind !== "ref") { p++; continue; } // tolerate stray tokens
    const id = Number(expect("ref").text);
    expect("eq");
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
    entities.set(id, entity);
  }

  return { entities };
}

// re-export for callers that only want the token kind type
export type { TokKind } from "./tokenizer.ts";
