// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — ISO-10303-21 HEADER parser. Extracts the provenance fields (originating CAD system,
// schema, author, timestamp) that a viewer shows as file info. Independent of the DATA-section
// parser: the header is a handful of records before `DATA;`, so it is cheap to read on file load,
// before (or without) tessellation.
import { tokenize, type Token } from "./tokenizer.ts";

export interface StepHeader {
  /** FILE_DESCRIPTION text (all description strings joined). */
  description?: string;
  /** FILE_NAME[0] — the model/file name recorded by the exporter. */
  name?: string;
  /** FILE_NAME[1] — ISO-8601 timestamp of the export. */
  timeStamp?: string;
  /** FILE_NAME[2] — author(s), joined. */
  author?: string;
  /** FILE_NAME[3] — organization(s), joined. */
  organization?: string;
  /** FILE_NAME[4] — preprocessor version string. */
  preprocessor?: string;
  /** FILE_NAME[5] — originating system: the CAD program that wrote the file. */
  originatingSystem?: string;
  /** FILE_SCHEMA[0] raw schema identifier (e.g. `AUTOMOTIVE_DESIGN { ... }`). */
  schema?: string;
  /** Friendly application-protocol label derived from the schema (AP203 / AP214 / AP242 / …). */
  schemaLabel?: string;
}

/** A header value is a string leaf, a (possibly nested) list, or null ($ / *). */
type HVal = string | HVal[] | null;

/** Decode the common Part-21 control directives so display strings read cleanly. */
function decodeStepStr(s: string): string {
  const units = (h: string, w: number): string => {
    let out = "";
    for (let i = 0; i + w <= h.length; i += w) out += String.fromCodePoint(parseInt(h.slice(i, i + w), 16));
    return out;
  };
  return s
    .replace(/\\X2\\([0-9A-Fa-f]+)\\X0\\/g, (_m, h: string) => units(h, 4)) // UTF-16
    .replace(/\\X4\\([0-9A-Fa-f]+)\\X0\\/g, (_m, h: string) => units(h, 8)) // UTF-32
    .replace(/\\X\\([0-9A-Fa-f]{2})/g, (_m, h: string) => String.fromCharCode(parseInt(h, 16)))
    .replace(/\\S\\(.)/g, (_m, c: string) => c); // ISO-8859 upper-half shift (approximate)
}

function readValue(toks: Token[], p: number): [HVal, number] {
  const t = toks[p];
  if (!t) return [null, p];
  if (t.kind === "str") return [decodeStepStr(t.text), p + 1];
  if (t.kind === "num" || t.kind === "enum") return [t.text, p + 1];
  if (t.kind === "lparen") {
    const arr: HVal[] = [];
    p++;
    while (toks[p] && toks[p]!.kind !== "rparen" && toks[p]!.kind !== "eof") {
      let v: HVal;
      [v, p] = readValue(toks, p);
      arr.push(v);
      if (toks[p]?.kind === "comma") p++;
    }
    if (toks[p]?.kind === "rparen") p++;
    return [arr, p];
  }
  if (t.kind === "kw" && toks[p + 1]?.kind === "lparen") {
    // typed value like FOO(...) — skip its payload, keep nothing (unused in headers).
    const [, np] = readValue(toks, p + 1);
    return [null, np];
  }
  return [null, p + 1]; // $ / * / stray punctuation
}

/** Flatten a header value into a single trimmed string; drops empty pieces. */
function flat(v: HVal | undefined): string | undefined {
  if (v == null) return undefined;
  if (typeof v === "string") { const s = v.trim(); return s || undefined; }
  const parts = v.map(flat).filter((s): s is string => !!s);
  return parts.length ? parts.join(", ") : undefined;
}

/** Map a STEP schema identifier to its application-protocol short name. */
function schemaLabel(schema: string): string {
  const u = schema.toUpperCase();
  if (u.includes("AP242") || u.includes("MANAGED_MODEL_BASED")) return "AP242";
  if (u.includes("AUTOMOTIVE_DESIGN")) return "AP214";
  if (u.includes("CONFIG_CONTROL_DESIGN")) return "AP203";
  const m = u.match(/10303\s+(\d{3})/); // identifier braces carry the AP number
  if (m) return `AP${m[1]}`;
  const ap = u.match(/AP(\d{3})/);
  if (ap) return `AP${ap[1]}`;
  return schema.split(/[ {]/)[0] || schema;
}

/**
 * Parse the ISO-10303-21 HEADER section. Returns whatever provenance fields are present; missing or
 * malformed headers yield an empty object rather than throwing (a viewer should still render the geometry).
 */
export function parseStepHeader(src: string): StepHeader {
  try {
    const hi = src.indexOf("HEADER;");
    if (hi < 0) return {};
    const endIdx = src.indexOf("ENDSEC;", hi);
    const block = src.slice(hi + "HEADER;".length, endIdx < 0 ? undefined : endIdx);
    const toks = tokenize(block);

    const records = new Map<string, HVal[]>();
    let p = 0;
    while (toks[p] && toks[p]!.kind !== "eof") {
      const t = toks[p]!;
      if (t.kind === "kw" && toks[p + 1]?.kind === "lparen") {
        const [params, np] = readValue(toks, p + 1);
        if (Array.isArray(params)) records.set(t.text.toUpperCase(), params);
        p = np;
        continue;
      }
      p++;
    }

    const fn = records.get("FILE_NAME") ?? [];
    const fd = records.get("FILE_DESCRIPTION") ?? [];
    const fs = records.get("FILE_SCHEMA") ?? [];

    const h: StepHeader = {
      description: flat(fd[0]),
      name: flat(fn[0]),
      timeStamp: flat(fn[1]),
      author: flat(fn[2]),
      organization: flat(fn[3]),
      preprocessor: flat(fn[4]),
      originatingSystem: flat(fn[5]),
      schema: flat(fs[0]),
    };
    if (h.schema) h.schemaLabel = schemaLabel(h.schema);
    return h;
  } catch {
    return {};
  }
}
