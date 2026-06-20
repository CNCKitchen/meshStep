// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — ISO-10303-21 (STEP Part 21) tokenizer.

export type TokKind =
  | "kw" | "ref" | "num" | "str" | "enum"
  | "lparen" | "rparen" | "comma" | "semi" | "eq" | "dollar" | "star" | "eof";

export interface Token {
  kind: TokKind;
  /** str: decoded content · ref: digits · num: raw · enum: inner word · kw: name. */
  text: string;
  pos: number;
}

/** Extract the DATA section body (between `DATA;` and the following `ENDSEC;`). */
export function extractDataSection(src: string): string {
  const di = src.indexOf("DATA;");
  if (di < 0) throw new Error("STEP: no DATA section found");
  const end = src.indexOf("ENDSEC;", di);
  if (end < 0) throw new Error("STEP: DATA section not terminated");
  return src.slice(di + "DATA;".length, end);
}

const isDigit = (c: string): boolean => c >= "0" && c <= "9";
const isAlpha = (c: string): boolean =>
  (c >= "A" && c <= "Z") || (c >= "a" && c <= "z") || c === "_";
const isNumChar = (c: string): boolean =>
  isDigit(c) || c === "." || c === "e" || c === "E" || c === "+" || c === "-";

export function tokenize(src: string): Token[] {
  const toks: Token[] = [];
  let i = 0;
  const n = src.length;
  while (i < n) {
    const c = src[i]!;
    if (c === " " || c === "\t" || c === "\r" || c === "\n") { i++; continue; }
    // block comment /* ... */
    if (c === "/" && src[i + 1] === "*") {
      const e = src.indexOf("*/", i + 2);
      i = e < 0 ? n : e + 2;
      continue;
    }
    const start = i;
    // string literal '...'  ('' is an escaped quote; may span newlines)
    if (c === "'") {
      i++;
      let s = "";
      while (i < n) {
        if (src[i] === "'") {
          if (src[i + 1] === "'") { s += "'"; i += 2; continue; }
          i++; break;
        }
        s += src[i++];
      }
      toks.push({ kind: "str", text: s, pos: start });
      continue;
    }
    // entity reference #123
    if (c === "#") {
      i++;
      let s = "";
      while (i < n && isDigit(src[i]!)) s += src[i++];
      toks.push({ kind: "ref", text: s, pos: start });
      continue;
    }
    // enumeration .WORD.
    if (c === "." && isAlpha(src[i + 1] ?? "")) {
      i++;
      let s = "";
      while (i < n && src[i] !== ".") s += src[i++];
      i++; // consume closing dot
      toks.push({ kind: "enum", text: s, pos: start });
      continue;
    }
    // number (digit, or signed/decimal start followed by a digit)
    if (isDigit(c) || ((c === "+" || c === "-" || c === ".") && isDigit(src[i + 1] ?? ""))) {
      let s = src[i++]!;
      while (i < n && isNumChar(src[i]!)) s += src[i++];
      toks.push({ kind: "num", text: s, pos: start });
      continue;
    }
    // keyword / typename
    if (isAlpha(c)) {
      let s = "";
      while (i < n && (isAlpha(src[i]!) || isDigit(src[i]!))) s += src[i++];
      toks.push({ kind: "kw", text: s, pos: start });
      continue;
    }
    // single-char punctuation
    i++;
    if (c === "(") toks.push({ kind: "lparen", text: c, pos: start });
    else if (c === ")") toks.push({ kind: "rparen", text: c, pos: start });
    else if (c === ",") toks.push({ kind: "comma", text: c, pos: start });
    else if (c === ";") toks.push({ kind: "semi", text: c, pos: start });
    else if (c === "=") toks.push({ kind: "eq", text: c, pos: start });
    else if (c === "$") toks.push({ kind: "dollar", text: c, pos: start });
    else if (c === "*") toks.push({ kind: "star", text: c, pos: start });
    // else: stray character, ignore
  }
  toks.push({ kind: "eof", text: "", pos: n });
  return toks;
}
