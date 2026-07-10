// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — STEP presentation colors: STYLED_ITEM / OVER_RIDING_STYLED_ITEM chains resolved to
// per-face and per-solid sRGB colors. AP214/AP242 exporters attach a color as
//   STYLED_ITEM -> PRESENTATION_STYLE_ASSIGNMENT -> SURFACE_STYLE_USAGE -> SURFACE_SIDE_STYLE
//   -> SURFACE_STYLE_FILL_AREA -> FILL_AREA_STYLE -> FILL_AREA_STYLE_COLOUR -> COLOUR_RGB
// where the styled item targets an ADVANCED_FACE (face color), a shell, or a whole solid body
// (default color for all its faces); OVER_RIDING_STYLED_ITEM re-colors an item that inherited
// its style, so it always wins over a plain STYLED_ITEM on the same item. Colors come out
// palette-indexed so consumers can group faces by shared color without comparing float triples.
import { Table, ref, refList, list, num, str } from "./entities.ts";
import type { EntityRecord, Param } from "./parser.ts";
import type { BSolid } from "../brep/build.ts";

/** sRGB color, each channel 0..1 (as stored in COLOUR_RGB). */
export type RGB = [number, number, number];

export interface ModelColors {
  /** Distinct colors present in the model (sRGB, 0..1). */
  palette: RGB[];
  /** Palette index per B-rep face — keys are the same ADVANCED_FACE entity ids as the values of
   * MeshResult.faceOfTri, so `palette[faceColor.get(faceOfTri[t])]` colors triangle t. Face-level
   * styles override the owning solid's color; unstyled faces are absent from the map. */
  faceColor: Map<number, number>;
  /** Palette index per solid (body) — keys match MeshResult.solidOfTri values. Only bodies styled
   * as a whole appear here; their faces already carry the color in faceColor. */
  solidColor: Map<number, number>;
}

// ISO 10303-46 pre-defined colour names (DRAUGHTING_PRE_DEFINED_COLOUR).
const PREDEFINED: Record<string, RGB> = {
  red: [1, 0, 0], green: [0, 1, 0], blue: [0, 0, 1], yellow: [1, 1, 0],
  magenta: [1, 0, 1], cyan: [0, 1, 1], black: [0, 0, 0], white: [1, 1, 1],
};

const resolveColour = (t: Table, id: number): RGB | null => {
  const rgb = t.sub(id, "COLOUR_RGB");
  if (rgb) return [num(rgb.params[1]!), num(rgb.params[2]!), num(rgb.params[3]!)];
  const pre = t.sub(id, "DRAUGHTING_PRE_DEFINED_COLOUR") ?? t.sub(id, "PRE_DEFINED_COLOUR");
  return pre ? (PREDEFINED[str(pre.params[0]!).toLowerCase()] ?? null) : null;
};

/** Surface colour of one PRESENTATION_STYLE_ASSIGNMENT: the fill-area colour of a
 * SURFACE_STYLE_USAGE's side style, falling back to the rendering colour. */
function colourOfPSA(t: Table, psaId: number): RGB | null {
  const psa = t.sub(psaId, "PRESENTATION_STYLE_ASSIGNMENT") ?? t.sub(psaId, "PRESENTATION_STYLE_BY_CONTEXT");
  if (!psa || psa.params[0]?.k !== "list") return null;
  for (const s of list(psa.params[0]!)) {
    if (s.k !== "ref") continue;
    const ssu = t.sub(s.v, "SURFACE_STYLE_USAGE");
    if (!ssu || ssu.params[1]?.k !== "ref") continue;
    const side = t.sub(ref(ssu.params[1]!), "SURFACE_SIDE_STYLE");
    if (!side) continue;
    let rendering: RGB | null = null;
    for (const el of refList(side.params[1]!)) {
      const fill = t.sub(el, "SURFACE_STYLE_FILL_AREA");
      if (fill) {
        const fas = t.sub(ref(fill.params[0]!), "FILL_AREA_STYLE");
        if (fas) for (const item of refList(fas.params[1]!)) {
          const fasc = t.sub(item, "FILL_AREA_STYLE_COLOUR");
          if (fasc) {
            const c = resolveColour(t, ref(fasc.params[1]!));
            if (c) return c;
          }
        }
      }
      const rend = t.sub(el, "SURFACE_STYLE_RENDERING") ?? t.sub(el, "SURFACE_STYLE_RENDERING_WITH_PROPERTIES");
      if (rend && rend.params[1]?.k === "ref") rendering ??= resolveColour(t, ref(rend.params[1]!));
    }
    if (rendering) return rendering;
  }
  return null;
}

/**
 * Resolve every styled item in the file to a color and compose the per-face result over the
 * built solids: face-level styles beat shell-level ones, both beat the solid's body color,
 * and OVER_RIDING_STYLED_ITEM beats STYLED_ITEM at the same level. Returns null when the file
 * carries no surface colors. Malformed style chains are skipped, never fatal.
 */
export function extractColors(table: Table, solids: readonly Pick<BSolid, "id" | "faces">[]): ModelColors | null {
  type Entry = { rank: number; rgb: RGB };
  const faceRaw = new Map<number, Entry>();
  const solidRaw = new Map<number, Entry>();
  const put = (m: Map<number, Entry>, id: number, rank: number, rgb: RGB): void => {
    const cur = m.get(id);
    if (!cur || rank > cur.rank) m.set(id, { rank, rgb });
  };

  const unwrapFace = (id: number): number => {
    while (table.typeOf(id) === "ORIENTED_FACE") id = ref(table.record(id).params[2]!);
    return id;
  };
  const shellFaces = (id: number): number[] => {
    const or = table.sub(id, "ORIENTED_CLOSED_SHELL");
    if (or) return shellFaces(ref(or.params[2]!));
    const sh = table.sub(id, "CLOSED_SHELL") ?? table.sub(id, "OPEN_SHELL");
    return sh ? refList(sh.params[1]!) : [];
  };

  // STYLED_ITEM(name, (style...), item); OVER_RIDING_STYLED_ITEM adds a 4th param (the style it
  // overrides) after the same first three.
  const apply = (rec: EntityRecord, overriding: boolean): void => {
    if (rec.params[1]?.k !== "list" || rec.params[2]?.k !== "ref") return;
    let rgb: RGB | null = null;
    for (const s of list(rec.params[1]!) as Param[]) {
      if (s.k === "ref") rgb = colourOfPSA(table, s.v);
      if (rgb) break;
    }
    if (!rgb) return;
    const item = ref(rec.params[2]!);
    const bump = overriding ? 1 : 0;
    switch (table.typeOf(item)) {
      case "ORIENTED_FACE":
      case "FACE_SURFACE":
      case "ADVANCED_FACE":
        put(faceRaw, unwrapFace(item), 3 + bump, rgb);
        break;
      case "CLOSED_SHELL":
      case "OPEN_SHELL":
      case "ORIENTED_CLOSED_SHELL":
        for (const f of shellFaces(item)) put(faceRaw, unwrapFace(f), 1 + bump, rgb);
        break;
      case "MANIFOLD_SOLID_BREP":
      case "BREP_WITH_VOIDS":
      case "SHELL_BASED_SURFACE_MODEL":
        put(solidRaw, item, 1 + bump, rgb);
        break;
    }
  };
  for (const [, rec] of table.byType("STYLED_ITEM")) { try { apply(rec, false); } catch { /* skip malformed */ } }
  for (const [, rec] of table.byType("OVER_RIDING_STYLED_ITEM")) { try { apply(rec, true); } catch { /* skip malformed */ } }
  if (faceRaw.size === 0 && solidRaw.size === 0) return null;

  const palette: RGB[] = [];
  const index = new Map<string, number>();
  const idx = (c: RGB): number => {
    const k = `${c[0]},${c[1]},${c[2]}`;
    let i = index.get(k);
    if (i === undefined) { i = palette.length; palette.push(c); index.set(k, i); }
    return i;
  };
  const faceColor = new Map<number, number>();
  const solidColor = new Map<number, number>();
  for (const solid of solids) {
    const sc = solidRaw.get(solid.id);
    if (sc) solidColor.set(solid.id, idx(sc.rgb));
    for (const face of solid.faces) {
      const fc = faceRaw.get(face.faceId) ?? sc;
      if (fc) faceColor.set(face.faceId, idx(fc.rgb));
    }
  }
  if (faceColor.size === 0 && solidColor.size === 0) return null;
  return { palette, faceColor, solidColor };
}
