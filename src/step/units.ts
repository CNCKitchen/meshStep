// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — STEP length-unit detection; returns the scale to millimetres.
import type { Table } from "./entities.ts";

const SI_PREFIX: Record<string, number> = {
  EXA: 1e18, PETA: 1e15, TERA: 1e12, GIGA: 1e9, MEGA: 1e6, KILO: 1e3,
  HECTO: 1e2, DECA: 1e1, DECI: 1e-1, CENTI: 1e-2, MILLI: 1e-3,
  MICRO: 1e-6, NANO: 1e-9, PICO: 1e-12, FEMTO: 1e-15, ATTO: 1e-18,
};

export interface Units {
  /** Multiply STEP length values by this to obtain millimetres. */
  mmPerUnit: number;
  label: string;
}

/**
 * Find the model's length unit and return the factor to millimetres. Handles the common
 * SI_UNIT case (e.g. MILLI METRE). Falls back to assuming millimetres.
 */
export function detectUnits(table: Table): Units {
  for (const [, e] of table.model.entities) {
    if (!("complex" in e)) continue;
    if (!e.complex.some((r) => r.type === "LENGTH_UNIT")) continue;
    const si = e.complex.find((r) => r.type === "SI_UNIT");
    if (!si) continue;
    const nameP = si.params[1];
    const name = nameP && nameP.k === "enum" ? nameP.v : "METRE";
    if (name !== "METRE") continue;
    let factor = 1000; // metre -> mm
    const pfx = si.params[0];
    let prefixLabel = "";
    if (pfx && pfx.k === "enum") {
      factor *= SI_PREFIX[pfx.v] ?? 1;
      prefixLabel = pfx.v.toLowerCase();
    }
    return { mmPerUnit: factor, label: `${prefixLabel}metre` };
  }
  return { mmPerUnit: 1, label: "assumed mm" };
}
