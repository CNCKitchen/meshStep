// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — STEP unit detection: length (scale to millimetres) and plane angle (scale to radians).
import type { Table } from "./entities.ts";
import type { EntityRecord } from "./parser.ts";
import { num, ref } from "./entities.ts";

const SI_PREFIX: Record<string, number> = {
  EXA: 1e18, PETA: 1e15, TERA: 1e12, GIGA: 1e9, MEGA: 1e6, KILO: 1e3,
  HECTO: 1e2, DECA: 1e1, DECI: 1e-1, CENTI: 1e-2, MILLI: 1e-3,
  MICRO: 1e-6, NANO: 1e-9, PICO: 1e-12, FEMTO: 1e-15, ATTO: 1e-18,
};

export interface Units {
  /** Multiply STEP length values by this to obtain millimetres. */
  mmPerUnit: number;
  /** Multiply STEP plane-angle values by this to obtain radians (1 for radian, π/180 for degree). */
  radPerAngle: number;
  label: string;
}

/** SI_UNIT(prefix, name) -> factor to mm (length) or radians (angle); null if not recognised. */
function siFactor(si: EntityRecord, kind: "length" | "angle"): number | null {
  const nameP = si.params[1];
  const name = nameP && nameP.k === "enum" ? nameP.v : "";
  if (kind === "length" && name !== "METRE") return null;
  if (kind === "angle" && name !== "RADIAN") return null;
  let f = kind === "length" ? 1000 : 1;
  const pfx = si.params[0];
  if (pfx && pfx.k === "enum") f *= SI_PREFIX[pfx.v] ?? 1;
  return f;
}

/**
 * Resolve a unit instance (simple or complex) to its factor. Handles SI_UNIT directly and
 * CONVERSION_BASED_UNIT('INCH'|'DEGREE'|..., measure_with_unit#) by multiplying the measure value
 * with the factor of the unit it is expressed in (one level of recursion covers real files).
 */
function unitFactor(t: Table, id: number, kind: "length" | "angle", depth = 0): number | null {
  if (depth > 4) return null;
  const si = t.sub(id, "SI_UNIT");
  if (si) return siFactor(si, kind);
  const cbu = t.sub(id, "CONVERSION_BASED_UNIT"); // (name, measure_with_unit#)
  if (cbu && cbu.params[1]?.k === "ref") {
    const mwu = t.get(ref(cbu.params[1]));
    // MEASURE_WITH_UNIT(value_component, unit#) — value may be a typed LENGTH_MEASURE(x) etc.
    const rec = "complex" in mwu ? mwu.complex.find((r) => r.params.length >= 2) : mwu;
    if (!rec) return null;
    const vP = rec.params[0]!;
    const v = vP.k === "num" ? vP.v : vP.k === "typed" && vP.params[0]?.k === "num" ? vP.params[0].v : null;
    if (v === null || rec.params[1]?.k !== "ref") return null;
    const base = unitFactor(t, ref(rec.params[1]), kind, depth + 1);
    return base === null ? null : v * base;
  }
  return null;
}

/**
 * Find the model's length and plane-angle units. Prefers the units the geometry context actually
 * assigns (GLOBAL_UNIT_ASSIGNED_CONTEXT) — a file routinely also contains the RADIAN basis unit its
 * DEGREE conversion is defined in, so a global scan can pick the wrong one. Handles SI_UNIT
 * (MILLI METRE, RADIAN) and CONVERSION_BASED_UNIT (INCH, DEGREE, ...). Falls back to mm / radians.
 */
/**
 * Per-representation-context length scale (mm per unit). A multi-part assembly may declare a
 * DIFFERENT length unit per part representation (Inventor 2026 mixes plain METRE and MILLI METRE
 * contexts in one file); geometry must be scaled by the unit of the context its representation
 * references, not a single global pick. Returns only contexts that resolve a length unit — callers
 * fall back to the global detectUnits() scale for anything absent.
 */
export function contextLengthScales(table: Table): Map<number, number> {
  const out = new Map<number, number>();
  for (const [id, guac] of table.byType("GLOBAL_UNIT_ASSIGNED_CONTEXT")) {
    const units = guac.params.find((p) => p.k === "list");
    if (!units || units.k !== "list") continue;
    for (const u of units.v) {
      if (u.k !== "ref") continue;
      const e = table.model.entities.get(u.v);
      const isLen = e && ("complex" in e
        ? e.complex.some((r) => r.type === "LENGTH_UNIT")
        : e.type === "SI_UNIT"); // a bare SI_UNIT length (rare) — unitFactor rejects non-METRE anyway
      if (!isLen) continue;
      const f = unitFactor(table, u.v, "length");
      if (f !== null) { out.set(id, f); break; }
    }
  }
  return out;
}

export function detectUnits(table: Table): Units {
  let mmPerUnit: number | null = null;
  let radPerAngle: number | null = null;
  let label = "";

  const takeUnit = (id: number): void => {
    const e = table.model.entities.get(id);
    if (!e || !("complex" in e)) return;
    if (mmPerUnit === null && e.complex.some((r) => r.type === "LENGTH_UNIT")) {
      mmPerUnit = unitFactor(table, id, "length");
      if (mmPerUnit !== null) {
        const cbu = e.complex.find((r) => r.type === "CONVERSION_BASED_UNIT");
        label = cbu && cbu.params[0]?.k === "str" ? cbu.params[0].v.toLowerCase()
          : mmPerUnit === 1 ? "millimetre" : mmPerUnit === 1000 ? "metre" : `${mmPerUnit}mm/unit`;
      }
    }
    if (radPerAngle === null && e.complex.some((r) => r.type === "PLANE_ANGLE_UNIT")) {
      radPerAngle = unitFactor(table, id, "angle");
    }
  };

  // GLOBAL_UNIT_ASSIGNED_CONTEXT((unit#...)) inside the representation context names the units in use.
  for (const [, guac] of table.byType("GLOBAL_UNIT_ASSIGNED_CONTEXT")) {
    const units = guac.params.find((p) => p.k === "list");
    if (units && units.k === "list") for (const u of units.v) if (u.k === "ref") takeUnit(u.v);
    if (mmPerUnit !== null && radPerAngle !== null) break;
  }
  // Fallback: scan every complex entity (some files skip the assigned-context wrapper).
  if (mmPerUnit === null || radPerAngle === null) {
    for (const [id, e] of table.model.entities) {
      if (!("complex" in e)) continue;
      takeUnit(id);
      if (mmPerUnit !== null && radPerAngle !== null) break;
    }
  }
  return {
    mmPerUnit: mmPerUnit ?? 1,
    radPerAngle: radPerAngle ?? 1,
    label: label || "assumed mm",
  };
}
