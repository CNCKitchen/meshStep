// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — typed access over the parsed STEP entity table.
import type { Entity, EntityRecord, Param, StepModel } from "./parser.ts";

export class Table {
  readonly model: StepModel;
  constructor(model: StepModel) {
    this.model = model;
  }

  get(id: number): Entity {
    const e = this.model.entities.get(id);
    if (!e) throw new Error(`STEP: missing entity #${id}`);
    return e;
  }

  /** A simple (non-complex) record; throws if the instance is complex. */
  record(id: number): EntityRecord {
    const e = this.get(id);
    if ("complex" in e) throw new Error(`STEP: entity #${id} is complex`);
    return e;
  }

  isComplex(id: number): boolean {
    return "complex" in this.get(id);
  }

  /** From a (possibly complex) instance, find a partial record of a given type. */
  sub(id: number, type: string): EntityRecord | undefined {
    const e = this.get(id);
    if ("complex" in e) return e.complex.find((r) => r.type === type);
    return e.type === type ? e : undefined;
  }

  typeOf(id: number): string | undefined {
    const e = this.get(id);
    return "complex" in e ? undefined : e.type;
  }

  *byType(type: string): Generator<[number, EntityRecord]> {
    for (const [id, e] of this.model.entities) {
      if ("complex" in e) {
        for (const r of e.complex) if (r.type === type) yield [id, r];
      } else if (e.type === type) {
        yield [id, e];
      }
    }
  }

  histogram(): Map<string, number> {
    const h = new Map<string, number>();
    for (const e of this.model.entities.values()) {
      const key = "complex" in e ? "(complex)" : e.type;
      h.set(key, (h.get(key) ?? 0) + 1);
    }
    return h;
  }
}

// ---- Param accessors (throw on type mismatch) ----
export const num = (p: Param): number => {
  if (p.k === "num") return p.v;
  throw new Error(`expected number, got ${p.k}`);
};
export const str = (p: Param): string => {
  if (p.k === "str") return p.v;
  throw new Error(`expected string, got ${p.k}`);
};
export const ref = (p: Param): number => {
  if (p.k === "ref") return p.v;
  throw new Error(`expected ref, got ${p.k}`);
};
export const enumOf = (p: Param): string => {
  if (p.k === "enum") return p.v;
  throw new Error(`expected enum, got ${p.k}`);
};
export const list = (p: Param): Param[] => {
  if (p.k === "list") return p.v;
  throw new Error(`expected list, got ${p.k}`);
};
export const refList = (p: Param): number[] => list(p).map(ref);
export const numList = (p: Param): number[] => list(p).map(num);
export const isNone = (p: Param): boolean => p.k === "none";
