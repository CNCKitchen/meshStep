// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — assembly relationship links with the parent/child direction resolved.
// REPRESENTATION_RELATIONSHIP(_WITH_TRANSFORMATION) relates a component's rep to its assembly's,
// but ISO 10303 does not fix which of (rep_1, rep_2) is the component: most exporters write
// (child, parent), others write (parent, child) — the ETA 6497 movement writes every link as
// (root assembly, part), which read the other way around collapses ALL placements to identity
// (no part rep ever appears as a child, so every solid degrades to the identity frame) and turns
// the product tree inside out. The authoritative signal is the CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
// annotating each relationship: its PRODUCT_DEFINITION_SHAPE resolves to the
// NEXT_ASSEMBLY_USAGE_OCCURRENCE naming the relating (parent) and related (child) products, and
// SHAPE_DEFINITION_REPRESENTATION ties each rep to its product definition. Links without that
// chain keep the historical (child, parent) reading. The ITEM_DEFINED_TRANSFORMATION items pair
// positionally with the reps (item1 in rep_1's context, item2 in rep_2's) and swap alongside.
import { Table, ref } from "./entities.ts";

export interface AssemblyLink {
  /** Rep of the component occurrence (the child being placed). */
  child: number;
  /** Rep of the assembly that instantiates it. */
  parent: number;
  /** Placement id in the child rep — the component frame being mapped (absent: malformed IDT). */
  childItem?: number;
  /** Placement id in the parent rep — where the component frame lands (absent: malformed IDT). */
  parentItem?: number;
}

/** Extract every RRWT assembly link (complex and simple form), oriented child -> parent. */
export function assemblyLinks(t: Table): AssemblyLink[] {
  // rep -> product definition, via SHAPE_DEFINITION_REPRESENTATION(PDS, rep).
  const pdOfRep = new Map<number, number>();
  for (const [, sdr] of t.byType("SHAPE_DEFINITION_REPRESENTATION")) {
    if (sdr.params[0]?.k !== "ref" || sdr.params[1]?.k !== "ref") continue;
    const pds = t.sub(ref(sdr.params[0]), "PRODUCT_DEFINITION_SHAPE");
    if (pds?.params[2]?.k !== "ref") continue;
    pdOfRep.set(ref(sdr.params[1]), ref(pds.params[2]));
  }
  // relationship id -> its NAUO's (parent, child) product definitions, via
  // CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(relationship, PDS -> NAUO(relating, related)).
  const nauoOfRel = new Map<number, { parentPd: number; childPd: number }>();
  for (const [, cdsr] of t.byType("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")) {
    if (cdsr.params[0]?.k !== "ref" || cdsr.params[1]?.k !== "ref") continue;
    const pds = t.sub(ref(cdsr.params[1]), "PRODUCT_DEFINITION_SHAPE");
    if (pds?.params[2]?.k !== "ref") continue;
    const nauo = t.sub(ref(pds.params[2]), "NEXT_ASSEMBLY_USAGE_OCCURRENCE");
    if (nauo?.params[3]?.k !== "ref" || nauo.params[4]?.k !== "ref") continue;
    nauoOfRel.set(ref(cdsr.params[0]), { parentPd: ref(nauo.params[3]), childPd: ref(nauo.params[4]) });
  }

  const links: AssemblyLink[] = [];
  for (const [id, rrwt] of t.byType("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")) {
    // Complex form: reps live on the REPRESENTATION_RELATIONSHIP partial record and the RRWT
    // partial holds only the transformation. Simple form: one record carries all five params.
    const rr = t.isComplex(id) ? t.sub(id, "REPRESENTATION_RELATIONSHIP") : rrwt;
    if (!rr || rr.params[2]?.k !== "ref" || rr.params[3]?.k !== "ref") continue;
    const r1 = ref(rr.params[2]), r2 = ref(rr.params[3]);
    const xform = t.isComplex(id) ? rrwt.params[0] : rrwt.params[4];
    let i1: number | undefined, i2: number | undefined;
    if (xform?.k === "ref") {
      const idt = t.sub(ref(xform), "ITEM_DEFINED_TRANSFORMATION");
      if (idt?.params[2]?.k === "ref" && idt.params[3]?.k === "ref") {
        i1 = ref(idt.params[2]); i2 = ref(idt.params[3]);
      }
    }
    // Direction: rep_1 = child unless the NAUO says rep_1's product is the parent (or rep_2's
    // the child). A one-sided product match decides too — placeholder reps sometimes lack an SDR.
    let swapped = false;
    const nauo = nauoOfRel.get(id);
    if (nauo && nauo.parentPd !== nauo.childPd) {
      const pd1 = pdOfRep.get(r1), pd2 = pdOfRep.get(r2);
      if (pd1 === nauo.parentPd || pd2 === nauo.childPd) swapped = true;
    }
    links.push(swapped
      ? { child: r2, parent: r1, childItem: i2, parentItem: i1 }
      : { child: r1, parent: r2, childItem: i1, parentItem: i2 });
  }
  return links;
}
