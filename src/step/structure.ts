// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — STEP product structure: the assembly/part tree with product names.
// Parts live in their own shape representation; SHAPE_DEFINITION_REPRESENTATION ties a
// representation to its PRODUCT (via PRODUCT_DEFINITION_SHAPE -> PRODUCT_DEFINITION ->
// PRODUCT_DEFINITION_FORMATION), and REPRESENTATION_RELATIONSHIP(_WITH_TRANSFORMATION) links a
// child rep into its parent — the same graph the placement resolver walks in brep/build.ts.
// The tree is per PART, not per occurrence: a part used N times in the assembly is meshed once
// and replicated with the SAME solid ids, so one node with `occurrences: N` is the finest
// granularity a consumer can act on (hiding it hides all N instances).
import { Table, ref, refList, str } from "./entities.ts";
import type { BSolid } from "../brep/build.ts";

/** One solid body: `id` matches the values of MeshResult.solidOfTri. */
export interface PartBody { id: number; name: string; }

export interface PartNode {
  /** PRODUCT name from the STEP product structure ('' when the file carries none). */
  name: string;
  /** Assembly occurrences merged into this node (its solids' triangles cover all of them). */
  occurrences: number;
  /** Solid bodies whose geometry lives directly in this node's representation. */
  bodies: PartBody[];
  children: PartNode[];
}

const SOLID_TYPES = ["MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS", "SHELL_BASED_SURFACE_MODEL"];

/**
 * Build the part/component tree over the imported solids. Always returns a root node: for a
 * single-part file it is the part itself with its bodies; for an assembly it is the top product
 * with nested children. Solids the product graph doesn't reach (or a malformed graph) degrade to
 * flat bodies on the root, never an error.
 */
export function extractStructure(table: Table, solids: readonly Pick<BSolid, "id">[]): PartNode {
  const bodyName = (id: number): string => {
    for (const ty of SOLID_TYPES) {
      const rec = table.sub(id, ty);
      if (rec?.params[0]?.k === "str") return rec.params[0].v;
    }
    return "";
  };
  const fallback = (): PartNode => ({
    name: "", occurrences: 1, children: [],
    bodies: solids.map((s) => ({ id: s.id, name: bodyName(s.id) })),
  });
  try {
    return buildTree(table, solids, bodyName) ?? fallback();
  } catch {
    return fallback();
  }
}

function buildTree(
  table: Table, solids: readonly Pick<BSolid, "id">[], bodyName: (id: number) => string,
): PartNode | null {
  const wanted = new Set(solids.map((s) => s.id));

  // Geometry rep -> the imported solids it contains.
  const solidsOfRep = new Map<number, number[]>();
  for (const ty of ["ADVANCED_BREP_SHAPE_REPRESENTATION", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION"]) {
    for (const [repId, rep] of table.byType(ty)) {
      if (rep.params[1]?.k !== "list") continue;
      for (const item of refList(rep.params[1]!)) {
        if (!wanted.has(item)) continue;
        (solidsOfRep.get(repId) ?? solidsOfRep.set(repId, []).get(repId)!).push(item);
      }
    }
  }

  // Rep -> product name, via SHAPE_DEFINITION_REPRESENTATION(PDS, rep).
  const nameOfRep = new Map<number, string>();
  for (const [, sdr] of table.byType("SHAPE_DEFINITION_REPRESENTATION")) {
    try {
      if (sdr.params[0]?.k !== "ref" || sdr.params[1]?.k !== "ref") continue;
      const pds = table.sub(ref(sdr.params[0]!), "PRODUCT_DEFINITION_SHAPE");
      if (!pds) continue;
      const pd = table.sub(ref(pds.params[2]!), "PRODUCT_DEFINITION");
      if (!pd) continue;
      const pdfId = ref(pd.params[2]!);
      const pdf = table.sub(pdfId, "PRODUCT_DEFINITION_FORMATION")
        ?? table.sub(pdfId, "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE");
      if (!pdf) continue;
      const prod = table.sub(ref(pdf.params[2]!), "PRODUCT");
      if (!prod) continue;
      const name = prod.params[1]?.k === "str" ? prod.params[1].v : "";
      nameOfRep.set(ref(sdr.params[1]!), name || (prod.params[0]?.k === "str" ? prod.params[0].v : ""));
    } catch { /* skip malformed product chain */ }
  }

  // Parent/child links between reps (RRWT), plus identity SRR links bridging a product's
  // placeholder SHAPE_REPRESENTATION to its geometry rep — mirrors assemblyInfo in brep/build.ts.
  const childrenOf = new Map<number, Map<number, number>>(); // parent -> child -> occurrence count
  const hasParent = new Set<number>();
  const equiv = new Map<number, number>();
  for (const [id, srr] of table.byType("SHAPE_REPRESENTATION_RELATIONSHIP")) {
    if (table.isComplex(id) || srr.params[2]?.k !== "ref" || srr.params[3]?.k !== "ref") continue;
    const a = ref(srr.params[2]!), b = ref(srr.params[3]!);
    equiv.set(a, b); equiv.set(b, a);
  }
  const links: { child: number; parent: number }[] = [];
  for (const [id] of table.byType("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")) {
    const rr = table.sub(id, "REPRESENTATION_RELATIONSHIP");
    if (!rr || rr.params[2]?.k !== "ref" || rr.params[3]?.k !== "ref") continue;
    links.push({ child: ref(rr.params[2]!), parent: ref(rr.params[3]!) });
  }
  if (links.length === 0 && solidsOfRep.size === 1) {
    // Single-part file: one node with the product's own name.
    const [repId, ids] = [...solidsOfRep][0]!;
    return {
      name: nameOfRep.get(repId) ?? nameOfRep.get(equiv.get(repId) ?? -1) ?? "",
      occurrences: 1,
      bodies: ids.map((id) => ({ id, name: bodyName(id) })),
      children: [],
    };
  }
  // A child rep with parent links keeps its own identity; one linked only through its SRR twin
  // is folded onto that twin so geometry, name and links land on a single node.
  const linkedReps = new Set<number>();
  for (const l of links) { linkedReps.add(l.child); linkedReps.add(l.parent); }
  const resolve = (rep: number): number => (linkedReps.has(rep) ? rep : (equiv.get(rep) ?? rep));
  for (const l of links) {
    const child = resolve(l.child), parent = resolve(l.parent);
    hasParent.add(child);
    const m = childrenOf.get(parent) ?? childrenOf.set(parent, new Map()).get(parent)!;
    m.set(child, (m.get(child) ?? 0) + 1);
  }
  const nodeSolids = new Map<number, number[]>();
  for (const [repId, ids] of solidsOfRep) {
    const r = resolve(repId);
    (nodeSolids.get(r) ?? nodeSolids.set(r, []).get(r)!).push(...ids);
  }
  const nodeName = (rep: number): string => nameOfRep.get(rep) ?? nameOfRep.get(equiv.get(rep) ?? -1) ?? "";

  // Assemble the tree from the roots down. A rep can appear under several parents; the node is
  // rebuilt per path but the solid ids stay shared (per-part granularity, see module doc).
  const placed = new Set<number>();
  const build = (rep: number, occurrences: number, onPath: Set<number>): PartNode => {
    placed.add(rep);
    const children: PartNode[] = [];
    const kids = childrenOf.get(rep);
    if (kids && !onPath.has(rep)) {
      onPath.add(rep);
      for (const [child, count] of kids) children.push(build(child, count, onPath));
      onPath.delete(rep);
    }
    children.sort((a, b) => a.name.localeCompare(b.name));
    return {
      name: nodeName(rep),
      occurrences,
      bodies: (nodeSolids.get(rep) ?? []).map((id) => ({ id, name: bodyName(id) })),
      children,
    };
  };
  const rootReps = [...new Set([...childrenOf.keys()])].filter((r) => !hasParent.has(r));
  const roots = rootReps.map((r) => build(r, 1, new Set()));

  // Solids whose rep the relationship graph never reached become their own top-level nodes.
  for (const [rep, ids] of nodeSolids) {
    if (placed.has(rep)) continue;
    roots.push({
      name: nodeName(rep), occurrences: 1,
      bodies: ids.map((id) => ({ id, name: bodyName(id) })), children: [],
    });
  }
  if (roots.length === 0) return null;
  if (roots.length === 1) return roots[0]!;
  return { name: "", occurrences: 1, bodies: [], children: roots };
}
