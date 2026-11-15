// SPDX-License-Identifier: AGPL-3.0-only
// Is the model closed in the STEP file itself? Read straight from the file's B-rep topology, with
// no converter involved, so the "watertight" KPI only asks for watertightness where the input has it.
//
// For every shell a solid or surface model references, count how often each edge is used by the
// shell's faces and in which direction. The file describes a closed, manifold, consistently
// oriented shell exactly when every edge is used twice, in opposite directions. A model counts as
// closed when it has at least one shell and all of them pass. Surface bodies (OPEN_SHELL), files
// with bare trimmed surfaces and no topology, and shells with free or non-manifold edges do not.
//
//   node bench/topology.ts <file.step> ...     prints the verdict per file
import { existsSync, readFileSync, readdirSync, writeFileSync, mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, "..");

export interface Topology {
  /** Shells referenced by solids (MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS, FACETED_BREP) and surface models,
   * plus AP242 tessellated solids and shells. */
  shells: number;
  /** Shells where some edge is not used exactly twice in opposite directions. */
  openShells: number;
  /** Edges used once (free boundary). */
  freeEdges: number;
  /** Edges used more than twice. */
  nonManifoldEdges: number;
  /** Edges used twice in the same direction (the file's face orientation is inconsistent). */
  misorientedEdges: number;
  /** The file describes only closed, manifold, consistently oriented shells. */
  closed: boolean;
}

type Arg = number | string | boolean | null | { ref: number } | Arg[];
interface Rec { type: string; args: Arg[] }

const WANT = new Set([
  "MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS", "FACETED_BREP", "SHELL_BASED_SURFACE_MODEL",
  "CLOSED_SHELL", "OPEN_SHELL", "ORIENTED_CLOSED_SHELL", "ORIENTED_OPEN_SHELL",
  "ADVANCED_FACE", "FACE_SURFACE", "ORIENTED_FACE",
  "FACE_OUTER_BOUND", "FACE_BOUND", "EDGE_LOOP", "POLY_LOOP", "ORIENTED_EDGE",
  "TESSELLATED_SOLID", "TESSELLATED_SHELL", "TRIANGULATED_FACE", "COMPLEX_TRIANGULATED_FACE", "COORDINATES_LIST",
]);

/** Parse a record's parameter list: numbers, #refs, enums (.T./.F. as booleans), strings, $ and *, nested lists. */
function parseArgs(s: string, i: number): [Arg[], number] {
  const out: Arg[] = [];
  while (i < s.length) {
    const c = s[i]!;
    if (c === ")") return [out, i + 1];
    if (c === "," || c === " " || c === "\n" || c === "\r" || c === "\t") { i++; continue; }
    if (c === "(") { const [a, j] = parseArgs(s, i + 1); out.push(a); i = j; continue; }
    if (c === "'") { let j = i + 1; while (j < s.length && !(s[j] === "'" && s[j + 1] !== "'")) j += s[j] === "'" ? 2 : 1; out.push(s.slice(i + 1, j)); i = j + 1; continue; }
    if (c === "#") { let j = i + 1; while (j < s.length && s.charCodeAt(j) >= 48 && s.charCodeAt(j) <= 57) j++; out.push({ ref: Number(s.slice(i + 1, j)) }); i = j; continue; }
    if (c === "$" || c === "*") { out.push(null); i++; continue; }
    if (c === ".") { const j = s.indexOf(".", i + 1); const e = s.slice(i + 1, j); out.push(e === "T" ? true : e === "F" ? false : e); i = j + 1; continue; }
    let j = i; while (j < s.length && !",() ".includes(s[j]!)) j++;
    const tok = s.slice(i, j);
    // typed parameter like LENGTH_MEASURE(1.0): keep the inner value
    if (s[j] === "(" && /^[A-Z_]/.test(tok)) { const [a, k] = parseArgs(s, j + 1); out.push(a[0] ?? null); i = k; continue; }
    out.push(Number(tok)); i = j;
  }
  return [out, i];
}

function parse(text: string): Map<number, Rec> {
  const recs = new Map<number, Rec>();
  const start = text.indexOf("DATA;");
  const body = start >= 0 ? text.slice(start + 5) : text;
  // records end with ';' — split at the ';' that is followed by the next record (or ENDSEC),
  // which keeps ';' inside quoted names intact
  for (const raw of body.split(/;\s*(?=#\d+\s*=|ENDSEC)/)) {
    const m = /^\s*#(\d+)\s*=\s*([A-Z0-9_]+)\s*\(/.exec(raw);
    if (!m || !WANT.has(m[2]!)) continue;
    recs.set(Number(m[1]), { type: m[2]!, args: parseArgs(raw, m[0].length)[0] });
  }
  return recs;
}

const ref = (a: Arg | undefined): number | undefined => a && typeof a === "object" && !Array.isArray(a) ? a.ref : undefined;
const refs = (a: Arg | undefined): number[] => Array.isArray(a) ? a.map(ref).filter((x): x is number => x !== undefined) : [];
const sign = (a: Arg | undefined): number => a === false ? -1 : 1;

export function fileTopology(text: string): Topology {
  const R = parse(text);
  const t: Topology = { shells: 0, openShells: 0, freeEdges: 0, nonManifoldEdges: 0, misorientedEdges: 0, closed: false };
  /** Tally one shell's edge uses: [count, signed direction sum] per edge key. */
  const judge = (uses: Map<string, [number, number]>, faces: number) => {
    t.shells++;
    let bad = faces === 0;
    for (const [n, s] of uses.values()) {
      if (n === 1) { t.freeEdges++; bad = true; }
      else if (n > 2) { t.nonManifoldEdges++; bad = true; }
      else if (s !== 0) { t.misorientedEdges++; bad = true; }
    }
    if (bad) t.openShells++;
  };
  const use = (uses: Map<string, [number, number]>, key: string, dir: number) => { const u = uses.get(key); if (u) { u[0]++; u[1] += dir; } else uses.set(key, [1, dir]); };

  // ---- B-rep shells referenced by a solid or a surface model
  const shellIds = new Set<number>();
  for (const r of R.values()) {
    if (r.type === "MANIFOLD_SOLID_BREP" || r.type === "FACETED_BREP") { const s = ref(r.args[1]); if (s !== undefined) shellIds.add(s); }
    else if (r.type === "BREP_WITH_VOIDS") { const s = ref(r.args[1]); if (s !== undefined) shellIds.add(s); for (const v of refs(r.args[2])) shellIds.add(v); }
    else if (r.type === "SHELL_BASED_SURFACE_MODEL") for (const s of refs(r.args[1])) shellIds.add(s);
  }
  for (const sid of shellIds) {
    let shell = R.get(sid);
    // ORIENTED_*_SHELL(name, *, base, orient) wraps a base shell; flipping all of it keeps consistency
    if (shell && shell.type.startsWith("ORIENTED_")) shell = R.get(ref(shell.args[2])!);
    if (!shell) continue;
    const uses = new Map<string, [number, number]>();
    const faces = refs(shell.args[1]);
    for (const fid of faces) {
      let fsign = 1;
      let face = R.get(fid);
      if (face?.type === "ORIENTED_FACE") { fsign = sign(face.args[3]); face = R.get(ref(face.args[2])!); }
      if (!face) continue;
      // ADVANCED_FACE / FACE_SURFACE (name, (bounds), surface, same_sense): loops are oriented
      // against the face's own normal, so same_sense does not enter the edge directions
      for (const bid of refs(face.args[1])) {
        const bound = R.get(bid);
        if (!bound) continue;
        const bsign = sign(bound.args[2]) * fsign;
        const loop = R.get(ref(bound.args[1])!);
        if (!loop) continue; // VERTEX_LOOP (sphere pole, full torus): no edges
        if (loop.type === "POLY_LOOP") {
          const p = refs(loop.args[1]);
          for (let i = 0; i < p.length; i++) {
            const a = p[i]!, b = p[(i + 1) % p.length]!;
            use(uses, a < b ? `${a}-${b}` : `${b}-${a}`, (a < b ? 1 : -1) * bsign);
          }
        } else if (loop.type === "EDGE_LOOP") {
          for (const oid of refs(loop.args[1])) {
            let sg = bsign, id = oid, e = R.get(oid);
            // ORIENTED_EDGE(name, *, *, edge_element, orient); edge_element may itself be oriented
            while (e?.type === "ORIENTED_EDGE") { sg *= sign(e.args[4]); id = ref(e.args[3])!; e = R.get(id); }
            use(uses, String(id), sg);
          }
        }
      }
    }
    judge(uses, faces.length);
  }

  // ---- AP242 tessellated bodies: triangles indexing a coordinate list, edges matched by point value
  const coords = new Map<number, string[]>();
  const pointKeys = (id: number): string[] => {
    let c = coords.get(id);
    if (!c) {
      const r = R.get(id);
      const pts = r && Array.isArray(r.args[2]) ? r.args[2] as Arg[] : [];
      c = pts.map(p => Array.isArray(p) ? p.map(x => Number(x).toPrecision(9)).join(",") : "");
      coords.set(id, c);
    }
    return c;
  };
  for (const r of R.values()) {
    if (r.type !== "TESSELLATED_SOLID" && r.type !== "TESSELLATED_SHELL") continue;
    const uses = new Map<string, [number, number]>();
    const faces = refs(r.args[1]);
    for (const fid of faces) {
      const f = R.get(fid);
      if (!f || (f.type !== "TRIANGULATED_FACE" && f.type !== "COMPLEX_TRIANGULATED_FACE")) continue;
      const pts = pointKeys(ref(f.args[1])!);
      const pn = Array.isArray(f.args[5]) ? (f.args[5] as number[]) : [];
      const P = (i: number) => pts[(pn.length ? pn[i - 1]! : i) - 1] ?? `?${i}`;
      const tri = (a: number, b: number, c: number) => {
        const k = [P(a), P(b), P(c)];
        if (k[0] === k[1] || k[1] === k[2] || k[0] === k[2]) return;
        for (let e = 0; e < 3; e++) { const x = k[e]!, y = k[(e + 1) % 3]!; use(uses, x < y ? `${x}|${y}` : `${y}|${x}`, x < y ? 1 : -1); }
      };
      if (f.type === "TRIANGULATED_FACE") {
        for (const t3 of (f.args[6] as number[][]) ?? []) tri(t3[0]!, t3[1]!, t3[2]!);
      } else {
        for (const s of (f.args[6] as number[][]) ?? []) for (let i = 0; i + 2 < s.length; i++) i % 2 ? tri(s[i + 1]!, s[i]!, s[i + 2]!) : tri(s[i]!, s[i + 1]!, s[i + 2]!);
        for (const s of (f.args[7] as number[][]) ?? []) for (let i = 1; i + 1 < s.length; i++) tri(s[0]!, s[i]!, s[i + 1]!);
      }
    }
    judge(uses, faces.length);
  }
  t.closed = t.shells > 0 && t.openShells === 0;
  return t;
}

/** The STEP file behind a benchmark model key (as run.ts names them). */
export function modelPath(key: string): string | undefined {
  if (key.startsWith("abc/")) {
    const dir = join(root, "abc_0002_step_v00", key.slice(4));
    if (!existsSync(dir)) return undefined;
    const f = readdirSync(dir).find(x => /\.(step|stp)$/i.test(x));
    return f ? join(dir, f) : undefined;
  }
  const base = join(root, key);
  for (const ext of [".step", ".stp", ".STEP", ".STP"]) if (existsSync(base + ext)) return base + ext;
  const d = dirname(base);
  const f = existsSync(d) ? readdirSync(d).find(x => x.replace(/\.(step|stp)$/i, "") === key.split("/").pop() && /\.(step|stp)$/i.test(x)) : undefined;
  return f ? join(d, f) : undefined;
}

const cacheFile = join(here, "out", "cache", "topology.json");
let cache: Record<string, Topology> | undefined;

/** Topology verdict for a model key, cached in out/cache/topology.json. */
export function topologyOf(key: string): Topology | undefined {
  cache ??= existsSync(cacheFile) ? JSON.parse(readFileSync(cacheFile, "utf8")) : {};
  if (cache![key]) return cache![key];
  const p = modelPath(key);
  if (!p) return undefined;
  const t = fileTopology(readFileSync(p, "latin1"));
  cache![key] = t;
  mkdirSync(dirname(cacheFile), { recursive: true });
  writeFileSync(cacheFile, JSON.stringify(cache, null, 1));
  return t;
}

if (process.argv[1] && fileURLToPath(import.meta.url) === process.argv[1]) {
  for (const f of process.argv.slice(2)) {
    const t = fileTopology(readFileSync(f, "latin1"));
    console.log(`${t.closed ? "closed" : "OPEN  "}  shells ${t.shells} (open ${t.openShells}) free ${t.freeEdges} non-manifold ${t.nonManifoldEdges} misoriented ${t.misorientedEdges}  ${f}`);
  }
}
