// SPDX-License-Identifier: AGPL-3.0-only
// Tests for the geometry clean-up before meshing (src/brep/defeature.ts, option `mergeTolerance`):
// edges shorter than the tolerance collapse to a point, and a face left as a sliver between two
// edges closer than the tolerance goes with them.
//
//   node test/defeature.ts
//
// The fixtures are small polyhedra written as STEP text here (planar faces, straight edges), so
// every length in them is known exactly.
import { importStep } from "../src/index.ts";

let failed = 0;
const check = (ok: boolean, msg: string): void => { console.log(`${ok ? "ok  " : "FAIL"} ${msg}`); if (!ok) failed++; };

type P3 = [number, number, number];

/** STEP text of a closed polyhedron: `faces` are vertex loops, counter-clockwise seen from outside. */
function polyhedronStep(pts: P3[], faces: number[][]): string {
  const out: string[] = [];
  let id = 0;
  const add = (s: string): number => { out.push(`#${++id}=${s};`); return id; };
  const f = (x: number): string => (Number.isInteger(x) ? `${x}.` : String(x));
  const dir = (d: P3): number => { const l = Math.hypot(...d); return add(`DIRECTION('',(${f(d[0] / l)},${f(d[1] / l)},${f(d[2] / l)}))`); };
  const cp = pts.map((p) => add(`CARTESIAN_POINT('',(${f(p[0])},${f(p[1])},${f(p[2])}))`));
  const vp = cp.map((c) => add(`VERTEX_POINT('',#${c})`));
  const edges = new Map<string, number>();
  const edge = (a: number, b: number): [number, boolean] => {
    const lo = Math.min(a, b), hi = Math.max(a, b), key = `${lo},${hi}`;
    if (!edges.has(key)) {
      const d: P3 = [pts[hi]![0] - pts[lo]![0], pts[hi]![1] - pts[lo]![1], pts[hi]![2] - pts[lo]![2]];
      const v = add(`VECTOR('',#${dir(d)},${f(Math.hypot(...d))})`);
      const ln = add(`LINE('',#${cp[lo]},#${v})`);
      edges.set(key, add(`EDGE_CURVE('',#${vp[lo]},#${vp[hi]},#${ln},.T.)`));
    }
    return [edges.get(key)!, a < b];
  };
  const faceIds: number[] = [];
  for (const loop of faces) {
    const oes = loop.map((a, k) => { const [e, fwd] = edge(a, loop[(k + 1) % loop.length]!); return add(`ORIENTED_EDGE('',*,*,#${e},${fwd ? ".T." : ".F."})`); });
    const el = add(`EDGE_LOOP('',(${oes.map((x) => `#${x}`).join(",")}))`);
    const fb = add(`FACE_OUTER_BOUND('',#${el},.T.)`);
    // Newell normal; the reference direction is the first edge.
    const n: P3 = [0, 0, 0];
    for (let k = 0; k < loop.length; k++) {
      const a = pts[loop[k]!]!, b = pts[loop[(k + 1) % loop.length]!]!;
      n[0] += (a[1] - b[1]) * (a[2] + b[2]); n[1] += (a[2] - b[2]) * (a[0] + b[0]); n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    const a = pts[loop[0]!]!, b = pts[loop[1]!]!;
    const ax = add(`AXIS2_PLACEMENT_3D('',#${cp[loop[0]!]},#${dir(n)},#${dir([b[0] - a[0], b[1] - a[1], b[2] - a[2]])})`);
    const pl = add(`PLANE('',#${ax})`);
    faceIds.push(add(`ADVANCED_FACE('',(#${fb}),#${pl},.T.)`));
  }
  const sh = add(`CLOSED_SHELL('',(${faceIds.map((x) => `#${x}`).join(",")}))`);
  add(`MANIFOLD_SOLID_BREP('',#${sh})`);
  return `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${out.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
}

interface Stats { tris: number; minEdge: number; open: number; volume: number; faces: number; merged?: { edges: number; faces: number; maxMove: number; reverted: boolean } }
function stats(src: string, mergeTolerance?: number, isotropic = false): Stats {
  const r = importStep(src, { surfaceDeviation: 0.01, maxEdge: 2, isotropic, ...(mergeTolerance !== undefined ? { mergeTolerance } : {}) });
  const P = r.mesh.positions, I = r.mesh.indices;
  let minEdge = Infinity, volume = 0;
  const count = new Map<string, number>();
  for (let t = 0; t < I.length; t += 3) {
    const v = [I[t]!, I[t + 1]!, I[t + 2]!];
    const p = v.map((i) => [P[i * 3]!, P[i * 3 + 1]!, P[i * 3 + 2]!] as P3);
    for (let k = 0; k < 3; k++) {
      const a = v[k]!, b = v[(k + 1) % 3]!;
      minEdge = Math.min(minEdge, Math.hypot(p[k]![0] - p[(k + 1) % 3]![0], p[k]![1] - p[(k + 1) % 3]![1], p[k]![2] - p[(k + 1) % 3]![2]));
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      count.set(key, (count.get(key) ?? 0) + 1);
    }
    const [a, b, c] = p as [P3, P3, P3];
    volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
  }
  let open = 0;
  for (const n of count.values()) if (n !== 2) open++;
  return { tris: I.length / 3, minEdge, open, volume: Math.abs(volume), faces: new Set(r.faceOfTri).size, merged: r.merged };
}

// ---- a short edge: a box with one top corner cut off by 0.03 mm ---------------------------------
// The cut leaves a triangular speck with three 0.042 mm edges; everything else is 10 mm.
{
  const c = 0.03;
  const pts: P3[] = [
    [0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0],
    [0, 0, 10], [10, 0, 10], [0, 10, 10],
    [10 - c, 10, 10], [10, 10 - c, 10], [10, 10, 10 - c],
  ];
  const src = polyhedronStep(pts, [
    [0, 3, 2, 1], [4, 5, 8, 7, 6], [0, 1, 5, 4], [1, 2, 9, 8, 5], [2, 3, 6, 7, 9], [3, 0, 4, 6], [7, 8, 9],
  ]);
  for (const iso of [false, true]) {
    const mode = iso ? "isotropic" : "default";
    const plain = stats(src, undefined, iso), off = stats(src, 0, iso), fine = stats(src, 0.01, iso), merged = stats(src, 0.1, iso);
    check(plain.open === 0 && plain.minEdge < 0.05, `${mode}: the cut corner carries a ${plain.minEdge.toFixed(3)} mm edge without merging (${plain.tris} triangles)`);
    check(off.tris === plain.tris && fine.tris === plain.tris && fine.minEdge === plain.minEdge, `${mode}: a tolerance of 0 or below the edge length changes nothing`);
    check(merged.open === 0, `${mode}: merged mesh is closed`);
    check(merged.minEdge > 0.5, `${mode}: no short edge is left after merging (shortest ${merged.minEdge.toFixed(3)} mm)`);
    check(merged.faces === 6, `${mode}: the speck is gone, six faces remain (${merged.faces})`);
    check(merged.tris <= plain.tris, `${mode}: merging does not add triangles (${plain.tris} -> ${merged.tris})`);
    // The two faces that follow the moved corner each tilt one triangle by up to c: at most
    // c x face area / 3 each.
    check(Math.abs(merged.volume - plain.volume) < 2 * c * 100 / 3, `${mode}: volume moves by ${Math.abs(merged.volume - plain.volume).toFixed(3)} mm3 of 1000`);
  }
}

// ---- a step: the top of a box in two halves, 0.04 mm apart in height ------------------------------
// The riser between the halves is a face 0.04 mm high and 10 mm long.
{
  const h = 0.04;
  const pts: P3[] = [
    [0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0],
    [0, 0, 10], [5, 0, 10], [5, 10, 10], [0, 10, 10],
    [5, 0, 10 + h], [10, 0, 10 + h], [10, 10, 10 + h], [5, 10, 10 + h],
  ];
  const src = polyhedronStep(pts, [
    [0, 3, 2, 1], [4, 5, 6, 7], [8, 9, 10, 11], [5, 8, 11, 6],
    [0, 1, 9, 8, 5, 4], [1, 2, 10, 9], [2, 3, 7, 6, 11, 10], [3, 0, 4, 7],
  ]);
  for (const iso of [false, true]) {
    const mode = iso ? "isotropic" : "default";
    const plain = stats(src, undefined, iso), merged = stats(src, 0.1, iso);
    check(plain.open === 0 && plain.minEdge <= h + 1e-9, `${mode}: the step carries ${plain.minEdge.toFixed(3)} mm edges without merging (${plain.tris} triangles)`);
    check(merged.open === 0, `${mode}: merged mesh is closed`);
    check(merged.minEdge > 0.5, `${mode}: no short edge is left after merging (shortest ${merged.minEdge.toFixed(3)} mm)`);
    check(merged.faces === 7, `${mode}: the riser is gone, seven faces remain (${merged.faces})`);
    check(merged.tris < plain.tris, `${mode}: fewer triangles (${plain.tris} -> ${merged.tris})`);
    check(Math.abs(merged.volume - plain.volume) < 5 * 10 * h + 1e-6, `${mode}: volume moves by ${Math.abs(merged.volume - plain.volume).toFixed(3)} mm3 (at most the step's own volume)`);
  }
}

// ---- a thin wall: a plate 0.04 mm thick must keep its thickness --------------------------------
// Its rim edges are as short as the step's above, but the faces at their ends are the plate's
// two sides: merging them would press the plate flat.
{
  const t = 0.04;
  const pts: P3[] = [[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0], [0, 0, t], [10, 0, t], [10, 10, t], [0, 10, t]];
  const src = polyhedronStep(pts, [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]);
  for (const iso of [false, true]) {
    const mode = iso ? "isotropic" : "default";
    const plain = stats(src, undefined, iso), merged = stats(src, 0.1, iso);
    check(merged.tris === plain.tris && merged.open === 0 && Math.abs(merged.volume - 100 * t) < 1e-9, `${mode}: a plate thinner than the tolerance is left alone (volume ${merged.volume.toFixed(3)} mm3)`);
  }
}

// ---- a groove that must not be pinched shut -------------------------------------------------------
// A V-groove 0.04 mm wide runs from one side of the box to its middle. Its inner end may taper
// to a point (three short edges merge). Its mouth may not: the two rims of the groove are parts
// of the top face's boundary, and merging the mouth would make that boundary touch itself.
{
  const w = 0.04, d = 0.02;
  const pts: P3[] = [
    [0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0], [0, 0, 10], [10, 0, 10], [10, 10, 10], [0, 10, 10],
    [0, 5 - w / 2, 10], [0, 5 + w / 2, 10], [0, 5, 10 - d],
    [5, 5 - w / 2, 10], [5, 5 + w / 2, 10], [5, 5, 10 - d],
  ];
  const src = polyhedronStep(pts, [
    [0, 3, 2, 1], [4, 5, 6, 7, 9, 12, 11, 8], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6],
    [3, 0, 4, 8, 10, 9, 7], [8, 11, 13, 10], [9, 10, 13, 12], [11, 12, 13],
  ]);
  for (const iso of [false, true]) {
    const mode = iso ? "isotropic" : "default";
    const plain = stats(src, undefined, iso), merged = stats(src, 0.05, iso);
    check(plain.open === 0 && plain.faces === 9, `${mode}: the grooved box meshes closed without merging (${plain.tris} triangles)`);
    check(merged.open === 0 && merged.merged?.reverted === false, `${mode}: merged mesh is closed and was not reverted`);
    check(merged.merged?.edges === 3 && merged.faces === 8, `${mode}: the groove's inner end merges (${merged.merged?.edges} edges, ${merged.faces} faces left)`);
    check(merged.minEdge < 0.03, `${mode}: the groove's mouth keeps its ${merged.minEdge.toFixed(3)} mm edges`);
    check(Math.abs(merged.volume - plain.volume) < 0.01, `${mode}: volume moves by ${Math.abs(merged.volume - plain.volume).toExponential(1)} mm3`);
  }
}

console.log(failed ? `\n${failed} check(s) FAILED` : "\nall defeature checks passed");
process.exit(failed ? 1 : 0);
