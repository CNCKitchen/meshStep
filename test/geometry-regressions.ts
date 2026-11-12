// SPDX-License-Identifier: AGPL-3.0-only
// Synthetic regressions for ABC-corpus fixes whose original models are not committed. Each case is a
// minimal hand-built input that reproduces the failure mode, not the corpus model itself.
//   #6c  B-spline curve projection lands on the wrong branch of a curve that passes near itself.
//   #6e  a degenerate "closed" curve (empty parameter span) and a wild projection made the periodic
//        unwrap spin for minutes / forever.
//   torus band with a backtracking rim (Ontos tapered collet nut): the band mesher folded it.
// (#6d, the pole-to-pole hemisphere rim, has no synthetic reproduction: a hand-built half-ball either
// meshes through another path or hits a separate grid issue, so it is covered only by the corpus.)
import { parseStep } from "../src/step/parser.ts";
import { Table } from "../src/step/entities.ts";
import { makeCurve } from "../src/geom/curves.ts";
import { makeSurface } from "../src/geom/surfaces.ts";
import { unwrap } from "../src/mesh/tessellate.ts";
import { importStep } from "../src/index.ts";
import { selfIntersections } from "./self-intersect.ts";

let failures = 0;
const check = (label: string, cond: boolean, detail: string): void => {
  if (cond) console.log(`ok   ${label}`);
  else { failures++; console.log(`  ASSERT FAILED [${label}]: ${detail}`); }
};
const dist = (a: readonly number[], b: readonly number[]): number => Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!);
const table = (data: string): Table => new Table(parseStep(`ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${data}\nENDSEC;\nEND-ISO-10303-21;\n`));
const pt = (id: number, p: number[]): string => `#${id}=CARTESIAN_POINT('',(${p.map((x) => x.toFixed(6)).join(",")}));`;

// ---- #6c: closed hairpin B-spline, branches ~0.1 apart, well inside one sample spacing (~0.4). The
// return branch's control points are offset by half a span so its samples sit between the outgoing
// branch's: the nearest SAMPLE is then often on the wrong branch, and at the turn and the start/end
// junction the shared sample sends a greedy refine down the wrong side.
{
  const cps = [[0, 0, 0], [2, 0.05, 0], [4, 0.05, 0], [6, 0.05, 0], [8, 0.05, 0], [10, 0.05, 0], [10.5, 0, 0],
    [9, -0.05, 0], [7, -0.05, 0], [5, -0.05, 0], [3, -0.05, 0], [1, -0.05, 0], [0, 0, 0]];
  const ids = cps.map((_, i) => 100 + i);
  const knots = Array.from({ length: 11 }, (_, i) => `${i}.`).join(",");
  const mults = [4, ...Array(9).fill(1), 4].join(",");
  const t = table([...cps.map((p, i) => pt(ids[i]!, p)),
    `#1=B_SPLINE_CURVE_WITH_KNOTS('',3,(${ids.map((i) => `#${i}`).join(",")}),.UNSPECIFIED.,.T.,.F.,(${mults}),(${knots}),.UNSPECIFIED.);`].join("\n"));
  const c = makeCurve(t, 1, 1)!;
  check("hairpin: curve built and closed", !!c && c.closed, `closed=${c?.closed}`);
  let worst = 0, worstT = 0;
  for (let i = 0; i <= 2000; i++) {
    const tt = c.t0 + ((c.t1 - c.t0) * i) / 2000;
    const p = c.evaluate(tt);
    const r = dist(c.evaluate(c.project(p)), p);
    if (r > worst) { worst = r; worstT = tt; }
  }
  check("hairpin: every on-curve point projects back onto itself", worst < 1e-5, `worst residual ${worst.toExponential(2)} at t=${worstT.toFixed(4)}`);
}

// ---- #6e: empty-span B-spline is not closed; its extrusion is not periodic ----
{
  const t = table([pt(201, [1, 2, 3]), pt(202, [1, 2, 3]), `#203=DIRECTION('',(0.,0.,1.));`, `#204=VECTOR('',#203,1.);`,
    `#1=B_SPLINE_CURVE_WITH_KNOTS('',1,(#201,#202),.UNSPECIFIED.,.F.,.F.,(2,2),(0.,1.E-17),.UNSPECIFIED.);`,
    `#2=SURFACE_OF_LINEAR_EXTRUSION('',#1,#204);`].join("\n"));
  const c = makeCurve(t, 1, 1);
  check("degenerate curve: empty parameter span is not closed", !!c && !c.closed, `closed=${c?.closed}`);
  const s = makeSurface(t, 2, 1);
  check("degenerate curve: its extrusion is not periodic", !!s && !s.periodicU, `periodicU=${s?.periodicU} uPeriod=${s?.uPeriod}`);
}
// ---- closed B-spline profile: the surface normal wraps the periodic parameter like evaluate does.
// The unroll charts a seam-spanning face past t1; the extrusion's normal there clamped both tangent
// probes to t1 and fell back to +Z, the revolution's to its axis, and every triangle beyond the
// seam was oriented against that (ABC 00021718 #172: 281 flipped slivers along the seam side).
{
  const loop = (ps: number[][]): string => {
    const ids = ps.map((_, i) => 300 + i);
    return [...ps.map((p, i) => pt(ids[i]!, p)),
      `#1=B_SPLINE_CURVE_WITH_KNOTS('',3,(${ids.map((i) => `#${i}`).join(",")}),.UNSPECIFIED.,.T.,.F.,(4,1,1,4),(0.,1.,2.,3.),.UNSPECIFIED.);`].join("\n");
  };
  const near = (a: readonly number[], b: readonly number[]): boolean => dist(a, b) < 1e-9;
  const fmt = (n: readonly number[]): string => n.map((x) => x.toFixed(4)).join(",");
  const ex = makeSurface(table([loop([[10, 0, 0], [10, 10, 0], [-10, 10, 0], [-10, -10, 0], [10, -10, 0], [10, 0, 0]]),
    `#203=DIRECTION('',(0.,0.,1.));`, `#204=VECTOR('',#203,5.);`, `#2=SURFACE_OF_LINEAR_EXTRUSION('',#1,#204);`].join("\n")), 2, 1)!;
  check("closed extrusion: periodic in u", !!ex?.periodicU && ex.uPeriod === 3, `periodicU=${ex?.periodicU} uPeriod=${ex?.uPeriod}`);
  for (const d of [0.05, -0.05]) {
    const inside = ex.normal((d > 0 ? 0 : 3) + d, 0.5), past = ex.normal((d > 0 ? 3 : 0) + d, 0.5);
    check(`closed extrusion: normal ${d > 0 ? "past t1" : "before t0"} matches the wrapped one`, near(inside, past), `${fmt(past)} vs ${fmt(inside)}`);
  }
  const rv = makeSurface(table([loop([[25, 0, 0], [25, 0, 5], [15, 0, 5], [15, 0, -5], [25, 0, -5], [25, 0, 0]]),
    pt(210, [0, 0, 0]), `#211=DIRECTION('',(0.,0.,1.));`, `#212=AXIS1_PLACEMENT('',#210,#211);`, `#2=SURFACE_OF_REVOLUTION('',#1,#212);`].join("\n")), 2, 1)!;
  check("closed revolution profile: periodic in v", !!rv?.periodicV && rv.vPeriod === 3, `periodicV=${rv?.periodicV} vPeriod=${rv?.vPeriod}`);
  for (const d of [0.05, -0.05]) {
    const inside = rv.normal(0.3, (d > 0 ? 0 : 3) + d), past = rv.normal(0.3, (d > 0 ? 3 : 0) + d);
    check(`closed revolution profile: normal ${d > 0 ? "past t1" : "before t0"} matches the wrapped one`, near(inside, past), `${fmt(past)} vs ${fmt(inside)}`);
  }
}
{
  // unwrap must return promptly on a non-positive period (formerly an infinite loop) and on a point
  // hundreds of millions of periods away (formerly ~5e8 single-period steps).
  const t0 = Date.now();
  const a: [number, number][] = [[1e-17, 0], [1e-17, 0]];
  unwrap(a, 0, -1e-17);
  check("unwrap: non-positive period is a no-op", a[1]![0] === 1e-17, `u=${a[1]![0]}`);
  const b: [number, number][] = [[-0.3239, 0], [-4.8885e11, 0]]; // ~5e11 single-period steps before
  unwrap(b, 0, 1.04);
  const d = b[1]![0] - b[0]![0];
  check("unwrap: far jump lands within half a period", d > -0.52 && d <= 0.52, `d=${d}`);
  const c: [number, number][] = [[0, 0], [3, 0], [-3, 0]];
  unwrap(c, 0, 2 * Math.PI);
  check("unwrap: ordinary single-period wrap unchanged", c[1]![0] === 3 && c[2]![0] === -3 + 2 * Math.PI, JSON.stringify(c));
  check("unwrap: returns promptly", Date.now() - t0 < 1000, `${Date.now() - t0} ms`);
}

// ---- bounded-surface model (GEOMETRIC_SET of CURVE_BOUNDED_SURFACE, no solid): it imports as one
// synthetic body with id 0, which is no entity in the file. The product-structure pass looked that
// id up and threw "STEP: missing entity #0" (gear.stp, a 1998 Pro/E surface export).
{
  const corners = [[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0]];
  const lines: string[] = [`#1=CARTESIAN_POINT('',(0.,0.,0.));`, `#2=DIRECTION('',(0.,0.,1.));`, `#3=DIRECTION('',(1.,0.,0.));`,
    `#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);`, `#5=PLANE('',#4);`];
  const segs: number[] = [];
  corners.forEach((p, i) => {
    const q = corners[(i + 1) % 4]!, b = 100 + i * 10, d = [q[0]! - p[0]!, q[1]! - p[1]!, 0], len = Math.hypot(d[0]!, d[1]!);
    lines.push(pt(b, p), `#${b + 1}=DIRECTION('',(${d.map((x) => (x / len).toFixed(6)).join(",")}));`, `#${b + 2}=VECTOR('',#${b + 1},${len.toFixed(1)});`,
      `#${b + 3}=LINE('',#${b},#${b + 2});`, `#${b + 4}=TRIMMED_CURVE('',#${b + 3},(0.),(1.),.T.,.UNSPECIFIED.);`,
      `#${b + 5}=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,.T.,#${b + 4});`);
    segs.push(b + 5);
  });
  lines.push(`#200=OUTER_BOUNDARY_CURVE('',(${segs.map((s) => `#${s}`).join(",")}),.F.);`, `#201=CURVE_BOUNDED_SURFACE('',#5,(#200),.F.);`,
    `#202=GEOMETRIC_SET('',(#201));`);
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  let tris = 0, err = "";
  try { tris = importStep(src).mesh.indices.length / 3; } catch (e) { err = (e as Error).message; }
  check("bounded-surface model: imports without a product-structure error", !err && tris > 0, err || `tris=${tris}`);
}

// ---- torus band whose rim BACKTRACKS in u (Ontos tapered collet nut, face #92192). The inner side
// of a ring torus (R=65, r=49.7) between a bottom circle and a slotted zigzag rim whose curl runs
// back in u for 7° per slot. The band mesher's per-angle loft folded under each curl: the strip was
// covered 2-3x (7368 same-face triangle crossings on the real part) while the mesh stayed watertight,
// on-surface and consistently oriented — no diagnostic fired. Area is exact via Green's theorem on
// the torus metric dA = r(R + r cos v) du dv, so both an over-covered fold and a chord-gouged seam
// (the unroll fallback's straight seam sides) fail it.
{
  const R = 65, r = 49.7, D = Math.PI / 180;
  const onT = (u: number, v: number): number[] => [(R + r * Math.cos(v)) * Math.cos(u), (R + r * Math.cos(v)) * Math.sin(u), r * Math.sin(v)];
  const corners: [number, number][] = []; // (u°, v°), u decreasing; per 60° slot: diagonal, bottom, curl, top run
  for (let k = 0; k < 6; k++) { const u0 = -k * 60; corners.push([u0, 166], [u0 - 28, 181.5], [u0 - 37, 180], [u0 - 40, 174], [u0 - 33, 166]); }
  corners.push([-360, 166]);
  const uv: [number, number][] = [];
  for (let i = 0; i + 1 < corners.length; i++) {
    const [ua, va] = corners[i]!, [ub, vb] = corners[i + 1]!;
    const n = Math.max(2, Math.ceil(Math.hypot(ub - ua, vb - va) / 0.5));
    for (let j = 0; j < n; j++) uv.push([(ua + ((ub - ua) * j) / n) * D, (va + ((vb - va) * j) / n) * D]);
  }
  uv.push(uv[0]!);
  const vBot = 184 * D, rBot = R + r * Math.cos(vBot), zBot = r * Math.sin(vBot);
  const cps = uv.map(([u, v], i) => pt(1000 + i, onT(u, v)));
  const nC = uv.length;
  const lines = [...cps,
    `#10=B_SPLINE_CURVE_WITH_KNOTS('',1,(${uv.map((_, i) => `#${1000 + i}`).join(",")}),.POLYLINE_FORM.,.T.,.F.,(2,${Array(nC - 2).fill(1).join(",")},2),(${Array.from({ length: nC }, (_, i) => `${i}.`).join(",")}),.UNSPECIFIED.);`,
    `#11=VERTEX_POINT('',#1000);`, `#12=EDGE_CURVE('',#11,#11,#10,.T.);`, `#13=ORIENTED_EDGE('',*,*,#12,.T.);`, `#14=EDGE_LOOP('',(#13));`,
    pt(20, [0, 0, zBot]), `#21=DIRECTION('',(0.,0.,1.));`, `#22=DIRECTION('',(1.,0.,0.));`, `#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);`,
    `#24=CIRCLE('',#23,${rBot.toFixed(9)});`, pt(25, [rBot, 0, zBot]), `#26=VERTEX_POINT('',#25);`,
    `#27=EDGE_CURVE('',#26,#26,#24,.T.);`, `#28=ORIENTED_EDGE('',*,*,#27,.T.);`, `#29=EDGE_LOOP('',(#28));`,
    pt(30, [0, 0, 0]), `#31=AXIS2_PLACEMENT_3D('',#30,#21,#22);`, `#32=TOROIDAL_SURFACE('',#31,${R},${r});`,
    `#33=FACE_BOUND('',#14,.T.);`, `#34=FACE_BOUND('',#29,.T.);`, `#35=ADVANCED_FACE('',(#33,#34),#32,.F.);`,
    `#36=OPEN_SHELL('',(#35));`, `#37=SHELL_BASED_SURFACE_MODEL('',(#36));`];
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  const res = importStep(src);
  const P = res.mesh.positions, I = res.mesh.indices;
  let area = 0;
  for (let t = 0; t < I.length / 3; t++) {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const e1 = [P[b]! - P[a]!, P[b + 1]! - P[a + 1]!, P[b + 2]! - P[a + 2]!], e2 = [P[c]! - P[a]!, P[c + 1]! - P[a + 1]!, P[c + 2]! - P[a + 2]!];
    area += 0.5 * Math.hypot(e1[1]! * e2[2]! - e1[2]! * e2[1]!, e1[2]! * e2[0]! - e1[0]! * e2[2]!, e1[0]! * e2[1]! - e1[1]! * e2[0]!);
  }
  // exact: |∮zig G(v) du − ∮bottom G(v) du| with G(v) = r(R v + r sin v), both loops walked in −u
  const G = (v: number): number => r * (R * v + r * Math.sin(v));
  let zig = 0;
  // (the closing sample repeats uv[0] at u=0; the unwrapped walk ends at u=−2π)
  const uvU = [...uv.slice(0, -1), [uv[0]![0] - 2 * Math.PI, uv[0]![1]] as [number, number]];
  for (let i = 0; i + 1 < uvU.length; i++) zig += G((uvU[i]![1] + uvU[i + 1]![1]) / 2) * (uvU[i + 1]![0] - uvU[i]![0]);
  const exact = Math.abs(zig + G(vBot) * 2 * Math.PI);
  const si = selfIntersections(P, I, res.faceOfTri);
  check("backtracking torus rim: no same-face triangle crossings (fold)", si.sameFace === 0, `${si.sameFace} crossing pairs`);
  check("backtracking torus rim: area matches the trimmed band", Math.abs(area - exact) < 0.005 * exact, `mesh ${area.toFixed(2)} vs exact ${exact.toFixed(2)} mm²`);
  check("backtracking torus rim: watertight against its rims", res.diagnostics.nonManifoldEdges === 0, `nonManifold=${res.diagnostics.nonManifoldEdges}`);
}

// ---- cylinder band whose top rim backtracks by only 1° (jack_in_the_box cone #6986: a lobe B-spline
// leaving its rim 0.6° backwards). The band loft runs every interior ring at the bottom rim's
// angles, so even this small backtrack became a reversed column folded up the FULL band height;
// the former 5%-of-a-turn slack let it through. The loft runs at the LOWER rim's angles, so the
// curling rim is the bottom one (full circle above at z=20). Exact area = r·|∮ (zTop − z) du|.
{
  const r = 10, D = Math.PI / 180, zTop = 20;
  // (u°, z): u decreasing; at u=-100 the rim steps 1° BACK while climbing, then runs on
  const corners: [number, number][] = [[0, 10], [-100, 10], [-99, 12], [-110, 13], [-120, 10], [-360, 10]];
  const uz: [number, number][] = [];
  for (let i = 0; i + 1 < corners.length; i++) {
    const [ua, za] = corners[i]!, [ub, zb] = corners[i + 1]!;
    const n = Math.max(2, Math.ceil(Math.hypot((ub - ua) * D * r, zb - za) / 0.25));
    for (let j = 0; j < n; j++) uz.push([(ua + ((ub - ua) * j) / n) * D, za + ((zb - za) * j) / n]);
  }
  const nC = uz.length + 1;
  const lines = [...uz.map(([u, z], i) => pt(1000 + i, [r * Math.cos(u), r * Math.sin(u), z])),
    `#10=B_SPLINE_CURVE_WITH_KNOTS('',1,(${[...uz.map((_, i) => `#${1000 + i}`), "#1000"].join(",")}),.POLYLINE_FORM.,.T.,.F.,(2,${Array(nC - 2).fill(1).join(",")},2),(${Array.from({ length: nC }, (_, i) => `${i}.`).join(",")}),.UNSPECIFIED.);`,
    `#11=VERTEX_POINT('',#1000);`, `#12=EDGE_CURVE('',#11,#11,#10,.T.);`, `#13=ORIENTED_EDGE('',*,*,#12,.T.);`, `#14=EDGE_LOOP('',(#13));`,
    pt(20, [0, 0, zTop]), `#21=DIRECTION('',(0.,0.,1.));`, `#22=DIRECTION('',(1.,0.,0.));`, `#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);`,
    `#24=CIRCLE('',#23,${r});`, pt(25, [r, 0, zTop]), `#26=VERTEX_POINT('',#25);`,
    `#27=EDGE_CURVE('',#26,#26,#24,.T.);`, `#28=ORIENTED_EDGE('',*,*,#27,.T.);`, `#29=EDGE_LOOP('',(#28));`,
    pt(30, [0, 0, 0]), `#31=AXIS2_PLACEMENT_3D('',#30,#21,#22);`, `#32=CYLINDRICAL_SURFACE('',#31,${r});`,
    `#33=FACE_BOUND('',#14,.T.);`, `#34=FACE_BOUND('',#29,.T.);`, `#35=ADVANCED_FACE('',(#33,#34),#32,.T.);`,
    `#36=OPEN_SHELL('',(#35));`, `#37=SHELL_BASED_SURFACE_MODEL('',(#36));`];
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  const res = importStep(src);
  const P = res.mesh.positions, I = res.mesh.indices;
  let area = 0;
  for (let t = 0; t < I.length / 3; t++) {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const e1 = [P[b]! - P[a]!, P[b + 1]! - P[a + 1]!, P[b + 2]! - P[a + 2]!], e2 = [P[c]! - P[a]!, P[c + 1]! - P[a + 1]!, P[c + 2]! - P[a + 2]!];
    area += 0.5 * Math.hypot(e1[1]! * e2[2]! - e1[2]! * e2[1]!, e1[2]! * e2[0]! - e1[0]! * e2[2]!, e1[0]! * e2[1]! - e1[1]! * e2[0]!);
  }
  const walk = [...uz, [-2 * Math.PI, uz[0]![1]] as [number, number]];
  let zu = 0;
  for (let i = 0; i + 1 < walk.length; i++) zu += (zTop - (walk[i]![1] + walk[i + 1]![1]) / 2) * (walk[i + 1]![0] - walk[i]![0]);
  const exact = r * Math.abs(zu);
  const si = selfIntersections(P, I, res.faceOfTri);
  check("1° rim backtrack on a cylinder band: no fold", si.sameFace === 0, `${si.sameFace} crossing pairs`);
  check("1° rim backtrack on a cylinder band: area matches", Math.abs(area - exact) < 0.005 * exact, `mesh ${area.toFixed(2)} vs exact ${exact.toFixed(2)} mm²`);
}

// ---- cylinder band whose top rim STEPS within one bottom column (ABC 00020182 cone #836: a 7°
// slanted step on a rim lofted from a 27-point bottom circle, 13° apart). The loft's rings live
// at the bottom's angles, so the last ring cut the step's corner, landed on the wrong side of the
// top rim and the closing stitch folded. Such a band now goes to the unroll (true-boundary CDT).
{
  const r = 1.5, D = Math.PI / 180, zBot = -2.5;
  const corners: [number, number][] = [[0, 0], [-187.4, 0], [-194.6, 5.5], [-352.8, 5.5], [-360, 0]];
  const uz: [number, number][] = [];
  for (let i = 0; i + 1 < corners.length; i++) {
    const [ua, za] = corners[i]!, [ub, zb] = corners[i + 1]!;
    const n = Math.max(2, Math.ceil(Math.hypot((ub - ua) * D * r, zb - za) / 0.05));
    for (let j = 0; j < n; j++) uz.push([(ua + ((ub - ua) * j) / n) * D, za + ((zb - za) * j) / n]);
  }
  const nC = uz.length + 1;
  const lines = [...uz.map(([u, z], i) => pt(1000 + i, [r * Math.cos(u), r * Math.sin(u), z])),
    `#10=B_SPLINE_CURVE_WITH_KNOTS('',1,(${[...uz.map((_, i) => `#${1000 + i}`), "#1000"].join(",")}),.POLYLINE_FORM.,.T.,.F.,(2,${Array(nC - 2).fill(1).join(",")},2),(${Array.from({ length: nC }, (_, i) => `${i}.`).join(",")}),.UNSPECIFIED.);`,
    `#11=VERTEX_POINT('',#1000);`, `#12=EDGE_CURVE('',#11,#11,#10,.T.);`, `#13=ORIENTED_EDGE('',*,*,#12,.T.);`, `#14=EDGE_LOOP('',(#13));`,
    pt(20, [0, 0, zBot]), `#21=DIRECTION('',(0.,0.,1.));`, `#22=DIRECTION('',(1.,0.,0.));`, `#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);`,
    `#24=CIRCLE('',#23,${r});`, pt(25, [r, 0, zBot]), `#26=VERTEX_POINT('',#25);`,
    `#27=EDGE_CURVE('',#26,#26,#24,.T.);`, `#28=ORIENTED_EDGE('',*,*,#27,.F.);`, `#29=EDGE_LOOP('',(#28));`,
    pt(30, [0, 0, 0]), `#31=AXIS2_PLACEMENT_3D('',#30,#21,#22);`, `#32=CYLINDRICAL_SURFACE('',#31,${r});`,
    `#33=FACE_BOUND('',#14,.T.);`, `#34=FACE_BOUND('',#29,.T.);`, `#35=ADVANCED_FACE('',(#33,#34),#32,.T.);`,
    `#36=OPEN_SHELL('',(#35));`, `#37=SHELL_BASED_SURFACE_MODEL('',(#36));`];
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  const res = importStep(src);
  const P = res.mesh.positions, I = res.mesh.indices;
  let area = 0;
  for (let t = 0; t < I.length / 3; t++) {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const e1 = [P[b]! - P[a]!, P[b + 1]! - P[a + 1]!, P[b + 2]! - P[a + 2]!], e2 = [P[c]! - P[a]!, P[c + 1]! - P[a + 1]!, P[c + 2]! - P[a + 2]!];
    area += 0.5 * Math.hypot(e1[1]! * e2[2]! - e1[2]! * e2[1]!, e1[2]! * e2[0]! - e1[0]! * e2[2]!, e1[0]! * e2[1]! - e1[1]! * e2[0]!);
  }
  const walk = [...uz, [-2 * Math.PI, uz[0]![1]] as [number, number]];
  let zu = 0;
  for (let i = 0; i + 1 < walk.length; i++) zu += ((walk[i]![1] + walk[i + 1]![1]) / 2 - zBot) * (walk[i + 1]![0] - walk[i]![0]);
  const exact = r * Math.abs(zu);
  const si = selfIntersections(P, I, res.faceOfTri);
  check("rim step narrower than a loft column: no fold", si.sameFace === 0, `${si.sameFace} crossing pairs`);
  check("rim step narrower than a loft column: area matches", Math.abs(area - exact) < 0.01 * exact, `mesh ${area.toFixed(3)} vs exact ${exact.toFixed(3)} mm²`);
}

// ---- torus TUBE rim (an elbow's end section, rims winding v) that steps 1° back in v. A 5% backtrack
// slack for such rims made 28 ABC models worse (00041690: 31 -> 48858 crossing pairs). Here the loft
// with that slack folded (55 pairs) AND meshed the complementary 320° of the torus (area 8x).
// Exact area: |∮ (U(v) - uCircle) dF(v)| along the stepped rim, F(v) = r(R v + r sin v).
{
  const R = 20, r = 5, D = Math.PI / 180;
  const onT = (u: number, v: number): number[] => [(R + r * Math.cos(v)) * Math.cos(u), (R + r * Math.cos(v)) * Math.sin(u), r * Math.sin(v)];
  const corners: [number, number][] = [[0, 40], [-150, 40], [-149, 45], [-160, 46], [-170, 40], [-360, 40]]; // (v°, u°)
  const vu: [number, number][] = [];
  for (let i = 0; i + 1 < corners.length; i++) {
    const [va, ua] = corners[i]!, [vb, ub] = corners[i + 1]!;
    const n = Math.max(2, Math.ceil(Math.hypot(vb - va, ub - ua) / 2));
    for (let j = 0; j < n; j++) vu.push([(va + ((vb - va) * j) / n) * D, (ua + ((ub - ua) * j) / n) * D]);
  }
  const nC = vu.length + 1;
  const lines = [...vu.map(([v, u], i) => pt(1000 + i, onT(u, v))),
    `#10=B_SPLINE_CURVE_WITH_KNOTS('',1,(${[...vu.map((_, i) => `#${1000 + i}`), "#1000"].join(",")}),.POLYLINE_FORM.,.T.,.F.,(2,${Array(nC - 2).fill(1).join(",")},2),(${Array.from({ length: nC }, (_, i) => `${i}.`).join(",")}),.UNSPECIFIED.);`,
    `#11=VERTEX_POINT('',#1000);`, `#12=EDGE_CURVE('',#11,#11,#10,.T.);`, `#13=ORIENTED_EDGE('',*,*,#12,.T.);`, `#14=EDGE_LOOP('',(#13));`,
    pt(20, [R, 0, 0]), `#21=DIRECTION('',(0.,1.,0.));`, `#22=DIRECTION('',(1.,0.,0.));`, `#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);`,
    `#24=CIRCLE('',#23,${r});`, pt(25, onT(0, 0)), `#26=VERTEX_POINT('',#25);`,
    `#27=EDGE_CURVE('',#26,#26,#24,.T.);`, `#28=ORIENTED_EDGE('',*,*,#27,.T.);`, `#29=EDGE_LOOP('',(#28));`,
    pt(30, [0, 0, 0]), `#31=DIRECTION('',(0.,0.,1.));`, `#32=AXIS2_PLACEMENT_3D('',#30,#31,#22);`, `#34=TOROIDAL_SURFACE('',#32,${R},${r});`,
    `#35=FACE_BOUND('',#14,.T.);`, `#36=FACE_BOUND('',#29,.T.);`, `#37=ADVANCED_FACE('',(#35,#36),#34,.T.);`,
    `#38=OPEN_SHELL('',(#37));`, `#39=SHELL_BASED_SURFACE_MODEL('',(#38));`];
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  const res = importStep(src);
  const P = res.mesh.positions, I = res.mesh.indices;
  let area = 0;
  for (let t = 0; t < I.length / 3; t++) {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const e1 = [P[b]! - P[a]!, P[b + 1]! - P[a + 1]!, P[b + 2]! - P[a + 2]!], e2 = [P[c]! - P[a]!, P[c + 1]! - P[a + 1]!, P[c + 2]! - P[a + 2]!];
    area += 0.5 * Math.hypot(e1[1]! * e2[2]! - e1[2]! * e2[1]!, e1[2]! * e2[0]! - e1[0]! * e2[2]!, e1[0]! * e2[1]! - e1[1]! * e2[0]!);
  }
  const F = (v: number): number => r * (R * v + r * Math.sin(v));
  const walk = [...vu, [-2 * Math.PI, vu[0]![1]] as [number, number]];
  let A = 0;
  for (let i = 0; i + 1 < walk.length; i++) A += ((walk[i]![1] + walk[i + 1]![1]) / 2) * (F(walk[i + 1]![0]) - F(walk[i]![0]));
  const exact = Math.abs(A);
  const si = selfIntersections(P, I, res.faceOfTri);
  check("torus tube rim stepping back 1°: no fold", si.sameFace === 0, `${si.sameFace} crossing pairs`);
  check("torus tube rim stepping back 1°: area matches the elbow segment", Math.abs(area - exact) < 0.005 * exact, `mesh ${area.toFixed(2)} vs exact ${exact.toFixed(2)} mm²`);
}

// ---- band whose rims CROSS each other (ABC 00036584 cone #4871: a B-spline rim that dips to the apex
// and spikes 2x above the opposite circle). That is no loft-column artefact; declining it to the
// unroll left 2-6 boundary constraints missing in the CDT (open edges against the neighbours) where
// the loft stays on its rims. The face's mesh boundary must be its two rims: length within 0.5%.
{
  const r = 5, D = Math.PI / 180, zC = 3.37;
  const corners: [number, number][] = [[0, 3.03], [-36, 0.76], [-87, 0], [-138, 0.76], [-174, 3.03], [-245, 3.03], [-268, 7.12], [-291, 3.03], [-360, 3.03]];
  const uz: [number, number][] = [];
  for (let i = 0; i + 1 < corners.length; i++) {
    const [ua, za] = corners[i]!, [ub, zb] = corners[i + 1]!;
    const n = Math.max(2, Math.ceil(Math.hypot((ub - ua) * D * r, zb - za) / 0.5));
    for (let j = 0; j < n; j++) uz.push([(ua + ((ub - ua) * j) / n) * D, za + ((zb - za) * j) / n]);
  }
  const nC = uz.length + 1;
  const lines = [...uz.map(([u, z], i) => pt(1000 + i, [r * Math.cos(u), r * Math.sin(u), z])),
    `#10=B_SPLINE_CURVE_WITH_KNOTS('',1,(${[...uz.map((_, i) => `#${1000 + i}`), "#1000"].join(",")}),.POLYLINE_FORM.,.T.,.F.,(2,${Array(nC - 2).fill(1).join(",")},2),(${Array.from({ length: nC }, (_, i) => `${i}.`).join(",")}),.UNSPECIFIED.);`,
    `#11=VERTEX_POINT('',#1000);`, `#12=EDGE_CURVE('',#11,#11,#10,.T.);`, `#13=ORIENTED_EDGE('',*,*,#12,.T.);`, `#14=EDGE_LOOP('',(#13));`,
    pt(20, [0, 0, zC]), `#21=DIRECTION('',(0.,0.,1.));`, `#22=DIRECTION('',(1.,0.,0.));`, `#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);`,
    `#24=CIRCLE('',#23,${r});`, pt(25, [r, 0, zC]), `#26=VERTEX_POINT('',#25);`,
    `#27=EDGE_CURVE('',#26,#26,#24,.T.);`, `#28=ORIENTED_EDGE('',*,*,#27,.F.);`, `#29=EDGE_LOOP('',(#28));`,
    pt(30, [0, 0, 0]), `#31=AXIS2_PLACEMENT_3D('',#30,#21,#22);`, `#32=CYLINDRICAL_SURFACE('',#31,${r});`,
    `#33=FACE_BOUND('',#14,.T.);`, `#34=FACE_BOUND('',#29,.T.);`, `#35=ADVANCED_FACE('',(#33,#34),#32,.T.);`,
    `#36=OPEN_SHELL('',(#35));`, `#37=SHELL_BASED_SURFACE_MODEL('',(#36));`];
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  const res = importStep(src);
  const P = res.mesh.positions, I = res.mesh.indices;
  const cnt = new Map<string, number>();
  for (let t = 0; t < I.length / 3; t++) for (let e = 0; e < 3; e++) {
    const a = I[t * 3 + e]!, b = I[t * 3 + (e + 1) % 3]!, k = a < b ? `${a},${b}` : `${b},${a}`;
    cnt.set(k, (cnt.get(k) ?? 0) + 1);
  }
  let bLen = 0;
  for (const [k, n] of cnt) if (n === 1) { const [a, b] = k.split(",").map(Number); bLen += dist([P[a! * 3]!, P[a! * 3 + 1]!, P[a! * 3 + 2]!], [P[b! * 3]!, P[b! * 3 + 1]!, P[b! * 3 + 2]!]); }
  let rimLen = 2 * Math.PI * r;
  const w = [...uz, uz[0]!];
  for (let i = 0; i + 1 < w.length; i++) rimLen += dist([r * Math.cos(w[i]![0]), r * Math.sin(w[i]![0]), w[i]![1]], [r * Math.cos(w[i + 1]![0]), r * Math.sin(w[i + 1]![0]), w[i + 1]![1]]);
  check("band rims crossing each other: mesh boundary is the two rims", Math.abs(bLen - rimLen) < 0.005 * rimLen, `boundary ${bLen.toFixed(3)} vs rims ${rimLen.toFixed(3)}`);
}

// ---- cylinder band whose bottom rim has narrow DEEP slots under a coarse circle (ABC 00025895 #259:
// a slotted collet, 12 slots 0.02 rad wide and 14 deep, under a 34-point circle). Every loft ring
// keeps the slots at 1/nRings of their depth with near-vertical walls; the closing stitch fans each
// coarse circle point to a whole wall and folds behind one wall of every slot (374 crossing pairs
// here, area +2.9%, 48k triangles). The band now replays that stitch and declines to the unroll.
{
  const r = 5, D = Math.PI / 180, zRim = -3, zTop = -1.32;
  const corners: [number, number][] = [];
  for (let k = 0; k < 12; k++) { const u0 = -k * 30; corners.push([u0, zRim], [u0 - 1, zRim - 14], [u0 - 2, zRim]); }
  corners.push([-360, zRim]);
  const uz: [number, number][] = [];
  for (let i = 0; i + 1 < corners.length; i++) {
    const [ua, za] = corners[i]!, [ub, zb] = corners[i + 1]!;
    const n = Math.max(2, Math.ceil(Math.hypot((ub - ua) * D * r, zb - za) / 0.25));
    for (let j = 0; j < n; j++) uz.push([(ua + ((ub - ua) * j) / n) * D, za + ((zb - za) * j) / n]);
  }
  const nC = uz.length + 1;
  const lines = [...uz.map(([u, z], i) => pt(1000 + i, [r * Math.cos(u), r * Math.sin(u), z])),
    `#10=B_SPLINE_CURVE_WITH_KNOTS('',1,(${[...uz.map((_, i) => `#${1000 + i}`), "#1000"].join(",")}),.POLYLINE_FORM.,.T.,.F.,(2,${Array(nC - 2).fill(1).join(",")},2),(${Array.from({ length: nC }, (_, i) => `${i}.`).join(",")}),.UNSPECIFIED.);`,
    `#11=VERTEX_POINT('',#1000);`, `#12=EDGE_CURVE('',#11,#11,#10,.T.);`, `#13=ORIENTED_EDGE('',*,*,#12,.T.);`, `#14=EDGE_LOOP('',(#13));`,
    pt(20, [0, 0, zTop]), `#21=DIRECTION('',(0.,0.,1.));`, `#22=DIRECTION('',(1.,0.,0.));`, `#23=AXIS2_PLACEMENT_3D('',#20,#21,#22);`,
    `#24=CIRCLE('',#23,${r});`, pt(25, [r, 0, zTop]), `#26=VERTEX_POINT('',#25);`,
    `#27=EDGE_CURVE('',#26,#26,#24,.T.);`, `#28=ORIENTED_EDGE('',*,*,#27,.F.);`, `#29=EDGE_LOOP('',(#28));`,
    pt(30, [0, 0, 0]), `#31=AXIS2_PLACEMENT_3D('',#30,#21,#22);`, `#32=CYLINDRICAL_SURFACE('',#31,${r});`,
    `#33=FACE_BOUND('',#14,.T.);`, `#34=FACE_BOUND('',#29,.T.);`, `#35=ADVANCED_FACE('',(#33,#34),#32,.T.);`,
    `#36=OPEN_SHELL('',(#35));`, `#37=SHELL_BASED_SURFACE_MODEL('',(#36));`];
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  const res = importStep(src);
  const P = res.mesh.positions, I = res.mesh.indices;
  let area = 0;
  for (let t = 0; t < I.length / 3; t++) {
    const a = I[t * 3]! * 3, b = I[t * 3 + 1]! * 3, c = I[t * 3 + 2]! * 3;
    const e1 = [P[b]! - P[a]!, P[b + 1]! - P[a + 1]!, P[b + 2]! - P[a + 2]!], e2 = [P[c]! - P[a]!, P[c + 1]! - P[a + 1]!, P[c + 2]! - P[a + 2]!];
    area += 0.5 * Math.hypot(e1[1]! * e2[2]! - e1[2]! * e2[1]!, e1[2]! * e2[0]! - e1[0]! * e2[2]!, e1[0]! * e2[1]! - e1[1]! * e2[0]!);
  }
  const walk = [...uz, [-2 * Math.PI, uz[0]![1]] as [number, number]];
  let zu = 0;
  for (let i = 0; i + 1 < walk.length; i++) zu += (zTop - (walk[i]![1] + walk[i + 1]![1]) / 2) * (walk[i + 1]![0] - walk[i]![0]);
  const exact = r * Math.abs(zu);
  const si = selfIntersections(P, I, res.faceOfTri);
  check("slotted rim under a coarse circle: no fold", si.sameFace === 0, `${si.sameFace} crossing pairs`);
  check("slotted rim under a coarse circle: area matches", Math.abs(area - exact) < 0.005 * exact, `mesh ${area.toFixed(3)} vs exact ${exact.toFixed(3)} mm²`);
}

// ---- torus patches meshed by the param grid (u around the axis, v around the tube). The grid spaces
// its columns uniformly in u at the face's MEDIAN parallel and sized them by the tube radius alone.
// A chord spanning du on the parallel at v leaves the surface by |(R + r cos v) cos v|·du²/8, so
// the face's most bent parallel must set the column angle, both for the isotropic target and for
// the stretch along the parallels (whose step is a length at the median parallel).
const torusPatch = (R: number, r: number, uDeg: number, v0Deg: number, v1Deg: number, tol: number, maxEdge: number) => {
  const D = Math.PI / 180, u0 = 0, u1 = uDeg * D, v0 = v0Deg * D, v1 = v1Deg * D;
  const onT = (u: number, v: number): number[] => [(R + r * Math.cos(v)) * Math.cos(u), (R + r * Math.cos(v)) * Math.sin(u), r * Math.sin(v)];
  const dir = (id: number, d: number[]): string => `#${id}=DIRECTION('',(${d.map((x) => x.toFixed(12)).join(",")}));`;
  const pt9 = (id: number, p: number[]): string => `#${id}=CARTESIAN_POINT('',(${p.map((x) => x.toFixed(9)).join(",")}));`;
  const lines = [pt9(1, [0, 0, 0]), dir(2, [0, 0, 1]), dir(3, [1, 0, 0]), `#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);`, `#5=TOROIDAL_SURFACE('',#4,${R},${r});`,
    pt9(20, onT(u0, v0)), pt9(21, onT(u1, v0)), pt9(22, onT(u1, v1)), pt9(23, onT(u0, v1)),
    ...[10, 11, 12, 13].map((id) => `#${id}=VERTEX_POINT('',#${id + 10});`)];
  // parallels at v0 (#30) and v1 (#31); tube circles at u0 (#32) and u1 (#33)
  for (const [id, v] of [[30, v0], [31, v1]] as const) lines.push(pt9(id + 100, [0, 0, r * Math.sin(v)]), `#${id + 200}=AXIS2_PLACEMENT_3D('',#${id + 100},#2,#3);`, `#${id}=CIRCLE('',#${id + 200},${(R + r * Math.cos(v)).toFixed(9)});`);
  for (const [id, u] of [[32, u0], [33, u1]] as const) lines.push(pt9(id + 100, [R * Math.cos(u), R * Math.sin(u), 0]), dir(id + 300, [-Math.sin(u), Math.cos(u), 0]), dir(id + 400, [Math.cos(u), Math.sin(u), 0]), `#${id + 200}=AXIS2_PLACEMENT_3D('',#${id + 100},#${id + 300},#${id + 400});`, `#${id}=CIRCLE('',#${id + 200},${r});`);
  lines.push(`#40=EDGE_CURVE('',#10,#11,#30,.T.);`, `#41=EDGE_CURVE('',#12,#11,#33,.T.);`, `#42=EDGE_CURVE('',#13,#12,#31,.T.);`, `#43=EDGE_CURVE('',#13,#10,#32,.T.);`,
    `#50=ORIENTED_EDGE('',*,*,#40,.T.);`, `#51=ORIENTED_EDGE('',*,*,#41,.F.);`, `#52=ORIENTED_EDGE('',*,*,#42,.F.);`, `#53=ORIENTED_EDGE('',*,*,#43,.T.);`,
    `#60=EDGE_LOOP('',(#50,#51,#52,#53));`, `#61=FACE_OUTER_BOUND('',#60,.T.);`, `#62=ADVANCED_FACE('',(#61),#5,.T.);`, `#63=OPEN_SHELL('',(#62));`, `#64=SHELL_BASED_SURFACE_MODEL('',(#63));`);
  const src = `ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n${lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
  const res = importStep(src, { surfaceDeviation: tol, maxEdge, normalDeviation: 60 });
  const P = res.mesh.positions, I = res.mesh.indices;
  const offT = (p: number[]): number => Math.abs(Math.hypot(Math.hypot(p[0]!, p[1]!) - R, p[2]!) - r);
  let area = 0, devEdge = 0;
  for (let t = 0; t < I.length / 3; t++) {
    const V = [0, 1, 2].map((k) => [P[I[t * 3 + k]! * 3]!, P[I[t * 3 + k]! * 3 + 1]!, P[I[t * 3 + k]! * 3 + 2]!]);
    for (let e = 0; e < 3; e++) devEdge = Math.max(devEdge, offT([0, 1, 2].map((k) => (V[e]![k]! + V[(e + 1) % 3]![k]!) / 2)));
    const e1 = [0, 1, 2].map((k) => V[1]![k]! - V[0]![k]!), e2 = [0, 1, 2].map((k) => V[2]![k]! - V[0]![k]!);
    area += 0.5 * Math.hypot(e1[1]! * e2[2]! - e1[2]! * e2[1]!, e1[2]! * e2[0]! - e1[0]! * e2[2]!, e1[0]! * e2[1]! - e1[1]! * e2[0]!);
  }
  const exact = r * (u1 - u0) * (R * (v1 - v0) + r * (Math.sin(v1) - Math.sin(v0)));
  return { surface: makeSurface(table(lines.slice(0, 5).join("\n")), 5, 1)!, devEdge: devEdge / tol, area, exact, folds: selfIntersections(P, I, res.faceOfTri).sameFace };
};
// Fat torus (R < 2r), inner side (ABC 00036882 #255: R 240, r 239): around the axis is the tighter
// bend. The patch got 23° columns and edge midpoints 4.8x the chord tolerance off the surface.
{
  const R = 240, r = 239, D = Math.PI / 180, tol = 0.2;
  const t = torusPatch(R, r, 90, 127, 170, tol, 200);
  const near = (a: number, b: number): boolean => Math.abs(a - b) < 1e-9 * Math.max(1, Math.abs(b));
  // floor 0: the step is exact - the chord term at the most bent parallel, or the normal term
  const col = (a: number, b: number, tolC: number, nd: number, fl = 0): number => t.surface.parallelColumnAngle!(a * D, b * D, tolC, nd, fl);
  const K = (v: number): number => Math.abs((R + r * Math.cos(v)) * Math.cos(v));
  const chordDu = (k: number, tolC: number): number => Math.sqrt((8 * tolC) / k);
  check("fat torus: widest parallel of the patch sets the column angle", near(col(127, 170, tol, 1), chordDu(K(127 * D), tol)), `${col(127, 170, tol, 1)} vs ${chordDu(K(127 * D), tol)}`);
  check("fat torus: column angle peaks inside the range where cos v = -R/2r", near(col(100, 140, tol, 1), chordDu((R * R) / (4 * r), tol)), `${col(100, 140, tol, 1)} vs ${chordDu((R * R) / (4 * r), tol)}`);
  check("fat torus: a whole turn sees the outer equator", near(col(-180, 180, tol, 1), chordDu(R + r, tol)) && near(col(300, 400, tol, 1), chordDu(R + r, tol)), `${col(-180, 180, tol, 1)}, ${col(300, 400, tol, 1)}`);
  check("fat torus: the normal term binds at the innermost parallel", near(col(127, 180, 1e3, 0.2), 0.2), `${col(127, 180, 1e3, 0.2)} vs 0.2`);
  // the innermost parallel has radius 1 mm: a 6 mm floor exempts it; the 15 mm one at 160° binds
  const rho160 = R + r * Math.cos(160 * D);
  check("fat torus: parallels shorter than the floor ask for nothing finer", Math.abs(col(160, 180, 1e3, 0.2, 6) - 6 / rho160) < 0.05 * (6 / rho160), `${col(160, 180, 1e3, 0.2, 6)} vs ~${6 / rho160}`);
  check("fat torus inner patch: edge chords within 1.5x the chord tolerance", t.devEdge < 1.5, `${t.devEdge.toFixed(2)}x`);
  check("fat torus inner patch: no fold", t.folds === 0, `${t.folds} crossing pairs`);
  check("fat torus inner patch: area matches", Math.abs(t.area - t.exact) < 0.005 * t.exact, `mesh ${t.area.toFixed(2)} vs exact ${t.exact.toFixed(2)} mm²`);
}
// Ordinary torus (R 40, r 20), u 0-30°, v ±150°: the tube radius was the tighter bend, so the target
// was right, but the grid stretched its columns to the u curvature step, a length it measured at
// the outer equator and laid out at the median parallel (ρ ≈ 43 vs 60): 2.8x the chord tolerance
// on the equator. The same stretch left 3.9x on unrolled elbow sections (ABC 00032129).
{
  const t = torusPatch(40, 20, 30, -150, 150, 0.05, 40);
  check("wide torus patch: equator chords within 1.6x the chord tolerance", t.devEdge < 1.6, `${t.devEdge.toFixed(2)}x`);
  check("wide torus patch: no fold", t.folds === 0, `${t.folds} crossing pairs`);
  check("wide torus patch: area matches", Math.abs(t.area - t.exact) < 0.005 * t.exact, `mesh ${t.area.toFixed(2)} vs exact ${t.exact.toFixed(2)} mm²`);
}

if (failures) { console.log(`geometry-regressions: ${failures} assertion(s) failed`); process.exit(1); }
console.log("geometry-regressions: all passed");
