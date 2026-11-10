// SPDX-License-Identifier: AGPL-3.0-only
// Synthetic regressions for ABC-corpus fixes whose original models are not committed. Each case is a
// minimal hand-built input that reproduces the failure mode, not the corpus model itself.
//   #6c  B-spline curve projection lands on the wrong branch of a curve that passes near itself.
//   #6e  a degenerate "closed" curve (empty parameter span) and a wild projection made the periodic
//        unwrap spin for minutes / forever.
// (#6d, the pole-to-pole hemisphere rim, has no synthetic reproduction: a hand-built half-ball either
// meshes through another path or hits a separate grid issue, so it is covered only by the corpus.)
import { parseStep } from "../src/step/parser.ts";
import { Table } from "../src/step/entities.ts";
import { makeCurve } from "../src/geom/curves.ts";
import { makeSurface } from "../src/geom/surfaces.ts";
import { unwrap } from "../src/mesh/tessellate.ts";
import { importStep } from "../src/index.ts";

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

if (failures) { console.log(`geometry-regressions: ${failures} assertion(s) failed`); process.exit(1); }
console.log("geometry-regressions: all passed");
