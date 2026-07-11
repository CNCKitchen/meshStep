// SPDX-License-Identifier: AGPL-3.0-only
// 3MF reader self-test: builds archives in memory (stored + deflated entries) covering the core
// spec, basematerials / m:colorgroup colors, components, mirrored build transforms, unit scaling
// and Bambu-style production-extension p:path sub-parts — then asserts on what read3MF returns.
// Usage: node test/threemf.ts [--out <dir>]   (--out also writes the generated .3mf files)
import { deflateRawSync, crc32 } from "node:zlib";
import { writeFileSync, mkdirSync } from "node:fs";
import { join } from "node:path";
import { read3MF } from "../src/index.ts";

// ---- minimal ZIP writer ----
interface ZipFile { name: string; text: string; deflate?: boolean }
function makeZip(files: ZipFile[]): Uint8Array {
  const chunks: Buffer[] = [];
  const central: Buffer[] = [];
  let offset = 0;
  for (const f of files) {
    const raw = Buffer.from(f.text);
    const comp = f.deflate ? deflateRawSync(raw) : raw;
    const method = f.deflate ? 8 : 0;
    const crc = crc32(raw) >>> 0;
    const name = Buffer.from(f.name);
    const lh = Buffer.alloc(30);
    lh.writeUInt32LE(0x04034b50, 0);
    lh.writeUInt16LE(20, 4);
    lh.writeUInt16LE(method, 8);
    lh.writeUInt32LE(crc, 14);
    lh.writeUInt32LE(comp.length, 18);
    lh.writeUInt32LE(raw.length, 22);
    lh.writeUInt16LE(name.length, 26);
    chunks.push(lh, name, Buffer.from(comp));
    const cd = Buffer.alloc(46);
    cd.writeUInt32LE(0x02014b50, 0);
    cd.writeUInt16LE(20, 4);
    cd.writeUInt16LE(20, 6);
    cd.writeUInt16LE(method, 10);
    cd.writeUInt32LE(crc, 16);
    cd.writeUInt32LE(comp.length, 20);
    cd.writeUInt32LE(raw.length, 24);
    cd.writeUInt16LE(name.length, 28);
    cd.writeUInt32LE(offset, 42);
    central.push(cd, name);
    offset += 30 + name.length + comp.length;
  }
  const cdBuf = Buffer.concat(central);
  const eocd = Buffer.alloc(22);
  eocd.writeUInt32LE(0x06054b50, 0);
  eocd.writeUInt16LE(files.length, 8);
  eocd.writeUInt16LE(files.length, 10);
  eocd.writeUInt32LE(cdBuf.length, 12);
  eocd.writeUInt32LE(offset, 16);
  return new Uint8Array(Buffer.concat([...chunks, cdBuf, eocd]));
}

// ---- fixtures ----
const CUBE_VERTS: [number, number, number][] = [
  [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
  [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
];
const CUBE_TRIS: [number, number, number][] = [
  [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
  [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
  [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
];
function cubeXml(size: number, triExtra: (t: number) => string = () => ""): string {
  const v = CUBE_VERTS.map(([x, y, z]) => `<vertex x="${x * size}" y="${y * size}" z="${z * size}"/>`).join("");
  const t = CUBE_TRIS.map(([a, b, c], i) => `<triangle v1="${a}" v2="${b}" v3="${c}"${triExtra(i)}/>`).join("");
  return `<mesh><vertices>${v}</vertices><triangles>${t}</triangles></mesh>`;
}

const CONTENT_TYPES = `<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>`;
const RELS = `<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>`;

// 1) Minimal single-object cube, stored (uncompressed) entries.
const cube3mf = makeZip([
  { name: "[Content_Types].xml", text: CONTENT_TYPES },
  { name: "_rels/.rels", text: RELS },
  {
    name: "3D/3dmodel.model",
    text: `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources><object id="1" type="model" name="Cube">${cubeXml(10)}</object></resources>
 <build><item objectid="1"/></build>
</model>`,
  },
]);

// 2) Colors + instancing + mirroring + inches, deflated entries. Object 1 defaults to red via
// basematerials pid/pindex with two triangles overridden (blue via m:colorgroup); object 2 is
// placed twice — translated and mirrored (negative determinant must not flip it inside out).
const colored3mf = makeZip([
  { name: "[Content_Types].xml", text: CONTENT_TYPES, deflate: true },
  { name: "_rels/.rels", text: RELS, deflate: true },
  {
    name: "3D/3dmodel.model",
    deflate: true,
    text: `<?xml version="1.0" encoding="UTF-8"?>
<model unit="inch" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
 xmlns:m="http://schemas.microsoft.com/3dmanufacturing/material/2015/02">
 <resources>
  <basematerials id="5"><base name="Red" displaycolor="#FF0000"/><base name="Green" displaycolor="#00FF00"/></basematerials>
  <m:colorgroup id="6"><m:color color="#0000FFCC"/></m:colorgroup>
  <object id="1" type="model" name="Painted" pid="5" pindex="0">${cubeXml(1, (t) => (t < 2 ? ` pid="6" p1="0"` : ""))}</object>
  <object id="2" type="model" name="Plain">${cubeXml(1)}</object>
 </resources>
 <build>
  <item objectid="1"/>
  <item objectid="2" transform="1 0 0 0 1 0 0 0 1 2 0 0"/>
  <item objectid="2" transform="-1 0 0 0 1 0 0 0 1 -2 0 0"/>
 </build>
</model>`,
  },
]);

// 3) Bambu/Orca-style production-extension layout: the root part holds a component object whose
// geometry lives in its own model part, referenced via p:path; the component adds a transform.
const bambu3mf = makeZip([
  { name: "[Content_Types].xml", text: CONTENT_TYPES, deflate: true },
  { name: "_rels/.rels", text: RELS, deflate: true },
  {
    name: "3D/3dmodel.model",
    deflate: true,
    text: `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
 xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">
 <resources>
  <object id="2" p:UUID="00000001-0000-0000-0000-000000000001" type="model" name="Benchy Part">
   <components><component p:path="/3D/Objects/object_1.model" objectid="1" transform="1 0 0 0 1 0 0 0 1 5 5 0"/></components>
  </object>
 </resources>
 <build><item objectid="2" transform="1 0 0 0 1 0 0 0 1 100 100 0"/></build>
</model>`,
  },
  {
    name: "3D/Objects/object_1.model",
    deflate: true,
    text: `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources><object id="1" type="model">${cubeXml(10)}</object></resources>
 <build/>
</model>`,
  },
]);

// ---- checks ----
let failures = 0;
function check(label: string, ok: boolean, detail = ""): void {
  console.log(`${ok ? "  ok " : "FAIL "} ${label}${ok || !detail ? "" : ` — ${detail}`}`);
  if (!ok) failures++;
}

function signedVolume(pos: Float64Array, idx: Uint32Array): number {
  let vol = 0;
  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t]! * 3, b = idx[t + 1]! * 3, c = idx[t + 2]! * 3;
    vol += pos[a]! * (pos[b + 1]! * pos[c + 2]! - pos[b + 2]! * pos[c + 1]!)
      - pos[a + 1]! * (pos[b]! * pos[c + 2]! - pos[b + 2]! * pos[c]!)
      + pos[a + 2]! * (pos[b]! * pos[c + 1]! - pos[b + 1]! * pos[c]!);
  }
  return vol / 6;
}

const cube = await read3MF(cube3mf);
check("cube: one item", cube.items.length === 1);
check("cube: name", cube.items[0]?.name === "Cube");
check("cube: 12 tris / 8 verts", cube.items[0]?.indices.length === 36 && cube.items[0]?.positions.length === 24);
check("cube: unit mm", cube.unit === "mm");
check("cube: volume 1000", Math.abs(signedVolume(cube.items[0]!.positions, cube.items[0]!.indices) - 1000) < 1e-9,
  String(signedVolume(cube.items[0]!.positions, cube.items[0]!.indices)));
check("cube: no colors", cube.palette.length === 0 && cube.items[0]?.colorOfTri === null);

const col = await read3MF(colored3mf);
check("colored: three items (instancing)", col.items.length === 3, String(col.items.length));
check("colored: unit inch", col.unit === "inch");
const painted = col.items[0]!;
check("colored: palette red, green, blue", col.palette.length === 3, JSON.stringify(col.palette));
check("colored: default red on tri 2", painted.colorOfTri?.[2] === 0 && col.palette[0]?.[0] === 1);
check("colored: override blue on tri 0", painted.colorOfTri?.[0] === 2 && col.palette[2]?.[2] === 1);
const inch = 25.4;
check("colored: inch scaling", Math.abs(signedVolume(painted.positions, painted.indices) - inch ** 3) < 1e-6,
  String(signedVolume(painted.positions, painted.indices)));
const inst = col.items[1]!, mirr = col.items[2]!;
check("colored: translated instance", Math.abs(inst.positions[0]! - 2 * inch) < 1e-9, String(inst.positions[0]));
check("colored: mirrored instance keeps positive volume",
  signedVolume(mirr.positions, mirr.indices) > 0, String(signedVolume(mirr.positions, mirr.indices)));
check("colored: mirrored instance placed at -2in..-3in", Math.min(...mirr.positions.filter((_, i) => i % 3 === 0)) < -2 * inch + 1e-9);

const bam = await read3MF(bambu3mf);
check("bambu: one item", bam.items.length === 1);
check("bambu: name from root part", bam.items[0]?.name === "Benchy Part");
check("bambu: component+item transforms compose", (() => {
  const p = bam.items[0]!.positions;
  let minX = Infinity, minY = Infinity;
  for (let i = 0; i < p.length; i += 3) { minX = Math.min(minX, p[i]!); minY = Math.min(minY, p[i + 1]!); }
  return Math.abs(minX - 105) < 1e-9 && Math.abs(minY - 105) < 1e-9;
})(), JSON.stringify(bam.items[0]?.positions.slice(0, 3)));
check("bambu: volume 1000", Math.abs(signedVolume(bam.items[0]!.positions, bam.items[0]!.indices) - 1000) < 1e-9);

const outIdx = process.argv.indexOf("--out");
if (outIdx >= 0 && process.argv[outIdx + 1]) {
  const dir = process.argv[outIdx + 1]!;
  mkdirSync(dir, { recursive: true });
  writeFileSync(join(dir, "test-cube.3mf"), cube3mf);
  writeFileSync(join(dir, "test-colored.3mf"), colored3mf);
  writeFileSync(join(dir, "test-bambu.3mf"), bambu3mf);
  console.log(`wrote sample archives to ${dir}`);
}

console.log(failures === 0 ? "\nAll 3MF checks passed." : `\n${failures} check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
