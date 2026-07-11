// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — 3MF (3D Manufacturing Format) read: OPC/ZIP container + model-part XML.
//
// Coverage: the core spec (indexed meshes, <components> re-use, <build> items, 4×3 row-major
// transforms, unit scaling to mm), colors from <basematerials> and the materials extension's
// <m:colorgroup>, and the production extension's p:path cross-part references (Bambu/Orca
// project files keep each object in its own /3D/Objects/*.model part). Textures, composites
// and slicer-private metadata are ignored. Zero dependencies: the ZIP central directory is
// walked by hand and deflate entries go through the platform's DecompressionStream
// (browser + Node ≥ 18), which is why reading is async.

export type RGB3MF = [number, number, number];

export interface ThreeMFItem {
  /** Object display name (`name` attribute); null when the file carries none (most slicers). */
  name: string | null;
  /** 3MF object type: "model" | "solidsupport" are solids; "surface" and "support" are open
   * geometry BY DESIGN and must not be audited for watertightness. */
  type: string;
  /** 3 numbers per vertex, in mm, build transform applied. */
  positions: Float64Array;
  /** 3 indices per triangle. */
  indices: Uint32Array;
  /** Palette index per triangle (-1 = unstyled); null when nothing in this item is colored. */
  colorOfTri: Int32Array | null;
}

export interface ThreeMFModel {
  /** Length-unit label of the model part ("mm", "inch", …); positions are always mm. */
  unit: string;
  /** One entry per build item (a re-used object appears once per instance). */
  items: ThreeMFItem[];
  /** Distinct sRGB colors (0..1) referenced by the items' colorOfTri indices. */
  palette: RGB3MF[];
}

// ---------------------------------------------------------------- ZIP container

interface ZipEntry { method: number; csize: number; localOffset: number; }

function readZipDirectory(bytes: Uint8Array): Map<string, ZipEntry> {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  // End-of-central-directory record: scan back over the (possibly present) archive comment.
  let eocd = -1;
  for (let i = bytes.length - 22, lo = Math.max(0, bytes.length - 65557); i >= lo; i--) {
    if (dv.getUint32(i, true) === 0x06054b50) { eocd = i; break; }
  }
  if (eocd < 0) throw new Error("Not a 3MF file (no ZIP directory found)");
  const count = dv.getUint16(eocd + 10, true);
  let off = dv.getUint32(eocd + 16, true);
  const entries = new Map<string, ZipEntry>();
  const dec = new TextDecoder();
  for (let i = 0; i < count && off + 46 <= bytes.length; i++) {
    if (dv.getUint32(off, true) !== 0x02014b50) break;
    const method = dv.getUint16(off + 10, true);
    const csize = dv.getUint32(off + 20, true);
    const nameLen = dv.getUint16(off + 28, true);
    const extraLen = dv.getUint16(off + 30, true);
    const commentLen = dv.getUint16(off + 32, true);
    const localOffset = dv.getUint32(off + 42, true);
    if (csize === 0xffffffff || localOffset === 0xffffffff) throw new Error("ZIP64 3MF archives are not supported");
    const name = dec.decode(bytes.subarray(off + 46, off + 46 + nameLen));
    entries.set(name.replace(/\\/g, "/"), { method, csize, localOffset });
    off += 46 + nameLen + extraLen + commentLen;
  }
  return entries;
}

async function zipExtract(bytes: Uint8Array, e: ZipEntry): Promise<Uint8Array> {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (dv.getUint32(e.localOffset, true) !== 0x04034b50) throw new Error("Corrupt 3MF: bad ZIP local header");
  // Sizes come from the central directory — the local header may carry zeros (data descriptor).
  const nameLen = dv.getUint16(e.localOffset + 26, true);
  const extraLen = dv.getUint16(e.localOffset + 28, true);
  const start = e.localOffset + 30 + nameLen + extraLen;
  const data = bytes.subarray(start, start + e.csize);
  if (e.method === 0) return data;
  if (e.method !== 8) throw new Error(`Unsupported ZIP compression method ${e.method} in 3MF`);
  // Copy: Blob wants a view over a plain (non-shared) ArrayBuffer.
  const stream = new Blob([new Uint8Array(data)]).stream().pipeThrough(new DecompressionStream("deflate-raw"));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

// ---------------------------------------------------------------- XML attribute scanning

/** Attribute value from an element's tag text; handles either quote style and whitespace
 * around "=". `name` must match exactly (no namespace prefix). */
function attr(tag: string, name: string): string | null {
  const m = new RegExp(`(?:^|[\\s"'])${name}\\s*=\\s*("([^"]*)"|'([^']*)')`).exec(tag);
  return m ? (m[2] ?? m[3])! : null;
}

/** Like attr() but the name may carry any namespace prefix (p:path, m:color …). */
function attrNS(tag: string, name: string): string | null {
  const m = new RegExp(`(?:^|[\\s"'])(?:[\\w.-]+:)?${name}\\s*=\\s*("([^"]*)"|'([^']*)')`).exec(tag);
  return m ? (m[2] ?? m[3])! : null;
}

/** Fast attribute scan for the vertex/triangle hot loops (millions of tags). Assumes
 * machine-written XML: no whitespace around "=", double or single quotes. */
function rawAttr(tag: string, name: string): string | null {
  for (const q of ['"', "'"]) {
    const probe = `${name}=${q}`;
    let i = -1;
    for (;;) {
      i = tag.indexOf(probe, i + 1);
      if (i < 0) break;
      // "pid=" must not match inside e.g. a hypothetical "xpid=" attribute.
      if (i === 0 || !/[\w:.-]/.test(tag[i - 1]!)) {
        const v0 = i + probe.length;
        const v1 = tag.indexOf(q, v0);
        return tag.slice(v0, v1 < 0 ? tag.length : v1);
      }
    }
  }
  return null;
}

// ---------------------------------------------------------------- model-part parsing

interface Ref { objectId: string; path: string | null; transform: Float64Array | null; }

interface ObjRes {
  type: string;
  name: string | null;
  /** colorOfTri is always allocated (all -1 when uncolored) — dropped at item level if unused. */
  mesh: { positions: Float64Array; indices: Uint32Array; colorOfTri: Int32Array } | null;
  components: Ref[];
}

interface ModelPart {
  unit: string;
  objects: Map<string, ObjRes>;
  items: Ref[];
}

interface ParseCtx {
  palette: RGB3MF[];
  paletteIdx: Map<string, number>;
}

function parseTransform(s: string | null): Float64Array | null {
  if (!s) return null;
  const n = s.trim().split(/\s+/).map(Number);
  return n.length === 12 && n.every(isFinite) ? Float64Array.from(n) : null;
}

function paletteIndex(ctx: ParseCtx, hex: string): number {
  // "#RRGGBB" or "#RRGGBBAA" (sRGB); alpha is ignored.
  const m = /^#?([0-9a-fA-F]{6})/.exec(hex.trim());
  if (!m) return -1;
  const key = m[1]!.toLowerCase();
  let idx = ctx.paletteIdx.get(key);
  if (idx === undefined) {
    idx = ctx.palette.length;
    const v = parseInt(key, 16);
    ctx.palette.push([((v >> 16) & 255) / 255, ((v >> 8) & 255) / 255, (v & 255) / 255]);
    ctx.paletteIdx.set(key, idx);
  }
  return idx;
}

/** Color property groups: <basematerials><base displaycolor=…/> and the materials extension's
 * <m:colorgroup><m:color color=…/>. Both map (group id, index) -> global palette index. */
function parseColorGroups(xml: string, ctx: ParseCtx): Map<string, number[]> {
  const groups = new Map<string, number[]>();
  const scan = (blockRe: RegExp, childName: string, colorAttr: string): void => {
    let m: RegExpExecArray | null;
    while ((m = blockRe.exec(xml)) !== null) {
      const id = attr(m[1]!, "id");
      if (!id) continue;
      const colors: number[] = [];
      const childRe = new RegExp(`<(?:[\\w.-]+:)?${childName}\\b([^>]*)`, "g");
      let c: RegExpExecArray | null;
      while ((c = childRe.exec(m[2] ?? "")) !== null) {
        const hex = attrNS(c[1]!, colorAttr);
        colors.push(hex ? paletteIndex(ctx, hex) : -1);
      }
      groups.set(id, colors);
    }
  };
  scan(/<basematerials\b([^>]*?)(?:\/>|>([\s\S]*?)<\/basematerials>)/g, "base", "displaycolor");
  scan(/<(?:[\w.-]+:)?colorgroup\b([^>]*?)(?:\/>|>([\s\S]*?)<\/(?:[\w.-]+:)?colorgroup>)/g, "color", "color");
  return groups;
}

function parseMesh(
  xml: string, from: number, to: number,
  colorGroups: Map<string, number[]>, defaultColor: number,
): ObjRes["mesh"] {
  const pos: number[] = [];
  let i = xml.indexOf("<vertices", from);
  if (i >= 0 && i < to) {
    const vEnd = xml.indexOf("</vertices>", i);
    const stop = vEnd < 0 || vEnd > to ? to : vEnd;
    while ((i = xml.indexOf("<vertex", i)) >= 0 && i < stop) {
      const e = xml.indexOf(">", i);
      const tag = xml.slice(i + 7, e);
      pos.push(
        parseFloat(rawAttr(tag, "x") ?? "0"),
        parseFloat(rawAttr(tag, "y") ?? "0"),
        parseFloat(rawAttr(tag, "z") ?? "0"),
      );
      i = e + 1;
    }
  }
  const idx: number[] = [];
  const col: number[] = [];
  i = xml.indexOf("<triangles", from);
  if (i >= 0 && i < to) {
    const tEnd = xml.indexOf("</triangles>", i);
    const stop = tEnd < 0 || tEnd > to ? to : tEnd;
    while ((i = xml.indexOf("<triangle", i)) >= 0 && i < stop) {
      if (xml[i + 9] === "s") { i += 9; continue; } // the <triangles> container itself
      const e = xml.indexOf(">", i);
      const tag = xml.slice(i + 9, e);
      idx.push(
        parseInt(rawAttr(tag, "v1") ?? "0", 10),
        parseInt(rawAttr(tag, "v2") ?? "0", 10),
        parseInt(rawAttr(tag, "v3") ?? "0", 10),
      );
      let c = defaultColor;
      const pid = rawAttr(tag, "pid");
      const p1 = rawAttr(tag, "p1");
      if (pid !== null || p1 !== null) {
        const group = pid !== null ? colorGroups.get(pid) : undefined;
        // A pid pointing at a non-color group (texture, composite) stays unstyled, not default.
        c = group ? (group[p1 !== null ? parseInt(p1, 10) : 0] ?? -1) : (pid !== null ? -1 : defaultColor);
      }
      col.push(c);
      i = e + 1;
    }
  }
  if (idx.length === 0) return null;
  return {
    positions: Float64Array.from(pos),
    indices: Uint32Array.from(idx),
    colorOfTri: Int32Array.from(col),
  };
}

function parseModelPart(xml: string, ctx: ParseCtx): ModelPart {
  const modelTag = /<model\b[^>]*/.exec(xml)?.[0] ?? "";
  const unit = attr(modelTag, "unit") ?? "millimeter";
  const colorGroups = parseColorGroups(xml, ctx);

  const objects = new Map<string, ObjRes>();
  let i = 0;
  for (;;) {
    i = xml.indexOf("<object", i);
    if (i < 0) break;
    const after = xml[i + 7];
    if (after !== " " && after !== "\t" && after !== "\n" && after !== "\r" && after !== ">" && after !== "/") { i += 7; continue; }
    const tagEnd = xml.indexOf(">", i);
    if (tagEnd < 0) break;
    const tag = xml.slice(i, tagEnd);
    const selfClosed = xml[tagEnd - 1] === "/";
    const blockEnd = selfClosed ? tagEnd + 1 : xml.indexOf("</object>", tagEnd);
    const end = blockEnd < 0 ? xml.length : blockEnd;
    const id = attr(tag, "id");
    if (id) {
      const pid = attr(tag, "pid");
      const pindex = parseInt(attr(tag, "pindex") ?? "0", 10);
      const defaultColor = pid !== null ? (colorGroups.get(pid)?.[pindex] ?? -1) : -1;
      const mesh = selfClosed ? null : parseMesh(xml, tagEnd, end, colorGroups, defaultColor);
      // Per spec an object holds EITHER a mesh OR components — skipping the component scan on
      // mesh objects keeps this pass linear on large files.
      const components: Ref[] = [];
      if (!mesh && !selfClosed) {
        let ci = tagEnd;
        while ((ci = xml.indexOf("<component", ci)) >= 0 && ci < end) {
          const ce = xml.indexOf(">", ci);
          const ctag = xml.slice(ci + 10, ce < 0 ? end : ce);
          if (/^[\w-]/.test(ctag)) { ci += 10; continue; } // "<components>" container, not a ref
          const cid = attr(ctag, "objectid");
          if (cid) components.push({ objectId: cid, path: attrNS(ctag, "path"), transform: parseTransform(attr(ctag, "transform")) });
          ci = ce < 0 ? end : ce + 1;
        }
      }
      objects.set(id, {
        type: attr(tag, "type") ?? "model",
        name: attr(tag, "name"),
        mesh,
        components,
      });
    }
    i = end + 1;
  }

  const items: Ref[] = [];
  const buildStart = xml.indexOf("<build");
  if (buildStart >= 0) {
    const buildEnd = xml.indexOf("</build>", buildStart);
    const itemRe = /<item\b([^>]*)/g;
    itemRe.lastIndex = buildStart;
    let m: RegExpExecArray | null;
    while ((m = itemRe.exec(xml)) !== null && (buildEnd < 0 || m.index < buildEnd)) {
      const oid = attr(m[1]!, "objectid");
      if (oid) items.push({ objectId: oid, path: attrNS(m[1]!, "path"), transform: parseTransform(attr(m[1]!, "transform")) });
    }
  }
  return { unit, objects, items };
}

// ---------------------------------------------------------------- transforms (4×3, row-vector)

// 3MF transform string "m00 m01 m02 m10 … m30 m31 m32": points are row vectors, translation is
// the last row — p' = [x y z 1] · M.
function applyXf(t: Float64Array, x: number, y: number, z: number, out: Float64Array, o: number): void {
  out[o] = x * t[0]! + y * t[3]! + z * t[6]! + t[9]!;
  out[o + 1] = x * t[1]! + y * t[4]! + z * t[7]! + t[10]!;
  out[o + 2] = x * t[2]! + y * t[5]! + z * t[8]! + t[11]!;
}

/** Compose "a then b" (both row-vector 4×3): C = A·B. */
function composeXf(a: Float64Array | null, b: Float64Array | null): Float64Array | null {
  if (!a) return b;
  if (!b) return a;
  const c = new Float64Array(12);
  for (let r = 0; r < 3; r++) {
    for (let k = 0; k < 3; k++) {
      c[r * 3 + k] = a[r * 3]! * b[k]! + a[r * 3 + 1]! * b[3 + k]! + a[r * 3 + 2]! * b[6 + k]!;
    }
  }
  for (let k = 0; k < 3; k++) {
    c[9 + k] = a[9]! * b[k]! + a[10]! * b[3 + k]! + a[11]! * b[6 + k]! + b[9 + k]!;
  }
  return c;
}

/** Determinant of the rotation block — negative means the transform mirrors, flipping winding. */
function xfDet(t: Float64Array): number {
  return t[0]! * (t[4]! * t[8]! - t[5]! * t[7]!)
    - t[1]! * (t[3]! * t[8]! - t[5]! * t[6]!)
    + t[2]! * (t[3]! * t[7]! - t[4]! * t[6]!);
}

// ---------------------------------------------------------------- reader

const UNIT_SCALE: Record<string, [string, number]> = {
  micron: ["µm", 0.001],
  millimeter: ["mm", 1],
  centimeter: ["cm", 10],
  inch: ["inch", 25.4],
  foot: ["ft", 304.8],
  meter: ["m", 1000],
};

/** Normalize an OPC part reference ("/3D/3dmodel.model") to the ZIP entry name. */
function partName(path: string): string {
  return path.replace(/^\//, "");
}

interface MeshPiece { mesh: NonNullable<ObjRes["mesh"]>; xf: Float64Array | null; }

/** Read a 3MF archive into per-build-item meshes (mm, transforms applied) plus a color palette. */
export async function read3MF(buf: ArrayBuffer | Uint8Array): Promise<ThreeMFModel> {
  const bytes = buf instanceof Uint8Array ? buf : new Uint8Array(buf);
  const entries = readZipDirectory(bytes);
  const decoder = new TextDecoder();
  const loadText = async (path: string): Promise<string | null> => {
    const e = entries.get(partName(path));
    return e ? decoder.decode(await zipExtract(bytes, e)) : null;
  };

  // Root model part via the OPC start-part relationship; fall back to the conventional name.
  let rootPath = "3D/3dmodel.model";
  const rels = await loadText("_rels/.rels");
  if (rels) {
    const relRe = /<Relationship\b[^>]*/g;
    let m: RegExpExecArray | null;
    while ((m = relRe.exec(rels)) !== null) {
      if ((attr(m[0]!, "Type") ?? "").endsWith("3dmodel")) { rootPath = partName(attr(m[0]!, "Target") ?? rootPath); break; }
    }
  }
  if (!entries.has(rootPath)) {
    const any = [...entries.keys()].find((n) => n.toLowerCase().endsWith(".model"));
    if (!any) throw new Error("No 3D model part found in the 3MF archive");
    rootPath = any;
  }

  const ctx: ParseCtx = { palette: [], paletteIdx: new Map() };
  const parts = new Map<string, ModelPart>();
  const getPart = async (path: string): Promise<ModelPart | null> => {
    let p = parts.get(path);
    if (!p) {
      const xml = await loadText(path);
      if (xml === null) return null;
      p = parseModelPart(xml, ctx);
      parts.set(path, p);
    }
    return p;
  };

  const root = await getPart(rootPath);
  if (!root) throw new Error("No 3D model part found in the 3MF archive");
  const [unitLabel, scale] = UNIT_SCALE[root.unit.toLowerCase()] ?? ["mm", 1];

  // Collect an object's mesh pieces, recursing through <components> (transforms compose,
  // p:path hops into other model parts). Depth-capped against reference cycles.
  const collect = async (path: string, id: string, xf: Float64Array | null, out: MeshPiece[], depth: number): Promise<void> => {
    if (depth > 32) return;
    const obj = (await getPart(path))?.objects.get(id);
    if (!obj) return;
    if (obj.mesh) out.push({ mesh: obj.mesh, xf });
    for (const c of obj.components) {
      await collect(c.path ? partName(c.path) : path, c.objectId, composeXf(c.transform, xf), out, depth + 1);
    }
  };

  // No <build> section (unusual but legal for libraries of objects): show every mesh object.
  const buildItems: Ref[] = root.items.length > 0
    ? root.items
    : [...root.objects.keys()].map((id) => ({ objectId: id, path: null, transform: null }));

  const items: ThreeMFItem[] = [];
  for (const it of buildItems) {
    const itemPath = it.path ? partName(it.path) : rootPath;
    const obj = (await getPart(itemPath))?.objects.get(it.objectId);
    if (!obj || obj.type === "other") continue;
    const pieces: MeshPiece[] = [];
    await collect(itemPath, it.objectId, it.transform, pieces, 0);
    let nV = 0, nT = 0;
    for (const p of pieces) { nV += p.mesh.positions.length / 3; nT += p.mesh.indices.length / 3; }
    if (nT === 0) continue;
    const positions = new Float64Array(nV * 3);
    const indices = new Uint32Array(nT * 3);
    const colorOfTri = new Int32Array(nT);
    let colored = false;
    let vo = 0, to = 0;
    for (const p of pieces) {
      const P = p.mesh.positions, I = p.mesh.indices, C = p.mesh.colorOfTri;
      const pv = P.length / 3, pt = I.length / 3;
      if (p.xf) {
        for (let v = 0; v < pv; v++) applyXf(p.xf, P[v * 3]!, P[v * 3 + 1]!, P[v * 3 + 2]!, positions, (vo + v) * 3);
      } else {
        positions.set(P, vo * 3);
      }
      // A mirroring transform flips winding; swap two indices so outward orientation survives.
      const mirrored = p.xf ? xfDet(p.xf) < 0 : false;
      for (let t = 0; t < pt; t++) {
        const b = (to + t) * 3;
        indices[b] = I[t * 3]! + vo;
        indices[b + 1] = I[t * 3 + (mirrored ? 2 : 1)]! + vo;
        indices[b + 2] = I[t * 3 + (mirrored ? 1 : 2)]! + vo;
        const c = C[t]!;
        colorOfTri[to + t] = c;
        if (c >= 0) colored = true;
      }
      vo += pv; to += pt;
    }
    if (scale !== 1) for (let k = 0; k < positions.length; k++) positions[k]! *= scale;
    items.push({
      name: obj.name,
      type: obj.type,
      positions,
      indices,
      colorOfTri: colored ? colorOfTri : null,
    });
  }

  return { unit: unitLabel, items, palette: ctx.palette };
}
