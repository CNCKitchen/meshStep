// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — binary/ASCII STL read & write.

export interface TriSoup {
  /** Flat triangle vertices: 9 numbers per triangle (v0xyz, v1xyz, v2xyz). */
  positions: Float64Array;
  triangleCount: number;
}

export interface IndexedMesh {
  /** 3 numbers per vertex. */
  positions: Float64Array;
  /** 3 indices per triangle. */
  indices: Uint32Array;
}

export interface BBox {
  min: [number, number, number];
  max: [number, number, number];
  diagonal: number;
}

const decoder = new TextDecoder();

/** Detect format and parse an STL (binary or ASCII) into a flat triangle soup. */
export function readSTL(buf: ArrayBuffer | Uint8Array): TriSoup {
  const bytes = buf instanceof Uint8Array ? buf : new Uint8Array(buf);
  if (isBinarySTL(bytes)) return readBinarySTL(bytes);
  return readAsciiSTL(decoder.decode(bytes));
}

/**
 * Robust binary detection: a binary STL is exactly 84 + 50*count bytes, where `count` is the
 * uint32 at offset 80. ASCII files almost never satisfy this by coincidence, and (unlike the
 * "starts with 'solid'" heuristic) this is not fooled by binary headers that begin with "solid".
 */
export function isBinarySTL(bytes: Uint8Array): boolean {
  if (bytes.length < 84) return false;
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = dv.getUint32(80, true);
  return bytes.length === 84 + count * 50;
}

export function readBinarySTL(bytes: Uint8Array): TriSoup {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = dv.getUint32(80, true);
  const positions = new Float64Array(count * 9);
  let o = 84;
  for (let i = 0; i < count; i++) {
    const base = i * 9;
    // Skip the 3-float face normal (offset o..o+12), read the 9 vertex floats.
    for (let k = 0; k < 9; k++) positions[base + k] = dv.getFloat32(o + 12 + k * 4, true);
    o += 50; // 12 (normal) + 36 (verts) + 2 (attr byte count)
  }
  return { positions, triangleCount: count };
}

export function readAsciiSTL(text: string): TriSoup {
  const nums: number[] = [];
  // Full float syntax including negative exponents ("1.234e-05"): a class like [\d.eE+] without
  // "-" fails on the exponent sign and silently drops the whole vertex.
  const F = String.raw`([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)`;
  const re = new RegExp(String.raw`vertex\s+${F}\s+${F}\s+${F}`, "g");
  let m: RegExpExecArray | null;
  while ((m = re.exec(text)) !== null) {
    nums.push(parseFloat(m[1]!), parseFloat(m[2]!), parseFloat(m[3]!));
  }
  const positions = Float64Array.from(nums);
  return { positions, triangleCount: positions.length / 9 };
}

/** Write a binary STL from an indexed mesh; flat per-triangle normals are computed. */
export function writeBinarySTL(mesh: IndexedMesh, header = "meshStep binary STL"): Uint8Array {
  const triCount = mesh.indices.length / 3;
  const out = new Uint8Array(84 + triCount * 50);
  const dv = new DataView(out.buffer);
  new TextEncoder().encodeInto(header.slice(0, 79), out.subarray(0, 79));
  dv.setUint32(80, triCount, true);
  let o = 84;
  const p = mesh.positions;
  const idx = mesh.indices;
  for (let t = 0; t < triCount; t++) {
    const ia = idx[t * 3]! * 3, ib = idx[t * 3 + 1]! * 3, ic = idx[t * 3 + 2]! * 3;
    const ax = p[ia]!, ay = p[ia + 1]!, az = p[ia + 2]!;
    const bx = p[ib]!, by = p[ib + 1]!, bz = p[ib + 2]!;
    const cx = p[ic]!, cy = p[ic + 1]!, cz = p[ic + 2]!;
    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const wx = cx - ax, wy = cy - ay, wz = cz - az;
    let nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
    const nl = Math.hypot(nx, ny, nz) || 1;
    nx /= nl; ny /= nl; nz /= nl;
    dv.setFloat32(o, nx, true); dv.setFloat32(o + 4, ny, true); dv.setFloat32(o + 8, nz, true);
    dv.setFloat32(o + 12, ax, true); dv.setFloat32(o + 16, ay, true); dv.setFloat32(o + 20, az, true);
    dv.setFloat32(o + 24, bx, true); dv.setFloat32(o + 28, by, true); dv.setFloat32(o + 32, bz, true);
    dv.setFloat32(o + 36, cx, true); dv.setFloat32(o + 40, cy, true); dv.setFloat32(o + 44, cz, true);
    dv.setUint16(o + 48, 0, true);
    o += 50;
  }
  return out;
}

/** Weld a triangle soup's coincident vertices (quantised to eps) into an indexed mesh, so edge
 * topology — open/non-manifold edge audits — works on it. STL facets repeat shared vertices
 * bitwise-identically, so the quantisation only ever merges genuinely coincident points. */
export function indexSoup(soup: TriSoup, eps = 1e-6): IndexedMesh {
  const p = soup.positions;
  const map = new Map<string, number>();
  const pos: number[] = [];
  const indices = new Uint32Array(soup.triangleCount * 3);
  for (let i = 0; i < soup.triangleCount * 3; i++) {
    const x = p[i * 3]!, y = p[i * 3 + 1]!, z = p[i * 3 + 2]!;
    const key = `${Math.round(x / eps)},${Math.round(y / eps)},${Math.round(z / eps)}`;
    let idx = map.get(key);
    if (idx === undefined) {
      idx = pos.length / 3;
      pos.push(x, y, z);
      map.set(key, idx);
    }
    indices[i] = idx;
  }
  return { positions: Float64Array.from(pos), indices };
}

export function bboxOfSoup(s: TriSoup): BBox {
  const p = s.positions;
  let minx = Infinity, miny = Infinity, minz = Infinity;
  let maxx = -Infinity, maxy = -Infinity, maxz = -Infinity;
  for (let i = 0; i < p.length; i += 3) {
    const x = p[i]!, y = p[i + 1]!, z = p[i + 2]!;
    if (x < minx) minx = x; if (y < miny) miny = y; if (z < minz) minz = z;
    if (x > maxx) maxx = x; if (y > maxy) maxy = y; if (z > maxz) maxz = z;
  }
  return {
    min: [minx, miny, minz],
    max: [maxx, maxy, maxz],
    diagonal: Math.hypot(maxx - minx, maxy - miny, maxz - minz),
  };
}
