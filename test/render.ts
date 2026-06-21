// SPDX-License-Identifier: AGPL-3.0-only
// Headless software renderer for visual mesh inspection: shades triangles (z-buffered Lambert) and
// overdraws OPEN edges in red / NON-MANIFOLD edges in magenta, so problem areas are visible at a
// glance. Pure TS + Node's zlib for PNG (no runtime deps). Usage:
//   node --experimental-strip-types test/render.ts <stepFileUnderSourceModels> [az] [el] [out.png]
import { readFileSync, writeFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { deflateSync } from "node:zlib";
import { importStep } from "../src/index.ts";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const file = process.argv[2] ?? "OpenVessel.step";
const az = (Number(process.argv[3] ?? 35)) * Math.PI / 180;
const el = (Number(process.argv[4] ?? 25)) * Math.PI / 180;
const outName = process.argv[5] ?? `render_${file.replace(/\.[^.]+$/, "")}.png`;
const zoom = Number(process.argv[6] ?? 1);
const target = process.argv[7] ? process.argv[7].split(",").map(Number) : null; // "x,y,z" to center+zoom on
const W = 1100, H = 900;

const src = readFileSync(join(root, "sourceModels", file), "utf8");
const m = importStep(src, { remesh: false }).mesh;
const P = m.positions, I = m.indices, nt = I.length / 3;

// Edge incidence -> classify open (1) / non-manifold (>2).
const inc = new Map<string, number>();
for (let t = 0; t < nt; t++) for (let e = 0; e < 3; e++) { const a = I[t * 3 + e]!, b = I[t * 3 + (e + 1) % 3]!; const k = a < b ? `${a}_${b}` : `${b}_${a}`; inc.set(k, (inc.get(k) ?? 0) + 1); }

// Camera: center on bbox, orthographic with azimuth/elevation rotation, scale to fit.
let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
for (let v = 0; v < P.length; v += 3) for (let k = 0; k < 3; k++) { if (P[v + k]! < lo[k]!) lo[k] = P[v + k]!; if (P[v + k]! > hi[k]!) hi[k] = P[v + k]!; }
const c = target ?? [(lo[0]! + hi[0]!) / 2, (lo[1]! + hi[1]!) / 2, (lo[2]! + hi[2]!) / 2];
const diag = Math.hypot(hi[0]! - lo[0]!, hi[1]! - lo[1]!, hi[2]! - lo[2]!);
const ca = Math.cos(az), sa = Math.sin(az), ce = Math.cos(el), se = Math.sin(el);
// rotate about Z by az, then about X by el; camera looks down -Z' after.
function viewXform(x: number, y: number, z: number): [number, number, number] {
  x -= c[0]!; y -= c[1]!; z -= c[2]!;
  const x1 = ca * x - sa * y, y1 = sa * x + ca * y, z1 = z;       // yaw
  const y2 = ce * y1 - se * z1, z2 = se * y1 + ce * z1;            // pitch
  return [x1, y2, z2];
}
const scale = 0.8 * Math.min(W, H) / diag * zoom;
const proj = (vx: number, vy: number): [number, number] => [W / 2 + vx * scale, H / 2 - vy * scale];

const col = new Uint8Array(W * H * 3);
for (let i = 0; i < col.length; i += 3) { col[i] = 18; col[i + 1] = 22; col[i + 2] = 30; } // dark bg
const zbuf = new Float64Array(W * H).fill(-Infinity);
const light = (() => { const l = [0.3, 0.5, 0.8]; const n = Math.hypot(l[0]!, l[1]!, l[2]!); return [l[0]! / n, l[1]! / n, l[2]! / n]; })();

function shadeTri(ax: number, ay: number, az_: number, bx: number, by: number, bz: number, cx: number, cy: number, cz: number): void {
  // screen coords
  const [sax, say] = proj(ax, ay), [sbx, sby] = proj(bx, by), [scx, scy] = proj(cx, cy);
  // view-space normal for shading (z is depth toward camera = +z2)
  const ux = bx - ax, uy = by - ay, uz = bz - az_, vx = cx - ax, vy = cy - ay, vz = cz - az_;
  let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const nl = Math.hypot(nx, ny, nz) || 1; nx /= nl; ny /= nl; nz /= nl;
  if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; } // face the camera for two-sided shading
  const lam = Math.max(0, nx * light[0]! + ny * light[1]! + nz * light[2]!);
  const shade = 0.25 + 0.7 * lam;
  const r = Math.min(255, 90 * shade + 30), g = Math.min(255, 110 * shade + 35), b = Math.min(255, 140 * shade + 45);
  // rasterize (bbox + barycentric), z = depth (bz/etc are view z2)
  const minx = Math.max(0, Math.floor(Math.min(sax, sbx, scx))), maxx = Math.min(W - 1, Math.ceil(Math.max(sax, sbx, scx)));
  const miny = Math.max(0, Math.floor(Math.min(say, sby, scy))), maxy = Math.min(H - 1, Math.ceil(Math.max(say, sby, scy)));
  const area = (sbx - sax) * (scy - say) - (sby - say) * (scx - sax);
  if (Math.abs(area) < 1e-9) return;
  for (let y = miny; y <= maxy; y++) for (let x = minx; x <= maxx; x++) {
    const w0 = ((sbx - x) * (scy - y) - (sby - y) * (scx - x)) / area;
    const w1 = ((scx - x) * (say - y) - (scy - y) * (sax - x)) / area;
    const w2 = 1 - w0 - w1;
    if (w0 < -0.001 || w1 < -0.001 || w2 < -0.001) continue;
    const depth = w0 * az_ + w1 * bz + w2 * cz;
    const idx = y * W + x;
    if (depth <= zbuf[idx]!) continue;
    zbuf[idx] = depth;
    col[idx * 3] = r; col[idx * 3 + 1] = g; col[idx * 3 + 2] = b;
  }
}

// Pass 1: shade all triangles.
const vv: [number, number, number][] = [];
for (let v = 0; v < P.length / 3; v++) vv.push(viewXform(P[v * 3]!, P[v * 3 + 1]!, P[v * 3 + 2]!));
for (let t = 0; t < nt; t++) {
  const a = vv[I[t * 3]!]!, b = vv[I[t * 3 + 1]!]!, cc = vv[I[t * 3 + 2]!]!;
  shadeTri(a[0], a[1], a[2], b[0], b[1], b[2], cc[0], cc[1], cc[2]);
}

// Pass 2: overdraw problem edges (red=open, magenta=non-manifold), ignoring z so they always show.
function line(x0: number, y0: number, x1: number, y1: number, r: number, g: number, b: number): void {
  x0 = Math.round(x0); y0 = Math.round(y0); x1 = Math.round(x1); y1 = Math.round(y1);
  const dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  let err = dx + dy;
  for (;;) {
    if (x0 >= 0 && x0 < W && y0 >= 0 && y0 < H) {
      for (let oy = -1; oy <= 1; oy++) for (let ox = -1; ox <= 1; ox++) { const xx = x0 + ox, yy = y0 + oy; if (xx >= 0 && xx < W && yy >= 0 && yy < H) { const i = (yy * W + xx) * 3; col[i] = r; col[i + 1] = g; col[i + 2] = b; } }
    }
    if (x0 === x1 && y0 === y1) break;
    const e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}
let nOpen = 0, nNm = 0;
for (const [k, n] of inc) {
  if (n === 2) continue;
  const [a, b] = k.split("_").map(Number) as [number, number];
  const [sax, say] = proj(vv[a]![0], vv[a]![1]), [sbx, sby] = proj(vv[b]![0], vv[b]![1]);
  if (n === 1) { line(sax, say, sbx, sby, 255, 40, 40); nOpen++; } // open = red
  else { line(sax, say, sbx, sby, 255, 0, 255); nNm++; }          // non-manifold = magenta
}

// ---- PNG encode (truecolor, filter 0) ----
const crcTable = (() => { const t = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; } return t; })();
function crc32(buf: Buffer): number { let c = 0xffffffff; for (let i = 0; i < buf.length; i++) c = crcTable[(c ^ buf[i]!) & 0xff]! ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; }
function chunk(type: string, data: Buffer): Buffer {
  const len = Buffer.alloc(4); len.writeUInt32BE(data.length, 0);
  const t = Buffer.from(type, "ascii");
  const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(Buffer.concat([t, data])), 0);
  return Buffer.concat([len, t, data, crc]);
}
const raw = Buffer.alloc(H * (1 + W * 3));
for (let y = 0; y < H; y++) { raw[y * (1 + W * 3)] = 0; for (let x = 0; x < W * 3; x++) raw[y * (1 + W * 3) + 1 + x] = col[y * W * 3 + x]!; }
const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(W, 0); ihdr.writeUInt32BE(H, 4); ihdr[8] = 8; ihdr[9] = 2; // 8-bit truecolor
const png = Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk("IHDR", ihdr), chunk("IDAT", deflateSync(raw)), chunk("IEND", Buffer.alloc(0))]);
const outPath = join(root, "out", outName);
writeFileSync(outPath, png);
console.log(`${file}: tris=${nt} open=${nOpen} nm=${nNm} -> out/${outName}`);
