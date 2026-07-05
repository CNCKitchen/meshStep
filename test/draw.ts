// SPDX-License-Identifier: AGPL-3.0-only
// Library version of the software renderer in render.ts, importable by the gapcheck
// worker: orthographic z-buffered flat shading with arbitrary colored layers (body,
// flagged faces, missing-reference overlay) plus overlay edge segments drawn on top
// ignoring depth (a leak must be visible even inside a crevice). Supports framing a
// sub-region so failure pictures can zoom onto the defect. PNG via node:zlib.
import { deflateSync } from "node:zlib";
import { writeFileSync } from "node:fs";

// ---------- PNG ----------

const CRC_TABLE = new Uint32Array(256);
for (let n = 0; n < 256; n++) {
  let c = n;
  for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
  CRC_TABLE[n] = c >>> 0;
}
const crc32 = (buf: Uint8Array): number => {
  let c = 0xffffffff;
  for (let i = 0; i < buf.length; i++) c = CRC_TABLE[(c ^ buf[i]!) & 0xff]! ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
};
const pngChunk = (type: string, data: Uint8Array): Buffer => {
  const out = Buffer.alloc(12 + data.length);
  out.writeUInt32BE(data.length, 0);
  out.write(type, 4, "ascii");
  out.set(data, 8);
  out.writeUInt32BE(crc32(out.subarray(4, 8 + data.length)), 8 + data.length);
  return out;
};

export function encodePNG(width: number, height: number, rgb: Uint8Array): Buffer {
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width, 0);
  ihdr.writeUInt32BE(height, 4);
  ihdr[8] = 8; ihdr[9] = 2; // 8-bit truecolor
  const raw = Buffer.alloc(height * (1 + width * 3));
  for (let y = 0; y < height; y++) {
    raw[y * (1 + width * 3)] = 0; // filter: none
    raw.set(rgb.subarray(y * width * 3, (y + 1) * width * 3), y * (1 + width * 3) + 1);
  }
  return Buffer.concat([
    Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]),
    pngChunk("IHDR", ihdr),
    pngChunk("IDAT", deflateSync(raw, { level: 6 })),
    pngChunk("IEND", new Uint8Array(0)),
  ]);
}

// ---------- rasterizer ----------

export interface Layer { tris: Float64Array; rgb: [number, number, number] }
export interface EdgeSet { segs: Float64Array; rgb: [number, number, number]; thick?: number }
export interface Box3 { min: [number, number, number]; max: [number, number, number] }

export interface ViewOptions {
  width: number;
  height: number;
  layers: Layer[];
  edges?: EdgeSet[];
  /** World-space direction the camera looks along. */
  viewDir: [number, number, number];
  /** World-space region to frame; defaults to the bbox of all layers. */
  fit?: Box3;
}

export const bboxOf = (arrs: Float64Array[]): Box3 => {
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (const T of arrs) {
    for (let i = 0; i < T.length; i += 3) {
      for (let k = 0; k < 3; k++) {
        const v = T[i + k]!;
        if (v < min[k]!) min[k] = v;
        if (v > max[k]!) max[k] = v;
      }
    }
  }
  return { min, max };
};

export const growBox = (b: Box3, margin: number, minSize: number): Box3 => {
  const min: Box3["min"] = [0, 0, 0], max: Box3["max"] = [0, 0, 0];
  for (let k = 0; k < 3; k++) {
    const size = b.max[k]! - b.min[k]!;
    const pad = Math.max(size * margin, (minSize - size) / 2);
    min[k] = b.min[k]! - pad;
    max[k] = b.max[k]! + pad;
  }
  return { min, max };
};

const LIGHT = [0.3, 0.5, 0.8].map((v, _, l) => v / Math.hypot(...l)) as [number, number, number];

export function renderView(o: ViewOptions): Uint8Array {
  const { width: W, height: H } = o;
  const img = new Uint8Array(W * H * 3);
  for (let i = 0; i < img.length; i += 3) { img[i] = 18; img[i + 1] = 22; img[i + 2] = 30; } // dark bg
  const zbuf = new Float32Array(W * H).fill(Infinity);

  // camera basis: look along viewDir, world-Z as up unless viewing along Z
  let [vx, vy, vz] = o.viewDir;
  const vl = Math.hypot(vx, vy, vz) || 1;
  vx /= vl; vy /= vl; vz /= vl;
  const upW: [number, number, number] = Math.abs(vz) > 0.9 ? [0, 1, 0] : [0, 0, 1];
  let rx = upW[1] * vz - upW[2] * vy, ry = upW[2] * vx - upW[0] * vz, rz = upW[0] * vy - upW[1] * vx;
  const rl = Math.hypot(rx, ry, rz) || 1;
  rx /= rl; ry /= rl; rz /= rl;
  const ux = vy * rz - vz * ry, uy = vz * rx - vx * rz, uz = vx * ry - vy * rx;

  // frame the fit region
  const fit = o.fit ?? bboxOf(o.layers.map((l) => l.tris));
  let uMin = Infinity, uMax = -Infinity, wMin = Infinity, wMax = -Infinity;
  for (let c = 0; c < 8; c++) {
    const x = (c & 1 ? fit.max : fit.min)[0]!, y = (c & 2 ? fit.max : fit.min)[1]!, z = (c & 4 ? fit.max : fit.min)[2]!;
    const u = x * rx + y * ry + z * rz, w = x * ux + y * uy + z * uz;
    if (u < uMin) uMin = u; if (u > uMax) uMax = u;
    if (w < wMin) wMin = w; if (w > wMax) wMax = w;
  }
  const scale = Math.min((W * 0.92) / Math.max(uMax - uMin, 1e-12), (H * 0.92) / Math.max(wMax - wMin, 1e-12));
  const uc = (uMin + uMax) / 2, wc = (wMin + wMax) / 2;
  const px = (x: number, y: number, z: number): [number, number, number] => [
    (x * rx + y * ry + z * rz - uc) * scale + W / 2,
    H / 2 - (x * ux + y * uy + z * uz - wc) * scale,
    x * vx + y * vy + z * vz,
  ];

  for (const layer of o.layers) {
    const T = layer.tris;
    for (let t = 0; t < T.length; t += 9) {
      const [x0, y0, d0] = px(T[t]!, T[t + 1]!, T[t + 2]!);
      const [x1, y1, d1] = px(T[t + 3]!, T[t + 4]!, T[t + 5]!);
      const [x2, y2, d2] = px(T[t + 6]!, T[t + 7]!, T[t + 8]!);
      const area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
      if (Math.abs(area) < 1e-9) continue;
      const minX = Math.max(0, Math.floor(Math.min(x0, x1, x2)));
      const maxX = Math.min(W - 1, Math.ceil(Math.max(x0, x1, x2)));
      const minY = Math.max(0, Math.floor(Math.min(y0, y1, y2)));
      const maxY = Math.min(H - 1, Math.ceil(Math.max(y0, y1, y2)));
      if (minX > maxX || minY > maxY) continue;
      // world normal, two-sided Lambert toward the light
      const e1x = T[t + 3]! - T[t]!, e1y = T[t + 4]! - T[t + 1]!, e1z = T[t + 5]! - T[t + 2]!;
      const e2x = T[t + 6]! - T[t]!, e2y = T[t + 7]! - T[t + 1]!, e2z = T[t + 8]! - T[t + 2]!;
      const nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
      const nl = Math.hypot(nx, ny, nz) || 1;
      const lum = 0.3 + 0.7 * Math.abs((nx * LIGHT[0] + ny * LIGHT[1] + nz * LIGHT[2]) / nl);
      const cr = layer.rgb[0] * lum, cg = layer.rgb[1] * lum, cb = layer.rgb[2] * lum;
      const inv = 1 / area;
      for (let y = minY; y <= maxY; y++) {
        for (let x = minX; x <= maxX; x++) {
          const w0 = ((x1 - x) * (y2 - y) - (x2 - x) * (y1 - y)) * inv;
          const w1 = ((x2 - x) * (y0 - y) - (x0 - x) * (y2 - y)) * inv;
          const w2 = 1 - w0 - w1;
          if (w0 < -1e-3 || w1 < -1e-3 || w2 < -1e-3) continue;
          const d = w0 * d0 + w1 * d1 + w2 * d2;
          const i = y * W + x;
          if (d >= zbuf[i]!) continue;
          zbuf[i] = d;
          img[i * 3] = cr; img[i * 3 + 1] = cg; img[i * 3 + 2] = cb;
        }
      }
    }
  }

  // overlay edges, always on top
  for (const es of o.edges ?? []) {
    const thick = es.thick ?? 1;
    const S = es.segs;
    for (let s = 0; s < S.length; s += 6) {
      let [ax, ay] = px(S[s]!, S[s + 1]!, S[s + 2]!);
      let [bx, by] = px(S[s + 3]!, S[s + 4]!, S[s + 5]!);
      // Liang–Barsky clip to the viewport (zoom views put most segments far outside)
      const dx = bx - ax, dy = by - ay;
      let t0 = 0, t1 = 1, ok = true;
      for (const [p, q] of [[-dx, ax], [dx, W - 1 - ax], [-dy, ay], [dy, H - 1 - ay]] as const) {
        if (p === 0) { if (q < 0) { ok = false; break; } continue; }
        const r = q / p;
        if (p < 0) { if (r > t1) { ok = false; break; } if (r > t0) t0 = r; }
        else { if (r < t0) { ok = false; break; } if (r < t1) t1 = r; }
      }
      if (!ok) continue;
      bx = ax + t1 * dx; by = ay + t1 * dy;
      ax = ax + t0 * dx; ay = ay + t0 * dy;
      const steps = Math.max(1, Math.ceil(Math.max(Math.abs(bx - ax), Math.abs(by - ay))));
      for (let i = 0; i <= steps; i++) {
        const x = Math.round(ax + ((bx - ax) * i) / steps);
        const y = Math.round(ay + ((by - ay) * i) / steps);
        for (let oy = -thick; oy <= thick; oy++) {
          for (let ox = -thick; ox <= thick; ox++) {
            const X = x + ox, Y = y + oy;
            if (X < 0 || X >= W || Y < 0 || Y >= H) continue;
            const j = (Y * W + X) * 3;
            img[j] = es.rgb[0]; img[j + 1] = es.rgb[1]; img[j + 2] = es.rgb[2];
          }
        }
      }
    }
  }
  return img;
}

export function renderPNGFile(path: string, o: ViewOptions): void {
  writeFileSync(path, encodePNG(o.width, o.height, renderView(o)));
}
