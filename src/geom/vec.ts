// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — minimal 3D vector math (immutable tuple style).

export type Vec3 = readonly [number, number, number];

export const v = (x: number, y: number, z: number): Vec3 => [x, y, z];
export const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
export const mul = (a: Vec3, b: Vec3): Vec3 => [a[0] * b[0], a[1] * b[1], a[2] * b[2]];
export const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
/**
 * Math.hypot for exactly two / three arguments, as plain arithmetic. V8's built-in copies its
 * arguments into a freshly allocated array on every call, which makes it several times slower
 * than the arithmetic it performs; these do the same steps (scale by the largest magnitude,
 * compensated sum of the squares, square root, scale back) in the same order, so on V8 they
 * return the identical double for every input (checked against Math.hypot on millions of random
 * and special values), and they return that same value on every other engine too.
 */
export function hypot2(a: number, b: number): number {
  if (a !== a || b !== b) return Math.abs(a) === Infinity || Math.abs(b) === Infinity ? Infinity : NaN;
  a = Math.abs(a); b = Math.abs(b);
  const max = a > b ? a : b;
  if (max === Infinity) return Infinity;
  if (max === 0) return 0;
  let sum = 0, comp = 0, n: number, summand: number, pre: number;
  n = a / max; summand = n * n - comp; pre = sum + summand; comp = (pre - sum) - summand; sum = pre;
  n = b / max; summand = n * n - comp; pre = sum + summand;
  return Math.sqrt(pre) * max;
}
export function hypot3(a: number, b: number, c: number): number {
  if (a !== a || b !== b || c !== c) {
    return Math.abs(a) === Infinity || Math.abs(b) === Infinity || Math.abs(c) === Infinity ? Infinity : NaN;
  }
  a = Math.abs(a); b = Math.abs(b); c = Math.abs(c);
  let max = a > b ? a : b;
  if (c > max) max = c;
  if (max === Infinity) return Infinity;
  if (max === 0) return 0;
  let sum = 0, comp = 0, n: number, summand: number, pre: number;
  n = a / max; summand = n * n - comp; pre = sum + summand; comp = (pre - sum) - summand; sum = pre;
  n = b / max; summand = n * n - comp; pre = sum + summand; comp = (pre - sum) - summand; sum = pre;
  n = c / max; summand = n * n - comp; pre = sum + summand;
  return Math.sqrt(pre) * max;
}

export const len = (a: Vec3): number => hypot3(a[0], a[1], a[2]);
export const len2 = (a: Vec3): number => a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
export const dist = (a: Vec3, b: Vec3): number => hypot3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
export const lerp = (a: Vec3, b: Vec3, t: number): Vec3 => [
  a[0] + (b[0] - a[0]) * t,
  a[1] + (b[1] - a[1]) * t,
  a[2] + (b[2] - a[2]) * t,
];

export const normalize = (a: Vec3): Vec3 => {
  const l = len(a);
  return l > 0 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0];
};

/** Angle between two vectors in radians (numerically stable). */
export const angleBetween = (a: Vec3, b: Vec3): number => {
  const an = normalize(a);
  const bn = normalize(b);
  const c = Math.max(-1, Math.min(1, dot(an, bn)));
  return Math.acos(c);
};
