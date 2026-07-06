// SPDX-License-Identifier: AGPL-3.0-only
// meshStep — ear-clipping triangulator for a simple polygon WITH HOLES, on a plane. This is the
// watertight fallback for PLANE faces whose constrained-Delaunay parity flood folds (a multi-loop
// tray/embossed-text face where the region extraction misclassifies in/out and over-covers a
// boundary segment). Unlike the CDT it never floods a region: each hole is bridged into the outer
// boundary via a mutually-visible cut, giving one simple polygon that is ear-clipped directly — so
// every boundary edge is covered exactly once and every diagonal exactly twice, watertight by
// construction for any loop count. Valid ONLY where (u,v)->3D is affine (a plane); on a curved
// surface the (u,v) triangles would be chords/folds.
//
// Algorithm: Mapbox earcut (github.com/mapbox/earcut), reimplemented — hole elimination by
// left-to-right bridging, O(n^2) ear removal (no z-order hashing; the failing faces are only
// hundreds of vertices), with local-intersection cure and polygon-split passes for self-touching
// input. Ports faithfully to keep its well-tested robustness on real CAD boundaries.

interface ENode {
  i: number; // vertex index into the caller's flat coordinate array
  x: number;
  y: number;
  prev: ENode;
  next: ENode;
  steiner: boolean;
}

function createNode(i: number, x: number, y: number): ENode {
  // prev/next are self-linked until inserted; the non-null assertion keeps the interface tight.
  const n = { i, x, y, steiner: false } as unknown as ENode;
  n.prev = n;
  n.next = n;
  return n;
}

/** Triangulate a polygon with holes. `data` is a flat [x0,y0,x1,y1,...] vertex list; `holeIndices`
 * gives the start VERTEX index of each hole (outer ring is [0, holeIndices[0])). Returns triangle
 * vertex-index triples flattened. Empty on failure. */
export function earcut(data: number[], holeIndices: number[] | null): number[] {
  const hasHoles = holeIndices && holeIndices.length > 0;
  const outerLen = hasHoles ? holeIndices![0]! * 2 : data.length;
  let outerNode = linkedList(data, 0, outerLen, true);
  const triangles: number[] = [];
  if (!outerNode || outerNode.next === outerNode.prev) return triangles;
  if (hasHoles) outerNode = eliminateHoles(data, holeIndices!, outerNode);
  earcutLinked(outerNode, triangles, 0);
  return triangles;
}

/** Build a circular doubly-linked list from a ring of the flat data, forced to the requested winding
 * (outer clockwise, holes counter-clockwise in this coordinate convention). */
function linkedList(data: number[], start: number, end: number, clockwise: boolean): ENode | null {
  let last: ENode | null = null;
  if (clockwise === signedArea(data, start, end) > 0) {
    for (let i = start; i < end; i += 2) last = insertNode((i / 2) | 0, data[i]!, data[i + 1]!, last);
  } else {
    for (let i = end - 2; i >= start; i -= 2) last = insertNode((i / 2) | 0, data[i]!, data[i + 1]!, last);
  }
  if (last && equals(last, last.next)) {
    removeNode(last);
    last = last.next;
  }
  return last;
}

/** Remove collinear or duplicate points (they cannot be ears and only stall the clip). */
function filterPoints(start: ENode | null, end?: ENode): ENode | null {
  if (!start) return start;
  if (!end) end = start;
  let p = start;
  let again: boolean;
  do {
    again = false;
    if (!p.steiner && (equals(p, p.next) || area(p.prev, p, p.next) === 0)) {
      removeNode(p);
      p = end = p.prev;
      if (p === p.next) break;
      again = true;
    } else {
      p = p.next;
    }
  } while (again || p !== end);
  return end;
}

/** Main ear-slicing loop. `pass` escalates the robustness fallbacks when no simple ear is found. */
function earcutLinked(earStart: ENode | null, triangles: number[], pass: number): void {
  let ear = earStart;
  if (!ear) return;
  let stop = ear;
  let prev: ENode, next: ENode;
  while (ear.prev !== ear.next) {
    prev = ear.prev;
    next = ear.next;
    if (isEar(ear)) {
      triangles.push(prev.i, ear.i, next.i);
      removeNode(ear);
      ear = next.next;
      stop = next.next;
      continue;
    }
    ear = next;
    if (ear === stop) {
      // No ear found in a full loop — recover from bad input by escalating.
      if (!pass) {
        earcutLinked(filterPoints(ear), triangles, 1);
      } else if (pass === 1) {
        const cured = cureLocalIntersections(filterPoints(ear)!, triangles);
        earcutLinked(cured, triangles, 2);
      } else if (pass === 2) {
        splitEarcut(ear, triangles);
      }
      break;
    }
  }
}

/** Is the vertex `ear` a valid ear (convex, and no other vertex inside its triangle)? */
function isEar(ear: ENode): boolean {
  const a = ear.prev, b = ear, c = ear.next;
  if (area(a, b, c) >= 0) return false; // reflex, can't be an ear
  const ax = a.x, bx = b.x, cx = c.x, ay = a.y, by = b.y, cy = c.y;
  const x0 = Math.min(ax, bx, cx), y0 = Math.min(ay, by, cy);
  const x1 = Math.max(ax, bx, cx), y1 = Math.max(ay, by, cy);
  let p = c.next;
  while (p !== a) {
    if (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1 &&
        pointInTriangle(ax, ay, bx, by, cx, cy, p.x, p.y) &&
        area(p.prev, p, p.next) >= 0) return false;
    p = p.next;
  }
  return true;
}

/** Go through all polygon nodes and cure small local self-intersections by clipping the offending
 * short diagonal (a robustness pass for slightly-tangled boundaries). */
function cureLocalIntersections(start: ENode, triangles: number[]): ENode | null {
  let p = start;
  do {
    const a = p.prev, b = p.next.next;
    if (!equals(a, b) && intersects(a, p, p.next, b) && locallyInside(a, b) && locallyInside(b, a)) {
      triangles.push(a.i, p.i, b.i);
      removeNode(p);
      removeNode(p.next);
      p = start = b;
    }
    p = p.next;
  } while (p !== start);
  return filterPoints(p);
}

/** Try splitting the polygon into two on a valid interior diagonal, then triangulate each half —
 * the last resort for a polygon with no findable ears. */
function splitEarcut(start: ENode, triangles: number[]): void {
  let a = start;
  do {
    let b = a.next.next;
    while (b !== a.prev) {
      if (a.i !== b.i && isValidDiagonal(a, b)) {
        let c: ENode | null = splitPolygon(a, b);
        const aa = filterPoints(a, a.next);
        c = filterPoints(c, c.next);
        earcutLinked(aa, triangles, 0);
        earcutLinked(c, triangles, 0);
        return;
      }
      b = b.next;
    }
    a = a.next;
  } while (a !== start);
}

/** Link every hole into the outer loop, left to right, via a mutually-visible bridge. */
function eliminateHoles(data: number[], holeIndices: number[], outerNode: ENode): ENode {
  const queue: ENode[] = [];
  for (let i = 0; i < holeIndices.length; i++) {
    const start = holeIndices[i]! * 2;
    const end = i < holeIndices.length - 1 ? holeIndices[i + 1]! * 2 : data.length;
    const list = linkedList(data, start, end, false);
    if (list) {
      if (list === list.next) list.steiner = true;
      queue.push(getLeftmost(list));
    }
  }
  queue.sort((p, q) => p.x - q.x);
  for (const h of queue) outerNode = eliminateHole(h, outerNode);
  return outerNode;
}

function eliminateHole(hole: ENode, outerNode: ENode): ENode {
  const bridge = findHoleBridge(hole, outerNode);
  if (!bridge) return outerNode;
  const bridgeReverse = splitPolygon(bridge, hole);
  filterPoints(bridgeReverse, bridgeReverse.next);
  return filterPoints(bridge, bridge.next)!;
}

/** David Eberly's algorithm for finding a bridge between a hole and the outer polygon: cast a ray
 * from the hole's leftmost point to the left, take the outer edge it hits, then refine to the
 * visible vertex of minimum angle. */
function findHoleBridge(hole: ENode, outerNode: ENode): ENode | null {
  let p = outerNode;
  const hx = hole.x, hy = hole.y;
  let qx = -Infinity;
  let m: ENode | null = null;
  do {
    if (hy <= p.y && hy >= p.next.y && p.next.y !== p.y) {
      const x = p.x + ((hy - p.y) * (p.next.x - p.x)) / (p.next.y - p.y);
      if (x <= hx && x > qx) {
        qx = x;
        m = p.x < p.next.x ? p : p.next;
        if (x === hx) return m; // hole touches the outer directly at a vertex
      }
    }
    p = p.next;
  } while (p !== outerNode);

  if (!m) return null;

  // Look for points strictly inside the triangle (hole point, edge intersection, endpoint); of any
  // found, pick the one of minimum angle with the ray (and, on ties, the more clockwise), so the
  // bridge does not cross the outer boundary.
  const stop = m;
  const mx = m.x, my = m.y;
  let tanMin = Infinity;
  p = m;
  do {
    if (hx >= p.x && p.x >= mx && hx !== p.x &&
        pointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p.x, p.y)) {
      const tan = Math.abs(hy - p.y) / (hx - p.x);
      if (locallyInside(p, hole) &&
          (tan < tanMin || (tan === tanMin && (p.x > m!.x || (p.x === m!.x && sectorContainsSector(m!, p)))))) {
        m = p;
        tanMin = tan;
      }
    }
    p = p.next;
  } while (p !== stop);

  return m;
}

/** Whether sector [m.prev,m,m.next] contains sector [p.prev,p,p.next] — a bridge tie-break. */
function sectorContainsSector(m: ENode, p: ENode): boolean {
  return area(m.prev, m, p.prev) < 0 && area(p.next, m, m.next) < 0;
}

function getLeftmost(start: ENode): ENode {
  let p = start, leftmost = start;
  do {
    if (p.x < leftmost.x || (p.x === leftmost.x && p.y < leftmost.y)) leftmost = p;
    p = p.next;
  } while (p !== start);
  return leftmost;
}

function pointInTriangle(
  ax: number, ay: number, bx: number, by: number, cx: number, cy: number, px: number, py: number,
): boolean {
  return (cx - px) * (ay - py) - (ax - px) * (cy - py) >= 0 &&
         (ax - px) * (by - py) - (bx - px) * (ay - py) >= 0 &&
         (bx - px) * (cy - py) - (cx - px) * (by - py) >= 0;
}

/** A diagonal a->b is valid if it stays inside the polygon and crosses no edge. */
function isValidDiagonal(a: ENode, b: ENode): boolean {
  return a.next.i !== b.i && a.prev.i !== b.i && !intersectsPolygon(a, b) &&
    ((locallyInside(a, b) && locallyInside(b, a) && middleInside(a, b) &&
      (area(a.prev, a, b.prev) !== 0 || area(a, b.prev, b) !== 0)) ||
     (equals(a, b) && area(a.prev, a, a.next) > 0 && area(b.prev, b, b.next) > 0));
}

/** Signed area of triangle p-q-r (negative for CCW in this convention). */
function area(p: ENode, q: ENode, r: ENode): number {
  return (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
}

function equals(p1: ENode, p2: ENode): boolean {
  return p1.x === p2.x && p1.y === p2.y;
}

/** Do segments p1q1 and p2q2 intersect (including collinear overlap)? */
function intersects(p1: ENode, q1: ENode, p2: ENode, q2: ENode): boolean {
  const o1 = sign(area(p1, q1, p2));
  const o2 = sign(area(p1, q1, q2));
  const o3 = sign(area(p2, q2, p1));
  const o4 = sign(area(p2, q2, q1));
  if (o1 !== o2 && o3 !== o4) return true; // general case
  if (o1 === 0 && onSegment(p1, p2, q1)) return true;
  if (o2 === 0 && onSegment(p1, q2, q1)) return true;
  if (o3 === 0 && onSegment(p2, p1, q2)) return true;
  if (o4 === 0 && onSegment(p2, q1, q2)) return true;
  return false;
}

/** Does the diagonal a->b cross any polygon edge? */
function intersectsPolygon(a: ENode, b: ENode): boolean {
  let p = a;
  do {
    if (p.i !== a.i && p.next.i !== a.i && p.i !== b.i && p.next.i !== b.i && intersects(p, p.next, a, b)) return true;
    p = p.next;
  } while (p !== a);
  return false;
}

/** Is the diagonal a->b locally inside the polygon at a? */
function locallyInside(a: ENode, b: ENode): boolean {
  return area(a.prev, a, a.next) < 0
    ? area(a, b, a.next) >= 0 && area(a, a.prev, b) >= 0
    : area(a, b, a.prev) < 0 || area(a, a.next, b) < 0;
}

/** Is the midpoint of a->b inside the polygon (even-odd ray cast)? */
function middleInside(a: ENode, b: ENode): boolean {
  let p = a, inside = false;
  const px = (a.x + b.x) / 2, py = (a.y + b.y) / 2;
  do {
    if (p.y > py !== p.next.y > py && p.next.y !== p.y &&
        px < ((p.next.x - p.x) * (py - p.y)) / (p.next.y - p.y) + p.x) inside = !inside;
    p = p.next;
  } while (p !== a);
  return inside;
}

function onSegment(p: ENode, q: ENode, r: ENode): boolean {
  return q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) &&
         q.y <= Math.max(p.y, r.y) && q.y >= Math.min(p.y, r.y);
}

function sign(n: number): number {
  return n > 0 ? 1 : n < 0 ? -1 : 0;
}

/** Split a polygon into two by a bridge a<->b; returns the second polygon's start node. Also used to
 * splice a hole into the outer loop. */
function splitPolygon(a: ENode, b: ENode): ENode {
  const a2 = createNode(a.i, a.x, a.y);
  const b2 = createNode(b.i, b.x, b.y);
  const an = a.next, bp = b.prev;
  a.next = b;
  b.prev = a;
  a2.next = an;
  an.prev = a2;
  b2.next = a2;
  a2.prev = b2;
  bp.next = b2;
  b2.prev = bp;
  return b2;
}

function insertNode(i: number, x: number, y: number, last: ENode | null): ENode {
  const p = createNode(i, x, y);
  if (!last) {
    p.prev = p;
    p.next = p;
  } else {
    p.next = last.next;
    p.prev = last;
    last.next.prev = p;
    last.next = p;
  }
  return p;
}

function removeNode(p: ENode): void {
  p.next.prev = p.prev;
  p.prev.next = p.next;
}

function signedArea(data: number[], start: number, end: number): number {
  let sum = 0;
  for (let i = start, j = end - 2; i < end; i += 2) {
    sum += (data[j]! - data[i]!) * (data[i + 1]! + data[j + 1]!);
    j = i;
  }
  return sum;
}
