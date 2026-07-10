// SPDX-License-Identifier: AGPL-3.0-only
import "./styles.css";
import * as THREE from "three";
import { readSTL, writeBinarySTL, parseStepHeader, autoTessellation, type PartNode } from "../../src/index.ts";
import { Viewer, BASE_COLOR } from "./viewer.ts";
import { ReferenceSurface, type DeviationResult } from "./deviation.ts";
import {
  buildIndexedGeometry,
  buildSoupGeometry,
  boundaryEdges,
  dropSolidSegments,
  featureEdges,
  splitByTriColor,
  triCount,
  diverging,
  type EdgeSet,
  type RawMesh,
} from "./mesh-utils.ts";
import type { ConvertResult, WorkerOut } from "./worker.ts";

// ---- DOM helpers ----
const $ = <T extends HTMLElement = HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`#${id} missing`);
  return el as T;
};

const stepFile = $<HTMLInputElement>("stepFile");
const stepName = $("stepName");
const refFile = $<HTMLInputElement>("refFile");
const refName = $("refName");
const convertBtn = $<HTMLButtonElement>("convertBtn");
const exportBtn = $<HTMLButtonElement>("exportBtn");
const statusEl = $("status");
const overlay = $("overlay");
const overlayText = $("overlayText");
const progressBar = $("progressBar");
const progressFill = $("progressFill");

const tColors = $<HTMLInputElement>("tColors");
const tWire = $<HTMLInputElement>("tWire");
const tEdges = $<HTMLInputElement>("tEdges");
const tFeature = $<HTMLInputElement>("tFeature");
const tTransparent = $<HTMLInputElement>("tTransparent");
const tEdgesOnly = $<HTMLInputElement>("tEdgesOnly");
const tSection = $<HTMLInputElement>("tSection");
const sectionRow = $("sectionRow");
const projBtn = $<HTMLButtonElement>("projBtn");
const tRef = $<HTMLInputElement>("tRef");
const tDev = $<HTMLInputElement>("tDev");
const devRange = $<HTMLInputElement>("devRange");
const rangeField = $("rangeField");
const autoRange = $<HTMLButtonElement>("autoRange");
const legend = $("legend");
const legNeg = $("legNeg");
const legPos = $("legPos");
const legMeta = $("legMeta");
const statsEl = $("stats");
const partsPanel = $("partsPanel");
const partsTree = $("partsTree");
const partsShowAll = $<HTMLButtonElement>("partsShowAll");
const partsHideAll = $<HTMLButtonElement>("partsHideAll");
const autoNote = $("autoNote");
const surfaceDeviationEl = $<HTMLInputElement>("surfaceDeviation");
const maxEdgeEl = $<HTMLInputElement>("maxEdge");

// info panel
const infoPanel = $("infoPanel");
const watertight = $("watertight");
const iSystem = $("iSystem");
const iSchema = $("iSchema");
const iUnits = $("iUnits");
const iDims = $("iDims");
const iBodies = $("iBodies");
const iFaces = $("iFaces");
const iTris = $("iTris");
const iDate = $("iDate");

// ---- store funnel (mirrors bumpmesh) ----
const storeCta = $("store-cta-wrapper");
if (localStorage.getItem("meshstep.cta.dismissed") === "1") storeCta.hidden = true;
$<HTMLButtonElement>("store-cta-dismiss").addEventListener("click", () => {
  storeCta.hidden = true;
  localStorage.setItem("meshstep.cta.dismissed", "1");
});

const sponsorOverlay = $("sponsor-overlay");
const sponsorDontShow = $<HTMLInputElement>("sponsor-dontshow");
// Shows once per page session on the first conversion — unless the user opted out for good.
let sponsorShown = localStorage.getItem("meshstep.sponsor.dismissed") === "1";
$<HTMLButtonElement>("sponsor-close").addEventListener("click", () => {
  sponsorOverlay.hidden = true;
  if (sponsorDontShow.checked) localStorage.setItem("meshstep.sponsor.dismissed", "1");
});
function maybeShowSponsor(): void {
  if (sponsorShown) return;
  sponsorShown = true;
  sponsorOverlay.hidden = false;
}

// ---- theme ----
// The inline head script already set data-theme (no flash); here we sync the 3D scene + button.
const themeToggle = $<HTMLButtonElement>("themeToggle");
function applyTheme(mode: "light" | "dark"): void {
  document.documentElement.dataset.theme = mode;
  viewer.setTheme(mode);
  themeToggle.textContent = mode === "light" ? "☾" : "☀";
  localStorage.setItem("meshstep.theme", mode);
}
themeToggle.addEventListener("click", () => {
  applyTheme(document.documentElement.dataset.theme === "light" ? "dark" : "light");
});

// ---- state ----
const viewer = new Viewer($("viewport"));
applyTheme(document.documentElement.dataset.theme === "light" ? "light" : "dark");
let stepText: string | null = null;
let stepBaseName = "mesh";
let worker: Worker | null = null;

let mesh: RawMesh | null = null;
let geo: THREE.BufferGeometry | null = null;
let openEdges = 0; // unexpected (defect) open edges — sheet-body boundaries excluded
let sheetBodies = 0;
let openSolidsSet = new Set<number>();
let solidOfTri: Uint32Array | null = null; // per-triangle body id (context-menu part info)
let defectEdges: EdgeSet | null = null; // open-edge segments, sheet bodies excluded

let reference: ReferenceSurface | null = null;
let dev: DeviationResult | null = null;
let manualRange: number | null = null; // null => auto

// ---- input: STEP ----
// Size-adaptive tessellation defaults: a fast worker pre-pass estimates the model's bbox
// diagonal on load and scales surface deviation / max edge to it — unless the user already
// touched those fields for this file (their numbers win).
let tessEdited = false;
surfaceDeviationEl.addEventListener("input", () => { tessEdited = true; autoNote.hidden = true; });
maxEdgeEl.addEventListener("input", () => { tessEdited = true; autoNote.hidden = true; });

function applyAutoDefaults(diag: number): void {
  if (tessEdited) return;
  const d = autoTessellation(diag);
  surfaceDeviationEl.value = String(d.surfaceDeviation);
  maxEdgeEl.value = String(d.maxEdge);
  autoNote.textContent = `auto for ~${fmtLen(diag)} mm model`;
  autoNote.hidden = false;
}

stepFile.addEventListener("change", async () => {
  const f = stepFile.files?.[0];
  if (!f) return;
  stepFile.value = ""; // so re-picking the same file fires change again (the name lives in #stepName)
  stepText = await f.text();
  stepBaseName = f.name.replace(/\.(step|stp)$/i, "") || "mesh";
  stepName.textContent = f.name;
  convertBtn.disabled = false;
  tessEdited = false;
  autoNote.hidden = true;
  setStatus("");
  showHeaderInfo(stepText);
  // Measure in the background; Convert terminates this worker, so a click before the estimate
  // lands simply keeps the current field values.
  worker?.terminate();
  worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
  worker.onmessage = (ev: MessageEvent<WorkerOut>) => onWorkerMessage(ev.data);
  worker.postMessage({ type: "measure", stepText });
});

// ---- input: reference STL ----
refFile.addEventListener("change", async () => {
  const f = refFile.files?.[0];
  if (!f) return;
  refFile.value = ""; // same-file re-pick must fire change (name lives in #refName)
  try {
    const buf = await f.arrayBuffer();
    const soup = readSTL(buf);
    const geoRef = buildSoupGeometry(soup.positions);
    reference?.dispose();
    reference = new ReferenceSurface(geoRef);
    refName.textContent = `${f.name}  ·  ${soup.triangleCount.toLocaleString()} tris`;
    viewer.setReference(geoRef);
    tRef.disabled = false;
    tDev.disabled = false;
    recomputeDeviation();
    refreshStats();
  } catch (err) {
    refName.textContent = "Failed to read STL";
    console.error(err);
  }
});

// ---- convert ----
convertBtn.addEventListener("click", () => {
  if (!stepText) return;
  const opts = {
    surfaceDeviation: num("surfaceDeviation", 0.01),
    normalDeviation: num("normalDeviation", 15),
    maxEdge: num("maxEdge", 1),
    remesh: false,
  };
  showOverlay("Starting…");
  maybeShowSponsor();
  convertBtn.disabled = true;

  worker?.terminate();
  worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
  worker.onmessage = (ev: MessageEvent<WorkerOut>) => onWorkerMessage(ev.data);
  worker.onerror = (e) => {
    hideOverlay();
    convertBtn.disabled = false;
    setStatus(`Worker error: ${e.message}`, true);
  };
  worker.postMessage({ type: "convert", stepText, opts });
});

function onWorkerMessage(msg: WorkerOut): void {
  if (msg.type === "size") {
    if (msg.estimate) applyAutoDefaults(msg.estimate.diag);
    return;
  }
  if (msg.type === "progress") {
    overlayText.textContent = msg.stage;
    // No fraction => the stage's duration is unknown; keep the last bar state rather than
    // flashing it away (parse has no bar yet, finalize arrives pinned at 100%).
    if (msg.fraction !== undefined) {
      progressBar.hidden = false;
      progressFill.style.width = `${(msg.fraction * 100).toFixed(1)}%`;
    }
    return;
  }
  if (msg.type === "error") {
    hideOverlay();
    convertBtn.disabled = false;
    setStatus(msg.message.split("\n")[0] ?? "Conversion failed", true);
    console.error(msg.message);
    return;
  }
  applyResult(msg);
}

function applyResult(res: ConvertResult): void {
  mesh = res.mesh;

  // STEP face colors: per-triangle palette index -> per-vertex color attribute, splitting welded
  // vertices on color borders so colors don't bleed across faces. The raw mesh stays untouched
  // for export and edge extraction; only the display geometry gets the extra border vertices.
  let faceColors: Float32Array | null = null;
  let displayMesh: RawMesh = mesh;
  if (res.colors) {
    const { palette, faceColor } = res.colors;
    const colorOfTri = new Int32Array(res.mesh.faceOfTri.length);
    for (let t = 0; t < colorOfTri.length; t++) colorOfTri[t] = faceColor.get(res.mesh.faceOfTri[t]!) ?? -1;
    // COLOUR_RGB values are sRGB; vertex colors feed the shader in the linear working space.
    const c = new THREE.Color();
    const linear = palette.map(([r, g, b]): [number, number, number] => {
      c.setRGB(r, g, b, THREE.SRGBColorSpace);
      return [c.r, c.g, c.b];
    });
    c.set(BASE_COLOR); // hex is converted like the material's base color, so unstyled faces match
    const split = splitByTriColor(mesh, colorOfTri, linear, [c.r, c.g, c.b]);
    displayMesh = split.mesh;
    faceColors = split.colors;
  }
  tColors.disabled = !faceColors;
  tSection.disabled = false; // there is a solid to cut now
  geo = buildIndexedGeometry(displayMesh);

  // Open-edge (defect) overlay: a sheet body's boundary is open BY DESIGN — drop those solids'
  // segments so the red highlight and the counters only report unexpected openings. The
  // authoritative count comes from the import diagnostics (same exclusion, core-side).
  openSolidsSet = new Set(res.openSolids);
  sheetBodies = res.openSolids.length;
  openEdges = res.diagnostics.openEdges;
  const b = dropSolidSegments(boundaryEdges(mesh, res.mesh.solidOfTri), openSolidsSet);
  const feat = featureEdges(mesh, res.mesh.faceOfTri, res.mesh.solidOfTri);
  solidOfTri = res.mesh.solidOfTri;
  defectEdges = b;
  closeMenus(); // a stale context menu would act on the previous model's ids

  viewer.setMesh(geo, b, feat, faceColors, res.mesh.solidOfTri);
  viewer.fit();
  buildPartsPanel(res.structure);

  showGeometryInfo(res, openEdges);

  dev = null;
  recomputeDeviation();
  refreshStats();

  hideOverlay();
  convertBtn.disabled = false;
  exportBtn.disabled = false;
  setStatus(`Done · ${triCount(mesh).toLocaleString()} tris`);
}

// ---- parts tree ----
// Hidden state lives in one Set of solid ids; checkbox checked/indeterminate states are derived
// from it after every change, so parent and child boxes can never disagree with the viewer.
let hiddenParts = new Set<number>();
let allPartSolids: number[] = [];
const partRows: { cb: HTMLInputElement; row: HTMLElement; solids: number[] }[] = [];

// Right-click lookup: body id -> display label + owning part. The context menu acts on ONE
// body — the same granularity as the tree's finest rows (a multi-body part lists each body
// as its own row, so right-clicking one must not drag its siblings along). `label` mirrors
// the tree's row naming.
interface SolidInfo { label: string; partName: string; occurrences: number }
let solidInfo = new Map<number, SolidInfo>();

function indexSolidInfo(n: PartNode, occ: number): void {
  const o = occ * n.occurrences;
  // A leaf part with a single body is named after the part; a body sharing its node with
  // siblings (or subassemblies) goes by its own name, like its tree row.
  const soleLeafBody = n.children.length === 0 && n.bodies.length === 1;
  for (const [i, b] of n.bodies.entries()) {
    const label = soleLeafBody
      ? n.name || b.name || "(unnamed part)"
      : b.name || (n.bodies.length > 1 ? `Body ${i + 1}` : "Body");
    solidInfo.set(b.id, { label, partName: n.name, occurrences: o });
  }
  for (const c of n.children) indexSolidInfo(c, o);
}

/** Collapse wrapper levels (a product whose rep only points at one sub-product carries no
 * geometry of its own): keep the outermost name, multiply occurrence counts. */
function mergeChains(n: PartNode): PartNode {
  let name = n.name;
  let occurrences = n.occurrences;
  let cur = n;
  while (cur.bodies.length === 0 && cur.children.length === 1) {
    cur = cur.children[0]!;
    occurrences *= cur.occurrences;
    if (!name) name = cur.name;
  }
  return { name, occurrences, bodies: cur.bodies, children: cur.children.map(mergeChains) };
}

function subtreeSolids(n: PartNode, out: number[] = []): number[] {
  for (const b of n.bodies) out.push(b.id);
  for (const c of n.children) subtreeSolids(c, out);
  return out;
}

function applyHiddenParts(): void {
  viewer.setHiddenSolids(hiddenParts);
  for (const r of partRows) {
    const hidden = r.solids.reduce((s, id) => s + (hiddenParts.has(id) ? 1 : 0), 0);
    r.cb.checked = hidden === 0;
    r.cb.indeterminate = hidden > 0 && hidden < r.solids.length;
    r.row.classList.toggle("off", hidden === r.solids.length && r.solids.length > 0);
  }
}

function partRow(label: string, meta: string, solids: number[], depth: number, children: (() => HTMLElement[]) | null): HTMLElement {
  const row = document.createElement("div");
  row.className = "part-row";
  row.style.paddingLeft = `${depth * 14}px`;

  const caret = document.createElement("span");
  caret.className = "part-caret" + (children ? "" : " empty");
  caret.textContent = "▶";

  const cb = document.createElement("input");
  cb.type = "checkbox";
  cb.checked = true;

  const name = document.createElement("span");
  name.className = "part-name";
  name.textContent = label;
  name.title = label;

  const metaEl = document.createElement("span");
  metaEl.className = "part-meta";
  metaEl.textContent = meta;

  row.append(caret, cb, name, metaEl);
  partRows.push({ cb, row, solids });

  cb.addEventListener("change", () => {
    for (const id of solids) cb.checked ? hiddenParts.delete(id) : hiddenParts.add(id);
    applyHiddenParts();
  });
  row.addEventListener("mouseenter", () => viewer.setHighlightSolids(new Set(solids)));
  row.addEventListener("mouseleave", () => viewer.setHighlightSolids(null));

  const wrap = document.createElement("div");
  wrap.appendChild(row);
  if (children) {
    // Children are built lazily on first expand (a PCB's copper layer alone is ~700 bodies).
    let box: HTMLElement | null = null;
    caret.addEventListener("click", () => {
      if (!box) {
        box = document.createElement("div");
        for (const el of children()) box.appendChild(el);
        wrap.appendChild(box);
        applyHiddenParts(); // sync the freshly created checkboxes
      } else {
        box.hidden = !box.hidden;
      }
      caret.textContent = box.hidden ? "▶" : "▼";
    });
  }
  return wrap;
}

function nodeRows(n: PartNode, depth: number): HTMLElement[] {
  const rows: HTMLElement[] = [];
  // "sheet" tags a surface body (open by design) so a boundary-edged part reads as intended.
  const sheetTag = (ids: number[]): string =>
    ids.length > 0 && ids.every((id) => openSolidsSet.has(id)) ? "sheet" : "";
  for (const child of n.children) {
    const solids = subtreeSolids(child);
    const meta = [
      child.occurrences > 1 ? `×${child.occurrences}` : "",
      child.bodies.length > 1 ? `${child.bodies.length} bodies` : "",
      sheetTag(solids),
    ].filter(Boolean).join(" · ");
    const expandable = child.children.length > 0 || child.bodies.length > 1;
    rows.push(partRow(child.name || "(unnamed)", meta, solids, depth,
      expandable ? () => nodeRows(child, depth + 1) : null));
  }
  if (n.children.length > 0 && n.bodies.length === 1) {
    // A single own body next to subassemblies still deserves a row of its own.
    const b = n.bodies[0]!;
    rows.push(partRow(b.name || "Body", sheetTag([b.id]), [b.id], depth, null));
  } else if (n.bodies.length > 1 || (n.children.length === 0 && n.bodies.length > 0)) {
    for (const [i, b] of n.bodies.entries()) {
      rows.push(partRow(b.name || `Body ${i + 1}`, sheetTag([b.id]), [b.id], depth, null));
    }
  }
  return rows;
}

function buildPartsPanel(root: PartNode): void {
  hiddenParts = new Set();
  partRows.length = 0;
  partsTree.textContent = "";
  const merged = mergeChains(root);
  allPartSolids = subtreeSolids(merged);
  solidInfo = new Map();
  indexSolidInfo(merged, 1); // the context menu needs the lookup even for single-part files
  const show = merged.children.length > 0 || merged.bodies.length > 1;
  partsPanel.hidden = !show;
  if (!show) return;
  for (const el of nodeRows(merged, 0)) partsTree.appendChild(el);
}

partsShowAll.addEventListener("click", () => {
  hiddenParts.clear();
  applyHiddenParts();
});
partsHideAll.addEventListener("click", () => {
  hiddenParts = new Set(allPartSolids);
  applyHiddenParts();
});

// ---- viewport context menu ----
// Right-click (no drag) on the model: menu for the part under the cursor; on empty space:
// viewport-level actions. Both popups are rebuilt per open, fixed-positioned at the cursor.
const ctxMenu = $("ctxMenu");
const partPop = $("partPop");

function closeMenus(): void {
  ctxMenu.hidden = true;
  partPop.hidden = true;
  viewer.setHighlightSolids(null);
}

function menuItem(label: string, action: () => void, enabled = true): HTMLButtonElement {
  const b = document.createElement("button");
  b.type = "button";
  b.className = "ctx-item";
  b.textContent = label;
  b.disabled = !enabled;
  b.addEventListener("click", () => {
    closeMenus();
    action();
  });
  return b;
}

/** Show a fixed-position popup at (x, y), nudged to stay inside the window. */
function placeAt(el: HTMLElement, x: number, y: number): void {
  el.hidden = false;
  el.style.left = "0px"; // reset so offsetWidth/Height measure the natural size
  el.style.top = "0px";
  el.style.left = `${Math.max(4, Math.min(x, window.innerWidth - el.offsetWidth - 8))}px`;
  el.style.top = `${Math.max(4, Math.min(y, window.innerHeight - el.offsetHeight - 8))}px`;
}

viewer.onContextMenu = (solidId, x, y) => {
  closeMenus();
  if (!mesh) return;
  ctxMenu.textContent = "";
  const info = solidId == null ? undefined : solidInfo.get(solidId);
  if (info && solidId != null) {
    const id = solidId;
    const title = document.createElement("div");
    title.className = "ctx-title";
    title.textContent = info.label;
    ctxMenu.appendChild(title);
    viewer.setHighlightSolids(new Set([id])); // flash what the menu will act on
    const others = allPartSolids.filter((s) => s !== id);
    ctxMenu.append(
      menuItem("Isolate part", () => {
        hiddenParts = new Set(others);
        applyHiddenParts();
      }, others.length > 0),
      menuItem("Hide part", () => {
        hiddenParts.add(id);
        applyHiddenParts();
      }),
      menuItem("Zoom to part", () => viewer.fitSolids(new Set([id]))),
      menuItem("Part info…", () => showPartInfo(info, id, x, y)),
    );
    const sep = document.createElement("div");
    sep.className = "ctx-sep";
    ctxMenu.appendChild(sep);
  }
  ctxMenu.append(
    menuItem("Show all parts", () => {
      hiddenParts.clear();
      applyHiddenParts();
    }, hiddenParts.size > 0),
    menuItem("Fit view", () => viewer.fit()),
  );
  placeAt(ctxMenu, x, y);
};

function showPartInfo(info: SolidInfo, solidId: number, x: number, y: number): void {
  if (!mesh || !solidOfTri) return;
  const ids = new Set([solidId]);
  viewer.setHighlightSolids(ids); // keep the part marked while the popup is up
  // Triangles, bbox, surface area and signed volume (divergence theorem — only meaningful
  // for a closed mesh) of the body. Instances share solid ids, so everything covers ALL
  // occurrences of a multiply-placed part (labelled accordingly below).
  let tris = 0;
  let area = 0;
  let vol6 = 0; // 6 × signed volume
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  const idx = mesh.indices, pos = mesh.positions;
  const p = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (let t = 0; t < solidOfTri.length; t++) {
    if (!ids.has(solidOfTri[t]!)) continue;
    tris++;
    for (let e = 0; e < 3; e++) {
      const v = idx[t * 3 + e]! * 3;
      const px = pos[v]!, py = pos[v + 1]!, pz = pos[v + 2]!;
      p[e * 3] = px; p[e * 3 + 1] = py; p[e * 3 + 2] = pz;
      if (px < minX) minX = px;
      if (px > maxX) maxX = px;
      if (py < minY) minY = py;
      if (py > maxY) maxY = py;
      if (pz < minZ) minZ = pz;
      if (pz > maxZ) maxZ = pz;
    }
    // cross(b - a, c - a): 2 × area vector; a · (b × c): 6 × signed tet volume from the origin
    const abx = p[3]! - p[0]!, aby = p[4]! - p[1]!, abz = p[5]! - p[2]!;
    const acx = p[6]! - p[0]!, acy = p[7]! - p[1]!, acz = p[8]! - p[2]!;
    const cx = aby * acz - abz * acy;
    const cy = abz * acx - abx * acz;
    const cz = abx * acy - aby * acx;
    area += Math.sqrt(cx * cx + cy * cy + cz * cz) / 2;
    vol6 += p[0]! * (p[4]! * p[8]! - p[5]! * p[7]!)
          - p[1]! * (p[3]! * p[8]! - p[5]! * p[6]!)
          + p[2]! * (p[3]! * p[7]! - p[4]! * p[6]!);
  }
  let open = 0;
  if (defectEdges) {
    for (let s = 0; s < defectEdges.count; s++) if (ids.has(defectEdges.solidOfSeg[s]!)) open++;
  }
  const isSheet = openSolidsSet.has(solidId);

  partPop.textContent = "";
  const title = document.createElement("div");
  title.className = "ctx-title";
  title.textContent = info.label;
  title.title = title.textContent;
  partPop.appendChild(title);
  const dl = document.createElement("dl");
  dl.className = "info";
  const row = (dt: string, dd: string): void => {
    const div = document.createElement("div");
    const dtEl = document.createElement("dt");
    dtEl.textContent = dt;
    const ddEl = document.createElement("dd");
    ddEl.textContent = dd;
    div.append(dtEl, ddEl);
    dl.appendChild(div);
  };
  if (info.partName && info.partName !== info.label) row("Part", info.partName);
  if (info.occurrences > 1) row("Instances", `×${info.occurrences}`);
  row("Triangles", tris.toLocaleString());
  if (tris > 0) {
    row(info.occurrences > 1 ? "Extent (all)" : "Size",
      `${fmtLen(maxX - minX)} × ${fmtLen(maxY - minY)} × ${fmtLen(maxZ - minZ)} mm`);
    // Volume needs a closed surface — an open or sheet body would give a bogus number.
    if (!isSheet && open === 0) row(info.occurrences > 1 ? "Volume (all)" : "Volume", fmtVol(Math.abs(vol6) / 6));
    row(info.occurrences > 1 ? "Area (all)" : "Surface area", fmtArea(area));
  }
  row("Type", isSheet ? "sheet body (open by design)"
    : open === 0 ? "solid · watertight"
    : `solid · ${open.toLocaleString()} open edge${open === 1 ? "" : "s"}`);
  partPop.appendChild(dl);
  placeAt(partPop, x, y);
}

document.addEventListener("pointerdown", (ev) => {
  if (ctxMenu.hidden && partPop.hidden) return;
  const t = ev.target as Node;
  if (ctxMenu.contains(t) || partPop.contains(t)) return;
  closeMenus();
});
document.addEventListener("keydown", (ev) => {
  if (ev.key === "Escape") closeMenus();
});
// Zooming under an open menu would leave it hovering over the wrong spot.
document.addEventListener("wheel", () => {
  if (!ctxMenu.hidden || !partPop.hidden) closeMenus();
}, { capture: true, passive: true });

// ---- export ----
exportBtn.addEventListener("click", () => {
  if (!mesh) return;
  const stl = writeBinarySTL(mesh);
  const blob = new Blob([stl.buffer as ArrayBuffer], { type: "model/stl" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `${stepBaseName}.stl`;
  a.click();
  URL.revokeObjectURL(url);
});

// ---- deviation ----
function recomputeDeviation(): void {
  if (!reference || !geo) {
    dev = null;
    legend.hidden = true;
    return;
  }
  dev = reference.deviationFor(geo, null);
  recolorDeviation();
}

function recolorDeviation(): void {
  if (!dev) return;
  const clamp = manualRange ?? Math.max(dev.maxAbs, 1e-9);
  viewer.setDeviationColors(colorize(dev.signed, clamp));
  updateLegend(clamp);
  refreshStats();
}

function colorize(signed: Float32Array, clamp: number): Float32Array {
  const out = new Float32Array(signed.length * 3);
  for (let i = 0; i < signed.length; i++) {
    const [r, g, b] = diverging(signed[i]!, clamp);
    out[i * 3] = r;
    out[i * 3 + 1] = g;
    out[i * 3 + 2] = b;
  }
  return out;
}

function updateLegend(clamp: number): void {
  if (!tDev.checked) {
    legend.hidden = true;
    return;
  }
  legend.hidden = false;
  legNeg.textContent = `-${clamp.toFixed(3)}`;
  legPos.textContent = `+${clamp.toFixed(3)}`;
  if (manualRange == null) devRange.value = clamp.toFixed(3);
  legMeta.textContent = dev ? `max |dev| ${dev.maxAbs.toFixed(4)} · rms ${dev.rms.toFixed(4)} mm` : "";
}

// ---- toggles ----
tColors.addEventListener("change", () => viewer.setShowColors(tColors.checked));
tWire.addEventListener("change", () => viewer.setWireframe(tWire.checked));
tEdges.addEventListener("change", () => viewer.setOpenEdges(tEdges.checked));
tFeature.addEventListener("change", () => viewer.setFeatureEdges(tFeature.checked));
tTransparent.addEventListener("change", () => viewer.setTransparent(tTransparent.checked));
tEdgesOnly.addEventListener("change", () => {
  viewer.setSurfacesVisible(!tEdgesOnly.checked);
  // A pure edge view is meaningless without the CAD boundaries — turn them on.
  if (tEdgesOnly.checked && !tFeature.checked) {
    tFeature.checked = true;
    viewer.setFeatureEdges(true);
  }
});
tSection.addEventListener("change", () => {
  viewer.setSection(tSection.checked);
  sectionRow.hidden = !tSection.checked;
});
$<HTMLButtonElement>("secX").addEventListener("click", () => viewer.setSectionAxis("x"));
$<HTMLButtonElement>("secY").addEventListener("click", () => viewer.setSectionAxis("y"));
$<HTMLButtonElement>("secZ").addEventListener("click", () => viewer.setSectionAxis("z"));
$<HTMLButtonElement>("secFlip").addEventListener("click", () => viewer.flipSection());

let orthoOn = false;
projBtn.addEventListener("click", () => {
  orthoOn = !orthoOn;
  viewer.setProjection(orthoOn);
  projBtn.textContent = orthoOn ? "Ortho" : "Persp";
});
tRef.addEventListener("change", () => viewer.setReferenceVisible(tRef.checked));
tDev.addEventListener("change", () => {
  viewer.setDeviation(tDev.checked);
  rangeField.hidden = !tDev.checked;
  if (tDev.checked && !dev) recomputeDeviation();
  legend.hidden = !tDev.checked;
  if (tDev.checked) recolorDeviation();
});
devRange.addEventListener("input", () => {
  const v = parseFloat(devRange.value);
  manualRange = isFinite(v) && v > 0 ? v : null;
  recolorDeviation();
});
autoRange.addEventListener("click", () => {
  manualRange = null;
  recolorDeviation();
});

$<HTMLButtonElement>("fitBtn").addEventListener("click", () => viewer.fit());

// ---- stats panel ----
function refreshStats(): void {
  if (!mesh) {
    statsEl.textContent = "";
    return;
  }
  const lines = [
    `${triCount(mesh).toLocaleString()} triangles`,
    `${openEdges}${sheetBodies > 0 ? " unexpected" : ""} open edge${openEdges === 1 ? "" : "s"}`,
  ];
  if (sheetBodies > 0) lines.push(`${sheetBodies} sheet bod${sheetBodies === 1 ? "y" : "ies"}`);
  if (dev && tDev.checked) {
    lines.push(`max dev ${dev.maxAbs.toFixed(4)} mm`, `rms ${dev.rms.toFixed(4)} mm`);
  }
  statsEl.textContent = lines.join("\n");
}

// ---- file info panel ----
// Header fields render on file load (no tessellation needed); geometry fields fill in on Convert.
function showHeaderInfo(text: string): void {
  const h = parseStepHeader(text);
  infoPanel.hidden = false;
  iSystem.textContent = h.originatingSystem ?? h.preprocessor ?? "—";
  iSchema.textContent = h.schemaLabel ?? h.schema ?? "—";
  iDate.textContent = fmtDate(h.timeStamp);
  // reset geometry fields until this file is converted
  iUnits.textContent = "—";
  iDims.textContent = "—";
  iBodies.textContent = "—";
  iFaces.textContent = "—";
  iTris.textContent = "—";
  watertight.hidden = true;
}

function showGeometryInfo(res: ConvertResult, edges: number): void {
  infoPanel.hidden = false;
  iUnits.textContent = res.units || "—";
  iDims.textContent = res.bbox ? fmtDims(res.bbox) : "—";
  const sheets = res.openSolids.length;
  iBodies.textContent = sheets > 0
    ? `${res.stats.solids.toLocaleString()} (${sheets.toLocaleString()} sheet${sheets === 1 ? "" : "s"})`
    : res.stats.solids.toLocaleString();
  iFaces.textContent = res.stats.facesTotal.toLocaleString();
  iTris.textContent = triCount(res.mesh).toLocaleString();
  watertight.hidden = false;
  if (edges === 0 && sheets === 0) {
    watertight.textContent = "✓ Watertight · print-ready";
    watertight.className = "badge ok";
    watertight.title = "";
  } else if (edges === 0) {
    // Sheet bodies (OPEN_SHELL surface models) have boundary edges by design — with none
    // unexpected, the conversion is clean, it just isn't a solid.
    watertight.textContent = `✓ Clean · ${sheets.toLocaleString()} sheet bod${sheets === 1 ? "y" : "ies"}`;
    watertight.className = "badge ok";
    watertight.title = "Surface (sheet) bodies are open by design; their boundary edges are not defects. No unexpected open edges found.";
  } else {
    watertight.textContent = `⚠ ${edges.toLocaleString()} unexpected open edge${edges === 1 ? "" : "s"}`;
    watertight.className = "badge warn";
    watertight.title = sheets > 0 ? `Sheet-body boundaries (${sheets} bodies) are excluded from this count.` : "";
  }
}

function fmtLen(v: number): string {
  const a = Math.abs(v);
  const s = a >= 100 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
  return s.replace(/\.?0+$/, "");
}
function fmtDims(b: [number, number, number, number, number, number]): string {
  return `${fmtLen(b[3] - b[0])} × ${fmtLen(b[4] - b[1])} × ${fmtLen(b[5] - b[2])} mm`;
}
/** mm³, switching to cm³ from 1000 mm³ up (the natural unit for printable parts). */
function fmtVol(v: number): string {
  return v >= 1000
    ? `${(v / 1000).toLocaleString(undefined, { maximumFractionDigits: 2 })} cm³`
    : `${v.toLocaleString(undefined, { maximumFractionDigits: 2 })} mm³`;
}
/** mm², switching to cm² from 1000 mm² up. */
function fmtArea(a: number): string {
  return a >= 1000
    ? `${(a / 100).toLocaleString(undefined, { maximumFractionDigits: 2 })} cm²`
    : `${a.toLocaleString(undefined, { maximumFractionDigits: 2 })} mm²`;
}
function fmtDate(ts: string | undefined): string {
  if (!ts) return "—";
  const d = new Date(ts);
  return isNaN(d.getTime()) ? ts : d.toLocaleDateString(undefined, { year: "numeric", month: "short", day: "numeric" });
}

// ---- small utils ----
function num(id: string, fallback: number): number {
  const v = parseFloat($<HTMLInputElement>(id).value);
  return isFinite(v) ? v : fallback;
}
function setStatus(text: string, error = false): void {
  statusEl.textContent = text;
  statusEl.classList.toggle("error", error);
}
function showOverlay(text: string): void {
  overlayText.textContent = text;
  progressBar.hidden = true;
  progressFill.style.width = "0%";
  overlay.hidden = false;
}
function hideOverlay(): void {
  overlay.hidden = true;
}
