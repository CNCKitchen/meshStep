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

const tColors = $<HTMLInputElement>("tColors");
const tWire = $<HTMLInputElement>("tWire");
const tEdges = $<HTMLInputElement>("tEdges");
const tFeature = $<HTMLInputElement>("tFeature");
const tTransparent = $<HTMLInputElement>("tTransparent");
const tEdgesOnly = $<HTMLInputElement>("tEdgesOnly");
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
  geo = buildIndexedGeometry(displayMesh);

  // Open-edge (defect) overlay: a sheet body's boundary is open BY DESIGN — drop those solids'
  // segments so the red highlight and the counters only report unexpected openings. The
  // authoritative count comes from the import diagnostics (same exclusion, core-side).
  openSolidsSet = new Set(res.openSolids);
  sheetBodies = res.openSolids.length;
  openEdges = res.diagnostics.openEdges;
  const b = dropSolidSegments(boundaryEdges(mesh, res.mesh.solidOfTri), openSolidsSet);
  const feat = featureEdges(mesh, res.mesh.faceOfTri, res.mesh.solidOfTri);

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
const partRows: { cb: HTMLInputElement; row: HTMLElement; solids: number[] }[] = [];

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
  const show = merged.children.length > 0 || merged.bodies.length > 1;
  partsPanel.hidden = !show;
  if (!show) return;
  for (const el of nodeRows(merged, 0)) partsTree.appendChild(el);
}

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
  overlay.hidden = false;
}
function hideOverlay(): void {
  overlay.hidden = true;
}
