// SPDX-License-Identifier: AGPL-3.0-only
import "./styles.css";
import * as THREE from "three";
import { readSTL, writeBinarySTL, parseStepHeader } from "../../src/index.ts";
import { Viewer } from "./viewer.ts";
import { ReferenceSurface, type DeviationResult } from "./deviation.ts";
import {
  buildIndexedGeometry,
  buildSoupGeometry,
  boundaryEdges,
  featureEdges,
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
let openEdges = 0;

let reference: ReferenceSurface | null = null;
let dev: DeviationResult | null = null;
let manualRange: number | null = null; // null => auto

// ---- input: STEP ----
stepFile.addEventListener("change", async () => {
  const f = stepFile.files?.[0];
  if (!f) return;
  stepText = await f.text();
  stepBaseName = f.name.replace(/\.(step|stp)$/i, "") || "mesh";
  stepName.textContent = f.name;
  convertBtn.disabled = false;
  setStatus("");
  showHeaderInfo(stepText);
});

// ---- input: reference STL ----
refFile.addEventListener("change", async () => {
  const f = refFile.files?.[0];
  if (!f) return;
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
  geo = buildIndexedGeometry(mesh);

  const b = boundaryEdges(mesh);
  openEdges = b.count;
  const feat = featureEdges(mesh, res.mesh.faceOfTri);

  viewer.setMesh(geo, b.positions, feat.positions);
  viewer.fit();

  showGeometryInfo(res, openEdges);

  dev = null;
  recomputeDeviation();
  refreshStats();

  hideOverlay();
  convertBtn.disabled = false;
  exportBtn.disabled = false;
  setStatus(`Done · ${triCount(mesh).toLocaleString()} tris`);
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
    `${openEdges} open edge${openEdges === 1 ? "" : "s"}`,
  ];
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
  iBodies.textContent = res.stats.solids.toLocaleString();
  iFaces.textContent = res.stats.facesTotal.toLocaleString();
  iTris.textContent = triCount(res.mesh).toLocaleString();
  watertight.hidden = false;
  if (edges === 0) {
    watertight.textContent = "✓ Watertight · print-ready";
    watertight.className = "badge ok";
  } else {
    watertight.textContent = `⚠ ${edges.toLocaleString()} open edge${edges === 1 ? "" : "s"}`;
    watertight.className = "badge warn";
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
