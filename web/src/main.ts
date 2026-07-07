// SPDX-License-Identifier: AGPL-3.0-only
import "./styles.css";
import * as THREE from "three";
import { readSTL, writeBinarySTL } from "../../src/index.ts";
import { Viewer } from "./viewer.ts";
import { ReferenceSurface, type DeviationResult } from "./deviation.ts";
import {
  buildIndexedGeometry,
  buildSoupGeometry,
  boundaryEdges,
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

// ---- state ----
const viewer = new Viewer($("viewport"));
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

  viewer.setMesh(geo, b.positions);
  viewer.fit();

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
