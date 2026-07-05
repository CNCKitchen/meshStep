// SPDX-License-Identifier: AGPL-3.0-only
import "./styles.css";
import * as THREE from "three";
import { readSTL } from "../../src/index.ts";
import { DualViewer, type Side } from "./viewer.ts";
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
const statsLeft = $("statsLeft");
const statsRight = $("statsRight");

// ---- state ----
const viewer = new DualViewer($("viewport"));
let stepText: string | null = null;
let worker: Worker | null = null;

let meshLeft: RawMesh | null = null;
let meshRight: RawMesh | null = null;
let geoLeft: THREE.BufferGeometry | null = null;
let geoRight: THREE.BufferGeometry | null = null;
let openLeft = 0;
let openRight = 0;

let reference: ReferenceSurface | null = null;
let devLeft: DeviationResult | null = null;
let devRight: DeviationResult | null = null;
let manualRange: number | null = null; // null => auto

// ---- input: STEP ----
stepFile.addEventListener("change", async () => {
  const f = stepFile.files?.[0];
  if (!f) return;
  stepText = await f.text();
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
    const geo = buildSoupGeometry(soup.positions);
    reference?.dispose();
    reference = new ReferenceSurface(geo);
    refName.textContent = `${f.name}  ·  ${soup.triangleCount.toLocaleString()} tris`;
    viewer.setReference(geo);
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
    remeshIterations: Math.round(num("remeshIterations", 8)),
    remesh: true,
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
  meshLeft = res.tessellated;
  meshRight = res.remeshed;

  geoLeft = buildIndexedGeometry(meshLeft);
  geoRight = buildIndexedGeometry(meshRight);

  const bl = boundaryEdges(meshLeft);
  const br = boundaryEdges(meshRight);
  openLeft = bl.count;
  openRight = br.count;

  viewer.setSide("left", geoLeft, bl.positions);
  viewer.setSide("right", geoRight, br.positions);
  viewer.fit();

  devLeft = devRight = null;
  recomputeDeviation();
  refreshStats();

  hideOverlay();
  convertBtn.disabled = false;
  setStatus(`Done · ${triCount(meshLeft).toLocaleString()} → ${triCount(meshRight).toLocaleString()} tris`);
}

// ---- deviation ----
function recomputeDeviation(): void {
  if (!reference || !geoLeft || !geoRight) {
    devLeft = devRight = null;
    legend.hidden = true;
    return;
  }
  devLeft = reference.deviationFor(geoLeft, null);
  devRight = reference.deviationFor(geoRight, null);
  recolorDeviation();
}

function recolorDeviation(): void {
  if (!devLeft || !devRight) return;
  const clamp = manualRange ?? Math.max(devLeft.maxAbs, devRight.maxAbs, 1e-9);
  viewer.setDeviationColors("left", colorize(devLeft.signed, clamp));
  viewer.setDeviationColors("right", colorize(devRight.signed, clamp));
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
  const m = devRight ?? devLeft;
  legMeta.textContent = m ? `max |dev| ${m.maxAbs.toFixed(4)} · rms ${m.rms.toFixed(4)} mm` : "";
}

// ---- toggles ----
tWire.addEventListener("change", () => viewer.setWireframe(tWire.checked));
tEdges.addEventListener("change", () => viewer.setOpenEdges(tEdges.checked));
tRef.addEventListener("change", () => viewer.setReferenceVisible(tRef.checked));
tDev.addEventListener("change", () => {
  viewer.setDeviation(tDev.checked);
  rangeField.hidden = !tDev.checked;
  if (tDev.checked && (!devLeft || !devRight)) recomputeDeviation();
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

// ---- stats panels ----
function refreshStats(): void {
  statsLeft.textContent = sideStats(meshLeft, openLeft, devLeft);
  statsRight.textContent = sideStats(meshRight, openRight, devRight);
}

function sideStats(mesh: RawMesh | null, open: number, dev: DeviationResult | null): string {
  if (!mesh) return "";
  const lines = [`${triCount(mesh).toLocaleString()} triangles`, `${open} open edge${open === 1 ? "" : "s"}`];
  if (dev && tDev.checked) {
    lines.push(`max dev ${dev.maxAbs.toFixed(4)} mm`, `rms ${dev.rms.toFixed(4)} mm`);
  }
  return lines.join("\n");
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
