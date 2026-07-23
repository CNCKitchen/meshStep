// SPDX-License-Identifier: AGPL-3.0-only

/**
 * Minimal dynamic ruler along the left viewport edge, shown in orthographic
 * views only. Zero sits near the top edge and values grow downward; the major
 * step follows a 1-2-5 progression so labels stay round at any zoom. Strokes
 * are theme ink over a hairline opposite-color halo: invisible on the plain
 * background, but it keeps ticks readable where a mid-gray part sits beneath
 * (blend-mode inversion fails exactly there — 50% gray inverts to itself).
 * Purely visual (pointer-events: none) and cheap: it redraws only when the
 * mm-per-pixel scale, viewport height, pixel ratio or theme changes.
 */

const CSS_WIDTH = 64; // canvas strip width: tick + label room
const MAJOR_PX = 90; // target on-screen spacing between labelled ticks
const MAJOR_LEN = 15;
const MINOR_LEN = 7;
const TOP_FRAC = 0.1; // zero sits this far down the viewport: headroom to align a feature

const INK = { light: "#1b2430", dark: "#dfe6ee" };
const HALO = { light: "rgba(255, 255, 255, 0.7)", dark: "rgba(10, 14, 18, 0.7)" };

export class RulerOverlay {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private theme: "light" | "dark" = "dark";
  private lastKey = ""; // change detector: scale|height|dpr|theme

  constructor(container: HTMLElement) {
    this.canvas = document.createElement("canvas");
    const s = this.canvas.style;
    s.position = "absolute";
    s.left = "0";
    s.top = "0";
    s.width = `${CSS_WIDTH}px`;
    s.pointerEvents = "none";
    s.display = "none";
    container.appendChild(this.canvas);
    this.ctx = this.canvas.getContext("2d")!;
  }

  setTheme(mode: "light" | "dark"): void {
    this.theme = mode;
    this.lastKey = "";
  }

  /** Per-frame hook. `mmPerPx` is world mm per CSS pixel; `h` the viewport CSS height. */
  update(visible: boolean, mmPerPx: number, h: number): void {
    if (!visible) {
      this.canvas.style.display = "none";
      return;
    }
    this.canvas.style.display = "block";
    const dpr = window.devicePixelRatio || 1;
    const key = `${mmPerPx}|${h}|${dpr}|${this.theme}`;
    if (key === this.lastKey) return;
    this.lastKey = key;
    this.draw(mmPerPx, h, dpr);
  }

  private draw(mmPerPx: number, h: number, dpr: number): void {
    const ctx = this.ctx;
    this.canvas.width = Math.round(CSS_WIDTH * dpr);
    this.canvas.height = Math.round(h * dpr);
    this.canvas.style.height = `${h}px`;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, CSS_WIDTH, h);
    if (!(mmPerPx > 0) || h <= 0) return;

    // Major step: smallest 1/2/5·10^n whose on-screen spacing is >= MAJOR_PX.
    const target = MAJOR_PX * mmPerPx;
    const pow = 10 ** Math.floor(Math.log10(target));
    const m = target / pow;
    const mant = m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10;
    const step = mant * pow;
    const perMajor = mant === 2 ? 4 : 5; // minors per major (2 splits into quarters)
    const minor = step / perMajor;
    const decimals = Math.max(0, -Math.floor(Math.log10(step) + 1e-9));

    const zeroY = Math.round(TOP_FRAC * h);
    const ink = INK[this.theme];
    const halo = HALO[this.theme];
    ctx.font = "10px ui-sans-serif, system-ui, sans-serif";
    ctx.textBaseline = "middle";
    ctx.lineJoin = "round";

    // One path for baseline + ticks, stroked twice: a soft opposite-color halo
    // underneath, then the crisp 1px ink line.
    const n = Math.floor(((h - zeroY) * mmPerPx) / minor);
    ctx.beginPath();
    ctx.moveTo(0.5, zeroY);
    ctx.lineTo(0.5, h);
    for (let i = 0; i <= n; i++) {
      const y = Math.round(zeroY + (i * minor) / mmPerPx) + 0.5;
      const len = i % perMajor === 0 ? MAJOR_LEN : MINOR_LEN;
      ctx.moveTo(0, y);
      ctx.lineTo(len, y);
    }
    ctx.lineWidth = 3;
    ctx.strokeStyle = halo;
    ctx.stroke();
    ctx.lineWidth = 1;
    ctx.strokeStyle = ink;
    ctx.stroke();

    ctx.lineWidth = 3;
    ctx.strokeStyle = halo;
    ctx.fillStyle = ink;
    for (let i = 0; i <= n; i += perMajor) {
      const v = i * minor;
      const y = Math.min(Math.max(zeroY + v / mmPerPx, 6), h - 6);
      const label = v.toFixed(decimals);
      ctx.strokeText(label, MAJOR_LEN + 4, y);
      ctx.fillText(label, MAJOR_LEN + 4, y);
      if (i === 0) {
        ctx.strokeText("mm", MAJOR_LEN + 4, y + 12); // unit hint under the 0
        ctx.fillText("mm", MAJOR_LEN + 4, y + 12);
      }
    }
  }
}
