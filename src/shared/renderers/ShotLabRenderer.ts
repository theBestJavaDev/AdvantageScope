// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import TabRenderer from "./TabRenderer";

/** Everything the Shot Lab renderer draws. Pre-projected by the controller so satellites stay dumb. */
export type ShotLabRendererCommand = {
  /** Increments whenever the results change. */
  version: number;
  status: { text: string; kind: "info" | "busy" | "ok" | "warn" };
  result: ShotLabRendererResult | null;
};

export type ShotLabRendererResult = {
  cards: { label: string; value: string; detail: string; kind?: "good" | "bad" | "neutral" }[];
  side: {
    /** [distance from robot along the shot line, height] in meters. */
    nominal: [number, number][];
    samples: { points: [number, number][]; hit: boolean }[];
    target: { near: number; far: number; bottom: number; top: number };
    launch: [number, number];
  };
  top: {
    /** Crossings of the target plane relative to the target center, field frame (m). */
    points: { x: number; y: number; hit: boolean }[];
    halfX: number;
    halfY: number;
    /** Unit vector of the shot direction (field frame) for the arrow. */
    direction: [number, number];
    mean: [number, number] | null;
  };
  budget:
    | {
        label: string;
        hitRate: number;
        cost: number;
        share: number;
        rangeStd: number;
        lateralStd: number;
        summary: boolean;
      }[]
    | null;
};

export default class ShotLabRenderer implements TabRenderer {
  private STATUS: HTMLElement;
  private CARDS: HTMLElement;
  private SIDE_CANVAS: HTMLCanvasElement;
  private TOP_CANVAS: HTMLCanvasElement;
  private BUDGET_CONTAINER: HTMLElement;
  private BUDGET_BODY: HTMLElement;
  private EMPTY: HTMLElement;

  private lastVersion = -1;
  private lastStatus = "";
  private lastSizes = "";
  private lastDark: boolean | null = null;

  constructor(root: HTMLElement) {
    this.STATUS = root.getElementsByClassName("shotlab-status")[0] as HTMLElement;
    this.CARDS = root.getElementsByClassName("shotlab-cards")[0] as HTMLElement;
    this.SIDE_CANVAS = root.getElementsByClassName("shotlab-side")[0] as HTMLCanvasElement;
    this.TOP_CANVAS = root.getElementsByClassName("shotlab-top")[0] as HTMLCanvasElement;
    this.BUDGET_CONTAINER = root.getElementsByClassName("shotlab-budget-container")[0] as HTMLElement;
    this.BUDGET_BODY = root.getElementsByClassName("shotlab-budget")[0].firstElementChild as HTMLElement;
    this.EMPTY = root.getElementsByClassName("shotlab-empty")[0] as HTMLElement;
  }

  saveState(): unknown {
    return null;
  }

  restoreState(): void {}

  getAspectRatio(): number | null {
    return null;
  }

  render(command: ShotLabRendererCommand): void {
    if (command === null || command === undefined) return;

    // Status line
    const statusKey = command.status.kind + command.status.text;
    if (statusKey !== this.lastStatus) {
      this.lastStatus = statusKey;
      this.STATUS.innerText = command.status.text;
      this.STATUS.className = "shotlab-status " + command.status.kind;
    }

    const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
    const sizes = [this.SIDE_CANVAS, this.TOP_CANVAS]
      .map((c) => c.clientWidth + "x" + c.clientHeight)
      .concat([window.devicePixelRatio.toString()])
      .join(",");
    const resultsChanged = command.version !== this.lastVersion;
    if (!resultsChanged && sizes === this.lastSizes && dark === this.lastDark) return;
    this.lastVersion = command.version;
    this.lastSizes = sizes;
    this.lastDark = dark;

    const result = command.result;
    this.EMPTY.hidden = result !== null;
    if (resultsChanged) {
      this.renderCards(result);
      this.renderBudget(result);
    }
    this.renderSide(result, dark);
    this.renderTop(result, dark);
  }

  private renderCards(result: ShotLabRendererResult | null) {
    this.CARDS.replaceChildren();
    if (result === null) return;
    for (const card of result.cards) {
      const el = document.createElement("div");
      el.classList.add("shotlab-card");
      if (card.kind) el.classList.add(card.kind);
      const label = document.createElement("div");
      label.classList.add("label");
      label.innerText = card.label;
      const value = document.createElement("div");
      value.classList.add("value");
      value.innerText = card.value;
      const detail = document.createElement("div");
      detail.classList.add("detail");
      detail.innerText = card.detail;
      el.append(label, value, detail);
      this.CARDS.appendChild(el);
    }
  }

  private renderBudget(result: ShotLabRendererResult | null) {
    while (this.BUDGET_BODY.childElementCount > 1) {
      this.BUDGET_BODY.removeChild(this.BUDGET_BODY.lastChild as HTMLElement);
    }
    const budget = result?.budget ?? null;
    this.BUDGET_CONTAINER.hidden = budget === null;
    if (budget === null) return;
    for (const row of budget) {
      const tr = document.createElement("tr");
      if (row.summary) tr.classList.add("summary");
      const cells = [
        row.label,
        pct(row.hitRate),
        row.summary ? "" : (row.cost > 0 ? "−" : "") + pct(Math.abs(row.cost)),
        "",
        cm(row.rangeStd) + " / " + cm(row.lateralStd)
      ];
      cells.forEach((text, i) => {
        const td = document.createElement("td");
        if (i === 3 && !row.summary) {
          const bar = document.createElement("div");
          bar.classList.add("shotlab-bar");
          const fill = document.createElement("div");
          fill.style.width = (Math.max(0, Math.min(1, row.share)) * 100).toFixed(1) + "%";
          bar.appendChild(fill);
          const label = document.createElement("span");
          label.innerText = pct(row.share);
          td.append(bar, label);
        } else {
          td.innerText = text;
        }
        tr.appendChild(td);
      });
      this.BUDGET_BODY.appendChild(tr);
    }
  }

  private setupCanvas(canvas: HTMLCanvasElement): { ctx: CanvasRenderingContext2D; w: number; h: number } | null {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w <= 0 || h <= 0) return null;
    const dpr = window.devicePixelRatio;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    const ctx = canvas.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
  }

  private renderSide(result: ShotLabRendererResult | null, dark: boolean) {
    const setup = this.setupCanvas(this.SIDE_CANVAS);
    if (setup === null) return;
    const { ctx, w, h } = setup;
    const colors = palette(dark);
    drawTitle(ctx, "Side view (along the shot)", colors);
    if (result === null) return;
    const side = result.side;

    // Bounds
    let maxX = side.target.far + 0.3;
    let maxY = side.target.top + 0.3;
    const all = side.nominal.concat(...side.samples.map((s) => s.points));
    for (const p of all) {
      maxX = Math.max(maxX, p[0]);
      maxY = Math.max(maxY, p[1]);
    }
    const minX = Math.min(0, side.launch[0]) - 0.2;
    const pad = { l: 44, r: 12, t: 26, b: 26 };
    const sx = (w - pad.l - pad.r) / (maxX - minX);
    const sy = (h - pad.t - pad.b) / (maxY * 1.05);
    const s = Math.min(sx, sy);
    const X = (x: number) => pad.l + (x - minX) * s;
    const Y = (y: number) => h - pad.b - y * s;

    drawGrid(ctx, colors, minX, maxX, 0, maxY * 1.05, X, Y, "m");

    // Floor
    ctx.strokeStyle = colors.axis;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(X(minX), Y(0));
    ctx.lineTo(X(maxX), Y(0));
    ctx.stroke();

    // Target
    ctx.fillStyle = colors.targetFill;
    ctx.strokeStyle = colors.target;
    ctx.lineWidth = 1.5;
    const tx = X(side.target.near);
    const ty = Y(side.target.top);
    const tw = (side.target.far - side.target.near) * s;
    const th = (side.target.top - side.target.bottom) * s;
    ctx.fillRect(tx, ty, tw, th);
    ctx.strokeRect(tx, ty, tw, th);

    // Sample shots
    ctx.lineWidth = 1;
    for (const sample of side.samples) {
      ctx.strokeStyle = sample.hit ? colors.hitFaint : colors.missFaint;
      polyline(ctx, sample.points, X, Y);
    }

    // Nominal
    ctx.strokeStyle = colors.nominal;
    ctx.lineWidth = 2.5;
    polyline(ctx, side.nominal, X, Y);

    // Launch point
    ctx.fillStyle = colors.nominal;
    ctx.beginPath();
    ctx.arc(X(side.launch[0]), Y(side.launch[1]), 4, 0, Math.PI * 2);
    ctx.fill();
  }

  private renderTop(result: ShotLabRendererResult | null, dark: boolean) {
    const setup = this.setupCanvas(this.TOP_CANVAS);
    if (setup === null) return;
    const { ctx, w, h } = setup;
    const colors = palette(dark);
    drawTitle(ctx, "Target plane (top-down)", colors);
    if (result === null) return;
    const top = result.top;

    // Square view centered on the target, big enough for the target and most shots
    let extent = Math.max(top.halfX, top.halfY) * 1.6;
    const dists = top.points.map((p) => Math.max(Math.abs(p.x), Math.abs(p.y))).sort((a, b) => a - b);
    if (dists.length > 0) extent = Math.max(extent, dists[Math.floor((dists.length - 1) * 0.95)] * 1.1);
    const pad = { l: 44, r: 12, t: 26, b: 26 };
    const size = Math.min(w - pad.l - pad.r, h - pad.t - pad.b);
    const s = size / (2 * extent);
    const cx = pad.l + (w - pad.l - pad.r) / 2;
    const cy = pad.t + (h - pad.t - pad.b) / 2;
    const X = (x: number) => cx + x * s;
    const Y = (y: number) => cy - y * s;

    drawGrid(ctx, colors, -extent, extent, -extent, extent, X, Y, "m");

    // Target opening
    ctx.fillStyle = colors.targetFill;
    ctx.strokeStyle = colors.target;
    ctx.lineWidth = 1.5;
    ctx.fillRect(X(-top.halfX), Y(top.halfY), top.halfX * 2 * s, top.halfY * 2 * s);
    ctx.strokeRect(X(-top.halfX), Y(top.halfY), top.halfX * 2 * s, top.halfY * 2 * s);

    // Shots
    for (const p of top.points) {
      if (Math.abs(p.x) > extent || Math.abs(p.y) > extent) continue;
      ctx.fillStyle = p.hit ? colors.hit : colors.miss;
      ctx.beginPath();
      ctx.arc(X(p.x), Y(p.y), 2.5, 0, Math.PI * 2);
      ctx.fill();
    }

    // Mean point of impact
    if (top.mean !== null) {
      ctx.strokeStyle = colors.nominal;
      ctx.lineWidth = 2;
      const mx = X(top.mean[0]);
      const my = Y(top.mean[1]);
      ctx.beginPath();
      ctx.moveTo(mx - 7, my);
      ctx.lineTo(mx + 7, my);
      ctx.moveTo(mx, my - 7);
      ctx.lineTo(mx, my + 7);
      ctx.stroke();
    }

    // Shot direction arrow (from the corner)
    const ax = pad.l + 24;
    const ay = h - pad.b - 24;
    const len = 16;
    const [dx, dy] = top.direction;
    ctx.strokeStyle = colors.text;
    ctx.fillStyle = colors.text;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(ax - dx * len, ay + dy * len);
    ctx.lineTo(ax + dx * len, ay - dy * len);
    ctx.stroke();
    const angle = Math.atan2(-dy, dx);
    ctx.beginPath();
    ctx.moveTo(ax + dx * len, ay - dy * len);
    ctx.lineTo(ax + dx * len - 6 * Math.cos(angle - 0.5), ay - dy * len - 6 * Math.sin(angle - 0.5));
    ctx.lineTo(ax + dx * len - 6 * Math.cos(angle + 0.5), ay - dy * len - 6 * Math.sin(angle + 0.5));
    ctx.closePath();
    ctx.fill();
    ctx.font = "11px 'Inter Variable', sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("shot", ax, ay + 26);
  }
}

// ---------------------------------------------------------------------------------------------

function pct(x: number): string {
  return isFinite(x) ? (x * 100).toFixed(1) + "%" : "—";
}

function cm(x: number): string {
  return isFinite(x) ? (x * 100).toFixed(1) + " cm" : "—";
}

type Palette = ReturnType<typeof palette>;

function palette(dark: boolean) {
  // Seattle Solvers colors: gold for the error-free shot, cream / ink for the target
  return dark
    ? {
        text: "#d8d2ae",
        grid: "rgba(252,249,222,0.07)",
        axis: "rgba(252,249,222,0.4)",
        nominal: "#f2ce00",
        hit: "#3ddc84",
        miss: "#ff5c5c",
        hitFaint: "rgba(61,220,132,0.35)",
        missFaint: "rgba(255,92,92,0.35)",
        target: "#fcf9de",
        targetFill: "rgba(252,249,222,0.1)"
      }
    : {
        text: "#3a3726",
        grid: "rgba(11,10,6,0.07)",
        axis: "rgba(11,10,6,0.45)",
        nominal: "#b99d00",
        hit: "#1a9e55",
        miss: "#d63a3a",
        hitFaint: "rgba(26,158,85,0.35)",
        missFaint: "rgba(214,58,58,0.35)",
        target: "#3a3726",
        targetFill: "rgba(11,10,6,0.07)"
      };
}

function drawTitle(ctx: CanvasRenderingContext2D, title: string, colors: Palette) {
  ctx.fillStyle = colors.text;
  ctx.font = "600 11px 'Azeret Mono', monospace";
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillText(title.toUpperCase(), 10, 18);
}

function niceStep(range: number, targetLines: number): number {
  const raw = range / targetLines;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  return (norm < 1.5 ? 1 : norm < 3.5 ? 2 : norm < 7.5 ? 5 : 10) * mag;
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  colors: Palette,
  x0: number,
  x1: number,
  y0: number,
  y1: number,
  X: (x: number) => number,
  Y: (y: number) => number,
  unit: string
) {
  ctx.strokeStyle = colors.grid;
  ctx.fillStyle = colors.text;
  ctx.lineWidth = 1;
  ctx.font = "10px 'Inter Variable', sans-serif";
  const stepX = niceStep(x1 - x0, 8);
  const stepY = niceStep(y1 - y0, 6);
  const decimals = (step: number) => Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
  const fmt = (v: number, step: number) => (Math.abs(v) < step * 1e-6 ? 0 : v).toFixed(decimals(step));
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let x = Math.ceil(x0 / stepX) * stepX; x <= x1 + 1e-9; x += stepX) {
    ctx.beginPath();
    ctx.moveTo(X(x), Y(y0));
    ctx.lineTo(X(x), Y(y1));
    ctx.stroke();
    ctx.fillText(fmt(x, stepX), X(x), Y(y0) + 4);
  }
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let y = Math.ceil(y0 / stepY) * stepY; y <= y1 + 1e-9; y += stepY) {
    ctx.beginPath();
    ctx.moveTo(X(x0), Y(y));
    ctx.lineTo(X(x1), Y(y));
    ctx.stroke();
    ctx.fillText(fmt(y, stepY) + (Math.abs(y) < 1e-9 ? " " + unit : ""), X(x0) - 4, Y(y));
  }
}

function polyline(
  ctx: CanvasRenderingContext2D,
  points: [number, number][],
  X: (x: number) => number,
  Y: (y: number) => number
) {
  if (points.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(X(points[0][0]), Y(points[0][1]));
  for (let i = 1; i < points.length; i++) ctx.lineTo(X(points[i][0]), Y(points[i][1]));
  ctx.stroke();
}
