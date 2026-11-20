// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { themeColor } from "../themeColors";
import TabRenderer from "./TabRenderer";

export type TelemetryRendererCommand = {
  header: {
    title: string;
    opMode: string | null;
    /** INIT / RUNNING / STOPPED, or null when unknown. */
    state: string | null;
    battery: number | null;
    warning: string | null;
    error: string | null;
    time: number | null;
  };
  /** Driver Station lines, in log order. */
  lines: { label: string; value: string; kind: "number" | "boolean" | "string" }[];
  /** telemetry.addLine() text. */
  freeLines: string[];
  metrics: TelemetryMetric[];
  /** Shown instead of the panels when there is nothing to display. */
  empty: string | null;
};

export type TelemetryMetric =
  | {
      kind: "number";
      key: string;
      label: string;
      value: number;
      /** Recent history (oldest first) for the sparkline. */
      history: number[];
      min: number;
      max: number;
      mean: number;
    }
  | { kind: "boolean"; key: string; label: string; value: boolean; trueFraction: number };

function formatNumber(value: number): string {
  if (!isFinite(value)) return String(value);
  const abs = Math.abs(value);
  if (abs >= 10000) return value.toFixed(0);
  if (abs >= 100) return value.toFixed(1);
  if (abs >= 1) return value.toFixed(2);
  if (abs === 0) return "0";
  return value.toFixed(3);
}

export default class TelemetryRenderer implements TabRenderer {
  private ROOT: HTMLElement;
  private EMPTY: HTMLElement;
  private BODY: HTMLElement;
  private DS_TITLE: HTMLElement;
  private DS_OPMODE: HTMLElement;
  private DS_STATE: HTMLElement;
  private DS_BATTERY: HTMLElement;
  private DS_TIME: HTMLElement;
  private DS_ALERTS: HTMLElement;
  private DS_LINES: HTMLElement;
  private GRID: HTMLElement;

  private cards = new Map<
    string,
    {
      root: HTMLElement;
      label: HTMLElement;
      value: HTMLElement;
      canvas: HTMLCanvasElement | null;
      stats: HTMLElement;
      lastDraw: string;
    }
  >();
  private lastLines = "";

  constructor(root: HTMLElement) {
    this.ROOT = root;
    const q = (cls: string) => root.getElementsByClassName(cls)[0] as HTMLElement;
    this.EMPTY = q("telemetry-empty");
    this.BODY = q("telemetry-body");
    this.DS_TITLE = q("ds-title");
    this.DS_OPMODE = q("ds-opmode");
    this.DS_STATE = q("ds-state");
    this.DS_BATTERY = q("ds-battery");
    this.DS_TIME = q("ds-time");
    this.DS_ALERTS = q("ds-alerts");
    this.DS_LINES = q("ds-lines");
    this.GRID = q("telemetry-grid");
  }

  saveState(): unknown {
    return null;
  }

  restoreState(): void {}

  getAspectRatio(): number | null {
    return null;
  }

  render(command: TelemetryRendererCommand): void {
    if (command === null || command === undefined) return;
    this.EMPTY.hidden = command.empty === null;
    this.BODY.hidden = command.empty !== null;
    if (command.empty !== null) {
      this.EMPTY.innerText = command.empty;
      return;
    }
    this.renderHeader(command);
    this.renderLines(command);
    this.renderMetrics(command);
  }

  private renderHeader(command: TelemetryRendererCommand) {
    const h = command.header;
    setText(this.DS_TITLE, h.title);
    setText(this.DS_OPMODE, h.opMode ?? "No OpMode");
    const state = h.state ?? "—";
    setText(this.DS_STATE, state);
    this.DS_STATE.className = "ds-state " + state.toLowerCase();
    setText(this.DS_BATTERY, h.battery === null ? "— V" : h.battery.toFixed(2) + " V");
    this.DS_BATTERY.classList.toggle("low", h.battery !== null && h.battery < 12);
    setText(this.DS_TIME, h.time === null ? "" : "t = " + h.time.toFixed(2) + " s");

    const alerts = JSON.stringify([h.warning, h.error]);
    if (this.DS_ALERTS.dataset.last !== alerts) {
      this.DS_ALERTS.dataset.last = alerts;
      this.DS_ALERTS.replaceChildren();
      for (const [text, kind] of [
        [h.error, "error"],
        [h.warning, "warning"]
      ] as const) {
        if (!text) continue;
        const el = document.createElement("div");
        el.classList.add("ds-alert", kind);
        el.innerText = text;
        this.DS_ALERTS.appendChild(el);
      }
    }
  }

  private renderLines(command: TelemetryRendererCommand) {
    const key = JSON.stringify([command.lines, command.freeLines]);
    if (key === this.lastLines) return;
    this.lastLines = key;
    this.DS_LINES.replaceChildren();
    for (const line of command.lines) {
      const row = document.createElement("div");
      row.classList.add("ds-line", line.kind);
      const label = document.createElement("span");
      label.classList.add("ds-key");
      label.innerText = line.label;
      const value = document.createElement("span");
      value.classList.add("ds-value");
      value.innerText = line.value;
      row.append(label, value);
      this.DS_LINES.appendChild(row);
    }
    for (const text of command.freeLines) {
      const row = document.createElement("div");
      row.classList.add("ds-line", "free");
      row.innerText = text;
      this.DS_LINES.appendChild(row);
    }
    if (command.lines.length === 0 && command.freeLines.length === 0) {
      const row = document.createElement("div");
      row.classList.add("ds-line", "free", "dim");
      row.innerText = "No telemetry at this time.";
      this.DS_LINES.appendChild(row);
    }
  }

  private renderMetrics(command: TelemetryRendererCommand) {
    const seen = new Set<string>();
    command.metrics.forEach((metric, index) => {
      seen.add(metric.key);
      let card = this.cards.get(metric.key);
      if (card === undefined || (card.canvas === null) !== (metric.kind === "boolean")) {
        card?.root.remove();
        card = this.makeCard(metric.kind);
        this.cards.set(metric.key, card);
      }
      // Keep DOM order matching the command
      if (this.GRID.children[index] !== card.root) {
        this.GRID.insertBefore(card.root, this.GRID.children[index] ?? null);
      }
      card.root.title = metric.key;
      setText(card.label, metric.label);
      if (metric.kind === "number") {
        setText(card.value, formatNumber(metric.value));
        setText(
          card.stats,
          "min " +
            formatNumber(metric.min) +
            " · avg " +
            formatNumber(metric.mean) +
            " · max " +
            formatNumber(metric.max)
        );
        this.drawSparkline(card, metric.history);
      } else {
        setText(card.value, metric.value ? "TRUE" : "FALSE");
        card.value.classList.toggle("true", metric.value);
        card.value.classList.toggle("false", !metric.value);
        setText(card.stats, "true " + Math.round(metric.trueFraction * 100) + "% of the window");
      }
    });
    for (const [key, card] of this.cards) {
      if (!seen.has(key)) {
        card.root.remove();
        this.cards.delete(key);
      }
    }
  }

  private makeCard(kind: "number" | "boolean") {
    const root = document.createElement("div");
    root.classList.add("telemetry-card", kind);
    const label = document.createElement("div");
    label.classList.add("label");
    const value = document.createElement("div");
    value.classList.add("value");
    let canvas: HTMLCanvasElement | null = null;
    root.append(label, value);
    if (kind === "number") {
      canvas = document.createElement("canvas");
      canvas.classList.add("spark");
      root.appendChild(canvas);
    }
    const stats = document.createElement("div");
    stats.classList.add("stats");
    root.appendChild(stats);
    return { root, label, value, canvas, stats, lastDraw: "" };
  }

  private drawSparkline(card: { canvas: HTMLCanvasElement | null; lastDraw: string }, history: number[]) {
    const canvas = card.canvas;
    if (canvas === null) return;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w <= 0 || h <= 0) return;
    const accent = themeColor("--ss-accent-line", "#f2ce00");
    const drawKey = w + "x" + h + accent + history.join(",");
    if (drawKey === card.lastDraw) return;
    card.lastDraw = drawKey;
    const dpr = window.devicePixelRatio;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    const ctx = canvas.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    if (history.length < 2) return;
    let min = Math.min(...history);
    let max = Math.max(...history);
    if (max - min < 1e-9) {
      min -= 1;
      max += 1;
    }
    const X = (i: number) => (i / (history.length - 1)) * (w - 2) + 1;
    const Y = (v: number) => h - 3 - ((v - min) / (max - min)) * (h - 6);
    // Fill
    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, hexToRgba(accent, 0.28));
    grad.addColorStop(1, hexToRgba(accent, 0));
    ctx.beginPath();
    ctx.moveTo(X(0), h);
    history.forEach((v, i) => ctx.lineTo(X(i), Y(v)));
    ctx.lineTo(X(history.length - 1), h);
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();
    // Line
    ctx.beginPath();
    history.forEach((v, i) => (i === 0 ? ctx.moveTo(X(i), Y(v)) : ctx.lineTo(X(i), Y(v))));
    ctx.strokeStyle = accent;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    // Current value dot
    ctx.beginPath();
    ctx.arc(X(history.length - 1), Y(history[history.length - 1]), 2.5, 0, Math.PI * 2);
    ctx.fillStyle = accent;
    ctx.fill();
  }
}

/** textContent (not innerText) so frame-by-frame updates never force a layout. */
function setText(el: HTMLElement, text: string) {
  if (el.textContent !== text) el.textContent = text;
}

function hexToRgba(color: string, alpha: number): string {
  const m = /^#([0-9a-f]{6})$/i.exec(color.trim());
  if (!m) return color;
  const n = parseInt(m[1], 16);
  return "rgba(" + ((n >> 16) & 255) + "," + ((n >> 8) & 255) + "," + (n & 255) + "," + alpha + ")";
}
