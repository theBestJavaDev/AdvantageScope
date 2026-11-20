// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { AutoCheckResult, describeMouthOffset, describeTune } from "../physics/AutoShotCheck";
import TabRenderer from "./TabRenderer";

export type AutoCheckRendererCommand = {
  status: { text: string; kind: "info" | "busy" | "ok" | "warn" };
  result: AutoCheckResult | null;
  errorSources: string[];
  spinLabel: string;
  /** The random match shown on the 3D field (missed shots as red balls), or null before the first check. */
  match?: { inCount: number; total: number; missed: number[] } | null;
};

function pct(x: number): string {
  return isFinite(x) ? Math.round(x * 100) + "%" : "—";
}

export default class AutoCheckRenderer implements TabRenderer {
  private STATUS: HTMLElement;
  private CARDS: HTMLElement;
  private DIST: HTMLElement;
  private ERRORS: HTMLElement;
  private TABLE: HTMLElement;
  private EMPTY: HTMLElement;
  private lastKey = "";

  constructor(root: HTMLElement) {
    const q = (cls: string) => root.getElementsByClassName(cls)[0] as HTMLElement;
    this.STATUS = q("autocheck-status");
    this.CARDS = q("autocheck-cards");
    this.DIST = q("autocheck-dist");
    this.ERRORS = q("autocheck-errors");
    this.TABLE = q("autocheck-table");
    this.EMPTY = q("autocheck-empty");
  }

  saveState(): unknown {
    return null;
  }

  restoreState(): void {}

  getAspectRatio(): number | null {
    return null;
  }

  render(command: AutoCheckRendererCommand): void {
    if (command === null || command === undefined) return;
    const key = JSON.stringify([
      command.status,
      command.result?.durationMs,
      command.result?.expectedIn,
      command.spinLabel,
      command.match
    ]);
    if (key === this.lastKey) return;
    this.lastKey = key;

    this.STATUS.textContent = command.status.text;
    this.STATUS.className = "autocheck-status shotlab-status " + command.status.kind;
    const r = command.result;
    this.EMPTY.hidden = r !== null;
    this.CARDS.replaceChildren();
    this.DIST.replaceChildren();
    this.TABLE.replaceChildren();
    this.ERRORS.textContent = "";
    if (r === null) return;
    const n = r.shots.length;

    // Summary cards: perfect robot sim -> + Magnus -> + match error
    const card = (label: string, value: string, detail: string, kind = "") => {
      const el = document.createElement("div");
      el.className = "shotlab-card " + kind;
      const l = document.createElement("div");
      l.className = "label";
      l.textContent = label;
      const v = document.createElement("div");
      v.className = "value";
      v.textContent = value;
      const d = document.createElement("div");
      d.className = "detail";
      d.textContent = detail;
      el.append(l, v, d);
      this.CARDS.appendChild(el);
    };
    card("Robot-code sim", r.robotSimIn + " / " + n, "perfect robot, as logged");
    card("Re-flown, no spin", r.noSpinIn + " / " + n, "SolversScope physics, drag only");
    card(
      "With Magnus",
      r.spinIn + " / " + n,
      command.spinLabel,
      r.spinIn < r.noSpinIn ? "bad" : r.spinIn > r.noSpinIn ? "good" : ""
    );
    card(
      "With match error",
      r.expectedIn.toFixed(1) + " / " + n,
      "expected · 90% range " + r.range90[0] + "–" + r.range90[1],
      r.expectedIn >= 0.85 * n ? "good" : r.expectedIn < 0.6 * n ? "bad" : ""
    );
    if (command.match) {
      const m = command.match;
      card(
        "This random match",
        m.inCount + " / " + m.total,
        m.missed.length === 0
          ? "every shot went in this time · 🎲 to roll another"
          : "missed #" + m.missed.join(", #") + " · red trails on the 3D field",
        m.missed.length === 0 ? "good" : ""
      );
    }
    if (r.tunedExpectedIn > r.expectedIn + 0.05) {
      card(
        "With tuned speeds",
        r.tunedExpectedIn.toFixed(1) + " / " + n,
        "+" + (r.tunedExpectedIn - r.expectedIn).toFixed(1) + " · each shot centered in the mouth (Tune column)",
        "good"
      );
    }

    // Distribution of POLLEN scored
    const peak = Math.max(...r.distribution);
    const title = document.createElement("div");
    title.className = "autocheck-dist-title";
    title.textContent = "POLLEN in, with match error";
    this.DIST.appendChild(title);
    const bars = document.createElement("div");
    bars.className = "autocheck-bars";
    r.distribution.forEach((p, k) => {
      const col = document.createElement("div");
      col.className = "autocheck-bar";
      col.title = k + " in: " + (p * 100).toFixed(1) + "%";
      const fill = document.createElement("div");
      fill.className = "fill" + (k >= r.range90[0] && k <= r.range90[1] ? " in-range" : "");
      fill.style.height = (peak > 0 ? (p / peak) * 100 : 0).toFixed(1) + "%";
      const label = document.createElement("div");
      label.className = "k";
      label.textContent = String(k);
      col.append(fill, label);
      bars.appendChild(col);
    });
    this.DIST.appendChild(bars);

    this.ERRORS.textContent =
      "Error sources (from Shot Lab): " + (command.errorSources.length > 0 ? command.errorSources.join(" · ") : "none");

    // One-line takeaway: where the shots cross the mouth on average and the speed change that centers them
    const crossing = r.shots.filter((s) => isFinite(s.mouthOffset));
    const tuned = r.shots.filter((s) => isFinite(s.tunedExitSpeed));
    if (crossing.length > 0 && tuned.length > 0 && r.tunedExpectedIn > r.expectedIn + 0.05) {
      const avgOffset = crossing.reduce((a, s) => a + s.mouthOffset, 0) / crossing.length;
      const avgChange = tuned.reduce((a, s) => a + (s.tunedExitSpeed - s.exitSpeed) / s.exitSpeed, 0) / tuned.length;
      const low = crossing.filter((s) => s.mouthOffset < 0).length;
      const where =
        low === crossing.length
          ? "All " + low + " shots cross the mouth low"
          : low === 0
          ? "All " + crossing.length + " shots cross the mouth high"
          : low + " of " + crossing.length + " shots cross the mouth low";
      const hint = document.createElement("div");
      hint.className = "autocheck-hint";
      hint.textContent =
        where +
        " (average " +
        (avgOffset >= 0 ? "+" : "") +
        (avgOffset * 100).toFixed(1) +
        " cm from the middle). Exit speed " +
        (avgChange >= 0 ? "+" : "") +
        (avgChange * 100).toFixed(1) +
        "% on average centers them: " +
        r.expectedIn.toFixed(1) +
        " → " +
        r.tunedExpectedIn.toFixed(1) +
        " expected with match error.";
      this.TABLE.appendChild(hint);
    }

    // Per-shot table
    const table = document.createElement("table");
    table.className = "autocheck-shots";
    const head = table.createTHead().insertRow();
    [
      "#",
      "Time",
      "CELL",
      "Exit",
      "Angle",
      "Robot",
      "Robot sim",
      "No spin",
      "Magnus",
      "Spin moves it (up the mouth)",
      "Crosses the mouth",
      "Chance in",
      "Tune"
    ].forEach((h) => {
      const th = document.createElement("th");
      th.textContent = h;
      head.appendChild(th);
    });
    const body = table.createTBody();
    r.shots.forEach((s, i) => {
      const row = body.insertRow();
      const cells: (string | HTMLElement)[] = [
        String(i + 1),
        s.shot.time.toFixed(2) + " s",
        s.shot.cell === null ? "—" : s.shot.cell.alliance + " " + s.shot.cell.hiveCell,
        s.exitSpeed.toFixed(2) + " m/s",
        s.elevationDeg.toFixed(1) + "°",
        Math.hypot(...s.shot.robotVelocity).toFixed(2) + " m/s",
        s.shot.robotSimResult.toUpperCase(),
        s.noSpinHit ? "IN" : "MISS",
        s.spinHit ? "IN" : "MISS",
        isFinite(s.magnusShift) ? (s.magnusShift >= 0 ? "+" : "") + (s.magnusShift * 100).toFixed(1) + " cm" : "—",
        describeMouthOffset(s.mouthOffset, s.mouthHalfHeight)
      ];
      cells.forEach((c, j) => {
        const td = row.insertCell();
        td.textContent = c as string;
        if (j >= 6 && j <= 8) td.className = c === "IN" ? "in" : c === "MISS" ? "miss" : "";
      });
      const chance = row.insertCell();
      const bar = document.createElement("div");
      bar.className = "shotlab-bar";
      const fill = document.createElement("div");
      fill.style.width = (isFinite(s.pIn) ? s.pIn * 100 : 0).toFixed(1) + "%";
      bar.appendChild(fill);
      const text = document.createElement("span");
      text.textContent = pct(s.pIn);
      chance.append(bar, text);
      const tune = row.insertCell();
      tune.textContent = describeTune(s.exitSpeed, s.tunedExitSpeed, s.pIn, s.tunedPIn);
      tune.className = tune.textContent.includes("→") ? "tune" : "";
    });
    this.TABLE.appendChild(table);
  }
}
