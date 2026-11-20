// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { AutoCheckResult, describeMouthOffset, describeTune } from "../shared/physics/AutoShotCheck";

/** The latest Auto Check, so the Telemetry tab's Markdown report can include it. */
export interface StoredAutoCheck {
  /** Identifies the log the check was run on. */
  logKey: string;
  result: AutoCheckResult;
  errorSources: string[];
  spinLabel: string;
  samples: number;
}

let latest: StoredAutoCheck | null = null;

export function setLatestAutoCheck(check: StoredAutoCheck | null) {
  latest = check;
}

export function getLatestAutoCheck(): StoredAutoCheck | null {
  return latest;
}

/** Markdown section for the telemetry report. */
export function autoCheckMarkdown(check: StoredAutoCheck): string {
  const r = check.result;
  const n = r.shots.length;
  const pct = (x: number) => (isFinite(x) ? Math.round(x * 100) + "%" : "—");
  const lines: string[] = [];
  lines.push("## Auto Check: shots re-flown with Magnus and match error");
  lines.push("");
  lines.push("| | POLLEN in |");
  lines.push("|---|---:|");
  lines.push("| Robot-code sim (no error) | " + r.robotSimIn + " / " + n + " |");
  lines.push("| Re-flown, no spin | " + r.noSpinIn + " / " + n + " |");
  lines.push("| Re-flown with Magnus (" + check.spinLabel + ") | " + r.spinIn + " / " + n + " |");
  lines.push(
    "| With match error (" +
      check.samples +
      " tries per shot) | " +
      r.expectedIn.toFixed(1) +
      " expected, 90% range " +
      r.range90[0] +
      "–" +
      r.range90[1] +
      " |"
  );
  if (r.tunedExpectedIn > r.expectedIn + 0.05) {
    lines.push(
      "| With tuned exit speeds (Tune column) | " +
        r.tunedExpectedIn.toFixed(1) +
        " expected, 90% range " +
        r.tunedRange90[0] +
        "–" +
        r.tunedRange90[1] +
        " |"
    );
  }
  lines.push("");
  lines.push("Error sources: " + (check.errorSources.length > 0 ? check.errorSources.join(", ") : "none") + ".");
  lines.push("");
  lines.push(
    "| # | Time | CELL | Exit speed | Angle | Robot speed | Robot sim | No spin | Magnus | Spin moves it (up the mouth) | Crosses the mouth | Chance in | Tune |"
  );
  lines.push("|---:|---:|---|---:|---:|---:|---|---|---|---:|---|---:|---|");
  r.shots.forEach((s, i) => {
    const cell = s.shot.cell === null ? "—" : s.shot.cell.alliance + " " + s.shot.cell.hiveCell;
    const robotSpeed = Math.hypot(s.shot.robotVelocity[0], s.shot.robotVelocity[1]);
    lines.push(
      "| " +
        (i + 1) +
        " | " +
        s.shot.time.toFixed(2) +
        " s | " +
        cell +
        " | " +
        s.exitSpeed.toFixed(2) +
        " m/s | " +
        s.elevationDeg.toFixed(1) +
        "° | " +
        robotSpeed.toFixed(2) +
        " m/s | " +
        s.shot.robotSimResult +
        " | " +
        (s.noSpinHit ? "in" : "miss") +
        " | " +
        (s.spinHit ? "in" : "miss") +
        " | " +
        (isFinite(s.magnusShift) ? (s.magnusShift * 100).toFixed(1) + " cm" : "—") +
        " | " +
        describeMouthOffset(s.mouthOffset, s.mouthHalfHeight) +
        " | " +
        pct(s.pIn) +
        " | " +
        describeTune(s.exitSpeed, s.tunedExitSpeed, s.pIn, s.tunedPIn) +
        " |"
    );
  });
  lines.push("");
  return lines.join("\n");
}
