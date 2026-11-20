// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { AutoCheckOptions, AutoCheckResult, AutoShot } from "../../shared/physics/AutoShotCheck";
import { simToLogTranslation } from "../../shared/physics/GamePresets";
import { ERROR_SOURCES } from "../../shared/physics/ShotErrorModel";
import { SpinMode } from "../../shared/physics/ShotLaunch";
import { AutoCheckRendererCommand } from "../../shared/renderers/AutoCheckRenderer";
import { createUUID } from "../../shared/util";
import { setLatestAutoCheck } from "../autoCheckStore";
import { extractAutoShots, isRobotSimLog } from "../autoShotExtract";
import WorkerManager from "../WorkerManager";
import { getSharedShotLabConfig } from "./SimForm";
import TabController from "./TabController";

const LOG_PREFIX = "/SolversSim/AutoCheck";
const AUTO_CHECK_FIELDS = [
  "RealOutputs/Sim/Launch",
  "RealOutputs/Sim/Pollen/transitions",
  "RealOutputs/Sim/Hive",
  "RealOutputs/Field/Pollen",
  "RealOutputs/Field/Robot"
];

/** True while any field the extraction needs is still being decoded from the log. */
function stillLoading(): boolean {
  for (const field of window.getLoadingFields()) {
    if (AUTO_CHECK_FIELDS.some((prefix) => field === prefix || field.startsWith(prefix + "/"))) return true;
  }
  return false;
}

type State = { samples: number; spin: "backspin" | "topspin" | "none" };

/**
 * Auto Check tab: whenever a robot-code sim log (BioBuzz SimLoop) is open, re-flies every shot the
 * robot fired with Magnus and with the Shot Lab error model, automatically, and shows the hit
 * chance per shot and the POLLEN scored distribution.
 */
export default class AutoCheckController implements TabController {
  UUID = createUUID();

  private SAMPLES: HTMLSelectElement;
  private SPIN: HTMLSelectElement;
  private RERUN: HTMLButtonElement;
  private state: State = { samples: 200, spin: "backspin" };

  private command: AutoCheckRendererCommand = {
    status: { text: "Open a robot-code sim log (simAuto) to check its shots.", kind: "info" },
    result: null,
    errorSources: [],
    spinLabel: ""
  };
  private checkedKey: string | null = null;
  private running = false;
  private pending: number | null = null;
  /** Retries while the log is still decoding (fields load on demand, after the tab asks). */
  private retries = 0;
  private retryKey: string | null = null;

  constructor(root: HTMLElement) {
    this.SAMPLES = root.getElementsByClassName("autocheck-samples")[0] as HTMLSelectElement;
    this.SPIN = root.getElementsByClassName("autocheck-spin")[0] as HTMLSelectElement;
    this.RERUN = root.getElementsByClassName("autocheck-rerun")[0] as HTMLButtonElement;
    this.SAMPLES.addEventListener("change", () => {
      this.state.samples = Number(this.SAMPLES.value);
      this.run(true);
    });
    this.SPIN.addEventListener("change", () => {
      this.state.spin = this.SPIN.value as State["spin"];
      this.run(true);
    });
    this.RERUN.addEventListener("click", () => this.run(true));
    this.syncInputs();
    this.schedule();
  }

  private syncInputs() {
    this.SAMPLES.value = this.state.samples.toString();
    this.SPIN.value = this.state.spin;
  }

  /** Identifies the open log, so the check runs once per log (and again when it changes). */
  private logKey(): string {
    const counts = window.log.getNumber("RealOutputs/Sim/Launch/count", -Infinity, Infinity);
    const range = window.log.getTimestampRange();
    return document.title + "|" + (counts?.values.length ?? 0) + "|" + range[0].toFixed(3) + "|" + range[1].toFixed(3);
  }

  private schedule(delayMs = 600) {
    if (this.pending !== null) window.clearTimeout(this.pending);
    this.pending = window.setTimeout(() => {
      this.pending = null;
      this.run(false);
    }, delayMs);
  }

  /** How many launches the robot sim counted (its final Sim/Launch/count). */
  private expectedLaunches(): number {
    const counts = window.log.getNumber("RealOutputs/Sim/Launch/count", -Infinity, Infinity);
    return counts === undefined || counts.values.length === 0 ? 0 : counts.values[counts.values.length - 1];
  }

  private setStatus(text: string, kind: AutoCheckRendererCommand["status"]["kind"]) {
    this.command = { ...this.command, status: { text, kind } };
  }

  private run(force: boolean) {
    if (this.running) return;
    if (!isRobotSimLog()) {
      if (this.checkedKey !== null) {
        this.checkedKey = null;
        this.command = { ...this.command, result: null };
        setLatestAutoCheck(null);
      }
      this.setStatus("Open a robot-code sim log (simAuto) to check its shots.", "info");
      return;
    }
    const key = this.logKey();
    if (!force && key === this.checkedKey) return;

    // Fields decode on demand: wait until the ones the extraction reads are in
    if (stillLoading()) {
      this.setStatus("Loading the shots from the log…", "busy");
      this.schedule();
      return;
    }
    const shots: AutoShot[] = extractAutoShots();
    // The launch count can arrive before the ball positions; wait (up to ~20 s) until every
    // counted launch can be found, then check whatever is there
    if (this.retryKey !== key) {
      this.retryKey = key;
      this.retries = 0;
    }
    const expected = this.expectedLaunches();
    if (shots.length < expected && this.retries < 20) {
      this.retries++;
      this.setStatus("Loading the shots from the log… (" + shots.length + " of " + expected + ")", "busy");
      this.schedule(1000);
      return;
    }
    if (shots.length === 0) {
      this.setStatus("No launches found in this log.", "info");
      return;
    }
    const config = getSharedShotLabConfig();
    const errorSources =
      config.error.preset === "zero"
        ? []
        : ERROR_SOURCES.filter((s) => config.error.enabled[s.key]).map((s) => s.label);
    const options: Partial<AutoCheckOptions> = {
      samples: this.state.samples,
      spinMode: this.state.spin as SpinMode,
      spinRpm: "rolling"
    };
    const spinLabel = this.state.spin === "none" ? "no spin" : this.state.spin + " from the hood (rolling)";
    this.running = true;
    this.RERUN.disabled = true;
    this.setStatus(
      "Re-flying " +
        shots.length +
        " shots × " +
        this.state.samples +
        " with Magnus and " +
        errorSources.length +
        " error sources…",
      "busy"
    );
    WorkerManager.request("../bundles/shotLabWorker.js", { kind: "autoCheck", config, shots, options })
      .then((result: AutoCheckResult) => {
        this.checkedKey = key;
        this.command = {
          status: {
            text:
              "Checked " +
              shots.length +
              " shots in " +
              (result.durationMs / 1000).toFixed(1) +
              " s. Error settings come from the Shot Lab tab; results are also in the sidebar under " +
              LOG_PREFIX +
              " and in the Telemetry report.",
            kind: "ok"
          },
          result,
          errorSources,
          spinLabel
        };
        setLatestAutoCheck({ logKey: key, result, errorSources, spinLabel, samples: this.state.samples });
        this.writeToLog(result);
      })
      .catch(() => this.setStatus("The check failed. Try Re-run.", "warn"))
      .finally(() => {
        this.running = false;
        this.RERUN.disabled = false;
      });
  }

  /** Per-shot chance at each launch time + the totals, so graphs and the table can show them. */
  private writeToLog(result: AutoCheckResult) {
    const log = window.log;
    log.deleteFieldsWithPrefix(LOG_PREFIX);
    result.shots.forEach((s) => {
      log.putNumber(LOG_PREFIX + "/ShotChanceIn", s.shot.time, isFinite(s.pIn) ? s.pIn : 0);
      log.putBoolean(LOG_PREFIX + "/ShotInWithMagnus", s.shot.time, s.spinHit);
      log.putNumber(LOG_PREFIX + "/ShotMagnusShiftMeters", s.shot.time, isFinite(s.magnusShift) ? s.magnusShift : 0);
      if (isFinite(s.mouthOffset)) log.putNumber(LOG_PREFIX + "/ShotMouthOffsetMeters", s.shot.time, s.mouthOffset);
      if (isFinite(s.tunedExitSpeed)) {
        log.putNumber(LOG_PREFIX + "/ShotTunedExitSpeed", s.shot.time, s.tunedExitSpeed);
        log.putNumber(LOG_PREFIX + "/ShotTunedChanceIn", s.shot.time, s.tunedPIn);
      }
    });
    const t = result.shots.length > 0 ? result.shots[0].shot.time : log.getTimestampRange()[0];
    log.putNumber(LOG_PREFIX + "/ExpectedIn", t, result.expectedIn);
    log.putNumber(LOG_PREFIX + "/TunedExpectedIn", t, result.tunedExpectedIn);
    log.putNumber(LOG_PREFIX + "/Range90Low", t, result.range90[0]);
    log.putNumber(LOG_PREFIX + "/Range90High", t, result.range90[1]);
    log.putNumber(LOG_PREFIX + "/RobotSimIn", t, result.robotSimIn);
    log.putNumber(LOG_PREFIX + "/InWithMagnus", t, result.spinIn);
    // Launch points in the field log frame, for the 3D / 2D field views
    log.putTranslation3dArray(
      LOG_PREFIX + "/LaunchPoints",
      t,
      result.shots.map((s) => simToLogTranslation([...s.shot.position]))
    );
    window.sidebar.refresh();
  }

  saveState(): unknown {
    return this.state;
  }

  restoreState(state: unknown): void {
    if (typeof state !== "object" || state === null) return;
    const s = state as Partial<State>;
    if (typeof s.samples === "number" && s.samples >= 10) this.state.samples = s.samples;
    if (s.spin === "backspin" || s.spin === "topspin" || s.spin === "none") this.state.spin = s.spin;
    this.syncInputs();
  }

  refresh(): void {
    this.schedule();
  }

  newAssets(): void {}

  getActiveFields(): string[] {
    // Prefixes (matched as key or key + "/"): only what the shot extraction reads
    return AUTO_CHECK_FIELDS;
  }

  showTimeline(): boolean {
    return true;
  }

  getCommand(): AutoCheckRendererCommand {
    return this.command;
  }
}
