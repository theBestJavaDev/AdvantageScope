// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { BIOBUZZ, GAME_PRESETS, getGamePreset, mirrorCell } from "../../shared/physics/GamePresets";
import { fitDragCoefficient, fitRestitution } from "../../shared/physics/ShotCalibration";
import { ERROR_SOURCES } from "../../shared/physics/ShotErrorModel";
import {
  SHOT_LAB_LOG_PREFIX,
  ShotLabConfig,
  ShotLabRun,
  applyGamePreset,
  buildGamePiece,
  defaultShotLabConfig,
  mergeShotLabConfig,
  minimumShotDistance,
  placeRobotForCell,
  solveShot,
  writeShotLabToLog
} from "../../shared/physics/ShotLab";
import { SPIN_MODES } from "../../shared/physics/ShotLaunch";
import { ShotLabRendererCommand, ShotLabRendererResult } from "../../shared/renderers/ShotLabRenderer";
import { createUUID } from "../../shared/util";
import WorkerManager from "../WorkerManager";
import { FieldSpec, FormSection, SimForm, publishShotLabConfig } from "./SimForm";
import TabController from "./TabController";

const SPIN_LABELS: Record<string, string> = {
  none: "None",
  backspin: "Backspin",
  topspin: "Topspin",
  "sidespin-left": "Sidespin (curve left)",
  "sidespin-right": "Sidespin (curve right)"
};

const SECTIONS: FormSection[] = [
  {
    title: "Game",
    fields: [
      {
        kind: "select",
        path: "game.preset",
        label: "Season",
        options: [
          ...GAME_PRESETS.map((p) => ({ value: p.id, label: p.name + " " + p.season })),
          { value: "custom", label: "Custom target" }
        ],
        hint: BIOBUZZ.source
      },
      {
        kind: "select",
        path: "game.alliance",
        label: "Alliance",
        options: [
          { value: "red", label: "Red" },
          { value: "blue", label: "Blue" }
        ]
      },
      {
        kind: "select",
        path: "game.upCell",
        label: "HIVE CELL facing up",
        options: [
          { value: "rear", label: "Rear (away from audience)" },
          { value: "audience", label: "Audience side" }
        ],
        hint: "Each HIVE is bi-stable: launch into whichever CELL currently faces up"
      },
      {
        kind: "select",
        path: "game.piece",
        label: "Game piece",
        options: BIOBUZZ.pieces.map((p) => ({ value: p.id, label: p.name }))
      }
    ]
  },
  {
    title: "Shooter",
    fields: [
      { kind: "number", path: "shooter.exitSpeed", label: "Exit speed", unit: "m/s", step: 0.05, min: 0 },
      { kind: "number", path: "shooter.hoodAngleDeg", label: "Hood angle", unit: "°", step: 0.5, min: 0, max: 90 },
      { kind: "number", path: "shooter.turretYawDeg", label: "Turret yaw", unit: "°", step: 0.5 },
      {
        kind: "select",
        path: "shooter.spinMode",
        label: "Spin",
        options: SPIN_MODES.map((mode) => ({ value: mode, label: SPIN_LABELS[mode] }))
      },
      { kind: "number", path: "shooter.spinRpm", label: "Spin rate", unit: "RPM", step: 50, min: 0 },
      { kind: "number", path: "shooter.launchHeight", label: "Launch height", unit: "m", step: 0.01, min: 0 },
      { kind: "number", path: "shooter.offsetX", label: "Exit ahead of center", unit: "m", step: 0.01 },
      { kind: "checkbox", path: "shooter.inheritRobotVelocity", label: "Add robot velocity to shot" }
    ]
  },
  {
    title: "Robot",
    fields: [
      { kind: "number", path: "robot.x", label: "X", unit: "m", step: 0.05, min: -1.83, max: 1.83 },
      { kind: "number", path: "robot.y", label: "Y", unit: "m", step: 0.05, min: -1.83, max: 1.83 },
      { kind: "number", path: "robot.headingDeg", label: "Heading", unit: "°", step: 1 },
      { kind: "number", path: "robot.vx", label: "Velocity X", unit: "m/s", step: 0.1 },
      { kind: "number", path: "robot.vy", label: "Velocity Y", unit: "m/s", step: 0.1 },
      { kind: "number", path: "robot.omegaDegPerSec", label: "Turn rate", unit: "°/s", step: 5 },
      { kind: "checkbox", path: "autoAim", label: "Solve also aims the robot (leads when moving)" }
    ]
  },
  {
    title: "Target",
    collapsed: true,
    fields: [
      { kind: "number", path: "target.x", label: "Center X", unit: "m", step: 0.05 },
      { kind: "number", path: "target.y", label: "Center Y", unit: "m", step: 0.05 },
      { kind: "number", path: "target.z", label: "Center height", unit: "m", step: 0.01, min: 0 },
      { kind: "number", path: "target.sizeX", label: "Opening X", unit: "m", step: 0.01, min: 0.01 },
      { kind: "number", path: "target.sizeY", label: "Opening Y", unit: "m", step: 0.01, min: 0.01 },
      { kind: "number", path: "target.depth", label: "Depth", unit: "m", step: 0.01, min: 0.01 },
      { kind: "checkbox", path: "target.requireDescending", label: "Must enter falling" }
    ]
  },
  {
    title: "Game piece",
    collapsed: true,
    fields: [
      { kind: "number", path: "piece.massKg", label: "Mass", unit: "kg", step: 0.001, min: 0.001 },
      { kind: "number", path: "piece.diameterM", label: "Diameter", unit: "m", step: 0.001, min: 0.01 },
      { kind: "number", path: "piece.dragCoefficient", label: "Drag coeff.", unit: "Cd", step: 0.01, min: 0 },
      { kind: "number", path: "piece.magnusScale", label: "Magnus scale", unit: "×", step: 0.05, min: 0 },
      { kind: "number", path: "piece.restitution", label: "Bounciness", unit: "e", step: 0.05, min: 0, max: 1 }
    ]
  },
  {
    title: "Error sources",
    fields: [
      {
        kind: "select",
        path: "error.preset",
        label: "Values",
        options: [
          { value: "typical", label: "Typical estimates" },
          { value: "zero", label: "None (perfect robot)" }
        ]
      },
      ...ERROR_SOURCES.map(
        (source): FieldSpec => ({
          kind: "checkbox",
          path: "error.enabled." + source.key,
          label: source.label,
          hint: source.description
        })
      )
    ]
  },
  {
    title: "Calibrate game piece",
    collapsed: true,
    fields: [
      {
        kind: "number",
        path: "measurements.dropHeight",
        label: "Drop test: drop height",
        unit: "m",
        step: 0.05,
        min: 0.1,
        hint: "Drop the piece (no spin) onto a field tile from this height (bottom of the ball)"
      },
      {
        kind: "number",
        path: "measurements.bounceHeight",
        label: "Drop test: bounce height",
        unit: "m",
        step: 0.01,
        min: 0,
        hint: "Peak height of the first bounce (bottom of the ball); film it against a tape measure"
      },
      ...[0, 1, 2].flatMap((i): FieldSpec[] => [
        {
          kind: "number",
          path: "measurements.shots." + i + ".exitSpeed",
          label: "Shot " + (i + 1) + ": exit speed",
          unit: "m/s",
          step: 0.05,
          min: 0,
          hint: "From video: distance travelled in the first few frames / time (0 = unused row)"
        },
        {
          kind: "number",
          path: "measurements.shots." + i + ".hoodAngleDeg",
          label: "Shot " + (i + 1) + ": hood",
          unit: "°",
          step: 0.5,
          min: 1,
          max: 89
        },
        {
          kind: "number",
          path: "measurements.shots." + i + ".launchHeight",
          label: "Shot " + (i + 1) + ": exit height",
          unit: "m",
          step: 0.01,
          min: 0
        },
        {
          kind: "number",
          path: "measurements.shots." + i + ".landingDistance",
          label: "Shot " + (i + 1) + ": landed at",
          unit: "m",
          step: 0.01,
          min: 0,
          hint: "Floor distance from where the piece left the robot to its first touch of the floor"
        }
      ])
    ]
  },
  {
    title: "Monte Carlo",
    collapsed: true,
    fields: [
      { kind: "number", path: "monteCarlo.samples", label: "Shots", unit: "", step: 50, min: 10, max: 5000 },
      { kind: "number", path: "monteCarlo.burstSize", label: "Rapid-fire burst", unit: "shots", step: 1, min: 1 },
      { kind: "number", path: "monteCarlo.seed", label: "Random seed", unit: "", step: 1 }
    ]
  }
];

export default class ShotLabController implements TabController {
  UUID = createUUID();

  private config: ShotLabConfig = defaultShotLabConfig();
  private form: SimForm;
  private command: ShotLabRendererCommand = {
    version: 0,
    status: {
      text: "BIOBUZZ preset loaded (red HIVE, audience CELL up, POLLEN) with the robot in front of the CELL mouth, and solved. Press Run to see how errors spread the shots.",
      kind: "info"
    },
    result: null
  };
  private busy = false;

  constructor(root: HTMLElement) {
    const container = root.getElementsByClassName("shotlab-form")[0] as HTMLElement;
    this.form = new SimForm(
      container,
      SECTIONS,
      () => this.config,
      (spec) => this.onFieldChange(spec.path)
    );
    this.form.addAction(
      "Solve",
      "Find the exit speed (and aim) that puts an error-free shot in the target center",
      false,
      () => this.solve()
    );
    this.form.addAction("Run", "Simulate the shot with error and write the results to the log", true, () =>
      this.run(false)
    );
    this.form.addAction("Run + error budget", "Also test each error source by itself (slower)", false, () =>
      this.run(true)
    );
    this.form.addAction("Reset", "Restore default settings", false, () => {
      this.config = defaultShotLabConfig();
      this.syncInputs();
    });
    this.form.addSectionAction(
      "Calibrate game piece",
      "Fit bounciness",
      "Bounciness = square root of (bounce height / drop height)",
      () => this.fitBounce()
    );
    this.form.addSectionAction(
      "Calibrate game piece",
      "Fit drag",
      "Find the drag coefficient that makes the simulated landing spots match your shots (uses the Shooter spin settings)",
      () => this.fitDrag()
    );
    this.form.addSectionAction(
      "Calibrate game piece",
      "Clear calibration",
      "Go back to the preset's estimated values for this game piece",
      () => {
        delete this.config.calibration[this.config.game.piece];
        applyGamePreset(this.config);
        this.syncInputs();
        this.setStatus("Calibration cleared: " + this.pieceName() + " uses the preset's estimates again.", "info");
      }
    );
    this.syncInputs();
    // Show results right away instead of empty charts (after a restored layout applies its settings)
    window.setTimeout(() => this.run(false), 400);
  }

  private pieceName(): string {
    const preset = getGamePreset(this.config.game.preset);
    const piece = preset?.pieces.find((p) => p.id === this.config.game.piece);
    return piece?.name.split(" (")[0] ?? "the game piece";
  }

  /** Stores a measured piece value so it survives preset changes and reloads. */
  private calibrate(values: Partial<ShotLabConfig["piece"]>) {
    Object.assign(this.config.piece, values);
    if (this.config.game.preset !== "custom") {
      const id = this.config.game.piece;
      this.config.calibration[id] = { ...this.config.calibration[id], ...values };
    }
    this.syncInputs();
  }

  private fitBounce() {
    const m = this.config.measurements;
    const e = fitRestitution(m.dropHeight, m.bounceHeight);
    if (e === null) {
      this.setStatus("Enter a drop height and a bounce height (the bounce must be lower than the drop).", "warn");
      return;
    }
    this.calibrate({ restitution: Math.round(e * 1000) / 1000 });
    this.setStatus("Bounciness fit: e = " + e.toFixed(3) + " for " + this.pieceName() + ". Saved as calibrated.", "ok");
  }

  private fitDrag() {
    const fit = fitDragCoefficient(buildGamePiece(this.config), this.config.measurements.shots, {
      mode: this.config.shooter.spinMode,
      rpm: this.config.shooter.spinRpm
    });
    if (fit === null) {
      this.setStatus("Enter at least one shot with an exit speed and where it landed.", "warn");
      return;
    }
    this.calibrate({ dragCoefficient: Math.round(fit.dragCoefficient * 1000) / 1000 });
    const shots = fit.perShot
      .map((p, i) => "shot " + (i + 1) + ": " + p.simulated.toFixed(2) + " m vs " + p.measured.toFixed(2) + " m")
      .join(", ");
    this.setStatus(
      "Drag fit: Cd = " +
        fit.dragCoefficient.toFixed(3) +
        " for " +
        this.pieceName() +
        " (RMS error " +
        (fit.rmsError * 100).toFixed(1) +
        " cm; " +
        shots +
        ")." +
        (fit.atBound
          ? " The fit hit its limit: check the exit speeds, or the spin settings in Shooter."
          : " Saved as calibrated; press Solve to re-aim."),
      fit.atBound || fit.rmsError > 0.15 ? "warn" : "ok"
    );
  }

  private onFieldChange(path: string) {
    if (path.startsWith("game.")) {
      if (path === "game.alliance") {
        // Mirror the robot to the other alliance's side of the field (rotational symmetry)
        const r = this.config.robot;
        const ownSide = this.config.game.alliance === "red" ? -1 : 1;
        if (Math.sign(r.x) !== ownSide) {
          r.x = -r.x;
          r.y = -r.y;
          r.headingDeg = r.headingDeg + 180;
          r.vx = -r.vx;
          r.vy = -r.vy;
          // The field is rotationally symmetric: red's rear CELL corresponds to blue's audience CELL
          this.config.game.upCell = mirrorCell(this.config.game.upCell);
        }
      }
      applyGamePreset(this.config);
      // A CELL is scored through its mouth: stand in front of it
      placeRobotForCell(this.config);
      this.syncInputs();
      this.solve(true);
      return;
    }
    if (path.startsWith("piece.") && this.config.game.preset !== "custom") {
      // Hand-measured piece values are calibration: keep the preset, remember the value per piece
      const key = path.slice("piece.".length) as keyof ShotLabConfig["piece"];
      this.calibrate({ [key]: this.config.piece[key] });
      this.setStatus("Saved " + key + " as a measured value for " + this.pieceName() + ".", "ok");
    }
    if (path.startsWith("target.") && this.config.game.preset !== "custom") {
      // Hand-edited target: stop following the preset so the edit sticks
      this.config.game.preset = "custom";
      this.form.sync();
      this.setStatus("Switched to a custom target (preset target values were edited).", "info");
    }
    publishShotLabConfig(this.config);
  }

  private syncInputs() {
    this.form.sync();
    publishShotLabConfig(this.config);
  }

  private setStatus(text: string, kind: ShotLabRendererCommand["status"]["kind"]) {
    this.command = { ...this.command, status: { text, kind } };
  }

  private setBusy(busy: boolean) {
    this.busy = busy;
    this.form.setDisabled(busy);
  }

  // -------------------------------------------------------------------------------------------
  // Actions
  // -------------------------------------------------------------------------------------------

  private targetName(): string {
    const preset = getGamePreset(this.config.game.preset);
    if (preset === null) return "the target";
    const cell = preset.cells.find(
      (c) => c.alliance === this.config.game.alliance && c.hiveCell === this.config.game.upCell
    );
    const piece = preset.pieces.find((p) => p.id === this.config.game.piece);
    return (cell?.name ?? "the target") + " with " + (piece?.name.split(" (")[0] ?? "the piece");
  }

  private solve(fromPresetChange = false) {
    if (this.busy) return;
    const solution = solveShot(this.config);
    if (!solution.converged) {
      const t = this.config.target;
      const distance = Math.hypot(t.x - this.config.robot.x, t.y - this.config.robot.y);
      const minimum = minimumShotDistance(this.config);
      this.setStatus(
        distance < minimum
          ? "Too close to come down into " +
              this.targetName() +
              ": at a " +
              this.config.shooter.hoodAngleDeg.toFixed(0) +
              "° hood the robot must be at least " +
              minimum.toFixed(2) +
              " m away (it's " +
              distance.toFixed(2) +
              " m). Back up or raise the hood angle."
          : "Couldn't find an exit speed that reaches " +
              this.targetName() +
              " at a " +
              this.config.shooter.hoodAngleDeg.toFixed(0) +
              "° hood angle. Try a different hood angle or position.",
        "warn"
      );
      publishShotLabConfig(this.config);
      return;
    }
    this.config.shooter.exitSpeed = Math.round(solution.exitSpeed * 1000) / 1000;
    if (this.config.autoAim) this.config.robot.headingDeg = Math.round(solution.headingDeg * 100) / 100;
    this.syncInputs();
    this.setStatus(
      "Solved: exit speed " +
        this.config.shooter.exitSpeed.toFixed(2) +
        " m/s" +
        (this.config.autoAim ? ", heading " + this.config.robot.headingDeg.toFixed(1) + "°" : "") +
        (fromPresetChange ? " for " + this.targetName() : "") +
        ". Press Run to add error.",
      "ok"
    );
  }

  private run(includeBudget: boolean) {
    if (this.busy) return;
    this.setBusy(true);
    const label = includeBudget ? "Monte Carlo + error budget" : "Monte Carlo";
    this.setStatus("Running " + label + "…", "busy");
    const startMs = Date.now();
    WorkerManager.request(
      "../bundles/shotLabWorker.js",
      { config: JSON.parse(JSON.stringify(this.config)), includeBudget },
      (progress) => {
        if (progress > 0) this.setStatus("Running error budget (each source alone)…", "busy");
      }
    )
      .then((run: ShotLabRun) => {
        const range = window.log.getTimestampRange();
        const ownDataOnly = window.log.getFieldKeys().every((key) => key.startsWith(SHOT_LAB_LOG_PREFIX));
        writeShotLabToLog(window.log, run, ownDataOnly ? 0 : range[0]);
        window.sidebar.refresh();
        window.tabs.refresh();
        this.command = {
          version: this.command.version + 1,
          status: {
            text:
              "Done in " +
              ((Date.now() - startMs) / 1000).toFixed(2) +
              " s. Data is in the sidebar under " +
              SHOT_LAB_LOG_PREFIX +
              " (drag Nominal/Ball to a 3D Field as a game piece).",
            kind: "ok"
          },
          result: buildResult(run)
        };
      })
      .catch((error) => {
        console.error(error);
        this.setStatus("Simulation failed. Check the inputs (e.g. a shot that never leaves the robot).", "warn");
      })
      .finally(() => this.setBusy(false));
  }

  // -------------------------------------------------------------------------------------------
  // TabController
  // -------------------------------------------------------------------------------------------

  saveState(): unknown {
    return this.config;
  }

  restoreState(state: unknown): void {
    this.config = mergeShotLabConfig(state);
    this.syncInputs();
  }

  refresh(): void {}

  newAssets(): void {}

  getActiveFields(): string[] {
    return [];
  }

  showTimeline(): boolean {
    return false;
  }

  getCommand(): ShotLabRendererCommand {
    return this.command;
  }
}

// ---------------------------------------------------------------------------------------------
// Projection for the renderer
// ---------------------------------------------------------------------------------------------

function buildResult(run: ShotLabRun): ShotLabRendererResult {
  const { scenario, nominal, monteCarlo: mc } = run;
  const tz = scenario.target;
  const cx = (tz.min[0] + tz.max[0]) / 2;
  const cy = (tz.min[1] + tz.max[1]) / 2;
  const rx = scenario.robot.x;
  const ry = scenario.robot.y;
  const dl = Math.hypot(cx - rx, cy - ry) || 1;
  const ux = (cx - rx) / dl;
  const uy = (cy - ry) / dl;
  const along = (p: number[]): [number, number] => [(p[0] - rx) * ux + (p[1] - ry) * uy, p[2]];
  const halfAlong = (Math.abs(ux) * (tz.max[0] - tz.min[0]) + Math.abs(uy) * (tz.max[1] - tz.min[1])) / 2;
  const thin = <T>(items: T[], n: number) =>
    items.length <= n ? items : items.filter((_, i) => i % Math.ceil(items.length / n) === 0);

  const d = mc.dispersion;
  const ci = mc.hitRateInterval;
  const pct = (x: number) => (x * 100).toFixed(1) + "%";
  const cm = (x: number) => (isFinite(x) ? (x * 100).toFixed(1) + " cm" : "—");
  const cards: ShotLabRendererResult["cards"] = [
    {
      label: "Hit rate",
      value: pct(mc.hitRate),
      detail: "95% CI " + pct(ci[0]) + "–" + pct(ci[1]) + " (" + mc.hits + "/" + mc.samples + ")",
      kind: mc.hitRate >= 0.9 ? "good" : mc.hitRate < 0.6 ? "bad" : "neutral"
    },
    {
      label: "Error-free shot",
      value: nominal.target?.hit ? "Hit" : "Miss",
      detail: isNaN(run.nominalMiss.range)
        ? "never crosses the target plane"
        : (run.nominalMiss.range >= 0 ? "long " : "short ") +
          cm(Math.abs(run.nominalMiss.range)) +
          ", " +
          (run.nominalMiss.lateral >= 0 ? "left " : "right ") +
          cm(Math.abs(run.nominalMiss.lateral)),
      kind: nominal.target?.hit ? "good" : "bad"
    },
    {
      label: "Spread (1 sd)",
      value: cm(d.stdDevRange) + " / " + cm(d.stdDevLateral),
      detail: "range / side-to-side at the target plane"
    },
    {
      label: "Average miss offset",
      value: cm(Math.hypot(d.bias[0], d.bias[1])),
      detail: "mean impact vs. target center"
    },
    {
      label: "Entry angle",
      value: mc.entryAngleDeg ? mc.entryAngleDeg.mean.toFixed(1) + "°" : "—",
      detail: mc.entrySpeed ? "entry speed " + mc.entrySpeed.mean.toFixed(2) + " m/s" : "no hits"
    },
    {
      label: "Apex",
      value: nominal.apex.position[2].toFixed(2) + " m",
      detail: "flight to target " + (nominal.target?.time ?? NaN).toFixed(2) + " s"
    }
  ];

  return {
    cards,
    side: {
      nominal: thin(nominal.samples, 300).map((s) => along(s.position)),
      samples: mc.shots
        .filter((s) => s.trajectory !== null)
        .map((s) => ({ points: thin(s.trajectory!.samples, 60).map((x) => along(x.position)), hit: s.hit })),
      target: { near: dl - halfAlong, far: dl + halfAlong, bottom: tz.min[2], top: tz.max[2] },
      launch: along(nominal.samples[0].position)
    },
    top: {
      points: mc.shots
        .filter((s) => s.crossing !== null)
        .map((s) => ({ x: s.crossing![0] - cx, y: s.crossing![1] - cy, hit: s.hit })),
      halfX: (tz.max[0] - tz.min[0]) / 2,
      halfY: (tz.max[1] - tz.min[1]) / 2,
      direction: [ux, uy],
      mean: isFinite(d.bias[0]) ? [d.bias[0], d.bias[1]] : null
    },
    budget:
      run.budget === null
        ? null
        : run.budget.map((e) => ({
            label: e.label,
            hitRate: e.hitRate,
            cost: e.hitRateCost,
            share: e.varianceShare,
            rangeStd: e.dispersion.stdDevRange,
            lateralStd: e.dispersion.stdDevLateral,
            summary: e.key === "none" || e.key === "all"
          }))
  };
}
