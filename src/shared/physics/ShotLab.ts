// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Shot Lab: the glue between the UI and the physics. Turns a flat, serializable config into a
 * scenario, solves the nominal shot, runs the Monte Carlo + error budget, and writes everything
 * into a Log under /SolversSim/ShotLab so the normal 3D field, line graph and table tabs can show it.
 *
 * Kept free of DOM code so it can be tested from Node.
 */

import Log from "../log/Log";
import {
  Alliance,
  GAME_PRESETS,
  HiveCell,
  getGamePreset,
  simToLogHeading,
  simToLogTranslation,
  upwardCell
} from "./GamePresets";
import {
  DEFAULT_ENVIRONMENT,
  DEFAULT_GAME_PIECE,
  FTC_FIELD,
  GamePieceProperties,
  PhysicsEngine,
  TargetZone,
  TrajectoryResult,
  mouthCrossing
} from "./PhysicsEngine";
import { MeasuredShot } from "./ShotCalibration";
import {
  ERROR_SOURCES,
  ErrorBudgetEntry,
  ErrorSourceKey,
  MonteCarloResult,
  ShotErrorModel,
  ShotScenario,
  TYPICAL_ERROR_MODEL,
  ZERO_ERROR_MODEL,
  analyzeErrorBudget,
  isolateErrorSources,
  runMonteCarlo,
  targetPlaneHeight
} from "./ShotErrorModel";
import { RPM_TO_RAD_PER_SEC, SpinMode, computeLaunchState } from "./ShotLaunch";
import { Vector3, length } from "./vector";

const DEG = Math.PI / 180;
export const SHOT_LAB_LOG_PREFIX = "/SolversSim/ShotLab";

// ---------------------------------------------------------------------------------------------
// Config (flat + plain units so it can be saved in tab state and bound to inputs)
// ---------------------------------------------------------------------------------------------

export interface ShotLabConfig {
  robot: { x: number; y: number; headingDeg: number; vx: number; vy: number; omegaDegPerSec: number };
  shooter: {
    /** Exit point ahead of robot center (m). */
    offsetX: number;
    /** Exit point height above the floor (m). */
    launchHeight: number;
    hoodAngleDeg: number;
    turretYawDeg: number;
    exitSpeed: number;
    spinMode: SpinMode;
    spinRpm: number;
    inheritRobotVelocity: boolean;
  };
  target: {
    x: number;
    y: number;
    /** Height of the target opening's center (m). */
    z: number;
    sizeX: number;
    sizeY: number;
    depth: number;
    requireDescending: boolean;
  };
  piece: { massKg: number; diameterM: number; dragCoefficient: number; magnusScale: number; restitution: number };
  error: { preset: "typical" | "zero"; enabled: Record<ErrorSourceKey, boolean> };
  monteCarlo: { samples: number; burstSize: number; seed: number };
  /** When solving, also turn the robot so the shot lands on target (includes shoot-on-the-move lead). */
  autoAim: boolean;
  /**
   * Season preset. When not "custom", the target is the alliance's upward-facing scoring location
   * and the piece is the chosen game piece (see GamePresets.ts).
   */
  game: { preset: string; alliance: Alliance; upCell: HiveCell; piece: string };
  /** Measured overrides per game piece id (from the calibration tools or hand edits). */
  calibration: Record<string, Partial<ShotLabConfig["piece"]>>;
  /** Raw measurements the calibration tools fit from. */
  measurements: { dropHeight: number; bounceHeight: number; shots: MeasuredShot[] };
}

/** Sets target + game piece from the selected season preset (no-op for "custom"). */
export function applyGamePreset(config: ShotLabConfig): ShotLabConfig {
  const preset = getGamePreset(config.game.preset);
  if (preset === null) return config;
  const target = upwardCell(preset, config.game.alliance, config.game.upCell);
  config.target = {
    x: target.x,
    y: target.y,
    z: target.z,
    sizeX: target.sizeX,
    sizeY: target.sizeY,
    depth: target.depth,
    requireDescending: target.requireDescending
  };
  const piece = preset.pieces.find((p) => p.id === config.game.piece) ?? preset.pieces[0];
  config.game.piece = piece.id;
  config.piece = {
    massKg: piece.massKg,
    diameterM: piece.diameterM,
    dragCoefficient: piece.dragCoefficient,
    magnusScale: piece.magnusScale,
    restitution: piece.restitution,
    // Measured values win over the preset's estimates
    ...config.calibration[piece.id]
  };
  return config;
}

let cachedDefault: ShotLabConfig | null = null;

/** Defaults: BIOBUZZ, red alliance, already solved so the first shot hits. Cached (solving costs ~15 ms). */
export function defaultShotLabConfig(): ShotLabConfig {
  if (cachedDefault === null) {
    const config = applyGamePreset(rawDefaultConfig());
    placeRobotForCell(config);
    const solution = solveShot(config);
    if (solution.converged) {
      config.shooter.exitSpeed = Math.round(solution.exitSpeed * 1000) / 1000;
      config.robot.headingDeg = Math.round(solution.headingDeg * 100) / 100;
    }
    cachedDefault = config;
  }
  return JSON.parse(JSON.stringify(cachedDefault));
}

function rawDefaultConfig(): ShotLabConfig {
  const enabled = {} as Record<ErrorSourceKey, boolean>;
  ERROR_SOURCES.forEach((source) => (enabled[source.key] = true));
  return {
    // ~1 m in front of the red HIVE's audience-side CELL mouth (the match's starting state: red
    // audience CELL up). A CELL can only be scored through its mouth, so the robot faces it.
    robot: { x: -0.32, y: -1.4, headingDeg: 90, vx: 0, vy: 0, omegaDegPerSec: 0 },
    shooter: {
      offsetX: 0.1,
      launchHeight: 0.3,
      hoodAngleDeg: 70,
      turretYawDeg: 0,
      exitSpeed: 6.5,
      spinMode: "backspin",
      spinRpm: 1500,
      inheritRobotVelocity: true
    },
    // Replaced by the season preset below (custom targets keep whatever the user typed)
    target: { x: 1.45, y: 1.45, z: 0.98, sizeX: 0.4, sizeY: 0.4, depth: 0.12, requireDescending: true },
    piece: {
      massKg: DEFAULT_GAME_PIECE.mass,
      diameterM: DEFAULT_GAME_PIECE.radius * 2,
      dragCoefficient: DEFAULT_GAME_PIECE.dragCoefficient,
      magnusScale: DEFAULT_GAME_PIECE.magnusScale,
      restitution: DEFAULT_GAME_PIECE.floorRestitution
    },
    error: { preset: "typical", enabled },
    monteCarlo: { samples: 300, burstSize: 1, seed: 23511 },
    autoAim: true,
    game: { preset: GAME_PRESETS[0].id, alliance: "red", upCell: "audience", piece: GAME_PRESETS[0].pieces[0].id },
    calibration: {},
    measurements: {
      dropHeight: 1.0,
      bounceHeight: 0,
      shots: [0, 1, 2].map(() => ({ exitSpeed: 0, hoodAngleDeg: 45, launchHeight: 0.3, landingDistance: 0 }))
    }
  };
}

/** Fills in anything missing from an older saved config. */
export function mergeShotLabConfig(saved: unknown): ShotLabConfig {
  const base = defaultShotLabConfig();
  if (typeof saved !== "object" || saved === null) return base;
  const s = saved as Partial<Record<keyof ShotLabConfig, any>>;
  // Keep target/piece consistent with the preset (no-op for "custom")
  return applyGamePreset({
    robot: { ...base.robot, ...s.robot },
    shooter: { ...base.shooter, ...s.shooter },
    target: { ...base.target, ...s.target },
    piece: { ...base.piece, ...s.piece },
    error: {
      preset: s.error?.preset === "zero" ? "zero" : "typical",
      enabled: { ...base.error.enabled, ...s.error?.enabled }
    },
    monteCarlo: { ...base.monteCarlo, ...s.monteCarlo },
    autoAim: typeof s.autoAim === "boolean" ? s.autoAim : base.autoAim,
    game: {
      preset: typeof s.game?.preset === "string" ? s.game.preset : base.game.preset,
      alliance: s.game?.alliance === "blue" ? "blue" : "red",
      upCell: s.game?.upCell === "audience" ? "audience" : "rear",
      piece: typeof s.game?.piece === "string" ? s.game.piece : base.game.piece
    },
    calibration: typeof s.calibration === "object" && s.calibration !== null ? s.calibration : {},
    measurements: {
      dropHeight: Number(s.measurements?.dropHeight ?? base.measurements.dropHeight),
      bounceHeight: Number(s.measurements?.bounceHeight ?? base.measurements.bounceHeight),
      shots: base.measurements.shots.map((shot, i) => ({ ...shot, ...s.measurements?.shots?.[i] }))
    }
  });
}

export function buildTargetZone(config: ShotLabConfig): TargetZone {
  const t = config.target;
  const zone: TargetZone = {
    name: "Target",
    min: [t.x - t.sizeX / 2, t.y - t.sizeY / 2, t.z - t.depth / 2],
    max: [t.x + t.sizeX / 2, t.y + t.sizeY / 2, t.z + t.depth / 2],
    requireDescending: t.requireDescending
  };
  // A season CELL is scored through its tilted mouth (see GamePresets.cellMouth)
  const preset = getGamePreset(config.game.preset);
  if (preset !== null) {
    const cell = upwardCell(preset, config.game.alliance, config.game.upCell);
    if (cell.mouth !== undefined) zone.mouth = cell.mouth;
  }
  return zone;
}

/** Puts the robot on the side the target CELL's mouth faces (a CELL can only be scored from there). */
export function placeRobotForCell(config: ShotLabConfig): void {
  const preset = getGamePreset(config.game.preset);
  if (preset === null) return;
  const mouth = upwardCell(preset, config.game.alliance, config.game.upCell).mouth;
  if (mouth === undefined) return;
  const r = config.robot;
  const facing = Math.sign(mouth.normal[1]);
  // Same x side as the HIVE, in front of the mouth
  if (Math.sign(r.x) !== Math.sign(mouth.center[0])) r.x = -r.x;
  if (Math.sign(r.y - mouth.center[1]) !== facing) r.y = mouth.center[1] + facing * Math.abs(r.y - mouth.center[1]);
  // Stay on the field (robot half-size margin)
  const limit = FTC_FIELD.halfY - 0.25;
  r.y = Math.max(-limit, Math.min(limit, r.y));
}

export function buildGamePiece(config: ShotLabConfig): GamePieceProperties {
  const p = config.piece;
  return {
    ...DEFAULT_GAME_PIECE,
    name: "Shot Lab piece",
    mass: p.massKg,
    radius: p.diameterM / 2,
    dragCoefficient: p.dragCoefficient,
    magnusScale: p.magnusScale,
    floorRestitution: p.restitution,
    wallRestitution: Math.max(0, p.restitution - 0.1)
  };
}

export function buildScenario(config: ShotLabConfig): ShotScenario {
  const r = config.robot;
  const s = config.shooter;
  return {
    robot: { x: r.x, y: r.y, heading: r.headingDeg * DEG, vx: r.vx, vy: r.vy, omega: r.omegaDegPerSec * DEG },
    shooter: {
      offset: [s.offsetX, 0, s.launchHeight],
      hoodAngleDeg: s.hoodAngleDeg,
      turretYawDeg: s.turretYawDeg,
      exitSpeed: s.exitSpeed,
      spinMode: s.spinMode,
      spinRpm: s.spinRpm,
      inheritRobotVelocity: s.inheritRobotVelocity
    },
    piece: buildGamePiece(config),
    environment: DEFAULT_ENVIRONMENT,
    field: FTC_FIELD,
    target: buildTargetZone(config)
  };
}

export function buildErrorModel(config: ShotLabConfig): ShotErrorModel {
  const preset = config.error.preset === "zero" ? ZERO_ERROR_MODEL : TYPICAL_ERROR_MODEL;
  const keys = ERROR_SOURCES.map((s) => s.key).filter((key) => config.error.enabled[key]);
  return isolateErrorSources(preset, keys);
}

// ---------------------------------------------------------------------------------------------
// Nominal shot + solver
// ---------------------------------------------------------------------------------------------

const NOMINAL_OPTIONS = { duration: 4, timeStep: 0.001, sampleInterval: 0.01, stopAfterLanding: false };
const SOLVER_OPTIONS = { duration: 3, timeStep: 0.002, sampleInterval: 0.05, stopAfterLanding: true };

/** The error-free shot, simulated until the piece comes to rest (bounces included). */
export function simulateNominal(scenario: ShotScenario): TrajectoryResult {
  const engine = new PhysicsEngine(scenario.piece, scenario.environment, scenario.field);
  return engine.simulate(
    computeLaunchState(scenario.robot, scenario.shooter),
    { ...NOMINAL_OPTIONS, crossingHeight: targetPlaneHeight(scenario.target) },
    scenario.target
  );
}

export interface ShotMiss {
  /** + = long (past the target center), - = short. NaN if the shot never comes down through the plane. */
  range: number;
  /** + = left of the robot-to-target line. */
  lateral: number;
  /** True if the shot never got as high as the target plane. */
  tooLow: boolean;
}

/**
 * Height the solver aims through. A target that must be entered falling (a basket / CELL opening)
 * is aimed through its top face, so a solved shot drops in instead of clipping the side while
 * still rising; anything else is aimed through its middle.
 */
export function aimPlaneHeight(target: TargetZone): number {
  return targetPlaneHeight(target);
}

/** Where an error-free shot crosses the aim plane, relative to the target center. */
export function measureMiss(scenario: ShotScenario): ShotMiss {
  const engine = new PhysicsEngine(scenario.piece, scenario.environment, scenario.field);
  const mouth = scenario.target.mouth;
  if (mouth !== undefined) {
    // Tilted mouth: where the shot crosses the mouth plane inward. "range" is the height on the mouth
    // face (+ = high / long), "lateral" the left-right offset; no inward crossing = short.
    const result = engine.simulate(computeLaunchState(scenario.robot, scenario.shooter), {
      ...SOLVER_OPTIONS,
      sampleInterval: 0.004
    });
    for (let i = 1; i < result.samples.length; i++) {
      const c = mouthCrossing(result.samples[i - 1].position, result.samples[i].position, mouth);
      if (c === null) continue;
      const dx = mouth.center[0] - scenario.robot.x;
      const dy = mouth.center[1] - scenario.robot.y;
      const dl = Math.hypot(dx, dy) || 1;
      const ex = c.point[0] - mouth.center[0];
      const ey = c.point[1] - mouth.center[1];
      return { range: c.v, lateral: (-ex * dy + ey * dx) / dl, tooLow: false };
    }
    return { range: NaN, lateral: NaN, tooLow: true };
  }
  const planeZ = aimPlaneHeight(scenario.target);
  const result = engine.simulate(computeLaunchState(scenario.robot, scenario.shooter), {
    ...SOLVER_OPTIONS,
    crossingHeight: planeZ
  });
  const cx = (scenario.target.min[0] + scenario.target.max[0]) / 2;
  const cy = (scenario.target.min[1] + scenario.target.max[1]) / 2;
  const dx = cx - scenario.robot.x;
  const dy = cy - scenario.robot.y;
  const dl = Math.hypot(dx, dy) || 1;
  const ux = dx / dl;
  const uy = dy / dl;
  if (result.crossing === null) {
    return { range: NaN, lateral: NaN, tooLow: result.apex.position[2] < planeZ };
  }
  const ex = result.crossing[0] - cx;
  const ey = result.crossing[1] - cy;
  return { range: ex * ux + ey * uy, lateral: -ex * uy + ey * ux, tooLow: false };
}

/**
 * Closest horizontal distance from which a drag-free shot at this hood angle can still come DOWN
 * through the target plane: the descending crossing of height h exists only beyond 2 h / tan(hood).
 * (Drag makes the real minimum a little larger.) Infinity if the target is out of reach of any lob.
 */
export function minimumShotDistance(config: ShotLabConfig): number {
  const aimZ = config.target.requireDescending ? config.target.z + config.target.depth / 2 : config.target.z;
  const rise = aimZ - config.shooter.launchHeight;
  const tan = Math.tan(config.shooter.hoodAngleDeg * DEG);
  if (rise <= 0) return 0;
  if (tan <= 0) return Infinity;
  return (2 * rise) / tan;
}

export interface SolveResult {
  converged: boolean;
  exitSpeed: number;
  headingDeg: number;
  miss: ShotMiss;
  iterations: number;
}

/**
 * Finds the exit speed (and, with autoAim, the robot heading) that puts an error-free shot through
 * the center of the target plane at the current hood angle. Speed is bracketed + bisected (range
 * grows with speed for any sane hood angle); heading is corrected from the lateral miss each pass,
 * which also leads the target when shooting on the move.
 */
export function solveShot(config: ShotLabConfig, tolerance = 0.005): SolveResult {
  const scenario = buildScenario(config);
  const t = config.target;
  if (config.autoAim) {
    scenario.robot.heading =
      Math.atan2(t.y - scenario.robot.y, t.x - scenario.robot.x) - scenario.shooter.turretYawDeg * DEG;
  }
  const dist = Math.hypot(t.x - scenario.robot.x, t.y - scenario.robot.y);

  let lo = 0.5;
  let hi = 25;
  let miss: ShotMiss = { range: NaN, lateral: NaN, tooLow: true };
  let iterations = 0;
  for (; iterations < 60; iterations++) {
    scenario.shooter.exitSpeed = (lo + hi) / 2;
    miss = measureMiss(scenario);
    const short = miss.tooLow || (!isNaN(miss.range) && miss.range < 0);
    if (isNaN(miss.range) && !miss.tooLow) {
      // Got high enough but never came back down through the plane in the field: far too fast
      hi = scenario.shooter.exitSpeed;
    } else if (short) {
      lo = scenario.shooter.exitSpeed;
    } else {
      hi = scenario.shooter.exitSpeed;
    }
    if (config.autoAim && !isNaN(miss.lateral)) {
      scenario.robot.heading -= Math.atan2(miss.lateral, Math.max(0.2, dist));
    }
    const done =
      !isNaN(miss.range) && Math.abs(miss.range) < tolerance && (!config.autoAim || Math.abs(miss.lateral) < tolerance);
    if (done || hi - lo < 1e-4) break;
  }
  const converged =
    !isNaN(miss.range) &&
    Math.abs(miss.range) < tolerance * 4 &&
    (!config.autoAim || Math.abs(miss.lateral) < tolerance * 4);
  return {
    converged,
    exitSpeed: scenario.shooter.exitSpeed,
    headingDeg: scenario.robot.heading / DEG,
    miss,
    iterations: iterations + 1
  };
}

// ---------------------------------------------------------------------------------------------
// Running the lab
// ---------------------------------------------------------------------------------------------

export interface ShotLabRun {
  config: ShotLabConfig;
  scenario: ShotScenario;
  nominal: TrajectoryResult;
  nominalMiss: ShotMiss;
  monteCarlo: MonteCarloResult;
  /** Only present when the error budget was requested (it costs ~13 extra Monte Carlo runs). */
  budget: ErrorBudgetEntry[] | null;
  durationMs: number;
}

export function runShotLab(
  config: ShotLabConfig,
  includeBudget: boolean,
  onStage: ((stage: string) => void) | null = null
): ShotLabRun {
  const start = Date.now();
  const scenario = buildScenario(config);
  const model = buildErrorModel(config);
  const mcOptions = {
    samples: Math.max(1, Math.round(config.monteCarlo.samples)),
    burstSize: Math.max(1, Math.round(config.monteCarlo.burstSize)),
    seed: Math.round(config.monteCarlo.seed),
    keepTrajectories: 30
  };
  const nominal = simulateNominal(scenario);
  const nominalMiss = measureMiss(scenario);
  onStage?.("Monte Carlo");
  const monteCarlo = runMonteCarlo(scenario, model, mcOptions);
  if (includeBudget) onStage?.("error budget");
  const budget = includeBudget
    ? analyzeErrorBudget(scenario, model, mcOptions).filter(
        (e) => e.key === "none" || e.key === "all" || config.error.enabled[e.key]
      )
    : null;
  return { config, scenario, nominal, nominalMiss, monteCarlo, budget, durationMs: Date.now() - start };
}

// ---------------------------------------------------------------------------------------------
// Log output
// ---------------------------------------------------------------------------------------------

function downsample<T>(items: T[], maxCount: number): T[] {
  if (items.length <= maxCount) return items;
  const out: T[] = [];
  const step = (items.length - 1) / (maxCount - 1);
  for (let i = 0; i < maxCount; i++) out.push(items[Math.round(i * step)]);
  return out;
}

function safeKey(label: string): string {
  return label.replace(/[^A-Za-z0-9]+/g, "");
}

/**
 * Writes a run into the log. Old Shot Lab output is removed first so stale trajectories never mix
 * with new ones. Time-series data starts at `startTime` (seconds, log time base).
 *
 * Layout (all under /SolversSim/ShotLab):
 *   Robot                    Pose2d             robot pose the shot was taken from
 *   Target                   Translation3d      target center
 *   Nominal/Ball             Translation3d      animated error-free shot (drag to 3D Field as a game piece)
 *   Nominal/Trajectory       Translation3d[]    full error-free path (3D Field trajectory)
 *   Nominal/Speed, Height, DragForce, MagnusForce, SpinRPM    numbers vs. time (line graph)
 *   MonteCarlo/Trajectories/<n>  Translation3d[] a sample of the noisy shots
 *   MonteCarlo/Crossings     Translation3d[]    where each shot crossed the target plane
 *   MonteCarlo/Landings      Translation3d[]    first floor contact of each shot
 *   MonteCarlo/HitRate ...   numbers            summary stats (table)
 *   ErrorBudget/<Source>/... numbers            per-source hit-rate cost + variance share
 */
export function writeShotLabToLog(log: Log, run: ShotLabRun, startTime: number) {
  const P = SHOT_LAB_LOG_PREFIX;
  log.deleteFieldsWithPrefix(P);
  const t0 = startTime;

  // Positions go into the log in the FTC field frame (see simToLogTranslation)
  const L = simToLogTranslation;
  log.putPose(P + "/Robot", t0, {
    translation: L([run.scenario.robot.x, run.scenario.robot.y]),
    rotation: simToLogHeading(run.scenario.robot.heading)
  });
  const tz = run.scenario.target;
  log.putTranslation3d(
    P + "/Target",
    t0,
    L([(tz.min[0] + tz.max[0]) / 2, (tz.min[1] + tz.max[1]) / 2, (tz.min[2] + tz.max[2]) / 2] as Vector3)
  );

  // Nominal shot, animated
  for (const s of run.nominal.samples) {
    const t = t0 + s.time;
    log.putTranslation3d(P + "/Nominal/Ball", t, L(s.position));
    log.putNumber(P + "/Nominal/Speed", t, length(s.velocity));
    log.putNumber(P + "/Nominal/Height", t, s.position[2]);
    log.putNumber(P + "/Nominal/DragForce", t, s.dragForce);
    log.putNumber(P + "/Nominal/MagnusForce", t, s.magnusForce);
    log.putNumber(P + "/Nominal/SpinRPM", t, length(s.angularVelocity) / RPM_TO_RAD_PER_SEC);
  }
  log.putTranslation3dArray(
    P + "/Nominal/Trajectory",
    t0,
    downsample(run.nominal.samples, 200).map((s) => L(s.position))
  );
  log.putBoolean(P + "/Nominal/Hit", t0, run.nominal.target?.hit ?? false);
  log.putNumber(P + "/Nominal/RangeMiss", t0, run.nominalMiss.range);
  log.putNumber(P + "/Nominal/LateralMiss", t0, run.nominalMiss.lateral);
  log.putNumber(P + "/Nominal/ApexHeight", t0, run.nominal.apex.position[2]);

  // Monte Carlo
  const mc = run.monteCarlo;
  mc.shots
    .filter((s) => s.trajectory !== null)
    .forEach((s, i) => {
      log.putTranslation3dArray(
        P + "/MonteCarlo/Trajectories/" + i.toString(),
        t0,
        downsample(s.trajectory!.samples, 60).map((x) => L(x.position))
      );
    });
  log.putTranslation3dArray(
    P + "/MonteCarlo/Crossings",
    t0,
    mc.shots.filter((s) => s.crossing !== null).map((s) => L(s.crossing as Vector3))
  );
  log.putTranslation3dArray(
    P + "/MonteCarlo/Landings",
    t0,
    mc.shots.filter((s) => s.landing !== null).map((s) => L(s.landing as Vector3))
  );
  log.putNumber(P + "/MonteCarlo/Samples", t0, mc.samples);
  log.putNumber(P + "/MonteCarlo/HitRate", t0, mc.hitRate);
  log.putNumber(P + "/MonteCarlo/HitRateLow95", t0, mc.hitRateInterval[0]);
  log.putNumber(P + "/MonteCarlo/HitRateHigh95", t0, mc.hitRateInterval[1]);
  log.putNumber(P + "/MonteCarlo/MissDistanceMean", t0, mc.missDistance.mean);
  log.putNumber(P + "/MonteCarlo/DispersionRangeStdDev", t0, mc.dispersion.stdDevRange);
  log.putNumber(P + "/MonteCarlo/DispersionLateralStdDev", t0, mc.dispersion.stdDevLateral);
  log.putNumber(P + "/MonteCarlo/BiasX", t0, mc.dispersion.bias[0]);
  log.putNumber(P + "/MonteCarlo/BiasY", t0, mc.dispersion.bias[1]);
  if (mc.entrySpeed) log.putNumber(P + "/MonteCarlo/EntrySpeedMean", t0, mc.entrySpeed.mean);
  if (mc.entryAngleDeg) log.putNumber(P + "/MonteCarlo/EntryAngleMeanDeg", t0, mc.entryAngleDeg.mean);
  // Per-shot series (shot number as time) so the line graph shows rapid-fire droop, misses, etc.
  mc.shots.forEach((s, i) => {
    const t = t0 + i * 0.01;
    log.putNumber(P + "/MonteCarlo/PerShot/ExitSpeed", t, s.exitSpeed);
    log.putNumber(P + "/MonteCarlo/PerShot/MissDistance", t, s.missDistance);
    log.putBoolean(P + "/MonteCarlo/PerShot/Hit", t, s.hit);
  });

  // Error budget
  if (run.budget !== null) {
    for (const e of run.budget) {
      const base = P + "/ErrorBudget/" + safeKey(e.label);
      log.putNumber(base + "/HitRate", t0, e.hitRate);
      log.putNumber(base + "/HitRateCost", t0, e.hitRateCost);
      log.putNumber(base + "/VarianceShare", t0, e.varianceShare);
      log.putNumber(base + "/DispersionRangeStdDev", t0, e.dispersion.stdDevRange);
      log.putNumber(base + "/DispersionLateralStdDev", t0, e.dispersion.stdDevLateral);
    }
  }
}
