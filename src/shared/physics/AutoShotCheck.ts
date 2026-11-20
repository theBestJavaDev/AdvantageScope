// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Auto Check: takes the shots a robot-code simulation actually fired (the BioBuzz SimLoop RLOG) and
 * re-flies each one through SolversScope's physics to add what that simulation leaves out:
 *
 *  - Magnus: the same launch with the shooter's backspin (lift) versus no spin.
 *  - Match error: Monte Carlo over the Shot Lab error model (exit speed, battery, flywheel recovery,
 *    hood, turret, spin, release latency, localization, velocity estimate, launch height, game
 *    piece variation, air, defense).
 *
 * The result is a hit chance per shot and the distribution of POLLEN scored for the whole auto.
 * Pure (no DOM) so it runs in a worker and in tests. Positions are in the sim field frame
 * (GamePresets.ts): center origin, +X toward the blue wall, +Y away from the audience.
 */

import { Alliance, HiveCell, getGamePreset, upwardCell } from "./GamePresets";
import { DEFAULT_ENVIRONMENT, FTC_FIELD, PhysicsEngine, TargetZone } from "./PhysicsEngine";
import { ShotScenario, runMonteCarlo, targetPlaneHeight } from "./ShotErrorModel";
import { ShotLabConfig, buildErrorModel, buildGamePiece, measureMiss } from "./ShotLab";
import { RPM_TO_RAD_PER_SEC, SpinMode, computeLaunchState } from "./ShotLaunch";
import { Vector3 } from "./vector";

const DEG = Math.PI / 180;

/** One shot as the robot-code simulation fired it. */
export interface AutoShot {
  /** Log time the piece left the robot (s). */
  time: number;
  ballId: string | null;
  /** Where the piece left the robot (m, sim frame). */
  position: Vector3;
  /** Launch velocity in the field, robot motion included (m/s, sim frame). */
  velocity: Vector3;
  /** Robot velocity at launch (m/s, sim frame). */
  robotVelocity: [number, number];
  /** The CELL it was aimed at (the alliance's upward CELL at launch). */
  cell: { alliance: Alliance; hiveCell: HiveCell } | null;
  /** What the robot-code simulation said happened. */
  robotSimResult: "in" | "miss" | "unknown";
}

export interface AutoCheckOptions {
  /** Monte Carlo copies per shot. */
  samples: number;
  seed: number;
  /** Shooter spin. "rolling" estimates it from the exit speed (piece rolling off a hooded flywheel). */
  spinMode: SpinMode;
  spinRpm: number | "rolling";
}

export const DEFAULT_AUTO_CHECK_OPTIONS: AutoCheckOptions = {
  samples: 200,
  seed: 23511,
  spinMode: "backspin",
  spinRpm: "rolling"
};

export interface AutoShotResult {
  shot: AutoShot;
  /** Exit speed relative to the robot (m/s) and launch elevation (deg). */
  exitSpeed: number;
  elevationDeg: number;
  spinRpm: number;
  /** Re-flown with no spin and no error. */
  noSpinHit: boolean;
  /** Re-flown with the shooter's spin (Magnus) and no error. */
  spinHit: boolean;
  /** How far spin moved the impact (m): up the CELL mouth face (+ = higher), or along the shot. NaN if either never got there. */
  magnusShift: number;
  /** Chance it goes in with match error. */
  pIn: number;
  pInInterval: [number, number];
}

export interface AutoCheckResult {
  shots: AutoShotResult[];
  robotSimIn: number;
  noSpinIn: number;
  spinIn: number;
  /** Expected POLLEN in with match error. */
  expectedIn: number;
  stdDevIn: number;
  /** P(exactly k in), k = 0..shots. */
  distribution: number[];
  /** 5th and 95th percentile of POLLEN in. */
  range90: [number, number];
  durationMs: number;
}

function targetFor(config: ShotLabConfig, cell: AutoShot["cell"]): TargetZone | null {
  const preset = getGamePreset(config.game.preset) ?? getGamePreset("biobuzz");
  if (preset === null || cell === null) return null;
  const t = upwardCell(preset, cell.alliance, cell.hiveCell);
  return {
    name: t.name,
    min: [t.x - t.sizeX / 2, t.y - t.sizeY / 2, t.z - t.depth / 2],
    max: [t.x + t.sizeX / 2, t.y + t.sizeY / 2, t.z + t.depth / 2],
    requireDescending: t.requireDescending,
    mouth: t.mouth
  };
}

/** Rebuilds a Shot Lab scenario that reproduces the logged launch exactly (robot at the exit point). */
export function scenarioForShot(
  shot: AutoShot,
  config: ShotLabConfig,
  spin: { mode: SpinMode; rpm: number },
  target: TargetZone
): ShotScenario {
  const [rvx, rvy] = shot.robotVelocity;
  // Launch velocity relative to the robot: the shooter's own exit speed and direction
  const u: Vector3 = [shot.velocity[0] - rvx, shot.velocity[1] - rvy, shot.velocity[2]];
  const horizontal = Math.hypot(u[0], u[1]);
  const yaw = Math.atan2(u[1], u[0]);
  return {
    robot: { x: shot.position[0], y: shot.position[1], heading: yaw, vx: rvx, vy: rvy, omega: 0 },
    shooter: {
      offset: [0, 0, shot.position[2]],
      hoodAngleDeg: Math.atan2(u[2], horizontal) / DEG,
      turretYawDeg: 0,
      exitSpeed: Math.hypot(u[0], u[1], u[2]),
      spinMode: spin.mode,
      spinRpm: spin.rpm,
      inheritRobotVelocity: true
    },
    piece: buildGamePiece(config),
    environment: DEFAULT_ENVIRONMENT,
    field: FTC_FIELD,
    target
  };
}

function flyNominal(scenario: ShotScenario): { hit: boolean; crossing: Vector3 | null } {
  const engine = new PhysicsEngine(scenario.piece, scenario.environment, scenario.field);
  const result = engine.simulate(
    computeLaunchState(scenario.robot, scenario.shooter),
    {
      duration: 3,
      timeStep: 0.001,
      sampleInterval: 0.05,
      stopAfterLanding: true,
      crossingHeight: targetPlaneHeight(scenario.target)
    },
    scenario.target
  );
  return { hit: result.target?.hit ?? false, crossing: result.crossing };
}

/** Poisson-binomial: P(exactly k of the independent shots go in). */
export function countDistribution(probabilities: number[]): number[] {
  let dist = [1];
  for (const p of probabilities) {
    const next = new Array(dist.length + 1).fill(0);
    dist.forEach((q, k) => {
      next[k] += q * (1 - p);
      next[k + 1] += q * p;
    });
    dist = next;
  }
  return dist;
}

function percentile(dist: number[], fraction: number): number {
  let total = 0;
  for (let k = 0; k < dist.length; k++) {
    total += dist[k];
    if (total >= fraction - 1e-12) return k;
  }
  return dist.length - 1;
}

/** Re-flies every logged shot with spin and with match error. */
export function analyzeAutoShots(
  shots: AutoShot[],
  config: ShotLabConfig,
  options: Partial<AutoCheckOptions> = {}
): AutoCheckResult {
  const start = Date.now();
  const opts = { ...DEFAULT_AUTO_CHECK_OPTIONS, ...options };
  const model = buildErrorModel(config);
  const piece = buildGamePiece(config);
  const results: AutoShotResult[] = [];

  shots.forEach((shot, index) => {
    const target = targetFor(config, shot.cell);
    const relative: Vector3 = [
      shot.velocity[0] - shot.robotVelocity[0],
      shot.velocity[1] - shot.robotVelocity[1],
      shot.velocity[2]
    ];
    const exitSpeed = Math.hypot(...relative);
    const elevationDeg = Math.atan2(relative[2], Math.hypot(relative[0], relative[1])) / DEG;
    const spinRpm =
      opts.spinRpm === "rolling" ? exitSpeed / piece.radius / RPM_TO_RAD_PER_SEC : Math.max(0, opts.spinRpm);
    if (target === null) {
      results.push({
        shot,
        exitSpeed,
        elevationDeg,
        spinRpm,
        noSpinHit: false,
        spinHit: false,
        magnusShift: NaN,
        pIn: NaN,
        pInInterval: [NaN, NaN]
      });
      return;
    }
    const noSpin = flyNominal(scenarioForShot(shot, config, { mode: "none", rpm: 0 }, target));
    const spinScenario = scenarioForShot(shot, config, { mode: opts.spinMode, rpm: spinRpm }, target);
    const withSpin = flyNominal(spinScenario);

    // How far spin moves the impact: on a CELL mouth, up/down the mouth face (+ = higher); otherwise
    // along the shot at the target plane (+ = longer)
    let magnusShift = NaN;
    if (target.mouth !== undefined) {
      const a = measureMiss(scenarioForShot(shot, config, { mode: "none", rpm: 0 }, target));
      const b = measureMiss(spinScenario);
      if (isFinite(a.range) && isFinite(b.range)) magnusShift = b.range - a.range;
    } else if (noSpin.crossing !== null && withSpin.crossing !== null) {
      const h = Math.hypot(relative[0], relative[1]) || 1;
      const ux = relative[0] / h;
      const uy = relative[1] / h;
      magnusShift = (withSpin.crossing[0] - noSpin.crossing[0]) * ux + (withSpin.crossing[1] - noSpin.crossing[1]) * uy;
    }

    const mc = runMonteCarlo(spinScenario, model, {
      samples: Math.max(10, Math.round(opts.samples)),
      seed: opts.seed + index * 7919,
      burstSize: 1,
      keepTrajectories: 0
    });
    results.push({
      shot,
      exitSpeed,
      elevationDeg,
      spinRpm,
      noSpinHit: noSpin.hit,
      spinHit: withSpin.hit,
      magnusShift,
      pIn: mc.hitRate,
      pInInterval: mc.hitRateInterval
    });
  });

  const probabilities = results.map((r) => (isFinite(r.pIn) ? r.pIn : 0));
  const distribution = countDistribution(probabilities);
  const expectedIn = probabilities.reduce((a, b) => a + b, 0);
  const variance = probabilities.reduce((a, p) => a + p * (1 - p), 0);
  return {
    shots: results,
    robotSimIn: shots.filter((s) => s.robotSimResult === "in").length,
    noSpinIn: results.filter((r) => r.noSpinHit).length,
    spinIn: results.filter((r) => r.spinHit).length,
    expectedIn,
    stdDevIn: Math.sqrt(variance),
    distribution,
    range90: [percentile(distribution, 0.05), percentile(distribution, 0.95)],
    durationMs: Date.now() - start
  };
}
