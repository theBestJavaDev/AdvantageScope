// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Match error model + Monte Carlo shot analysis.
 *
 * Every source of shot-to-shot variation seen in real matches is a separate, independently toggleable
 * entry so the tool can report not just "you hit 72%" but "heading error costs you 15%, battery sag 8%".
 * All std devs are 1-sigma. A value of 0 disables that source.
 */

import {
  DEFAULT_ENVIRONMENT,
  DEFAULT_GAME_PIECE,
  EnvironmentProperties,
  FTC_FIELD,
  FieldGeometry,
  GamePieceProperties,
  PhysicsEngine,
  SimulationOptions,
  TargetZone,
  TrajectoryResult
} from "./PhysicsEngine";
import { RobotState2d, ShooterConfig, computeLaunchState, launchDirection } from "./ShotLaunch";
import { Random, Vector3, cross, dot, length } from "./vector";

const DEG = Math.PI / 180;

export interface ShotErrorModel {
  /** Shot-to-shot exit speed variation (flywheel PID ripple, compression, contact). */
  exitSpeed: { stdDevPct: number };
  /** Battery voltage changes flywheel speed when the controller can't fully compensate. */
  battery: { nominalVolts: number; stdDevVolts: number; referenceVolts: number; speedPctPerVolt: number };
  /** Rapid fire: each shot drops flywheel speed, which recovers exponentially. */
  flywheelRecovery: { dropPct: number; recoveryTauSec: number; shotIntervalSec: number };
  /** Hood (launch elevation) error: mechanical slop, servo resolution, calibration bias. */
  hoodAngle: { stdDevDeg: number; biasDeg: number };
  /** Turret / heading-aim error of the shooter itself. */
  turretYaw: { stdDevDeg: number; biasDeg: number };
  /** Spin rate variation and spin axis tilt (tilted axis turns backspin partly into sidespin). */
  spin: { rpmStdDevPct: number; axisTiltStdDevDeg: number };
  /** Delay between "shoot" command and release; robot keeps moving during it. */
  releaseLatency: { meanMs: number; stdDevMs: number; compensatedMs: number };
  /** Robot thinks it's somewhere slightly different than it is (odometry/vision drift). */
  localization: { xyStdDev: number; headingStdDevDeg: number };
  /** Shooting on the move with a wrong velocity estimate (only applies while moving). */
  velocityEstimate: { stdDev: number };
  /** Exit height variation (intake/indexer seating, frame flex). */
  launchHeight: { stdDev: number };
  /** Game piece variation: worn/scuffed/under-inflated pieces, manufacturing tolerance. */
  gamePiece: {
    massStdDevPct: number;
    radiusStdDev: number;
    dragStdDevPct: number;
    magnusStdDevPct: number;
    restitutionStdDev: number;
  };
  /** Venue air: HVAC drafts and turbulence. */
  air: { gustStdDev: number };
  /** Contact from defenders while shooting. */
  defense: { bumpProbability: number; bumpSpeedStdDev: number; bumpYawStdDevDeg: number };
  /** Ball slips on the wheels / hood at launch: always loses some speed and spin, by a varying amount. */
  wheelSlip: { speedLossMeanPct: number; speedLossStdDevPct: number; spinLossMeanPct: number; spinLossStdDevPct: number };
  /** Frame flex and vibration while the robot drives: extra aim jitter that grows with robot speed. */
  launchVibration: { baseStdDevDeg: number; extraDegPerMps: number };
  /** Ball seats off-center in the shooter (sideways offset at release). */
  seating: { lateralStdDev: number };
  /** Venue air density (temperature, humidity, altitude drift between matches). */
  airDensity: { stdDevPct: number };
  /** Floor and wall randomness per bounce: tile seams, dust, scuffs, ball seam orientation. */
  surface: { restitutionStdDev: number; frictionStdDev: number; bounceAngleStdDevDeg: number };
}

export type ErrorSourceKey = keyof ShotErrorModel;

export const ERROR_SOURCES: { key: ErrorSourceKey; label: string; description: string }[] = [
  { key: "exitSpeed", label: "Exit speed", description: "Shot-to-shot flywheel/contact speed variation" },
  { key: "battery", label: "Battery voltage", description: "Voltage sag and drift changing flywheel speed" },
  { key: "flywheelRecovery", label: "Flywheel recovery", description: "Speed drop between rapid-fire shots" },
  { key: "hoodAngle", label: "Hood angle", description: "Launch elevation slop, resolution, and bias" },
  { key: "turretYaw", label: "Turret / aim", description: "Shooter yaw error and bias" },
  { key: "spin", label: "Spin", description: "Spin rate variation and spin axis tilt" },
  { key: "releaseLatency", label: "Release latency", description: "Command-to-release delay while moving" },
  { key: "localization", label: "Localization", description: "Pose estimate error (position + heading)" },
  { key: "velocityEstimate", label: "Velocity estimate", description: "Shoot-on-the-move velocity error" },
  { key: "launchHeight", label: "Launch height", description: "Exit point height variation" },
  { key: "gamePiece", label: "Game piece", description: "Mass, size, drag, grip and bounce variation" },
  { key: "air", label: "Air", description: "Drafts and turbulence in the venue" },
  { key: "defense", label: "Defense", description: "Getting bumped while shooting" },
  { key: "wheelSlip", label: "Wheel slip", description: "Ball slips on the wheels: lost speed and spin" },
  { key: "launchVibration", label: "Launch vibration", description: "Frame flex and shake while driving" },
  { key: "seating", label: "Ball seating", description: "Ball sits off-center in the shooter" },
  { key: "airDensity", label: "Air density", description: "Temperature, humidity and altitude changing drag and lift" },
  { key: "surface", label: "Floor & walls", description: "Bounce randomness from seams, dust and scuffs" }
];

/** Everything off: a perfect robot. */
export const ZERO_ERROR_MODEL: ShotErrorModel = {
  exitSpeed: { stdDevPct: 0 },
  battery: { nominalVolts: 13, stdDevVolts: 0, referenceVolts: 13, speedPctPerVolt: 0 },
  flywheelRecovery: { dropPct: 0, recoveryTauSec: 0.3, shotIntervalSec: 0.5 },
  hoodAngle: { stdDevDeg: 0, biasDeg: 0 },
  turretYaw: { stdDevDeg: 0, biasDeg: 0 },
  spin: { rpmStdDevPct: 0, axisTiltStdDevDeg: 0 },
  releaseLatency: { meanMs: 0, stdDevMs: 0, compensatedMs: 0 },
  localization: { xyStdDev: 0, headingStdDevDeg: 0 },
  velocityEstimate: { stdDev: 0 },
  launchHeight: { stdDev: 0 },
  gamePiece: { massStdDevPct: 0, radiusStdDev: 0, dragStdDevPct: 0, magnusStdDevPct: 0, restitutionStdDev: 0 },
  air: { gustStdDev: 0 },
  defense: { bumpProbability: 0, bumpSpeedStdDev: 0, bumpYawStdDevDeg: 0 },
  wheelSlip: { speedLossMeanPct: 0, speedLossStdDevPct: 0, spinLossMeanPct: 0, spinLossStdDevPct: 0 },
  launchVibration: { baseStdDevDeg: 0, extraDegPerMps: 0 },
  seating: { lateralStdDev: 0 },
  airDensity: { stdDevPct: 0 },
  surface: { restitutionStdDev: 0, frictionStdDev: 0, bounceAngleStdDevDeg: 0 }
};

/**
 * Typical starting values for a decent FTC shooter. These are ESTIMATES: replace each with numbers
 * measured on your robot (log flywheel RPM, battery voltage, pose error vs. tape marks, etc).
 */
export const TYPICAL_ERROR_MODEL: ShotErrorModel = {
  exitSpeed: { stdDevPct: 2 },
  battery: { nominalVolts: 12.6, stdDevVolts: 0.4, referenceVolts: 12.6, speedPctPerVolt: 1.5 },
  flywheelRecovery: { dropPct: 6, recoveryTauSec: 0.25, shotIntervalSec: 0.4 },
  hoodAngle: { stdDevDeg: 0.75, biasDeg: 0 },
  turretYaw: { stdDevDeg: 0.75, biasDeg: 0 },
  spin: { rpmStdDevPct: 5, axisTiltStdDevDeg: 4 },
  releaseLatency: { meanMs: 40, stdDevMs: 15, compensatedMs: 40 },
  localization: { xyStdDev: 0.03, headingStdDevDeg: 1 },
  velocityEstimate: { stdDev: 0.08 },
  launchHeight: { stdDev: 0.005 },
  gamePiece: { massStdDevPct: 3, radiusStdDev: 0.001, dragStdDevPct: 5, magnusStdDevPct: 10, restitutionStdDev: 0.05 },
  air: { gustStdDev: 0.15 },
  defense: { bumpProbability: 0.05, bumpSpeedStdDev: 0.4, bumpYawStdDevDeg: 5 },
  wheelSlip: { speedLossMeanPct: 1.5, speedLossStdDevPct: 1.5, spinLossMeanPct: 6, spinLossStdDevPct: 6 },
  launchVibration: { baseStdDevDeg: 0.15, extraDegPerMps: 0.6 },
  seating: { lateralStdDev: 0.004 },
  airDensity: { stdDevPct: 3 },
  surface: { restitutionStdDev: 0.04, frictionStdDev: 0.08, bounceAngleStdDevDeg: 3 }
};

/** Copy of the model with only the given sources enabled. */
export function isolateErrorSources(model: ShotErrorModel, keys: ErrorSourceKey[]): ShotErrorModel {
  const out = structuredClone(ZERO_ERROR_MODEL) as ShotErrorModel;
  for (const key of keys) {
    (out as any)[key] = structuredClone(model[key]);
  }
  // Battery: keep the nominal operating point even when disabled so "off" means "no variation"
  if (!keys.includes("battery")) {
    out.battery.nominalVolts = model.battery.referenceVolts;
    out.battery.referenceVolts = model.battery.referenceVolts;
  }
  return out;
}

export interface ShotScenario {
  robot: RobotState2d;
  shooter: ShooterConfig;
  piece: GamePieceProperties;
  environment: EnvironmentProperties;
  field: FieldGeometry;
  target: TargetZone;
}

export interface SampledShot {
  robot: RobotState2d;
  shooter: ShooterConfig;
  piece: GamePieceProperties;
  /** Which shot in a rapid-fire sequence (0 = first). */
  shotIndex: number;
  bumped: boolean;
}

/** Rotates v about a unit axis by angle (Rodrigues). */
function rotateAbout(v: Vector3, axis: Vector3, angle: number): Vector3 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const kv = cross(axis, v);
  const kdv = dot(axis, v);
  return [
    v[0] * c + kv[0] * s + axis[0] * kdv * (1 - c),
    v[1] * c + kv[1] * s + axis[1] * kdv * (1 - c),
    v[2] * c + kv[2] * s + axis[2] * kdv * (1 - c)
  ];
}

/** Draws one perturbed shot from the error model. */
export function sampleShot(scenario: ShotScenario, model: ShotErrorModel, rng: Random, shotIndex = 0): SampledShot {
  const robot = { ...scenario.robot };
  const shooter = { ...scenario.shooter, offset: [...scenario.shooter.offset] as Vector3 };
  const piece = { ...scenario.piece };

  // --- Exit speed: random ripple, battery, rapid-fire droop ---
  let speedFactor = 1 + rng.gaussian(0, model.exitSpeed.stdDevPct / 100);
  const volts = rng.gaussian(model.battery.nominalVolts, model.battery.stdDevVolts);
  speedFactor *= 1 + ((volts - model.battery.referenceVolts) * model.battery.speedPctPerVolt) / 100;
  if (shotIndex > 0 && model.flywheelRecovery.dropPct > 0) {
    // Steady-state deficit for a periodic train of drops that each recover exponentially
    const decay = Math.exp(
      -model.flywheelRecovery.shotIntervalSec / Math.max(1e-6, model.flywheelRecovery.recoveryTauSec)
    );
    const deficit = (model.flywheelRecovery.dropPct / 100) * decay * ((1 - Math.pow(decay, shotIndex)) / (1 - decay));
    speedFactor *= 1 - deficit;
  }
  shooter.exitSpeed *= Math.max(0, speedFactor);

  // --- Aim ---
  shooter.hoodAngleDeg += model.hoodAngle.biasDeg + rng.gaussian(0, model.hoodAngle.stdDevDeg);
  shooter.turretYawDeg += model.turretYaw.biasDeg + rng.gaussian(0, model.turretYaw.stdDevDeg);
  shooter.offset[2] += rng.gaussian(0, model.launchHeight.stdDev);

  // --- Spin rate ---
  shooter.spinRpm *= Math.max(0, 1 + rng.gaussian(0, model.spin.rpmStdDevPct / 100));

  // --- Wheel slip: the ball never quite picks up the wheel's full speed / spin ---
  const ws = model.wheelSlip;
  if (ws.speedLossMeanPct > 0 || ws.speedLossStdDevPct > 0) {
    const loss = Math.min(0.6, Math.max(0, rng.gaussian(ws.speedLossMeanPct, ws.speedLossStdDevPct) / 100));
    shooter.exitSpeed *= 1 - loss;
  }
  if (ws.spinLossMeanPct > 0 || ws.spinLossStdDevPct > 0) {
    const loss = Math.min(1, Math.max(0, rng.gaussian(ws.spinLossMeanPct, ws.spinLossStdDevPct) / 100));
    shooter.spinRpm *= 1 - loss;
  }

  // --- Ball seated off-center ---
  shooter.offset[1] += rng.gaussian(0, model.seating.lateralStdDev);

  // --- Where the robot really is vs. where it thinks it is ---
  robot.x += rng.gaussian(0, model.localization.xyStdDev);
  robot.y += rng.gaussian(0, model.localization.xyStdDev);
  robot.heading += rng.gaussian(0, model.localization.headingStdDevDeg) * DEG;

  const moving = Math.hypot(robot.vx, robot.vy) > 0.05 || Math.abs(robot.omega) > 0.05;
  if (moving) {
    robot.vx += rng.gaussian(0, model.velocityEstimate.stdDev);
    robot.vy += rng.gaussian(0, model.velocityEstimate.stdDev);
  }

  // --- Launch vibration: shake grows with how fast the robot is driving / turning ---
  {
    const lv = model.launchVibration;
    const speed = Math.hypot(robot.vx, robot.vy) + Math.abs(robot.omega) * 0.2;
    const jitterDeg = lv.baseStdDevDeg + lv.extraDegPerMps * speed;
    if (jitterDeg > 0) {
      shooter.hoodAngleDeg += rng.gaussian(0, jitterDeg);
      shooter.turretYawDeg += rng.gaussian(0, jitterDeg);
    }
  }

  // --- Defense bump ---
  const bumped = rng.chance(model.defense.bumpProbability);
  if (bumped) {
    robot.vx += rng.gaussian(0, model.defense.bumpSpeedStdDev);
    robot.vy += rng.gaussian(0, model.defense.bumpSpeedStdDev);
    robot.heading += rng.gaussian(0, model.defense.bumpYawStdDevDeg) * DEG;
  }

  // --- Release latency: robot drifts during the un-compensated part of the delay ---
  const latency =
    (model.releaseLatency.meanMs -
      model.releaseLatency.compensatedMs +
      rng.gaussian(0, model.releaseLatency.stdDevMs)) /
    1000;
  robot.x += robot.vx * latency;
  robot.y += robot.vy * latency;
  robot.heading += robot.omega * latency;

  // --- Game piece variation ---
  const gp = model.gamePiece;
  piece.mass *= Math.max(0.1, 1 + rng.gaussian(0, gp.massStdDevPct / 100));
  piece.radius = Math.max(0.005, piece.radius + rng.gaussian(0, gp.radiusStdDev));
  piece.dragCoefficient *= Math.max(0, 1 + rng.gaussian(0, gp.dragStdDevPct / 100));
  piece.magnusScale *= Math.max(0, 1 + rng.gaussian(0, gp.magnusStdDevPct / 100));
  const restitutionDelta = rng.gaussian(0, gp.restitutionStdDev);
  piece.floorRestitution = clamp01(piece.floorRestitution + restitutionDelta);
  piece.wallRestitution = clamp01(piece.wallRestitution + restitutionDelta);

  return { robot, shooter, piece, shotIndex, bumped };
}

function clamp01(x: number): number {
  return Math.min(1, Math.max(0, x));
}

/** Simulates one sampled shot. Spin axis tilt is applied here because it needs the launch direction. */
export function simulateSampledShot(
  sample: SampledShot,
  scenario: ShotScenario,
  model: ShotErrorModel,
  rng: Random,
  options: Partial<SimulationOptions>
): TrajectoryResult {
  const environment =
    model.airDensity.stdDevPct > 0
      ? {
          ...scenario.environment,
          airDensity: Math.max(
            0.3,
            scenario.environment.airDensity * (1 + rng.gaussian(0, model.airDensity.stdDevPct / 100))
          )
        }
      : scenario.environment;
  const engine = new PhysicsEngine(sample.piece, environment, scenario.field);
  const sf = model.surface;
  if (sf.restitutionStdDev > 0 || sf.frictionStdDev > 0 || sf.bounceAngleStdDevDeg > 0) {
    engine.contactNoise = {
      rng: rng,
      restitutionStdDev: sf.restitutionStdDev,
      frictionStdDev: sf.frictionStdDev,
      bounceAngleStdDevDeg: sf.bounceAngleStdDevDeg
    };
  }
  const initial = computeLaunchState(sample.robot, sample.shooter);
  if (model.spin.axisTiltStdDevDeg > 0 && length(initial.angularVelocity) > 0) {
    const axis = launchDirection(sample.robot, sample.shooter);
    initial.angularVelocity = rotateAbout(
      initial.angularVelocity,
      axis,
      rng.gaussian(0, model.spin.axisTiltStdDevDeg) * DEG
    );
  }
  const gust = model.air.gustStdDev;
  const gustSampler =
    gust > 0 ? () => [rng.gaussian(0, gust), rng.gaussian(0, gust), rng.gaussian(0, gust / 2)] as Vector3 : null;
  return engine.simulate(initial, options, scenario.target, gustSampler);
}

// ---------------------------------------------------------------------------------------------
// Monte Carlo
// ---------------------------------------------------------------------------------------------

export interface MonteCarloOptions {
  samples: number;
  seed: number;
  /** Shots per rapid-fire burst (1 = single shots). Shot index cycles 0..burstSize-1. */
  burstSize: number;
  /** How many full trajectories to keep for drawing. */
  keepTrajectories: number;
  simulation: Partial<SimulationOptions>;
}

export const DEFAULT_MONTE_CARLO_OPTIONS: MonteCarloOptions = {
  samples: 300,
  seed: 23511,
  burstSize: 1,
  keepTrajectories: 40,
  // Keep simulating past target entry so every shot gets an impact-plane crossing for dispersion stats
  simulation: { timeStep: 0.002, sampleInterval: 0.02, duration: 3, stopOnTargetHit: false, stopAfterLanding: true }
};

/** Height of the impact plane used for dispersion: the target's vertical center. */
export function targetPlaneHeight(target: TargetZone): number {
  // A target entered falling (basket / CELL opening) is measured at its opening (top face);
  // anything else at its vertical middle
  return target.requireDescending ? target.max[2] : (target.min[2] + target.max[2]) / 2;
}

/** Shot spread where the shots cross the target plane. */
export interface Dispersion {
  /** Shots that reached the plane (descending). */
  count: number;
  /** Mean crossing point minus target center (systematic error), m. */
  bias: [number, number];
  /** Std dev along field X / Y, m. */
  stdDev: [number, number];
  /** Std dev along / across the shot direction, m (range error vs. aim error). */
  stdDevRange: number;
  stdDevLateral: number;
  /** Total variance (sx^2 + sy^2), m^2. Independent sources roughly add in variance. */
  variance: number;
}

export interface Stats {
  mean: number;
  stdDev: number;
  p50: number;
  p90: number;
  min: number;
  max: number;
}

export interface MonteCarloShot {
  hit: boolean;
  missDistance: number;
  landing: Vector3 | null;
  entrySpeed: number | null;
  /** Entry angle below horizontal, deg (null if missed). */
  entryAngleDeg: number | null;
  flightTime: number | null;
  apexHeight: number;
  /** Where the shot crossed the target plane, or null if it never got that high. */
  crossing: Vector3 | null;
  exitSpeed: number;
  shotIndex: number;
  bumped: boolean;
  trajectory: TrajectoryResult | null;
}

export interface MonteCarloResult {
  samples: number;
  hits: number;
  hitRate: number;
  /** 95% Wilson score interval on the hit rate. */
  hitRateInterval: [number, number];
  missDistance: Stats;
  entrySpeed: Stats | null;
  entryAngleDeg: Stats | null;
  apexHeight: Stats;
  dispersion: Dispersion;
  shots: MonteCarloShot[];
}

export function computeDispersion(shots: MonteCarloShot[], scenario: ShotScenario): Dispersion {
  const pts = shots.filter((s) => s.crossing !== null).map((s) => s.crossing!);
  const cx = (scenario.target.min[0] + scenario.target.max[0]) / 2;
  const cy = (scenario.target.min[1] + scenario.target.max[1]) / 2;
  if (pts.length < 2) {
    return {
      count: pts.length,
      bias: [NaN, NaN],
      stdDev: [NaN, NaN],
      stdDevRange: NaN,
      stdDevLateral: NaN,
      variance: NaN
    };
  }
  const mx = pts.reduce((a, p) => a + p[0], 0) / pts.length;
  const my = pts.reduce((a, p) => a + p[1], 0) / pts.length;
  // Shot direction: from robot to target center in the floor plane
  const dx = cx - scenario.robot.x;
  const dy = cy - scenario.robot.y;
  const dl = Math.hypot(dx, dy) || 1;
  const ux = dx / dl;
  const uy = dy / dl;
  let sxx = 0,
    syy = 0,
    srr = 0,
    sll = 0;
  for (const p of pts) {
    const ex = p[0] - mx;
    const ey = p[1] - my;
    sxx += ex * ex;
    syy += ey * ey;
    const along = ex * ux + ey * uy;
    const across = -ex * uy + ey * ux;
    srr += along * along;
    sll += across * across;
  }
  const n = pts.length - 1;
  return {
    count: pts.length,
    bias: [mx - cx, my - cy],
    stdDev: [Math.sqrt(sxx / n), Math.sqrt(syy / n)],
    stdDevRange: Math.sqrt(srr / n),
    stdDevLateral: Math.sqrt(sll / n),
    variance: (sxx + syy) / n
  };
}

export function computeStats(values: number[]): Stats {
  if (values.length === 0) return { mean: NaN, stdDev: NaN, p50: NaN, p90: NaN, min: NaN, max: NaN };
  const sorted = [...values].sort((a, b) => a - b);
  const mean = values.reduce((a, b) => a + b, 0) / values.length;
  const variance = values.reduce((a, b) => a + (b - mean) * (b - mean), 0) / Math.max(1, values.length - 1);
  const pct = (p: number) => sorted[Math.min(sorted.length - 1, Math.floor(p * (sorted.length - 1) + 0.5))];
  return {
    mean,
    stdDev: Math.sqrt(variance),
    p50: pct(0.5),
    p90: pct(0.9),
    min: sorted[0],
    max: sorted[sorted.length - 1]
  };
}

export function wilsonInterval(successes: number, n: number, z = 1.96): [number, number] {
  if (n === 0) return [0, 1];
  const p = successes / n;
  const denom = 1 + (z * z) / n;
  const center = (p + (z * z) / (2 * n)) / denom;
  const half = (z * Math.sqrt((p * (1 - p)) / n + (z * z) / (4 * n * n))) / denom;
  return [Math.max(0, center - half), Math.min(1, center + half)];
}

export function runMonteCarlo(
  scenario: ShotScenario,
  model: ShotErrorModel,
  options: Partial<MonteCarloOptions> = {}
): MonteCarloResult {
  const opts = { ...DEFAULT_MONTE_CARLO_OPTIONS, ...options };
  opts.simulation = {
    ...DEFAULT_MONTE_CARLO_OPTIONS.simulation,
    ...options.simulation,
    crossingHeight: targetPlaneHeight(scenario.target)
  };
  const rng = new Random(opts.seed);
  const shots: MonteCarloShot[] = [];

  for (let i = 0; i < opts.samples; i++) {
    const shotIndex = i % Math.max(1, opts.burstSize);
    const sample = sampleShot(scenario, model, rng, shotIndex);
    const result = simulateSampledShot(sample, scenario, model, rng, opts.simulation);
    const target = result.target!;
    let entrySpeed: number | null = null;
    let entryAngleDeg: number | null = null;
    if (target.hit && target.entryVelocity !== null) {
      entrySpeed = length(target.entryVelocity);
      const horiz = Math.hypot(target.entryVelocity[0], target.entryVelocity[1]);
      entryAngleDeg = (Math.atan2(-target.entryVelocity[2], horiz) * 180) / Math.PI;
    }
    shots.push({
      hit: target.hit,
      missDistance: target.closestDistance,
      landing: result.landing?.position ?? null,
      entrySpeed,
      entryAngleDeg,
      flightTime: target.hit ? target.time : result.landing?.time ?? null,
      apexHeight: result.apex.position[2],
      crossing: result.crossing,
      exitSpeed: sample.shooter.exitSpeed,
      shotIndex,
      bumped: sample.bumped,
      trajectory: i < opts.keepTrajectories ? result : null
    });
  }

  const hits = shots.filter((s) => s.hit).length;
  const entrySpeeds = shots.filter((s) => s.entrySpeed !== null).map((s) => s.entrySpeed!);
  const entryAngles = shots.filter((s) => s.entryAngleDeg !== null).map((s) => s.entryAngleDeg!);
  return {
    samples: shots.length,
    hits,
    hitRate: shots.length === 0 ? 0 : hits / shots.length,
    hitRateInterval: wilsonInterval(hits, shots.length),
    missDistance: computeStats(shots.map((s) => s.missDistance)),
    entrySpeed: entrySpeeds.length > 0 ? computeStats(entrySpeeds) : null,
    entryAngleDeg: entryAngles.length > 0 ? computeStats(entryAngles) : null,
    apexHeight: computeStats(shots.map((s) => s.apexHeight)),
    dispersion: computeDispersion(shots, scenario),
    shots
  };
}

export interface ErrorBudgetEntry {
  key: ErrorSourceKey | "all" | "none";
  label: string;
  hitRate: number;
  hitRateInterval: [number, number];
  /** Hit rate lost vs. a perfect robot when only this source is active. */
  hitRateCost: number;
  missDistanceStdDev: number;
  /** Spread at the target plane with only this source active. */
  dispersion: Dispersion;
  /** This source's share of the summed per-source variance (0-1). Shows what to fix first even at 100% hit rate. */
  varianceShare: number;
}

/**
 * Error budget: runs the scenario with no error, each source alone, and everything together.
 * Sorted by cost so the biggest problem is first.
 */
export function analyzeErrorBudget(
  scenario: ShotScenario,
  model: ShotErrorModel,
  options: Partial<MonteCarloOptions> = {}
): ErrorBudgetEntry[] {
  const base = { ...options, keepTrajectories: 0 };
  const toEntry = (
    key: ErrorBudgetEntry["key"],
    label: string,
    r: MonteCarloResult,
    perfect: number
  ): ErrorBudgetEntry => ({
    key,
    label,
    hitRate: r.hitRate,
    hitRateInterval: r.hitRateInterval,
    hitRateCost: perfect - r.hitRate,
    missDistanceStdDev: r.missDistance.stdDev,
    dispersion: r.dispersion,
    varianceShare: NaN
  });

  // The perfect robot is deterministic except for the RNG-free path, so 2 samples suffice
  const none = runMonteCarlo(scenario, ZERO_ERROR_MODEL, { ...base, samples: 2 });
  const perfect = none.hitRate;
  const entries: ErrorBudgetEntry[] = ERROR_SOURCES.map((source) =>
    toEntry(source.key, source.label, runMonteCarlo(scenario, isolateErrorSources(model, [source.key]), base), perfect)
  );
  const totalVariance = entries.reduce(
    (sum, e) => sum + (isFinite(e.dispersion.variance) ? e.dispersion.variance : 0),
    0
  );
  for (const e of entries) {
    e.varianceShare = totalVariance > 0 && isFinite(e.dispersion.variance) ? e.dispersion.variance / totalVariance : 0;
  }
  entries.sort((a, b) => b.hitRateCost - a.hitRateCost || b.varianceShare - a.varianceShare);
  return [
    toEntry("none", "Perfect robot", none, perfect),
    toEntry("all", "All sources", runMonteCarlo(scenario, model, base), perfect),
    ...entries
  ];
}

export function defaultScenario(robot: RobotState2d, shooter: ShooterConfig, target: TargetZone): ShotScenario {
  return { robot, shooter, piece: DEFAULT_GAME_PIECE, environment: DEFAULT_ENVIRONMENT, field: FTC_FIELD, target };
}
