// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Shooting from the driving robot: takes the live drive state and the Shot Lab setup, aims
 * (turret or fixed), picks the exit speed, applies the error model, and simulates the flight.
 */

import { DriveState } from "./DriveSim";
import { TrajectoryResult } from "./PhysicsEngine";
import { ZERO_ERROR_MODEL, sampleShot, simulateSampledShot, targetPlaneHeight } from "./ShotErrorModel";
import { ShotLabConfig, buildErrorModel, buildScenario, solveShot } from "./ShotLab";
import { Random } from "./vector";

const DEG = Math.PI / 180;

export type AimMode = "turret" | "fixed";

export interface ShootingOptions {
  /** turret: shooter swings to the solved direction. fixed: shoots wherever the robot points. */
  aim: AimMode;
  /** Solve exit speed for the current distance + motion (like a well-tuned lookup table). */
  autoSpeed: boolean;
  applyError: boolean;
}

export interface ShotSolution {
  valid: boolean;
  exitSpeed: number;
  /** Shooter direction relative to the robot (deg) after aiming. */
  turretYawDeg: number;
  distance: number;
}

export interface LiveShot {
  id: number;
  /** Drive-sim time the shot left the robot (s). */
  fireTime: number;
  trajectory: TrajectoryResult;
  hit: boolean;
  exitSpeed: number;
  turretYawDeg: number;
  /** Robot speed when fired (m/s). */
  robotSpeed: number;
  shotIndex: number;
  bumped: boolean;
}

function wrapDeg(deg: number): number {
  return ((((deg + 180) % 360) + 360) % 360) - 180;
}

/** Shot Lab config with the robot replaced by the live drive state. */
export function configForDriveState(base: ShotLabConfig, drive: DriveState): ShotLabConfig {
  return {
    ...base,
    robot: {
      x: drive.x,
      y: drive.y,
      headingDeg: drive.heading / DEG,
      vx: drive.vx,
      vy: drive.vy,
      omegaDegPerSec: drive.omega / DEG
    },
    shooter: { ...base.shooter }
  };
}

/** Where the shooter would point and how fast it would shoot right now. */
export function computeSolution(base: ShotLabConfig, drive: DriveState, options: ShootingOptions): ShotSolution {
  const config = configForDriveState(base, drive);
  const distance = Math.hypot(config.target.x - drive.x, config.target.y - drive.y);
  let turretYawDeg = config.shooter.turretYawDeg;
  let exitSpeed = config.shooter.exitSpeed;
  let valid = true;
  if (options.autoSpeed || options.aim === "turret") {
    config.autoAim = options.aim === "turret";
    if (options.aim === "turret") {
      // A turret turns the shooter, not the robot: the ball leaves from the same point whatever the aim.
      // Solve from that exit point (offset 0) so turning the robot in the solver doesn't move it, and keep
      // the exit point's extra speed from the robot's own rotation (omega x offset).
      const c = Math.cos(drive.heading);
      const s = Math.sin(drive.heading);
      const ox = config.shooter.offsetX * c;
      const oy = config.shooter.offsetX * s;
      config.robot.x = drive.x + ox;
      config.robot.y = drive.y + oy;
      config.robot.vx = drive.vx - drive.omega * oy;
      config.robot.vy = drive.vy + drive.omega * ox;
      config.robot.omegaDegPerSec = 0;
      config.shooter.offsetX = 0;
    }
    const solution = solveShot(config, 0.01);
    valid = solution.converged;
    if (options.autoSpeed) exitSpeed = solution.exitSpeed;
    if (options.aim === "turret") {
      // solveShot turns the robot; a turret turns the shooter instead, by the same amount
      turretYawDeg = wrapDeg(config.shooter.turretYawDeg + solution.headingDeg - config.robot.headingDeg);
    }
  }
  return { valid, exitSpeed, turretYawDeg, distance };
}

/** Fires one shot from the current drive state. */
export function fireShot(
  base: ShotLabConfig,
  drive: DriveState,
  options: ShootingOptions,
  solution: ShotSolution,
  rng: Random,
  id: number,
  shotIndex: number
): LiveShot {
  const config = configForDriveState(base, drive);
  config.shooter.exitSpeed = solution.exitSpeed;
  config.shooter.turretYawDeg = solution.turretYawDeg;
  const scenario = buildScenario(config);
  const model = options.applyError ? buildErrorModel(config) : ZERO_ERROR_MODEL;
  const sample = sampleShot(scenario, model, rng, shotIndex);
  const trajectory = simulateSampledShot(sample, scenario, model, rng, {
    duration: 5,
    timeStep: 0.002,
    sampleInterval: 1 / 120,
    stopAfterLanding: false,
    stopWhenAtRest: true,
    crossingHeight: targetPlaneHeight(scenario.target)
  });
  return {
    id,
    fireTime: drive.time,
    trajectory,
    hit: trajectory.target?.hit ?? false,
    exitSpeed: sample.shooter.exitSpeed,
    turretYawDeg: solution.turretYawDeg,
    robotSpeed: Math.hypot(drive.vx, drive.vy),
    shotIndex,
    bumped: sample.bumped
  };
}

/** Interpolated piece position `t` seconds after the shot was fired (null once it has stopped). */
export function shotPositionAt(shot: LiveShot, t: number): [number, number, number] | null {
  const samples = shot.trajectory.samples;
  if (samples.length === 0) return null;
  const t0 = samples[0].time;
  const tt = t0 + t;
  if (tt >= samples[samples.length - 1].time) return null;
  // Samples are uniform in time except possibly the last; binary search is cheap and safe
  let lo = 0;
  let hi = samples.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (samples[mid].time <= tt) lo = mid;
    else hi = mid;
  }
  const a = samples[lo];
  const b = samples[hi];
  const f = b.time > a.time ? (tt - a.time) / (b.time - a.time) : 0;
  return [
    a.position[0] + (b.position[0] - a.position[0]) * f,
    a.position[1] + (b.position[1] - a.position[1]) * f,
    a.position[2] + (b.position[2] - a.position[2]) * f
  ];
}

/** Where the piece ends up (rest position). */
export function shotFinalPosition(shot: LiveShot): [number, number, number] {
  const p = shot.trajectory.finalState.position;
  return [p[0], p[1], p[2]];
}
