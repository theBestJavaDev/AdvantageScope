// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Fits game piece properties to measurements, so the sim stops using guessed numbers.
 *
 *  - Bounciness from a drop test: e = sqrt(bounce height / drop height) (drag on a ~1 m drop is tiny).
 *  - Drag coefficient from filmed shots: for shots with a measured exit speed, hood angle and launch
 *    height, find the Cd whose simulated landing distances best match the measured ones (golden-
 *    section search on the summed squared error; landing distance falls monotonically with Cd).
 */

import { DEFAULT_ENVIRONMENT, FTC_FIELD, GamePieceProperties, PhysicsEngine } from "./PhysicsEngine";
import { RobotState2d, ShooterConfig, SpinMode, computeLaunchState } from "./ShotLaunch";

export interface MeasuredShot {
  /** Exit speed measured from video (m/s). */
  exitSpeed: number;
  hoodAngleDeg: number;
  /** Height the piece leaves the robot (m). */
  launchHeight: number;
  /** Horizontal distance from the exit point to where the piece first touched the floor (m). */
  landingDistance: number;
}

export function isUsableShot(s: MeasuredShot): boolean {
  return (
    s.exitSpeed > 0.5 && s.landingDistance > 0.1 && s.hoodAngleDeg > 0 && s.hoodAngleDeg < 90 && s.launchHeight >= 0
  );
}

/** Restitution from a drop test. Returns null for impossible inputs. */
export function fitRestitution(dropHeight: number, bounceHeight: number): number | null {
  if (!(dropHeight > 0) || !(bounceHeight >= 0) || bounceHeight > dropHeight) return null;
  return Math.sqrt(bounceHeight / dropHeight);
}

/** Simulated landing distance for one measured shot with the given piece. */
export function simulateLandingDistance(
  piece: GamePieceProperties,
  shot: MeasuredShot,
  spin: { mode: SpinMode; rpm: number }
): number | null {
  // Fire from the far wall toward the other wall so the flight has the whole field
  const robot: RobotState2d = { x: -FTC_FIELD.halfX + 0.05, y: 0, heading: 0, vx: 0, vy: 0, omega: 0 };
  const shooter: ShooterConfig = {
    offset: [0, 0, shot.launchHeight],
    hoodAngleDeg: shot.hoodAngleDeg,
    turretYawDeg: 0,
    exitSpeed: shot.exitSpeed,
    spinMode: spin.mode,
    spinRpm: spin.rpm,
    inheritRobotVelocity: false
  };
  // Wide virtual field so long calibration shots aren't stopped by the real perimeter
  const engine = new PhysicsEngine(piece, DEFAULT_ENVIRONMENT, { halfX: 50, halfY: 50, wallHeight: 0 });
  const start = computeLaunchState({ ...robot, x: 0 }, shooter);
  const result = engine.simulate(start, { duration: 6, timeStep: 0.001, sampleInterval: 0.05, stopAfterLanding: true });
  if (result.landing === null) return null;
  return result.landing.position[0] - start.position[0];
}

export interface DragFit {
  dragCoefficient: number;
  /** RMS landing error at the fitted Cd (m). */
  rmsError: number;
  perShot: { measured: number; simulated: number }[];
  /** True if the best Cd sits at a search bound (measurements may be inconsistent). */
  atBound: boolean;
}

/** Fits the drag coefficient to measured shots. Null if there are no usable shots. */
export function fitDragCoefficient(
  piece: GamePieceProperties,
  shots: MeasuredShot[],
  spin: { mode: SpinMode; rpm: number },
  bounds: [number, number] = [0.0, 2.5]
): DragFit | null {
  const usable = shots.filter(isUsableShot);
  if (usable.length === 0) return null;
  const cost = (cd: number) => {
    let sum = 0;
    for (const s of usable) {
      const d = simulateLandingDistance({ ...piece, dragCoefficient: cd }, s, spin);
      sum += d === null ? 100 : (d - s.landingDistance) ** 2;
    }
    return sum;
  };
  // Golden-section search
  const phi = (Math.sqrt(5) - 1) / 2;
  let [a, b] = bounds;
  let c = b - phi * (b - a);
  let d = a + phi * (b - a);
  let fc = cost(c);
  let fd = cost(d);
  for (let i = 0; i < 40 && b - a > 1e-4; i++) {
    if (fc < fd) {
      b = d;
      d = c;
      fd = fc;
      c = b - phi * (b - a);
      fc = cost(c);
    } else {
      a = c;
      c = d;
      fc = fd;
      d = a + phi * (b - a);
      fd = cost(d);
    }
  }
  const cd = (a + b) / 2;
  const perShot = usable.map((s) => ({
    measured: s.landingDistance,
    simulated: simulateLandingDistance({ ...piece, dragCoefficient: cd }, s, spin) ?? NaN
  }));
  const rmsError = Math.sqrt(perShot.reduce((sum, p) => sum + (p.simulated - p.measured) ** 2, 0) / perShot.length);
  const span = bounds[1] - bounds[0];
  return {
    dragCoefficient: cd,
    rmsError,
    perShot,
    atBound: cd - bounds[0] < span * 0.01 || bounds[1] - cd < span * 0.01
  };
}
