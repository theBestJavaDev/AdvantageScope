// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Converts a robot state + shooter setup into the game piece's initial state.
 *
 * Robot frame: +X forward, +Y left, +Z up. Field frame: FTC center origin, +Z up.
 */

import { BallState } from "./PhysicsEngine";
import { Vector3, add, cross, normalize, rotateZ, scale } from "./vector";

export const RPM_TO_RAD_PER_SEC = (2 * Math.PI) / 60;

/**
 * Spin direction relative to the launch direction.
 *  - backspin: bottom of the piece moves forward; Magnus pushes UP (flatter, longer shots; bounces "check")
 *  - topspin: top of the piece moves forward; Magnus pushes DOWN (steeper drop; bounces "kick" forward)
 *  - sidespin-left / sidespin-right: Magnus curves the shot left / right
 */
export type SpinMode = "none" | "backspin" | "topspin" | "sidespin-left" | "sidespin-right";

export const SPIN_MODES: SpinMode[] = ["none", "backspin", "topspin", "sidespin-left", "sidespin-right"];

export interface ShooterConfig {
  /** Exit point of the game piece in the robot frame, m. */
  offset: Vector3;
  /** Launch elevation above horizontal, deg (hood angle). */
  hoodAngleDeg: number;
  /** Shooter yaw relative to the robot's forward direction, deg (turret angle; 0 for fixed). */
  turretYawDeg: number;
  /** Game piece speed relative to the robot as it leaves the shooter, m/s. */
  exitSpeed: number;
  spinMode: SpinMode;
  /** Game piece spin rate, RPM (magnitude; direction comes from spinMode). */
  spinRpm: number;
  /** Add the robot's own velocity (incl. rotation) to the shot. True for physical accuracy. */
  inheritRobotVelocity: boolean;
}

export interface RobotState2d {
  /** Field position, m. */
  x: number;
  y: number;
  /** Field heading, rad (CCW from +X). */
  heading: number;
  /** Field-relative velocity, m/s. */
  vx: number;
  vy: number;
  /** Yaw rate, rad/s. */
  omega: number;
}

export const DEFAULT_SHOOTER: ShooterConfig = {
  offset: [0.1, 0, 0.3],
  hoodAngleDeg: 45,
  turretYawDeg: 0,
  exitSpeed: 6,
  spinMode: "backspin",
  spinRpm: 1500,
  inheritRobotVelocity: true
};

/** Unit vector the shooter points along, in the field frame. */
export function launchDirection(robot: RobotState2d, shooter: ShooterConfig): Vector3 {
  const yaw = robot.heading + (shooter.turretYawDeg * Math.PI) / 180;
  const pitch = (shooter.hoodAngleDeg * Math.PI) / 180;
  return [Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)];
}

/** World-frame angular velocity for a spin mode about the given launch direction. */
export function spinVector(direction: Vector3, mode: SpinMode, rpm: number): Vector3 {
  const rate = Math.abs(rpm) * RPM_TO_RAD_PER_SEC;
  if (mode === "none" || rate === 0) return [0, 0, 0];
  // Horizontal axis perpendicular to the shot; w = rate * (d x Z) gives w x v pointing up (backspin lift)
  let backspinAxis = normalize(cross(direction, [0, 0, 1]));
  if (backspinAxis[0] === 0 && backspinAxis[1] === 0 && backspinAxis[2] === 0) {
    backspinAxis = [0, -1, 0]; // Straight-up shot: pick an arbitrary horizontal axis
  }
  switch (mode) {
    case "backspin":
      return scale(backspinAxis, rate);
    case "topspin":
      return scale(backspinAxis, -rate);
    case "sidespin-left":
      return [0, 0, rate];
    case "sidespin-right":
      return [0, 0, -rate];
  }
}

/** Builds the initial game piece state for a shot. */
export function computeLaunchState(robot: RobotState2d, shooter: ShooterConfig): BallState {
  const offsetField = rotateZ(shooter.offset, robot.heading);
  const position: Vector3 = [robot.x + offsetField[0], robot.y + offsetField[1], offsetField[2]];

  const direction = launchDirection(robot, shooter);
  let velocity = scale(direction, shooter.exitSpeed);
  if (shooter.inheritRobotVelocity) {
    // Translation + tangential velocity of the exit point from robot rotation
    const tangential = cross([0, 0, robot.omega], [offsetField[0], offsetField[1], 0]);
    velocity = add(velocity, [robot.vx + tangential[0], robot.vy + tangential[1], 0]);
  }

  return {
    position,
    velocity,
    angularVelocity: spinVector(direction, shooter.spinMode, shooter.spinRpm),
    orientation: [1, 0, 0, 0],
    time: 0
  };
}

// ---------------------------------------------------------------------------------------------
// Flywheel helpers: estimate exit speed + spin from wheel speeds instead of guessing them
// ---------------------------------------------------------------------------------------------

/**
 * Single flywheel against a fixed hood. The piece rolls on the hood, so its center moves at about
 * half the wheel surface speed and it spins at surfaceSpeed / (2 r). Spin direction depends on which
 * side the wheel is on:
 *  - wheel below, hood above: bottom surface is driven forward, top is held back -> backspin
 *  - wheel above, hood below: top surface is driven forward -> topspin
 *
 * efficiency (0-1) lumps slip and compression losses; fit it from measured exit speeds.
 */
export function hoodedFlywheelLaunch(
  wheelRpm: number,
  wheelRadius: number,
  pieceRadius: number,
  wheelPosition: "below" | "above",
  efficiency = 0.8
): { exitSpeed: number; spinMode: SpinMode; spinRpm: number } {
  const surfaceSpeed = Math.abs(wheelRpm) * RPM_TO_RAD_PER_SEC * wheelRadius * efficiency;
  const spinRadPerSec = surfaceSpeed / (2 * pieceRadius);
  return {
    exitSpeed: surfaceSpeed / 2,
    spinMode: wheelPosition === "below" ? "backspin" : "topspin",
    spinRpm: spinRadPerSec / RPM_TO_RAD_PER_SEC
  };
}

/**
 * Two flywheels (top + bottom). Exit speed is the average surface speed; a speed difference
 * produces spin: bottom faster -> backspin, top faster -> topspin.
 */
export function dualFlywheelLaunch(
  topRpm: number,
  bottomRpm: number,
  wheelRadius: number,
  pieceRadius: number,
  efficiency = 0.85
): { exitSpeed: number; spinMode: SpinMode; spinRpm: number } {
  const vTop = Math.abs(topRpm) * RPM_TO_RAD_PER_SEC * wheelRadius * efficiency;
  const vBottom = Math.abs(bottomRpm) * RPM_TO_RAD_PER_SEC * wheelRadius * efficiency;
  const spinRadPerSec = Math.abs(vBottom - vTop) / (2 * pieceRadius);
  return {
    exitSpeed: (vTop + vBottom) / 2,
    spinMode: vBottom > vTop ? "backspin" : vTop > vBottom ? "topspin" : "none",
    spinRpm: spinRadPerSec / RPM_TO_RAD_PER_SEC
  };
}
