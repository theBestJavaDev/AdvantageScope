// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Robot driving physics for an FTC-sized robot on the 12 ft field.
 *
 * Model (per step):
 *  1. Driver input (-1..1 sticks) -> desired chassis speeds, field- or robot-centric.
 *  2. Drivetrain kinematics: compute each wheel/module speed, then scale everything down together
 *     if any wheel would exceed its top speed (so turning while driving costs speed, like the real
 *     thing). Mecanum strafing is less efficient than driving forward; tank can't strafe.
 *  3. Traction limit: the chassis velocity vector can only change by maxAccel * dt (tires slip
 *     beyond that), and turn rate by maxAngularAccel * dt.
 *  4. Integrate, then resolve collisions between the robot footprint and the perimeter walls:
 *     push the robot back in and remove the velocity going into the wall.
 *
 * Frames: field +X/+Y with FTC center origin, heading CCW from +X. Velocities in the state are
 * field-frame, matching RobotState2d in ShotLaunch.
 */

import { FTC_FIELD, FieldGeometry } from "./PhysicsEngine";

const DEG = Math.PI / 180;
const INCH = 0.0254;

export type DrivetrainType = "mecanum" | "swerve" | "tank";

export interface DrivetrainConfig {
  type: DrivetrainType;
  /** Top wheel surface speed at nominal voltage (m/s). */
  maxSpeed: number;
  /** Traction-limited acceleration (m/s^2). ~mu * g for the wheels; mecanum is lower. */
  maxAccel: number;
  /** Top turn rate (deg/s); also limited by wheel speed. */
  maxTurnRateDeg: number;
  /** Turn acceleration limit (deg/s^2). */
  maxTurnAccelDeg: number;
  /** Mecanum only: strafe speed as a fraction of forward speed (roller losses). */
  strafeEfficiency: number;
  /** Robot frame size (m). FTC robots must fit an 18 in cube at the start. */
  length: number;
  width: number;
  /** Track width / wheelbase for kinematics (m), wheel-center to wheel-center. */
  trackWidth: number;
  wheelBase: number;
  /** Top speed scales with battery voltage. */
  batteryVolts: number;
  nominalVolts: number;
  /** Voltage drop at full load (hard acceleration at speed). Real FTC packs sag ~1-2 V. 0 = off. */
  batterySagVolts: number;
  /** Resting voltage lost per minute of full-load driving (match-long drain). */
  batteryDrainVoltsPerMin: number;
}

export const DEFAULT_DRIVETRAIN: DrivetrainConfig = {
  type: "mecanum",
  maxSpeed: 1.8,
  maxAccel: 4.0,
  maxTurnRateDeg: 300,
  maxTurnAccelDeg: 1200,
  strafeEfficiency: 0.8,
  length: 18 * INCH,
  width: 18 * INCH,
  trackWidth: 15 * INCH,
  wheelBase: 13 * INCH,
  batteryVolts: 12.6,
  nominalVolts: 12.6,
  batterySagVolts: 1.5,
  batteryDrainVoltsPerMin: 0.4
};

export interface DriveInput {
  /** Forward (+) / back (-), -1..1. Field +X when field-centric. */
  x: number;
  /** Left (+) / right (-), -1..1. Field +Y when field-centric. */
  y: number;
  /** CCW (+) / CW (-), -1..1. */
  turn: number;
}

export interface DriveState {
  x: number;
  y: number;
  heading: number;
  /** Field-frame velocity (m/s). */
  vx: number;
  vy: number;
  /** Turn rate (rad/s). */
  omega: number;
  time: number;
  /** True if the robot touched a wall this step. */
  wallContact: boolean;
  /** Fraction of wheel top speed in use by the busiest wheel (0-1), for the HUD. */
  wheelLoad: number;
  /** Total distance driven (m). */
  distance: number;
  /** Resting battery voltage (drains slowly). NaN = take it from the config on the first step. */
  batteryRest: number;
  /** Voltage under the present load (what the motors see). */
  batteryVolts: number;
}

export function makeDriveState(x: number, y: number, headingDeg: number): DriveState {
  return {
    x,
    y,
    heading: headingDeg * DEG,
    vx: 0,
    vy: 0,
    omega: 0,
    time: 0,
    wallContact: false,
    wheelLoad: 0,
    distance: 0,
    batteryRest: NaN,
    batteryVolts: NaN
  };
}

/** Effective wheel top speed after battery scaling. */
export function effectiveMaxSpeed(config: DrivetrainConfig, volts: number = config.batteryVolts): number {
  const v = isFinite(volts) ? volts : config.batteryVolts;
  return config.maxSpeed * Math.max(0, Math.min(1.2, v / Math.max(1, config.nominalVolts)));
}

/**
 * Wheel-speed desaturation. Takes desired robot-frame speeds (forward vx, left vy, omega) and
 * returns the achievable speeds plus the busiest wheel's load (desired / max, before scaling).
 */
export function desaturate(
  vx: number,
  vy: number,
  omega: number,
  config: DrivetrainConfig,
  volts: number = config.batteryVolts
): { vx: number; vy: number; omega: number; load: number } {
  const vmax = effectiveMaxSpeed(config, volts);
  if (vmax <= 0) return { vx: 0, vy: 0, omega: 0, load: 0 };
  let peak = 0;
  switch (config.type) {
    case "mecanum": {
      // Wheel surface speeds; strafing needs 1/efficiency more wheel speed for the same chassis speed
      const k = (config.trackWidth + config.wheelBase) / 2;
      const s = vy / Math.max(0.05, config.strafeEfficiency);
      const wheels = [vx - s - omega * k, vx + s + omega * k, vx + s - omega * k, vx - s + omega * k];
      peak = Math.max(...wheels.map(Math.abs));
      break;
    }
    case "swerve": {
      // Module speed = |v + omega x r| at each corner module
      const hx = config.wheelBase / 2;
      const hy = config.trackWidth / 2;
      for (const [rx, ry] of [
        [hx, hy],
        [hx, -hy],
        [-hx, hy],
        [-hx, -hy]
      ]) {
        peak = Math.max(peak, Math.hypot(vx - omega * ry, vy + omega * rx));
      }
      break;
    }
    case "tank": {
      vy = 0;
      const half = config.trackWidth / 2;
      peak = Math.max(Math.abs(vx - omega * half), Math.abs(vx + omega * half));
      break;
    }
  }
  const load = peak / vmax;
  const scale = load > 1 ? 1 / load : 1;
  return { vx: vx * scale, vy: vy * scale, omega: omega * scale, load: Math.min(1, load) };
}

function clampMagnitude(x: number, y: number, max: number): [number, number] {
  const m = Math.hypot(x, y);
  return m > max && m > 0 ? [(x / m) * max, (y / m) * max] : [x, y];
}

/** Robot footprint corners in the field frame. */
export function robotCorners(
  state: { x: number; y: number; heading: number },
  config: DrivetrainConfig
): [number, number][] {
  const c = Math.cos(state.heading);
  const s = Math.sin(state.heading);
  const hl = config.length / 2;
  const hw = config.width / 2;
  return [
    [hl, hw],
    [hl, -hw],
    [-hl, -hw],
    [-hl, hw]
  ].map(([lx, ly]) => [state.x + lx * c - ly * s, state.y + lx * s + ly * c]);
}

/**
 * Advances the drive by dt. `fieldCentric` maps stick X/Y to field axes instead of robot axes.
 * dt should be small (<= 10 ms); call repeatedly for bigger frame times.
 */
export function stepDrive(
  state: DriveState,
  input: DriveInput,
  config: DrivetrainConfig,
  fieldCentric: boolean,
  dt: number,
  field: FieldGeometry = FTC_FIELD,
  obstacles: DriveObstacle[] = []
): DriveState {
  // Battery: start from the config on the first step, then follow the simulated pack
  const rest = isFinite(state.batteryRest) ? state.batteryRest : config.batteryVolts;
  const loaded = isFinite(state.batteryVolts) ? state.batteryVolts : rest;
  const vmax = effectiveMaxSpeed(config, loaded);
  const clampInput = (v: number) => Math.max(-1, Math.min(1, isFinite(v) ? v : 0));
  let ix = clampInput(input.x);
  let iy = clampInput(input.y);
  // Diagonal keyboard input shouldn't go faster than straight
  [ix, iy] = clampMagnitude(ix, iy, 1);
  const turn = clampInput(input.turn);

  // 1. Desired speeds in the robot frame
  const c = Math.cos(state.heading);
  const s = Math.sin(state.heading);
  let desVx = ix * vmax;
  let desVy = iy * vmax;
  if (fieldCentric) {
    const fx = desVx;
    const fy = desVy;
    desVx = fx * c + fy * s;
    desVy = -fx * s + fy * c;
  }
  if (config.type === "mecanum") desVy *= config.strafeEfficiency;
  const desOmega = turn * config.maxTurnRateDeg * DEG;

  // 2. Wheel limits
  const sat = desaturate(desVx, desVy, desOmega, config, loaded);

  // Back to the field frame
  const targetVx = sat.vx * c - sat.vy * s;
  const targetVy = sat.vx * s + sat.vy * c;

  // 3. Traction limits
  const [dvx, dvy] = clampMagnitude(targetVx - state.vx, targetVy - state.vy, config.maxAccel * dt);
  let vx = state.vx + dvx;
  let vy = state.vy + dvy;
  if (config.type === "tank") {
    // Tank wheels can't slide sideways: remove the robot-frame lateral velocity (friction does this)
    const forward = vx * c + vy * s;
    vx = forward * c;
    vy = forward * s;
  }
  const maxDOmega = config.maxTurnAccelDeg * DEG * dt;
  const omega = state.omega + Math.max(-maxDOmega, Math.min(maxDOmega, sat.omega - state.omega));

  // Battery sag: current draw grows with how hard the wheels push (traction used) and how fast
  // they spin; the pack voltage follows with a short time constant and the rest voltage drains.
  const tractionUse = Math.min(1, Math.hypot(dvx, dvy) / Math.max(1e-9, config.maxAccel * dt));
  const speedUse = Math.min(1, Math.hypot(vx, vy) / Math.max(1e-9, config.maxSpeed));
  const demand = Math.min(1, 0.65 * tractionUse + 0.35 * Math.max(speedUse, sat.load * 0.5));
  const nextRest = Math.max(0, rest - (config.batteryDrainVoltsPerMin * demand * dt) / 60);
  const targetVolts = nextRest - config.batterySagVolts * demand;
  const nextVolts = loaded + (targetVolts - loaded) * (1 - Math.exp(-dt / 0.08));

  // 4. Integrate (heading at the midpoint keeps arcs accurate)
  let next: DriveState = {
    x: state.x + vx * dt,
    y: state.y + vy * dt,
    heading: state.heading + ((state.omega + omega) / 2) * dt,
    vx,
    vy,
    omega,
    time: state.time + dt,
    wallContact: false,
    wheelLoad: sat.load,
    distance: state.distance + Math.hypot(vx, vy) * dt,
    batteryRest: nextRest,
    batteryVolts: nextVolts
  };
  next.heading = Math.atan2(Math.sin(next.heading), Math.cos(next.heading));
  next = resolveObstacles(next, config, obstacles);
  return resolveWalls(next, config, field);
}

/** Floor-level field element the robot can't drive through (axis-aligned box, m). */
export interface DriveObstacle {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

/**
 * Separating-axis test between the robot footprint (a rotated rectangle) and each obstacle box.
 * On overlap, pushes the robot out along the axis of least penetration and removes the velocity
 * going into the obstacle.
 */
export function resolveObstacles(state: DriveState, config: DrivetrainConfig, obstacles: DriveObstacle[]): DriveState {
  if (obstacles.length === 0) return state;
  let out = state;
  for (const box of obstacles) {
    const corners = robotCorners(out, config);
    const boxCorners: [number, number][] = [
      [box.minX, box.minY],
      [box.maxX, box.minY],
      [box.maxX, box.maxY],
      [box.minX, box.maxY]
    ];
    const c = Math.cos(out.heading);
    const s = Math.sin(out.heading);
    const axes: [number, number][] = [
      [1, 0],
      [0, 1],
      [c, s],
      [-s, c]
    ];
    let best: { depth: number; axis: [number, number] } | null = null;
    let separated = false;
    for (const axis of axes) {
      const project = (pts: [number, number][]) => pts.map((p) => p[0] * axis[0] + p[1] * axis[1]);
      const a = project(corners);
      const b = project(boxCorners);
      const overlap = Math.min(Math.max(...a), Math.max(...b)) - Math.max(Math.min(...a), Math.min(...b));
      if (overlap <= 0) {
        separated = true;
        break;
      }
      if (best === null || overlap < best.depth) best = { depth: overlap, axis };
    }
    if (separated || best === null) continue;
    // Push direction: from the box center toward the robot center along the chosen axis
    const bx = (box.minX + box.maxX) / 2;
    const by = (box.minY + box.maxY) / 2;
    const sign = (out.x - bx) * best.axis[0] + (out.y - by) * best.axis[1] >= 0 ? 1 : -1;
    const nx = best.axis[0] * sign;
    const ny = best.axis[1] * sign;
    const into = out.vx * nx + out.vy * ny;
    out = {
      ...out,
      x: out.x + nx * best.depth,
      y: out.y + ny * best.depth,
      vx: into < 0 ? out.vx - into * nx : out.vx,
      vy: into < 0 ? out.vy - into * ny : out.vy,
      omega: out.omega * 0.5,
      wallContact: true
    };
  }
  return out;
}

/** Pushes the robot footprint back inside the perimeter and kills velocity into the walls. */
export function resolveWalls(
  state: DriveState,
  config: DrivetrainConfig,
  field: FieldGeometry = FTC_FIELD
): DriveState {
  const corners = robotCorners(state, config);
  const maxX = Math.max(...corners.map((p) => p[0]));
  const minX = Math.min(...corners.map((p) => p[0]));
  const maxY = Math.max(...corners.map((p) => p[1]));
  const minY = Math.min(...corners.map((p) => p[1]));
  const out = { ...state };
  if (maxX > field.halfX) {
    out.x -= maxX - field.halfX;
    out.vx = Math.min(0, out.vx);
    out.wallContact = true;
  } else if (minX < -field.halfX) {
    out.x += -field.halfX - minX;
    out.vx = Math.max(0, out.vx);
    out.wallContact = true;
  }
  if (maxY > field.halfY) {
    out.y -= maxY - field.halfY;
    out.vy = Math.min(0, out.vy);
    out.wallContact = true;
  } else if (minY < -field.halfY) {
    out.y += -field.halfY - minY;
    out.vy = Math.max(0, out.vy);
    out.wallContact = true;
  }
  if (out.wallContact) {
    // Scrubbing against the wall also bleeds turn rate
    out.omega *= 0.5;
  }
  return out;
}

/** Runs stepDrive in fixed sub-steps covering `frameDt`. */
export function advanceDrive(
  state: DriveState,
  input: DriveInput,
  config: DrivetrainConfig,
  fieldCentric: boolean,
  frameDt: number,
  obstacles: DriveObstacle[] = [],
  maxStep = 0.005
): DriveState {
  const steps = Math.max(1, Math.ceil(frameDt / maxStep));
  const dt = frameDt / steps;
  let contact = false;
  for (let i = 0; i < steps; i++) {
    state = stepDrive(state, input, config, fieldCentric, dt, FTC_FIELD, obstacles);
    contact = contact || state.wallContact;
  }
  return { ...state, wallContact: contact };
}
