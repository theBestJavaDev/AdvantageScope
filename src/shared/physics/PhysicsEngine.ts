// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { Pose3d, Rotation3d, Translation3d } from "../geometry";

export type Vector3 = [number, number, number];
export type Quaternion = [number, number, number, number]; // [w, x, y, z]

export interface PhysicsState {
  position: Translation3d;
  velocity: Vector3;
  orientation: Rotation3d;
  angularVelocity: Vector3;
  time: number;
}

export interface PhysicsConfig {
  mass: number;
  radius: number;
  dragCoefficient: number;
  magnusCoefficient: number;
  restitution: number;
  restitutionVelocityDecay: number;
  frictionCoefficient: number;
  momentOfInertia: number;
  airDensity: number;
  gravity: number;
  randomnessTolerance: {
    enabled: boolean;
    velocityJitterStdDev: number;
    angleJitterStdDevRad: number;
    spinJitterStdDevRadPerSec: number;
    surfaceRoughnessStdDev: number;
    turbulenceMagnitude: number;
  };
}

export const DEFAULT_PHYSICS_CONFIG: PhysicsConfig = {
  mass: 0.25,
  radius: 0.12,
  dragCoefficient: 0.45,
  magnusCoefficient: 0.2,
  restitution: 0.72,
  restitutionVelocityDecay: 0.04,
  frictionCoefficient: 0.35,
  momentOfInertia: 0.0014,
  airDensity: 1.225,
  gravity: 9.80665,
  randomnessTolerance: {
    enabled: true,
    velocityJitterStdDev: 0.05,
    angleJitterStdDevRad: 0.015,
    spinJitterStdDevRadPerSec: 0.2,
    surfaceRoughnessStdDev: 0.03,
    turbulenceMagnitude: 0.08
  }
};

export interface FieldBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
  minZ: number;
  maxZ: number;
}

export const DEFAULT_FRC_FIELD_BOUNDS: FieldBounds = {
  minX: -8.27,
  maxX: 8.27,
  minY: -4.03,
  maxY: 4.03,
  minZ: 0.0,
  maxZ: 5.0
};

function randomGaussian(mean = 0, stdDev = 1): number {
  const u1 = Math.max(1e-10, Math.random());
  const u2 = Math.random();
  const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
  return mean + z0 * stdDev;
}

function vecAdd(a: Vector3, b: Vector3): Vector3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function vecSub(a: Vector3, b: Vector3): Vector3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function vecScale(v: Vector3, s: number): Vector3 {
  return [v[0] * s, v[1] * s, v[2] * s];
}

function vecDot(a: Vector3, b: Vector3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function vecCross(a: Vector3, b: Vector3): Vector3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
}

function vecLength(v: Vector3): number {
  return Math.hypot(v[0], v[1], v[2]);
}

function vecNormalize(v: Vector3): Vector3 {
  const len = vecLength(v);
  if (len < 1e-9) return [0, 0, 0];
  return [v[0] / len, v[1] / len, v[2] / len];
}

function quatMultiply(q1: Quaternion, q2: Quaternion): Quaternion {
  const [w1, x1, y1, z1] = q1;
  const [w2, x2, y2, z2] = q2;
  return [
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
  ];
}

function quatNormalize(q: Quaternion): Quaternion {
  const len = Math.hypot(q[0], q[1], q[2], q[3]);
  if (len < 1e-9) return [1, 0, 0, 0];
  return [q[0] / len, q[1] / len, q[2] / len, q[3] / len];
}

function integrateQuaternion(q: Quaternion, w: Vector3, dt: number): Quaternion {
  const omegaQuat: Quaternion = [0, w[0], w[1], w[2]];
  const dq = quatMultiply(omegaQuat, q);
  const newQ: Quaternion = [
    q[0] + 0.5 * dq[0] * dt,
    q[1] + 0.5 * dq[1] * dt,
    q[2] + 0.5 * dq[2] * dt,
    q[3] + 0.5 * dq[3] * dt
  ];
  return quatNormalize(newQ);
}

export class PhysicsEngine {
  private config: PhysicsConfig;

  constructor(config: Partial<PhysicsConfig> = {}) {
    this.config = {
      ...DEFAULT_PHYSICS_CONFIG,
      ...config,
      randomnessTolerance: {
        ...DEFAULT_PHYSICS_CONFIG.randomnessTolerance,
        ...(config.randomnessTolerance || {})
      }
    };
  }

  public applyRandomnessTolerance(
    velocity: Vector3,
    angularVelocity: Vector3
  ): { velocity: Vector3; angularVelocity: Vector3 } {
    if (!this.config.randomnessTolerance.enabled) {
      return { velocity, angularVelocity };
    }

    const { velocityJitterStdDev, angleJitterStdDevRad, spinJitterStdDevRadPerSec } =
      this.config.randomnessTolerance;

    let speed = vecLength(velocity);
    if (speed > 1e-5) {
      speed += randomGaussian(0, velocityJitterStdDev);
      speed = Math.max(0, speed);

      let dir = vecNormalize(velocity);
      const perp: Vector3 = [
        randomGaussian(0, angleJitterStdDevRad),
        randomGaussian(0, angleJitterStdDevRad),
        randomGaussian(0, angleJitterStdDevRad)
      ];
      dir = vecNormalize(vecAdd(dir, perp));
      velocity = vecScale(dir, speed);
    }

    const wJitter: Vector3 = [
      randomGaussian(0, spinJitterStdDevRadPerSec),
      randomGaussian(0, spinJitterStdDevRadPerSec),
      randomGaussian(0, spinJitterStdDevRadPerSec)
    ];

    return {
      velocity,
      angularVelocity: vecAdd(angularVelocity, wJitter)
    };
  }

  private computeAcceleration(pos: Vector3, vel: Vector3, angVel: Vector3): Vector3 {
    const { mass, radius, dragCoefficient, magnusCoefficient, airDensity, gravity, randomnessTolerance } =
      this.config;

    const gAcc: Vector3 = [0, 0, -gravity];
    const vSpeed = vecLength(vel);
    if (vSpeed < 1e-6) return gAcc;

    const area = Math.PI * radius * radius;
    const dragForceMag = 0.5 * airDensity * vSpeed * vSpeed * dragCoefficient * area;
    const dragAcc = vecScale(vecNormalize(vel), -dragForceMag / mass);

    let magnusAcc: Vector3 = [0, 0, 0];
    const wSpeed = vecLength(angVel);
    if (wSpeed > 1e-4) {
      const crossSpinVel = vecCross(angVel, vel);
      const magnusForceMag = 0.5 * airDensity * magnusCoefficient * area * radius * vecLength(crossSpinVel);
      magnusAcc = vecScale(vecNormalize(crossSpinVel), magnusForceMag / mass);
    }

    let turbulenceAcc: Vector3 = [0, 0, 0];
    if (randomnessTolerance.enabled && randomnessTolerance.turbulenceMagnitude > 0) {
      turbulenceAcc = [
        randomGaussian(0, randomnessTolerance.turbulenceMagnitude),
        randomGaussian(0, randomnessTolerance.turbulenceMagnitude),
        randomGaussian(0, randomnessTolerance.turbulenceMagnitude)
      ];
    }

    return vecAdd(vecAdd(vecAdd(gAcc, dragAcc), magnusAcc), turbulenceAcc);
  }

  public step(state: PhysicsState, dt: number, bounds: FieldBounds = DEFAULT_FRC_FIELD_BOUNDS): PhysicsState {
    let { position, velocity, orientation, angularVelocity, time } = state;

    const k1_v = this.computeAcceleration(position, velocity, angularVelocity);
    const k1_p = velocity;

    const p_k2 = vecAdd(position, vecScale(k1_p, dt * 0.5));
    const v_k2 = vecAdd(velocity, vecScale(k1_v, dt * 0.5));
    const k2_v = this.computeAcceleration(p_k2, v_k2, angularVelocity);
    const k2_p = v_k2;

    const p_k3 = vecAdd(position, vecScale(k2_p, dt * 0.5));
    const v_k3 = vecAdd(velocity, vecScale(k1_v, dt * 0.5));
    const k3_v = this.computeAcceleration(p_k3, v_k3, angularVelocity);
    const k3_p = v_k3;

    const p_k4 = vecAdd(position, vecScale(k3_p, dt));
    const v_k4 = vecAdd(velocity, vecScale(k3_v, dt));
    const k4_v = this.computeAcceleration(p_k4, v_k4, angularVelocity);
    const k4_p = v_k4;

    const dPos: Vector3 = vecScale(
      vecAdd(vecAdd(k1_p, vecScale(k2_p, 2)), vecAdd(vecScale(k3_p, 2), k4_p)),
      dt / 6.0
    );
    const dVel: Vector3 = vecScale(
      vecAdd(vecAdd(k1_v, vecScale(k2_v, 2)), vecAdd(vecScale(k3_v, 2), k4_v)),
      dt / 6.0
    );

    let newPosition = vecAdd(position, dPos);
    let newVelocity = vecAdd(velocity, dVel);

    const rotSpeed = vecLength(angularVelocity);
    if (rotSpeed > 1e-4) {
      const rotDamping = Math.exp(-0.15 * rotSpeed * dt);
      angularVelocity = vecScale(angularVelocity, rotDamping);
    }

    let newOrientation = integrateQuaternion(orientation, angularVelocity, dt);
    return {
      position: newPosition,
      velocity: newVelocity,
      orientation: newOrientation,
      angularVelocity,
      time: time + dt
    };
  }

  public simulateTrajectory(
    startPose: Pose3d,
    initialLinearVelocity: Vector3,
    initialAngularVelocity: Vector3 = [0, 0, 0],
    durationSeconds = 3.0,
    timeStepSeconds = 0.05,
    bounds: FieldBounds = DEFAULT_FRC_FIELD_BOUNDS
  ): { poses: Pose3d[]; velocities: Vector3[]; bounces: Vector3[] } {
    const randomized = this.applyRandomnessTolerance(initialLinearVelocity, initialAngularVelocity);

    let currentState: PhysicsState = {
      position: [...startPose.translation] as Translation3d,
      velocity: randomized.velocity,
      orientation: [...startPose.rotation] as Rotation3d,
      angularVelocity: randomized.angularVelocity,
      time: 0
    };

    const poses: Pose3d[] = [{ translation: currentState.position, rotation: currentState.orientation }];
    const velocities: Vector3[] = [currentState.velocity];
    const bounces: Vector3[] = [];

    const steps = Math.ceil(durationSeconds / timeStepSeconds);

    for (let i = 0; i < steps; i++) {
      const nextState = this.step(currentState, timeStepSeconds, bounds);
      if (vecDot(currentState.velocity, nextState.velocity) < 0 || vecLength(vecSub(nextState.velocity, currentState.velocity)) > 2.0) {
        bounces.push(nextState.position);
      }
      poses.push({
        translation: nextState.position,
        rotation: nextState.orientation
      });
      velocities.push(nextState.velocity);
      currentState = nextState;
      if (currentState.position[2] <= bounds.minZ + 0.15 && vecLength(currentState.velocity) < 0.05) {
        break;
      }
    }

    return { poses, velocities, bounces };
  }
}
