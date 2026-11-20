// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/** Small allocation-light 3D vector helpers used by the physics code. */

export type Vector3 = [number, number, number];

export const ZERO: Vector3 = [0, 0, 0];
export const UP: Vector3 = [0, 0, 1];

export function add(a: Vector3, b: Vector3): Vector3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function sub(a: Vector3, b: Vector3): Vector3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function scale(v: Vector3, s: number): Vector3 {
  return [v[0] * s, v[1] * s, v[2] * s];
}

/** a + b * s */
export function addScaled(a: Vector3, b: Vector3, s: number): Vector3 {
  return [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];
}

export function dot(a: Vector3, b: Vector3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function cross(a: Vector3, b: Vector3): Vector3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function length(v: Vector3): number {
  return Math.hypot(v[0], v[1], v[2]);
}

export function normalize(v: Vector3): Vector3 {
  const len = length(v);
  if (len < 1e-12) return [0, 0, 0];
  return [v[0] / len, v[1] / len, v[2] / len];
}

/** Rotates a vector about the +Z axis. */
export function rotateZ(v: Vector3, angle: number): Vector3 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]];
}

/** Distance from a point to an axis-aligned box (0 when inside). */
export function distanceToBox(p: Vector3, min: Vector3, max: Vector3): number {
  const dx = Math.max(min[0] - p[0], 0, p[0] - max[0]);
  const dy = Math.max(min[1] - p[1], 0, p[1] - max[1]);
  const dz = Math.max(min[2] - p[2], 0, p[2] - max[2]);
  return Math.hypot(dx, dy, dz);
}

/** Seeded PRNG (mulberry32) so Monte Carlo runs are reproducible. */
export class Random {
  private state: number;
  private spare: number | null = null;

  constructor(seed: number = Date.now()) {
    this.state = seed >>> 0 || 1;
  }

  /** Uniform in [0, 1). */
  next(): number {
    let t = (this.state = (this.state + 0x6d2b79f5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  /** Normally distributed sample. Returns the mean exactly when stdDev <= 0. */
  gaussian(mean = 0, stdDev = 1): number {
    if (!(stdDev > 0)) return mean;
    if (this.spare !== null) {
      const z = this.spare;
      this.spare = null;
      return mean + z * stdDev;
    }
    const u1 = Math.max(1e-12, this.next());
    const u2 = this.next();
    const mag = Math.sqrt(-2 * Math.log(u1));
    this.spare = mag * Math.sin(2 * Math.PI * u2);
    return mean + mag * Math.cos(2 * Math.PI * u2) * stdDev;
  }

  /** True with the given probability. */
  chance(probability: number): boolean {
    return probability > 0 && this.next() < probability;
  }
}
