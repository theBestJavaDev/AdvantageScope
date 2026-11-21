// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Game piece flight + contact physics for FTC fields.
 *
 * Model summary (all SI units, field frame: +X/+Y on the floor, +Z up, origin at field center):
 *  - Gravity
 *  - Quadratic air drag on the velocity relative to the air (wind/gusts supported)
 *  - Magnus lift from spin, using a saturating lift-coefficient curve in the spin parameter S = r|w|/|v|
 *  - Exponential spin decay from air torque
 *  - Impulse-based contacts with the floor tiles and perimeter walls: coefficient of restitution,
 *    Coulomb friction that couples spin <-> tangential velocity (backspin "checks", topspin "kicks"),
 *    and rolling resistance once the piece is rolling on the tiles
 *  - Fixed-step RK4 integration for the free-flight phase
 */

import { Rotation3d } from "../geometry";
import { Random, Vector3, add, addScaled, cross, distanceToBox, dot, length, normalize, scale, sub } from "./vector";

export type { Vector3 } from "./vector";

// ---------------------------------------------------------------------------------------------
// Configuration types
// ---------------------------------------------------------------------------------------------

export interface GamePieceProperties {
  name: string;
  /** kg */
  mass: number;
  /** m */
  radius: number;
  /** Drag coefficient Cd (smooth sphere ~0.47, foam/dimpled balls often 0.3-0.6). */
  dragCoefficient: number;
  /** Multiplier on the Magnus lift curve. 1 = textbook sphere; tune to match measured shots. */
  magnusScale: number;
  /** Upper bound on the lift coefficient. */
  maxLiftCoefficient: number;
  /** Moment of inertia factor k in I = k*m*r^2 (2/3 thin hollow shell, 2/5 solid sphere). */
  inertiaFactor: number;
  /** Seconds for spin to decay to 1/e in free flight. */
  spinDecayTimeConstant: number;
  /** Coefficient of restitution against the floor tiles. */
  floorRestitution: number;
  /** Coefficient of restitution against the perimeter walls. */
  wallRestitution: number;
  /** Sliding friction coefficient against field surfaces. */
  frictionCoefficient: number;
  /** Rolling resistance coefficient on the tiles (deceleration = Crr * g). */
  rollingResistance: number;
  /**
   * Fraction of tangential contact velocity returned on impact while gripping (Walton-Braun style
   * spin rebound: a ball landing with backspin "kicks" back a bit instead of dead-stopping the slip).
   * Default 0.2.
   */
  tangentialRestitution?: number;
  /**
   * How fast restitution falls with impact speed, per m/s above the 1 m drop-test speed (4.4 m/s).
   * Plastic shells soften on hard hits and bounce livelier on soft ones. 0 = constant. Default 0.03.
   */
  restitutionSoftening?: number;
  /** Static friction = this x the sliding coefficient (grip before slip). Default 1.2. */
  staticFrictionRatio?: number;
  /** Resistance to spin about the contact normal (a ball spinning flat on the floor slows). Default 0.1. */
  pivotFriction?: number;
}

export interface EnvironmentProperties {
  /** m/s^2 */
  gravity: number;
  /** kg/m^3 */
  airDensity: number;
  /** Steady air velocity, m/s (HVAC drafts etc). */
  wind: Vector3;
  /** Random gust std dev (m/s per axis), re-sampled every gustInterval seconds. */
  gustStdDev: number;
  /** s */
  gustInterval: number;
}

export interface FieldGeometry {
  /** Half the field length along X, m. */
  halfX: number;
  /** Half the field width along Y, m. */
  halfY: number;
  /** Height of the perimeter wall, m. Pieces above this fly out of the field. */
  wallHeight: number;
}

/**
 * Scoring target. By default an axis-aligned volume: a shot "hits" when the piece's center enters it.
 * With `mouth`, it is a tilted opening instead (a BIOBUZZ HIVE CELL): a hit is the piece's center
 * crossing the mouth face inward inside its outline, and min/max only bound it.
 */
export interface TargetZone {
  name: string;
  min: Vector3;
  max: Vector3;
  /** Only count entries while the piece is moving downward (e.g. dropping into a basket). */
  requireDescending: boolean;
  mouth?: TargetMouth;
}

/** A flat rectangular opening: center, outward unit normal, and its two in-face axes + half sizes. */
export interface TargetMouth {
  center: Vector3;
  normal: Vector3;
  u: Vector3;
  v: Vector3;
  halfU: number;
  halfV: number;
  /** Solid parts around the opening (the HIVE's trays and frame) that a piece bounces off. */
  colliders?: Collider[];
}

/** A solid part a game piece can bounce off: an oriented box or a capsule (a round tube). */
export type Collider =
  | { kind: "box"; center: Vector3; axes: [Vector3, Vector3, Vector3]; half: Vector3 }
  | { kind: "capsule"; a: Vector3; b: Vector3; radius: number };

/**
 * Closest approach of a point to a collider: the outward normal at the closest surface point and the
 * signed distance from the point to the surface (negative inside).
 */
export function colliderContact(p: Vector3, c: Collider): { normal: Vector3; distance: number } | null {
  if (c.kind === "capsule") {
    const ab = sub(c.b, c.a);
    const t = Math.max(0, Math.min(1, dot(sub(p, c.a), ab) / Math.max(dot(ab, ab), 1e-12)));
    const d = sub(p, addScaled(c.a, ab, t));
    const len = length(d);
    if (len < 1e-9) return null;
    return { normal: scale(d, 1 / len), distance: len - c.radius };
  }
  const d = sub(p, c.center);
  const local = c.axes.map((axis) => dot(d, axis));
  const clamped = local.map((x, i) => Math.max(-c.half[i], Math.min(c.half[i], x)));
  const out = local.map((x, i) => x - clamped[i]);
  const outLen = Math.hypot(out[0], out[1], out[2]);
  if (outLen > 1e-9) {
    let n: Vector3 = [0, 0, 0];
    for (let i = 0; i < 3; i++) n = addScaled(n, c.axes[i], out[i] / outLen);
    return { normal: n, distance: outLen };
  }
  // Inside: push out through the nearest face
  let best = 0;
  let bestDepth = Infinity;
  for (let i = 0; i < 3; i++) {
    const depth = c.half[i] - Math.abs(local[i]);
    if (depth < bestDepth) {
      bestDepth = depth;
      best = i;
    }
  }
  return { normal: scale(c.axes[best], local[best] >= 0 ? 1 : -1), distance: -bestDepth };
}

/** Radius of a sphere around the collider (for skipping far-away parts quickly). */
function colliderBound(c: Collider): { center: Vector3; radius: number } {
  if (c.kind === "capsule") {
    return { center: scale(add(c.a, c.b), 0.5), radius: length(sub(c.b, c.a)) / 2 + c.radius };
  }
  return { center: c.center, radius: length(c.half) };
}

/** Distance from p to a mouth rectangle (0 on it). */
export function distanceToMouth(p: Vector3, mouth: TargetMouth): number {
  const d = sub(p, mouth.center);
  const pu = Math.max(0, Math.abs(dot(d, mouth.u)) - mouth.halfU);
  const pv = Math.max(0, Math.abs(dot(d, mouth.v)) - mouth.halfV);
  return Math.hypot(pu, pv, dot(d, mouth.normal));
}

/**
 * Where the segment a -> b crosses the mouth plane going inward (outside -> inside), as in-face
 * coordinates relative to the mouth center, or null if it does not cross inward.
 */
export function mouthCrossing(
  a: Vector3,
  b: Vector3,
  mouth: TargetMouth
): { u: number; v: number; point: Vector3 } | null {
  const sa = dot(sub(a, mouth.center), mouth.normal);
  const sb = dot(sub(b, mouth.center), mouth.normal);
  if (!(sa > 0 && sb <= 0)) return null;
  const f = sa / (sa - sb);
  const point: Vector3 = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
  const d = sub(point, mouth.center);
  return { u: dot(d, mouth.u), v: dot(d, mouth.v), point };
}

export interface SimulationOptions {
  /** Max simulated time, s. */
  duration: number;
  /** Internal integration step, s. */
  timeStep: number;
  /** Interval between recorded samples, s. */
  sampleInterval: number;
  /** Stop as soon as the target zone is entered. */
  stopOnTargetHit: boolean;
  /** Stop once the piece is at rest on the floor. */
  stopWhenAtRest: boolean;
  /** Stop at the first floor contact (fast mode for Monte Carlo when bounces can't score). */
  stopAfterLanding: boolean;
  /** If set, record where the piece first crosses this height while descending (impact plane). */
  crossingHeight: number | null;
}

// ---------------------------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------------------------

const INCH = 0.0254;

/** Standard FTC field: 12 ft x 12 ft inside the perimeter, origin at center. */
export const FTC_FIELD: FieldGeometry = {
  halfX: 72 * INCH,
  halfY: 72 * INCH,
  // Verify against the current game manual / your field; perimeter height is configurable.
  wallHeight: 12 * INCH
};

/**
 * Starting values for a ~5 in hollow plastic ball. These are PLACEHOLDERS: weigh and measure the real
 * game piece, and tune Cd / magnusScale / restitution against filmed shots before trusting results.
 */
export const DEFAULT_GAME_PIECE: GamePieceProperties = {
  name: "5 in ball (placeholder - measure yours)",
  mass: 0.075,
  radius: 2.5 * INCH,
  dragCoefficient: 0.5,
  magnusScale: 1.0,
  maxLiftCoefficient: 0.4,
  inertiaFactor: 2 / 3,
  spinDecayTimeConstant: 1.5,
  floorRestitution: 0.6,
  wallRestitution: 0.5,
  frictionCoefficient: 0.4,
  rollingResistance: 0.03
};

export const DEFAULT_ENVIRONMENT: EnvironmentProperties = {
  gravity: 9.80665,
  airDensity: 1.2,
  wind: [0, 0, 0],
  gustStdDev: 0,
  gustInterval: 0.1
};

export const DEFAULT_SIMULATION_OPTIONS: SimulationOptions = {
  duration: 4,
  timeStep: 0.001,
  sampleInterval: 0.01,
  stopOnTargetHit: false,
  stopWhenAtRest: true,
  stopAfterLanding: false,
  crossingHeight: null
};

// ---------------------------------------------------------------------------------------------
// State + results
// ---------------------------------------------------------------------------------------------

export interface BallState {
  position: Vector3;
  velocity: Vector3;
  /** World-frame angular velocity, rad/s. */
  angularVelocity: Vector3;
  /** Quaternion (w, x, y, z). */
  orientation: Rotation3d;
  time: number;
}

export interface BallSample {
  time: number;
  position: Vector3;
  velocity: Vector3;
  angularVelocity: Vector3;
  orientation: Rotation3d;
  /** Magnitudes of the aerodynamic forces at this sample, N. */
  dragForce: number;
  magnusForce: number;
}

export type ContactSurface = "floor" | "wall+x" | "wall-x" | "wall+y" | "wall-y" | "hive";

export interface ContactEvent {
  time: number;
  position: Vector3;
  surface: ContactSurface;
  /** Speed into the surface just before impact, m/s. */
  impactSpeed: number;
  /** True if friction could not stop slip (piece skidded). */
  slipping: boolean;
}

export interface TrajectoryResult {
  samples: BallSample[];
  contacts: ContactEvent[];
  target: {
    hit: boolean;
    time: number | null;
    entryPosition: Vector3 | null;
    entryVelocity: Vector3 | null;
    /** Closest distance from the piece center to the zone (0 = entered). */
    closestDistance: number;
    closestTime: number;
  } | null;
  /** First floor contact, or null if it never landed. */
  landing: { time: number; position: Vector3 } | null;
  apex: { time: number; position: Vector3 };
  /** First descending crossing of options.crossingHeight (interpolated), or null. */
  crossing: Vector3 | null;
  leftField: boolean;
  finalState: BallState;
}

// ---------------------------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------------------------

/** Derivative of the free-flight state (position, velocity, angular velocity). */
interface Derivative {
  dp: Vector3;
  dv: Vector3;
  dw: Vector3;
}

/** Impact speed of a 1 m drop test, the reference for the restitution values (m/s). */
const DROP_TEST_SPEED = Math.sqrt(2 * 9.80665 * 1);

/** Lift coefficient vs spin parameter S = r|w|/|v| (saturating fit to published sphere data). */
export function liftCoefficient(spinParameter: number, magnusScale: number, maxCl: number): number {
  if (spinParameter <= 0) return 0;
  const cl = spinParameter / (2.022 * spinParameter + 0.981);
  return Math.min(maxCl, magnusScale * cl);
}

/** Per-bounce randomness (tile seams, dust, scuffs, seam orientation). Off unless set. */
export interface ContactNoise {
  rng: Random;
  restitutionStdDev: number;
  frictionStdDev: number;
  bounceAngleStdDevDeg: number;
}

export class PhysicsEngine {
  /** When set, every real impact draws its own restitution, friction and sideways deflection. */
  public contactNoise: ContactNoise | null = null;

  constructor(
    public piece: GamePieceProperties = DEFAULT_GAME_PIECE,
    public environment: EnvironmentProperties = DEFAULT_ENVIRONMENT,
    public field: FieldGeometry = FTC_FIELD
  ) {}

  /** Solid parts for the current simulation (from the target's mouth), with bounding spheres. */
  private colliders: { collider: Collider; center: Vector3; radius: number }[] = [];

  /** Sets the solid parts pieces bounce off (simulate() takes them from the target's mouth). */
  setColliders(colliders: Collider[]) {
    this.colliders = colliders.map((collider) => ({ collider, ...colliderBound(collider) }));
  }

  private get area(): number {
    return Math.PI * this.piece.radius * this.piece.radius;
  }

  private get inertia(): number {
    return this.piece.inertiaFactor * this.piece.mass * this.piece.radius * this.piece.radius;
  }

  /** Aerodynamic forces (N) for a given velocity / spin / air velocity. */
  aeroForces(velocity: Vector3, angularVelocity: Vector3, air: Vector3): { drag: Vector3; magnus: Vector3 } {
    const { radius, dragCoefficient, magnusScale, maxLiftCoefficient } = this.piece;
    const rho = this.environment.airDensity;
    const vRel = sub(velocity, air);
    const speed = length(vRel);
    if (speed < 1e-9) return { drag: [0, 0, 0], magnus: [0, 0, 0] };

    const q = 0.5 * rho * this.area * speed * speed; // dynamic pressure * area
    const drag = scale(vRel, (-q * dragCoefficient) / speed);

    let magnus: Vector3 = [0, 0, 0];
    const spin = length(angularVelocity);
    if (spin > 1e-6) {
      const cl = liftCoefficient((radius * spin) / speed, magnusScale, maxLiftCoefficient);
      const dir = normalize(cross(angularVelocity, vRel));
      magnus = scale(dir, q * cl);
    }
    return { drag, magnus };
  }

  private derivative(v: Vector3, w: Vector3, air: Vector3): Derivative {
    const { drag, magnus } = this.aeroForces(v, w, air);
    const invMass = 1 / this.piece.mass;
    const dv: Vector3 = [
      (drag[0] + magnus[0]) * invMass,
      (drag[1] + magnus[1]) * invMass,
      (drag[2] + magnus[2]) * invMass - this.environment.gravity
    ];
    const tau = this.piece.spinDecayTimeConstant;
    const dw = tau > 0 ? scale(w, -1 / tau) : ([0, 0, 0] as Vector3);
    return { dp: v, dv, dw };
  }

  /** One RK4 free-flight step (no contacts). */
  integrateFlight(state: BallState, dt: number, air: Vector3): BallState {
    const { position: p, velocity: v, angularVelocity: w } = state;

    const k1 = this.derivative(v, w, air);
    const v2 = addScaled(v, k1.dv, dt / 2);
    const w2 = addScaled(w, k1.dw, dt / 2);
    const k2 = this.derivative(v2, w2, air);
    const v3 = addScaled(v, k2.dv, dt / 2);
    const w3 = addScaled(w, k2.dw, dt / 2);
    const k3 = this.derivative(v3, w3, air);
    const v4 = addScaled(v, k3.dv, dt);
    const w4 = addScaled(w, k3.dw, dt);
    const k4 = this.derivative(v4, w4, air);

    const combine = (a: Vector3, b: Vector3, c: Vector3, d: Vector3): Vector3 => [
      ((a[0] + 2 * b[0] + 2 * c[0] + d[0]) * dt) / 6,
      ((a[1] + 2 * b[1] + 2 * c[1] + d[1]) * dt) / 6,
      ((a[2] + 2 * b[2] + 2 * c[2] + d[2]) * dt) / 6
    ];

    const newW = add(w, combine(k1.dw, k2.dw, k3.dw, k4.dw));
    return {
      position: add(p, combine(k1.dp, k2.dp, k3.dp, k4.dp)),
      velocity: add(v, combine(k1.dv, k2.dv, k3.dv, k4.dv)),
      angularVelocity: newW,
      orientation: integrateOrientation(state.orientation, w, newW, dt),
      time: state.time + dt
    };
  }

  /**
   * Resolves penetration with the floor and walls using impulses. Mutates and returns the state.
   * Friction couples spin and tangential velocity: with enough friction the contact point stops
   * slipping (rolling), otherwise the piece skids with Coulomb-limited impulse.
   */
  resolveContacts(state: BallState, dt: number, events: ContactEvent[] | null): BallState {
    const r = this.piece.radius;
    const m = this.piece.mass;
    const I = this.inertia;
    const k = this.piece.inertiaFactor;
    const mu = this.piece.frictionCoefficient;
    const g = this.environment.gravity;
    const { halfX, halfY, wallHeight } = this.field;

    const planes: { surface: ContactSurface; normal: Vector3; distance: number; restitution: number }[] = [
      { surface: "floor", normal: [0, 0, 1], distance: state.position[2], restitution: this.piece.floorRestitution }
    ];
    // Walls only exist below the wall top
    if (state.position[2] - r < wallHeight) {
      const e = this.piece.wallRestitution;
      planes.push(
        { surface: "wall+x", normal: [-1, 0, 0], distance: halfX - state.position[0], restitution: e },
        { surface: "wall-x", normal: [1, 0, 0], distance: state.position[0] + halfX, restitution: e },
        { surface: "wall+y", normal: [0, -1, 0], distance: halfY - state.position[1], restitution: e },
        { surface: "wall-y", normal: [0, 1, 0], distance: state.position[1] + halfY, restitution: e }
      );
    }

    // HIVE trays and frame (polycarbonate / aluminum: bounce like the walls)
    for (const { collider, center, radius } of this.colliders) {
      if (length(sub(state.position, center)) > radius + r) continue;
      const contact = colliderContact(state.position, collider);
      if (contact === null || contact.distance >= r) continue;
      planes.push({
        surface: "hive",
        normal: contact.normal,
        distance: contact.distance,
        restitution: this.piece.wallRestitution
      });
    }

    const betaT = this.piece.tangentialRestitution ?? 0.2;
    const softening = this.piece.restitutionSoftening ?? 0.03;
    const staticRatio = this.piece.staticFrictionRatio ?? 1.2;
    const pivot = this.piece.pivotFriction ?? 0.1;

    for (const plane of planes) {
      const penetration = r - plane.distance;
      if (penetration <= 0) continue;
      // Ignore walls the piece is already fully past (it left the field over the top)
      if (plane.surface.startsWith("wall") && plane.distance < -r) continue;

      const n = plane.normal;
      state.position = addScaled(state.position, n, penetration);

      const vn = dot(state.velocity, n);
      // Velocity-dependent restitution: slow impacts don't bounce (prevents endless micro-bounces),
      // and harder hits lose a little more energy than the drop-test value.
      const restThreshold = 2 * g * dt + 0.05;
      const impactSpeed = -vn;
      const isImpact = impactSpeed > restThreshold;
      const speedFactor = Math.min(1.15, Math.max(0.5, 1 - softening * (impactSpeed - DROP_TEST_SPEED)));
      // Each impact is a little different (where on the ball and tile it lands)
      const noise = isImpact ? this.contactNoise : null;
      const eNoise = noise === null ? 0 : noise.rng.gaussian(0, noise.restitutionStdDev);
      const muHit = noise === null ? mu : Math.max(0.02, mu * (1 + noise.rng.gaussian(0, noise.frictionStdDev)));
      const e = isImpact ? Math.min(0.98, Math.max(0, plane.restitution * speedFactor + eNoise)) : 0;
      let normalImpulse = vn < 0 ? -(1 + e) * m * vn : 0;
      // Resting on the floor: gravity's support impulse still provides friction
      if (plane.surface === "floor") normalImpulse = Math.max(normalImpulse, m * g * dt);

      // Tangential slip of the contact point
      const rc = scale(n, -r);
      const vContact = add(state.velocity, cross(state.angularVelocity, rc));
      const vSlip = addScaled(vContact, n, -dot(vContact, n));
      const slipSpeed = length(vSlip);

      // Impulse that would stop the slip (rolling without slipping). On a real impact the grip also
      // returns a share of the tangential velocity (spin rebound); at rest it must not, or it jitters.
      const stickGain = isImpact ? 1 + betaT : 1;
      let tangentImpulse = scale(vSlip, (-stickGain * m) / (1 + 1 / k));
      const stickLimit = muHit * staticRatio * normalImpulse;
      const slipping = length(tangentImpulse) > stickLimit;
      if (slipping && slipSpeed > 1e-9) {
        // Skidding: kinetic friction (a little below the static limit)
        tangentImpulse = scale(vSlip, (-muHit * normalImpulse) / slipSpeed);
      }

      state.velocity = add(state.velocity, scale(n, vn < 0 ? -(1 + e) * vn : 0));
      state.velocity = addScaled(state.velocity, tangentImpulse, 1 / m);
      state.angularVelocity = addScaled(state.angularVelocity, cross(rc, tangentImpulse), 1 / I);

      // Sideways kick: a seam or scuff turns the ball a few degrees off its path
      if (noise !== null && noise.bounceAngleStdDevDeg > 0) {
        const angle = noise.rng.gaussian(0, noise.bounceAngleStdDevDeg) * (Math.PI / 180);
        const vt = addScaled(state.velocity, n, -dot(state.velocity, n));
        const turned = add(scale(vt, Math.cos(angle) - 1), scale(cross(n, vt), Math.sin(angle)));
        state.velocity = add(state.velocity, turned);
      }

      // Pivot (spin-about-the-normal) friction: a ball spinning flat on the floor / against a wall slows.
      const spinAlongNormal = dot(state.angularVelocity, n);
      if (Math.abs(spinAlongNormal) > 1e-9) {
        const maxDelta = (pivot * mu * r * normalImpulse) / I;
        const delta = Math.sign(spinAlongNormal) * Math.min(Math.abs(spinAlongNormal), maxDelta);
        state.angularVelocity = addScaled(state.angularVelocity, n, -delta);
      }

      // Rolling resistance on the tiles (only while actually rolling; skids are handled by friction)
      if (plane.surface === "floor" && !isImpact && !slipping) {
        const horiz: Vector3 = [state.velocity[0], state.velocity[1], 0];
        const speed = length(horiz);
        const decel = this.piece.rollingResistance * g * dt;
        if (speed > 1e-9) {
          const newSpeed = Math.max(0, speed - decel);
          const ratio = newSpeed / speed;
          state.velocity = [state.velocity[0] * ratio, state.velocity[1] * ratio, state.velocity[2]];
          // Keep spin consistent with rolling
          state.angularVelocity = [
            state.angularVelocity[0] * ratio,
            state.angularVelocity[1] * ratio,
            state.angularVelocity[2]
          ];
        }
      }

      if (events !== null && isImpact) {
        events.push({
          time: state.time,
          position: [...state.position] as Vector3,
          surface: plane.surface,
          impactSpeed: impactSpeed,
          slipping
        });
      }
    }
    return state;
  }

  /** Simulates a launched game piece until it rests, leaves the field, hits the target, or times out. */
  simulate(
    initial: BallState,
    options: Partial<SimulationOptions> = {},
    target: TargetZone | null = null,
    gustSampler: (() => Vector3) | null = null
  ): TrajectoryResult {
    const opts = { ...DEFAULT_SIMULATION_OPTIONS, ...options };
    const dt = opts.timeStep;
    this.setColliders(target?.mouth?.colliders ?? []);
    const r = this.piece.radius;

    let state: BallState = {
      position: [...initial.position] as Vector3,
      velocity: [...initial.velocity] as Vector3,
      angularVelocity: [...initial.angularVelocity] as Vector3,
      orientation: [...initial.orientation] as Rotation3d,
      time: initial.time
    };

    const samples: BallSample[] = [];
    const contacts: ContactEvent[] = [];
    let landing: TrajectoryResult["landing"] = null;
    let apex = { time: state.time, position: [...state.position] as Vector3 };
    let leftField = false;
    let crossing: Vector3 | null = null;

    let targetHit = false;
    let targetTime: number | null = null;
    let entryPosition: Vector3 | null = null;
    let entryVelocity: Vector3 | null = null;
    let closestDistance = Infinity;
    let closestTime = state.time;

    let gust: Vector3 = [0, 0, 0];
    let nextGustTime = state.time;
    let nextSampleTime = state.time;

    const record = () => {
      const air = add(this.environment.wind, gust);
      const f = this.aeroForces(state.velocity, state.angularVelocity, air);
      samples.push({
        time: state.time,
        position: [...state.position] as Vector3,
        velocity: [...state.velocity] as Vector3,
        angularVelocity: [...state.angularVelocity] as Vector3,
        orientation: [...state.orientation] as Rotation3d,
        dragForce: length(f.drag),
        magnusForce: length(f.magnus)
      });
    };

    const endTime = state.time + opts.duration;
    while (state.time < endTime - 1e-12) {
      // Gusts are held constant between re-samples so RK4 sees a smooth field within a step
      if (gustSampler !== null && state.time >= nextGustTime) {
        gust = gustSampler();
        nextGustTime += this.environment.gustInterval;
      }
      if (state.time >= nextSampleTime - 1e-12) {
        record();
        nextSampleTime += opts.sampleInterval;
      }

      const air = add(this.environment.wind, gust);
      const contactCountBefore = contacts.length;
      const previousPosition = state.position;
      state = this.integrateFlight(state, dt, air);
      state = this.resolveContacts(state, dt, contacts);

      const h = opts.crossingHeight;
      if (
        h !== null &&
        crossing === null &&
        previousPosition[2] >= h &&
        state.position[2] < h &&
        state.velocity[2] < 0
      ) {
        const frac = (previousPosition[2] - h) / (previousPosition[2] - state.position[2]);
        crossing = [
          previousPosition[0] + (state.position[0] - previousPosition[0]) * frac,
          previousPosition[1] + (state.position[1] - previousPosition[1]) * frac,
          h
        ];
      }

      if (landing === null) {
        for (let i = contactCountBefore; i < contacts.length; i++) {
          if (contacts[i].surface === "floor") {
            landing = { time: contacts[i].time, position: contacts[i].position };
            break;
          }
        }
      }

      if (state.position[2] > apex.position[2]) {
        apex = { time: state.time, position: [...state.position] as Vector3 };
      }

      if (target !== null) {
        let entered: boolean;
        let d: number;
        if (target.mouth !== undefined) {
          // Tilted opening: in only through the mouth face, inward, inside its outline
          const c = mouthCrossing(previousPosition, state.position, target.mouth);
          entered = c !== null && Math.abs(c.u) <= target.mouth.halfU && Math.abs(c.v) <= target.mouth.halfV;
          d = entered ? 0 : distanceToMouth(state.position, target.mouth);
        } else {
          d = distanceToBox(state.position, target.min, target.max);
          entered = d === 0 && (!target.requireDescending || state.velocity[2] < 0);
        }
        if (d < closestDistance) {
          closestDistance = d;
          closestTime = state.time;
        }
        if (!targetHit && entered) {
          targetHit = true;
          targetTime = state.time;
          entryPosition = [...state.position] as Vector3;
          entryVelocity = [...state.velocity] as Vector3;
          if (opts.stopOnTargetHit) break;
        }
      }

      // Left the field over the wall
      if (
        Math.abs(state.position[0]) > this.field.halfX + 2 * r ||
        Math.abs(state.position[1]) > this.field.halfY + 2 * r
      ) {
        leftField = true;
        if (state.position[2] <= r) break;
      }

      if (opts.stopAfterLanding && landing !== null) break;
      if (opts.stopWhenAtRest && this.isAtRest(state)) break;
    }
    record();

    return {
      samples,
      contacts,
      target:
        target === null
          ? null
          : {
              hit: targetHit,
              time: targetTime,
              entryPosition,
              entryVelocity,
              closestDistance: targetHit ? 0 : closestDistance,
              closestTime: targetHit ? targetTime! : closestTime
            },
      landing,
      apex,
      crossing,
      leftField,
      finalState: state
    };
  }

  private isAtRest(state: BallState): boolean {
    return (
      state.position[2] <= this.piece.radius + 1e-3 &&
      length(state.velocity) < 0.02 &&
      length(state.angularVelocity) * this.piece.radius < 0.02
    );
  }
}

/** Integrates a quaternion with the average angular velocity over the step (exact for constant w). */
function integrateOrientation(q: Rotation3d, w0: Vector3, w1: Vector3, dt: number): Rotation3d {
  const w: Vector3 = [(w0[0] + w1[0]) / 2, (w0[1] + w1[1]) / 2, (w0[2] + w1[2]) / 2];
  const angle = length(w) * dt;
  if (angle < 1e-12) return q;
  const axis = normalize(w);
  const s = Math.sin(angle / 2);
  const dq: Rotation3d = [Math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s];
  // dq * q (world-frame rotation)
  const [w1q, x1, y1, z1] = dq;
  const [w2, x2, y2, z2] = q;
  const out: Rotation3d = [
    w1q * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1q * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1q * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1q * z2 + x1 * y2 - y1 * x2 + z1 * w2
  ];
  const n = Math.hypot(out[0], out[1], out[2], out[3]);
  return [out[0] / n, out[1] / n, out[2] / n, out[3] / n];
}

/** Convenience: fresh state at rest orientation. */
export function makeBallState(position: Vector3, velocity: Vector3, angularVelocity: Vector3): BallState {
  return { position, velocity, angularVelocity, orientation: [1, 0, 0, 0], time: 0 };
}
