// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import LoggableType from "../shared/log/LoggableType";
import { AutoShot, HiveSwing } from "../shared/physics/AutoShotCheck";
import { Alliance, HiveCell } from "../shared/physics/GamePresets";
import { Vector3 } from "../shared/physics/vector";

/**
 * Reads the shots out of a BioBuzz SimLoop RLOG (the robot repo's simAuto / sim tests):
 *
 *  - launches: RealOutputs/Sim/Launch/count increments; the ball id comes from
 *    Sim/Pollen/transitions ("30 HELD->IN_FLIGHT launch") and the robot sim's verdict from that ball's
 *    next transition ("IN_FLIGHT->SCORED red-audience" or "IN_FLIGHT->LOOSE ...").
 *  - launch point and velocity: the new ball in RealOutputs/Field/Pollen is tracked for the first
 *    frames of its flight and fitted to p0 + v0 t - g t^2 / 2 (drag is negligible over 0.1 s).
 *  - robot velocity: from RealOutputs/Field/Robot.
 *
 * Field/ keys are in AdvantageScope's FTC frame (+X toward the audience, +Y toward the blue wall);
 * they are turned into the sim frame (+X toward blue, +Y away from the audience): (x, y) -> (y, -x).
 */

const P = "RealOutputs/";
const LAUNCH_COUNT = P + "Sim/Launch/count";
const POLLEN = P + "Field/Pollen";
const ROBOT = P + "Field/Robot";
const TICK = 0.02;
const FIT_FRAMES = 6;
const G = 9.80665;
/** How long a HIVE bar takes to swing over in the robot sim (BioBuzzField.TIP_SWING_SECONDS). */
const SWING_SECONDS = 4.0;

export function isRobotSimLog(): boolean {
  return window.log.getType(LAUNCH_COUNT) === LoggableType.Number && window.log.getFieldKeys().includes(POLLEN);
}

function num(key: string, t: number): number | null {
  const set = window.log.getNumber(key, t, t);
  return set !== undefined && set.values.length > 0 && set.timestamps[0] <= t + 1e-9 ? set.values[0] : null;
}

/** All loose / flying POLLEN at time t, in the sim frame. */
function pollenAt(t: number): Vector3[] {
  const n = num(POLLEN + "/length", t) ?? 0;
  const out: Vector3[] = [];
  for (let i = 0; i < n; i++) {
    const x = num(POLLEN + "/" + i + "/x", t);
    const y = num(POLLEN + "/" + i + "/y", t);
    const z = num(POLLEN + "/" + i + "/z", t);
    if (x !== null && y !== null && z !== null) out.push([y, -x, z]);
  }
  return out;
}

function robotAt(t: number): [number, number] | null {
  const x = num(ROBOT + "/translation/x", t);
  const y = num(ROBOT + "/translation/y", t);
  return x === null || y === null ? null : [y, -x];
}

function dist(a: Vector3, b: Vector3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

interface Transition {
  t: number;
  id: string;
  from: string;
  to: string;
  detail: string;
}

function transitions(): Transition[] {
  const out: Transition[] = [];
  for (const key of window.log.getFieldKeys()) {
    if (!/^RealOutputs\/Sim\/Pollen\/transitions\/\d+$/.test(key)) continue;
    const set = window.log.getString(key, -Infinity, Infinity);
    set?.values.forEach((value, i) => {
      const m = /^(\S+)\s+(\S+)->(\S+)\s*(.*)$/.exec(value);
      if (m) out.push({ t: set.timestamps[i], id: m[1], from: m[2], to: m[3], detail: m[4] });
    });
  }
  return out.sort((a, b) => a.t - b.t);
}

/** Which CELL of a HIVE faces up at time t: the logged name, else the HIVE angle (< 0 = audience side). */
function upCellAt(alliance: Alliance, t: number): HiveCell {
  const set = window.log.getString(P + "Sim/Hive/" + alliance + "UpCell", t, t);
  const name = set !== undefined && set.values.length > 0 ? set.values[0] : null;
  if (name !== null) return name.endsWith("audience") ? "audience" : "rear";
  const angle = num(P + "Sim/Hive/" + alliance + "AngleRadians", t);
  return angle !== null && angle > 0 ? "rear" : "audience";
}

function cellFromName(name: string): { alliance: Alliance; hiveCell: HiveCell } | null {
  const m = /^(red|blue)-(audience|far)/.exec(name);
  return m ? { alliance: m[1] as Alliance, hiveCell: m[2] === "audience" ? "audience" : "rear" } : null;
}

/** Least-squares launch point + velocity from tracked ball positions (gravity known). */
function fitLaunch(track: { t: number; p: Vector3 }[]): { position: Vector3; velocity: Vector3 } {
  const t0 = track[0].t;
  const position: Vector3 = [0, 0, 0];
  const velocity: Vector3 = [0, 0, 0];
  for (let axis = 0; axis < 3; axis++) {
    // y(t) = a + b t, with gravity removed from z
    let sx = 0,
      sy = 0,
      sxx = 0,
      sxy = 0;
    for (const s of track) {
      const dt = s.t - t0;
      const y = s.p[axis] + (axis === 2 ? 0.5 * G * dt * dt : 0);
      sx += dt;
      sy += y;
      sxx += dt * dt;
      sxy += dt * y;
    }
    const n = track.length;
    const b = (n * sxy - sx * sy) / Math.max(1e-12, n * sxx - sx * sx);
    position[axis] = (sy - b * sx) / n;
    velocity[axis] = b;
  }
  return { position, velocity };
}

/** Every shot the robot code fired in the open log. */
export function extractAutoShots(): AutoShot[] {
  if (!isRobotSimLog()) return [];
  const counts = window.log.getNumber(LAUNCH_COUNT, -Infinity, Infinity);
  if (counts === undefined) return [];
  const launchTimes: number[] = [];
  for (let i = 1; i < counts.values.length; i++) {
    for (let k = counts.values[i - 1]; k < counts.values[i]; k++) launchTimes.push(counts.timestamps[i]);
  }

  const all = transitions();
  const shots: AutoShot[] = [];
  const usedIds = new Set<string>();
  launchTimes.forEach((t) => {
    const robot = robotAt(t);
    if (robot === null) return;

    // The new ball: off the floor, next to the shooter, and not where any ball was a tick earlier
    const before = pollenAt(t - TICK);
    const candidates = pollenAt(t).filter(
      (p) =>
        p[2] > 0.15 && before.every((q) => dist(p, q) > 0.02) && Math.hypot(p[0] - robot[0], p[1] - robot[1]) < 0.45
    );
    if (candidates.length === 0) return;
    const first = candidates.reduce((a, b) => (a[2] <= b[2] ? a : b)); // lowest = just left the shooter
    const track = [{ t, p: first }];
    for (let k = 1; k < FIT_FRAMES; k++) {
      const tk = t + k * TICK;
      const last = track[track.length - 1].p;
      const prev = track.length > 1 ? track[track.length - 2].p : last;
      const guess: Vector3 = [2 * last[0] - prev[0], 2 * last[1] - prev[1], 2 * last[2] - prev[2] - G * TICK * TICK];
      const now = pollenAt(tk);
      if (now.length === 0) break;
      const next = now.reduce((a, b) => (dist(a, guess) <= dist(b, guess) ? a : b));
      // First step has no velocity to predict with (a fast shot moves ~10 cm per tick): wider gate
      if (dist(next, guess) > (track.length === 1 ? 0.3 : 0.08)) break;
      track.push({ t: tk, p: next });
    }
    if (track.length < 3) return;
    const { position, velocity } = fitLaunch(track);

    // Robot velocity around the launch
    const ra = robotAt(t - 2 * TICK);
    const rb = robotAt(t + 2 * TICK);
    const robotVelocity: [number, number] =
      ra !== null && rb !== null ? [(rb[0] - ra[0]) / (4 * TICK), (rb[1] - ra[1]) / (4 * TICK)] : [0, 0];

    // Ball id + what the robot sim decided for it
    const launch = all.find(
      (tr) =>
        Math.abs(tr.t - t) < TICK / 2 && tr.to === "IN_FLIGHT" && tr.detail.startsWith("launch") && !usedIds.has(tr.id)
    );
    const ballId = launch?.id ?? null;
    if (ballId !== null) usedIds.add(ballId);
    const outcome =
      ballId === null ? undefined : all.find((tr) => tr.id === ballId && tr.t > t && tr.from === "IN_FLIGHT");
    const robotSimResult: AutoShot["robotSimResult"] =
      outcome === undefined ? "unknown" : outcome.to === "SCORED" ? "in" : "miss";

    // Target: the CELL it scored in, else the up CELL of the nearer HIVE (red is on the -X side)
    const alliance: Alliance = position[0] < 0 ? "red" : "blue";
    const cell = (outcome && outcome.to === "SCORED" ? cellFromName(outcome.detail) : null) ?? {
      alliance,
      hiveCell: upCellAt(alliance, t)
    };

    const scoredTime = outcome !== undefined && outcome.to === "SCORED" ? outcome.t : undefined;
    shots.push({ time: t, ballId, position, velocity, robotVelocity, cell, robotSimResult, scoredTime });
  });
  return shots;
}

/**
 * Every HIVE tip in the log: when the bar started to swing, which CELL was up, and what it held
 * (Sim/Hive/<alliance>AngleRadians, <alliance>UpCell, <alliance>-<far|audience>/pollen|nectar).
 */
export function extractHiveSwings(): HiveSwing[] {
  const swings: HiveSwing[] = [];
  for (const alliance of ["red", "blue"] as Alliance[]) {
    const angles = window.log.getNumber(P + "Sim/Hive/" + alliance + "AngleRadians", -Infinity, Infinity);
    const tips = window.log.getNumber(P + "Sim/Hive/" + alliance + "Tips", -Infinity, Infinity);
    if (angles === undefined || tips === undefined) continue;
    // Each finished tip (the counter goes up when the bar lands); the swing began when the angle
    // first moved clearly within the swing time before that
    for (let k = 1; k < tips.values.length; k++) {
      if (tips.values[k] <= tips.values[k - 1]) continue;
      const end = tips.timestamps[k];
      const from = end - SWING_SECONDS - 0.5;
      let settled: number | null = null;
      let start: number | null = null;
      let previous = from;
      for (let i = 0; i < angles.timestamps.length && angles.timestamps[i] <= end; i++) {
        const t = angles.timestamps[i];
        if (t < from) {
          settled = angles.values[i];
          continue;
        }
        if (settled === null) settled = angles.values[i];
        if (Math.abs(angles.values[i] - settled) > 1e-3) {
          start = previous;
          break;
        }
        previous = t;
      }
      if (start === null) continue;
      const upCell = upCellAt(alliance, start);
      const cellKey = P + "Sim/Hive/" + alliance + "-" + (upCell === "rear" ? "far" : "audience");
      swings.push({
        alliance,
        start,
        upCell,
        pollen: num(cellKey + "/pollen", start) ?? 0,
        nectar: num(cellKey + "/nectar", start) ?? 0
      });
    }
  }
  return swings.sort((a, b) => a.start - b.start);
}
