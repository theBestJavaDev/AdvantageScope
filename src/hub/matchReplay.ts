// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * The random match as a world for the 3D view. The robot-code log is a perfect robot: every shot it
 * fires goes in and the HIVEs tip on cue. In a random match some shots miss, so drawing the logged
 * POLLEN and HIVEs would show a missed shot still going in and its HIVE still tipping. This writes
 * copies of those tracks that follow the match instead:
 *
 *  - POLLEN: the logged pieces, minus each overridden shot's logged ball (followed frame to frame
 *    from its launch), which the match draws from its own physics instead.
 *  - HIVEs: as logged until a HIVE doesn't tip in this match; from then on it stays put, and the
 *    pieces in it (POLLEN and NECTAR) stay where they were instead of falling out.
 */

import { AutoShot, RandomMatch } from "../shared/physics/AutoShotCheck";
import { Alliance, simToLogTranslation } from "../shared/physics/GamePresets";
import { Vector3 } from "../shared/physics/vector";

const P = "RealOutputs/Field/";
export const MATCH_WORLD_PREFIX = "/SolversSim/AutoCheck/Match/World";
/** Logged key -> the match's copy, for the 3D view's sources. */
export const MATCH_WORLD_KEYS: Record<string, string> = {
  [P + "Pollen"]: MATCH_WORLD_PREFIX + "/Pollen",
  [P + "NectarRed"]: MATCH_WORLD_PREFIX + "/NectarRed",
  [P + "NectarBlue"]: MATCH_WORLD_PREFIX + "/NectarBlue",
  [P + "HiveRed"]: MATCH_WORLD_PREFIX + "/HiveRed",
  [P + "HiveBlue"]: MATCH_WORLD_PREFIX + "/HiveBlue"
};
const HIVE_KEYS: Record<Alliance, string> = { red: P + "HiveRed", blue: P + "HiveBlue" };
const PIECE_KEYS = [P + "Pollen", P + "NectarRed", P + "NectarBlue"];

const STEP = 0.02;
/** Robot sim's swing time (BioBuzzField.TIP_SWING_SECONDS). */
const SWING_SECONDS = 4.0;
const G = 9.80665;
/** HIVE pivots in the sim frame (25.5 in apart, 43.95 in up): pieces near one are "in the HIVE". */
const HIVE_X = (25.5 / 2) * 0.0254;
const IN_HIVE_RADIUS = 0.75;
const IN_HIVE_MIN_Z = 0.8;

interface Tracker {
  position: Vector3;
  velocity: Vector3;
  /** "hide": drop this ball from the drawing; "hold": draw it at `held` instead of where it went. */
  mode: "hide" | "hold";
  held: Vector3;
  lost: number;
}

/**
 * Reads a number field at increasing times without a log query per frame: the whole series once,
 * then a cursor that only moves forward.
 */
function sampler(key: string): (t: number) => number | null {
  const set = window.log.getNumber(key, -Infinity, Infinity);
  let i = 0;
  return (t: number) => {
    if (set === undefined || set.timestamps.length === 0 || t < set.timestamps[0] - 1e-9) return null;
    while (i + 1 < set.timestamps.length && set.timestamps[i + 1] <= t + 1e-9) i++;
    return set.values[i];
  };
}

/** Translation3d-array reader for increasing times (log frame). */
function piecesReader(key: string): (t: number) => Vector3[] {
  const length = sampler(key + "/length");
  const axes: ((t: number) => number | null)[][] = [];
  return (t: number) => {
    const n = length(t) ?? 0;
    while (axes.length < n) {
      const i = axes.length;
      axes.push(["x", "y", "z"].map((axis) => sampler(key + "/" + i + "/" + axis)));
    }
    const out: Vector3[] = [];
    for (let i = 0; i < n; i++) {
      const p = axes[i].map((read) => read(t));
      if (p.every((v) => v !== null)) out.push(p as Vector3);
    }
    return out;
  };
}

/** Pose3d reader for increasing times: translation + quaternion (w, x, y, z). */
function poseReader(key: string) {
  const parts = [
    "translation/x",
    "translation/y",
    "translation/z",
    "rotation/q/w",
    "rotation/q/x",
    "rotation/q/y",
    "rotation/q/z"
  ].map((child) => sampler(key + "/" + child));
  return (t: number): { translation: Vector3; rotation: [number, number, number, number] } | null => {
    const v = parts.map((read) => read(t));
    if (v.some((x) => x === null)) return null;
    const n = v as number[];
    return { translation: [n[0], n[1], n[2]], rotation: [n[3], n[4], n[5], n[6]] };
  };
}

function dist(a: Vector3, b: Vector3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/** Pairs each tracker with the nearest unclaimed piece this frame; returns the claimed indices. */
function follow(trackers: Tracker[], pieces: Vector3[], dt: number): Set<number> {
  const claimed = new Set<number>();
  for (const tr of trackers) {
    if (tr.lost > 5) continue;
    const airborne = tr.position[2] > 0.08;
    const guess: Vector3 = [
      tr.position[0] + tr.velocity[0] * dt,
      tr.position[1] + tr.velocity[1] * dt,
      tr.position[2] + tr.velocity[2] * dt - (airborne ? 0.5 * G * dt * dt : 0)
    ];
    const gate = 0.06 + 0.5 * Math.hypot(...tr.velocity) * dt;
    let best = -1;
    let bestDistance = Infinity;
    pieces.forEach((p, i) => {
      if (claimed.has(i)) return;
      const d = dist(p, guess);
      if (d < bestDistance) {
        bestDistance = d;
        best = i;
      }
    });
    if (best >= 0 && bestDistance <= gate) {
      claimed.add(best);
      const p = pieces[best];
      tr.velocity = [(p[0] - tr.position[0]) / dt, (p[1] - tr.position[1]) / dt, (p[2] - tr.position[2]) / dt];
      tr.position = p;
      tr.lost = 0;
    } else {
      tr.lost++;
      tr.position = guess;
      if (airborne) tr.velocity = [tr.velocity[0], tr.velocity[1], tr.velocity[2] - G * dt];
    }
  }
  return claimed;
}

/** Writes the match's POLLEN, NECTAR and HIVE tracks (MATCH_WORLD_KEYS). */
export function writeMatchWorld(match: RandomMatch, shots: AutoShot[]) {
  const log = window.log;
  log.deleteFieldsWithPrefix(MATCH_WORLD_PREFIX);
  const [logStart, logEnd] = log.getTimestampRange();
  const frozenFrom: Record<Alliance, number> = { red: Infinity, blue: Infinity };
  match.frozen.forEach((f) => (frozenFrom[f.alliance] = f.from));

  // Logged balls to hide: every shot the match draws itself, from its launch
  const hideStarts = match.shots
    .filter((s) => s.override)
    .map((s) => ({
      time: s.time,
      position: simToLogTranslation([...shots[s.index].position]) as Vector3,
      velocity: simToLogTranslation([...shots[s.index].velocity]) as Vector3
    }))
    .sort((a, b) => a.time - b.time);

  for (const key of PIECE_KEYS) {
    if (!log.getFieldKeys().includes(key)) continue;
    const trackers: Tracker[] = [];
    const piecesAt = piecesReader(key);
    let nextHide = 0;
    const holdDone: Record<Alliance, boolean> = { red: false, blue: false };
    for (let t = logStart; t <= logEnd + 1e-9; t += STEP) {
      const pieces = piecesAt(t);
      // New balls to hide (POLLEN only: shots are POLLEN)
      while (key === P + "Pollen" && nextHide < hideStarts.length && hideStarts[nextHide].time <= t) {
        const start = hideStarts[nextHide++];
        // Where the launched ball is at this frame (frames don't line up with the launch)
        const dt = t - start.time;
        const position: Vector3 = [
          start.position[0] + start.velocity[0] * dt,
          start.position[1] + start.velocity[1] * dt,
          start.position[2] + start.velocity[2] * dt - 0.5 * G * dt * dt
        ];
        const velocity: Vector3 = [start.velocity[0], start.velocity[1], start.velocity[2] - G * dt];
        // Placed one step back so this frame's `follow` lands on it
        trackers.push({
          position: [
            position[0] - velocity[0] * STEP,
            position[1] - velocity[1] * STEP,
            position[2] - velocity[2] * STEP
          ],
          velocity,
          mode: "hide",
          held: position,
          lost: 0
        });
      }
      // A HIVE that stops: hold whatever is in it right now
      for (const alliance of ["red", "blue"] as Alliance[]) {
        if (holdDone[alliance] || t < frozenFrom[alliance]) continue;
        holdDone[alliance] = true;
        const pivot = simToLogTranslation([(alliance === "red" ? -1 : 1) * HIVE_X, 0, 0]);
        pieces.forEach((p) => {
          if (p[2] > IN_HIVE_MIN_Z && Math.hypot(p[0] - pivot[0], p[1] - pivot[1]) < IN_HIVE_RADIUS) {
            trackers.push({ position: p, velocity: [0, 0, 0], mode: "hold", held: [...p] as Vector3, lost: 0 });
          }
        });
      }
      const claimed = follow(trackers, pieces, STEP);
      const out = pieces.filter((_, i) => !claimed.has(i));
      trackers.forEach((tr) => {
        if (tr.mode === "hold") out.push(tr.held);
      });
      log.putTranslation3dArray(MATCH_WORLD_KEYS[key], t, out);
    }
  }

  // HIVEs: as logged, then held where they were when the match stops them
  for (const alliance of ["red", "blue"] as Alliance[]) {
    const key = HIVE_KEYS[alliance];
    const poseAt = poseReader(key);
    const poseLaterAt = poseReader(key); // a second cursor, for the delayed swing
    const lateSwing = match.late.find((l) => l.alliance === alliance) ?? null;
    let held: ReturnType<typeof poseAt> = null;
    for (let t = logStart; t <= logEnd + 1e-9; t += STEP) {
      let pose = poseAt(t);
      // A late tip: the logged swing, started `delay` later (held where it was until then)
      if (lateSwing !== null && t >= lateSwing.from && t <= lateSwing.from + SWING_SECONDS + lateSwing.delay + 0.2) {
        pose = poseLaterAt(Math.max(lateSwing.from, t - lateSwing.delay));
      }
      if (t >= frozenFrom[alliance]) {
        if (held === null) held = pose; // the first frame it stops: where it was
        pose = held;
      }
      if (pose !== null) log.putPose3d(MATCH_WORLD_KEYS[key], t, pose.translation, pose.rotation);
    }
  }
}
