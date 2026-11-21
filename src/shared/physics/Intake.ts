// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * What the robot holds and how it gets more.
 *
 * A robot starts holding its 4 preloads (G304) and may control at most 4 scoring elements at once (G407). Each
 * shot uses one up. A shot that misses lands on the floor, and the robot only gets it back by driving its intake
 * over it (a piece that scored is in the CELL, not on the floor).
 */

import { DrivetrainConfig } from "./DriveSim";

export const HOLD_CAPACITY = 4;
/** A piece is picked up when its center is inside the robot's footprint or this far ahead of the front (m). */
export const INTAKE_REACH = 0.1;

export interface FloorPiece {
  x: number;
  y: number;
  /** True for a piece that scored: it is in the CELL, so it can't be picked up. */
  hit: boolean;
}

/**
 * Picks up every missed piece the robot is over, until it holds `capacity`.
 *
 * @returns the pieces left on the floor, the new number held and how many were picked up
 */
export function collectPieces<T extends FloorPiece>(
  pieces: T[],
  robot: { x: number; y: number; heading: number },
  config: DrivetrainConfig,
  held: number,
  pieceRadius: number,
  capacity: number = HOLD_CAPACITY
): { pieces: T[]; held: number; picked: number } {
  if (held >= capacity || pieces.length === 0) return { pieces, held, picked: 0 };
  const c = Math.cos(robot.heading);
  const s = Math.sin(robot.heading);
  const hl = config.length / 2;
  const hw = config.width / 2;
  let count = held;
  const left = pieces.filter((piece) => {
    if (piece.hit || count >= capacity) return true;
    const dx = piece.x - robot.x;
    const dy = piece.y - robot.y;
    const lx = dx * c + dy * s;
    const ly = -dx * s + dy * c;
    if (lx < -hl || lx > hl + INTAKE_REACH || Math.abs(ly) > hw + pieceRadius) return true;
    count++;
    return false;
  });
  return { pieces: left, held: count, picked: count - held };
}
