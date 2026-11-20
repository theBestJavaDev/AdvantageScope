// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Season presets: the scoring targets, field zones, obstacles and game pieces of an FTC game, in
 * the simulator's field frame.
 *
 * SIM FIELD FRAME (used by Shot Lab / Drive Sim): origin at the field center, viewed from the
 * audience: +X toward the BLUE alliance wall (right), +Y away from the audience (toward the rear
 * wall), +Z up. Red alliance wall is x = -72 in. If your robot code uses a different FTC field
 * frame, rotate its poses into this one before comparing.
 */

import { TargetMouth } from "./PhysicsEngine";

const IN = 0.0254;

// ---------------------------------------------------------------------------------------------
// Log frame
// ---------------------------------------------------------------------------------------------

/**
 * Positions written to the log use FTC's traditional field frame (AdvantageScope "center-rotated"),
 * the frame the bundled 2026-2027 FTC field declares. The 2D/3D field tabs then place simulated
 * robots, pieces and trajectories exactly where the sim had them. Mapping (sim -> log):
 * (x, y) -> (-y, x), heading + 90 deg. AdvantageScope maps back with (x, y) -> (y, -x).
 */
export function simToLogTranslation<T extends number[]>(p: T): T {
  const out = [...p] as T;
  out[0] = -p[1];
  out[1] = p[0];
  return out;
}

export function simToLogHeading(heading: number): number {
  return heading + Math.PI / 2;
}

/** The other HIVE's up CELL under the field's rotational symmetry (red rear <-> blue audience). */
export function mirrorCell(cell: HiveCell): HiveCell {
  return cell === "rear" ? "audience" : "rear";
}

export type Alliance = "red" | "blue";
export type HiveCell = "rear" | "audience";

export interface PresetPiece {
  id: string;
  name: string;
  /** CSS color for drawing. */
  color: string;
  diameterM: number;
  massKg: number;
  dragCoefficient: number;
  magnusScale: number;
  restitution: number;
}

export interface PresetTarget {
  id: string;
  name: string;
  alliance: Alliance;
  /** launch = pieces are shot in (Shot Lab target); place = scored by placing (drawn only). */
  kind: "launch" | "place";
  x: number;
  y: number;
  /** Height of the opening's center (m). */
  z: number;
  sizeX: number;
  sizeY: number;
  /** Vertical extent of the scoring volume (m). */
  depth: number;
  requireDescending: boolean;
  /** A tilted opening that is the real scoring surface (BIOBUZZ CELL mouths). */
  mouth?: TargetMouth;
}

export interface PresetZone {
  id: string;
  name: string;
  alliance: Alliance;
  kind: "loading" | "garden" | "alliance-area";
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

/** Floor-level obstacle the robot collides with (axis-aligned box, m). */
export interface PresetObstacle {
  id: string;
  name: string;
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

export interface GamePreset {
  id: string;
  name: string;
  season: string;
  /** Where the numbers came from. */
  source: string;
  wallHeight: number;
  pieces: PresetPiece[];
  /** HIVE CELLs: both cells of both HIVEs (which one is "up" is chosen by the user). */
  cells: (PresetTarget & { hiveCell: HiveCell })[];
  flowers: PresetTarget[];
  zones: PresetZone[];
  obstacles: PresetObstacle[];
  /** HIVE frame outline for drawing (robots can drive under the HIVEs, only the legs block). */
  frame: { minX: number; maxX: number; minY: number; maxY: number };
  /** HIVE pivot positions (for drawing the arms). */
  hives: { alliance: Alliance; x: number; y: number }[];
  starts: Record<Alliance, { x: number; y: number; headingDeg: number }>;
  /** Values that are estimates, not from the manual. Shown in the UI. */
  estimates: string[];
}

// ---------------------------------------------------------------------------------------------
// BIOBUZZ (2026-2027), Competition Manual TU10
// ---------------------------------------------------------------------------------------------

// CELL geometry: the same numbers the team's robot-code sim uses (BioBuzz-2026 repo, globals/Constants
// HIVE_* / CELL_* and sim/BioBuzzField), measured from the field CAD. A CELL is a tray on the HIVE bar,
// closed everywhere except its outer end, the mouth. In the bar's frame (x across the bar, y along it,
// z square to it, pivot at the origin) the tray runs from 9.644 in to the mouth at 21.394 in, is
// +/-10.07 in wide and spans z from -1.47 in (floor) to 12.52 in (roof ridge). The bar tips +/-30 deg
// about the pivot, 43.95 in up. Check: the up mouth then spans 53.4 - 65.5 in above the tiles (manual
// Fig 9-10: 53.5 - 65.6 in).
const HIVE_PIVOT_HEIGHT = 43.95 * IN;
const HIVE_TILT = (30 * Math.PI) / 180;
const CELL_MOUTH_DISTANCE = 21.394 * IN;
const CELL_HALF_WIDTH = 10.07 * IN;
const CELL_FLOOR_OFFSET = -1.47 * IN;
const CELL_RIDGE_HEIGHT = 12.52 * IN;
/** HIVEs are 25.5 in center to center (Fig 9-10); red on the red (left) side. */
const HIVE_X = (25.5 / 2) * IN;

/** The up CELL's mouth in the sim field frame: faces outward along the tipped bar (and 30 deg up). */
export function cellMouth(alliance: Alliance, hiveCell: HiveCell): TargetMouth {
  const pivotX = (alliance === "red" ? -1 : 1) * HIVE_X;
  const side = hiveCell === "rear" ? 1 : -1; // +y end of the bar faces the rear wall
  const angle = side * HIVE_TILT; // raises this end
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  // Bar frame -> field: rotate about x by `angle`, then move to the pivot
  const toField = (y: number, z: number): [number, number] => [y * c - z * s, y * s + z * c];
  const midZ = (CELL_FLOOR_OFFSET + CELL_RIDGE_HEIGHT) / 2;
  const [cy, cz] = toField(side * CELL_MOUTH_DISTANCE, midZ);
  const [ny, nz] = toField(side, 0);
  const [vy, vz] = toField(0, 1);
  return {
    center: [pivotX, cy, HIVE_PIVOT_HEIGHT + cz],
    normal: [0, ny, nz],
    u: [1, 0, 0],
    v: [0, vy, vz],
    halfU: CELL_HALF_WIDTH,
    halfV: (CELL_RIDGE_HEIGHT - CELL_FLOOR_OFFSET) / 2
  };
}

function cell(alliance: Alliance, hiveCell: HiveCell): PresetTarget & { hiveCell: HiveCell } {
  const mouth = cellMouth(alliance, hiveCell);
  // Bounding box of the mouth rectangle (for drawing, distances and the dispersion plane)
  const corners = [-1, 1].flatMap((a) =>
    [-1, 1].map((b) =>
      [0, 1, 2].map((i) => mouth.center[i] + a * mouth.halfU * mouth.u[i] + b * mouth.halfV * mouth.v[i])
    )
  );
  const lo = [0, 1, 2].map((i) => Math.min(...corners.map((p) => p[i])));
  const hi = [0, 1, 2].map((i) => Math.max(...corners.map((p) => p[i])));
  return {
    id: "cell-" + alliance + "-" + hiveCell,
    name: (alliance === "red" ? "Red" : "Blue") + " HIVE · " + (hiveCell === "rear" ? "rear" : "audience") + " CELL",
    alliance,
    hiveCell,
    kind: "launch",
    x: mouth.center[0],
    y: mouth.center[1],
    z: mouth.center[2],
    sizeX: hi[0] - lo[0],
    sizeY: Math.max(hi[1] - lo[1], 0.02),
    depth: hi[2] - lo[2],
    requireDescending: false,
    mouth
  };
}

function flower(id: string, x: number, y: number): PresetTarget {
  return {
    id,
    name: "FLOWER",
    alliance: "red",
    kind: "place",
    x,
    y,
    // Top opening ~4 in diameter, ~21.5 in above the TILES (9.7)
    z: 21.5 * IN,
    sizeX: 4 * IN,
    sizeY: 4 * IN,
    depth: 2 * IN,
    requireDescending: true
  };
}

const HALF = 72 * IN;
const FLOWER_INSET = 3 * IN; // flowers hang on the perimeter wall; inset is an estimate

// Frame base: 49.46 in wide (across, X) by 38.95 in deep (Y) (9.6.1); the A-frame feet are at its corners
const FRAME_HX = (49.46 / 2) * IN;
const FRAME_HY = (38.95 / 2) * IN;
const LEG = 1.5 * IN;

function leg(id: string, sx: number, sy: number): PresetObstacle {
  const cx = sx * FRAME_HX;
  const cy = sy * FRAME_HY;
  return { id, name: "HIVE frame leg", minX: cx - LEG, maxX: cx + LEG, minY: cy - LEG, maxY: cy + LEG };
}

export const BIOBUZZ: GamePreset = {
  id: "biobuzz",
  name: "BIOBUZZ",
  season: "2026-2027",
  source: "FTC BIOBUZZ Competition Manual TU10 (Sections 9.2-9.8; Figures 9-2, 9-4, 9-9, 9-10)",
  // Not in the manual: height of the am-0481 perimeter kit
  wallHeight: 12 * IN,
  pieces: [
    {
      id: "pollen",
      name: "POLLEN (2.8 in, yellow)",
      color: "#f2c94c",
      diameterM: 2.8 * IN,
      // Perforated polyethylene ball (Gopher ResisDent). Mass/drag/bounce are estimates: weigh one!
      massKg: 0.018,
      dragCoefficient: 0.6,
      magnusScale: 0.35,
      restitution: 0.45
    },
    {
      id: "nectar",
      name: "NECTAR (3.6 in, alliance color)",
      color: "#e5484d",
      diameterM: 3.6 * IN,
      massKg: 0.03,
      dragCoefficient: 0.6,
      magnusScale: 0.35,
      restitution: 0.45
    }
  ],
  cells: [cell("red", "rear"), cell("red", "audience"), cell("blue", "rear"), cell("blue", "audience")],
  // One FLOWER on each wall, on the tile seam 2 tiles from the corner (Fig 9-2 / 9-4), rotationally symmetric
  flowers: [
    flower("flower-rear", -24 * IN, HALF - FLOWER_INSET),
    flower("flower-blue", HALF - FLOWER_INSET, 24 * IN),
    flower("flower-audience", 24 * IN, -HALF + FLOWER_INSET),
    flower("flower-red", -HALF + FLOWER_INSET, -24 * IN)
  ],
  zones: [
    // LOADING ZONE: ~23 in x 11 in against the alliance's wall (Fig 9-2, 9-3)
    {
      id: "lz-red",
      name: "Red LOADING ZONE",
      alliance: "red",
      kind: "loading",
      minX: -HALF,
      maxX: -HALF + 11 * IN,
      minY: 24 * IN,
      maxY: 47 * IN
    },
    {
      id: "lz-blue",
      name: "Blue LOADING ZONE",
      alliance: "blue",
      kind: "loading",
      minX: HALF - 11 * IN,
      maxX: HALF,
      minY: -47 * IN,
      maxY: -24 * IN
    },
    // GARDEN: ~23 in x 2 in strip in opposite corners (red: audience-left, blue: rear-right)
    {
      id: "garden-red",
      name: "Red GARDEN",
      alliance: "red",
      kind: "garden",
      minX: -HALF,
      maxX: -HALF + 23 * IN,
      minY: -HALF,
      maxY: -HALF + 2 * IN
    },
    {
      id: "garden-blue",
      name: "Blue GARDEN",
      alliance: "blue",
      kind: "garden",
      minX: HALF - 23 * IN,
      maxX: HALF,
      minY: HALF - 2 * IN,
      maxY: HALF
    },
    // ALLIANCE AREA: ~97 in wide x 54 in deep outside the alliance wall (9.3, Fig 9-6)
    {
      id: "aa-red",
      name: "Red ALLIANCE AREA",
      alliance: "red",
      kind: "alliance-area",
      minX: -HALF - 54 * IN,
      maxX: -HALF,
      minY: -48.5 * IN,
      maxY: 48.5 * IN
    },
    {
      id: "aa-blue",
      name: "Blue ALLIANCE AREA",
      alliance: "blue",
      kind: "alliance-area",
      minX: HALF,
      maxX: HALF + 54 * IN,
      minY: -48.5 * IN,
      maxY: 48.5 * IN
    }
  ],
  obstacles: [leg("leg-1", -1, -1), leg("leg-2", -1, 1), leg("leg-3", 1, -1), leg("leg-4", 1, 1)],
  frame: { minX: -FRAME_HX, maxX: FRAME_HX, minY: -FRAME_HY, maxY: FRAME_HY },
  hives: [
    { alliance: "red", x: -HIVE_X, y: 0 },
    { alliance: "blue", x: HIVE_X, y: 0 }
  ],
  // Start touching your own wall, on your side, outside the LOADING ZONE (G304)
  starts: {
    red: { x: -HALF + 9 * IN, y: -12 * IN, headingDeg: 0 },
    blue: { x: HALF - 9 * IN, y: 12 * IN, headingDeg: 180 }
  },
  estimates: [
    "POLLEN / NECTAR mass, drag, Magnus and bounce (not in the manual: weigh and film real pieces)",
    "Perimeter wall height (12 in, from the am-0481 kit)",
    "FLOWER inset from the wall and the CELL opening footprint (from figures)"
  ]
};

export const GAME_PRESETS: GamePreset[] = [BIOBUZZ];

export function getGamePreset(id: string): GamePreset | null {
  return GAME_PRESETS.find((p) => p.id === id) ?? null;
}

/** The launch target for an alliance given which CELL of its HIVE is facing up. */
export function upwardCell(preset: GamePreset, alliance: Alliance, up: HiveCell): PresetTarget {
  return preset.cells.find((c) => c.alliance === alliance && c.hiveCell === up)!;
}
