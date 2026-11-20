// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * SolversScope physics checks: shots, spin, error model, BIOBUZZ preset, drive, battery and
 * calibration. Run with `npm run test:sim` (bundles this file with Rolldown and runs it in Node).
 * Each check states what would make it fail; exits non-zero if any check fails.
 */

import { centeringExitSpeed } from "../../src/shared/physics/AutoShotCheck";
import { computeSolution, fireShot } from "../../src/shared/physics/DriveShooting";
import { DEFAULT_DRIVETRAIN, advanceDrive, makeDriveState, robotCorners } from "../../src/shared/physics/DriveSim";
import { BIOBUZZ, cellMouth } from "../../src/shared/physics/GamePresets";
import {
  DEFAULT_ENVIRONMENT,
  DEFAULT_GAME_PIECE,
  FTC_FIELD,
  PhysicsEngine,
  makeBallState
} from "../../src/shared/physics/PhysicsEngine";
import { fitDragCoefficient, fitRestitution, simulateLandingDistance } from "../../src/shared/physics/ShotCalibration";
import {
  applyGamePreset,
  buildScenario,
  defaultShotLabConfig,
  measureMiss,
  mergeShotLabConfig,
  placeRobotForCell,
  runShotLab,
  solveShot
} from "../../src/shared/physics/ShotLab";
import { Random } from "../../src/shared/physics/vector";

let failures = 0;
let passes = 0;
function check(name: string, ok: boolean, info = "") {
  if (ok) passes++;
  else failures++;
  console.log((ok ? "PASS " : "FAIL ") + name + (info ? "  " + info : ""));
}

// ---------------------------------------------------------------- Flight physics
{
  // No air, no spin: must match the exact projectile range
  const engine = new PhysicsEngine(
    { ...DEFAULT_GAME_PIECE, dragCoefficient: 0, magnusScale: 0 },
    { ...DEFAULT_ENVIRONMENT, airDensity: 0 },
    { ...FTC_FIELD, halfX: 50, halfY: 50 }
  );
  const v = 6;
  const angle = Math.PI / 4;
  const r = engine.simulate(makeBallState([0, 0, 0.0635], [v * Math.cos(angle), 0, v * Math.sin(angle)], [0, 0, 0]), {
    timeStep: 0.0005,
    stopAfterLanding: true
  });
  const exact = (v * v * Math.sin(2 * angle)) / DEFAULT_ENVIRONMENT.gravity;
  check(
    "drag-free range matches the exact answer",
    r.landing !== null && Math.abs(r.landing.position[0] - exact) < 0.005,
    `${r.landing?.position[0].toFixed(4)} vs ${exact.toFixed(4)} m`
  );

  // Spin: backspin lifts (longer), topspin dives (shorter)
  const range = (w: number) => {
    const e = new PhysicsEngine(DEFAULT_GAME_PIECE, DEFAULT_ENVIRONMENT, { ...FTC_FIELD, halfX: 50, halfY: 50 });
    return e.simulate(makeBallState([0, 0, 0.3], [5, 0, 5], [0, w, 0]), { stopAfterLanding: true }).landing!
      .position[0];
  };
  const none = range(0);
  // +y spin with +x velocity gives w x v pointing down (topspin); -y is backspin
  check(
    "backspin > none > topspin (range)",
    range(-150) > none && none > range(150),
    `${range(-150).toFixed(2)} / ${none.toFixed(2)} / ${range(150).toFixed(2)} m`
  );
}

// ---------------------------------------------------------------- Shot Lab + BIOBUZZ preset
{
  const d = defaultShotLabConfig();
  check(
    "default is BIOBUZZ red, audience CELL up, POLLEN",
    d.game.preset === "biobuzz" &&
      d.game.alliance === "red" &&
      d.game.upCell === "audience" &&
      d.piece.diameterM === 2.8 * 0.0254
  );

  // CELL mouth geometry (robot repo's Constants) against the manual: up mouth 53.5 - 65.6 in (Fig 9-10)
  const mouth = cellMouth("red", "audience");
  const span = mouth.halfV * Math.abs(mouth.v[2]);
  const bottom = (mouth.center[2] - span) / 0.0254;
  const top = (mouth.center[2] + span) / 0.0254;
  check(
    "CELL mouth height matches the manual",
    Math.abs(bottom - 53.5) < 0.3 && Math.abs(top - 65.6) < 0.3,
    `${bottom.toFixed(2)}-${top.toFixed(2)} in`
  );
  check("audience CELL mouth faces the audience and up", mouth.normal[1] < -0.8 && mouth.normal[2] > 0.4);

  const behind = defaultShotLabConfig();
  behind.robot.y = 1.4; // behind the audience CELL (its closed end)
  placeRobotForCell(behind);
  check("robot is moved in front of the mouth", behind.robot.y < mouth.center[1], `y ${behind.robot.y.toFixed(2)} m`);
  const perfect = runShotLab({ ...d, error: { ...d.error, preset: "zero" } }, false);
  check("pre-solved default shot hits every time with no error", perfect.monteCarlo.hitRate === 1);
  const typical = runShotLab(d, false);
  check(
    "typical error gives a realistic hit rate (40-99.9%)",
    typical.monteCarlo.hitRate > 0.4 && typical.monteCarlo.hitRate < 1,
    `${(typical.monteCarlo.hitRate * 100).toFixed(1)}%`
  );

  for (const alliance of ["red", "blue"] as const) {
    for (const up of ["rear", "audience"] as const) {
      const c = JSON.parse(JSON.stringify(d));
      c.game = { ...c.game, alliance, upCell: up };
      applyGamePreset(c);
      placeRobotForCell(c);
      const s = solveShot(c);
      c.shooter.exitSpeed = s.exitSpeed;
      c.robot.headingDeg = s.headingDeg;
      const miss = measureMiss(buildScenario(c));
      check(
        `solves ${alliance} ${up} CELL`,
        s.converged && Math.abs(miss.range) < 0.02 && Math.abs(miss.lateral) < 0.02,
        `${s.exitSpeed.toFixed(2)} m/s`
      );
    }
  }

  const strafe = JSON.parse(JSON.stringify(d));
  strafe.robot.vx = -0.6;
  strafe.robot.vy = 0.8;
  const moving = solveShot(strafe);
  check(
    "shooting on the move leads the target",
    moving.converged && Math.abs(moving.headingDeg - d.robot.headingDeg) > 1,
    `${moving.headingDeg.toFixed(1)} vs ${d.robot.headingDeg.toFixed(1)} deg`
  );

  const migrated = mergeShotLabConfig({ target: { x: 1.45, y: 1.45 } });
  check("old saved configs move onto the preset target", migrated.game.preset === "biobuzz" && migrated.target.x < 0);
  const calibrated = defaultShotLabConfig();
  calibrated.calibration = { pollen: { massKg: 0.021 } };
  check(
    "calibrated piece values survive a reload",
    mergeShotLabConfig(JSON.parse(JSON.stringify(calibrated))).piece.massKg === 0.021
  );
}

// ---------------------------------------------------------------- Drive
{
  const ideal = { ...DEFAULT_DRIVETRAIN, batterySagVolts: 0, batteryDrainVoltsPerMin: 0 };
  const run = (
    cfg: typeof ideal,
    input: { x: number; y: number; turn: number },
    secs: number,
    start = makeDriveState(0, 0, 0)
  ) => {
    let s = start;
    for (let i = 0; i < secs * 60; i++) s = advanceDrive(s, input, cfg, false, 1 / 60, BIOBUZZ.obstacles);
    return s;
  };
  check(
    "acceleration is traction-limited",
    Math.abs(run(ideal, { x: 1, y: 0, turn: 0 }, 0.2, makeDriveState(-1.5, -1.2, 0)).vx - 0.8) < 0.02
  );
  check(
    "mecanum strafes at 80% speed",
    Math.abs(run(ideal, { x: 0, y: 1, turn: 0 }, 1, makeDriveState(-1.4, -1.5, 0)).vy - 1.44) < 1e-6
  );
  check(
    "tank cannot strafe",
    Math.hypot(run({ ...ideal, type: "tank" }, { x: 0, y: 1, turn: 0 }, 1, makeDriveState(-1.4, -1.5, 0)).vy) < 1e-9
  );
  const wall = run(ideal, { x: 1, y: 0, turn: 0 }, 4, makeDriveState(0.5, 1.4, 0));
  check(
    "the perimeter stops the robot",
    Math.max(...robotCorners(wall, ideal).map((p) => p[0])) <= FTC_FIELD.halfX + 1e-9
  );
  const leg = BIOBUZZ.obstacles[0];
  const atLeg = run(
    ideal,
    { x: 1, y: 0, turn: 0 },
    3,
    makeDriveState((leg.minX + leg.maxX) / 2 - 1, (leg.minY + leg.maxY) / 2, 0)
  );
  check("HIVE frame legs stop the robot", Math.abs(atLeg.x + ideal.length / 2 - leg.minX) < 0.005);
  check(
    "robot drives under the HIVE between the legs",
    run(ideal, { x: 1, y: 0, turn: 0 }, 2, makeDriveState(0, -1.2, 90)).y > 0.5
  );
  const sag = run(DEFAULT_DRIVETRAIN, { x: 1, y: 0, turn: 0 }, 1.5, makeDriveState(-1.5, -1.2, 0));
  check(
    "battery sag lowers top speed",
    sag.vx < 1.77 && sag.batteryVolts < 12.4,
    `${sag.vx.toFixed(3)} m/s at ${sag.batteryVolts.toFixed(2)} V`
  );

  const shot = defaultShotLabConfig();
  shot.error.preset = "zero";
  let hits = 0;
  let valid = 0;
  const cases: [number, number, number][] = [
    [0, 0, 0],
    [1.2, 0, 30],
    [0, -1.2, -90],
    [-1, 0.5, 45]
  ];
  cases.forEach(([vx, vy, h], i) => {
    const state = { ...makeDriveState(-0.32, -1.4, h), vx, vy };
    const options = { aim: "turret" as const, autoSpeed: true, applyError: false };
    const solution = computeSolution(shot, state, options);
    if (!solution.valid) return;
    valid++;
    if (fireShot(shot, state, options, solution, new Random(1), i, 0).hit) hits++;
  });
  check(
    "every solvable shot on the move goes in (perfect robot)",
    valid >= 3 && hits === valid,
    `${hits}/${valid} solvable of ${cases.length}`
  );
}

// ---------------------------------------------------------------- Calibration
{
  const piece = { ...DEFAULT_GAME_PIECE, mass: 0.018, radius: 0.0356 };
  const spin = { mode: "backspin" as const, rpm: 1500 };
  const shots = [
    [5.5, 45, 0.3],
    [6.5, 55, 0.3],
    [7.5, 40, 0.35]
  ].map(([v, h, z]) => ({ exitSpeed: v, hoodAngleDeg: h, launchHeight: z, landingDistance: 0 }));
  shots.forEach((s) => (s.landingDistance = simulateLandingDistance({ ...piece, dragCoefficient: 0.7 }, s, spin)!));
  const fit = fitDragCoefficient(piece, shots, spin)!;
  check(
    "drag fit recovers a known Cd",
    Math.abs(fit.dragCoefficient - 0.7) < 0.01,
    `Cd ${fit.dragCoefficient.toFixed(3)}`
  );
  check("drop test 1 m -> 0.36 m gives e = 0.6", Math.abs((fitRestitution(1, 0.36) ?? 0) - 0.6) < 1e-9);
}

// ---------------------------------------------------------------- Auto Check tuning
{
  const scenario = buildScenario(defaultShotLabConfig());
  scenario.shooter.exitSpeed *= 0.97; // start a little off center
  const tuned = centeringExitSpeed(scenario, false);
  const miss = measureMiss({ ...scenario, shooter: { ...scenario.shooter, exitSpeed: tuned } });
  check(
    "tuned exit speed centers the shot in the CELL mouth",
    isFinite(tuned) && Math.abs(miss.range) < 0.005,
    `${tuned.toFixed(3)} m/s, ${(miss.range * 100).toFixed(2)} cm from the middle`
  );
}

console.log("\n" + passes + " passed, " + failures + " failed");
if (failures > 0) process.exit(1);
