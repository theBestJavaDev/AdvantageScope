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
  simulateNominal,
  solveShot
} from "../../src/shared/physics/ShotLab";
import {
  TYPICAL_ERROR_MODEL,
  ZERO_ERROR_MODEL,
  isolateErrorSources,
  sampleShot
} from "../../src/shared/physics/ShotErrorModel";
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

// ---------------------------------------------------------------- Bounce, friction, rotation
{
  const big = { ...FTC_FIELD, halfX: 50, halfY: 50 };
  const calm = { ...DEFAULT_ENVIRONMENT, airDensity: 0 };
  const piece = { ...DEFAULT_GAME_PIECE, dragCoefficient: 0, magnusScale: 0 };
  const engine = new PhysicsEngine(piece, calm, big);
  const energy = (v: number[], w: number[], z: number) => {
    const m = piece.mass;
    const I = piece.inertiaFactor * m * piece.radius * piece.radius;
    return (
      0.5 * m * (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) + 0.5 * I * (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) + m * 9.80665 * z
    );
  };

  // 1 m drop: first rebound height follows e^2 (restitution is defined at the drop-test speed)
  {
    const r = engine.simulate(makeBallState([0, 0, 1 + piece.radius], [0, 0, 0], [0, 0, 0]), {
      duration: 1.2,
      timeStep: 0.0005
    });
    const afterBounce = r.samples.filter((x) => x.time > (r.landing?.time ?? 0));
    const apex = Math.max(...afterBounce.slice(0, 60).map((x) => x.position[2] - piece.radius));
    const expected = piece.floorRestitution ** 2;
    check(
      "1 m drop rebounds to e^2 of the height",
      Math.abs(apex - expected) < 0.04,
      `${apex.toFixed(3)} vs ${expected.toFixed(3)} m`
    );
  }

  // Harder hits bounce proportionally less than soft ones
  {
    const rebound = (v: number) => {
      const r = engine.simulate(makeBallState([0, 0, piece.radius + 0.0005], [0, 0, -v], [0, 0, 0]), {
        duration: 0.5,
        timeStep: 0.0002
      });
      const c = r.contacts.find((x) => x.surface === "floor");
      const after = r.samples.find((x) => c !== undefined && x.time > c.time + 0.002);
      return after === undefined ? 0 : after.velocity[2] / v;
    };
    const soft = rebound(2);
    const hard = rebound(9);
    check(
      "hard impacts lose more energy than soft ones",
      soft > hard + 0.03,
      `e = ${soft.toFixed(3)} (2 m/s) vs ${hard.toFixed(3)} (9 m/s)`
    );
  }

  // Backspin ball landing moving forward checks up (loses forward speed); topspin kicks forward
  {
    const land = (spinY: number) => {
      const r = engine.simulate(makeBallState([0, 0, 0.5], [3, 0, -3], [0, spinY, 0]), {
        duration: 0.5,
        timeStep: 0.0005
      });
      const c = r.contacts.find((x) => x.surface === "floor");
      const after = r.samples.find((x) => c !== undefined && x.time > c.time + 0.01);
      return after === undefined ? NaN : after.velocity[0];
    };
    // +Y spin axis: positive = topspin for +X travel (top surface moves forward)
    const top = land(150);
    const none = land(0);
    const back = land(-150);
    check(
      "backspin checks, topspin kicks (post-bounce forward speed)",
      back < none && none < top,
      `${back.toFixed(2)} / ${none.toFixed(2)} / ${top.toFixed(2)} m/s`
    );
  }

  // A sliding ball ends up rolling without slipping, and a rolling ball decays smoothly to rest
  {
    const r = engine.simulate(makeBallState([0, 0, piece.radius], [3, 0, 0], [0, 0, 0]), {
      duration: 6,
      timeStep: 0.001
    });
    const mid = r.samples[Math.floor(r.samples.length * 0.1)];
    const rolling =
      Math.abs(mid.velocity[0] - mid.angularVelocity[1] * piece.radius) < 0.15 * Math.abs(mid.velocity[0]) + 0.02;
    const last = r.finalState;
    check(
      "sliding ball turns into rolling (v = w r)",
      rolling,
      `v ${mid.velocity[0].toFixed(2)} vs w r ${(mid.angularVelocity[1] * piece.radius).toFixed(2)}`
    );
    check(
      "rolling ball comes to rest",
      Math.hypot(last.velocity[0], last.velocity[1]) < 0.05,
      `${Math.hypot(last.velocity[0], last.velocity[1]).toFixed(3)} m/s at the end`
    );
  }

  // A ball spinning flat on the floor slows down from pivot friction
  {
    const r = engine.simulate(makeBallState([0, 0, piece.radius], [0, 0, 0], [0, 0, 30]), {
      duration: 2,
      timeStep: 0.001,
      stopWhenAtRest: false
    });
    const w = r.finalState.angularVelocity[2];
    check("flat spin on the floor slows down", w < 29 && w >= 0, `${w.toFixed(1)} rad/s after 2 s`);
  }

  // Contacts never add energy
  {
    let worst = 0;
    for (const [vx, vz, wy] of [
      [4, -5, 80],
      [-2, -8, -120],
      [6, 3, 0],
      [0, -9, 200]
    ]) {
      const r = engine.simulate(makeBallState([0, 0, 0.4], [vx, 0, vz], [0, wy, 0]), {
        duration: 3,
        timeStep: 0.0005,
        stopWhenAtRest: false
      });
      const e0 = energy([vx, 0, vz], [0, wy, 0], 0.4);
      const s = r.finalState;
      worst = Math.max(worst, (energy(s.velocity, s.angularVelocity, s.position[2]) - e0) / e0);
    }
    check("bounces never create energy", worst < 0.02, `worst change ${(worst * 100).toFixed(2)} %`);
  }
}

// ---------------------------------------------------------------- Natural-looking error sources
{
  const scenario = buildScenario(defaultShotLabConfig());
  const draw = (keys: Parameters<typeof isolateErrorSources>[1], n = 400) => {
    const model = isolateErrorSources(TYPICAL_ERROR_MODEL, keys);
    const rng = new Random(7);
    return Array.from({ length: n }, () => sampleShot(scenario, model, rng));
  };
  const mean = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / xs.length;

  const slip = draw(["wheelSlip"]);
  check(
    "wheel slip only ever loses speed and spin",
    slip.every((x) => x.shooter.exitSpeed <= scenario.shooter.exitSpeed + 1e-9 && x.shooter.spinRpm <= scenario.shooter.spinRpm + 1e-9) &&
      mean(slip.map((x) => x.shooter.exitSpeed)) < scenario.shooter.exitSpeed * 0.995,
    `mean exit ${mean(slip.map((x) => x.shooter.exitSpeed)).toFixed(3)} vs ${scenario.shooter.exitSpeed.toFixed(3)} m/s`
  );

  const still = draw(["launchVibration"]);
  const moving = (() => {
    const model = isolateErrorSources(TYPICAL_ERROR_MODEL, ["launchVibration"]);
    const rng = new Random(7);
    const fast = { ...scenario, robot: { ...scenario.robot, vx: 2, vy: 0 } };
    return Array.from({ length: 400 }, () => sampleShot(fast, model, rng));
  })();
  const spread = (xs: { shooter: { hoodAngleDeg: number } }[]) => {
    const m = mean(xs.map((x) => x.shooter.hoodAngleDeg));
    return Math.sqrt(mean(xs.map((x) => (x.shooter.hoodAngleDeg - m) ** 2)));
  };
  check("launch vibration grows with robot speed", spread(moving) > spread(still) * 2, `${spread(still).toFixed(2)}° standing vs ${spread(moving).toFixed(2)}° at 2 m/s`);

  const seat = draw(["seating"]);
  check("ball seating moves the release point sideways", seat.some((x) => Math.abs(x.shooter.offset[1] - scenario.shooter.offset[1]) > 0.002));
  check("a perfect robot draws no variation", draw([]).every((x) => x.shooter.exitSpeed === scenario.shooter.exitSpeed && x.shooter.offset[1] === scenario.shooter.offset[1]));

  // Per-bounce randomness: identical drops land differently
  const bounceX = (noise: boolean, seed: number) => {
    const engine = new PhysicsEngine({ ...DEFAULT_GAME_PIECE, dragCoefficient: 0, magnusScale: 0 }, { ...DEFAULT_ENVIRONMENT, airDensity: 0 }, { ...FTC_FIELD, halfX: 50, halfY: 50 });
    if (noise) engine.contactNoise = { rng: new Random(seed), restitutionStdDev: 0.04, frictionStdDev: 0.08, bounceAngleStdDevDeg: 3 };
    const r = engine.simulate(makeBallState([0, 0, 0.6], [3, 0, -1], [0, 0, 0]), { duration: 1.5, timeStep: 0.001, stopAfterLanding: false });
    return r.finalState.position;
  };
  const plain = [1, 2, 3].map((i) => bounceX(false, i));
  const noisy = [1, 2, 3, 4, 5, 6].map((i) => bounceX(true, i));
  check("without noise, identical drops land identically", plain.every((p) => Math.abs(p[0] - plain[0][0]) < 1e-12 && Math.abs(p[1] - plain[0][1]) < 1e-12));
  check("with noise, identical drops scatter", new Set(noisy.map((p) => p[0].toFixed(3))).size > 3 && noisy.some((p) => Math.abs(p[1]) > 0.005), `x spread ${(Math.max(...noisy.map((p) => p[0])) - Math.min(...noisy.map((p) => p[0]))).toFixed(3)} m`);
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

// ---------------------------------------------------------------- HIVE is solid
{
  const engine = new PhysicsEngine({ ...DEFAULT_GAME_PIECE, radius: 1.4 * 0.0254 }, DEFAULT_ENVIRONMENT, FTC_FIELD);
  const mouth = cellMouth("red", "rear");
  const target = { name: "cell", min: mouth.center, max: mouth.center, requireDescending: false, mouth };
  const opts = { duration: 2, timeStep: 0.0005, stopWhenAtRest: false };
  check(
    "every CELL mouth comes with the HIVE's solid parts",
    (mouth.colliders?.length ?? 0) >= 10,
    `${mouth.colliders?.length ?? 0} parts`
  );

  // Thrown flat at the side of the red HIVE's bar (from the field center toward -x): bounces back
  {
    const start: [number, number, number] = [0, 0, 43.95 * 0.0254 + 0.08];
    const r = engine.simulate(makeBallState(start, [-4, 0, 1.5], [0, 0, 0]), opts, target);
    const hiveHit = r.contacts.find((c) => c.surface === "hive");
    const minX = Math.min(...r.samples.map((s) => s.position[0]));
    check(
      "a ball thrown at the side of a HIVE bounces off it",
      hiveHit !== undefined && minX > mouth.center[0] && !r.target!.hit,
      hiveHit ? `hit at x ${hiveHit.position[0].toFixed(2)} m, never past x ${minX.toFixed(2)} m` : "went through"
    );
  }

  // Dropped onto the HIVE's roof: first contact is on top of the bar, not inside it
  {
    const r = engine.simulate(makeBallState([mouth.center[0], 0, 2.2], [0, 0, 0], [0, 0, 0]), opts, target);
    const firstHive = r.contacts.find((c) => c.surface === "hive");
    check(
      "a ball dropped on a HIVE lands on its roof, not through it",
      firstHive !== undefined && firstHive.position[2] > 55 * 0.0254,
      firstHive ? `first contact ${(firstHive.position[2] / 0.0254).toFixed(1)} in up` : "fell through"
    );
  }

  // A shot centered in the mouth still goes in with the HIVE solid
  {
    const scenario = buildScenario(defaultShotLabConfig());
    scenario.shooter.exitSpeed = centeringExitSpeed(scenario, false);
    const r = simulateNominal(scenario);
    const hiveHits = r.contacts.filter((c) => c.surface === "hive" && c.time < (r.target?.time ?? Infinity)).length;
    check(
      "a centered shot still scores with the HIVE solid",
      r.target?.hit === true && hiveHits === 0,
      `${r.target?.hit ? "IN" : "MISS"}, ${hiveHits} HIVE touches before the mouth`
    );
  }
}

console.log("\n" + passes + " passed, " + failures + " failed");
if (failures > 0) process.exit(1);
