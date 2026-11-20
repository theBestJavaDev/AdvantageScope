// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import {
  AimMode,
  LiveShot,
  ShootingOptions,
  ShotSolution,
  computeSolution,
  fireShot,
  shotFinalPosition,
  shotPositionAt
} from "../../shared/physics/DriveShooting";
import {
  DEFAULT_DRIVETRAIN,
  DriveInput,
  DriveState,
  DrivetrainConfig,
  advanceDrive,
  makeDriveState
} from "../../shared/physics/DriveSim";
import {
  GamePreset,
  getGamePreset,
  mirrorCell,
  simToLogHeading,
  simToLogTranslation
} from "../../shared/physics/GamePresets";
import { FTC_FIELD } from "../../shared/physics/PhysicsEngine";
import { ShotLabConfig } from "../../shared/physics/ShotLab";
import { Random } from "../../shared/physics/vector";
import { DriveSimRendererCommand, DriveSimView } from "../../shared/renderers/DriveSimRenderer";
import TabType from "../../shared/TabType";
import { createUUID } from "../../shared/util";
import { FormSection, SimForm, getSharedShotLabConfig } from "./SimForm";
import TabController from "./TabController";

const DEG = Math.PI / 180;
export const DRIVE_SIM_LOG_PREFIX = "/SolversSim/Drive";

export interface DriveSimConfig {
  drivetrain: DrivetrainConfig;
  fieldCentric: boolean;
  /** driver = your alliance wall at the bottom (follows the Shot Lab alliance). */
  view: DriveSimView | "driver";
  /** Speed multiplier while holding Shift / LB. */
  slowFactor: number;
  shooting: ShootingOptions & { shotsPerSecond: number };
  /** alliance = the season preset's legal start for the Shot Lab alliance; custom = `start`. */
  startMode: "alliance" | "custom";
  start: { x: number; y: number; headingDeg: number };
}

function defaultConfig(): DriveSimConfig {
  return {
    drivetrain: { ...DEFAULT_DRIVETRAIN },
    fieldCentric: true,
    view: "driver",
    slowFactor: 0.4,
    shooting: { aim: "turret", autoSpeed: true, applyError: true, shotsPerSecond: 3 },
    startMode: "alliance",
    start: { x: -1.2, y: -1.2, headingDeg: 0 }
  };
}

function mergeConfig(saved: unknown): DriveSimConfig {
  const base = defaultConfig();
  if (typeof saved !== "object" || saved === null) return base;
  const s = saved as Partial<Record<keyof DriveSimConfig, any>>;
  return {
    drivetrain: { ...base.drivetrain, ...s.drivetrain },
    fieldCentric: typeof s.fieldCentric === "boolean" ? s.fieldCentric : base.fieldCentric,
    view: s.view === "x-right" || s.view === "x-up" ? s.view : "driver",
    slowFactor: typeof s.slowFactor === "number" ? s.slowFactor : base.slowFactor,
    shooting: { ...base.shooting, ...s.shooting },
    startMode: s.startMode === "custom" ? "custom" : "alliance",
    start: { ...base.start, ...s.start }
  };
}

const SECTIONS: FormSection[] = [
  {
    title: "Drivetrain",
    fields: [
      {
        kind: "select",
        path: "drivetrain.type",
        label: "Type",
        options: [
          { value: "mecanum", label: "Mecanum" },
          { value: "swerve", label: "Swerve" },
          { value: "tank", label: "Tank / differential" }
        ]
      },
      {
        kind: "number",
        path: "drivetrain.maxSpeed",
        label: "Top speed",
        unit: "m/s",
        step: 0.1,
        min: 0.1,
        max: 6,
        hint: "Wheel surface top speed at nominal voltage"
      },
      {
        kind: "number",
        path: "drivetrain.maxAccel",
        label: "Acceleration",
        unit: "m/s²",
        step: 0.5,
        min: 0.5,
        max: 20,
        hint: "Traction limit; above this the wheels slip"
      },
      { kind: "number", path: "drivetrain.maxTurnRateDeg", label: "Turn rate", unit: "°/s", step: 10, min: 10 },
      { kind: "number", path: "drivetrain.maxTurnAccelDeg", label: "Turn accel.", unit: "°/s²", step: 50, min: 50 },
      {
        kind: "number",
        path: "drivetrain.strafeEfficiency",
        label: "Strafe efficiency",
        unit: "×",
        step: 0.05,
        min: 0.3,
        max: 1,
        hint: "Mecanum only: strafe speed as a fraction of forward speed"
      },
      {
        kind: "number",
        path: "drivetrain.batteryVolts",
        label: "Battery (rested)",
        unit: "V",
        step: 0.1,
        min: 9,
        max: 14,
        hint: "Charged-pack voltage at the start; Reset robot puts a fresh pack in"
      },
      {
        kind: "number",
        path: "drivetrain.batterySagVolts",
        label: "Sag at full load",
        unit: "V",
        step: 0.1,
        min: 0,
        max: 4,
        hint: "How far the voltage dips when you floor it (0 = ideal battery)"
      },
      {
        kind: "number",
        path: "drivetrain.batteryDrainVoltsPerMin",
        label: "Drain",
        unit: "V/min",
        step: 0.05,
        min: 0,
        max: 3,
        hint: "Resting voltage lost per minute of hard driving"
      },
      { kind: "number", path: "drivetrain.length", label: "Robot length", unit: "m", step: 0.01, min: 0.15, max: 0.6 },
      { kind: "number", path: "drivetrain.width", label: "Robot width", unit: "m", step: 0.01, min: 0.15, max: 0.6 }
    ]
  },
  {
    title: "Driving",
    fields: [
      { kind: "checkbox", path: "fieldCentric", label: "Field-centric (F)" },
      {
        kind: "select",
        path: "view",
        label: "View",
        options: [
          { value: "driver", label: "Driver station (your wall at bottom)" },
          { value: "x-up", label: "+X up the screen" },
          { value: "x-right", label: "+X to the right" }
        ]
      },
      { kind: "number", path: "slowFactor", label: "Slow mode speed", unit: "×", step: 0.05, min: 0.1, max: 1 }
    ]
  },
  {
    title: "Shooting",
    fields: [
      {
        kind: "select",
        path: "shooting.aim",
        label: "Aim",
        options: [
          { value: "turret", label: "Turret (auto-aims)" },
          { value: "fixed", label: "Fixed (aim robot)" }
        ]
      },
      {
        kind: "checkbox",
        path: "shooting.autoSpeed",
        label: "Auto exit speed (solves on the move)",
        hint: "Off = always use the exit speed from the Shot Lab tab"
      },
      {
        kind: "checkbox",
        path: "shooting.applyError",
        label: "Apply Shot Lab error sources",
        hint: "Uses the error sources enabled in the Shot Lab tab"
      },
      { kind: "number", path: "shooting.shotsPerSecond", label: "Fire rate", unit: "/s", step: 0.5, min: 0.5, max: 10 }
    ]
  },
  {
    title: "Start position",
    collapsed: true,
    fields: [
      {
        kind: "select",
        path: "startMode",
        label: "Start",
        options: [
          { value: "alliance", label: "Alliance start (game preset)" },
          { value: "custom", label: "Custom" }
        ],
        hint: "Alliance start uses the Shot Lab alliance and the season's legal starting position"
      },
      { kind: "number", path: "start.x", label: "X", unit: "m", step: 0.05, min: -1.6, max: 1.6 },
      { kind: "number", path: "start.y", label: "Y", unit: "m", step: 0.05, min: -1.6, max: 1.6 },
      { kind: "number", path: "start.headingDeg", label: "Heading", unit: "°", step: 5 }
    ]
  }
];

const HANDLED_KEYS = new Set([
  "KeyW",
  "KeyA",
  "KeyS",
  "KeyD",
  "KeyQ",
  "KeyE",
  "KeyF",
  "KeyR",
  "KeyC",
  "Space",
  "ShiftLeft",
  "ShiftRight",
  "ArrowUp",
  "ArrowDown",
  "ArrowLeft",
  "ArrowRight"
]);

const DEADBAND = 0.08;

function deadband(v: number): number {
  if (Math.abs(v) < DEADBAND) return 0;
  // Rescale so output starts at 0 just past the deadband, then square for fine control
  const s = (Math.abs(v) - DEADBAND) / (1 - DEADBAND);
  return Math.sign(v) * s * s;
}

export default class DriveSimController implements TabController {
  UUID = createUUID();

  private CANVAS: HTMLCanvasElement;
  private form: SimForm;
  private recordButton: HTMLButtonElement;

  private config: DriveSimConfig = defaultConfig();
  private state: DriveState;
  private keys: Set<string> = new Set();
  private lastFrameMs: number | null = null;

  private shotConfig: ShotLabConfig = getSharedShotLabConfig();
  private rng = new Random(Date.now() & 0xffffffff);
  private shots: (LiveShot & { revealed: boolean })[] = [];
  private rests: { x: number; y: number; hit: boolean }[] = [];
  private feed: { text: string; hit: boolean; time: number }[] = [];
  /** Shots whose result is known (scored in the HUD). */
  private shotCount = 0;
  private firedCount = 0;
  private hitCount = 0;
  private nextShotId = 0;
  private lastShotTime = -Infinity;
  private burstIndex = 0;
  private solution: ShotSolution = { valid: false, exitSpeed: 0, turretYawDeg: 0, distance: 0 };
  private lastSolveTime = -Infinity;

  private trail: [number, number][] = [];
  private lastTrailTime = -Infinity;

  private inputSource: "keyboard" | "gamepad" | "none" = "none";
  private gamepadConnected = false;
  private gamepadButtons: boolean[] = [];

  private recording = false;
  private live = false;
  private liveButton: HTMLButtonElement;
  private recordStart = 0;
  private lastRecordTime = -Infinity;
  private lastSidebarRefresh = 0;

  constructor(root: HTMLElement) {
    this.CANVAS = root.getElementsByClassName("drivesim-canvas")[0] as HTMLCanvasElement;
    const container = root.getElementsByClassName("drivesim-form")[0] as HTMLElement;
    this.form = new SimForm(
      container,
      SECTIONS,
      () => this.config,
      (spec) => {
        if (spec.path.startsWith("start.")) this.config.startMode = "custom";
        if (spec.path.startsWith("start")) {
          this.form.sync();
          this.resetRobot();
        }
        if (spec.path.startsWith("drivetrain.")) this.state = { ...this.state }; // takes effect next step
      }
    );
    this.form.addAction("Drive", "Focus the field so the keyboard drives the robot", true, () => this.CANVAS.focus());
    this.recordButton = this.form.addAction(
      "● Record",
      "Write the robot pose, speeds, game pieces and shots to the log under " + DRIVE_SIM_LOG_PREFIX,
      false,
      () => this.toggleRecording()
    );
    this.form.addAction("Reset robot", "Move the robot back to the start position (R)", false, () => {
      this.resetRobot();
      this.CANVAS.focus();
    });
    this.form.addAction("Clear shots", "Clear the score and the pieces on the field (C)", false, () => {
      this.clearShots();
      this.CANVAS.focus();
    });
    this.liveButton = this.form.addAction(
      "▶ Live 3D view",
      "Record and stream the drive to a 3D Field in its own window, live, on the season field",
      true,
      () => this.toggleLive()
    );
    this.liveButton.classList.add("wide");
    const note = document.createElement("div");
    note.innerText = "Shooter, game piece, target and error settings come from the Shot Lab tab.";
    this.form.addNote(note);
    this.form.sync();

    const start = this.startPose();
    this.state = makeDriveState(start.x, start.y, start.headingDeg);

    // Keyboard only while the field has focus, so typing elsewhere in the app never drives the robot
    this.CANVAS.addEventListener("keydown", (event) => {
      if (!HANDLED_KEYS.has(event.code)) return;
      event.preventDefault();
      // Keep driving keys away from the app's global shortcuts (Space = play/pause, arrows = step)
      event.stopPropagation();
      if (event.repeat) return;
      this.keys.add(event.code);
      if (event.code === "KeyF") this.toggleFieldCentric();
      if (event.code === "KeyR") this.resetRobot();
      if (event.code === "KeyC") this.clearShots();
    });
    this.CANVAS.addEventListener("keyup", (event) => {
      if (HANDLED_KEYS.has(event.code)) event.stopPropagation();
      this.keys.delete(event.code);
    });
    this.CANVAS.addEventListener("blur", () => this.keys.clear());
    this.CANVAS.addEventListener("mousedown", () => this.CANVAS.focus());
  }

  // -------------------------------------------------------------------------------------------
  // Actions
  // -------------------------------------------------------------------------------------------

  private toggleFieldCentric() {
    this.config.fieldCentric = !this.config.fieldCentric;
    this.form.sync();
  }

  /** Where Reset puts the robot: the game preset's alliance start, or the custom start. */
  private startPose(): { x: number; y: number; headingDeg: number } {
    if (this.config.startMode === "alliance") {
      const shot = getSharedShotLabConfig();
      const preset = getGamePreset(shot.game.preset);
      if (preset !== null) return preset.starts[shot.game.alliance];
    }
    return this.config.start;
  }

  private resetRobot() {
    const start = this.startPose();
    this.state = {
      ...makeDriveState(start.x, start.y, start.headingDeg),
      time: this.state?.time ?? 0
    };
    this.trail = [];
  }

  private clearShots() {
    this.shots = [];
    this.rests = [];
    this.feed = [];
    this.shotCount = 0;
    this.firedCount = 0;
    this.hitCount = 0;
  }

  /**
   * Live 3D: records, makes the sim clock drive the timeline (like a live robot connection), and
   * opens a 3D Field in its own window showing the robot + game pieces on the season field.
   */
  private toggleLive() {
    if (this.live) {
      this.stopLive();
      this.CANVAS.focus();
      return;
    }
    if (window.selection.getCurrentLiveTime() !== null) {
      window.sendMainMessage("error", {
        title: "Already connected",
        content: "Disconnect from the robot / live source before starting the live 3D view."
      });
      return;
    }
    if (!this.recording) this.toggleRecording();
    this.live = true;
    window.selection.setLiveConnected(() => Math.max(0, this.state.time - this.recordStart));
    this.liveButton.innerText = "■ Stop live 3D";
    this.liveButton.classList.add("recording");
    window.tabs.addTabInSatellite(TabType.Field3d, this.live3dState());
    this.CANVAS.focus();
  }

  private stopLive() {
    if (!this.live) return;
    this.live = false;
    window.selection.setLiveDisconnected();
    this.liveButton.innerText = "▶ Live 3D view";
    this.liveButton.classList.remove("recording");
  }

  /** 3D Field tab state: FTC robot on the drive pose + the season's game piece model on the balls. */
  private live3dState(): unknown {
    const robots = window.assets?.robots ?? [];
    const model = (robots.find((r) => r.isFTC) ?? robots[0])?.name ?? "";
    const fields = (window.assets?.field3ds ?? []).filter((f) => f.isFTC);
    // Newest season field (same rule the 3D tab uses for its default)
    const year = (id: string) => Number(/(20\d{2})/.exec(id)?.[1] ?? -1);
    const field = fields.reduce<(typeof fields)[number] | undefined>(
      (best, f) => (best === undefined || year(f.id) > year(best.id) ? f : best),
      undefined
    );
    const pieceNames = field?.gamePieces.map((g) => g.name) ?? [];
    const wanted = this.shotConfig.game.piece.toLowerCase();
    const variant = pieceNames.find((n) => n.toLowerCase().includes(wanted)) ?? pieceNames[0] ?? "";
    return {
      sources: [
        {
          type: "robot",
          logKey: DRIVE_SIM_LOG_PREFIX + "/Pose",
          logType: "Pose2d",
          visible: true,
          options: { model }
        },
        {
          type: "gamePiece",
          logKey: DRIVE_SIM_LOG_PREFIX + "/GamePieces",
          logType: "Translation3d[]",
          visible: true,
          options: { variant }
        }
      ],
      ...(field !== undefined ? { game: field.id } : {})
    };
  }

  private toggleRecording() {
    this.recording = !this.recording;
    if (!this.recording) this.stopLive();
    if (this.recording) {
      window.log.deleteFieldsWithPrefix(DRIVE_SIM_LOG_PREFIX);
      this.recordStart = this.state.time;
      this.lastRecordTime = -Infinity;
      this.recordButton.innerText = "■ Stop recording";
      this.recordButton.classList.add("recording");
    } else {
      this.recordButton.innerText = "● Record";
      this.recordButton.classList.remove("recording");
    }
    window.sidebar.refresh();
    window.tabs.refresh();
    this.CANVAS.focus();
  }

  // -------------------------------------------------------------------------------------------
  // Input
  // -------------------------------------------------------------------------------------------

  /** Stick input in screen terms (up/left) + turn, from the keyboard or the first active gamepad. */
  private readInput(): { up: number; left: number; turn: number; shoot: boolean; slow: boolean } {
    const k = (code: string) => (this.keys.has(code) ? 1 : 0);
    const keyboard = {
      up: k("KeyW") + k("ArrowUp") - k("KeyS") - k("ArrowDown"),
      left: k("KeyA") + k("ArrowLeft") - k("KeyD") - k("ArrowRight"),
      turn: k("KeyQ") - k("KeyE"),
      shoot: this.keys.has("Space"),
      slow: this.keys.has("ShiftLeft") || this.keys.has("ShiftRight")
    };
    const keyboardActive = keyboard.up !== 0 || keyboard.left !== 0 || keyboard.turn !== 0 || keyboard.shoot;

    let pad: Gamepad | null = null;
    try {
      pad = Array.from(navigator.getGamepads?.() ?? []).find((g) => g !== null && g.connected) ?? null;
    } catch {
      pad = null;
    }
    this.gamepadConnected = pad !== null;
    if (pad !== null) {
      const axis = (i: number) => (pad!.axes.length > i ? pad!.axes[i] : 0);
      const pressed = (i: number) => pad!.buttons.length > i && pad!.buttons[i].pressed;
      const value = (i: number) => (pad!.buttons.length > i ? pad!.buttons[i].value : 0);
      const gp = {
        up: -deadband(axis(1)),
        left: -deadband(axis(0)),
        turn: -deadband(axis(2)),
        shoot: value(7) > 0.5 || pressed(0),
        slow: pressed(4)
      };
      // Edge-triggered buttons: Y toggles field-centric, Back resets
      const buttons = pad.buttons.map((b) => b.pressed);
      if (buttons[3] && !this.gamepadButtons[3]) this.toggleFieldCentric();
      if (buttons[8] && !this.gamepadButtons[8]) this.resetRobot();
      this.gamepadButtons = buttons;
      const gamepadActive = gp.up !== 0 || gp.left !== 0 || gp.turn !== 0 || gp.shoot;
      if (gamepadActive && !keyboardActive) {
        this.inputSource = "gamepad";
        return gp;
      }
    }
    if (keyboardActive || this.inputSource !== "gamepad") {
      this.inputSource =
        document.activeElement === this.CANVAS ? "keyboard" : this.gamepadConnected ? "gamepad" : "none";
    }
    return keyboard;
  }

  /** Driver-station view puts your own alliance wall at the bottom of the screen. */
  private screenView(): DriveSimView {
    if (this.config.view !== "driver") return this.config.view;
    return this.shotConfig.game.alliance === "blue" ? "x-down" : "x-up";
  }

  /** Converts screen-relative sticks to the drive's (x, y) input for the current mode + view. */
  private toDriveInput(up: number, left: number, turn: number, slow: boolean): DriveInput {
    const scale = slow ? this.config.slowFactor : 1;
    let x = up;
    let y = left;
    if (this.config.fieldCentric) {
      const view = this.screenView();
      if (view === "x-right") {
        // Screen up = +Y, screen left = -X
        x = -left;
        y = up;
      } else if (view === "x-down") {
        // Screen up = -X, screen left = -Y
        x = -up;
        y = -left;
      }
    }
    return { x: x * scale, y: y * scale, turn: turn * scale };
  }

  // -------------------------------------------------------------------------------------------
  // Simulation
  // -------------------------------------------------------------------------------------------

  private step() {
    const now = performance.now();
    const dt = this.lastFrameMs === null ? 0 : Math.min(0.05, Math.max(0, (now - this.lastFrameMs) / 1000));
    this.lastFrameMs = now;
    this.shotConfig = getSharedShotLabConfig();

    const input = this.readInput();
    if (dt > 0) {
      this.state = advanceDrive(
        this.state,
        this.toDriveInput(input.up, input.left, input.turn, input.slow),
        this.config.drivetrain,
        this.config.fieldCentric,
        dt,
        this.preset()?.obstacles ?? []
      );
    }
    const t = this.state.time;

    // Shot solution for the HUD/aim line (cheap enough at ~8 Hz)
    if (t - this.lastSolveTime > 0.12 || dt === 0) {
      this.solution = computeSolution(this.shotConfig, this.state, this.config.shooting);
      this.lastSolveTime = t;
    }

    // Fire
    if (input.shoot && t - this.lastShotTime >= 1 / Math.max(0.1, this.config.shooting.shotsPerSecond)) {
      // Back-to-back shots drain the flywheel (shotIndex feeds the flywheel recovery error source)
      this.burstIndex = t - this.lastShotTime < 1.0 ? this.burstIndex + 1 : 0;
      const solution = computeSolution(this.shotConfig, this.state, this.config.shooting);
      this.solution = solution;
      const shot = fireShot(
        this.shotConfig,
        this.state,
        this.config.shooting,
        solution,
        this.rng,
        this.nextShotId++,
        this.burstIndex
      );
      this.shots.push({ ...shot, revealed: false });
      this.firedCount++;
      this.lastShotTime = t;
      if (this.recording) this.recordShot(shot);
    }

    // Advance pieces in flight, reveal results when the piece gets there
    for (const shot of this.shots) {
      const age = t - shot.fireTime;
      const revealTime = shot.trajectory.target?.time ?? shot.trajectory.landing?.time ?? Infinity;
      if (!shot.revealed && (age >= revealTime || shotPositionAt(shot, age) === null)) {
        shot.revealed = true;
        this.shotCount++;
        if (shot.hit) this.hitCount++;
        this.feed.unshift({
          text:
            (shot.hit ? "Hit" : "Miss") +
            " · " +
            shot.exitSpeed.toFixed(2) +
            " m/s" +
            (shot.robotSpeed > 0.1 ? " · moving " + shot.robotSpeed.toFixed(1) + " m/s" : "") +
            (shot.bumped ? " · bumped" : ""),
          hit: shot.hit,
          time: t
        });
        this.feed = this.feed.slice(0, 5);
      }
    }
    const finished = this.shots.filter((s) => s.revealed && shotPositionAt(s, t - s.fireTime) === null);
    for (const s of finished) {
      const p = shotFinalPosition(s);
      if (Math.abs(p[0]) <= FTC_FIELD.halfX && Math.abs(p[1]) <= FTC_FIELD.halfY) {
        this.rests.push({ x: p[0], y: p[1], hit: s.hit });
      }
    }
    this.shots = this.shots.filter((s) => !finished.includes(s));
    this.rests = this.rests.slice(-60);

    // Trail
    if (t - this.lastTrailTime > 0.05) {
      this.trail.push([this.state.x, this.state.y]);
      this.trail = this.trail.slice(-80);
      this.lastTrailTime = t;
    }

    if (this.recording) this.record();
  }

  // -------------------------------------------------------------------------------------------
  // Recording into the log
  // -------------------------------------------------------------------------------------------

  private record() {
    const t = this.state.time - this.recordStart;
    if (t - this.lastRecordTime < 0.02) return;
    this.lastRecordTime = t;
    const P = DRIVE_SIM_LOG_PREFIX;
    const log = window.log;
    const s = this.state;
    // Field positions in the FTC log frame so they line up on the 2026-2027 field (see GamePresets)
    log.putPose(P + "/Pose", t, {
      translation: simToLogTranslation([s.x, s.y]),
      rotation: simToLogHeading(s.heading)
    });
    // Robot-relative speeds are what a real robot logs from its kinematics
    const c = Math.cos(s.heading);
    const sn = Math.sin(s.heading);
    log.putNumber(P + "/Velocity/Forward", t, s.vx * c + s.vy * sn);
    log.putNumber(P + "/Velocity/Left", t, -s.vx * sn + s.vy * c);
    log.putNumber(P + "/Velocity/TurnRateDegPerSec", t, s.omega / DEG);
    log.putNumber(P + "/Speed", t, Math.hypot(s.vx, s.vy));
    log.putNumber(P + "/WheelLoad", t, s.wheelLoad);
    log.putNumber(
      P + "/BatteryVolts",
      t,
      isFinite(s.batteryVolts) ? s.batteryVolts : this.config.drivetrain.batteryVolts
    );
    log.putNumber(
      P + "/BatteryRestVolts",
      t,
      isFinite(s.batteryRest) ? s.batteryRest : this.config.drivetrain.batteryVolts
    );
    log.putBoolean(P + "/WallContact", t, s.wallContact);
    log.putBoolean(P + "/ShotSolutionValid", t, this.solution.valid);
    log.putNumber(P + "/TargetDistance", t, this.solution.distance);
    log.putNumber(P + "/TurretYawDeg", t, this.solution.turretYawDeg);
    log.putNumber(P + "/Shots/Fired", t, this.firedCount);
    log.putNumber(P + "/Shots/Hits", t, this.hitCount);
    log.putTranslation3dArray(
      P + "/GamePieces",
      t,
      this.shots
        .map((shot) => shotPositionAt(shot, this.state.time - shot.fireTime))
        .filter((p): p is [number, number, number] => p !== null)
        .map((p) => simToLogTranslation(p))
    );
    const nowMs = Date.now();
    if (nowMs - this.lastSidebarRefresh > 1000) {
      this.lastSidebarRefresh = nowMs;
      window.sidebar.refresh();
    }
  }

  private recordShot(shot: LiveShot) {
    const t = this.state.time - this.recordStart;
    const P = DRIVE_SIM_LOG_PREFIX + "/Shots";
    window.log.putNumber(P + "/ExitSpeed", t, shot.exitSpeed);
    window.log.putNumber(P + "/RobotSpeedAtShot", t, shot.robotSpeed);
    window.log.putBoolean(P + "/Hit", t, shot.hit);
  }

  // -------------------------------------------------------------------------------------------
  // TabController
  // -------------------------------------------------------------------------------------------

  saveState(): unknown {
    return this.config;
  }

  restoreState(state: unknown): void {
    this.config = mergeConfig(state);
    this.form.sync();
    this.resetRobot();
  }

  refresh(): void {}

  newAssets(): void {}

  getActiveFields(): string[] {
    return [];
  }

  showTimeline(): boolean {
    return false;
  }

  private preset(): GamePreset | null {
    return getGamePreset(this.shotConfig.game.preset);
  }

  /** Field elements of the season preset, for drawing. */
  private gameLayer(): DriveSimRendererCommand["game"] {
    const preset = this.preset();
    if (preset === null) return null;
    const { alliance, upCell, piece } = this.shotConfig.game;
    return {
      name: preset.name + " " + preset.season,
      zones: preset.zones.map((z) => ({
        kind: z.kind,
        alliance: z.alliance,
        minX: z.minX,
        maxX: z.maxX,
        minY: z.minY,
        maxY: z.maxY
      })),
      frame: preset.frame,
      legs: preset.obstacles,
      hives: preset.hives.map((h) => ({ alliance: h.alliance, x: h.x, y: h.y })),
      cells: preset.cells.map((c) => ({
        alliance: c.alliance,
        x: c.x,
        y: c.y,
        sizeX: c.sizeX,
        sizeY: c.sizeY,
        // HIVEs tip independently; the other HIVE is shown in the rotationally symmetric state
        up: c.hiveCell === (c.alliance === alliance ? upCell : mirrorCell(upCell)),
        active: c.alliance === alliance && c.hiveCell === upCell,
        facing:
          c.mouth !== undefined
            ? ([c.mouth.normal[0], c.mouth.normal[1]].map(
                (v) => v / (Math.hypot(c.mouth!.normal[0], c.mouth!.normal[1]) || 1)
              ) as [number, number])
            : null
      })),
      flowers: preset.flowers.map((f) => ({ x: f.x, y: f.y, diameter: f.sizeX })),
      alliance,
      pieceColor:
        piece === "nectar"
          ? alliance === "red"
            ? "#e5484d"
            : "#3b82f6"
          : preset.pieces.find((p) => p.id === piece)?.color ?? "#f2c94c"
    };
  }

  getCommand(): DriveSimRendererCommand {
    this.step();
    const s = this.state;
    const d = this.config.drivetrain;
    const t = this.shotConfig.target;
    const radius = this.shotConfig.piece.diameterM / 2;
    return {
      view: this.screenView(),
      field: { halfX: FTC_FIELD.halfX, halfY: FTC_FIELD.halfY },
      robot: {
        x: s.x,
        y: s.y,
        heading: s.heading,
        length: d.length,
        width: d.width,
        vx: s.vx,
        vy: s.vy,
        drivetrain: d.type,
        turretYaw:
          (this.config.shooting.aim === "turret" ? this.solution.turretYawDeg : this.shotConfig.shooter.turretYawDeg) *
          DEG,
        shooterOffset: this.shotConfig.shooter.offsetX,
        wallContact: s.wallContact
      },
      trail: this.trail,
      balls: this.shots
        .map((shot) => shotPositionAt(shot, s.time - shot.fireTime))
        .filter((p): p is [number, number, number] => p !== null)
        .map((p) => ({ x: p[0], y: p[1], z: p[2], radius })),
      rests: this.rests,
      target: { x: t.x, y: t.y, sizeX: t.sizeX, sizeY: t.sizeY, z: t.z },
      game: this.gameLayer(),
      hud: {
        speed: Math.hypot(s.vx, s.vy),
        headingDeg: s.heading / DEG,
        fieldCentric: this.config.fieldCentric,
        slow: this.keys.has("ShiftLeft") || this.keys.has("ShiftRight"),
        input: this.inputSource,
        shots: this.shotCount,
        hits: this.hitCount,
        solution: { valid: this.solution.valid, exitSpeed: this.solution.exitSpeed, distance: this.solution.distance },
        aim: this.config.shooting.aim as AimMode,
        battery: isFinite(s.batteryVolts) ? s.batteryVolts : d.batteryVolts,
        wheelLoad: s.wheelLoad,
        recording: this.recording,
        recordTime: this.recording ? s.time - this.recordStart : 0,
        focused: document.activeElement === this.CANVAS,
        gamepadConnected: this.gamepadConnected
      },
      feed: this.feed.map((f) => ({ text: f.text, hit: f.hit, age: s.time - f.time }))
    };
  }
}
