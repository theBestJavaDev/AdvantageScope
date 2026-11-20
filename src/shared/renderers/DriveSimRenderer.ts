// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { themeColor } from "../themeColors";
import TabRenderer from "./TabRenderer";

/** Screen orientation: which field axis points up the screen. */
export type DriveSimView = "x-up" | "x-down" | "x-right";

export type DriveSimRendererCommand = {
  view: DriveSimView;
  field: { halfX: number; halfY: number };
  robot: {
    x: number;
    y: number;
    heading: number;
    length: number;
    width: number;
    vx: number;
    vy: number;
    drivetrain: "mecanum" | "swerve" | "tank";
    /** Shooter direction relative to the robot (rad). */
    turretYaw: number;
    /** Shooter exit point ahead of robot center (m). */
    shooterOffset: number;
    wallContact: boolean;
  };
  trail: [number, number][];
  balls: { x: number; y: number; z: number; radius: number }[];
  rests: { x: number; y: number; hit: boolean }[];
  target: { x: number; y: number; sizeX: number; sizeY: number; z: number };
  /** Season field elements (null = generic field). */
  game: {
    name: string;
    zones: {
      kind: "loading" | "garden" | "alliance-area";
      alliance: "red" | "blue";
      minX: number;
      maxX: number;
      minY: number;
      maxY: number;
    }[];
    frame: { minX: number; maxX: number; minY: number; maxY: number };
    legs: { minX: number; maxX: number; minY: number; maxY: number }[];
    hives: { alliance: "red" | "blue"; x: number; y: number }[];
    cells: {
      alliance: "red" | "blue";
      x: number;
      y: number;
      sizeX: number;
      sizeY: number;
      up: boolean;
      active: boolean;
      /** Horizontal direction the CELL's mouth faces (only scoreable from that side). */
      facing: [number, number] | null;
    }[];
    flowers: { x: number; y: number; diameter: number }[];
    alliance: "red" | "blue";
    pieceColor: string;
  } | null;
  hud: {
    speed: number;
    headingDeg: number;
    fieldCentric: boolean;
    slow: boolean;
    input: "keyboard" | "gamepad" | "none";
    shots: number;
    hits: number;
    solution: { valid: boolean; exitSpeed: number; distance: number };
    aim: "turret" | "fixed";
    battery: number;
    wheelLoad: number;
    recording: boolean;
    recordTime: number;
    focused: boolean;
    gamepadConnected: boolean;
  };
  feed: { text: string; hit: boolean; age: number }[];
};

type Palette = ReturnType<typeof palette>;
type HintItem = { k: string; label: string; kw: number; total: number };
type HudLayout = {
  rows: HintItem[][];
  /** Score panel moves under the chips when the canvas is too narrow for both top panels. */
  stacked: boolean;
  /** Space reserved above / below the field for the HUD (px). */
  top: number;
  bottom: number;
  leftWidth: number;
  rightWidth: number;
};

function palette(dark: boolean) {
  // Seattle Solvers colors (seattlesolvers.com): gold robot, warm neutrals, gold accents.
  // Red / blue stay for alliances and the hit / miss colors stay green / red.
  return dark
    ? {
        bg: "#14130c",
        tileA: "#3a3830",
        tileB: "#34322b",
        seam: "rgba(0,0,0,0.35)",
        wall: "#5c584a",
        wallEdge: "#8a8470",
        text: "#fcf9de",
        textDim: "rgba(252,249,222,0.62)",
        panel: "rgba(11,10,6,0.86)",
        panelBorder: "rgba(252,249,222,0.14)",
        robot: "#f2ce00",
        robotEdge: "#0b0a06",
        wheel: "#0b0a06",
        trail: "242,206,0",
        target: "#fcf9de",
        targetFill: "rgba(252,249,222,0.12)",
        hit: "#3ddc84",
        miss: "#ff5c5c",
        ball: "#ffe75c",
        shadow: "rgba(0,0,0,0.45)",
        aimGood: "rgba(61,220,132,0.75)",
        aimBad: "rgba(255,92,92,0.75)",
        keycap: "rgba(252,249,222,0.1)",
        accent: "#f2ce00",
        rec: "#ff3b30"
      }
    : {
        bg: "#fcf9de",
        tileA: "#cbc7b5",
        tileB: "#c3bfac",
        seam: "rgba(11,10,6,0.16)",
        wall: "#8a8470",
        wallEdge: "#625d47",
        text: "#0b0a06",
        textDim: "rgba(11,10,6,0.6)",
        panel: "rgba(252,249,222,0.92)",
        panelBorder: "rgba(11,10,6,0.16)",
        robot: "#f2ce00",
        robotEdge: "#0b0a06",
        wheel: "#1d1b10",
        trail: "185,157,0",
        target: "#3a3726",
        targetFill: "rgba(11,10,6,0.08)",
        hit: "#1a9e55",
        miss: "#d63a3a",
        ball: "#c9a800",
        shadow: "rgba(0,0,0,0.3)",
        aimGood: "rgba(26,158,85,0.8)",
        aimBad: "rgba(214,58,58,0.8)",
        keycap: "rgba(11,10,6,0.07)",
        accent: "#b99d00",
        rec: "#e0281e"
      };
}

export default class DriveSimRenderer implements TabRenderer {
  private CANVAS: HTMLCanvasElement;
  private FOCUS_HINT: HTMLElement | null;

  constructor(root: HTMLElement) {
    this.CANVAS = root.getElementsByClassName("drivesim-canvas")[0] as HTMLCanvasElement;
    this.FOCUS_HINT = root.getElementsByClassName("drivesim-focus-hint")[0] as HTMLElement | null;
  }

  saveState(): unknown {
    return null;
  }

  restoreState(): void {}

  getAspectRatio(): number | null {
    return null;
  }

  render(command: DriveSimRendererCommand): void {
    if (command === null || command === undefined) return;
    if (this.FOCUS_HINT) {
      this.FOCUS_HINT.hidden = command.hud.focused || command.hud.input === "gamepad";
    }

    const canvas = this.CANVAS;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w <= 0 || h <= 0) return;
    const dpr = window.devicePixelRatio;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const colors = palette(window.matchMedia("(prefers-color-scheme: dark)").matches);
    ctx.fillStyle = themeColor("--ss-bg", colors.bg);
    ctx.fillRect(0, 0, w, h);

    // Field fit (leave room for the HUD bars top and bottom)
    const { halfX, halfY } = command.field;
    const wall = 0.05;
    const layout = this.layoutHud(ctx, command, w);
    const top = layout.top;
    const bottom = layout.bottom;
    const xUp = command.view !== "x-right"; // x-up or x-down: field X runs vertically
    const flip = command.view === "x-down" ? -1 : 1;
    const fieldW = (xUp ? halfY : halfX) * 2 + wall * 2;
    const fieldH = (xUp ? halfX : halfY) * 2 + wall * 2;
    const scale = Math.min((w - 24) / fieldW, (h - top - bottom) / fieldH);
    const cx = w / 2;
    const cy = top + (h - top - bottom) / 2;
    // Field (x, y) -> screen
    const S = (x: number, y: number): [number, number] =>
      xUp ? [cx - flip * y * scale, cy - flip * x * scale] : [cx + x * scale, cy - y * scale];
    // Field heading -> screen angle (canvas rotation, clockwise-positive)
    const screenAngle = (heading: number) =>
      command.view === "x-up" ? -heading - Math.PI / 2 : command.view === "x-down" ? Math.PI / 2 - heading : -heading;

    this.drawField(ctx, colors, command, S, scale, xUp);
    if (command.game !== null) {
      this.drawGame(ctx, colors, command, S, scale);
    } else {
      this.drawTarget(ctx, colors, command, S, scale);
    }
    this.drawRests(ctx, colors, command, S, scale);
    this.drawTrail(ctx, colors, command, S);
    this.drawAim(ctx, colors, command, S);
    this.drawRobot(ctx, colors, command, S, scale, screenAngle);
    this.drawBalls(ctx, colors, command, S, scale);
    this.drawHud(ctx, colors, command, w, h, layout);
  }

  private drawField(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number],
    scale: number,
    xUp: boolean
  ) {
    const { halfX, halfY } = command.field;
    const wall = 0.05;
    // Perimeter wall
    const p0 = S(-halfX - wall, -halfY - wall);
    const p1 = S(halfX + wall, halfY + wall);
    const ax = Math.min(p0[0], p1[0]);
    const ay = Math.min(p0[1], p1[1]);
    const pw = Math.abs(p1[0] - p0[0]);
    const ph = Math.abs(p1[1] - p0[1]);
    void xUp;
    ctx.fillStyle = colors.wall;
    roundRect(ctx, ax, ay, pw, ph, 6);
    ctx.fill();

    // 6 x 6 foam tiles (24 in)
    const tile = (halfX * 2) / 6;
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        const x0 = -halfX + i * tile;
        const y0 = -halfY + j * tile;
        const corners = [S(x0, y0), S(x0 + tile, y0 + tile)];
        const sx = Math.min(corners[0][0], corners[1][0]);
        const sy = Math.min(corners[0][1], corners[1][1]);
        ctx.fillStyle = (i + j) % 2 === 0 ? colors.tileA : colors.tileB;
        ctx.fillRect(sx, sy, tile * scale, tile * scale);
      }
    }
    ctx.strokeStyle = colors.seam;
    ctx.lineWidth = 1;
    for (let i = 1; i < 6; i++) {
      const a = -halfX + i * tile;
      line(ctx, S(a, -halfY), S(a, halfY));
      const b = -halfY + i * tile;
      line(ctx, S(-halfX, b), S(halfX, b));
    }
    ctx.strokeStyle = colors.wallEdge;
    ctx.lineWidth = 2;
    const c0 = S(-halfX, -halfY);
    const c1 = S(halfX, halfY);
    ctx.strokeRect(Math.min(c0[0], c1[0]), Math.min(c0[1], c1[1]), Math.abs(c1[0] - c0[0]), Math.abs(c1[1] - c0[1]));

    // Axis key (bottom-left corner of the field)
    const [kx, ky] = S(-halfX + 0.18, -halfY + 0.18);
    const [kxx, kxy] = S(-halfX + 0.42, -halfY + 0.18);
    const [kyx, kyy] = S(-halfX + 0.18, -halfY + 0.42);
    ctx.lineWidth = 2;
    ctx.strokeStyle = colors.miss;
    arrow(ctx, kx, ky, kxx, kxy, 6);
    ctx.strokeStyle = colors.hit;
    arrow(ctx, kx, ky, kyx, kyy, 6);
    ctx.font = "600 10px 'Azeret Mono', monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillStyle = colors.miss;
    ctx.fillText("+X", kxx + (kxx - kx) * 0.35, kxy + (kxy - ky) * 0.35);
    ctx.fillStyle = colors.hit;
    ctx.fillText("+Y", kyx + (kyx - kx) * 0.35, kyy + (kyy - ky) * 0.35);
  }

  private drawTarget(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number],
    scale: number
  ) {
    const t = command.target;
    const a = S(t.x - t.sizeX / 2, t.y - t.sizeY / 2);
    const b = S(t.x + t.sizeX / 2, t.y + t.sizeY / 2);
    const x = Math.min(a[0], b[0]);
    const y = Math.min(a[1], b[1]);
    const ww = Math.abs(b[0] - a[0]);
    const hh = Math.abs(b[1] - a[1]);
    ctx.fillStyle = colors.targetFill;
    ctx.strokeStyle = colors.target;
    ctx.lineWidth = 2;
    roundRect(ctx, x, y, ww, hh, 4);
    ctx.fill();
    ctx.stroke();
    // Crosshair
    const [tx, ty] = S(t.x, t.y);
    ctx.lineWidth = 1;
    line(ctx, [tx - 6, ty], [tx + 6, ty]);
    line(ctx, [tx, ty - 6], [tx, ty + 6]);
    ctx.fillStyle = colors.target;
    ctx.font = "600 10px 'Azeret Mono', monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    ctx.fillText("TARGET " + t.z.toFixed(2) + " m", tx, y - 3);
    void scale;
  }

  /** Season field elements: alliance walls, zones, HIVE frame + CELLs, FLOWERS. */
  private drawGame(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number],
    scale: number
  ) {
    const game = command.game!;
    const rect = (minX: number, minY: number, maxX: number, maxY: number): [number, number, number, number] => {
      const a = S(minX, minY);
      const b = S(maxX, maxY);
      return [Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.abs(b[0] - a[0]), Math.abs(b[1] - a[1])];
    };
    const { halfX } = command.field;

    for (const zone of game.zones) {
      const color = (alpha: number) => allianceColor(zone.alliance, alpha);
      if (zone.kind === "alliance-area") {
        // Drawn as a thick band on the outside of the alliance's wall, along the area's width
        const wallX = zone.minX < 0 ? -halfX : halfX;
        const band = 0.05;
        const [x, y, w, h] = rect(
          wallX < 0 ? wallX - band : wallX,
          zone.minY,
          wallX < 0 ? wallX : wallX + band,
          zone.maxY
        );
        ctx.fillStyle = color(0.95);
        ctx.fillRect(x, y, w, h);
      } else if (zone.kind === "loading") {
        const [x, y, w, h] = rect(zone.minX, zone.minY, zone.maxX, zone.maxY);
        ctx.fillStyle = color(0.16);
        ctx.fillRect(x, y, w, h);
        ctx.strokeStyle = color(0.9);
        ctx.lineWidth = Math.max(1.5, 0.025 * scale);
        ctx.strokeRect(x, y, w, h);
        ctx.fillStyle = color(1);
        ctx.font = "600 9px 'Azeret Mono', monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.save();
        ctx.translate(x + w / 2, y + h / 2);
        if (h > w) ctx.rotate(-Math.PI / 2);
        ctx.fillText("LOADING", 0, 0);
        ctx.restore();
      } else {
        const [x, y, w, h] = rect(zone.minX, zone.minY, zone.maxX, zone.maxY);
        ctx.fillStyle = color(0.95);
        ctx.fillRect(x, y, Math.max(w, 3), Math.max(h, 3));
      }
    }

    // HIVE frame: dashed footprint (robots drive under it) + the four legs
    const [fx, fy, fw, fh] = rect(game.frame.minX, game.frame.minY, game.frame.maxX, game.frame.maxY);
    ctx.setLineDash([5, 5]);
    ctx.strokeStyle = colors.textDim;
    ctx.lineWidth = 1;
    ctx.strokeRect(fx, fy, fw, fh);
    ctx.setLineDash([]);
    ctx.fillStyle = colors.wall;
    for (const leg of game.legs) {
      const [x, y, w, h] = rect(leg.minX, leg.minY, leg.maxX, leg.maxY);
      ctx.fillRect(x - 1, y - 1, w + 2, h + 2);
    }

    // HIVE arms (pivot to both CELLs)
    for (const hive of game.hives) {
      const cells = game.cells.filter((c) => c.alliance === hive.alliance);
      ctx.strokeStyle = allianceColor(hive.alliance, 0.8);
      ctx.lineWidth = Math.max(2, 0.03 * scale);
      ctx.beginPath();
      cells.forEach((c, i) => {
        const [px, py] = S(c.x, c.y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      const [hx, hy] = S(hive.x, hive.y);
      ctx.fillStyle = colors.wall;
      ctx.beginPath();
      ctx.arc(hx, hy, Math.max(3, 0.03 * scale), 0, Math.PI * 2);
      ctx.fill();
    }

    // Mouth direction of the up CELLs: a CELL can only be scored from the side its mouth faces
    for (const cell of game.cells) {
      if (!cell.up || cell.facing === null) continue;
      const len = 0.32;
      const [ax, ay] = S(cell.x, cell.y);
      const [bx, by] = S(cell.x + cell.facing[0] * len, cell.y + cell.facing[1] * len);
      ctx.strokeStyle = allianceColor(cell.alliance, cell.active ? 1 : 0.6);
      ctx.lineWidth = cell.active ? 2.5 : 1.5;
      ctx.setLineDash([4, 3]);
      line(ctx, [ax, ay], [bx, by]);
      ctx.setLineDash([]);
      arrow(ctx, ax + (bx - ax) * 0.55, ay + (by - ay) * 0.55, bx, by, 7);
      if (cell.active) {
        ctx.fillStyle = allianceColor(cell.alliance, 1);
        ctx.font = "600 9px 'Azeret Mono', monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("MOUTH", bx + (bx - ax) * 0.25, by + (by - ay) * 0.25);
      }
    }

    // CELLs: the upward-facing one is solid; the active target glows
    for (const cell of game.cells) {
      const [x, y, w, h] = rect(
        cell.x - cell.sizeX / 2,
        cell.y - cell.sizeY / 2,
        cell.x + cell.sizeX / 2,
        cell.y + cell.sizeY / 2
      );
      if (cell.active) {
        ctx.save();
        ctx.shadowColor = allianceColor(cell.alliance, 0.9);
        ctx.shadowBlur = 16;
        ctx.fillStyle = allianceColor(cell.alliance, 0.55);
        roundRect(ctx, x, y, w, h, 4);
        ctx.fill();
        ctx.restore();
      } else {
        ctx.fillStyle = allianceColor(cell.alliance, cell.up ? 0.35 : 0.1);
        roundRect(ctx, x, y, w, h, 4);
        ctx.fill();
      }
      ctx.strokeStyle = allianceColor(cell.alliance, cell.up ? 1 : 0.45);
      ctx.lineWidth = cell.active ? 2.5 : 1.5;
      if (!cell.up) ctx.setLineDash([4, 3]);
      roundRect(ctx, x, y, w, h, 4);
      ctx.stroke();
      ctx.setLineDash([]);
      if (cell.up) {
        ctx.fillStyle = "#fff";
        ctx.font = "600 9px 'Azeret Mono', monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(cell.active ? "TARGET" : "UP", x + w / 2, y + h / 2);
      }
    }

    // FLOWERS on the walls
    for (const flower of game.flowers) {
      const [cx, cy] = S(flower.x, flower.y);
      const r = Math.max(4, (flower.diameter / 2) * scale);
      ctx.fillStyle = "#f2a7c9";
      for (let i = 0; i < 6; i++) {
        const a = (i / 6) * Math.PI * 2;
        ctx.beginPath();
        ctx.arc(cx + Math.cos(a) * r * 0.9, cy + Math.sin(a) * r * 0.9, r * 0.6, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.fillStyle = "#f2c94c";
      ctx.beginPath();
      ctx.arc(cx, cy, r * 0.7, 0, Math.PI * 2);
      ctx.fill();
    }

    // Season label in the field's top-left corner
    const [lx, ly] = rect(-halfX, -halfX, halfX, halfX);
    ctx.fillStyle = colors.textDim;
    ctx.font = "600 10px 'Azeret Mono', monospace";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText(game.name, lx + 6, ly + 6);
  }

  private drawRests(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number],
    scale: number
  ) {
    const r = Math.max(3, 0.03 * scale);
    for (const rest of command.rests) {
      const [x, y] = S(rest.x, rest.y);
      if (rest.hit) {
        ctx.fillStyle = colors.hit;
        ctx.globalAlpha = 0.8;
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.fill();
      } else {
        ctx.strokeStyle = colors.miss;
        ctx.globalAlpha = 0.8;
        ctx.lineWidth = 2;
        line(ctx, [x - r, y - r], [x + r, y + r]);
        line(ctx, [x - r, y + r], [x + r, y - r]);
      }
    }
    ctx.globalAlpha = 1;
  }

  private drawTrail(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number]
  ) {
    const trail = command.trail;
    ctx.lineWidth = 3;
    ctx.lineCap = "round";
    for (let i = 1; i < trail.length; i++) {
      const alpha = (i / trail.length) * 0.6;
      ctx.strokeStyle = "rgba(" + colors.trail + "," + alpha.toFixed(3) + ")";
      line(ctx, S(trail[i - 1][0], trail[i - 1][1]), S(trail[i][0], trail[i][1]));
    }
    ctx.lineCap = "butt";
  }

  private drawAim(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number]
  ) {
    const r = command.robot;
    const yaw = r.heading + r.turretYaw;
    const ox = r.x + Math.cos(r.heading) * r.shooterOffset;
    const oy = r.y + Math.sin(r.heading) * r.shooterOffset;
    const len = Math.max(0.6, command.hud.solution.distance);
    ctx.setLineDash([6, 6]);
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = command.hud.solution.valid ? colors.aimGood : colors.aimBad;
    line(ctx, S(ox, oy), S(ox + Math.cos(yaw) * len, oy + Math.sin(yaw) * len));
    ctx.setLineDash([]);
  }

  private drawRobot(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number],
    scale: number,
    screenAngle: (heading: number) => number
  ) {
    const r = command.robot;
    const [x, y] = S(r.x, r.y);
    const L = r.length * scale;
    const W = r.width * scale;

    // Velocity vector
    const speed = Math.hypot(r.vx, r.vy);
    if (speed > 0.05) {
      const [vx, vy] = S(r.x + r.vx * 0.5, r.y + r.vy * 0.5);
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = 2.5;
      arrow(ctx, x, y, vx, vy, 8);
    }

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(screenAngle(r.heading));
    // In this frame: +x = robot forward, +y = robot right

    // Shadow
    ctx.fillStyle = colors.shadow;
    roundRect(ctx, -L / 2 + 3, -W / 2 + 4, L, W, 6);
    ctx.fill();

    // Wheels / modules
    ctx.fillStyle = colors.wheel;
    const wl = L * 0.24;
    const ww = W * 0.12;
    for (const [sx, sy] of [
      [1, 1],
      [1, -1],
      [-1, 1],
      [-1, -1]
    ]) {
      const px = sx * (L / 2 - wl / 2 - L * 0.05);
      const py = sy * (W / 2 - ww / 2);
      ctx.save();
      ctx.translate(px, py);
      if (r.drivetrain === "swerve" && speed > 0.05) {
        // Point modules along the robot-relative velocity
        const rel = Math.atan2(r.vy, r.vx) - r.heading;
        ctx.rotate(-rel);
      }
      ctx.fillRect(-wl / 2, -ww / 2, wl, ww);
      if (r.drivetrain === "mecanum") {
        ctx.strokeStyle = "rgba(255,255,255,0.35)";
        ctx.lineWidth = 1;
        const dir = sx * sy;
        for (let k = -2; k <= 2; k++) {
          line(ctx, [k * (wl / 5) - ww / 2, (-ww / 2) * dir], [k * (wl / 5) + ww / 2, (ww / 2) * dir]);
        }
      }
      ctx.restore();
    }

    // Chassis
    ctx.fillStyle = colors.robot;
    // Outline in the alliance color (like the required ROBOT SIGNs), red when touching something
    ctx.strokeStyle = r.wallContact
      ? colors.miss
      : command.game !== null
      ? allianceColor(command.game.alliance, 1)
      : colors.robotEdge;
    ctx.lineWidth = r.wallContact ? 3 : 1.5;
    roundRect(ctx, -L / 2 + wl * 0.15, -W / 2 + ww, L - wl * 0.3, W - ww * 2, 5);
    ctx.fill();
    ctx.stroke();

    // Front bar + direction chevron
    ctx.fillStyle = colors.robotEdge;
    ctx.fillRect(L / 2 - wl * 0.15 - 4, -W / 2 + ww, 4, W - ww * 2);
    ctx.fillStyle = "rgba(11,10,6,0.82)";
    ctx.beginPath();
    ctx.moveTo(L * 0.28, 0);
    ctx.lineTo(L * 0.08, -W * 0.14);
    ctx.lineTo(L * 0.08, W * 0.14);
    ctx.closePath();
    ctx.fill();

    // Turret
    const tx = r.shooterOffset * scale;
    ctx.save();
    ctx.translate(Math.max(-L / 2, Math.min(L / 2, tx)), 0);
    ctx.rotate(-r.turretYaw);
    ctx.fillStyle = "#2b2b2b";
    ctx.fillRect(0, -3, L * 0.32, 6);
    ctx.beginPath();
    ctx.arc(0, 0, Math.max(5, L * 0.12), 0, Math.PI * 2);
    ctx.fillStyle = "#3a3a3a";
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,0.4)";
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.restore();

    ctx.restore();
  }

  private drawBalls(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    S: (x: number, y: number) => [number, number],
    scale: number
  ) {
    // Shadows first so balls draw over all shadows
    for (const b of command.balls) {
      const [x, y] = S(b.x, b.y);
      const r = b.radius * scale;
      ctx.fillStyle = colors.shadow;
      ctx.globalAlpha = Math.max(0.15, 1 - b.z / 2.5);
      ctx.beginPath();
      ctx.ellipse(x + b.z * 6, y + b.z * 10, r, r * 0.8, 0, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
    for (const b of command.balls) {
      const [x, y] = S(b.x, b.y);
      // Higher = bigger (perspective cue)
      const r = b.radius * scale * (1 + b.z * 0.35);
      const grad = ctx.createRadialGradient(x - r * 0.35, y - r * 0.35, r * 0.1, x, y, r);
      grad.addColorStop(0, "#fff7c2");
      grad.addColorStop(1, command.game?.pieceColor ?? colors.ball);
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(x, y, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(0,0,0,0.35)";
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }

  /** Works out where the HUD pieces go for this canvas size, so the field can fit around them. */
  private layoutHud(ctx: CanvasRenderingContext2D, command: DriveSimRendererCommand, w: number): HudLayout {
    const keys: [string, string][] =
      command.hud.input === "gamepad"
        ? [
            ["L stick", "drive"],
            ["R stick", "turn"],
            ["RT / A", "shoot"],
            ["LB", "slow"],
            ["Y", "field-centric"],
            ["Back", "reset"]
          ]
        : [
            ["W A S D", "drive"],
            ["Q E", "turn"],
            ["Space", "shoot"],
            ["Shift", "slow"],
            ["F", "field-centric"],
            ["R", "reset"],
            ["C", "clear"]
          ];
    const items: HintItem[] = keys.map(([k, label]) => {
      ctx.font = "bold 11px 'Inter Variable', sans-serif";
      const kw = ctx.measureText(k).width + 12;
      ctx.font = "11px 'Inter Variable', sans-serif";
      return { k, label, kw, total: kw + 5 + ctx.measureText(label).width + 14 };
    });
    // Greedy wrap into rows that fit the canvas
    const rows: HintItem[][] = [[]];
    let rowWidth = 0;
    for (const item of items) {
      if (rowWidth + item.total > w - 20 && rows[rows.length - 1].length > 0) {
        rows.push([]);
        rowWidth = 0;
      }
      rows[rows.length - 1].push(item);
      rowWidth += item.total;
    }
    const leftWidth = 250;
    const rightWidth = 210;
    const stacked = w < leftWidth + rightWidth + 30;
    return {
      rows,
      stacked,
      top: stacked ? 150 : 94,
      bottom: 44 + rows.length * 24,
      leftWidth,
      rightWidth
    };
  }

  private drawHud(
    ctx: CanvasRenderingContext2D,
    colors: Palette,
    command: DriveSimRendererCommand,
    w: number,
    h: number,
    layout: HudLayout
  ) {
    const hud = command.hud;

    // --- Top-left: drive telemetry ---
    const lw = layout.leftWidth;
    panel(ctx, colors, 10, 10, lw, 46);
    ctx.textBaseline = "alphabetic";
    ctx.textAlign = "left";
    ctx.fillStyle = colors.text;
    ctx.font = "bold 20px 'Inter Variable', sans-serif";
    const speedText = hud.speed.toFixed(2);
    ctx.fillText(speedText, 20, 40);
    const speedWidth = ctx.measureText(speedText).width;
    ctx.font = "11px 'Inter Variable', sans-serif";
    ctx.fillStyle = colors.textDim;
    ctx.fillText("m/s", 24 + speedWidth, 40);
    ctx.fillText("heading", 98, 26);
    ctx.fillStyle = colors.text;
    ctx.font = "bold 13px 'Inter Variable', sans-serif";
    ctx.fillText((((hud.headingDeg % 360) + 360) % 360).toFixed(0) + "°", 98, 43);
    ctx.fillStyle = colors.textDim;
    ctx.font = "11px 'Inter Variable', sans-serif";
    ctx.fillText("wheel load", 156, 26);
    ctx.fillStyle = colors.keycap;
    roundRect(ctx, 156, 34, lw - 166, 8, 4);
    ctx.fill();
    if (hud.wheelLoad > 0.01) {
      ctx.fillStyle = hud.wheelLoad > 0.98 ? colors.miss : colors.accent;
      roundRect(ctx, 156, 34, (lw - 166) * hud.wheelLoad, 8, 4);
      ctx.fill();
    }

    // --- Chip row: mode, slow, input, recording ---
    const chipY = 64;
    let chipX = 10;
    chipX += chip(ctx, colors, chipX, chipY, hud.fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC", colors.accent) + 6;
    if (hud.slow) chipX += chip(ctx, colors, chipX, chipY, "SLOW", colors.robot) + 6;
    const inputText =
      hud.input === "gamepad" ? "GAMEPAD" : hud.gamepadConnected ? "KEYBOARD · GAMEPAD READY" : "KEYBOARD";
    chipX += chip(ctx, colors, chipX, chipY, inputText, colors.textDim) + 6;
    if (hud.recording) {
      const blink = Math.floor(Date.now() / 500) % 2 === 0;
      chip(ctx, colors, chipX, chipY, (blink ? "● " : "○ ") + "REC " + formatTime(hud.recordTime), colors.rec);
    }

    // --- Score + shot feed: top-right, or under the chips when the canvas is narrow ---
    const rw = layout.stacked ? lw : layout.rightWidth;
    const px = layout.stacked ? 10 : w - rw - 10;
    const py = layout.stacked ? 90 : 10;
    panel(ctx, colors, px, py, rw, 46);
    ctx.textAlign = "left";
    ctx.textBaseline = "alphabetic";
    ctx.fillStyle = colors.text;
    ctx.font = "bold 20px 'Inter Variable', sans-serif";
    const scoreText = hud.hits + " / " + hud.shots;
    ctx.fillText(scoreText, px + 12, py + 30);
    const scoreWidth = ctx.measureText(scoreText).width;
    ctx.font = "11px 'Inter Variable', sans-serif";
    ctx.fillStyle = colors.textDim;
    ctx.fillText("hits", px + 17 + scoreWidth, py + 30);
    const rate = hud.shots > 0 ? hud.hits / hud.shots : null;
    ctx.textAlign = "right";
    ctx.font = "bold 20px 'Inter Variable', sans-serif";
    ctx.fillStyle = rate === null ? colors.textDim : rate >= 0.8 ? colors.hit : rate < 0.5 ? colors.miss : colors.text;
    ctx.fillText(rate === null ? "—" : Math.round(rate * 100) + "%", px + rw - 12, py + 30);

    const feed = command.feed.filter((item) => item.age < 6);
    if (feed.length > 0 && !layout.stacked) {
      panel(ctx, colors, px, py + 52, rw, feed.length * 18 + 10);
      feed.forEach((item, i) => {
        const y = py + 52 + 18 + i * 18;
        ctx.globalAlpha = Math.min(1, Math.max(0, (6 - item.age) / 1.5));
        ctx.fillStyle = item.hit ? colors.hit : colors.miss;
        ctx.beginPath();
        ctx.arc(px + 12, y - 4, 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = colors.text;
        ctx.textAlign = "left";
        ctx.font = "12px 'Inter Variable', sans-serif";
        ctx.fillText(ellipsize(ctx, item.text, rw - 30), px + 22, y);
      });
      ctx.globalAlpha = 1;
    }

    // --- Bottom: shot solution, then key hints ---
    const hintTop = h - layout.rows.length * 24 - 4;
    const sol = hud.solution;
    const solText = sol.valid
      ? (hud.aim === "turret" ? "Turret locked · " : "Fixed shooter · ") +
        sol.exitSpeed.toFixed(2) +
        " m/s · " +
        sol.distance.toFixed(2) +
        " m to target"
      : "No shot solution at " + sol.distance.toFixed(2) + " m · move or change the hood angle";
    ctx.font = "bold 12px 'Inter Variable', sans-serif";
    const solShown = ellipsize(ctx, solText, w - 140);
    const sw = ctx.measureText(solShown).width + 34;
    const sy = hintTop - 32;
    panel(ctx, colors, w / 2 - sw / 2, sy, sw, 24);
    ctx.fillStyle = sol.valid ? colors.hit : colors.miss;
    ctx.beginPath();
    ctx.arc(w / 2 - sw / 2 + 13, sy + 12, 4.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = colors.text;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText(solShown, w / 2 - sw / 2 + 24, sy + 12);

    // Battery on the same row, if there's room
    if (w / 2 + sw / 2 + 60 < w) {
      ctx.font = "11px 'Inter Variable', sans-serif";
      ctx.textAlign = "right";
      ctx.fillStyle = hud.battery < 12 ? colors.miss : colors.textDim;
      ctx.fillText(hud.battery.toFixed(1) + " V", w - 12, sy + 12);
    }

    layout.rows.forEach((row, r) => {
      const total = row.reduce((a, p) => a + p.total, 0) - 14;
      let kx = Math.max(10, w / 2 - total / 2);
      const ky = hintTop + 12 + r * 24;
      for (const p of row) {
        ctx.fillStyle = colors.keycap;
        roundRect(ctx, kx, ky - 9, p.kw, 18, 4);
        ctx.fill();
        ctx.strokeStyle = colors.panelBorder;
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.fillStyle = colors.text;
        ctx.font = "bold 11px 'Inter Variable', sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(p.k, kx + p.kw / 2, ky);
        ctx.fillStyle = colors.textDim;
        ctx.font = "11px 'Inter Variable', sans-serif";
        ctx.textAlign = "left";
        ctx.fillText(p.label, kx + p.kw + 5, ky);
        kx += p.total;
      }
    });
    ctx.textBaseline = "alphabetic";
  }
}

// ---------------------------------------------------------------------------------------------

function ellipsize(ctx: CanvasRenderingContext2D, text: string, maxWidth: number): string {
  if (ctx.measureText(text).width <= maxWidth) return text;
  let out = text;
  while (out.length > 1 && ctx.measureText(out + "…").width > maxWidth) out = out.slice(0, -1);
  return out + "…";
}

function allianceColor(alliance: "red" | "blue", alpha: number): string {
  return alliance === "red" ? "rgba(229,72,77," + alpha + ")" : "rgba(59,130,246," + alpha + ")";
}

function formatTime(seconds: number): string {
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return m.toString().padStart(2, "0") + ":" + s.toFixed(1).padStart(4, "0");
}

function roundRect(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number) {
  const rr = Math.max(0, Math.min(r, Math.abs(w) / 2, Math.abs(h) / 2));
  ctx.beginPath();
  ctx.moveTo(x + rr, y);
  ctx.arcTo(x + w, y, x + w, y + h, rr);
  ctx.arcTo(x + w, y + h, x, y + h, rr);
  ctx.arcTo(x, y + h, x, y, rr);
  ctx.arcTo(x, y, x + w, y, rr);
  ctx.closePath();
}

function panel(ctx: CanvasRenderingContext2D, colors: Palette, x: number, y: number, w: number, h: number) {
  ctx.fillStyle = colors.panel;
  ctx.strokeStyle = colors.panelBorder;
  ctx.lineWidth = 1;
  roundRect(ctx, x, y, w, h, 8);
  ctx.fill();
  ctx.stroke();
}

/** Draws a small pill label; returns its width. */
function chip(
  ctx: CanvasRenderingContext2D,
  colors: Palette,
  x: number,
  y: number,
  text: string,
  color: string
): number {
  ctx.font = "600 10px 'Azeret Mono', monospace";
  const w = ctx.measureText(text).width + 14;
  ctx.fillStyle = colors.panel;
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  roundRect(ctx, x, y, w, 18, 9);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  ctx.fillText(text, x + 7, y + 9.5);
  ctx.textBaseline = "alphabetic";
  return w;
}

function line(ctx: CanvasRenderingContext2D, a: [number, number], b: [number, number]) {
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.stroke();
}

function arrow(ctx: CanvasRenderingContext2D, x0: number, y0: number, x1: number, y1: number, head: number) {
  line(ctx, [x0, y0], [x1, y1]);
  const angle = Math.atan2(y1 - y0, x1 - x0);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - head * Math.cos(angle - 0.45), y1 - head * Math.sin(angle - 0.45));
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - head * Math.cos(angle + 0.45), y1 - head * Math.sin(angle + 0.45));
  ctx.stroke();
}
