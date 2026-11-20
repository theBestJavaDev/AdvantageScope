// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { Pose3d, RobotVelocities, rotation2dTo3d } from "../geometry";
import { PhysicsEngine } from "./PhysicsEngine";

export class TeleopController {
  private isTeleopActive = false;

  private maxLinearVelocity = 3.5; // m/s
  private maxAngularVelocity = Math.PI * 2; // rad/s

  private linearAccelRate = 12.0; // m/s^2
  private angularAccelRate = Math.PI * 6; // rad/s^2

  private currentVx = 0;
  private currentVy = 0;
  private currentOmega = 0;

  private keysPressed: Set<string> = new Set();

  constructor() {
    this.setupKeyboardListeners();
  }

  /**
   * Scans log text or code contents for teleop declaration line.
   * Format allowed: "# MODE: teleop", "isTeleop = true", or "RobotMode: teleop"
   */
  public detectModeFromFileContent(fileContent: string): "teleop" | "auto" {
    const lines = fileContent.split(/\r?\n/);
    for (const line of lines) {
      const cleanLine = line.trim().toLowerCase();
      if (
        cleanLine.includes("# mode: teleop") ||
        cleanLine.includes("mode: teleop") ||
        cleanLine.includes("isteleop = true") ||
        cleanLine.includes("robotmode: teleop")
      ) {
        this.isTeleopActive = true;
        return "teleop";
      }
    }
    this.isTeleopActive = false;
    return "auto";
  }

  public setTeleopActive(active: boolean) {
    this.isTeleopActive = active;
  }

  public isTeleop(): boolean {
    return this.isTeleopActive;
  }

  private setupKeyboardListeners() {
    window.addEventListener("keydown", (event) => {
      if (
        event.target instanceof HTMLInputElement ||
        event.target instanceof HTMLTextAreaElement ||
        (event.target instanceof HTMLElement && event.target.isContentEditable)
      ) {
        return;
      }
      this.keysPressed.add(event.code);
    });

    window.addEventListener("keyup", (event) => {
      this.keysPressed.delete(event.code);
    });
  }

  public updateVelocities(dt: number): RobotVelocities {
    if (!this.isTeleopActive) {
      return { vx: 0, vy: 0, omega: 0 };
    }

    let targetVx = 0;
    let targetVy = 0;
    let targetOmega = 0;

    // WASD Translation
    if (this.keysPressed.has("KeyW")) targetVx += this.maxLinearVelocity;
    if (this.keysPressed.has("KeyS")) targetVx -= this.maxLinearVelocity;
    if (this.keysPressed.has("KeyA")) targetVy += this.maxLinearVelocity;
    if (this.keysPressed.has("KeyD")) targetVy -= this.maxLinearVelocity;

    // QE Turning
    if (this.keysPressed.has("KeyQ")) targetOmega += this.maxAngularVelocity;
    if (this.keysPressed.has("KeyE")) targetOmega -= this.maxAngularVelocity;

    this.currentVx = this.approach(this.currentVx, targetVx, this.linearAccelRate * dt);
    this.currentVy = this.approach(this.currentVy, targetVy, this.linearAccelRate * dt);
    this.currentOmega = this.approach(this.currentOmega, targetOmega, this.angularAccelRate * dt);

    return {
      vx: this.currentVx,
      vy: this.currentVy,
      omega: this.currentOmega
    };
  }

  public stepRobotPose(
    currentPose: Pose3d,
    dt: number,
    physicsEngine: PhysicsEngine
  ): Pose3d {
    if (!this.isTeleopActive) return currentPose;

    const velocities = this.updateVelocities(dt);
    const q = currentPose.rotation;
    const yaw = Math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));

    const fieldVx = velocities.vx * Math.cos(yaw) - velocities.vy * Math.sin(yaw);
    const fieldVy = velocities.vx * Math.sin(yaw) + velocities.vy * Math.cos(yaw);

    const updatedHeading = yaw + velocities.omega * dt;
    const updatedRotation = rotation2dTo3d(updatedHeading);

    const updatedTranslation: [number, number, number] = [
      currentPose.translation[0] + fieldVx * dt,
      currentPose.translation[1] + fieldVy * dt,
      currentPose.translation[2]
    ];

    return {
      translation: updatedTranslation,
      rotation: updatedRotation
    };
  }

  private approach(current: number, target: number, maxStep: number): number {
    const diff = target - current;
    if (Math.abs(diff) <= maxStep) return target;
    return current + Math.sign(diff) * maxStep;
  }
}
