// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * "Choose your robot CAD" prompt. When a robot-sim log is drawn with a generic robot (no team CAD
 * installed), the hub asks for this. The chosen model is copied into the user assets folder as an
 * AdvantageScope robot, so it is there from then on.
 */

import { BrowserWindow, dialog } from "electron";
import fs from "fs";
import jsonfile from "jsonfile";
import path from "path";
import Preferences from "../../shared/Preferences";
import { getUserAssetsPath } from "./assetLoader";
import { PREFS_FILENAME } from "./ElectronConstants";

/**
 * Copies a robot model into the user assets folder and returns its name.
 *
 * - `model.glb` inside an AdvantageScope robot folder (one with config.json): the whole folder is copied.
 * - Any other .glb (e.g. exported from Onshape/Fusion, Y-up in meters): a robot folder is made for it.
 *   Adjust `rotations`/`position` in its config.json if it sits wrong.
 */
function installRobotCad(file: string): string {
  const userAssets = getUserAssetsPath();
  if (!fs.existsSync(userAssets)) fs.mkdirSync(userAssets, { recursive: true });
  const sourceFolder = path.dirname(file);
  const sourceConfig = path.join(sourceFolder, "config.json");

  if (path.basename(file) === "model.glb" && fs.existsSync(sourceConfig)) {
    const config = jsonfile.readFileSync(sourceConfig) as { name?: unknown };
    const name = typeof config.name === "string" ? config.name : path.basename(sourceFolder);
    let folderName = path.basename(sourceFolder);
    if (!folderName.startsWith("Robot_")) folderName = "Robot_" + folderName;
    const target = path.join(userAssets, folderName);
    if (path.resolve(target) !== path.resolve(sourceFolder)) {
      fs.cpSync(sourceFolder, target, { recursive: true });
    }
    return name;
  }

  const name = path.basename(file, path.extname(file)).trim() || "Robot CAD";
  const target = path.join(userAssets, "Robot_" + name.replace(/[^A-Za-z0-9_-]+/g, "_"));
  fs.mkdirSync(target, { recursive: true });
  fs.copyFileSync(file, path.join(target, "model.glb"));
  jsonfile.writeFileSync(
    path.join(target, "config.json"),
    {
      name: name,
      isFTC: true,
      disableSimplification: true,
      rotations: [{ axis: "x", degrees: 90 }], // glTF is Y-up; the field is Z-up
      position: [0, 0, 0],
      cameras: [],
      components: []
    },
    { spaces: 2 }
  );
  return name;
}

/**
 * Asks for the robot's CAD. Resolves to the installed model's name, or null if the user keeps the
 * default robot (or ticks "Don't ask again", which is saved).
 */
export async function promptRobotCad(window: BrowserWindow): Promise<string | null> {
  const prefs: Preferences = jsonfile.readFileSync(PREFS_FILENAME);
  if (prefs.skipRobotCadPrompt) return null;
  const answer = await dialog.showMessageBox(window, {
    type: "question",
    title: "Robot CAD",
    message: "No robot CAD found",
    detail:
      "This log's robot is drawn with a generic model. Choose your robot's CAD (a .glb file) to see your own " +
      "robot on the field. It is saved with your assets, so you only do this once.",
    buttons: ["Choose CAD…", "Use Default Robot"],
    defaultId: 0,
    cancelId: 1,
    checkboxLabel: "Don't ask again"
  });
  if (answer.checkboxChecked) {
    const latest: Preferences = jsonfile.readFileSync(PREFS_FILENAME);
    latest.skipRobotCadPrompt = true;
    jsonfile.writeFileSync(PREFS_FILENAME, latest);
  }
  if (answer.response !== 0) return null;

  const pick = await dialog.showOpenDialog(window, {
    title: "Choose your robot CAD",
    properties: ["openFile"],
    filters: [{ name: "Robot model (glTF binary)", extensions: ["glb"] }]
  });
  if (pick.canceled || pick.filePaths.length === 0) return null;
  try {
    return installRobotCad(pick.filePaths[0]);
  } catch (error) {
    dialog.showMessageBox(window, {
      type: "error",
      title: "Error",
      message: "Could not add the robot CAD",
      detail: String(error)
    });
    return null;
  }
}
