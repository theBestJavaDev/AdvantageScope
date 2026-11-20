// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { dialog } from "electron";
import { WINDOW_ICON } from "./ElectronConstants";

/** Standalone UpdateChecker: External network update checks disabled for SolversScope. */
export default class UpdateChecker {
  private shouldPrompt = false;

  async check() {
    this.shouldPrompt = false;
  }

  getShouldPrompt(): boolean {
    return false;
  }

  async showPrompt() {
    await dialog.showMessageBox({
      type: "info",
      title: "SolversScope",
      message: "No updates checked",
      detail: "SolversScope runs in offline standalone mode. Third-party telemetry and tracking are disabled.",
      icon: WINDOW_ICON
    });
  }
}
