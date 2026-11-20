// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { analyzeAutoShots } from "../shared/physics/AutoShotCheck";
import { mergeShotLabConfig, runShotLab } from "../shared/physics/ShotLab";

/**
 * Runs the heavy simulations off the UI thread.
 *  - Shot Lab: payload { config, includeBudget }
 *  - Auto Check: payload { kind: "autoCheck", config, shots, options }
 */
self.onmessage = (event) => {
  self.onmessage = null;
  const { id, payload } = event.data;
  try {
    const config = mergeShotLabConfig(payload.config);
    if (payload.kind === "autoCheck") {
      self.postMessage({ id: id, payload: analyzeAutoShots(payload.shots, config, payload.options) });
      return;
    }
    const stages = payload.includeBudget ? ["Monte Carlo", "error budget"] : ["Monte Carlo"];
    const run = runShotLab(config, payload.includeBudget === true, (stage) => {
      self.postMessage({ id: id, progress: stages.indexOf(stage) / stages.length });
    });
    self.postMessage({ id: id, payload: run });
  } catch (error) {
    console.error(error);
    self.postMessage({ id: id });
  }
};
