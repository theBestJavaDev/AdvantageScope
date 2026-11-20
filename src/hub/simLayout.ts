// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { TabsState } from "../shared/HubState";
import LoggableType from "../shared/log/LoggableType";
import TabType from "../shared/TabType";
import { createUUID } from "../shared/util";

/**
 * Opening a BioBuzz SimLoop log (the robot repo's `simAuto` / sim tests) with no field views set up
 * shows only an empty field, which looks like the run did nothing. When such a log loads and no
 * field or graph tab has anything assigned, this adds the team's sim layout (www/layouts/biobuzz-sim.json,
 * a copy of the robot repo's sim/advantagescope/layout.json): 3D + 2D field with the robot, POLLEN in
 * flight, HIVE tips and shot arcs, plus the mechanism and graph tabs. Then it plays the run from START.
 * Tabs the user configured, and the SolversScope tools, are kept.
 */

/** Present in every BioBuzz SimLoop RLOG (the true robot pose for the field views). */
const SIM_ROBOT_KEY = "RealOutputs/Field/Robot";
const LEG_KEY = "RealOutputs/Auto/leg";
const LAYOUT_URL = "layouts/biobuzz-sim.json";

let lastHandledLog: string | null = null;

/** Tab types that are empty views until fields are assigned. */
const VIEW_TYPES = [TabType.LineGraph, TabType.Field2d, TabType.Field3d, TabType.Mechanism];

function isUnconfiguredView(tab: TabsState["tabs"][number]): boolean {
  if (!VIEW_TYPES.includes(tab.type)) return false;
  const controller = tab.controller as Record<string, unknown> | null;
  if (controller === null || typeof controller !== "object") return true;
  const lists = ["sources", "leftSources", "rightSources", "discreteSources"]
    .map((key) => controller[key])
    .filter((value): value is unknown[] => Array.isArray(value));
  return lists.every((list) => list.length === 0);
}

/** The sim layout's 3D field: the 2026-2027 field without its fixed HIVE bars and staged balls. */
const TIPPING_FIELD_ID = "FTC:BioBuzz Sim Field";
const HIVE_SOURCE_KEY = "RealOutputs/Field/HiveRed";

/**
 * 3D tabs that draw the sim's HIVEs should sit on the tipping field (made by the robot repo's
 * make_field_model.py). The stock field has the HIVE bars fixed in place and staged balls of its own,
 * so tips would not show and balls would be doubled. A restore can land on the stock field when the
 * field list was not loaded yet, so this checks every time a sim log opens. No-op if the tipping
 * field is not installed on this computer.
 */
function useTippingField() {
  if (!(window.assets?.field3ds ?? []).some((field) => field.id === TIPPING_FIELD_ID)) return;
  const state = window.tabs.saveState();
  let changed = false;
  for (const tab of state.tabs) {
    if (tab.type !== TabType.Field3d) continue;
    const controller = tab.controller as { sources?: { logKey: string }[]; game?: string } | null;
    if (controller === null || !Array.isArray(controller.sources)) continue;
    if (!controller.sources.some((source) => source.logKey === HIVE_SOURCE_KEY)) continue;
    if (controller.game !== TIPPING_FIELD_ID) {
      controller.game = TIPPING_FIELD_ID;
      changed = true;
    }
  }
  if (changed) window.tabs.restoreState(state);
}

/** Puts an Auto Check tab next to the sim views (once), so every sim log gets its shots checked. */
function addAutoCheckTab() {
  const state = window.tabs.saveState();
  if (state.tabs.some((tab) => tab.type === TabType.AutoCheck)) return;
  state.tabs.push({
    type: TabType.AutoCheck,
    title: "",
    controller: null,
    controllerUUID: createUUID(),
    renderer: null,
    controlsHeight: 0
  } as TabsState["tabs"][number]);
  window.tabs.restoreState(state);
}

/** When the run starts: the auto's START (after INIT), else the start of the log. */
function startTime(): number {
  const range = window.log.getTimestampRange();
  if (window.log.getType(LEG_KEY) === LoggableType.String) {
    const legs = window.log.getString(LEG_KEY, -Infinity, Infinity);
    if (legs !== undefined) {
      const index = legs.values.findIndex((value) => value === "START");
      if (index >= 0) return legs.timestamps[index];
    }
  }
  return range[0];
}

/**
 * Called when a log finishes its initial decode. Applies the sim layout if this is a BioBuzz sim
 * log and nothing is set up to show it, then plays from START.
 *
 * @param logName name of the opened log, so each log is only handled once
 */
export async function maybeApplySimLayout(logName: string): Promise<void> {
  if (logName === lastHandledLog) return;
  if (!window.log.getFieldKeys().includes(SIM_ROBOT_KEY)) return;
  lastHandledLog = logName;

  const current = window.tabs.saveState();
  const fieldTabs = current.tabs.filter((tab) => tab.type === TabType.Field2d || tab.type === TabType.Field3d);
  const alreadySetUp = fieldTabs.some((tab) => !isUnconfiguredView(tab));

  if (!alreadySetUp) {
    let layout: { hubs?: { state?: { tabs?: TabsState } }[] };
    try {
      const response = await fetch(LAYOUT_URL);
      layout = await response.json();
    } catch (error) {
      console.error("Could not load the BioBuzz sim layout", error);
      return;
    }
    const simTabs = layout.hubs?.[0]?.state?.tabs;
    if (simTabs === undefined) return;

    // Keep the user's tools and configured tabs; drop only empty views. The layout's own
    // documentation tab is skipped when one already exists.
    const kept = current.tabs.filter((tab) => !isUnconfiguredView(tab));
    const hasDocs = kept.some((tab) => tab.type === TabType.Documentation);
    const added = simTabs.tabs.filter((tab) => !(hasDocs && tab.type === TabType.Documentation));
    const merged: TabsState = {
      selected: 0,
      tabs: [...kept, ...added]
    };
    const field3d = merged.tabs.findIndex((tab, i) => i >= kept.length && tab.type === TabType.Field3d);
    merged.selected = field3d >= 0 ? field3d : kept.length;
    window.tabs.restoreState(merged);
  }

  useTippingField();
  addAutoCheckTab();

  // Nothing is drawn until a time is selected: play the run from START
  window.selection.setSelectedTime(startTime());
  window.selection.play();
}
