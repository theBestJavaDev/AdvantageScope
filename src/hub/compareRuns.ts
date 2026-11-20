// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { TabsState } from "../shared/HubState";
import LoggableType from "../shared/log/LoggableType";
import { applyKeyPrefix, MERGE_PREFIX } from "../shared/log/LogKeyUtils";
import TabType from "../shared/TabType";
import { createUUID } from "../shared/util";
import { MultiFieldPaneState, MultiFieldState } from "./controllers/MultiFieldController";
import { SIM_LAYOUT_URL, TEAM_CAD, TIPPING_FIELD_ID } from "./simLayout";

/**
 * Compare Runs (File menu, or the Multi Field tab's button): up to three logs side by side in one
 * Multi Field tab, e.g. two BioBuzz autos (a 2-tip and a 2.5-tip auto) or a BioBuzz and a DECODE
 * auto. The logs load together (each under /Log0, /Log1, /Log2, lined up on one timeline) and each
 * run gets its own field pane, all playing in sync. Each pane has its own camera and field, so
 * different seasons work.
 *
 *  - BioBuzz SimLoop logs get the team's sim view (robot, POLLEN, tipping HIVEs, shot arc).
 *  - Any other log gets its robot pose (the best-named Pose2d/Pose3d field) on that season's field
 *    (DECODE for 2025-26 logs, else the newest FTC field). Change it from the pane's Field menu.
 */

export const MAX_COMPARE_RUNS = 3;

/** Runs loading for a comparison, and the Multi Field tab to show them in (null: a new Compare tab). */
let pending: { files: string[]; targetUuid: string | null } | null = null;
/** Single runs waiting to load into one field of a Multi Field tab ("Choose run…"). */
let pendingFields: { file: string; prefix: string; uuid: string; pane: number }[] = [];

/** A field's "Choose run…": once `file` has loaded under `prefix`, that field is set up for it. */
export function queueRunForField(file: string, prefix: string, uuid: string, pane: number) {
  pendingFields = pendingFields.filter((entry) => !(entry.uuid === uuid && entry.pane === pane));
  pendingFields.push({ file, prefix, uuid, pane });
}

async function fillQueuedFields(keys: string[]) {
  const ready = pendingFields.filter((entry) =>
    entry.prefix === "" ? keys.length > 0 : keys.some((key) => key.startsWith(entry.prefix + "/"))
  );
  if (ready.length === 0) return;
  pendingFields = pendingFields.filter((entry) => !ready.includes(entry));
  const states = await Promise.all(ready.map((entry) => stateForRun(entry.file, entry.prefix, keys)));
  const tabs = window.tabs.saveState();
  ready.forEach((entry, i) => {
    const tab = tabs.tabs.find((t) => t.controllerUUID === entry.uuid);
    const panes = (tab?.controller as MultiFieldState | null)?.panes;
    if (panes === undefined || entry.pane < 0 || entry.pane >= panes.length) return;
    panes[entry.pane] = { title: baseName(entry.file), field3d: states[i] };
  });
  window.tabs.restoreState(tabs);
  const starts = ready.map((entry) => runStart(entry.prefix)).filter((t): t is number => t !== null);
  if (starts.length > 0) window.selection.setSelectedTime(Math.min(...starts));
  window.selection.play();
}

/**
 * Starts a comparison: call before the logs are opened. Returns the files actually used (max 3).
 * With `targetUuid` (a field's "Choose run…" with several logs picked), that Multi Field tab shows
 * exactly these runs, one field each; otherwise a Compare tab is made.
 */
export function beginCompare(files: string[], targetUuid: string | null = null): string[] {
  const used = files.slice(0, MAX_COMPARE_RUNS);
  pending = { files: used, targetUuid };
  return used;
}

function baseName(file: string): string {
  const parts = file.split(/[\\/]/);
  return parts[parts.length - 1].replace(/\.[^.]+$/, "");
}

function fieldExists(id: string): boolean {
  return (window.assets?.field3ds ?? []).some((field) => field.id === id);
}

/** Newest FTC 3D field, or one for a given season year ("2025" -> the 2025-2026 field). */
function ftcField(seasonStart?: number): string | undefined {
  const fields = (window.assets?.field3ds ?? []).filter((field) => field.isFTC);
  const year = (id: string) => Number(/(20\d{2})/.exec(id)?.[1] ?? -1);
  if (seasonStart !== undefined) {
    const match = fields.find((field) => year(field.id) === seasonStart);
    if (match !== undefined) return match.id;
  }
  return fields.reduce<(typeof fields)[number] | undefined>(
    (best, field) => (best === undefined || year(field.id) > year(best.id) ? field : best),
    undefined
  )?.id;
}

/** The robot pose in a generic log: a Pose2d/Pose3d field, preferring names like Robot / Pose / Odometry. */
function findRobotPose(prefix: string, keys: string[]): { key: string; type: string } | null {
  const scored: { key: string; type: string; score: number }[] = [];
  for (const key of keys) {
    if (prefix !== "" && !key.startsWith(prefix + "/")) continue;
    const type = window.log.getStructuredType(key);
    if (type !== "Pose2d" && type !== "Pose3d") continue;
    const name = key.slice(prefix.length).toLowerCase();
    let score = type === "Pose2d" ? 1 : 0;
    if (/robot(pose)?$|\/pose$|odometry|estimated|fieldpose/.test(name)) score += 4;
    if (/robot/.test(name)) score += 2;
    if (/belief|target|ghost|setpoint|vision|camera|tag/.test(name)) score -= 3;
    scored.push({ key, type, score });
  }
  scored.sort((a, b) => b.score - a.score || a.key.length - b.key.length);
  return scored.length > 0 ? { key: scored[0].key, type: scored[0].type } : null;
}

type Source = { type: string; logKey: string; logType: string; visible: boolean; options: Record<string, string> };

/** 3D tab state for one run. */
async function stateForRun(file: string, prefix: string, keys: string[]): Promise<unknown> {
  const P = (key: string) => applyKeyPrefix(prefix, key);
  const isBioBuzzSim = keys.includes(P("RealOutputs/Field/Robot")) && keys.includes(P("RealOutputs/Field/HiveRed"));
  const teamCad = (window.assets?.robots ?? []).some((robot) => robot.name === TEAM_CAD);

  if (isBioBuzzSim) {
    try {
      const layout = await (await fetch(SIM_LAYOUT_URL)).json();
      const tabs: TabsState["tabs"] = layout.hubs[0].state.tabs.tabs;
      const field3d = tabs.find((tab) => tab.type === TabType.Field3d);
      const controller = JSON.parse(JSON.stringify(field3d!.controller)) as { sources: Source[]; game?: string };
      controller.sources.forEach((source) => {
        source.logKey = P(source.logKey);
        if ((source.type === "robot" || source.type === "ghost") && source.options.model === TEAM_CAD && !teamCad) {
          delete source.options.model; // the 3D tab picks a stock robot
        }
      });
      controller.game = fieldExists(TIPPING_FIELD_ID) ? TIPPING_FIELD_ID : ftcField(2026);
      return controller;
    } catch (error) {
      console.error("Could not load the BioBuzz sim layout", error);
    }
  }

  // Any other log: its robot pose on its season's field
  const sources: Source[] = [];
  const pose = findRobotPose(prefix, keys);
  if (pose !== null) {
    sources.push({
      type: "robot",
      logKey: pose.key,
      logType: pose.type,
      visible: true,
      options: teamCad ? { model: TEAM_CAD } : {}
    });
  }
  const text = (file + " " + keys.slice(0, 400).join(" ")).toLowerCase();
  const season = /decode|artifact|2025-?26/.test(text)
    ? 2025
    : /biobuzz|pollen|hive|2026-?27/.test(text)
    ? 2026
    : undefined;
  const game = ftcField(season);
  return { sources: sources, ...(game !== undefined ? { game } : {}) };
}

/** A Multi Field tab that is safe to replace: one made by Compare Runs, or one with nothing in it. */
function isReplaceable(controller: unknown): boolean {
  const panes = (controller as MultiFieldState | null)?.panes ?? [];
  return panes.every((pane) => {
    const sources = (pane.field3d as { sources?: { logKey: string }[] } | null)?.sources ?? [];
    return sources.length === 0 || sources.every((source) => /^\/Log\d+\//.test(source.logKey));
  });
}

/** When a run starts: the auto's START if logged, else the start of that run's data. */
function runStart(prefix: string): number | null {
  const leg = applyKeyPrefix(prefix, "RealOutputs/Auto/leg");
  if (window.log.getType(leg) === LoggableType.String) {
    const legs = window.log.getString(leg, -Infinity, Infinity);
    const index = legs?.values.findIndex((value) => value === "START") ?? -1;
    if (legs !== undefined && index >= 0) return legs.timestamps[index];
  }
  return null;
}

/**
 * Called while the logs decode; once every run's fields are known, shows one field per run.
 */
export async function maybeSetUpCompare(): Promise<void> {
  const keys = window.log.getFieldKeys();
  if (pendingFields.length > 0) await fillQueuedFields(keys);
  if (pending === null) return;
  const files = pending.files;
  const targetUuid = pending.targetUuid;
  const prefixes = files.length > 1 ? files.map((_, i) => "/" + MERGE_PREFIX + i.toString()) : [""];
  if (!prefixes.every((prefix) => (prefix === "" ? keys.length > 0 : keys.some((k) => k.startsWith(prefix + "/"))))) {
    return; // some run is still loading
  }
  pending = null;

  // One Multi Field tab, one pane per run. It replaces an earlier comparison's tab (and any empty
  // Multi Field tab); other tabs are kept.
  const panes: MultiFieldPaneState[] = [];
  for (let i = 0; i < files.length; i++) {
    panes.push({ title: baseName(files[i]), field3d: await stateForRun(files[i], prefixes[i], keys) });
  }
  const state = window.tabs.saveState();
  const target = targetUuid === null ? -1 : state.tabs.findIndex((tab) => tab.controllerUUID === targetUuid);
  if (target >= 0) {
    // The tab the runs were chosen from shows exactly the chosen runs
    (state.tabs[target].controller as MultiFieldState).panes = panes;
    window.tabs.restoreState({ selected: target, tabs: state.tabs });
    playFromStart(prefixes);
    return;
  }
  const kept = state.tabs.filter((tab) => tab.type !== TabType.MultiField || !isReplaceable(tab.controller));
  kept.push({
    type: TabType.MultiField,
    title: "Compare",
    controller: { panes: panes } as MultiFieldState,
    controllerUUID: createUUID(),
    renderer: null,
    controlsHeight: 96
  } as TabsState["tabs"][number]);
  window.tabs.restoreState({ selected: kept.length - 1, tabs: kept });
  playFromStart(prefixes);
}

/** Start at the first run's auto START and play. */
function playFromStart(prefixes: string[]) {
  const starts = prefixes.map((prefix) => runStart(prefix)).filter((t): t is number => t !== null);
  window.selection.setSelectedTime(starts.length > 0 ? Math.min(...starts) : window.log.getTimestampRange()[0]);
  window.selection.play();
}
