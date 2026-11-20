// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import LoggableType from "../shared/log/LoggableType";

/**
 * Builds the Telemetry tab's downloadable Markdown report for a time window of the current log:
 * a session header, a stats table per numeric field, on/off time for booleans, a timeline of text
 * changes (OpMode state, auto legs, status lines) and the Driver Station screen at the end.
 */

export interface ReportField {
  key: string;
  label: string;
}

export interface ReportInput {
  logName: string;
  sourceLabel: string;
  fields: ReportField[];
  start: number;
  end: number;
  /** "recorded" when the window came from Record, else "full log". */
  windowKind: "recorded" | "full log";
  /** Driver Station lines at the end of the window (already formatted). */
  finalLines: { label: string; value: string }[];
  freeLines: string[];
  opMode: string | null;
  states: string[];
  battery: { start: number; min: number; end: number } | null;
}

const MAX_EVENTS = 300;
const BOOLEAN_SAMPLES = 400;

function md(text: string): string {
  // Escape what would break a table cell or start formatting
  return text
    .replace(/\|/g, "\\|")
    .replace(/\r?\n/g, " ")
    .replace(/([*_`])/g, "\\$1");
}

function num(value: number): string {
  if (!isFinite(value)) return "—";
  const abs = Math.abs(value);
  if (abs >= 10000) return value.toFixed(0);
  if (abs >= 100) return value.toFixed(1);
  if (abs >= 1) return value.toFixed(3);
  if (abs === 0) return "0";
  return value.toPrecision(3);
}

function rel(t: number, start: number): string {
  return (t - start).toFixed(2) + " s";
}

function valueAt<T>(set: { timestamps: number[]; values: T[] } | undefined, t: number): T | null {
  if (set === undefined || set.values.length === 0) return null;
  let index = -1;
  for (let i = 0; i < set.timestamps.length && set.timestamps[i] <= t; i++) index = i;
  return index >= 0 ? set.values[index] : null;
}

export function buildTelemetryReport(input: ReportInput, generatedAt: Date = new Date()): string {
  const { start, end } = input;
  const duration = Math.max(0, end - start);
  const out: string[] = [];

  out.push("# Telemetry report");
  out.push("");
  out.push("_SolversScope · FTC Team 23511 Seattle Solvers_");
  out.push("");
  out.push("| | |");
  out.push("|---|---|");
  out.push("| Log | " + md(input.logName) + " |");
  out.push("| Source | " + md(input.sourceLabel) + " |");
  out.push(
    "| Window | " +
      input.windowKind +
      ": " +
      start.toFixed(2) +
      " s to " +
      end.toFixed(2) +
      " s (" +
      duration.toFixed(2) +
      " s) |"
  );
  if (input.opMode !== null) out.push("| OpMode | " + md(input.opMode) + " |");
  if (input.states.length > 0) out.push("| States seen | " + md(input.states.join(" → ")) + " |");
  if (input.battery !== null) {
    out.push(
      "| Battery | " +
        input.battery.start.toFixed(2) +
        " V at start, " +
        input.battery.min.toFixed(2) +
        " V lowest, " +
        input.battery.end.toFixed(2) +
        " V at end |"
    );
  }
  out.push("| Generated | " + generatedAt.toLocaleString() + " |");
  out.push("");

  const numeric: string[] = [];
  const booleans: string[] = [];
  const events: { t: number; label: string; value: string }[] = [];

  for (const field of input.fields) {
    const type = window.log.getType(field.key);
    if (type === LoggableType.Number) {
      const set = window.log.getNumber(field.key, start, end);
      if (set === undefined || set.values.length === 0) continue;
      const inWindow = set.values.filter((_, i) => set.timestamps[i] >= start && set.timestamps[i] <= end);
      const values = inWindow.length > 0 ? inWindow : [set.values[0]];
      const finite = values.filter((v) => isFinite(v));
      if (finite.length === 0) continue;
      const first = valueAt(set, start) ?? finite[0];
      const last = valueAt(set, end) ?? finite[finite.length - 1];
      const mean = finite.reduce((a, b) => a + b, 0) / finite.length;
      numeric.push(
        "| " +
          md(field.label) +
          " | " +
          num(first) +
          " | " +
          num(last) +
          " | " +
          num(Math.min(...finite)) +
          " | " +
          num(mean) +
          " | " +
          num(Math.max(...finite)) +
          " | " +
          num(last - first) +
          " |"
      );
    } else if (type === LoggableType.Boolean) {
      const set = window.log.getBoolean(field.key, start, end);
      if (set === undefined || set.values.length === 0) continue;
      let trueCount = 0;
      for (let i = 0; i < BOOLEAN_SAMPLES; i++) {
        const t = start + (duration * (i + 0.5)) / BOOLEAN_SAMPLES;
        if (valueAt(set, t) === true) trueCount++;
      }
      let changes = 0;
      for (let i = 1; i < set.values.length; i++) {
        if (set.values[i] !== set.values[i - 1] && set.timestamps[i] >= start && set.timestamps[i] <= end) changes++;
      }
      const final = valueAt(set, end);
      booleans.push(
        "| " +
          md(field.label) +
          " | " +
          Math.round((trueCount / BOOLEAN_SAMPLES) * 100) +
          "% | " +
          changes +
          " | " +
          (final === null ? "—" : final ? "true" : "false") +
          " |"
      );
    } else if (type === LoggableType.String) {
      const set = window.log.getString(field.key, start, end);
      if (set === undefined) continue;
      let previous: string | null = null;
      set.values.forEach((value, i) => {
        const t = set.timestamps[i];
        if (value !== previous && t >= start && t <= end) {
          events.push({ t, label: field.label, value });
        }
        previous = value;
      });
    }
  }

  out.push("## Numbers");
  out.push("");
  if (numeric.length === 0) {
    out.push("_No numeric fields in this window._");
  } else {
    out.push("| Field | Start | End | Min | Avg | Max | Change |");
    out.push("|---|---:|---:|---:|---:|---:|---:|");
    out.push(...numeric);
  }
  out.push("");

  if (booleans.length > 0) {
    out.push("## On / off");
    out.push("");
    out.push("| Field | Time true | Switches | At end |");
    out.push("|---|---:|---:|---|");
    out.push(...booleans);
    out.push("");
  }

  out.push("## Events");
  out.push("");
  if (events.length === 0) {
    out.push("_No text values changed in this window._");
  } else {
    events.sort((a, b) => a.t - b.t);
    out.push("| Time | Field | Value |");
    out.push("|---:|---|---|");
    events
      .slice(0, MAX_EVENTS)
      .forEach((e) => out.push("| " + rel(e.t, start) + " | " + md(e.label) + " | " + md(e.value) + " |"));
    if (events.length > MAX_EVENTS) out.push("", "_" + (events.length - MAX_EVENTS) + " more events not shown._");
  }
  out.push("");

  out.push("## Driver Station at the end");
  out.push("");
  if (input.finalLines.length === 0 && input.freeLines.length === 0) {
    out.push("_No telemetry at the end of the window._");
  } else {
    out.push("```");
    const width = Math.min(40, Math.max(0, ...input.finalLines.map((l) => l.label.length)));
    input.finalLines.forEach((l) => out.push(l.label.padEnd(width) + " : " + l.value));
    input.freeLines.forEach((line) => out.push(line));
    out.push("```");
  }
  out.push("");
  return out.join("\n");
}
