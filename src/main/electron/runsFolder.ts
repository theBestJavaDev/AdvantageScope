// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Lists the runs (log files) in a folder for the Multi Field run picker (src/hub/runPicker.ts), so
 * people tick checkboxes instead of Ctrl+clicking in the system file window.
 */

import { app } from "electron";
import fs from "fs";
import path from "path";

export type RunFile = { name: string; path: string; modified: number; sizeKb: number };
export type RunsList = { folder: string | null; runs: RunFile[] };

const RUN_EXTENSIONS = [".rlog", ".wpilog", ".wpilogxz", ".hoot", ".revlog", ".csv"];

/**
 * A robot project's sim runs folder under the user's home (`<project>/TeamCode/build/sim`), for when
 * no folder is known yet. The most recently changed one wins.
 */
export function findDefaultRunsFolder(): string | null {
  const home = app.getPath("home");
  const found: { folder: string; modified: number }[] = [];
  let entries: fs.Dirent[] = [];
  try {
    entries = fs.readdirSync(home, { withFileTypes: true });
  } catch {
    return null;
  }
  for (const entry of entries) {
    if (!entry.isDirectory() || entry.name.startsWith(".")) continue;
    const folder = path.join(home, entry.name, "TeamCode", "build", "sim");
    try {
      const stat = fs.statSync(folder);
      if (stat.isDirectory()) found.push({ folder, modified: stat.mtimeMs });
    } catch {
      // not a robot project
    }
  }
  found.sort((a, b) => b.modified - a.modified);
  return found.length > 0 ? found[0].folder : null;
}

/** The runs in `folder` (newest first), or in the default folder when none is given or it is gone. */
export function listRuns(folder: string | null | undefined): RunsList {
  let dir = folder ?? null;
  if (dir === null || !fs.existsSync(dir)) dir = findDefaultRunsFolder();
  if (dir === null) return { folder: null, runs: [] };
  const runs: RunFile[] = [];
  try {
    for (const name of fs.readdirSync(dir)) {
      if (!RUN_EXTENSIONS.includes(path.extname(name).toLowerCase())) continue;
      const full = path.join(dir, name);
      try {
        const stat = fs.statSync(full);
        if (stat.isFile())
          runs.push({ name, path: full, modified: stat.mtimeMs, sizeKb: Math.round(stat.size / 1024) });
      } catch {
        // skip unreadable files
      }
    }
  } catch {
    return { folder: dir, runs: [] };
  }
  runs.sort((a, b) => b.modified - a.modified);
  return { folder: dir, runs };
}
