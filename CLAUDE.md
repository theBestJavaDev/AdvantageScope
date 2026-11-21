# CLAUDE.md - SolversScope Developer Guide

Welcome to **SolversScope**, custom-developed for **FTC Team 23511 Seattle Solvers**. SolversScope is a high-performance robot diagnostics, log review/analysis, and 3D/2D field data visualization application built on Electron, TypeScript, and Three.js.

---

## 🚀 Common Commands

### Development
```bash
# Ensure Node.js is in PATH (if using NVM on macOS):
export PATH="/System/Volumes/Data/Users/brandon/.nvm/versions/node/v26.13.1/bin:$PATH"

# Install dependencies
npm install

# Compile TypeScript code (using Rolldown)
npm run compile

# Launch SolversScope in development mode
npm start
```

### Packaging & Distribution
```bash
# Build standalone installers (Mac DMG, Windows EXE, Linux AppImage):
npx electron-builder build --mac dmg
npx electron-builder build --win nsis --x64
```

---

## 📁 Repository Structure & Key Architecture

- **`src/main/electron/main.ts`**: Electron main process entry point, window management (`SolversScope`), native menus, and file dialogs.
- **`src/main/electron/assetLoader.ts`**: Assets loading manager (loads 3D fields, robot CAD models, joysticks from bundled assets and user asset folders).
- **`src/hub/hub.ts`**: Hub UI controller for tabs, sidebar, timeline playback, and state tracking.
- **`src/shared/renderers/Field3dRendererImpl.ts`**: Three.js 3D Field visualizer with PBR materials, soft shadow maps, orbit camera controls, and robot/game piece managers.
- **`src/shared/renderers/Field2dRenderer.ts`**: Canvas-based 2D Field visualizer with zoom/pan, pose tracking, heatmaps, and swerve module velocity vectors.
- **`src/shared/physics/PhysicsEngine.ts`**: 6-DoF rigid body mechanics engine with RK4 integration, aerodynamic drag, Magnus spin force, and non-linear impulse bouncing.
- **`src/shared/physics/TeleopController.ts`**: WASD translation and QE turning teleop driver controller for real-time code testing. (Not used by the app; the Drive Sim tab has its own input handling.)
- **`src/shared/physics/ShotLaunch.ts` / `ShotErrorModel.ts` / `ShotLab.ts`**: shooter launch state + spin modes, the 13-source shot error model with Monte Carlo + error budget, and the Shot Lab runner/log writer (`/SolversSim/ShotLab`).
- **`src/shared/physics/DriveSim.ts` / `DriveShooting.ts`**: drivetrain physics (mecanum/swerve/tank wheel limits, traction, battery sag + drain, walls, HIVE-leg collisions) and shooting on the move. Drive Sim's "Live 3D view" records and drives the timeline like a live robot (`window.selection.setLiveConnected`) and opens a 3D Field in a satellite window (`Tabs.addTabInSatellite`).
- **`src/shared/physics/ShotCalibration.ts`**: fits restitution (drop test) and drag coefficient (measured shots) for the Shot Lab "Calibrate game piece" section; fits are stored per piece in `ShotLabConfig.calibration`.
- **CELL mouths** (`GamePresets.cellMouth`): a BIOBUZZ CELL is scored only through its tilted mouth (geometry from the robot repo's Constants: pivot 43.95 in, 30° tilt). `PhysicsEngine` counts a hit when the piece crosses the mouth plane inward inside its rectangle; `ShotLab.placeRobotForCell` keeps the robot on the mouth side.
- **Auto Check tab** (`src/shared/physics/AutoShotCheck.ts`, `src/hub/autoShotExtract.ts`, `AutoCheckController`/`AutoCheckRenderer`): when a robot-sim log opens, it pulls each launch out of the log (`RealOutputs/Sim/Launch/count` + `Field/Pollen` tracks) and re-flies it in SolversScope physics three ways: no spin, with Magnus spin, and Monte Carlo with every Shot Lab error source turned on. Results go to `/SolversSim/AutoCheck/*` and the Telemetry .md report.
- **Sim/telemetry tabs**: `src/hub/controllers/{ShotLab,DriveSim,Telemetry}Controller.ts` + `src/shared/renderers/{ShotLab,DriveSim,Telemetry}Renderer.ts`; shared settings panel in `src/hub/controllers/SimForm.ts`; Monte Carlo runs off the UI thread in `src/hub/shotLabWorker.ts`.
- **`src/shared/physics/GamePresets.ts`**: season presets. `BIOBUZZ` (2026-27) holds HIVE CELLs, FLOWERS, zones, frame legs and pieces from Competition Manual TU10. Sim frame: center origin, +X toward the blue wall, +Y away from the audience. Logged positions are converted to FTC's traditional frame (`simToLogTranslation`) so they line up on the official `Field2d/Field3d_20262027FTCFieldV1` assets. Those come from FIRST's field CAD and are not committed: `npm install` downloads them into `lite/static/bundledAssets/`, which `assetLoader.ts` also reads (`DOWNLOADED_LITE_ASSETS`).
- **`www/solvers-theme.css`**: the SolversScope theme layer (color tokens for light/dark, loaded after the upstream CSS). Change the accent here to re-brand; canvas/WebGL renderers read the same tokens via `src/shared/themeColors.ts`.
- **`icons/`**: Application icons and branding assets for FTC Team 23511 Seattle Solvers.

---

## ⚙️ Key Conventions & Coding Standards

1. **Standalone & Offline Privacy**:
   - SolversScope runs in standalone offline mode. Third-party telemetry, GitHub update polling, and external tracking calls are disabled. All robot data (NT4, Phoenix, RLOG, FTC Dashboard) runs locally.

2. **TypeScript & Bundling**:
   - Code is written in modern TypeScript (`strict` mode).
   - Bundled using **Rolldown** (`npm run compile`).

3. **File Licensing**:
   - Standard Littleton Robotics & FTC Team 23511 Seattle Solvers header notice applies to source files.

---

## 🤖 Running the team's robot code (BioBuzz-2026 repo, SimLoop)

The robot repo's SimLoop sim runs the real OpModes on a simulated field and writes RLOGs that SolversScope opens:

```bash
# In the BioBuzz-2026 repo (v1.5 branch): runs the real AutoMax / AutoMaxBlue for the 30 s AUTO period
./gradlew.bat :TeamCode:simAuto -Pauto=AutoMax       # or -Pauto=AutoMaxBlue
```

With SolversScope open, the log opens in it automatically (the runner writes `%TEMP%/ascope-log-path.txt`, which
SolversScope watches). Per-computer setup:
- The team robot model `Robot_BioBuzzCad` (team CAD) is bundled in `bundledAssets/`.
- The tipping-HIVE field (`Field3d_BioBuzzSim`, `Robot_BioBuzzHiveRed/Blue`) comes from the robot repo's
  `sim/advantagescope/make_field_model.py` and lives only in `%APPDATA%/SolversScope/userAssets` (FIRST CAD: never commit it).
- No layout import needed: when a BioBuzz sim log opens and the field tabs are empty, `src/hub/simLayout.ts` adds
  the team's sim tabs (`www/layouts/biobuzz-sim.json`, a copy of the robot repo's `sim/advantagescope/layout.json`;
  re-copy it when that file changes) and plays the run from the auto's START.

---

## 🎨 Seattle Solvers branding & telemetry reports

- **Look:** `www/solvers-theme.css` uses seattlesolvers.com's design tokens (gold `#f2ce00` on `#0b0a06` / cream `#fcf9de`,
  square corners) and its fonts, bundled in `www/fonts/` (Orbitron, Inter, Azeret Mono; SIL OFL). Logo:
  `www/symbols/solvers-logo.svg`. The 2D sim canvases use the same palette (alliance red/blue kept).
- **Telemetry reports:** Telemetry tab → ● Record (marks a window) → ⬇ Download .md. Built by `src/hub/telemetryReport.ts`,
  saved through the `export-telemetry-report` main-process message.
- **Plain mode:** View → Plain AdvantageScope Mode (`plainMode` pref) hides the SolversScope tabs from New Tab, closes
  open ones, and skips the Auto Check tab on sim logs (`SOLVERS_TAB_TYPES` in `TabType.ts`, `removeSolversTabs` in `simLayout.ts`).
- **Random match:** the robot-code sim is a perfect robot, so Auto Check also flies each logged shot once with a fresh
  draw of the error model (`simulateRandomMatch`) and writes the misses to `/SolversSim/AutoCheck/Match/*`
  (POLLEN positions + 4 red trail slots); `simLayout.addMatchMissesSource` adds them to the sim 3D view. "🎲 New
  random match" rerolls. Don't draw misses as red balls: BioBuzz has red NECTAR.
- **Auto Check tuning:** each shot also gets where it crosses the CELL mouth (with spin) and the exit speed that
  centers it (`centeringExitSpeed`), with its hit chance under match error ("Tune" column, "With tuned speeds" card).
- **3D backdrop:** `src/shared/renderers/field3d/SolversArena.ts` (dome, lit floor, gold grid, fog in the theme
  colors). Mix colors in sRGB there (`mix`): lerping `THREE.Color` in linear space turns dark tints gray/olive.
- **Multi Field tab / Compare Runs:** up to 3 3D fields side by side in one tab (`MultiFieldController` /
  `MultiFieldRenderer`; each pane wraps a normal `Field3dController` / `Field3dRenderer`, built from the
  `controller3` / `renderer3` templates). File → Compare Runs (Ctrl+Alt+O) or the tab's button loads up to 3 logs
  merged under `/Log0..2` (aligned at the auto start) and builds one pane per run (`src/hub/compareRuns.ts`).
  `sendAllPreferences` finds File/App menu items by label, so new menu items are safe.
  "Choose run…" / "Choose runs…" open an in-app run picker (`src/hub/runPicker.ts`, folder listing in
  `src/main/electron/runsFolder.ts`): checkboxes, up to 3, starts in the last opened log's folder or a
  `<home>/*/TeamCode/build/sim`. One run fills that field (`load-run-into-field` → `queueRunForField`); 2–3 make
  the tab show exactly those (`load-runs-into-tab` → `beginCompare(files, uuid)`). Both go through
  `deliverChosenRuns` in main.ts. Pose lists hide behind "Details".
- **User guide:** `www/guide/guide.md` (the one copy; plain language for new members), shown in the app by
  `www/guide/index.html` (Help → How to Use SolversScope, F1; `open-guide` message with a section id, e.g. the
  Multi Field "?" opens `#compare`). Update it when a feature's buttons or wording change. `HOW_TO_USE.md` points to it.
- **Keep it simple for new members:** prefer buttons that do the whole job (Choose run…, Compare Runs) over
  drag-and-drop, plain-language status text, and views that show results on open (Shot Lab runs once on open).
- **Theme:** black and gold by default. "Match System" (the old default) also shows dark; only Preferences → Light
  gives the light theme (`solversThemeSource` in `main.ts`).
- **Robot CAD:** when a sim log's robot is on a generic model (stock Drive Base/KitBot) or a missing one,
  `simLayout.ts` switches to an installed team CAD (BioBuzz CAD first). With none installed, it asks once per session
  (`src/main/electron/robotCad.ts`): the chosen .glb is copied into userAssets as a robot. "Don't ask again" is
  saved in the `skipRobotCadPrompt` pref.
- **Physics checks:** `npm run test:sim` (`test/sim/simTests.ts`). Run it after changing anything in `src/shared/physics/`.
- **Solid HIVE:** `cellMouth` attaches `colliders` (the tipped bar's trays, open only at the mouths, plus the A-frame legs
  and axle; `hiveColliders` / `frameColliders` in `GamePresets.ts`). `PhysicsEngine.simulate` bounces pieces off the
  target's colliders (contact surface `"hive"`). The aim solvers (`measureMiss`) fly without them on purpose. Only the
  targeted HIVE's trays are solid; the frame tube sizes are estimates.
- **Cinematic 3D:** Brandon's tuning in the `Field3dRendererImpl` constructor (ACES at exposure 0.65, environment
  0.25, FTC spot lights 20). Check it on Windows too: brighter ACES settings washed the floor white there.
- **Startup:** reopens the last log (hub sends `remember-last-run` from `startHistorical`, saved in `last-run.txt`
  in userData; skipped in Plain mode or when a file was passed). Only the main window opens. Dev tools only with `SOLVERS_DEVTOOLS=1`; satellites aren't restored.
