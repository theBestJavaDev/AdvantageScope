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
- **`src/shared/physics/TeleopController.ts`**: WASD translation and QE turning teleop driver controller for real-time code testing.
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
