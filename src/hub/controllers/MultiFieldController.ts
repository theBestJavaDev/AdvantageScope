// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { MultiFieldRendererCommand } from "../../shared/renderers/MultiFieldRenderer";
import { createUUID } from "../../shared/util";
import Field3dController from "./Field3dController";
import TabController from "./TabController";

/** At most this many fields in one Multi Field tab. */
export const MAX_FIELDS = 3;

export type MultiFieldPaneState = { title: string; field3d: unknown };
export type MultiFieldState = { panes: MultiFieldPaneState[] };

/**
 * Multi Field tab: up to three 3D fields side by side in one view, all on the same timeline. Each
 * pane is a full 3D Field (its own sources, field, robot model and camera), so it can show a
 * different run (e.g. /Log0 and /Log1 after Compare Runs or "Add New Log(s)") or a different season.
 */
export default class MultiFieldController implements TabController {
  UUID = createUUID();

  private PANES: HTMLElement;
  private ADD_BUTTON: HTMLButtonElement;
  private panes: {
    controller: Field3dController;
    element: HTMLElement;
    titleInput: HTMLInputElement;
    detailsOpen: boolean;
  }[] = [];

  constructor(root: HTMLElement) {
    root.classList.add("multifield-controls");
    const toolbar = document.createElement("div");
    toolbar.className = "multifield-toolbar";
    this.ADD_BUTTON = document.createElement("button");
    this.ADD_BUTTON.className = "multifield-button";
    this.ADD_BUTTON.textContent = "+ Add field";
    this.ADD_BUTTON.title = "Add another field (up to " + MAX_FIELDS + ")";
    this.ADD_BUTTON.addEventListener("click", () => this.addPane());
    const compare = document.createElement("button");
    compare.className = "multifield-button";
    compare.textContent = "Load 2–3 runs at once…";
    compare.title = "Pick two or three log files: each gets its own field";
    compare.addEventListener("click", () => window.sendMainMessage("open-compare-runs"));
    const hint = document.createElement("span");
    hint.className = "multifield-hint";
    hint.textContent = "Press “Choose run…” and pick one log, or up to 3 to show them all side by side.";
    const help = document.createElement("button");
    help.className = "multifield-help";
    help.textContent = "?";
    help.title = "How to compare runs (opens the guide)";
    help.addEventListener("click", () => window.sendMainMessage("open-guide", "compare"));
    toolbar.append(this.ADD_BUTTON, compare, hint, help);
    this.PANES = document.createElement("div");
    this.PANES.className = "multifield-panes";
    root.append(toolbar, this.PANES);
    this.addPane();
  }

  /** Adds a field pane (no-op at the limit). */
  addPane(title?: string, field3dState?: unknown) {
    if (this.panes.length >= MAX_FIELDS) return;
    const template = document.getElementById("controller3");
    if (template === null) return;
    const controls = template.cloneNode(true) as HTMLElement;
    controls.removeAttribute("id");
    controls.classList.add("multifield-pane-controls");

    const element = document.createElement("div");
    element.className = "multifield-pane";
    const header = document.createElement("div");
    header.className = "multifield-pane-header";
    const titleInput = document.createElement("input");
    titleInput.className = "multifield-pane-title";
    titleInput.value = title ?? "Field " + (this.panes.length + 1);
    titleInput.title = "Name shown on this field";
    const choose = document.createElement("button");
    choose.className = "multifield-button multifield-choose";
    choose.textContent = "Choose run…";
    choose.title = "Pick one log for this field, or up to 3 logs to show them all side by side";
    const details = document.createElement("button");
    details.className = "multifield-details";
    details.textContent = "Details ▸";
    details.title = "Show what this field draws and which field model it uses";
    const remove = document.createElement("button");
    remove.className = "multifield-remove";
    remove.textContent = "✕";
    remove.title = "Remove this field";
    header.append(titleInput, choose, details, remove);
    element.append(header, controls);
    this.PANES.appendChild(element);

    const controller = new Field3dController(controls);
    if (field3dState !== undefined) controller.restoreState(field3dState);
    const pane = { controller, element, titleInput, detailsOpen: false };
    this.panes.push(pane);
    const showDetails = (open: boolean) => {
      pane.detailsOpen = open;
      controls.hidden = !open;
      details.textContent = open ? "Details ▾" : "Details ▸";
      if (open) window.tabs?.ensureControlsHeight(this.UUID, 260);
    };
    showDetails(false);
    details.addEventListener("click", () => showDetails(!pane.detailsOpen));
    choose.addEventListener("click", () =>
      window.sendMainMessage("choose-run-for-field", { uuid: this.UUID, pane: this.panes.indexOf(pane) })
    );
    remove.addEventListener("click", () => this.removePane(this.panes.indexOf(pane)));
    this.updateButtons();
  }

  private removePane(index: number) {
    if (index < 0 || this.panes.length <= 1) return;
    this.panes[index].element.remove();
    this.panes.splice(index, 1);
    this.updateButtons();
  }

  private updateButtons() {
    this.ADD_BUTTON.disabled = this.panes.length >= MAX_FIELDS;
    this.PANES.style.gridTemplateColumns = "repeat(" + this.panes.length + ", minmax(0, 1fr))";
    this.panes.forEach((pane) => {
      (pane.element.getElementsByClassName("multifield-remove")[0] as HTMLButtonElement).disabled =
        this.panes.length <= 1;
    });
  }

  saveState(): MultiFieldState {
    return {
      panes: this.panes.map((pane) => ({ title: pane.titleInput.value, field3d: pane.controller.saveState() }))
    };
  }

  restoreState(state: unknown): void {
    const panes = (state as MultiFieldState | null)?.panes;
    if (!Array.isArray(panes) || panes.length === 0) return;
    this.panes.forEach((pane) => pane.element.remove());
    this.panes = [];
    panes.slice(0, MAX_FIELDS).forEach((pane) => this.addPane(pane.title, pane.field3d));
  }

  refresh(): void {
    this.panes.forEach((pane) => pane.controller.refresh());
  }

  newAssets(): void {
    this.panes.forEach((pane) => pane.controller.newAssets());
  }

  getActiveFields(): string[] {
    return this.panes.flatMap((pane) => pane.controller.getActiveFields());
  }

  showTimeline(): boolean {
    return true;
  }

  getCommand(): MultiFieldRendererCommand {
    return {
      panes: this.panes.map((pane) => ({
        title: pane.titleInput.value,
        empty: ((pane.controller.saveState() as { sources?: unknown[] }).sources ?? []).length === 0,
        command: pane.controller.getCommand()
      }))
    };
  }
}
