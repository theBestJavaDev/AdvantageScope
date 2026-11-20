// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import Field3dRenderer, { Field3dRendererCommand } from "./Field3dRenderer";
import TabRenderer from "./TabRenderer";

export type MultiFieldRendererCommand = {
  /** `empty`: nothing assigned to this field yet (shows how to fill it). */
  panes: { title: string; empty: boolean; command: Field3dRendererCommand }[];
};

/** Up to three 3D fields side by side (Multi Field tab). Each pane is a normal 3D field view. */
export default class MultiFieldRenderer implements TabRenderer {
  private GRID: HTMLElement;
  private cells: { element: HTMLElement; label: HTMLElement; empty: HTMLElement; renderer: Field3dRenderer }[] = [];
  private restoreCache: unknown[] = [];

  constructor(root: HTMLElement) {
    this.GRID = document.createElement("div");
    this.GRID.className = "multifield-grid";
    root.appendChild(this.GRID);
  }

  saveState(): unknown {
    return { cameras: this.cells.map((cell) => cell.renderer.saveState()) };
  }

  restoreState(state: unknown): void {
    const cameras = (state as { cameras?: unknown[] } | null)?.cameras;
    if (!Array.isArray(cameras)) return;
    this.restoreCache = cameras;
    this.cells.forEach((cell, i) => {
      if (i < cameras.length && cameras[i] !== null) cell.renderer.restoreState(cameras[i]);
    });
  }

  getAspectRatio(): number | null {
    return null;
  }

  private addCell() {
    const template = document.getElementById("renderer3");
    if (template === null) return;
    const element = document.createElement("div");
    element.className = "multifield-cell";
    const view = template.cloneNode(true) as HTMLElement;
    view.removeAttribute("id");
    view.className = "multifield-view";
    const label = document.createElement("div");
    label.className = "multifield-cell-label";
    const empty = document.createElement("div");
    empty.className = "multifield-empty";
    const emptyTitle = document.createElement("div");
    emptyTitle.className = "multifield-empty-title";
    emptyTitle.textContent = "No run here yet";
    const emptyText = document.createElement("div");
    emptyText.textContent = "Press “Choose run…” under this field. Pick one log, or up to 3 to compare them.";
    empty.append(emptyTitle, emptyText);
    element.append(view, label, empty);
    this.GRID.appendChild(element);
    const renderer = new Field3dRenderer(view);
    const index = this.cells.length;
    if (index < this.restoreCache.length && this.restoreCache[index] !== null) {
      renderer.restoreState(this.restoreCache[index]);
    }
    this.cells.push({ element, label, empty, renderer });
  }

  private removeCell() {
    const cell = this.cells.pop();
    if (cell === undefined) return;
    // Free the WebGL context now rather than waiting for garbage collection
    const canvas = cell.element.getElementsByTagName("canvas")[0];
    const gl = canvas?.getContext("webgl2") ?? canvas?.getContext("webgl");
    gl?.getExtension("WEBGL_lose_context")?.loseContext();
    cell.element.remove();
  }

  render(command: MultiFieldRendererCommand): void {
    if (command === null || command === undefined || !Array.isArray(command.panes)) return;
    const count = command.panes.length;
    while (this.cells.length < count) this.addCell();
    while (this.cells.length > count) this.removeCell();
    this.GRID.style.gridTemplateColumns = "repeat(" + Math.max(1, count) + ", minmax(0, 1fr))";
    command.panes.forEach((pane, i) => {
      const cell = this.cells[i];
      if (cell === undefined) return;
      if (cell.label.textContent !== pane.title) cell.label.textContent = pane.title;
      cell.empty.hidden = !pane.empty;
      cell.renderer.render(pane.command);
    });
  }
}
