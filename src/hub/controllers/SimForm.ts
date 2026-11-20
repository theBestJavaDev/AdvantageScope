// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import { ShotLabConfig, defaultShotLabConfig, mergeShotLabConfig } from "../../shared/physics/ShotLab";

/** One input in a simulation settings panel, bound to a dotted path in the config object. */
export type FieldSpec =
  | {
      kind: "number";
      path: string;
      label: string;
      unit: string;
      step: number;
      min?: number;
      max?: number;
      hint?: string;
    }
  | { kind: "checkbox"; path: string; label: string; hint?: string }
  | { kind: "select"; path: string; label: string; options: { value: string; label: string }[]; hint?: string };

export type FormSection = { title: string; fields: FieldSpec[]; collapsed?: boolean };

export function getPath(obj: any, path: string): any {
  return path.split(".").reduce((o, k) => (o === undefined ? undefined : o[k]), obj);
}

export function setPath(obj: any, path: string, value: any) {
  const keys = path.split(".");
  const last = keys.pop()!;
  keys.reduce((o, k) => o[k], obj)[last] = value;
}

/**
 * Builds the shared simulation settings panel (sticky action buttons + collapsible sections).
 * Used by the Shot Lab and Drive Sim tabs so they look and behave the same.
 */
export class SimForm {
  private container: HTMLElement;
  private actions: HTMLElement;
  private buttons: HTMLButtonElement[] = [];
  private inputs: { spec: FieldSpec; element: HTMLInputElement | HTMLSelectElement }[] = [];

  constructor(
    container: HTMLElement,
    sections: FormSection[],
    private getConfig: () => any,
    private onChange: (spec: FieldSpec) => void
  ) {
    this.container = container;
    this.container.replaceChildren();
    this.actions = document.createElement("div");
    this.actions.classList.add("shotlab-actions");
    this.container.appendChild(this.actions);
    sections.forEach((section) => this.addSection(section));
  }

  /** Adds a button to the sticky action bar at the top of the panel. */
  addAction(label: string, title: string, primary: boolean, onClick: () => void): HTMLButtonElement {
    const button = document.createElement("button");
    button.innerText = label;
    button.title = title;
    if (primary) button.classList.add("primary");
    button.addEventListener("click", onClick);
    this.actions.appendChild(button);
    this.buttons.push(button);
    return button;
  }

  /** Adds a free-form note under the action bar (e.g. key bindings). */
  addNote(element: HTMLElement) {
    element.classList.add("shotlab-note");
    this.actions.after(element);
  }

  /** Adds a button row at the end of the section with the given title. */
  addSectionAction(sectionTitle: string, label: string, title: string, onClick: () => void): HTMLButtonElement {
    const details = Array.from(this.container.querySelectorAll("details")).find(
      (d) => d.querySelector("summary")?.innerText === sectionTitle
    );
    if (details === undefined) throw new Error("No section " + sectionTitle);
    let row = details.querySelector("div.shotlab-section-actions") as HTMLElement | null;
    if (row === null) {
      row = document.createElement("div");
      row.classList.add("shotlab-actions", "shotlab-section-actions");
      details.appendChild(row);
    }
    const button = document.createElement("button");
    button.innerText = label;
    button.title = title;
    button.addEventListener("click", onClick);
    row.appendChild(button);
    this.buttons.push(button);
    return button;
  }

  setDisabled(disabled: boolean) {
    this.buttons.forEach((b) => (b.disabled = disabled));
  }

  private addSection(section: FormSection) {
    const details = document.createElement("details");
    details.open = !section.collapsed;
    const summary = document.createElement("summary");
    summary.innerText = section.title;
    details.appendChild(summary);

    for (const spec of section.fields) {
      const row = document.createElement("label");
      row.classList.add("shotlab-row", spec.kind);
      if (spec.hint) row.title = spec.hint;
      const text = document.createElement("span");
      text.innerText = spec.label;
      let element: HTMLInputElement | HTMLSelectElement;
      if (spec.kind === "select") {
        element = document.createElement("select");
        for (const option of spec.options) {
          const el = document.createElement("option");
          el.value = option.value;
          el.innerText = option.label;
          element.appendChild(el);
        }
        row.append(text, element);
      } else if (spec.kind === "checkbox") {
        element = document.createElement("input");
        element.type = "checkbox";
        row.append(element, text);
      } else {
        element = document.createElement("input");
        element.type = "number";
        element.step = spec.step.toString();
        if (spec.min !== undefined) element.min = spec.min.toString();
        if (spec.max !== undefined) element.max = spec.max.toString();
        const unit = document.createElement("span");
        unit.classList.add("unit");
        unit.innerText = spec.unit;
        row.append(text, element, unit);
      }
      element.addEventListener("change", () => this.readInput(spec, element));
      details.appendChild(row);
      this.inputs.push({ spec, element });
    }
    this.container.appendChild(details);
  }

  private readInput(spec: FieldSpec, element: HTMLInputElement | HTMLSelectElement) {
    const config = this.getConfig();
    if (spec.kind === "checkbox") {
      setPath(config, spec.path, (element as HTMLInputElement).checked);
    } else if (spec.kind === "select") {
      setPath(config, spec.path, element.value);
    } else {
      let value = Number(element.value);
      if (!isFinite(value) || element.value.trim() === "") {
        element.value = getPath(config, spec.path).toString();
        return;
      }
      if (spec.min !== undefined) value = Math.max(spec.min, value);
      if (spec.max !== undefined) value = Math.min(spec.max, value);
      setPath(config, spec.path, value);
      element.value = value.toString();
    }
    this.onChange(spec);
  }

  /** Copies the current config values into the inputs. */
  sync() {
    const config = this.getConfig();
    for (const { spec, element } of this.inputs) {
      const value = getPath(config, spec.path);
      if (spec.kind === "checkbox") {
        (element as HTMLInputElement).checked = Boolean(value);
      } else if (spec.kind === "number") {
        element.value = Number(Number(value).toFixed(4)).toString();
      } else {
        element.value = String(value);
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------
// Shared shooter setup: the Drive Sim shoots with whatever was last set up in a Shot Lab tab
// ---------------------------------------------------------------------------------------------

let sharedShotLabConfig: ShotLabConfig | null = null;

export function publishShotLabConfig(config: ShotLabConfig) {
  sharedShotLabConfig = config;
}

/** A copy of the most recently edited Shot Lab config (defaults if no Shot Lab tab exists). */
export function getSharedShotLabConfig(): ShotLabConfig {
  return sharedShotLabConfig === null
    ? defaultShotLabConfig()
    : mergeShotLabConfig(JSON.parse(JSON.stringify(sharedShotLabConfig)));
}
