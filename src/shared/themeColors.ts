// Copyright (c) 2021-2026 Littleton Robotics & FTC Team 23511 Seattle Solvers
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

/**
 * Reads SolversScope theme colors (CSS custom properties from www/solvers-theme.css) so canvas
 * and WebGL renderers use the same background as the rest of the app. Falls back when the theme
 * stylesheet isn't loaded (e.g. the XR client).
 */

const cache = new Map<string, string>();
let cacheDark: boolean | null = null;

export function themeColor(name: string, fallback: string): string {
  if (typeof window === "undefined" || typeof document === "undefined") return fallback;
  const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
  if (dark !== cacheDark) {
    cache.clear();
    cacheDark = dark;
  }
  const cached = cache.get(name);
  if (cached !== undefined) return cached;
  let value = fallback;
  try {
    const raw = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    if (raw.length > 0) value = raw;
  } catch {}
  cache.set(name, value);
  return value;
}
