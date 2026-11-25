#!/usr/bin/env python3
"""Report figures from the Webots batch results (needs matplotlib).

Reads ``results/webots/runs.jsonl`` and the per-run CSVs and writes PNGs to
``results/webots/figures/``:

* ``success_by_scenario.png``   - mission success, baseline vs team (vs team_robust)
* ``rmse_by_scenario.png``      - position RMSE per trial (log scale)
* ``kidnap_recovery.png``       - estimate error vs time around the kidnap
* ``uncertain_start_convergence.png`` - estimate error during global localization
* ``extension_degradation.png`` - extension: success and time vs degradation level

Usage::

    python tools/plot_webots_summary.py
"""

from __future__ import annotations

import csv
import json
import statistics
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNS = PROJECT_ROOT / "results" / "webots" / "runs.jsonl"
FIGURES = PROJECT_ROOT / "results" / "webots" / "figures"

sys.path.insert(0, str(PROJECT_ROOT / "tools"))
from summarize_webots import load_rows  # noqa: E402

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit("matplotlib is required: python -m pip install matplotlib") from exc

SCENARIOS = ["nominal", "uncertain_start", "blocked_route", "kidnapped"]
SCENARIO_LABELS = ["Nominal", "Uncertain start", "Blocked route", "Kidnapped"]
METHODS = ["baseline", "team", "team_robust"]
METHOD_LABELS = {"baseline": "Baseline (odometry + fixed route)", "team": "Team (MCL + A*)", "team_robust": "Team + robust sensing"}
# Fixed categorical order (validated palette, first three slots).
COLORS = {"baseline": "#f47230", "team": "#257de0", "team_robust": "#00b894"}
TEXT = "#191919"
MUTED = "#666666"
GRID = "#e6e6e6"


def style(ax) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=TEXT)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def mission_success(r: dict) -> bool:
    return bool(
        r.get("mission_complete_reported")
        and (r.get("goals_reached_ground_truth") or 0) >= 4
        and (r.get("collision_episodes") or 0) == 0
        and r.get("final_home_error_m") is not None
        and r["final_home_error_m"] <= 0.20
    )


def load_latest() -> dict[tuple, dict]:
    latest: dict[tuple, dict] = {}
    for line in RUNS.read_text(encoding="utf-8").splitlines():
        if line.strip():
            r = json.loads(line)
            r.setdefault("degradation", "none")
            latest[(r["method"], r["scenario"], r["degradation"], r["seed"])] = r
    return latest


def group(latest, method, scenario, degradation="none") -> list[dict]:
    return [r for (m, s, d, _), r in sorted(latest.items()) if (m, s, d) == (method, scenario, degradation)]


def bar_success(latest) -> None:
    fig, ax = plt.subplots(figsize=(8, 3.6))
    methods = [m for m in METHODS if any(group(latest, m, s) for s in SCENARIOS)]
    width = 0.8 / len(methods)
    for i, method in enumerate(methods):
        for j, scenario in enumerate(SCENARIOS):
            g = group(latest, method, scenario)
            if not g:
                continue
            ok = sum(mission_success(r) for r in g)
            x = j + (i - (len(methods) - 1) / 2) * width
            ax.bar(x, ok / len(g) * 100, width - 0.04, color=COLORS[method], label=METHOD_LABELS[method] if j == 0 else None)
            ax.text(x, ok / len(g) * 100 + 2, f"{ok}/{len(g)}", ha="center", va="bottom", fontsize=8, color=TEXT)
    ax.set_xticks(range(len(SCENARIOS)), SCENARIO_LABELS)
    ax.set_ylim(0, 115)
    ax.set_ylabel("Mission success (%)")
    ax.set_title("Mission success in Webots (5 seeds per bar)", loc="left", color=TEXT)
    style(ax)
    ax.legend(frameon=False, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3)
    fig.tight_layout()
    fig.savefig(FIGURES / "success_by_scenario.png", dpi=200)
    plt.close(fig)


def strip_rmse(latest) -> None:
    fig, ax = plt.subplots(figsize=(8, 3.6))
    methods = [m for m in METHODS if any(group(latest, m, s) for s in SCENARIOS)]
    width = 0.8 / len(methods)
    for i, method in enumerate(methods):
        for j, scenario in enumerate(SCENARIOS):
            values = [r["position_rmse_m"] for r in group(latest, method, scenario) if r.get("position_rmse_m")]
            if not values:
                continue
            x = j + (i - (len(methods) - 1) / 2) * width
            offsets = [(k - (len(values) - 1) / 2) * 0.03 for k in range(len(values))]
            ax.scatter([x + o for o in offsets], values, s=28, color=COLORS[method], edgecolor="white", linewidth=1,
                       label=METHOD_LABELS[method] if j == 0 else None, zorder=3)
            ax.hlines(statistics.median(values), x - width * 0.4, x + width * 0.4, color=TEXT, linewidth=1, zorder=4)
    ax.set_yscale("log")
    ax.set_xticks(range(len(SCENARIOS)), SCENARIO_LABELS)
    ax.set_ylabel("Position RMSE (m, log scale)")
    ax.set_title("Localization error over the whole run (dots = trials, line = median)", loc="left", color=TEXT)
    style(ax)
    ax.legend(frameon=False, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3)
    fig.tight_layout()
    fig.savefig(FIGURES / "rmse_by_scenario.png", dpi=200)
    plt.close(fig)


def error_traces(latest, scenario, filename, title, align_on_kidnap) -> None:
    methods = [m for m in METHODS if group(latest, m, scenario)]
    if not methods:
        return
    fig, axes = plt.subplots(1, len(methods), figsize=(4 * len(methods), 3.6), sharey=True, squeeze=False)
    for ax, method in zip(axes[0], methods):
        for r in group(latest, method, scenario):
            rows = load_rows(r)
            if not rows:
                continue
            t0 = r.get("disturbance_time_s") if align_on_kidnap else 0.0
            if t0 is None:
                t0 = 0.0
            ts, es = [], []
            for row in rows:
                try:
                    ts.append(float(row["time_s"]) - t0)
                    es.append(float(row["position_error_m"]))
                except (KeyError, ValueError):
                    continue
            ok = mission_success(r)
            ax.plot(ts, es, linewidth=1.5, color=COLORS[method], alpha=1.0 if ok else 0.45,
                    linestyle="-" if ok else "--")
        ax.axhline(0.20, color=MUTED, linewidth=0.8, linestyle=":")
        if align_on_kidnap:
            ax.axvline(0.0, color=MUTED, linewidth=0.8)
            ax.set_xlim(-30, 200)
            ax.set_xlabel("Time since kidnap (s)")
        else:
            ax.set_xlim(0, 60)
            ax.set_xlabel("Time (s)")
        ax.set_yscale("log")
        ax.set_title(METHOD_LABELS[method], fontsize=9, color=TEXT, loc="left")
        style(ax)
    axes[0][0].set_ylabel("Position error (m)")
    fig.suptitle(title + "  (one line per seed; solid = mission success, dashed = failure; dotted = 0.20 m)",
                 fontsize=9, color=TEXT, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(FIGURES / filename, dpi=200)
    plt.close(fig)


def extension(latest) -> None:
    levels = ["none", "moderate", "severe"]
    scenarios = [("nominal", "Nominal"), ("blocked_route", "Blocked route")]
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.4), sharey=True)
    for ax, (scenario, label) in zip(axes, scenarios):
        for i, method in enumerate(["team", "team_robust"]):
            for j, level in enumerate(levels):
                g = group(latest, method, scenario, level)
                if not g:
                    continue
                ok = sum(mission_success(r) for r in g)
                x = j + (i - 0.5) * 0.4
                ax.bar(x, ok / len(g) * 100, 0.36, color=COLORS[method], label=METHOD_LABELS[method] if j == 0 else None)
                times = [r["duration_s"] for r in g if mission_success(r)]
                note = f"{ok}/{len(g)}" + (f"\n{statistics.mean(times):.0f} s" if times else "")
                ax.text(x, ok / len(g) * 100 + 2, note, ha="center", va="bottom", fontsize=7, color=TEXT)
        ax.set_xticks(range(3), ["No degradation", "Moderate", "Severe"])
        ax.set_ylim(0, 130)
        ax.set_title(label, fontsize=9, color=TEXT, loc="left")
        style(ax)
    axes[0].set_ylabel("Mission success (%)")
    axes[0].legend(frameon=False, fontsize=8, loc="upper left", ncol=2)
    fig.suptitle("Extension: degraded LiDAR (labels: successes, mean completion time)", fontsize=9, color=TEXT, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(FIGURES / "extension_degradation.png", dpi=200)
    plt.close(fig)


def main() -> None:
    FIGURES.mkdir(parents=True, exist_ok=True)
    latest = load_latest()
    bar_success(latest)
    strip_rmse(latest)
    error_traces(latest, "kidnapped", "kidnap_recovery.png", "Kidnapped robot: estimate error", True)
    error_traces(latest, "uncertain_start", "uncertain_start_convergence.png", "Uncertain start: global localization", False)
    extension(latest)
    for path in sorted(FIGURES.glob("*.png")):
        print(path.relative_to(PROJECT_ROOT))


if __name__ == "__main__":
    main()
