#!/usr/bin/env python3
"""Run Webots experiments unattended (headless) and collect the logs.

For every method x scenario x seed this script:
1. writes ``config/run_overrides.json`` (as ``tools/set_run_config.py`` does);
2. writes ``worlds/_batch.wbt``: the official world plus a scripted
   ``scenario_driver`` Supervisor that performs the scenario's key presses
   at fixed times and quits Webots at the end (the official world and the
   evaluation Supervisor are not modified);
3. runs Webots in fast, no-rendering batch mode;
4. moves the evaluation Supervisor's CSV to
   ``results/webots/<method>/<scenario>/seed<N>.csv`` and appends the health
   check metrics to ``results/webots/runs.jsonl``.

Examples::

    python tools/run_webots_batch.py --method baseline --scenario nominal --seeds 1
    python tools/run_webots_batch.py --method team --all-scenarios --seeds 1 2 3 4 5
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from check_run_health import evaluate  # noqa: E402
from mission import RUN_OVERRIDES_PATH, deep_merge  # noqa: E402

WEBOTS_CANDIDATES = [
    Path(r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"),
    Path("/Applications/Webots.app/Contents/MacOS/webots"),
    Path("/usr/local/bin/webots"),
    Path("/usr/bin/webots"),
]
OFFICIAL_WORLD = PROJECT_ROOT / "worlds" / "cmpe185_indoor_delivery.wbt"
BATCH_WORLD = PROJECT_ROOT / "worlds" / "_batch.wbt"
RESULTS = PROJECT_ROOT / "results"
OUTPUT = RESULTS / "webots"

DRIVER_NODE = """
Robot {{
  children [
    Receiver {{
      name "driver receiver"
      channel 7
    }}
  ]
  name "scenario driver"
  controller "scenario_driver"
  controllerArgs [
    "{scenario}"
    "{max_time}"
  ]
  supervisor TRUE
}}
"""


def find_webots(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit)
    for candidate in WEBOTS_CANDIDATES:
        if candidate.exists():
            return candidate
    found = shutil.which("webots")
    if found:
        return Path(found)
    raise SystemExit("Webots executable not found; pass --webots PATH")


def write_overrides(experiments: dict, method: str, scenario: str, seed: int) -> None:
    overrides = deep_merge(experiments["methods"][method]["config"], experiments["scenarios"][scenario].get("config", {}))
    overrides = deep_merge(overrides, {"experiment": {"method": method, "scenario": scenario, "seed": seed}})
    RUN_OVERRIDES_PATH.write_text(json.dumps(overrides, indent=2) + "\n", encoding="utf-8")


def write_batch_world(scenario: str, max_time: float) -> None:
    world = OFFICIAL_WORLD.read_text(encoding="utf-8")
    BATCH_WORLD.write_text(world.rstrip() + "\n" + DRIVER_NODE.format(scenario=scenario, max_time=max_time), encoding="utf-8")


def run_one(webots: Path, experiments: dict, method: str, scenario: str, seed: int, max_time: float, wall_timeout: float) -> dict:
    write_overrides(experiments, method, scenario, seed)
    write_batch_world(scenario, max_time)
    before = set(RESULTS.glob("run_*.csv"))
    start = time.time()
    command = [str(webots), "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout", "--stderr", str(BATCH_WORLD)]
    log_dir = OUTPUT / method / scenario
    log_dir.mkdir(parents=True, exist_ok=True)
    console = log_dir / f"seed{seed}_console.txt"
    with console.open("w", encoding="utf-8", errors="replace") as handle:
        try:
            completed = subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, timeout=wall_timeout)
            exit_code = completed.returncode
        except subprocess.TimeoutExpired:
            exit_code = "timeout"
            subprocess.run(["taskkill", "/F", "/T", "/IM", "webots-bin.exe"], capture_output=True) if sys.platform == "win32" else None
    new_logs = sorted(set(RESULTS.glob("run_*.csv")) - before, key=lambda path: path.stat().st_mtime)
    record = {"method": method, "scenario": scenario, "seed": seed, "webots_exit": exit_code, "wall_clock_s": round(time.time() - start, 1)}
    if not new_logs:
        record["error"] = "no evaluation CSV was written"
        return record
    destination = log_dir / f"seed{seed}.csv"
    shutil.move(str(new_logs[-1]), destination)
    for extra in new_logs[:-1]:
        extra.unlink()
    with destination.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    expected = 4 if experiments["methods"][method]["config"]["controller"]["planning_mode"] == "astar" else 23
    passed, issues, metrics = evaluate(rows, require_complete=True, expected_waypoints=expected)
    record.update({"csv": str(destination.relative_to(PROJECT_ROOT)), "health_pass": passed, "issues": issues, **metrics})
    return record


def main() -> None:
    experiments = json.loads((PROJECT_ROOT / "config" / "experiments.json").read_text(encoding="utf-8"))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--method", choices=sorted(experiments["methods"]), action="append")
    parser.add_argument("--scenario", choices=sorted(experiments["scenarios"]), action="append")
    parser.add_argument("--all-scenarios", action="store_true")
    parser.add_argument("--seeds", type=int, nargs="+", default=[1])
    parser.add_argument("--max-time-s", type=float, default=900.0)
    parser.add_argument("--wall-timeout-s", type=float, default=3600.0)
    parser.add_argument("--webots")
    args = parser.parse_args()

    webots = find_webots(args.webots)
    methods = args.method or ["team"]
    scenarios = experiments["required_scenarios"] if args.all_scenarios else (args.scenario or ["nominal"])
    OUTPUT.mkdir(parents=True, exist_ok=True)
    try:
        for method in methods:
            for scenario in scenarios:
                for seed in args.seeds:
                    record = run_one(webots, experiments, method, scenario, seed, args.max_time_s, args.wall_timeout_s)
                    with (OUTPUT / "runs.jsonl").open("a", encoding="utf-8") as handle:
                        handle.write(json.dumps(record) + "\n")
                    print(json.dumps({k: record.get(k) for k in (
                        "method", "scenario", "seed", "health_pass", "goals_reached_ground_truth",
                        "collision_episodes", "duration_s", "position_rmse_m", "final_home_error_m", "wall_clock_s", "error")}), flush=True)
    finally:
        if RUN_OVERRIDES_PATH.exists():
            RUN_OVERRIDES_PATH.unlink()


if __name__ == "__main__":
    main()
