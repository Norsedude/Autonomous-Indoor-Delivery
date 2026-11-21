#!/usr/bin/env python3
"""Select the method, scenario, and seed for the next Webots run.

Writes ``config/run_overrides.json``, which the robot controller merges on top
of ``config/project_config.json`` when it starts.  Reload or reset the world
in Webots after changing it.

Examples::

    python tools/set_run_config.py --method baseline --scenario nominal --seed 1
    python tools/set_run_config.py --method team --scenario kidnapped --seed 3
    python tools/set_run_config.py --method team_robust --scenario nominal --degradation severe
    python tools/set_run_config.py --show
    python tools/set_run_config.py --clear      # back to project_config.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))

from mission import RUN_OVERRIDES_PATH, deep_merge, load_project_config  # noqa: E402


def main() -> None:
    experiments = json.loads((PROJECT_ROOT / "config" / "experiments.json").read_text(encoding="utf-8"))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--method", choices=sorted(experiments["methods"]))
    parser.add_argument("--scenario", choices=sorted(experiments["scenarios"]))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--degradation", choices=sorted(experiments["degradations"]), default="none",
                        help="Extension: inject LiDAR degradation (none, moderate, severe)")
    parser.add_argument("--show", action="store_true", help="Print the effective configuration summary")
    parser.add_argument("--clear", action="store_true", help="Delete config/run_overrides.json")
    args = parser.parse_args()

    if args.clear:
        if RUN_OVERRIDES_PATH.exists():
            RUN_OVERRIDES_PATH.unlink()
        print("Removed run overrides; Webots uses config/project_config.json as is.")
        return

    if args.method or args.scenario:
        method = args.method or "team"
        scenario = args.scenario or "nominal"
        overrides = deep_merge(experiments["methods"][method]["config"], experiments["scenarios"][scenario].get("config", {}))
        overrides = deep_merge(overrides, {"experiment": {"method": method, "scenario": scenario, "seed": args.seed}})
        if args.degradation != "none":
            overrides = deep_merge(overrides, {"sensor_degradation": experiments["degradations"][args.degradation]})
        RUN_OVERRIDES_PATH.write_text(json.dumps(overrides, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {RUN_OVERRIDES_PATH.relative_to(PROJECT_ROOT)}")
        print(f"Webots instructions: {experiments['scenarios'][scenario].get('webots_instructions', '')}")

    config = load_project_config()
    print(
        "Effective run: "
        f"method={config.get('experiment', {}).get('method')} "
        f"localization={config['localization']['mode']} "
        f"planning={config['controller']['planning_mode']} "
        f"belief={config['localization'].get('initial_belief')} "
        f"scenario={config.get('experiment', {}).get('scenario')} "
        f"seed={config.get('experiment', {}).get('seed')} "
        f"robust_sensing={config.get('sensing', {}).get('robust', False)} "
        f"degradation={config.get('sensor_degradation', {}).get('name', 'none')}"
    )
    print("Reset the simulation in Webots (Ctrl+Shift+T) so the controller reloads it.")


if __name__ == "__main__":
    main()
