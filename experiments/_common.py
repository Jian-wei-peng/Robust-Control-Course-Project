from __future__ import annotations

import sys
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
MPL_CACHE = ROOT / ".matplotlib_cache"
MPL_CACHE.mkdir(exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CACHE))

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robust_robot.analysis import save_metrics, save_raw_result, summarize_metrics
from robust_robot.config import load_config, make_output_dir
from robust_robot.plotting import save_standard_plots
from robust_robot.simulation import run_simulation


def run_scenario(scenario_config: str, controllers: list[str] | None = None) -> list[dict]:
    controllers = controllers or ["lqr", "lqr_dob"]
    cfg = load_config(ROOT / "configs" / "default.yaml", ROOT / "configs" / scenario_config)
    metrics = []

    for controller_name in controllers:
        result = run_simulation(cfg, controller_name)
        output_dir = make_output_dir(cfg, result.scenario_name, result.controller_name)
        save_raw_result(result, output_dir)
        save_standard_plots(result, output_dir)
        metrics.append(summarize_metrics(result))

    metrics_path = ROOT / "results" / "tables" / f"{cfg['scenario']['name']}_metrics.csv"
    save_metrics(metrics, metrics_path)
    print(f"Saved metrics: {metrics_path}")
    return metrics
