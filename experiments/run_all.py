from pathlib import Path

from _common import ROOT, run_scenario
from robust_robot.analysis import save_metrics
from robust_robot.plotting import plot_metrics_bar


if __name__ == "__main__":
    scenario_configs = [
        "nominal.yaml",
        "step_disturbance.yaml",
        "piecewise_disturbance.yaml",
        "payload_change.yaml",
        "noise_disturbance.yaml",
    ]

    all_metrics = []
    for scenario_config in scenario_configs:
        all_metrics.extend(run_scenario(scenario_config))

    summary_path = ROOT / "results" / "tables" / "summary_metrics.csv"
    df = save_metrics(all_metrics, summary_path)
    plot_metrics_bar(df, ROOT / "results" / "figures" / "summary_metrics.png")
    print(f"Saved summary: {summary_path}")
