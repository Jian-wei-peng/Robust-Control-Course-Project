from copy import deepcopy

from _common import ROOT
from robust_robot.analysis import save_metrics, save_raw_result, summarize_metrics
from robust_robot.config import load_config, make_output_dir
from robust_robot.plotting import plot_dob_sweep, save_standard_plots
from robust_robot.simulation import run_simulation


if __name__ == "__main__":
    base_cfg = load_config(
        ROOT / "configs" / "default.yaml",
        ROOT / "configs" / "piecewise_disturbance.yaml",
    )

    metrics = []
    for speed_factor in [2.0, 3.0, 5.0, 8.0, 12.0]:
        cfg = deepcopy(base_cfg)
        cfg["dob"]["speed_factor"] = speed_factor
        result = run_simulation(cfg, "lqr_dob")
        output_dir = make_output_dir(
            cfg,
            result.scenario_name,
            f"{result.controller_name}_speed_{speed_factor:g}",
        )
        save_raw_result(result, output_dir)
        save_standard_plots(result, output_dir)
        row = summarize_metrics(result)
        row["dob_speed_factor"] = speed_factor
        metrics.append(row)

    summary_path = ROOT / "results" / "tables" / "dob_sweep_metrics.csv"
    df = save_metrics(metrics, summary_path)
    plot_dob_sweep(df, ROOT / "results" / "figures" / "dob_sweep.png")
    print(f"Saved DOB sweep: {summary_path}")
