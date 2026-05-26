from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from .config import save_config
from .simulation import SimulationResult


def compute_rmse(xi: np.ndarray) -> float:
    e_p_sq = xi[:, 0] ** 2 + xi[:, 1] ** 2
    return float(np.sqrt(np.mean(e_p_sq)))


def compute_itae(t: np.ndarray, xi: np.ndarray) -> float:
    e_p = np.sqrt(xi[:, 0] ** 2 + xi[:, 1] ** 2)
    return float(np.trapz(t * e_p, t))


def compute_peak_error(xi: np.ndarray) -> float:
    e_p = np.sqrt(xi[:, 0] ** 2 + xi[:, 1] ** 2)
    return float(np.max(e_p))


def compute_control_effort(t: np.ndarray, delta_tau: np.ndarray) -> float:
    effort = np.sum(delta_tau**2, axis=1)
    return float(np.trapz(effort, t))


def compute_disturbance_rmse(d: np.ndarray, d_hat: np.ndarray | None) -> float:
    if d_hat is None:
        return float("nan")
    err = d - d_hat
    return float(np.sqrt(np.mean(np.sum(err**2, axis=1))))


def summarize_metrics(result: SimulationResult) -> dict[str, Any]:
    return {
        "scenario": result.scenario_name,
        "controller": result.controller_name,
        "rmse": compute_rmse(result.xi),
        "itae": compute_itae(result.t, result.xi),
        "peak_error": compute_peak_error(result.xi),
        "control_effort": compute_control_effort(result.t, result.delta_tau),
        "disturbance_rmse": compute_disturbance_rmse(result.d, result.d_hat),
    }


def save_raw_result(result: SimulationResult, output_dir: str | Path) -> None:
    output_dir = Path(output_dir)
    np.savez(
        output_dir / "raw_data.npz",
        t=result.t,
        state=result.state,
        xi=result.xi,
        q_r=result.q_r,
        eta_r=result.eta_r,
        tau=result.tau,
        tau_ff=result.tau_ff,
        delta_tau=result.delta_tau,
        tau_d=result.tau_d,
        d=result.d,
        d_hat=np.asarray(result.d_hat) if result.d_hat is not None else np.empty((0, 2)),
        K=result.K,
        P=result.P,
        L=np.asarray(result.L) if result.L is not None else np.empty((0, 0)),
    )
    save_config(result.cfg, output_dir)
    with (output_dir / "system_checks.json").open("w", encoding="utf-8") as f:
        json.dump(result.system_checks, f, indent=2)


def save_metrics(metrics: list[dict[str, Any]], path: str | Path) -> pd.DataFrame:
    df = pd.DataFrame(metrics)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False)
    return df
