from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .simulation import SimulationResult


def _save_show(fig: plt.Figure, path: Path, show: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    if show:
        plt.show()
    plt.close(fig)


def plot_trajectory(result: SimulationResult, path: str | Path, show: bool = False) -> None:
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(result.q_r[:, 0], result.q_r[:, 1], "k--", label="reference")
    ax.plot(result.state[:, 0], result.state[:, 1], label=result.controller_name)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Trajectory - {result.scenario_name}")
    ax.grid(True)
    ax.legend()
    _save_show(fig, Path(path), show)


def plot_errors(result: SimulationResult, path: str | Path, show: bool = False) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    labels = ["x_e [m]", "y_e [m]", "theta_e [rad]"]
    for idx, ax in enumerate(axes):
        ax.plot(result.t, result.xi[:, idx])
        ax.set_ylabel(labels[idx])
        ax.grid(True)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"Tracking Errors - {result.scenario_name} - {result.controller_name}")
    _save_show(fig, Path(path), show)


def plot_velocity_errors(result: SimulationResult, path: str | Path, show: bool = False) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    labels = ["v_e [m/s]", "omega_e [rad/s]"]
    for idx, ax in enumerate(axes):
        ax.plot(result.t, result.xi[:, idx + 3])
        ax.set_ylabel(labels[idx])
        ax.grid(True)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"Velocity Errors - {result.scenario_name} - {result.controller_name}")
    _save_show(fig, Path(path), show)


def plot_controls(result: SimulationResult, path: str | Path, show: bool = False) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    labels = ["tau_v", "tau_omega"]
    for idx, ax in enumerate(axes):
        ax.plot(result.t, result.tau[:, idx], label="tau")
        ax.plot(result.t, result.tau_ff[:, idx], "k--", label="tau_ff")
        ax.set_ylabel(labels[idx])
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"Control Inputs - {result.scenario_name} - {result.controller_name}")
    _save_show(fig, Path(path), show)


def plot_disturbance_estimate(result: SimulationResult, path: str | Path, show: bool = False) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    labels = ["d_1", "d_2"]
    for idx, ax in enumerate(axes):
        ax.plot(result.t, result.d[:, idx], "k--", label="d equivalent")
        if result.d_hat is not None:
            ax.plot(result.t, result.d_hat[:, idx], label="d_hat")
        ax.plot(result.t, result.tau_d[:, idx], ":", label="tau_d")
        ax.set_ylabel(labels[idx])
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"Disturbance Estimate - {result.scenario_name} - {result.controller_name}")
    _save_show(fig, Path(path), show)


def plot_metrics_bar(df: pd.DataFrame, path: str | Path, show: bool = False) -> None:
    metrics = ["rmse", "itae", "peak_error", "control_effort"]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    axes = axes.ravel()
    for metric, ax in zip(metrics, axes):
        pivot = df.pivot(index="scenario", columns="controller", values=metric)
        pivot.plot(kind="bar", ax=ax)
        ax.set_title(metric)
        ax.grid(True, axis="y")
    fig.suptitle("Controller Metrics Comparison")
    _save_show(fig, Path(path), show)


def plot_dob_sweep(df: pd.DataFrame, path: str | Path, show: bool = False) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True)
    axes = axes.ravel()
    metrics = ["rmse", "itae", "control_effort", "disturbance_rmse"]
    for metric, ax in zip(metrics, axes):
        ax.plot(df["dob_speed_factor"], df[metric], marker="o")
        ax.set_ylabel(metric)
        ax.grid(True)
    axes[-2].set_xlabel("DOB speed factor")
    axes[-1].set_xlabel("DOB speed factor")
    fig.suptitle("DOB Bandwidth Sweep")
    _save_show(fig, Path(path), show)


def _label(result: SimulationResult) -> str:
    return "LQR + DOB" if result.controller_name == "lqr_dob" else "Pure LQR"


def plot_trajectory_compare(results: dict[str, SimulationResult], path: str | Path, show: bool = False) -> None:
    fig, ax = plt.subplots(figsize=(6, 5))
    first = next(iter(results.values()))
    ax.plot(first.q_r[:, 0], first.q_r[:, 1], "k--", label="reference")
    for result in results.values():
        ax.plot(result.state[:, 0], result.state[:, 1], label=_label(result))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Trajectory Comparison - {first.scenario_name}")
    ax.grid(True)
    ax.legend()
    _save_show(fig, Path(path), show)


def plot_position_error_compare(results: dict[str, SimulationResult], path: str | Path, show: bool = False) -> None:
    fig, ax = plt.subplots(figsize=(7, 4))
    first = next(iter(results.values()))
    for result in results.values():
        e_p = np.sqrt(result.xi[:, 0] ** 2 + result.xi[:, 1] ** 2)
        ax.plot(result.t, e_p, label=_label(result))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("position error [m]")
    ax.set_title(f"Position Error Comparison - {first.scenario_name}")
    ax.grid(True)
    ax.legend()
    _save_show(fig, Path(path), show)


def plot_state_error_compare(results: dict[str, SimulationResult], path: str | Path, show: bool = False) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    first = next(iter(results.values()))
    indices = [2, 3, 4]
    labels = ["theta_e [rad]", "v_e [m/s]", "omega_e [rad/s]"]
    for idx, ax, ylabel in zip(indices, axes, labels):
        for result in results.values():
            ax.plot(result.t, result.xi[:, idx], label=_label(result))
        ax.set_ylabel(ylabel)
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"State Error Comparison - {first.scenario_name}")
    _save_show(fig, Path(path), show)


def plot_control_compare(results: dict[str, SimulationResult], path: str | Path, show: bool = False) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    first = next(iter(results.values()))
    labels = ["tau_v", "tau_omega"]
    for idx, ax in enumerate(axes):
        for result in results.values():
            ax.plot(result.t, result.tau[:, idx], label=_label(result))
        ax.set_ylabel(labels[idx])
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"Control Input Comparison - {first.scenario_name}")
    _save_show(fig, Path(path), show)


def plot_disturbance_compare(results: dict[str, SimulationResult], path: str | Path, show: bool = False) -> None:
    result = results.get("lqr_dob")
    if result is None:
        return
    plot_disturbance_estimate(result, path, show)


def save_comparison_plots(results: dict[str, SimulationResult], output_dir: str | Path) -> None:
    if not results:
        return
    output_dir = Path(output_dir)
    first = next(iter(results.values()))
    show = bool(first.cfg["output"].get("show", False))
    scenario_name = first.scenario_name
    plot_trajectory_compare(results, output_dir / f"{scenario_name}_trajectory_compare.png", show)
    plot_position_error_compare(results, output_dir / f"{scenario_name}_position_error_compare.png", show)
    plot_state_error_compare(results, output_dir / f"{scenario_name}_state_error_compare.png", show)
    plot_control_compare(results, output_dir / f"{scenario_name}_control_compare.png", show)
    plot_disturbance_compare(results, output_dir / f"{scenario_name}_disturbance_compare.png", show)


def save_standard_plots(result: SimulationResult, output_dir: str | Path) -> None:
    output_dir = Path(output_dir)
    show = bool(result.cfg["output"].get("show", False))
    plot_trajectory(result, output_dir / "trajectory.png", show)
    plot_errors(result, output_dir / "errors.png", show)
    plot_velocity_errors(result, output_dir / "velocity_errors.png", show)
    plot_controls(result, output_dir / "controls.png", show)
    plot_disturbance_estimate(result, output_dir / "disturbance_estimate.png", show)
