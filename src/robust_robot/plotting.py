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


def save_standard_plots(result: SimulationResult, output_dir: str | Path) -> None:
    output_dir = Path(output_dir)
    show = bool(result.cfg["output"].get("show", False))
    plot_trajectory(result, output_dir / "trajectory.png", show)
    plot_errors(result, output_dir / "errors.png", show)
    plot_velocity_errors(result, output_dir / "velocity_errors.png", show)
    plot_controls(result, output_dir / "controls.png", show)
    plot_disturbance_estimate(result, output_dir / "disturbance_estimate.png", show)
