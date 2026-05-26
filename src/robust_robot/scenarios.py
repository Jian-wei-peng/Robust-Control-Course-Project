from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


@dataclass
class NoiseSignal:
    times: np.ndarray
    values: np.ndarray

    def __call__(self, t: float) -> np.ndarray:
        idx = int(np.searchsorted(self.times, t, side="right") - 1)
        idx = int(np.clip(idx, 0, len(self.values) - 1))
        return self.values[idx]


def make_noise_signal(cfg: dict, rng: np.random.Generator) -> NoiseSignal | None:
    disturbance = cfg["scenario"].get("disturbance", {})
    if disturbance.get("type") != "piecewise_plus_noise":
        return None

    tf = float(cfg["simulation"]["tf"])
    Ts = float(disturbance.get("noise_sample_time", 0.05))
    noise_std = np.asarray(disturbance.get("noise_std", [0.0, 0.0]), dtype=float)
    times = np.arange(0.0, tf + Ts, Ts)
    values = rng.normal(loc=0.0, scale=noise_std, size=(len(times), 2))
    return NoiseSignal(times=times, values=values)


def _piecewise_value(t: float, segments: list) -> np.ndarray:
    for start, end, value in segments:
        if float(start) <= t < float(end):
            return np.asarray(value, dtype=float)
    if segments:
        return np.asarray(segments[-1][2], dtype=float)
    return np.zeros(2, dtype=float)


def tau_d_at(t: float, cfg: dict, noise_signal: Callable[[float], np.ndarray] | None = None) -> np.ndarray:
    disturbance = cfg["scenario"].get("disturbance", {"type": "none"})
    disturbance_type = disturbance.get("type", "none")

    if disturbance_type == "none":
        tau_d = np.zeros(2, dtype=float)
    elif disturbance_type == "step":
        start_time = float(disturbance.get("start_time", 0.0))
        value = np.asarray(disturbance.get("value", [0.0, 0.0]), dtype=float)
        tau_d = value if t >= start_time else np.zeros(2, dtype=float)
    elif disturbance_type == "piecewise":
        tau_d = _piecewise_value(t, disturbance.get("segments", []))
    elif disturbance_type == "piecewise_plus_noise":
        tau_d = _piecewise_value(t, disturbance.get("segments", []))
        if noise_signal is not None:
            tau_d = tau_d + noise_signal(t)
    else:
        raise ValueError(f"Unsupported disturbance type: {disturbance_type}")

    return tau_d


def M_D_at(t: float, cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    robot = cfg["robot"]
    m = float(robot["m0"])
    I = float(robot["I0"])
    d_v = float(robot["d_v0"])
    d_omega = float(robot["d_omega0"])

    payload = cfg["scenario"].get("payload", {})
    if payload.get("enabled", False) and t >= float(payload.get("change_time", 0.0)):
        m += float(payload.get("delta_m", 0.0))
        I += float(payload.get("delta_I", 0.0))

    friction = cfg["scenario"].get("friction", {})
    if friction.get("enabled", False) and t >= float(friction.get("change_time", 0.0)):
        d_v += float(friction.get("delta_d_v", 0.0))
        d_omega += float(friction.get("delta_d_omega", 0.0))

    M = np.diag([m, I])
    D = np.diag([d_v, d_omega])
    return M, D


def equivalent_disturbance(
    t: float,
    state: np.ndarray,
    tau: np.ndarray,
    cfg: dict,
    tau_d: np.ndarray,
) -> np.ndarray:
    """Return the matched disturbance d in the nominal error channel.

    It is defined by M0 eta_dot_nominal = tau - D0 eta - d_equiv, so d_equiv
    equals the input-channel disturbance that explains the true acceleration
    under the nominal model.
    """
    robot = cfg["robot"]
    M0 = np.diag([robot["m0"], robot["I0"]]).astype(float)
    D0 = np.diag([robot["d_v0"], robot["d_omega0"]]).astype(float)
    M, D = M_D_at(t, cfg)
    eta = state[3:5]
    eta_dot_true = np.linalg.solve(M, tau - D @ eta - tau_d)
    return tau - D0 @ eta - M0 @ eta_dot_true
