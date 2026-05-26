from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Reference:
    q_r: np.ndarray
    eta_r: np.ndarray
    eta_r_dot: np.ndarray


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def circle_reference(t: float, cfg: dict) -> Reference:
    R_c = float(cfg["trajectory"]["radius"])
    Omega = float(cfg["trajectory"]["omega"])
    x_r = R_c * np.cos(Omega * t)
    y_r = R_c * np.sin(Omega * t)
    theta_r = wrap_angle(Omega * t + np.pi / 2.0)
    v_r = R_c * Omega
    omega_r = Omega
    q_r = np.array([x_r, y_r, theta_r], dtype=float)
    eta_r = np.array([v_r, omega_r], dtype=float)
    eta_r_dot = np.zeros(2, dtype=float)
    return Reference(q_r=q_r, eta_r=eta_r, eta_r_dot=eta_r_dot)


def reference(t: float, cfg: dict) -> Reference:
    trajectory_type = cfg["trajectory"].get("type", "circle")
    if trajectory_type != "circle":
        raise ValueError(f"Unsupported trajectory type: {trajectory_type}")
    return circle_reference(t, cfg)


def initial_state(cfg: dict) -> np.ndarray:
    x0 = cfg["initial_state"]
    return np.array(
        [x0["x"], x0["y"], x0["theta"], x0["v"], x0["omega"]],
        dtype=float,
    )


def nominal_matrices(cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    robot = cfg["robot"]
    M0 = np.diag([robot["m0"], robot["I0"]]).astype(float)
    D0 = np.diag([robot["d_v0"], robot["d_omega0"]]).astype(float)
    return M0, D0


def tracking_error(state: np.ndarray, ref: Reference) -> np.ndarray:
    x, y, theta, v, omega = state
    x_r, y_r, theta_r = ref.q_r
    v_r, omega_r = ref.eta_r

    dx = x_r - x
    dy = y_r - y
    c = np.cos(theta)
    s = np.sin(theta)
    x_e = c * dx + s * dy
    y_e = -s * dx + c * dy
    theta_e = wrap_angle(theta_r - theta)
    v_e = v_r - v
    omega_e = omega_r - omega
    return np.array([x_e, y_e, theta_e, v_e, omega_e], dtype=float)


def build_linear_error_model(cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    ref0 = reference(0.0, cfg)
    v_r, omega_r = ref0.eta_r
    robot = cfg["robot"]
    m = float(robot["m0"])
    I = float(robot["I0"])
    d_v = float(robot["d_v0"])
    d_omega = float(robot["d_omega0"])

    A = np.array(
        [
            [0.0, omega_r, 0.0, 1.0, 0.0],
            [-omega_r, 0.0, v_r, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, -d_v / m, 0.0],
            [0.0, 0.0, 0.0, 0.0, -d_omega / I],
        ],
        dtype=float,
    )
    B = np.array(
        [
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [1.0 / m, 0.0],
            [0.0, 1.0 / I],
        ],
        dtype=float,
    )
    return A, B


def tau_ff(ref: Reference, cfg: dict) -> np.ndarray:
    M0, D0 = nominal_matrices(cfg)
    return M0 @ ref.eta_r_dot + D0 @ ref.eta_r


def robot_dynamics(
    state: np.ndarray,
    tau: np.ndarray,
    M: np.ndarray,
    D: np.ndarray,
    tau_d: np.ndarray,
) -> np.ndarray:
    x, y, theta, v, omega = state
    eta = np.array([v, omega], dtype=float)
    eta_dot = np.linalg.solve(M, tau - D @ eta - tau_d)
    return np.array(
        [
            v * np.cos(theta),
            v * np.sin(theta),
            omega,
            eta_dot[0],
            eta_dot[1],
        ],
        dtype=float,
    )
