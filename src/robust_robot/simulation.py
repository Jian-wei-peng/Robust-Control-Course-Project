from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.integrate import solve_ivp

from .config import set_random_seed
from .controllers import compute_lqr_dob_tau, compute_lqr_tau
from .dob import (
    build_augmented_observer_model,
    check_observability,
    choose_observer_poles,
    design_dob_gain,
    observer_dynamics,
)
from .dynamics import (
    build_linear_error_model,
    initial_state,
    reference,
    robot_dynamics,
    tau_ff,
    tracking_error,
    wrap_angle,
)
from .lqr import (
    check_controllability,
    closed_loop_eigs,
    closed_loop_matrix,
    diagonal_weights,
    solve_lqr,
)
from .scenarios import M_D_at, equivalent_disturbance, make_noise_signal, tau_d_at


@dataclass
class SimulationResult:
    cfg: dict[str, Any]
    scenario_name: str
    controller_name: str
    t: np.ndarray
    state: np.ndarray
    xi: np.ndarray
    q_r: np.ndarray
    eta_r: np.ndarray
    tau: np.ndarray
    tau_ff: np.ndarray
    delta_tau: np.ndarray
    tau_d: np.ndarray
    d: np.ndarray
    d_hat: np.ndarray | None
    x_a_hat: np.ndarray | None
    A: np.ndarray
    B: np.ndarray
    Q: np.ndarray
    R: np.ndarray
    P: np.ndarray
    K: np.ndarray
    A_K: np.ndarray
    L: np.ndarray | None
    system_checks: dict[str, Any]


def _design_controller(cfg: dict) -> dict[str, Any]:
    A, B = build_linear_error_model(cfg)
    Q = diagonal_weights(cfg["lqr"]["Q"])
    R = diagonal_weights(cfg["lqr"]["R"])
    K, P = solve_lqr(A, B, Q, R)
    A_K = closed_loop_matrix(A, B, K)
    eig_A_K = closed_loop_eigs(A, B, K)
    checks = {
        "controllability_rank": check_controllability(A, B),
        "eig_A_K_real": np.real(eig_A_K).tolist(),
        "eig_A_K_imag": np.imag(eig_A_K).tolist(),
    }
    return {"A": A, "B": B, "Q": Q, "R": R, "K": K, "P": P, "A_K": A_K, "checks": checks}


def _design_observer(cfg: dict, A: np.ndarray, B: np.ndarray, A_K: np.ndarray) -> dict[str, Any]:
    A_a, B_a, C_a = build_augmented_observer_model(A, B)
    observer_poles = choose_observer_poles(A_K, float(cfg["dob"].get("speed_factor", 5.0)))
    L = design_dob_gain(A_a, C_a, observer_poles)
    eig_obs = np.linalg.eigvals(A_a - L @ C_a)
    return {
        "A_a": A_a,
        "B_a": B_a,
        "C_a": C_a,
        "L": L,
        "observer_poles": observer_poles,
        "checks": {
            "observability_rank": check_observability(A_a, C_a),
            "eig_observer_real": np.real(eig_obs).tolist(),
            "eig_observer_imag": np.imag(eig_obs).tolist(),
        },
    }


def run_simulation(cfg: dict, controller_name: str) -> SimulationResult:
    if controller_name not in {"lqr", "lqr_dob"}:
        raise ValueError(f"Unsupported controller: {controller_name}")

    rng = set_random_seed(cfg)
    noise_signal = make_noise_signal(cfg, rng)
    design = _design_controller(cfg)
    A = design["A"]
    B = design["B"]
    K = design["K"]
    P = design["P"]
    Q = design["Q"]
    R = design["R"]
    A_K = design["A_K"]
    system_checks = dict(design["checks"])

    observer = None
    L = None
    if controller_name == "lqr_dob":
        observer = _design_observer(cfg, A, B, A_K)
        L = observer["L"]
        system_checks.update(observer["checks"])

    state0 = initial_state(cfg)
    if controller_name == "lqr_dob":
        z0 = np.concatenate([state0, np.zeros(7, dtype=float)])
    else:
        z0 = state0

    tf = float(cfg["simulation"]["tf"])
    dt = float(cfg["simulation"]["dt"])
    t_eval = np.arange(0.0, tf + 0.5 * dt, dt)

    def ode(t: float, z: np.ndarray) -> np.ndarray:
        state = z[:5].copy()
        state[2] = wrap_angle(state[2])
        ref = reference(t, cfg)
        xi = tracking_error(state, ref)
        current_tau_ff = tau_ff(ref, cfg)

        if controller_name == "lqr_dob":
            assert observer is not None
            x_a_hat = z[5:12]
            d_hat = x_a_hat[5:7]
            tau, _, u = compute_lqr_dob_tau(K, xi, d_hat, current_tau_ff)
            y = xi
            x_a_hat_dot = observer_dynamics(
                x_a_hat,
                y,
                u,
                observer["A_a"],
                observer["B_a"],
                observer["C_a"],
                observer["L"],
            )
        else:
            d_hat = None
            tau, _, _ = compute_lqr_tau(K, xi, current_tau_ff)
            x_a_hat_dot = None

        M, D = M_D_at(t, cfg)
        tau_d = tau_d_at(t, cfg, noise_signal)
        state_dot = robot_dynamics(state, tau, M, D, tau_d)

        if controller_name == "lqr_dob":
            return np.concatenate([state_dot, x_a_hat_dot])
        return state_dot

    sol = solve_ivp(
        ode,
        (0.0, tf),
        z0,
        t_eval=t_eval,
        rtol=float(cfg["simulation"].get("rtol", 1e-7)),
        atol=float(cfg["simulation"].get("atol", 1e-9)),
    )
    if not sol.success:
        raise RuntimeError(sol.message)

    t = sol.t
    z = sol.y.T
    state = z[:, :5].copy()
    state[:, 2] = wrap_angle(state[:, 2])
    x_a_hat = z[:, 5:12].copy() if controller_name == "lqr_dob" else None

    N = len(t)
    xi_hist = np.zeros((N, 5))
    q_r_hist = np.zeros((N, 3))
    eta_r_hist = np.zeros((N, 2))
    tau_hist = np.zeros((N, 2))
    tau_ff_hist = np.zeros((N, 2))
    delta_tau_hist = np.zeros((N, 2))
    tau_d_hist = np.zeros((N, 2))
    d_hist = np.zeros((N, 2))
    d_hat_hist = np.zeros((N, 2)) if controller_name == "lqr_dob" else None

    for i, ti in enumerate(t):
        ref = reference(float(ti), cfg)
        xi = tracking_error(state[i], ref)
        current_tau_ff = tau_ff(ref, cfg)
        if controller_name == "lqr_dob":
            assert x_a_hat is not None and d_hat_hist is not None
            d_hat = x_a_hat[i, 5:7]
            tau, delta_tau, _ = compute_lqr_dob_tau(K, xi, d_hat, current_tau_ff)
            d_hat_hist[i] = d_hat
        else:
            tau, delta_tau, _ = compute_lqr_tau(K, xi, current_tau_ff)

        tau_d = tau_d_at(float(ti), cfg, noise_signal)
        xi_hist[i] = xi
        q_r_hist[i] = ref.q_r
        eta_r_hist[i] = ref.eta_r
        tau_hist[i] = tau
        tau_ff_hist[i] = current_tau_ff
        delta_tau_hist[i] = delta_tau
        tau_d_hist[i] = tau_d
        d_hist[i] = equivalent_disturbance(float(ti), state[i], tau, cfg, tau_d)

    return SimulationResult(
        cfg=cfg,
        scenario_name=cfg["scenario"]["name"],
        controller_name=controller_name,
        t=t,
        state=state,
        xi=xi_hist,
        q_r=q_r_hist,
        eta_r=eta_r_hist,
        tau=tau_hist,
        tau_ff=tau_ff_hist,
        delta_tau=delta_tau_hist,
        tau_d=tau_d_hist,
        d=d_hist,
        d_hat=d_hat_hist,
        x_a_hat=x_a_hat,
        A=A,
        B=B,
        Q=Q,
        R=R,
        P=P,
        K=K,
        A_K=A_K,
        L=L,
        system_checks=system_checks,
    )
