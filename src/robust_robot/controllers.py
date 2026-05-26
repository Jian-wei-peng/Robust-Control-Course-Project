from __future__ import annotations

import numpy as np


def compute_lqr_u(K: np.ndarray, xi: np.ndarray) -> np.ndarray:
    return -K @ xi


def compute_lqr_tau(K: np.ndarray, xi: np.ndarray, tau_ff: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    u = compute_lqr_u(K, xi)
    delta_tau = -u
    tau = tau_ff + delta_tau
    return tau, delta_tau, u


def compute_lqr_dob_tau(
    K: np.ndarray,
    xi: np.ndarray,
    d_hat: np.ndarray,
    tau_ff: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    u = -K @ xi - d_hat
    delta_tau = -u
    tau = tau_ff + delta_tau
    return tau, delta_tau, u
