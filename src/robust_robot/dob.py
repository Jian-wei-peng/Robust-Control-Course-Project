from __future__ import annotations

import numpy as np
from scipy.signal import place_poles


def build_augmented_observer_model(A: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = A.shape[0]
    m = B.shape[1]
    A_a = np.block(
        [
            [A, B],
            [np.zeros((m, n)), np.zeros((m, m))],
        ]
    )
    B_a = np.vstack([B, np.zeros((m, m))])
    C_a = np.hstack([np.eye(n), np.zeros((n, m))])
    return A_a, B_a, C_a


def observability_matrix(A_a: np.ndarray, C_a: np.ndarray) -> np.ndarray:
    n = A_a.shape[0]
    blocks = [C_a]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A_a
        blocks.append(C_a @ Ak)
    return np.vstack(blocks)


def check_observability(A_a: np.ndarray, C_a: np.ndarray, tol: float = 1e-9) -> int:
    return int(np.linalg.matrix_rank(observability_matrix(A_a, C_a), tol=tol))


def choose_observer_poles(A_K: np.ndarray, speed_factor: float) -> np.ndarray:
    eigs = np.linalg.eigvals(A_K)
    stable_rates = np.abs(np.real(eigs[np.real(eigs) < -1e-8]))
    alpha = float(np.min(stable_rates)) if stable_rates.size else 1.0
    base = max(alpha * speed_factor, 1.0)
    multipliers = np.array([1.0, 1.15, 1.3, 1.5, 1.75, 2.05, 2.4])
    return -base * multipliers


def design_dob_gain(A_a: np.ndarray, C_a: np.ndarray, observer_poles: np.ndarray) -> np.ndarray:
    placed = place_poles(A_a.T, C_a.T, observer_poles, method="YT")
    return placed.gain_matrix.T


def observer_dynamics(
    x_a_hat: np.ndarray,
    y: np.ndarray,
    u: np.ndarray,
    A_a: np.ndarray,
    B_a: np.ndarray,
    C_a: np.ndarray,
    L: np.ndarray,
) -> np.ndarray:
    return A_a @ x_a_hat + B_a @ u + L @ (y - C_a @ x_a_hat)
