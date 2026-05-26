from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are


def diagonal_weights(values: list[float]) -> np.ndarray:
    return np.diag(np.asarray(values, dtype=float))


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    n = A.shape[0]
    blocks = [B]
    Ak = np.eye(n)
    for _ in range(1, n):
        Ak = Ak @ A
        blocks.append(Ak @ B)
    return np.hstack(blocks)


def check_controllability(A: np.ndarray, B: np.ndarray, tol: float = 1e-9) -> int:
    return int(np.linalg.matrix_rank(controllability_matrix(A, B), tol=tol))


def solve_lqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K, P


def closed_loop_matrix(A: np.ndarray, B: np.ndarray, K: np.ndarray) -> np.ndarray:
    return A - B @ K


def closed_loop_eigs(A: np.ndarray, B: np.ndarray, K: np.ndarray) -> np.ndarray:
    return np.linalg.eigvals(closed_loop_matrix(A, B, K))
