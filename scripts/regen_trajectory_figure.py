"""Re-generate the LQR vs LQR+DOB trajectory figure with:
  - enhanced (larger) piecewise disturbance so the two traces diverge visibly
  - explicit START and END markers
  - reference circle drawn full + reference arc traced during the sim
The output overwrites
  report/figs/piecewise_disturbance_trajectory_compare.png
  slides/figs/piecewise_disturbance_trajectory_compare.png
so the existing \includegraphics paths in main.tex and slides.tex keep working.
"""
from __future__ import annotations

import sys
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
MPL_CACHE = ROOT / ".matplotlib_cache"
MPL_CACHE.mkdir(exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CACHE))

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from robust_robot.config import load_yaml
from robust_robot.simulation import run_simulation


def make_cfg() -> dict:
    cfg = load_yaml(ROOT / "configs" / "default.yaml")
    # enhanced piecewise: ~5-8x larger torque disturbances than the metrics-table scenario
    cfg["scenario"] = {
        "name": "piecewise_demo_large",
        "disturbance": {
            "type": "piecewise",
            "segments": [
                [0.0,  5.0, [0.0, 0.0]],
                [5.0, 12.0, [3.0, 0.8]],
                [12.0, 20.0, [-2.5, -0.7]],
                [20.0, 30.0, [2.0, 0.5]],
            ],
        },
        "payload":  {"enabled": False},
        "friction": {"enabled": False},
    }
    return cfg


def main() -> None:
    cfg = make_cfg()
    print("Running LQR ...")
    rL = run_simulation(cfg, "lqr")
    print("Running LQR+DOB ...")
    rD = run_simulation(cfg, "lqr_dob")

    t = rL.t
    sL = rL.state
    sD = rD.state

    # reference: full circle (closed) drawn as light dotted for visual frame
    theta_full = np.linspace(0.0, 2.0 * np.pi, 400)
    xf = 2.0 * np.cos(theta_full)
    yf = 2.0 * np.sin(theta_full)

    # reference arc actually traced during [0, tf]
    xr = rL.q_r[:, 0]
    yr = rL.q_r[:, 1]

    fig, ax = plt.subplots(figsize=(7, 6.5))
    # full ideal circle (faint)
    ax.plot(xf, yf, color="0.6", linestyle=":", linewidth=1.0,
            label=r"ideal circle ($R_c=2$ m)")
    # reference traced during sim
    ax.plot(xr, yr, "k--", linewidth=1.6, label="reference (traced)")
    # LQR trace (drawn first so it shows where it differs)
    ax.plot(sL[:, 0], sL[:, 1], color="C0", linewidth=2.2,
            label="Pure LQR", alpha=0.95)
    # DOB on top (drawn last)
    ax.plot(sD[:, 0], sD[:, 1], color="C1", linewidth=2.2,
            label="LQR + DOB", alpha=0.95)

    # start markers
    ax.plot(sL[0, 0], sL[0, 1], marker="o", color="green", markersize=13,
            markeredgecolor="k", linestyle="None", label="START (robot)")
    ax.plot(xr[0], yr[0], marker="^", color="lime", markersize=13,
            markeredgecolor="k", linestyle="None", label="START (reference)")
    ax.annotate("START\nrobot\n(2.20, 0.10)",
                xy=(sL[0, 0], sL[0, 1]),
                xytext=(sL[0, 0] + 0.35, sL[0, 1] + 0.55),
                fontsize=9, color="darkgreen",
                arrowprops=dict(arrowstyle="->", color="darkgreen", lw=1.2))

    # end markers
    ax.plot(sL[-1, 0], sL[-1, 1], marker="s", color="C0", markersize=12,
            markeredgecolor="k", linestyle="None", label="END LQR")
    ax.plot(sD[-1, 0], sD[-1, 1], marker="s", color="C1", markersize=12,
            markeredgecolor="k", linestyle="None", label="END LQR+DOB")
    ax.plot(xr[-1], yr[-1], marker="X", color="red", markersize=13,
            markeredgecolor="k", linestyle="None", label="END (reference)")

    # annotate end deviations
    eL = np.hypot(sL[-1, 0] - xr[-1], sL[-1, 1] - yr[-1])
    eD = np.hypot(sD[-1, 0] - xr[-1], sD[-1, 1] - yr[-1])
    ax.annotate(
        f"LQR end\noff by {eL:.2f} m",
        xy=(sL[-1, 0], sL[-1, 1]),
        xytext=(sL[-1, 0] - 1.4, sL[-1, 1] - 1.0),
        fontsize=9, color="C0",
        arrowprops=dict(arrowstyle="->", color="C0", lw=1.2),
    )
    ax.annotate(
        f"DOB end\noff by {eD:.2f} m",
        xy=(sD[-1, 0], sD[-1, 1]),
        xytext=(sD[-1, 0] + 0.25, sD[-1, 1] - 1.3),
        fontsize=9, color="C1",
        arrowprops=dict(arrowstyle="->", color="C1", lw=1.2),
    )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        "Trajectory under enhanced piecewise disturbance\n"
        r"$\tau_d \in \{(0,0),\ (3.0,0.8),\ (-2.5,-0.7),\ (2.0,0.5)\}$ N$\cdot$m",
        fontsize=11,
    )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left", fontsize=8, ncol=2, framealpha=0.9)
    ax.set_xlim(-3.0, 3.4)
    ax.set_ylim(-3.0, 3.0)

    out_paths = [
        ROOT / "report" / "figs" / "piecewise_disturbance_trajectory_compare.png",
        ROOT / "slides" / "figs" / "piecewise_disturbance_trajectory_compare.png",
        ROOT / "results" / "figures" / "comparison" / "piecewise_disturbance_trajectory_compare.png",
    ]
    fig.tight_layout()
    for p in out_paths:
        p.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(p, dpi=130, bbox_inches="tight")
        print(f"saved {p}")

    # quick metrics for the caption
    print(f"\n--- summary ---")
    print(f"LQR  final position : ({sL[-1, 0]:.3f}, {sL[-1, 1]:.3f})  off by {eL:.3f} m")
    print(f"DOB  final position : ({sD[-1, 0]:.3f}, {sD[-1, 1]:.3f})  off by {eD:.3f} m")
    rms_L = np.sqrt(np.mean((sL[:, 0] - xr) ** 2 + (sL[:, 1] - yr) ** 2))
    rms_D = np.sqrt(np.mean((sD[:, 0] - xr) ** 2 + (sD[:, 1] - yr) ** 2))
    print(f"LQR  full-run RMSE  : {rms_L:.3f} m")
    print(f"DOB  full-run RMSE  : {rms_D:.3f} m")


if __name__ == "__main__":
    main()
