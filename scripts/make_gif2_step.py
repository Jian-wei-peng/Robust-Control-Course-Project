"""GIF 2: Step disturbance — left trajectory, right position error vs time."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT = "/home/yh/桌面/DocumentSync/学习文档/鲁棒控制基础/project/Robust-Control-Course-Project"
OUT = f"{ROOT}/slides/gifs/step_traj_and_error.gif"

lqr = np.load(f"{ROOT}/results/raw/step_disturbance_lqr/raw_data.npz")
dob = np.load(f"{ROOT}/results/raw/step_disturbance_lqr_dob/raw_data.npz")
t  = lqr["t"]; sl = lqr["state"]; sd = dob["state"]

Omega = 0.2; Rc = 2.0
xr_all = Rc*np.cos(Omega*t); yr_all = Rc*np.sin(Omega*t)
# precompute |position error| at every step
eL = np.hypot(sl[:,0]-xr_all, sl[:,1]-yr_all)
eD = np.hypot(sd[:,0]-xr_all, sd[:,1]-yr_all)

step = 6
idx = np.arange(0, len(t), step)
T = t[idx]; SL = sl[idx]; SD = sd[idx]; EL = eL[idx]; ED = eD[idx]

theta = np.linspace(0, 2*np.pi, 200)
xc = 2*np.cos(theta); yc = 2*np.sin(theta)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.2),
                                gridspec_kw={"width_ratios":[1, 1.15]})
# left: trajectory
ax1.plot(xc, yc, "k:", lw=1, alpha=0.4, label="ideal circle")
ax1.set_xlim(-2.6, 2.6); ax1.set_ylim(-2.6, 2.6)
ax1.set_aspect("equal"); ax1.grid(True, alpha=0.3)
ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]")
ax1.set_title("Trajectory", fontsize=12, fontweight="bold")

trail_L, = ax1.plot([], [], "C0-", lw=1.5, alpha=0.85, label="LQR")
trail_D, = ax1.plot([], [], "C1-", lw=1.5, alpha=0.85, label="LQR+DOB")
robot_L, = ax1.plot([], [], "C0o", ms=9)
robot_D, = ax1.plot([], [], "C1o", ms=9)
ref_m,   = ax1.plot([], [], "k*", ms=11, label="reference")
ax1.legend(loc="lower right", fontsize=9)

# right: error vs time
ax2.set_xlim(0, 30); ax2.set_ylim(0, 0.25)
ax2.grid(True, alpha=0.3)
ax2.set_xlabel("t [s]"); ax2.set_ylabel("|position error| [m]")
ax2.set_title("Position error vs time", fontsize=12, fontweight="bold")
ax2.axvline(8, color="darkred", linestyle="--", lw=1.2, alpha=0.7)
ax2.text(8.2, 0.23, "step at t=8s", color="darkred", fontsize=9)
ax2.axhline(0.108, color="C0", linestyle=":", lw=1, alpha=0.5)
ax2.text(20, 0.115, "LQR steady-state 0.11 m", color="C0", fontsize=9)

err_L, = ax2.plot([], [], "C0-", lw=2.0, label="LQR")
err_D, = ax2.plot([], [], "C1-", lw=2.0, label="LQR+DOB")
dot_L,  = ax2.plot([], [], "C0o", ms=7)
dot_D,  = ax2.plot([], [], "C1o", ms=7)
ax2.legend(loc="upper right", fontsize=10)

time_text = fig.text(0.5, 0.96, "", ha="center", fontsize=14, fontweight="bold")

def init():
    for art in [trail_L, trail_D, robot_L, robot_D, ref_m, err_L, err_D, dot_L, dot_D]:
        art.set_data([], [])
    return ()

def update(i):
    tnow = T[i]
    xr = Rc*np.cos(Omega*tnow); yr = Rc*np.sin(Omega*tnow)
    trail_L.set_data(SL[:i+1, 0], SL[:i+1, 1])
    trail_D.set_data(SD[:i+1, 0], SD[:i+1, 1])
    robot_L.set_data([SL[i, 0]], [SL[i, 1]])
    robot_D.set_data([SD[i, 0]], [SD[i, 1]])
    ref_m.set_data([xr], [yr])
    err_L.set_data(T[:i+1], EL[:i+1])
    err_D.set_data(T[:i+1], ED[:i+1])
    dot_L.set_data([T[i]], [EL[i]])
    dot_D.set_data([T[i]], [ED[i]])
    time_text.set_text(f"t = {tnow:5.2f} s    "
                       f"|err| LQR={EL[i]:.3f} m   LQR+DOB={ED[i]:.3f} m")
    return ()

ani = FuncAnimation(fig, update, frames=len(T), init_func=init,
                    blit=False, interval=40)
print(f"Frames: {len(T)}")
ani.save(OUT, writer=PillowWriter(fps=25), dpi=90)
print(f"Done: {OUT}")
