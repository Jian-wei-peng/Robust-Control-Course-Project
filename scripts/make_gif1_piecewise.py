"""GIF 1: Side-by-side LQR vs LQR+DOB under piecewise disturbance."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT = "/home/yh/桌面/DocumentSync/学习文档/鲁棒控制基础/project/Robust-Control-Course-Project"
OUT = f"{ROOT}/slides/gifs/piecewise_side_by_side.gif"

lqr = np.load(f"{ROOT}/results/raw/piecewise_disturbance_lqr/raw_data.npz")
dob = np.load(f"{ROOT}/results/raw/piecewise_disturbance_lqr_dob/raw_data.npz")
t  = lqr["t"]; sl = lqr["state"]; sd = dob["state"]

# subsample for GIF: every 6th sample (5x speedup at 25fps → 6s GIF for 30s sim)
step = 6
idx = np.arange(0, len(t), step)
T = t[idx]
SL = sl[idx]; SD = sd[idx]

# reference full circle for visual frame
theta = np.linspace(0, 2*np.pi, 200)
xc = 2*np.cos(theta); yc = 2*np.sin(theta)

# disturbance segment labels for piecewise: [0,5):0  [5,10):d1  [10,15):d2  [15,30):d3
def dist_label(time):
    if time < 5.0:  return "$\\tau_d = (0, 0)$"
    if time < 10.0: return "$\\tau_d = (0.4, 0)$"
    if time < 15.0: return "$\\tau_d = (-0.2, 0.1)$"
    return "$\\tau_d = (0.2, -0.15)$"

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5.5))
for ax, ctrl in zip([ax1, ax2], ["Pure LQR", "LQR + DOB"]):
    ax.plot(xc, yc, "k:", lw=1, alpha=0.4)
    ax.set_xlim(-2.6, 2.6); ax.set_ylim(-2.6, 2.6)
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(ctrl, fontsize=13, fontweight="bold")

# initial reference position is computed from circle param x=R*cos(Omega*t), y=R*sin(Omega*t)
Omega = 0.2; Rc = 2.0

# objects to animate
trail_lqr,  = ax1.plot([], [], "C0-", lw=2.0, alpha=0.85)
robot_lqr,  = ax1.plot([], [], "C0o", ms=10)
ref_lqr,    = ax1.plot([], [], "k*", ms=12)
trail_dob,  = ax2.plot([], [], "C1-", lw=2.0, alpha=0.85)
robot_dob,  = ax2.plot([], [], "C1o", ms=10)
ref_dob,    = ax2.plot([], [], "k*", ms=12)

time_text   = fig.text(0.5, 0.95, "", ha="center", fontsize=14, fontweight="bold")
dist_text   = fig.text(0.5, 0.91, "", ha="center", fontsize=11, color="darkred")
err1_text   = ax1.text(0.02, 0.97, "", transform=ax1.transAxes, fontsize=10,
                       verticalalignment="top",
                       bbox=dict(boxstyle="round", facecolor="white", alpha=0.85))
err2_text   = ax2.text(0.02, 0.97, "", transform=ax2.transAxes, fontsize=10,
                       verticalalignment="top",
                       bbox=dict(boxstyle="round", facecolor="white", alpha=0.85))

# legends
ax1.legend([trail_lqr, robot_lqr, ref_lqr], ["LQR trace", "robot", "reference"],
           loc="lower right", fontsize=9)
ax2.legend([trail_dob, robot_dob, ref_dob], ["LQR+DOB trace", "robot", "reference"],
           loc="lower right", fontsize=9)

def init():
    for line in [trail_lqr, robot_lqr, ref_lqr, trail_dob, robot_dob, ref_dob]:
        line.set_data([], [])
    return trail_lqr, robot_lqr, ref_lqr, trail_dob, robot_dob, ref_dob, time_text, dist_text, err1_text, err2_text

def update(i):
    tnow = T[i]
    xr = Rc*np.cos(Omega*tnow); yr = Rc*np.sin(Omega*tnow)
    # trail (history up to i)
    trail_lqr.set_data(SL[:i+1, 0], SL[:i+1, 1])
    trail_dob.set_data(SD[:i+1, 0], SD[:i+1, 1])
    # current robot position
    robot_lqr.set_data([SL[i, 0]], [SL[i, 1]])
    robot_dob.set_data([SD[i, 0]], [SD[i, 1]])
    ref_lqr.set_data([xr], [yr])
    ref_dob.set_data([xr], [yr])
    # text
    time_text.set_text(f"t = {tnow:5.2f} s")
    dist_text.set_text(f"disturbance:  {dist_label(tnow)}")
    eL = np.hypot(SL[i,0]-xr, SL[i,1]-yr)
    eD = np.hypot(SD[i,0]-xr, SD[i,1]-yr)
    err1_text.set_text(f"|err| = {eL:.3f} m")
    err2_text.set_text(f"|err| = {eD:.3f} m")
    return trail_lqr, robot_lqr, ref_lqr, trail_dob, robot_dob, ref_dob, time_text, dist_text, err1_text, err2_text

ani = FuncAnimation(fig, update, frames=len(T), init_func=init,
                    blit=False, interval=40)  # ~25 fps
print(f"Frames: {len(T)} | Writing {OUT} ...")
ani.save(OUT, writer=PillowWriter(fps=25), dpi=90)
print("Done.")
