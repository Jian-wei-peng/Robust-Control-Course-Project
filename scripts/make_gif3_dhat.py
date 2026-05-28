"""GIF 3: DOB disturbance estimate tracking (piecewise scenario)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT = "/home/yh/桌面/DocumentSync/学习文档/鲁棒控制基础/project/Robust-Control-Course-Project"
OUT = f"{ROOT}/slides/gifs/dob_disturbance_estimate.gif"

dob = np.load(f"{ROOT}/results/raw/piecewise_disturbance_lqr_dob/raw_data.npz")
t   = dob["t"]
d     = dob["d"]      # equivalent matched disturbance (truth)
d_hat = dob["d_hat"]  # observer estimate

step = 6
idx = np.arange(0, len(t), step)
T = t[idx]; D = d[idx]; DH = d_hat[idx]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
for k, ax in enumerate([ax1, ax2]):
    ax.set_xlim(0, 30)
    ax.grid(True, alpha=0.3)
    for sw in [5, 10, 15]:
        ax.axvline(sw, color="grey", lw=0.8, alpha=0.5)
    ax.set_ylabel(f"$d_{k+1}$")
ax1.set_ylim(min(d[:,0].min(), -0.5)-0.1, max(d[:,0].max(), 0.5)+0.2)
ax2.set_ylim(min(d[:,1].min(), -0.5)-0.1, max(d[:,1].max(), 0.5)+0.2)
ax2.set_xlabel("t [s]")

ax1.set_title("Force-channel disturbance $d_1$", fontsize=11)
ax2.set_title("Torque-channel disturbance $d_2$", fontsize=11)

# truth curves (drawn over full horizon as ground-truth reference, faded)
ax1.plot(t, d[:,0], "k--", lw=1.2, alpha=0.35, label="ground truth $d_1$")
ax2.plot(t, d[:,1], "k--", lw=1.2, alpha=0.35, label="ground truth $d_2$")

# animated lines
dhat1_line, = ax1.plot([], [], "C1-", lw=2, label="DOB estimate $\\hat d_1$")
dhat2_line, = ax2.plot([], [], "C1-", lw=2, label="DOB estimate $\\hat d_2$")
dhat1_dot,  = ax1.plot([], [], "C1o", ms=7)
dhat2_dot,  = ax2.plot([], [], "C1o", ms=7)
true1_dot,  = ax1.plot([], [], "ks", ms=7)
true2_dot,  = ax2.plot([], [], "ks", ms=7)

ax1.legend(loc="upper right", fontsize=9)
ax2.legend(loc="upper right", fontsize=9)

time_text = fig.text(0.5, 0.97, "", ha="center", fontsize=13, fontweight="bold")

def init():
    for line in [dhat1_line, dhat2_line, dhat1_dot, dhat2_dot, true1_dot, true2_dot]:
        line.set_data([], [])
    return ()

def update(i):
    dhat1_line.set_data(T[:i+1], DH[:i+1, 0])
    dhat2_line.set_data(T[:i+1], DH[:i+1, 1])
    dhat1_dot.set_data([T[i]], [DH[i, 0]])
    dhat2_dot.set_data([T[i]], [DH[i, 1]])
    true1_dot.set_data([T[i]], [D[i, 0]])
    true2_dot.set_data([T[i]], [D[i, 1]])
    e1 = DH[i,0]-D[i,0]; e2 = DH[i,1]-D[i,1]
    time_text.set_text(f"t = {T[i]:5.2f} s    "
                       f"estimation error: $|\\tilde d_1|={abs(e1):.3f}$, "
                       f"$|\\tilde d_2|={abs(e2):.3f}$")
    return ()

ani = FuncAnimation(fig, update, frames=len(T), init_func=init,
                    blit=False, interval=40)
print(f"Frames: {len(T)}")
ani.save(OUT, writer=PillowWriter(fps=25), dpi=90)
print(f"Done: {OUT}")
