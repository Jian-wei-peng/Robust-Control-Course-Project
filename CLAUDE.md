# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

A simulation study for the *Robust Control* course: trajectory tracking of a differential-drive mobile robot using **LQR** (baseline) versus **LQR + Disturbance Observer (DOB)** under several uncertainty scenarios (step / piecewise / noisy disturbances, payload and friction changes). All simulation is in Python on NumPy/SciPy; there are no hardware bindings, no tests, and no build system.

The course assignment brief lives in `鲁棒控制project.md` (Chinese) — it defines the problem, the linearized error model, the LQR weighting, the augmented-state DOB design, the five scenarios, and the metrics. **Read it first** when modifying controller math or scenario definitions; the code is meant to mirror its formulation.

## Running Experiments

Always run experiment scripts from the **repo root** (they put `src/` on `sys.path` via `_common.py`):

```bash
pip install -r requirements.txt    # numpy scipy matplotlib pandas pyyaml

# Single scenario (one of nominal / step / piecewise / payload / noise)
python experiments/run_nominal.py
python experiments/run_step_disturbance.py
python experiments/run_piecewise_disturbance.py
python experiments/run_payload_change.py
python experiments/run_noise_disturbance.py

# All five scenarios + summary CSV + bar chart
python experiments/run_all.py

# DOB observer-speed sweep (speed_factor ∈ {2,3,5,8,12} on piecewise_disturbance)
python experiments/run_dob_sweep.py
```

The README suggests the conda env `pyrobot` (`conda run -n pyrobot python experiments/run_all.py`).

Each scenario script runs **both controllers** (`lqr` and `lqr_dob`) by default — see `experiments/_common.py::run_scenario`. To run just one, pass `controllers=["lqr_dob"]`.

### Where outputs land

- `results/raw/<scenario>_<controller>/` — `raw_data.npz`, `config.yaml`, `system_checks.json`, plus per-run PNG plots
- `results/tables/<scenario>_metrics.csv` — RMSE / ITAE / peak error / control effort / disturbance-estimate RMSE
- `results/tables/summary_metrics.csv` + `results/figures/summary_metrics.png` — aggregated across all scenarios (produced by `run_all.py`)
- `results/figures/comparison/` — LQR vs LQR+DOB overlay plots per scenario
- `results/figures/dob_sweep.png` — observer-speed sweep result

Reruns overwrite previous outputs in-place.

## Configuration Layering

There is no CLI for tuning — everything is YAML. `load_config(default, scenario)` deep-merges a scenario file *on top of* `configs/default.yaml`:

- `configs/default.yaml` — simulation horizon, ODE tolerances, **nominal** robot params (`m0, I0, d_v0, d_omega0`), circular reference, LQR `Q`/`R`, DOB `speed_factor`, initial state, output dirs
- `configs/<scenario>.yaml` — only the `scenario:` block, declaring `disturbance` (`none|step|piecewise|piecewise_plus_noise`), optional `payload` and `friction` changes with `change_time`

To create a new scenario: drop a YAML in `configs/` defining just the `scenario:` block, then either write a one-liner `experiments/run_<name>.py` (see `run_nominal.py` for the template) or call `run_scenario("<name>.yaml")` directly.

## Architecture: how a single simulation flows

The runtime path is linear and entirely driven by `run_simulation(cfg, controller_name)` in `src/robust_robot/simulation.py`. Understanding this function is the key to the codebase.

1. **Linearization** (`dynamics.build_linear_error_model`) — builds the 5-state error model `(x_e, y_e, theta_e, v_e, omega_e)` linearized around the nominal circular reference (constant `v_r, omega_r`). Returns `(A, B)`.
2. **LQR design** (`lqr.solve_lqr`) — `solve_continuous_are` → gain `K`, cost `P`, closed-loop `A_K = A - B@K`.
3. **DOB design** (only when `controller_name == "lqr_dob"`, `dob.build_augmented_observer_model` + `design_dob_gain`) — augments the error state with a 2-dim disturbance integrator, picks observer poles via `choose_observer_poles(A_K, speed_factor)` (faster than the slowest stable LQR mode by `speed_factor`×), and solves `place_poles` (Tits–Yang) to get the 7×5 gain `L`.
4. **ODE integration** (`solve_ivp` with `t_eval = arange(0, tf+dt/2, dt)`) — the integrated state is either the 5-d robot state (LQR) or 5 + 7 = 12-d (LQR+DOB, where the 7 trailing dims are the augmented observer state `x_a_hat`). The vector field:
   - reads `xi = tracking_error(state, ref)` (with `theta` wrapped to `[-π, π]`),
   - computes torque `tau = tau_ff(ref) + delta_tau` where `delta_tau = -u` and `u = -K@xi` (LQR) or `u = -K@xi - d_hat` (LQR+DOB),
   - evaluates **true** `(M, D)` at time `t` via `M_D_at` — these differ from nominal when payload/friction switches activate,
   - evaluates **true** disturbance `tau_d_at(t)` (step / piecewise / + Gaussian noise sampled on a fixed `Ts` grid via `NoiseSignal`),
   - integrates `robot_dynamics` (the full nonlinear model) plus, for DOB, `observer_dynamics(x_a_hat, y=xi, u, A_a, B_a, C_a, L)`.
5. **Post-pass** — re-evaluates per-timestep histories of `tau`, `delta_tau`, `tau_d`, and the equivalent matched disturbance `d` (`scenarios.equivalent_disturbance` — projects mismatches in `(M, D, tau_d)` onto the input channel using the **nominal** `(M0, D0)`; this is what `d_hat` is supposed to track). Returns a `SimulationResult` dataclass with everything.

### Things that are easy to miss

- **Two disturbance vectors**: `tau_d` is the *external* torque disturbance you set in YAML; `d` is the *equivalent matched* disturbance in the nominal-model input channel — only `d` (not `tau_d`) is comparable to `d_hat`. The DOB estimates `d`.
- **Sign convention**: `u = -K@xi`, then `delta_tau = -u`. Don't "simplify" — the sign aligns the LQR derivation in the project brief with the feed-forward subtraction.
- **Observer pole selection** depends on the LQR closed-loop spectrum. Don't pole-place the DOB before solving the ARE.
- **Angle wrapping** happens both inside the ODE (each evaluation) and in the post-pass — if you add new error states, mirror this.
- **Noise reproducibility**: `make_noise_signal` consumes a `Generator` seeded by `cfg.simulation.seed`; only `piecewise_plus_noise` uses it. Changing `dt` doesn't change the noise samples (they're on their own `noise_sample_time` grid).
- **Friction switches use `delta_d_v / delta_d_omega`** added to nominal at `change_time` — they don't *replace* the nominal damping. Same for `delta_m, delta_I`.

## Module map (`src/robust_robot/`)

- `dynamics.py` — reference trajectory, nominal `(M0, D0)`, error coordinates, linearized `(A, B)`, feed-forward `tau_ff`, full nonlinear `robot_dynamics`
- `lqr.py` — controllability check, `solve_continuous_are`-backed LQR
- `dob.py` — augmented `(A_a, B_a, C_a)`, observability check, pole-placement gain `L`, observer ODE
- `controllers.py` — the two control laws (`compute_lqr_tau`, `compute_lqr_dob_tau`)
- `scenarios.py` — disturbance signal builders (`tau_d_at`), time-varying `M_D_at`, matched-disturbance projection `equivalent_disturbance`, noise sampling
- `simulation.py` — orchestrates everything above; defines `SimulationResult`
- `analysis.py` — metric definitions (RMSE / ITAE / peak / effort / disturbance-RMSE) and the `.npz` saver
- `plotting.py` — per-run standard plots, scenario-level LQR-vs-DOB comparison plots, summary bar chart, DOB sweep chart
- `config.py` — YAML loading + deep-merge, seed handling, output-dir creation

## Conventions for changes

- The brief and code use `eta = [v, omega]` (body-frame velocities), `q = [x, y, theta]` (pose), `xi = [x_e, y_e, theta_e, v_e, omega_e]` (5-d error state). Keep these names.
- New scenarios should keep the same `scenario.name` field — it's used in output directory naming and metric tables.
- When adding a new metric, extend `analysis.summarize_metrics` so it shows up in every CSV automatically.
- Plots write through `_save_show` with `show=False` by default (headless-friendly); `MPLCONFIGDIR` is forced to `.matplotlib_cache/` by `_common.py` to avoid `$HOME` writes.
