# Robust Control Course Project

This project simulates robust trajectory tracking control for a differential
drive mobile robot using LQR with an augmented disturbance observer (DOB).

## Setup

```bash
pip install -r requirements.txt
```

If you use the existing conda environment:

```bash
conda run -n pyrobot python experiments/run_all.py
```

## Run Experiments

```bash
python experiments/run_nominal.py
python experiments/run_step_disturbance.py
python experiments/run_piecewise_disturbance.py
python experiments/run_payload_change.py
python experiments/run_noise_disturbance.py
python experiments/run_all.py
python experiments/run_dob_sweep.py
```

Results are saved under `results/raw`, `results/tables`, and `results/figures`.
