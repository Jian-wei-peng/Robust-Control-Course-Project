# Course Report (LaTeX, IEEEtran)

This directory contains the LaTeX source for the *Robust Control* course
project report. The template is **IEEEtran (conference)** — the standard for
controls coursework, double-column, 9–10 pages.

## Files

| File         | Purpose                                                  |
|--------------|----------------------------------------------------------|
| `main.tex`   | Top-level document. Already pre-filled with section      |
|              | skeleton tailored to the LQR vs. LQR+DOB project.        |
| `refs.bib`   | BibTeX database. Add new references here.                |
| `README.md`  | This file.                                               |

## Build

### One-shot (latexmk, recommended)

```bash
cd report
latexmk -pdf main.tex          # builds main.pdf; rebuilds on subsequent runs
latexmk -c                     # cleans aux files (keeps main.pdf)
```

### Manual

```bash
cd report
pdflatex main
bibtex   main
pdflatex main
pdflatex main
```

The standard *texlive-full* (or MiKTeX on Windows) ships with IEEEtran and
all packages used here — no manual installation needed.

## Sections (matches the assignment brief)

1. **Introduction** — `\section{Introduction}` in `main.tex`. Background +
   contributions + outline.
2. **Literature review** — `\section{Related Work}`. LQR, DOBC, sliding
   mode, H∞, tube MPC.
3. **Methodology** — `\section{System Modeling and Problem Formulation}`
   + `\section{Controller Design and Stability Analysis}`. Includes the
   Riccati equation, augmented-state observer, Lyapunov stability proof
   sketch, and the observer pole-placement rule.
4. **Case study / simulation** — `\section{Simulation Case Study}`. Five
   scenarios, metric table, observer-speed sweep, discussion. Figures pull
   directly from `../results/figures/` and `../results/raw/`.
5. **References** — `\bibliography{refs}` (IEEEtran style).

## Including figures from `results/`

The template references images via relative paths:

```tex
\includegraphics[width=0.95\linewidth]{../results/figures/comparison/piecewise_trajectory_compare.png}
```

This works as long as you compile from inside `report/`. If you prefer a
flat layout, copy the PNGs into `report/figs/` and update the paths.

## Switching to Chinese

If you need to write the report in Chinese, uncomment in `main.tex`:

```tex
\usepackage{ctex}
```

and compile with `xelatex` instead of `pdflatex`:

```bash
latexmk -xelatex main.tex
```

## Status

All major content is already filled in based on the actual simulation
outputs:

- Authors: Jianwei Peng, Han Yang, Ziling Lu
- Abstract: real RMSE/ITAE reduction percentages
- Section III: closed-form + numerical $(A,B)$ matrices
- Section IV: LQR closed-loop eigenvalues, observer pole-placement rule
- Section V: all metric tables filled from `results/tables/`
- Appendix A: full derivation of $(A,B)$
- Appendix B: full Lyapunov proof for both constant and time-varying $d$

Five figures are embedded under `figs/`:
- `summary_metrics.png` — bar chart of all metrics × all scenarios
- `dob_sweep.png` — observer-speed sweep
- `piecewise_disturbance_trajectory_compare.png` — LQR vs LQR+DOB tracks
- `step_disturbance_position_error_compare.png` — steady-state error
- `piecewise_disturbance_d_hat.png` — DOB estimate vs true $d$

## Last-mile customisation

- Update `\IEEEauthorblockA{...}` department / email to your actual values
- Adjust the Discussion section (V-F) if any reviewer comments come back
- Re-run experiments via `python experiments/run_all.py` if you change
  any YAML; figures and tables are then refreshed in-place
