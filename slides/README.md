# Course Presentation (Beamer)

Slide deck for the Robust Control course project, built with Beamer +
metropolis theme. Mirrors the report (`../report/`) but condensed to
~18 slides for a 15-20 minute presentation.

## Files

| File          | Purpose                                          |
|---------------|--------------------------------------------------|
| `slides.tex`  | Top-level Beamer source. Self-contained.         |
| `figs/`       | 4 PNGs reused from `results/` (auto-copied).     |
| `README.md`   | This file.                                       |

## Build

### Overleaf (recommended)

Upload this folder (or the `slides-overleaf.zip` packaged at the repo
root) as a new Overleaf project. Set the compiler to `pdfLaTeX` and
click Recompile.

### Local

```bash
cd slides
latexmk -pdf slides.tex
# or:
pdflatex slides && pdflatex slides
```

Requires `texlive-full` (or the `beamer` + `metropolis` packages
specifically).

## Slide structure

| #  | Section          | Slide title                                |
|----|------------------|--------------------------------------------|
| 1  | (title)          | Title page                                 |
| 2  | (outline)        | Outline                                    |
| 3  | Motivation       | The trajectory-tracking problem            |
| 4  | Motivation       | Why plain LQR is not enough                |
| 5  | Related Work     | Spectrum of robust controllers             |
| 6  | System Model     | Differential-drive dynamics                |
| 7  | System Model     | Tracking error and linearisation           |
| 8  | Controller Design| Step 1 — Baseline LQR                      |
| 9  | Controller Design| Step 2 — Augmented-state DOB               |
| 10 | Controller Design| Step 3 — Observer pole placement           |
| 11 | Stability        | Closed-loop dynamics and Theorem 1         |
| 12 | Stability        | Proof sketch                               |
| 13 | Results          | Simulation setup                           |
| 14 | Results          | Trajectory tracking under piecewise        |
| 15 | Results          | Step disturbance position error (key)      |
| 16 | Results          | Disturbance estimate vs. truth             |
| 17 | Results          | Quantitative summary                       |
| 18 | Results          | Observer-speed sweep                       |
| 19 | Conclusion       | Key take-aways                             |
| 20 | Conclusion       | Limitations and future work                |
| 21 | (final)          | Thank you / Q&A                            |
| 22 | (appendix)       | References                                 |

## Theme fallback

If your installation lacks the `metropolis` theme, edit `slides.tex`:

```tex
% comment out
\usetheme{metropolis}
\metroset{block=fill,sectionpage=progressbar,subsectionpage=progressbar}

% uncomment
\usetheme{Madrid}
\usecolortheme{whale}
```
