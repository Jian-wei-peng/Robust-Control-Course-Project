from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data or {}


def merge_dict(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_dict(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def load_config(default_path: str | Path, scenario_path: str | Path) -> dict[str, Any]:
    cfg = merge_dict(load_yaml(default_path), load_yaml(scenario_path))
    return cfg


def save_config(cfg: dict[str, Any], output_dir: str | Path) -> None:
    output_path = Path(output_dir) / "config.yaml"
    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)


def set_random_seed(cfg: dict[str, Any]) -> np.random.Generator:
    seed = int(cfg["simulation"].get("seed", 1))
    return np.random.default_rng(seed)


def make_output_dir(cfg: dict[str, Any], scenario_name: str, controller_name: str) -> Path:
    root_dir = Path(cfg["output"].get("root_dir", "results"))
    output_dir = root_dir / "raw" / f"{scenario_name}_{controller_name}"
    output_dir.mkdir(parents=True, exist_ok=True)
    (root_dir / "figures").mkdir(parents=True, exist_ok=True)
    (root_dir / "tables").mkdir(parents=True, exist_ok=True)
    return output_dir


def repo_path(*parts: str) -> Path:
    return Path(__file__).resolve().parents[2].joinpath(*parts)
