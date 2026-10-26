"""Planner profiles: one tuned configuration per benchmark or simulator.

A profile is a YAML file in the configuration format of
``load_local_planner_config`` that pins every planner setting, so a profile's
results do not move when a default changes. Shipped profiles::

    commonroad       CommonRoad scenarios (closed loop, recorded traffic)
    carla_scenarios  the CARLA closed-loop scenarios (fiss_planner.carla.closed_loop)

Each profile is tuned only on its benchmark's development split (see
``benchmarks/tune.py``)::

    cfg, vehicle = fp.profiles.load("commonroad", vehicle)
    cfg, vehicle = fp.profiles.override(cfg, vehicle, {"cost.w_speed": 2.0})
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

from .._core import LocalPlannerConfig, VehicleParams, load_local_planner_config, parse_local_planner_config
from .._core import to_yaml as _to_yaml

DIR = Path(__file__).resolve().parent


def names() -> list[str]:
    """The shipped profiles."""
    return sorted(p.stem for p in DIR.glob("*.yaml"))


def path(profile: str | Path) -> Path:
    """The file of a shipped profile (by name) or of a YAML file (by path)."""
    p = Path(profile)
    if p.suffix in (".yaml", ".yml") or p.exists():
        if not p.is_file():
            raise FileNotFoundError(f"profile file '{p}' does not exist")
        return p
    shipped = DIR / f"{profile}.yaml"
    if not shipped.is_file():
        raise ValueError(f"unknown profile '{profile}' (shipped: {', '.join(names())}; or a .yaml path)")
    return shipped


def name(profile: str | Path | None) -> str:
    """Short label of a profile for result tables: its file stem ('' for none)."""
    return Path(profile).stem if profile else ""


def load(
    profile: str | Path | None, vehicle: VehicleParams | None = None
) -> tuple[LocalPlannerConfig, VehicleParams]:
    """(config, vehicle) of a profile; `vehicle` (default: the defaults) keeps
    every field the profile does not set. ``None`` gives the core defaults."""
    vehicle = vehicle if vehicle is not None else VehicleParams()
    if profile is None:
        return LocalPlannerConfig(), vehicle
    return load_local_planner_config(str(path(profile)), vehicle)


def _scalar(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if math.isinf(value):
            return ".inf" if value > 0 else "-.inf"
        return repr(float(value))  # shortest exact text (also for numpy floats)
    return str(value)


def override(
    config: LocalPlannerConfig, vehicle: VehicleParams | None, values: dict
) -> tuple[LocalPlannerConfig, VehicleParams]:
    """A copy of (config, vehicle) with settings changed by their YAML keys,
    e.g. ``{"cost.w_speed": 2.0, "route.urgency_time": 8.0, "desired_speed": 12.0}``.
    Unknown keys and invalid values raise ValueError."""
    top, sections = [], {}
    for key, value in values.items():
        section, _, field = key.rpartition(".")
        if section:
            sections.setdefault(section, []).append(f"  {field}: {_scalar(value)}")
        else:
            top.append(f"{field}: {_scalar(value)}")
    text = "\n".join(top + [f"{s}:\n" + "\n".join(lines) for s, lines in sections.items()])
    vehicle = vehicle if vehicle is not None else VehicleParams()
    return parse_local_planner_config(text, vehicle, config)


def to_yaml(config: LocalPlannerConfig, vehicle: VehicleParams | None = None) -> str:
    """Every setting as profile YAML (with a vehicle section if `vehicle` is given)."""
    return _to_yaml(config, vehicle)


def config_hash(config: LocalPlannerConfig, vehicle: VehicleParams | None = None) -> str:
    """Short fingerprint of the settings that affect the plans (not
    ``planner.keep_candidates`` or ``planner.num_threads``), to tell results
    of different configurations apart."""
    cfg, _ = override(config, vehicle, {"planner.keep_candidates": False, "planner.num_threads": 1})
    return hashlib.sha256(_to_yaml(cfg, vehicle).encode()).hexdigest()[:12]
