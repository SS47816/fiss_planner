"""Sampling-based Frenet trajectory planners: FOP, FOP+, FISS and FISS+.

The heavy lifting happens in the C++ core (``fiss_planner._core``); this
package re-exports it and adds dataset/simulator adapters and benchmarks::

    import fiss_planner as fp

    ref = fp.ReferenceLine(x, y)
    corridor = fp.Corridor.parallel_lanes(0.0, ref.length, 3.5, 1, 1)
    planner = fp.make_planner(fp.PlannerConfig())
    result = planner.plan(fp.FrenetState(s=0.0, s_d=10.0), ref, corridor, obstacles, target_speed=13.0)
"""

from ._core import *  # noqa: F401,F403
from ._core import PlannerConfig, PlannerType, planner_type_from_string

__version__ = "1.1.0"


def make_config(planner: "str | PlannerType" = "fiss_plus", **sampling) -> PlannerConfig:
    """Planner configuration with the given type and optional sampling overrides,
    e.g. ``make_config("fiss", num_width=7)``."""
    cfg = PlannerConfig()
    cfg.type = planner_type_from_string(planner) if isinstance(planner, str) else planner
    for key, value in sampling.items():
        if not hasattr(cfg.sampling, key):
            raise AttributeError(f"unknown sampling option '{key}'")
        setattr(cfg.sampling, key, value)
    return cfg
