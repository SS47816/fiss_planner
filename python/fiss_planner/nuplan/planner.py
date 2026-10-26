"""The planner as a nuPlan planner (``nuplan.planning.simulation.planner.AbstractPlanner``).

Use it in nuPlan simulations through the Hydra config in
``benchmarks/nuplan/config/simulation/planner/fiss_planner.yaml`` (see
``benchmarks/nuplan/README.md``), or directly::

    planner = FissPlanner(profile="nuplan")
"""

from __future__ import annotations

import json
import os
import time

from nuplan.planning.simulation.observation.observation_type import DetectionsTracks, Observation
from nuplan.planning.simulation.planner.abstract_planner import (
    AbstractPlanner,
    PlannerInitialization,
    PlannerInput,
)
from nuplan.planning.simulation.trajectory.abstract_trajectory import AbstractTrajectory
from nuplan.planning.simulation.trajectory.interpolated_trajectory import InterpolatedTrajectory

import fiss_planner as fp

from . import adapter


class FissPlanner(AbstractPlanner):
    """LocalPlanner on the nuPlan map around the ego, along the mission's
    roadblocks, with red lights as stop lines and tracked objects as obstacles.

    `profile`: planner profile (fiss_planner.profiles name or YAML file);
    `planner`: overrides its planner type unless None; `map_radius`: lanes
    within this distance of the start [m] are loaded, besides the route's;
    `obstacle_range`: objects farther from the ego are ignored [m]."""

    requires_scenario = False

    def __init__(
        self,
        profile: str = "nuplan",
        planner: str | None = None,
        prediction: str = "lane_following",
        map_radius: float = 200.0,
        obstacle_range: float = 100.0,
    ):
        self._profile = profile
        self._planner_type = planner
        self._prediction = prediction
        self._map_radius = map_radius
        self._obstacle_range = obstacle_range
        self._init: PlannerInitialization | None = None
        self._lp: fp.LocalPlanner | None = None
        self.stats: list[dict] = []

    _RUNTIME = ("_init", "_lp", "_cfg", "_vehicle", "_lanes", "_road", "_ids")

    def __getstate__(self) -> dict:
        # nuPlan pickles the planner with the simulation log: keep the settings, not the scene.
        return {k: v for k, v in self.__dict__.items() if k not in self._RUNTIME}

    def __setstate__(self, state: dict) -> None:
        self.__dict__.update(state)
        self._init, self._lp = None, None

    def name(self) -> str:
        return "fiss_planner"

    def observation_type(self) -> type[Observation]:
        return DetectionsTracks  # type: ignore

    def initialize(self, initialization: PlannerInitialization) -> None:
        self._init = initialization
        self._lp = None
        self._ids = adapter.ObstacleIds()
        self.stats = []

    def _setup(self, ego) -> None:
        init = self._init
        cfg, vehicle = fp.profiles.load(self._profile)
        vehicle = adapter.vehicle_params(ego.car_footprint.vehicle_parameters, vehicle)
        if self._planner_type:
            cfg.planner.type = fp.planner_type_from_string(self._planner_type)
        self._cfg, self._vehicle = cfg, vehicle
        c = ego.center
        blocks = adapter.correct_route(init.map_api, init.route_roadblock_ids, c.x, c.y, c.heading)
        self._lanes = adapter.map_lanes(init.map_api, (c.x, c.y), self._map_radius, blocks)
        self._road = adapter.road_map_from_nuplan(self._lanes)
        self.route = adapter.route_lanes(self._lanes, self._road, blocks, c.x, c.y, c.heading)
        goal = init.mission_goal
        self._lp = fp.LocalPlanner(cfg, vehicle)
        self._lp.set_map(self._road, self.route, (goal.x, goal.y) if self.route else None)

    def compute_planner_trajectory(self, current_input: PlannerInput) -> AbstractTrajectory:
        ego, observation = current_input.history.current_state
        if self._lp is None:
            self._setup(ego)
        t0 = time.perf_counter()
        t_now = ego.time_point.time_s
        for sl_id, active in adapter.stop_line_states(self._road, current_input.traffic_light_data).items():
            self._lp.set_stop_line_active(sl_id, active)
        state = adapter.ego_state(ego)
        horizon = self._cfg.planner.sampling.max_t + 0.5
        obstacles = adapter.obstacles(
            observation.tracked_objects,
            self._ids,
            self._road,
            (state.x, state.y),
            horizon,
            0.1,
            self._obstacle_range,
            self._prediction,
        )
        out = self._lp.step(t_now, state, obstacles)
        # The tracker samples the plan at the simulation time, which can precede the time
        # stamps by microseconds: start the trajectory a millisecond early.
        t0_us = min(current_input.iteration.time_point.time_us, ego.time_point.time_us) - 1000
        states = adapter.trajectory_states(out.trajectory, ego, t0_us)
        if len(states) < 2:  # no plan at all: hold position
            states = adapter.trajectory_states(_standstill(state), ego, t0_us)
        self.stats.append(
            {
                "success": out.success,
                "fallback": out.fallback,
                "emergency": out.emergency,
                "plan_ms": out.stats.time_ms,
                "step_ms": 1000.0 * (time.perf_counter() - t0),
            }
        )
        if os.environ.get("FISS_NUPLAN_TRACE"):
            self._trace(t_now, state, out, obstacles)
        return InterpolatedTrajectory(states)

    def _trace(self, t_now: float, state, out, obstacles) -> None:
        """Append this cycle to $FISS_NUPLAN_TRACE/<route start>_<start time>.jsonl (debugging)."""
        if not hasattr(self, "_trace_path"):
            d = os.environ["FISS_NUPLAN_TRACE"]
            os.makedirs(d, exist_ok=True)
            self._trace_path = os.path.join(d, f"{self._init.route_roadblock_ids[0]}_{t_now:.1f}.jsonl")
            with open(self._trace_path, "w") as f:
                lanes = {lid: self._road.lane(lid).centerline.tolist() for lid in self._road.lane_ids()}
                f.write(
                    json.dumps(
                        {
                            "route": self.route,
                            "lanes": lanes,
                            "goal": [self._init.mission_goal.x, self._init.mission_goal.y],
                        }
                    )
                    + "\n"
                )
        tr = out.trajectory
        row = {
            "t": t_now,
            "x": state.x,
            "y": state.y,
            "yaw": state.yaw,
            "v": state.v,
            "mode": "E" if out.emergency else "F" if out.fallback else "D" if out.degraded else "P",
            "ego_lane": out.ego_lane,
            "target": out.target_speed,
            "bands": [
                [b.offset, b.lane_id, b.change_allowed, round(b.extra_cost, 2), round(b.urgency, 2)]
                for b in sorted(out.corridor.lanes, key=lambda b: b.offset)
            ],
            "rej": {k: v for k, v in out.stats.rejections.items() if v},
            "lead": [out.stats.lead.id, out.stats.lead.gap],
            "blocking": out.stats.blocking_id,
            "traj": [list(map(float, tr.x))[::5], list(map(float, tr.y))[::5], list(map(float, tr.v))[::5]],
            "obs": [
                [o.id, *(lambda b: [b.cx, b.cy, b.yaw, o.length, o.width])(o.box_at(0.0))]
                for o in obstacles
                if o.box_at(0.0) is not None
            ],
        }
        with open(self._trace_path, "a") as f:
            f.write(json.dumps(row) + "\n")


class _standstill:
    """A trajectory standing at the ego's position for 5 s."""

    def __init__(self, s: fp.EgoState):
        self.t = [0.0, 5.0]
        self.x, self.y, self.yaw = [s.x] * 2, [s.y] * 2, [s.yaw] * 2
        self.v = self.a = self.kappa = [0.0, 0.0]
