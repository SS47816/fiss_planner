"""The planner as a CARLA Leaderboard 2.0 agent (Bench2Drive), with privileged
inputs: the map around the route, the other actors and the traffic lights come
from the simulator (as for the rule-based expert PDM-Lite), not from sensors.

    python leaderboard/leaderboard/leaderboard_evaluator.py \\
        --agent=<repo>/python/fiss_planner/carla/leaderboard_agent.py --agent-config=bench2drive ...

`--agent-config` names the planner profile (a fiss_planner.profiles name or a
YAML file); see benchmarks/bench2drive/README.md.
"""

from __future__ import annotations

import math
import os

import carla
import numpy as np
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import fiss_planner as fp
from fiss_planner.carla import adapter
from fiss_planner.carla.closed_loop import Actuation

_STOP_STATES = (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow)


_COST_TERMS = ("time", "speed", "lateral", "lane_change", "proximity", "accel", "jerk")


def _best_per_lane(candidates) -> dict:
    """{lane offset: [total cost, end speed, horizon, cost terms...] of its best valid candidate} (trace)."""
    best = {}
    for c in candidates:
        if not str(c.rejection).endswith("OK"):
            continue
        row = [round(c.total_cost(), 2), round(c.end.s_d, 1), round(c.end.T, 1)]
        row += [round(getattr(c.cost, k), 2) for k in _COST_TERMS]
        if str(c.lane) not in best or row[0] < best[str(c.lane)][0]:
            best[str(c.lane)] = row
    return best


def get_entry_point() -> str:
    return "FissAgent"


class FissAgent(AutonomousAgent):
    """Plans at 10 Hz along the route with the LocalPlanner and tracks the plan
    with the MPC controller at the simulator's 20 Hz."""

    map_radius = 80.0  # [m] lanes this close to the route are mapped
    obstacle_range = 80.0  # [m]
    plan_every = 2  # ticks (20 Hz control, 10 Hz planning)
    mpc_min_speed = 1.5  # [m/s] below this the Stanley/PID controller tracks the plan

    def setup(self, path_to_conf_file):
        self.track = Track.SENSORS
        # Bench2Drive appends "+<run name>" to the agent configuration.
        self._profile = (path_to_conf_file or "").split("+")[0] or "bench2drive"
        self._lp = None
        self._tick = 0
        self._traj, self._traj_t0 = None, 0.0
        # (set_global_plan may come first: keep its route)

    def sensors(self):
        return [{"type": "sensor.speedometer", "reading_frequency": 20, "id": "speed"}]

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        super().set_global_plan(global_plan_gps, global_plan_world_coord)
        self._route = [tf for tf, _ in global_plan_world_coord]

    def _setup_planner(self) -> None:
        self._ego = CarlaDataProvider.get_hero_actor()
        world = CarlaDataProvider.get_world()
        carla_map = CarlaDataProvider.get_map()
        pts = np.asarray([(tf.location.x, tf.location.y) for tf in self._route])
        self._road, key_to_id = adapter.road_map_from_carla(carla_map, 2.0, (pts, self.map_radius))
        lights = [
            tl
            for tl in world.get_actors().filter("traffic.traffic_light*")
            if np.min(np.hypot(pts[:, 0] - tl.get_location().x, pts[:, 1] - tl.get_location().y))
            < self.map_radius + 50.0
        ]
        self._lights = adapter.traffic_light_stop_lines(self._road, key_to_id, lights)
        signs = [
            s
            for s in world.get_actors().filter("traffic.stop")
            if np.min(np.hypot(pts[:, 0] - s.get_location().x, pts[:, 1] - s.get_location().y))
            < self.map_radius + 50.0
        ]
        adapter.stop_sign_stop_lines(self._road, key_to_id, carla_map, signs, 1 + len(self._lights))
        route_wps = [carla_map.get_waypoint(tf.location) for tf in self._route]
        route = adapter.route_lane_ids(route_wps, key_to_id)
        cfg, vehicle = fp.profiles.load(self._profile, adapter.vehicle_params(self._ego))
        cfg.planner.keep_candidates = bool(os.environ.get("FISS_B2D_TRACE"))
        self._cfg, self._vehicle = cfg, vehicle
        self._lp = fp.LocalPlanner(cfg, vehicle)
        goal = adapter.to_planner(self._route[-1].location.x, self._route[-1].location.y)
        self._lp.set_map(self._road, route, goal[:2])
        self._ctrl = fp.make_controller("mpc", vehicle)
        # The kinematic MPC degenerates near standstill (steering has no effect there):
        # pulling away and creeping use Stanley steering and PID speed control.
        self._slow_ctrl = fp.make_controller("stanley_pid", vehicle)
        self._actuation = Actuation(0.05)
        self._world = world

    def run_step(self, input_data, timestamp):
        if self._lp is None:
            self._setup_planner()
        t = float(timestamp)
        ego_s = adapter.ego_state(self._ego)
        if self._tick % self.plan_every == 0:
            for sid, tl in self._lights.items():
                state = tl.get_state()
                self._lp.set_stop_line_active(
                    sid, state in _STOP_STATES, state == carla.TrafficLightState.Yellow
                )
            obs = adapter.obstacles(
                self._world,
                self._ego.id,
                self._cfg.planner.sampling.max_t + 0.5,
                0.1,
                max_range=self.obstacle_range,
                ego_xy=(ego_s.x, ego_s.y),
                road_map=self._road,
                ego_z=self._ego.get_location().z,
            )
            out = self._lp.step(t, ego_s, obs)
            if len(out.trajectory) >= 2:
                self._traj, self._traj_t0 = out.trajectory, t
            if os.environ.get("FISS_B2D_TRACE"):
                self._trace(t, ego_s, out, obs)
        self._tick += 1
        if self._traj is None:
            return carla.VehicleControl(brake=1.0)
        ctrl = self._ctrl if ego_s.v >= self.mpc_min_speed else self._slow_ctrl
        cmd = ctrl.compute(ego_s, self._traj, t - self._traj_t0)
        v_ref = self._traj.sample(t - self._traj_t0 + 0.3).v
        v_ahead = max(self._traj.sample(t - self._traj_t0 + h).v for h in (0.3, 0.6, 1.0))
        control = self._actuation.control(cmd, v_ref, ego_s.v, self._vehicle.max_steer, v_ahead)
        self._last = (cmd.accel, cmd.steer, v_ref, control.throttle, control.brake, control.steer)
        return control

    def _trace(self, t, ego_s, out, obs) -> None:
        """Append this cycle to $FISS_B2D_TRACE/<town>_<start>.jsonl (debugging)."""
        import json

        if not hasattr(self, "_trace_path"):
            d = os.environ["FISS_B2D_TRACE"]
            os.makedirs(d, exist_ok=True)
            name = CarlaDataProvider.get_map().name.split("/")[-1]
            self._trace_path = os.path.join(d, f"{name}_{int(self._route[0].location.x)}.jsonl")
            route = [adapter.to_planner(tf.location.x, tf.location.y)[:2] for tf in self._route]
            lanes = {i: self._road.lane(i).centerline.tolist() for i in self._road.lane_ids()}
            with open(self._trace_path, "w") as f:
                v = self._vehicle
                vehicle = {
                    k: getattr(v, k)
                    for k in ("length", "width", "wheelbase", "rear_axle_to_center", "max_steer")
                }
                stops = [
                    [sl.id, sl.lane_id, sl.s, self._road.lane(sl.lane_id).length()]
                    for sl in self._road.stop_lines
                ]
                header = {"route": route, "lanes": lanes, "vehicle": vehicle, "stop_lines": stops}
                f.write(json.dumps(header) + "\n")
        tr = out.trajectory
        row = {
            "t": t,
            "x": ego_s.x,
            "y": ego_s.y,
            "yaw": ego_s.yaw,
            "v": ego_s.v,
            "mode": "E" if out.emergency else "F" if out.fallback else "D" if out.degraded else "P",
            "ego_lane": out.ego_lane,
            "target": out.target_speed,
            "rej": {k: v for k, v in out.stats.rejections.items() if v},
            "lead": [out.stats.lead.id, out.stats.lead.gap],
            "blocking": out.stats.blocking_id,
            "red": [sid for sid, tl in self._lights.items() if tl.get_state() in _STOP_STATES],
            "behavior": list(out.behavior),
            "lane_best": _best_per_lane(out.candidates),
            "lane_rej": [[c.lane, str(c.rejection).split(".")[-1], c.blocking_id] for c in out.candidates],
            "bands": [
                [b.offset, b.lane_id, b.change_allowed, round(b.extra_cost, 2)]
                for b in sorted(out.corridor.lanes, key=lambda b: b.offset)
            ],
            "control": list(getattr(self, "_last", ())),
            "types": {
                o.id: getattr(self._world.get_actor(o.id), "type_id", "?")
                for o in obs
                if o.box_at(0.0) is not None
                and math.hypot(o.box_at(0.0).cx - ego_s.x, o.box_at(0.0).cy - ego_s.y) < 30
            },
            "traj": [list(map(float, tr.x))[::5], list(map(float, tr.y))[::5], list(map(float, tr.v))[::5]],
            "obs": [
                [o.id, *(lambda b: [b.cx, b.cy, b.yaw, o.length, o.width])(o.box_at(0.0))]
                for o in obs
                if o.box_at(0.0) is not None
            ],
        }
        with open(self._trace_path, "a") as f:
            f.write(json.dumps(row) + "\n")

    def destroy(self):
        self._lp = None
