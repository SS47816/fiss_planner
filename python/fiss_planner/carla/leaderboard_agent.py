"""The planner as a CARLA Leaderboard 2.0 agent (Bench2Drive), with privileged
inputs: the map around the route, the other actors and the traffic lights come
from the simulator (as for the rule-based expert PDM-Lite), not from sensors.

    python leaderboard/leaderboard/leaderboard_evaluator.py \\
        --agent=<repo>/python/fiss_planner/carla/leaderboard_agent.py --agent-config=bench2drive ...

`--agent-config` names the planner profile (a fiss_planner.profiles name or a
YAML file); see benchmarks/bench2drive/README.md.
"""

from __future__ import annotations

import carla
import numpy as np
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import fiss_planner as fp
from fiss_planner.carla import adapter
from fiss_planner.carla.closed_loop import Actuation


def get_entry_point() -> str:
    return "FissAgent"


class FissAgent(AutonomousAgent):
    """Plans at 10 Hz along the route with the LocalPlanner and tracks the plan
    with the MPC controller at the simulator's 20 Hz."""

    map_radius = 80.0  # [m] lanes this close to the route are mapped
    obstacle_range = 80.0  # [m]
    plan_every = 2  # ticks (20 Hz control, 10 Hz planning)

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
        route_wps = [carla_map.get_waypoint(tf.location) for tf in self._route]
        route = adapter.route_lane_ids(route_wps, key_to_id)
        cfg, vehicle = fp.profiles.load(self._profile, adapter.vehicle_params(self._ego))
        self._cfg, self._vehicle = cfg, vehicle
        self._lp = fp.LocalPlanner(cfg, vehicle)
        goal = adapter.to_planner(self._route[-1].location.x, self._route[-1].location.y)
        self._lp.set_map(self._road, route, goal[:2])
        self._ctrl = fp.make_controller("mpc", vehicle)
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
                    sid, state in (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow)
                )
            obs = adapter.obstacles(
                self._world,
                self._ego.id,
                self._cfg.planner.sampling.max_t + 0.5,
                0.1,
                max_range=self.obstacle_range,
                ego_xy=(ego_s.x, ego_s.y),
                road_map=self._road,
            )
            out = self._lp.step(t, ego_s, obs)
            if len(out.trajectory) >= 2:
                self._traj, self._traj_t0 = out.trajectory, t
        self._tick += 1
        if self._traj is None:
            return carla.VehicleControl(brake=1.0)
        cmd = self._ctrl.compute(ego_s, self._traj, t - self._traj_t0)
        v_ref = self._traj.sample(t - self._traj_t0 + 0.3).v
        return self._actuation.control(cmd, v_ref, ego_s.v, self._vehicle.max_steer)

    def destroy(self):
        self._lp = None
