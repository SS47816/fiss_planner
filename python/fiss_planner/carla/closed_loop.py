"""Closed-loop driving in CARLA 0.9.16 with the FISS planner and a tracking controller.

The simulator runs in synchronous mode. Every planning period the planner
gets the ego state, the CARLA map (as a RoadMap with route and lane-change
rules) and all other actors with lane-following (or constant-velocity)
predictions; the MPC (or Stanley) controller tracks the plan at every tick.
Collisions and lane invasions come from CARLA sensors.

Start the server first (headless):
    $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen -quality-level=Low &
Then:
    python -m fiss_planner.carla.closed_loop --scenario cut_in --planner fiss_plus --gif results/carla
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import fiss_planner as fp

from . import adapter

SCENARIOS = ["cruise", "lead_vehicle", "static_obstacle", "cut_in", "traffic"]


@dataclass
class Result:
    scenario: str
    planner: str
    controller: str
    smoother: bool
    duration_s: float = 0.0
    distance_m: float = 0.0
    route_completion: float = 0.0
    collisions: int = 0
    lane_invasions: int = 0
    plan_success_rate: float = 0.0
    emergency_rate: float = 0.0
    fallback_rate: float = 0.0  # cycles executing the rest of the previous plan
    degraded_rate: float = 0.0  # cycles planned with the shortened collision horizon
    stopped_without_lead_s: float = 0.0  # standing still with nothing ahead in the path
    min_ttc_s: float = math.inf  # to the vehicle ahead in the ego's path, while keeping the lane
    min_time_headway_s: float = math.inf
    time_below_1s_headway_s: float = 0.0
    min_clearance_m: float = math.inf  # smallest box-to-box distance to any other road user
    plan_ms_mean: float = 0.0
    plan_ms_p95: float = 0.0
    tracking_rms_m: float = 0.0  # distance to the time-indexed reference point
    lateral_rms_m: float = 0.0  # ... its component across the reference heading
    along_track_rms_m: float = 0.0  # ... and along it (schedule lag)
    mean_speed: float = 0.0
    rms_jerk: float = 0.0
    max_lat_accel: float = 0.0
    extra: dict = field(default_factory=dict)


class _Hooks:
    """Fan out runner callbacks to several hook objects."""

    def __init__(self, hooks):
        self.hooks = [h for h in (hooks if isinstance(hooks, (list, tuple)) else [hooks]) if h is not None]
        self.render = any(getattr(h, "render", False) for h in self.hooks)

    def on_start(self, *args):
        for h in self.hooks:
            h.on_start(*args)

    def on_tick(self, *args):
        for h in self.hooks:
            h.on_tick(*args)

    def on_end(self):
        for h in self.hooks:
            h.on_end()


def _smoothed_derivative(x: np.ndarray, dt: float, order: int, window: int = 21) -> np.ndarray:
    """Savitzky-Golay estimate of the `order`-th derivative (cubic fit over `window` samples)."""
    if len(x) < window:
        return np.zeros(1)
    half = window // 2
    tt = np.arange(-half, half + 1) * dt
    coeffs = np.linalg.pinv(np.vander(tt, 4, increasing=True))[order] * math.factorial(order)
    return np.convolve(x, coeffs[::-1], mode="valid")


def _carla_agents_path():
    root = os.environ.get("CARLA_ROOT")
    if root:
        p = Path(root) / "PythonAPI" / "carla"
        if p.exists() and str(p) not in sys.path:
            sys.path.append(str(p))


def _spawn(world, blueprint_name: str, transform, autopilot_tm=None, color=None):
    bp = world.get_blueprint_library().find(blueprint_name)
    if color and bp.has_attribute("color"):
        bp.set_attribute("color", color)
    actor = world.try_spawn_actor(bp, transform)
    if actor is not None and autopilot_tm is not None:
        actor.set_autopilot(True, autopilot_tm.get_port())
    return actor


def _setup(world, carla_map, scenario: str, tm, seed: int):
    """Spawn ego and scenario actors; return (ego, actors, route_waypoints)."""
    import carla

    _carla_agents_path()
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    rng = random.Random(seed)
    grp = GlobalRoutePlanner(carla_map, 2.0)
    spawns = carla_map.get_spawn_points()
    actors = []

    def lane_ahead(wp, dist):
        nxt = wp.next(dist)
        return nxt[0] if nxt else None

    if scenario == "traffic":
        start = spawns[rng.randrange(len(spawns))]
        ego = _spawn(world, "vehicle.lincoln.mkz_2020", start, color="0,90,200")
        far = max(spawns, key=lambda s: s.location.distance(start.location))
        route = [wp for wp, _ in grp.trace_route(start.location, far.location)]
        rng.shuffle(spawns)
        for sp in spawns[:60]:
            if sp.location.distance(start.location) < 15.0:
                continue
            bp = rng.choice(world.get_blueprint_library().filter("vehicle.*"))
            if int(bp.get_attribute("number_of_wheels")) != 4:
                continue
            a = world.try_spawn_actor(bp, sp)
            if a is not None:
                a.set_autopilot(True, tm.get_port())
                actors.append(a)
        return ego, actors, route

    # Highway scenarios: >= 3 same-direction lanes, straight for 150 m.
    def same_dir_lanes(wp):
        n, cur = 1, wp
        for get in (lambda w: w.get_left_lane(), lambda w: w.get_right_lane()):
            cur = get(wp)
            while (
                cur is not None and cur.lane_type == carla.LaneType.Driving and cur.lane_id * wp.lane_id > 0
            ):
                n += 1
                cur = get(cur)
        return n

    start_wp = None
    candidates = [carla_map.get_waypoint(sp.location) for sp in spawns]
    rng.shuffle(candidates)
    for wp in candidates:
        left = wp.get_left_lane()
        if (
            wp.is_junction
            or left is None
            or left.lane_type != carla.LaneType.Driving
            or left.lane_id * wp.lane_id <= 0
        ):
            continue
        if same_dir_lanes(wp) < 3:
            continue
        ahead = [lane_ahead(wp, d) for d in (50.0, 100.0, 150.0)]
        if any(a is None or a.is_junction for a in ahead):
            continue
        yaws = [wp.transform.rotation.yaw] + [a.transform.rotation.yaw for a in ahead]
        if max(abs((y - yaws[0] + 180) % 360 - 180) for y in yaws) < 3.0:
            start_wp = wp
            break
    if start_wp is None:
        raise RuntimeError("no suitable multi-lane straight in this town")
    start_tf = start_wp.transform
    start_tf.location.z += 0.5
    ego = _spawn(world, "vehicle.lincoln.mkz_2020", start_tf, color="0,90,200")
    end_wp = lane_ahead(start_wp, 400.0) or lane_ahead(start_wp, 200.0)
    route = [wp for wp, _ in grp.trace_route(start_wp.transform.location, end_wp.transform.location)]

    def place(wp, z=0.5):
        tf = wp.transform
        tf.location.z += z
        return tf

    if scenario == "cruise":
        pass  # empty road: isolates tracking quality
    elif scenario == "lead_vehicle":
        lead = _spawn(world, "vehicle.tesla.model3", place(lane_ahead(start_wp, 35.0)), autopilot_tm=tm)
        if lead:
            tm.set_desired_speed(lead, 25.0)  # km/h
            tm.auto_lane_change(lead, False)
            actors.append(lead)
    elif scenario == "static_obstacle":
        parked = _spawn(world, "vehicle.audi.tt", place(lane_ahead(start_wp, 60.0)))
        if parked:
            parked.apply_control(carla.VehicleControl(hand_brake=True))
            actors.append(parked)
    elif scenario == "cut_in":
        # Scripted (kinematic) cutter: deterministic, unlike the cautious Traffic Manager.
        left = start_wp.get_left_lane()
        cutter = _spawn(world, "vehicle.nissan.patrol", place(lane_ahead(left, 40.0)))
        if cutter:
            cutter.set_simulate_physics(False)
            actors.append(cutter)
    else:
        raise ValueError(f"unknown scenario {scenario}")
    return ego, actors, route


class Actuation:
    """Map (steer [rad], accel [m/s^2]) to CARLA controls: feedforward on the
    commanded acceleration plus a PI loop on the reference speed."""

    def __init__(self, dt: float, k_ff: float = 0.08, kp: float = 0.25, ki: float = 0.05):
        self.dt, self.k_ff, self.kp, self.ki = dt, k_ff, kp, ki
        self.integral = 0.0

    def apply(self, ego, cmd, v_ref: float, v: float, max_steer: float, v_ahead: float | None = None) -> None:
        ego.apply_control(self.control(cmd, v_ref, v, max_steer, v_ahead))

    def control(self, cmd, v_ref: float, v: float, max_steer: float, v_ahead: float | None = None):
        """`v_ahead`: highest planned speed over the next second (default v_ref);
        the brakes hold the vehicle only while the plan stays (almost) stopped."""
        import carla

        err = v_ref - v
        self.integral = float(np.clip(self.integral + err * self.dt, -10.0, 10.0))
        u = self.k_ff * cmd.accel + self.kp * err + self.ki * self.integral
        ctrl = carla.VehicleControl()
        ctrl.steer = float(np.clip(-cmd.steer / max_steer, -1.0, 1.0))  # CARLA steer is clockwise-positive
        if (v_ref if v_ahead is None else v_ahead) < 0.1 and v < 0.5:
            ctrl.brake, self.integral = 0.5, 0.0
        elif u >= 0.0:
            ctrl.throttle = float(np.clip(u, 0.0, 0.8))
        else:
            ctrl.brake = float(np.clip(-u, 0.0, 1.0))
        return ctrl


class FreeSpaceTracker:
    """Track a free-space trajectory (LocalPlanOutput.free_space; negative
    speeds drive backwards) at low speed: pure pursuit of the rear axle along
    the path in either gear, and a PI loop on the speed. The gear changes only
    at standstill."""

    def __init__(self, dt: float, vehicle, lookahead: float = 2.0, kp: float = 0.3, ki: float = 0.1):
        self.dt, self.vehicle, self.lookahead, self.kp, self.ki = dt, vehicle, lookahead, kp, ki
        self.integral = 0.0
        self.reverse = False

    def _rear(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        r = self.vehicle.rear_axle_to_center
        return x - r * math.cos(yaw), y - r * math.sin(yaw)

    def control(self, ego, v_long: float, traj, t_rel: float):
        """`ego`: EgoState (center), `v_long`: signed speed along the heading."""
        import carla

        ctrl = carla.VehicleControl()
        ahead = [traj.sample(t_rel + h).v for h in (0.1, 0.3, 0.6, 1.0)]
        moving = [v for v in ahead if abs(v) > 0.05]
        if not moving:
            ctrl.brake, ctrl.reverse, self.integral = 1.0, self.reverse, 0.0
            return ctrl
        reverse = moving[0] < 0.0
        if reverse != self.reverse and abs(v_long) > 0.2:
            ctrl.brake, ctrl.reverse, self.integral = 1.0, self.reverse, 0.0  # stop before changing gear
            return ctrl
        self.reverse = reverse

        # Look-ahead point on the rear axle's path, in the current direction;
        # past the last such point, along its heading.
        rx, ry = self._rear(ego.x, ego.y, ego.yaw)
        sign = -1.0 if reverse else 1.0
        tx, ty, t, last_yaw = rx, ry, t_rel, ego.yaw
        end = t_rel + 20.0 if len(traj) == 0 else float(traj.t[-1])
        while t < end:
            t += 0.1
            s = traj.sample(t)
            if s.v * sign < -0.05:
                break
            tx, ty = self._rear(s.x, s.y, s.yaw)
            last_yaw = s.yaw
            if math.hypot(tx - rx, ty - ry) >= self.lookahead:
                break
        d = math.hypot(tx - rx, ty - ry)
        if d < self.lookahead:
            tx += sign * (self.lookahead - d) * math.cos(last_yaw)
            ty += sign * (self.lookahead - d) * math.sin(last_yaw)
        heading = ego.yaw + (math.pi if reverse else 0.0)
        alpha = math.atan2(ty - ry, tx - rx) - heading
        ld = max(math.hypot(tx - rx, ty - ry), 1e-3)
        delta = math.atan2(2.0 * self.vehicle.wheelbase * math.sin(alpha), ld)
        if reverse:
            delta = -delta
        ctrl.steer = float(np.clip(-delta / self.vehicle.max_steer, -1.0, 1.0))  # CARLA steer is clockwise
        ctrl.reverse = reverse

        v_ref = abs(traj.sample(t_rel + 0.3).v)
        err = v_ref - abs(v_long)
        self.integral = float(np.clip(self.integral + err * self.dt, -3.0, 3.0))
        u = self.kp * err + self.ki * self.integral + (0.2 if v_ref > 0.1 else 0.0)
        if v_ref < 0.05 and abs(v_long) < 0.3:
            ctrl.brake = 1.0
        elif u >= 0.0:
            ctrl.throttle = float(np.clip(u, 0.0, 0.6))
        else:
            ctrl.brake = float(np.clip(-u, 0.0, 1.0))
        return ctrl


def _drive_cutter(cutter, state, ego_s, t, dt, result, speed=9.0, duration=2.5):
    """Advance the scripted cut-in vehicle by one tick. It follows its lane at
    `speed` and, once 8-20 m ahead of the ego, blends into the ego lane."""
    import carla

    carla_map = cutter.get_world().get_map()
    if state is None:
        state = {"wp": carla_map.get_waypoint(cutter.get_transform().location), "t_cut": None}
    nxt = state["wp"].next(speed * dt)
    if nxt:
        state["wp"] = nxt[0]
    wp = state["wp"]
    loc = wp.transform.location
    cx, cy, _ = adapter.to_planner(loc.x, loc.y)
    ahead = (cx - ego_s.x) * math.cos(ego_s.yaw) + (cy - ego_s.y) * math.sin(ego_s.yaw)
    if state["t_cut"] is None and 8.0 <= ahead <= 20.0:
        state["t_cut"] = t
        result.extra["cut_in_at_s"] = round(t, 2)
    alpha = 0.0
    if state["t_cut"] is not None:
        u = min(1.0, (t - state["t_cut"]) / duration)
        alpha = 0.5 - 0.5 * math.cos(math.pi * u)  # smooth lateral blend
    target = wp.get_right_lane() if alpha > 0.0 else None
    tf = wp.transform
    if target is not None:
        tl = target.transform.location
        tf.location = carla.Location(loc.x + alpha * (tl.x - loc.x), loc.y + alpha * (tl.y - loc.y), loc.z)
        tf.rotation.yaw += 12.0 * math.sin(math.pi * min(1.0, (t - state["t_cut"]) / duration))
    tf.location.z += 0.1
    cutter.set_transform(tf)
    fwd = tf.get_forward_vector()
    # Physics is off, so CARLA reports no velocity; keep the true one for the planner.
    state["velocity"] = carla.Vector3D(fwd.x * speed, fwd.y * speed, 0.0)
    return state


def run(
    scenario: str,
    planner: str = "fiss_plus",
    controller: str = "mpc",
    smoother: bool | None = None,
    town: str = "Town04",
    duration: float = 40.0,
    seed: int = 0,
    gif_dir: str | None = None,
    host: str = "localhost",
    port: int = 2000,
    desired_speed: float | None = None,
    hooks=None,
    prediction: str = "lane_following",
    configure=None,
    profile: str | None = "carla_scenarios",
) -> Result:
    """Run one scenario. `hooks` (optional, one or a list) get ``on_start(world, ego, vehicle, road)``
    once and ``on_tick(t, ego_state, plan_output, world, obstacles)`` after every simulator tick
    (`obstacles`: those of the last planning cycle); a hook with
    ``render = True`` keeps CARLA's rendering on (for cameras). The planner settings come from
    `profile` (a name in fiss_planner.profiles or a YAML file); `planner`, and `smoother` and
    `desired_speed` unless None, override it, and `configure` (optional) may edit the
    LocalPlannerConfig before the run."""
    import carla
    from shapely.geometry import Polygon

    hooks = _Hooks(hooks) if hooks is not None else None
    client = carla.Client(host, port)
    client.set_timeout(60.0)
    world = client.get_world()
    if not world.get_map().name.endswith(town):
        world = client.load_world(town)
    settings = world.get_settings()
    dt = 0.05
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    settings.no_rendering_mode = not getattr(hooks, "render", False)
    world.apply_settings(settings)
    tm = client.get_trafficmanager(8000 + seed % 100)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(seed)
    carla_map = world.get_map()

    actors, sensors = [], []
    result = Result(scenario, planner, controller, bool(smoother))
    try:
        ego, actors, route_wps = _setup(world, carla_map, scenario, tm, seed)
        if ego is None:
            raise RuntimeError("failed to spawn the ego vehicle")
        result.extra["actors"] = len(actors)
        world.tick()
        road, key_to_id = adapter.road_map_from_carla(carla_map)
        route = adapter.route_lane_ids(route_wps, key_to_id)
        route_xy = np.asarray(
            [adapter.to_planner(w.transform.location.x, w.transform.location.y)[:2] for w in route_wps]
        )
        cfg, vehicle = fp.profiles.load(profile, adapter.vehicle_params(ego))

        events = {"collisions": 0, "invasions": 0}
        bp = world.get_blueprint_library()
        col = world.spawn_actor(bp.find("sensor.other.collision"), carla.Transform(), attach_to=ego)
        col.listen(lambda e: events.__setitem__("collisions", events["collisions"] + 1))
        inv = world.spawn_actor(bp.find("sensor.other.lane_invasion"), carla.Transform(), attach_to=ego)
        inv.listen(lambda e: events.__setitem__("invasions", events["invasions"] + 1))
        sensors = [col, inv]

        cfg.planner.type = fp.planner_type_from_string(planner)
        cfg.planner.keep_candidates = gif_dir is not None or hooks is not None
        if smoother is not None:
            cfg.smoother.enabled = smoother
        if desired_speed is not None:
            cfg.desired_speed = desired_speed
        if configure is not None:
            configure(cfg)
        result.smoother = cfg.smoother.enabled
        result.extra["profile"] = fp.profiles.name(profile)
        result.extra["config_hash"] = fp.profiles.config_hash(cfg, vehicle)
        lp = fp.LocalPlanner(cfg, vehicle)
        lp.set_map(road, route, tuple(route_xy[-1]))  # the route's lane changes are due by its end
        ctrl = fp.make_controller(controller, vehicle)
        actuation = Actuation(dt)
        if hooks is not None:
            hooks.on_start(world, ego, vehicle, road)

        steps = int(duration / dt)
        plan_every = 2  # 10 Hz planning, 20 Hz control
        traj, traj_t0 = None, 0.0
        out = None
        obs = []
        cutter_state = None
        stats, track_err, lat_err, lon_err, states, frames = [], [], [], [], [], []
        leads, fallbacks, stops = [], [], []
        for k in range(steps):
            t = k * dt
            ego_s = adapter.ego_state(ego)
            states.append(ego_s)
            if scenario == "cut_in" and actors:
                cutter_state = _drive_cutter(actors[0], cutter_state, ego_s, t, dt, result)
            if k % plan_every == 0:
                obs = adapter.obstacles(
                    world,
                    ego.id,
                    cfg.planner.sampling.max_t + 0.5,
                    0.1,
                    ego_xy=(ego_s.x, ego_s.y),
                    road_map=road,
                    prediction=prediction,
                    velocities={actors[0].id: cutter_state["velocity"]} if cutter_state else None,
                )
                out = lp.step(t, ego_s, obs)
                ego_box = Polygon(
                    fp.Box2d(ego_s.x, ego_s.y, ego_s.yaw, vehicle.length, vehicle.width).corners()
                )
                gaps = [
                    ego_box.distance(Polygon(b.corners()))
                    for b in (o.box_at(0.0) for o in obs)
                    if b is not None
                ]
                result.min_clearance_m = min([result.min_clearance_m, *gaps])
                stats.append((out.success, out.emergency, out.stats.time_ms, out.fallback, out.degraded))
                lead = out.stats.lead
                keeping_lane = (
                    abs(out.start.d_d) < 0.3
                )  # gaps while pulling out to overtake are not following
                leads.append((lead.id, lead.gap, lead.time_headway, lead.ttc, ego_s.v, keeping_lane))
                if ego_s.v < 0.3 and lead.gap > 30.0 and any(x[4] > 2.0 for x in leads) and len(stops) < 50:
                    stops.append(
                        {
                            "t": round(t, 2),
                            "blocking_id": out.stats.blocking_id,
                            "rejections": {k: v for k, v in out.stats.rejections.items() if v},
                        }
                    )
                if (out.emergency or out.fallback or out.degraded) and len(fallbacks) < 200:
                    fallbacks.append(_blocking_event(world, carla_map, ego, t, out))
                if len(out.trajectory) >= 2:
                    traj, traj_t0 = out.trajectory, t
                if gif_dir is not None and k % 4 == 0:
                    frames.append(
                        (
                            ego_s,
                            [(o.box_at(0.0)) for o in obs],
                            np.column_stack([out.trajectory.x, out.trajectory.y])
                            if len(out.trajectory)
                            else None,
                            [
                                np.column_stack([c.x, c.y])
                                for c in sorted(out.candidates, key=lambda c: c.total_cost())[:150]
                            ],
                        )
                    )
            if traj is not None:
                cmd = ctrl.compute(ego_s, traj, t - traj_t0)
                ref = traj.sample(t - traj_t0)
                track_err.append(math.hypot(ref.x - ego_s.x, ref.y - ego_s.y))
                ex, ey = ego_s.x - ref.x, ego_s.y - ref.y
                lat_err.append(-ex * math.sin(ref.yaw) + ey * math.cos(ref.yaw))
                lon_err.append(ex * math.cos(ref.yaw) + ey * math.sin(ref.yaw))
                v_ahead = max(traj.sample(t - traj_t0 + h).v for h in (0.3, 0.6, 1.0))
                actuation.apply(
                    ego, cmd, traj.sample(t - traj_t0 + 0.3).v, ego_s.v, vehicle.max_steer, v_ahead
                )
            world.tick()
            if hooks is not None:
                hooks.on_tick(t, ego_s, out, world, obs)
            if np.linalg.norm(route_xy[-1] - [ego_s.x, ego_s.y]) < 5.0:
                break

        xy = np.asarray([[s.x, s.y] for s in states])
        dists = np.linalg.norm(route_xy[:, None, :] - xy[None, :, :], axis=2)
        progress_idx = int(np.max(np.argmin(dists, axis=0))) if len(xy) else 0
        v = np.asarray([s.v for s in states])
        # CARLA's reported acceleration is noisy physics output: differentiate a
        # lightly smoothed speed signal instead.
        kap = np.asarray([s.kappa for s in states])
        jerk = _smoothed_derivative(v, dt, order=2)
        pt = [s[2] for s in stats]
        result.duration_s = len(states) * dt
        result.distance_m = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0
        result.route_completion = progress_idx / max(1, len(route_xy) - 1)
        result.collisions = events["collisions"]
        result.lane_invasions = events["invasions"]
        result.plan_success_rate = float(np.mean([s[0] for s in stats])) if stats else 0.0
        result.emergency_rate = float(np.mean([s[1] for s in stats])) if stats else 0.0
        result.fallback_rate = float(np.mean([s[3] for s in stats])) if stats else 0.0
        result.degraded_rate = float(np.mean([s[4] for s in stats])) if stats else 0.0
        period = plan_every * dt
        moving = [x for x in leads if x[4] > 1.0 and x[5]]
        result.min_ttc_s = min((x[3] for x in moving), default=math.inf)
        result.min_time_headway_s = min((x[2] for x in moving), default=math.inf)
        result.time_below_1s_headway_s = period * sum(1 for x in moving if x[2] < 1.0)
        started = next((i for i, x in enumerate(leads) if x[4] > 2.0), len(leads))  # skip the standing start
        result.stopped_without_lead_s = period * sum(1 for x in leads[started:] if x[4] < 0.3 and x[1] > 30.0)
        result.extra["fallbacks"] = fallbacks
        result.extra["stops"] = stops
        result.plan_ms_mean = float(np.mean(pt)) if pt else 0.0
        result.plan_ms_p95 = float(np.percentile(pt, 95)) if pt else 0.0
        result.tracking_rms_m = float(np.sqrt(np.mean(np.square(track_err)))) if track_err else 0.0
        result.lateral_rms_m = float(np.sqrt(np.mean(np.square(lat_err)))) if lat_err else 0.0
        result.along_track_rms_m = float(np.sqrt(np.mean(np.square(lon_err)))) if lon_err else 0.0
        result.mean_speed = float(np.mean(v)) if len(v) else 0.0
        result.rms_jerk = float(np.sqrt(np.mean(jerk**2)))
        result.max_lat_accel = float(np.max(np.abs(v**2 * kap))) if len(v) else 0.0
        if gif_dir is not None and frames:
            from .render import render_gif

            path = Path(gif_dir) / f"{scenario}_{planner}{'_qp' if result.smoother else ''}_{controller}.gif"
            render_gif(frames, road, vehicle, path)
            result.extra["gif"] = str(path)
    finally:
        if hooks is not None:
            hooks.on_end()
        # Release Traffic Manager control before destroying anything: the TM
        # would otherwise operate on destroyed actors and abort the client.
        for s in sensors:
            s.stop()
        for a in actors:
            if a.is_alive and a.type_id.startswith("vehicle."):
                a.set_autopilot(False, tm.get_port())
        world.tick()
        tm.set_synchronous_mode(False)
        doomed = list(sensors) + [a for a in actors if a.is_alive]
        if "ego" in locals() and ego is not None and ego.is_alive:
            doomed.append(ego)
        client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in doomed], True)
        settings.synchronous_mode = False
        world.apply_settings(settings)
    return result


def _finite(o):
    """JSON-safe copy: non-finite floats (e.g. no vehicle ahead) become None."""
    if isinstance(o, float) and not math.isfinite(o):
        return None
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_finite(v) for v in o]
    return o


def _blocking_event(world, carla_map, ego, t: float, out) -> dict:
    """Why a cycle fell back: the obstacle that blocked the cheapest candidate, relative to the ego."""
    kind = "emergency" if out.emergency else "fallback" if out.fallback else "degraded"
    event = {
        "t": round(t, 2),
        "kind": kind,
        "blocking_id": out.stats.blocking_id,
        "blocking_t": round(out.stats.blocking_t, 2),
        "rejections": dict(out.stats.rejections),
    }
    actor = world.get_actor(out.stats.blocking_id) if out.stats.blocking_id >= 0 else None
    if actor is not None:
        et, at = ego.get_transform(), actor.get_transform()
        dx, dy = at.location.x - et.location.x, at.location.y - et.location.y
        yaw = math.radians(et.rotation.yaw)
        ewp, awp = carla_map.get_waypoint(et.location), carla_map.get_waypoint(at.location)
        vel = actor.get_velocity()
        event.update(
            type=actor.type_id,
            ahead_m=round(dx * math.cos(yaw) + dy * math.sin(yaw), 1),
            left_m=round(dx * math.sin(yaw) - dy * math.cos(yaw), 1),  # CARLA is left-handed
            rel_heading_deg=round((at.rotation.yaw - et.rotation.yaw + 180) % 360 - 180, 1),
            speed=round(math.hypot(vel.x, vel.y), 1),
            same_road=ewp.road_id == awp.road_id,
            same_lane=ewp.road_id == awp.road_id and ewp.lane_id == awp.lane_id,
            junction=awp.is_junction,
        )
    return event


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", nargs="+", default=SCENARIOS, choices=SCENARIOS)
    ap.add_argument("--planner", nargs="+", default=["fiss_plus"])
    ap.add_argument("--controller", nargs="+", default=["mpc"], choices=["mpc", "stanley_pid"])
    ap.add_argument(
        "--profile",
        default="carla_scenarios",
        help="planner profile: a shipped name (fiss_planner.profiles) or a YAML file",
    )
    ap.add_argument(
        "--smoother", action=argparse.BooleanOptionalAction, help="QP smoother (default: the profile's)"
    )
    ap.add_argument("--town", default="Town04")
    ap.add_argument("--traffic-town", default="Town04")
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--seed", type=int, nargs="+", default=[0])
    ap.add_argument("--gif", default=None, help="directory for top-down GIFs")
    ap.add_argument("--mcap", default=None, help="directory for MCAP recordings (Foxglove), one per run")
    ap.add_argument(
        "--foxglove-live", action="store_true", help="stream runs to Foxglove (ws://localhost:8765)"
    )
    ap.add_argument("--prediction", default="lane_following", choices=["lane_following", "constant_velocity"])
    ap.add_argument("--out", default="results/carla.json")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--append", action="store_true", help="append to an existing --out file")
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    results = json.loads(out.read_text()) if args.append and out.exists() else []
    runs = [
        (sc, pl, ctrl, seed)
        for sc in args.scenario
        for pl in args.planner
        for ctrl in args.controller
        for seed in (args.seed if sc == "traffic" else args.seed[:1])
    ]
    for scenario, planner, controller, seed in runs:
        town = args.traffic_town if scenario == "traffic" else args.town
        t0 = time.time()
        hooks = None
        if args.mcap or args.foxglove_live:
            from .record import FoxgloveHook

            mcap = None
            if args.mcap:
                Path(args.mcap).mkdir(parents=True, exist_ok=True)
                mcap = str(Path(args.mcap) / f"{scenario}_{planner}_{controller}_seed{seed}.mcap")
            hooks = FoxgloveHook(planner, mcap, args.foxglove_live)
        r = run(
            scenario,
            planner,
            controller,
            args.smoother,
            town,
            args.duration,
            seed,
            args.gif,
            port=args.port,
            hooks=hooks,
            prediction=args.prediction,
            profile=args.profile,
        )
        r.extra["seed"] = seed
        d = r.__dict__
        row = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items() if k != "extra"}
        print(json.dumps(row), flush=True)
        d["wall_s"] = time.time() - t0
        results.append(d)
        out.write_text(json.dumps(_finite(results), indent=2))  # keep partial results if the simulator dies


if __name__ == "__main__":
    main()
