"""Closed-loop CommonRoad benchmark of the FOP/FOP+/FISS/FISS+ planners.

Each scenario is simulated step by step: at every time step the planner gets
the ego state and the obstacles (with ground-truth or constant-velocity
predictions), and the ego advances along the planned trajectory by one step
(perfect tracking, as in the FISS/FISS+ papers). Unlike the original harness,
the executed trajectory is checked with the CommonRoad drivability checker
(obstacles and road boundary) and success requires the official goal check.

Example::

    python -m frenet_planner.commonroad.benchmark --scenarios data/cache/scenarios/NGSIM \\
        --planners fop fop_plus fiss fiss_plus --jobs 20 --out results/ngsim.csv
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import multiprocessing as mp
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

PLANNERS = ["fop", "fop_plus", "fiss", "fiss_plus"]


@dataclasses.dataclass
class Options:
    planner: str = "fiss_plus"
    samples: tuple[int, int, int] = (5, 5, 5)
    adjacent_lanes: int = 1
    prediction: str = "ground_truth"
    config: str | None = None
    min_t: float | None = None
    max_t: float | None = None
    max_steps: int = 600
    vehicle: str = "VW_VANAGON"
    keep_candidates: bool = False
    num_threads: int = 1
    smoother: bool = False
    safety_envelope: bool | None = None  # None: keep the (config's) default
    proximity_cost: bool | None = None


def _make_local_planner(opts: Options, vehicle):
    import frenet_planner as fp

    cfg = fp.LocalPlannerConfig()
    if opts.config:
        planner_cfg, _ = fp.load_planner_config(opts.config)
        cfg.planner = planner_cfg
    cfg.planner.type = fp.planner_type_from_string(opts.planner)
    w, v, t = opts.samples
    cfg.planner.sampling.num_width = w
    cfg.planner.sampling.num_speed = v
    cfg.planner.sampling.num_t = t
    if opts.min_t is not None:
        cfg.planner.sampling.min_t = opts.min_t
    if opts.max_t is not None:
        cfg.planner.sampling.max_t = opts.max_t
    cfg.planner.keep_candidates = opts.keep_candidates
    cfg.planner.num_threads = opts.num_threads
    cfg.route.max_adjacent_lanes = opts.adjacent_lanes
    cfg.smoother.enabled = opts.smoother
    if opts.safety_envelope is not None:
        cfg.planner.safety.enabled = opts.safety_envelope
    if opts.proximity_cost is False:
        cost = cfg.planner.cost
        cost.w_headway = cost.w_ttc = cost.w_clearance = cost.follow_time = 0.0
    return fp.LocalPlanner(cfg, vehicle)


def _goal_speed(problem) -> float | None:
    st = problem.goal.state_list[0]
    if st.has_value("velocity"):
        vel = st.velocity
        hi = getattr(vel, "end", vel)
        return float(hi)
    return None


def _goal_time_end(problem) -> int | None:
    st = problem.goal.state_list[0]
    if st.has_value("time_step"):
        ts = st.time_step
        return int(getattr(ts, "end", ts))
    return None


class GoalSpeedPolicy:
    """Target speed that reaches the goal region within its time window.

    The goal center is projected onto the current reference line; the target
    is the average speed needed to cover the remaining distance by the middle
    of the goal time window, clipped to the goal velocity interval (or to
    [0, v_max] without one). Without a position or time goal the policy keeps
    the initial speed (or `default`)."""

    def __init__(self, problem, initial_speed: float, v_max: float, default: float = 13.5):
        st = problem.goal.state_list[0]
        self.center = None
        self.geometry = None
        if st.has_value("position"):
            shape = st.position
            if hasattr(shape, "shapes"):  # ShapeGroup
                from shapely.ops import unary_union

                geom = unary_union([sh.shapely_object for sh in shape.shapes])
            else:
                geom = shape.shapely_object
            c = geom.centroid
            self.center = (c.x, c.y)
            self.geometry = geom
        self.t_mid = None
        if st.has_value("time_step"):
            ts = st.time_step
            self.t_mid = 0.5 * (float(getattr(ts, "start", ts)) + float(getattr(ts, "end", ts)))
        if st.has_value("velocity"):
            self.v_lo = float(getattr(st.velocity, "start", st.velocity))
            self.v_hi = float(getattr(st.velocity, "end", st.velocity))
        else:
            self.v_lo, self.v_hi = 0.0, v_max
        self.cruise = min(max(initial_speed, default if initial_speed < 0.5 else initial_speed), v_max)
        self.cruise = float(np.clip(self.cruise, self.v_lo, self.v_hi))

    def __call__(self, time_step: int, dt: float, reference, ego_s: float) -> float:
        if self.center is None or self.t_mid is None or reference is None or reference.empty():
            return self.cruise
        goal_s = reference.project(self.center[0], self.center[1]).s  # global: the goal may be far ahead
        remaining_t = (self.t_mid - time_step) * dt
        if remaining_t <= 0.5:
            return self.v_hi
        return float(np.clip((goal_s - ego_s) / remaining_t, self.v_lo, self.v_hi))


def min_post_encroachment_time(track: list, dt: float, window: float = 3.0) -> float:
    """Smallest time gap [s] between the ego and a crossing or oncoming vehicle
    (heading more than 45 deg apart) occupying the same space; 0 = collision.
    `track` holds (ego box, {obstacle id: box}) per time step."""
    import frenet_planner as fp

    w = int(round(window / dt))
    best = math.inf
    for i, (ego, _) in enumerate(track):
        r_ego = 0.5 * math.hypot(ego.length, ego.width)
        for j in range(max(0, i - w), min(len(track), i + w + 1)):
            gap = abs(i - j) * dt
            if gap >= best:
                continue
            for box in track[j][1].values():
                if abs(math.remainder(box.yaw - ego.yaw, math.tau)) <= math.pi / 4:
                    continue
                reach = r_ego + 0.5 * math.hypot(box.length, box.width)
                if math.hypot(box.cx - ego.cx, box.cy - ego.cy) < reach and fp.overlaps(ego, box):
                    best = gap
                    break
    return best


def run_scenario(path: str, opts: Options, record: bool = False, on_step=None) -> tuple[dict, dict | None]:
    """Simulate one scenario. Returns (metrics row, optional per-step record).
    `on_step(t, ego, obstacles, out, road)` (optional) is called after every planning step."""
    import commonroad_dc.pycrcc as pycrcc
    from commonroad.common.file_reader import CommonRoadFileReader
    from commonroad.scenario.state import CustomState
    from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
    from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker
    from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
    from commonroad_route_planner.route_planner import RoutePlanner
    from shapely.geometry import Polygon

    import frenet_planner as fp

    from . import adapter

    label = opts.planner + ("+qp" if opts.smoother else "")
    row: dict = {"scenario": Path(path).stem, "planner": label, "status": "", "error": ""}
    scenario, problems = CommonRoadFileReader(path).open()
    problem = list(problems.planning_problem_dict.values())[0]
    dt = float(scenario.dt)
    params = VehicleParameterMapping[opts.vehicle].value
    vehicle = adapter.vehicle_params_from_commonroad(params)

    country = getattr(scenario.scenario_id, "country_id", None)
    try:
        from commonroad.scenario.traffic_sign import SupportedTrafficSignCountry

        country = SupportedTrafficSignCountry(country) if country else None
    except Exception:  # noqa: BLE001
        country = None
    road = adapter.road_map_from_lanelet_network(scenario.lanelet_network, country)
    try:
        routes = RoutePlanner(scenario.lanelet_network, problem, scenario).plan_routes()
        route = list(routes[0].lanelet_ids) if routes else []
    except Exception:  # noqa: BLE001 - the route planner fails on some maps; plan without a route
        route = []
    row["route_lanelets"] = len(route)

    lp = _make_local_planner(opts, vehicle)
    lp.set_map(road, route)
    horizon = lp.config.planner.sampling.max_t + 0.5

    goal_speed = GoalSpeedPolicy(problem, float(problem.initial_state.velocity), vehicle.max_speed)
    reference, ego_s = None, 0.0
    t_end = _goal_time_end(problem)
    final_obstacle_step = max(
        [ob.prediction.final_time_step for ob in scenario.dynamic_obstacles if ob.prediction is not None]
        or [0]
    )
    max_steps = min(opts.max_steps, (t_end + 1) if t_end is not None else final_obstacle_step + 50)

    cc = create_collision_checker(scenario)
    _, road_boundary = create_road_boundary_obstacle(scenario, method="obb_rectangles")
    goal_lanelets = (
        {lid for ids in problem.goal.lanelets_of_goal_position.values() for lid in ids}
        if problem.goal.lanelets_of_goal_position
        else set()
    )

    ego = adapter.ego_state_from_commonroad(problem.initial_state)
    if abs(float(getattr(problem.initial_state, "yaw_rate", 0.0) or 0.0)) < 1e-9:
        # Initial states usually carry yaw_rate = 0 even on curves; assume the
        # ego follows its lane's curvature instead of driving straight off it.
        rc = fp.build_route_corridor(road, route, ego)
        if rc is not None:
            ref = rc.reference.at(rc.ego_s)
            d = rc.reference.project(ego.x, ego.y, rc.ego_s).d
            ego.kappa = ref.kappa / max(1e-3, 1.0 - ref.kappa * d)
    k0 = int(problem.initial_state.time_step)
    obstacles_all = scenario.obstacles
    states = [ego]
    plan_ms, cycle_ms, generated, validated, checks = [], [], [], [], []
    fallbacks = 0
    emergencies = 0
    degraded = 0
    smoothed = 0
    leads = []  # (time headway, ttc, ego speed, keeping the lane) per step
    min_clearance = math.inf  # smallest box-to-box distance to any obstacle
    track = []  # (ego box, {moving obstacle id: box}) per step
    goal_lanelet_reached = False
    goal_area_reached = False
    record_data = {"steps": []} if record else None
    status = "timeout"

    for k in range(k0, k0 + max_steps):
        if scenario.lanelet_network.traffic_lights:
            for sl_id, active in adapter.traffic_light_stop_lines(scenario.lanelet_network, k).items():
                lp.set_stop_line_active(sl_id, active)
        obstacles = adapter.obstacles_at(obstacles_all, k, dt, horizon, opts.prediction, road)
        if reference is None:
            rc0 = fp.build_route_corridor(road, route, ego)
            if rc0 is not None:
                reference, ego_s = rc0.reference, rc0.ego_s
        elif not reference.empty():
            ego_s = reference.project(ego.x, ego.y, ego_s).s
        desired = goal_speed(k, dt, reference, ego_s)
        t0 = time.perf_counter()
        out = lp.step(k * dt, ego, obstacles, desired)
        if out.success:
            reference = out.reference
            ego_s = out.start.s
        cycle_ms.append(1000.0 * (time.perf_counter() - t0))
        plan_ms.append(out.stats.time_ms)
        generated.append(out.stats.num_generated)
        validated.append(out.stats.num_validated)
        checks.append(out.stats.num_collision_checks)
        fallbacks += out.fallback
        emergencies += out.emergency
        degraded += out.degraded
        smoothed += out.smoothed
        leads.append((out.stats.lead.time_headway, out.stats.lead.ttc, ego.v, abs(out.start.d_d) < 0.3))
        if on_step is not None:
            on_step((k - k0) * dt, ego, obstacles, out, road)
        if len(out.trajectory) < 2:
            status = "no_plan"
            break
        now = ego  # the state at time step k, matching `obstacles` and the plan
        ego_box = fp.Box2d(now.x, now.y, now.yaw, vehicle.length, vehicle.width)
        current = [(o, b) for o, b in ((o, o.box_at(0.0)) for o in obstacles) if b is not None]
        boxes = [b for _, b in current]
        if boxes:
            ego_poly = Polygon(ego_box.corners())
            min_clearance = min(min_clearance, min(ego_poly.distance(Polygon(b.corners())) for b in boxes))
        track.append((ego_box, {o.id: b for o, b in current if not o.is_static}))
        ego = out.trajectory.sample(dt)
        states.append(ego)
        if record_data is not None:
            record_data["steps"].append(
                {
                    "time_step": k,
                    "ego": (now.x, now.y, now.yaw, now.v),
                    "plan_ms": out.stats.time_ms,
                    "generated": out.stats.num_generated,
                    "lead": (
                        out.stats.lead.id,
                        out.stats.lead.gap,
                        out.stats.lead.time_headway,
                        out.stats.lead.ttc,
                    ),
                    "blocking": out.stats.blocking_id,
                    "mode": "emergency" if out.emergency else "fallback" if out.fallback else "plan",
                    "obstacles": [b.corners().tolist() for b in boxes],
                    "traj": np.column_stack([out.trajectory.x, out.trajectory.y]).tolist(),
                    "candidates": [np.column_stack([c.x, c.y]).tolist() for c in out.candidates[:400]]
                    if opts.keep_candidates
                    else [],
                    "cand_costs": [c.total_cost() for c in out.candidates[:400]]
                    if opts.keep_candidates
                    else [],
                }
            )

        # Evaluate the new ego pose at time step k + 1.
        rect = pycrcc.RectOBB(vehicle.length / 2, vehicle.width / 2, ego.yaw, ego.x, ego.y)
        if cc.time_slice(k + 1).collide(rect):
            status = "collision"
            break
        if road_boundary.collide(rect):
            status = "off_road"
            break
        if goal_lanelets and not goal_lanelet_reached:
            goal_lanelet_reached = bool(
                set(scenario.lanelet_network.find_lanelet_by_position([np.array([ego.x, ego.y])])[0])
                & goal_lanelets
            )
        if goal_speed.geometry is not None and not goal_area_reached:
            from shapely.geometry import Point

            goal_area_reached = goal_speed.geometry.contains(Point(ego.x, ego.y))
        cr_state = CustomState(
            time_step=k + 1,
            position=np.array([ego.x, ego.y]),
            orientation=ego.yaw,
            velocity=ego.v,
            acceleration=ego.a,
        )
        if problem.goal.is_reached(cr_state):
            status = "success"
            break

    v = np.array([s.v for s in states])
    a = np.array([s.a for s in states])
    kappa = np.array([s.kappa for s in states])
    jerk = np.diff(a) / dt if len(a) > 1 else np.zeros(1)
    xy = np.array([[s.x, s.y] for s in states])
    moving = [x for x in leads if x[2] > 1.0 and x[3]]  # following: moving and keeping the lane
    row.update(
        status=status,
        success=status == "success",
        collision=status == "collision",
        off_road=status == "off_road",
        goal_lanelet_reached=goal_lanelet_reached,
        goal_area_reached=goal_area_reached or status == "success",
        steps=len(states) - 1,
        fallback_steps=fallbacks,
        emergency_steps=emergencies,
        degraded_steps=degraded,
        smoothed_steps=smoothed,
        min_time_headway=min((x[0] for x in moving), default=math.inf),
        min_ttc=min((x[1] for x in moving), default=math.inf),
        time_below_1s_headway=dt * sum(1 for x in moving if x[0] < 1.0),
        min_clearance=min_clearance,
        min_pet=min_post_encroachment_time(track, dt),
        distance=float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0,
        mean_speed=float(np.mean(v)),
        rms_jerk=float(np.sqrt(np.mean(jerk**2))),
        max_abs_accel=float(np.max(np.abs(a))),
        max_lat_accel=float(np.max(np.abs(v**2 * kappa))),
        plan_ms_mean=float(np.mean(plan_ms)) if plan_ms else math.nan,
        plan_ms_p95=float(np.percentile(plan_ms, 95)) if plan_ms else math.nan,
        plan_ms_max=float(np.max(plan_ms)) if plan_ms else math.nan,
        cycle_ms_mean=float(np.mean(cycle_ms)) if cycle_ms else math.nan,
        generated_mean=float(np.mean(generated)) if generated else math.nan,
        validated_mean=float(np.mean(validated)) if validated else math.nan,
        collision_checks_mean=float(np.mean(checks)) if checks else math.nan,
    )
    if record_data is not None:
        record_data["ego"] = xy.tolist()
        record_data["vehicle"] = (vehicle.length, vehicle.width)
        record_data["yaw"] = [s.yaw for s in states]
    return row, record_data


def _worker(task: tuple[str, Options]) -> dict:
    path, opts = task
    try:
        row, _ = run_scenario(path, opts)
    except Exception as e:  # noqa: BLE001 - one broken scenario must not stop the benchmark
        row = {
            "scenario": Path(path).stem,
            "planner": opts.planner + ("+qp" if opts.smoother else ""),
            "status": "error",
            "error": f"{type(e).__name__}: {e}",
            "trace": traceback.format_exc(limit=3),
        }
    return row


def summarize(rows: list[dict]):
    import pandas as pd

    df = pd.DataFrame(rows)
    ok = df[df["status"] != "error"].copy()
    for col in ("success", "collision", "off_road", "goal_lanelet_reached", "goal_area_reached"):
        ok[col] = ok[col].astype(bool)
    ok["timeout"] = ok["status"] == "timeout"
    ok["no_plan"] = ok["status"] == "no_plan"
    g = ok.groupby("planner")
    summary = pd.DataFrame(
        {
            "scenarios": g.size(),
            "success_%": 100 * g["success"].mean(),
            "collision_%": 100 * g["collision"].mean(),
            "off_road_%": 100 * g["off_road"].mean(),
            "timeout_%": 100 * g["timeout"].mean(),
            "no_plan_%": 100 * g["no_plan"].mean(),
            "goal_area_%": 100 * g["goal_area_reached"].mean(),
            "plan_ms_mean": g["plan_ms_mean"].mean(),
            "plan_ms_p95": g["plan_ms_p95"].mean(),
            "generated": g["generated_mean"].mean(),
            "validated": g["validated_mean"].mean(),
            "rms_jerk": g["rms_jerk"].mean(),
            "thw_below_1s_s": g["time_below_1s_headway"].mean(),
            "min_ttc_below_2s_%": 100 * g["min_ttc"].apply(lambda s: (s < 2.0).mean()),
        }
    )
    known = PLANNERS + [p + "+qp" for p in PLANNERS]
    order = [p for p in known if p in summary.index] + [p for p in summary.index if p not in known]
    errors = int((df["status"] == "error").sum())
    return summary.loc[order], errors


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", nargs="+", required=True, help="CommonRoad .xml files or directories")
    ap.add_argument("--planners", nargs="+", default=PLANNERS)
    ap.add_argument("--samples", nargs=3, type=int, default=[5, 5, 5], metavar=("W", "V", "T"))
    ap.add_argument(
        "--adjacent-lanes", type=int, default=1, help="lanes sampled on each side (0 = ego lane only)"
    )
    ap.add_argument(
        "--prediction",
        choices=["ground_truth", "constant_velocity", "lane_following"],
        default="ground_truth",
    )
    ap.add_argument("--no-safety-envelope", action="store_true", help="disable the hard gap/TTC envelope")
    ap.add_argument("--no-proximity-cost", action="store_true", help="disable gap costs and car following")
    ap.add_argument("--config", help="planner YAML config (sampling counts and planner type are overridden)")
    ap.add_argument("--smoother", action="store_true", help="refine trajectories with the QP smoother")
    ap.add_argument("--min-t", type=float)
    ap.add_argument("--max-t", type=float)
    ap.add_argument(
        "--limit", type=int, default=0, help="use the first N scenarios after shuffling with --seed"
    )
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", default="results/commonroad.csv")
    args = ap.parse_args(argv)

    files: list[str] = []
    for item in args.scenarios:
        p = Path(item)
        files += sorted(str(f) for f in p.rglob("*.xml")) if p.is_dir() else [str(p)]
    if args.limit:
        rng = np.random.default_rng(args.seed)
        files = sorted(rng.permutation(files)[: args.limit].tolist())
    tasks = [
        (
            f,
            Options(
                planner=pl,
                samples=tuple(args.samples),
                adjacent_lanes=args.adjacent_lanes,
                prediction=args.prediction,
                config=args.config,
                min_t=args.min_t,
                max_t=args.max_t,
                smoother=args.smoother,
                safety_envelope=False if args.no_safety_envelope else None,
                proximity_cost=False if args.no_proximity_cost else None,
            ),
        )
        for f in files
        for pl in args.planners
    ]
    print(
        f"{len(files)} scenarios x {len(args.planners)} planners = {len(tasks)} runs, {args.jobs} workers",
        file=sys.stderr,
    )

    rows: list[dict] = []
    t0 = time.time()
    with mp.get_context("spawn").Pool(args.jobs) as pool:
        for i, row in enumerate(pool.imap_unordered(_worker, tasks, chunksize=1), 1):
            rows.append(row)
            if i % 25 == 0 or i == len(tasks):
                print(f"  {i}/{len(tasks)} done ({time.time() - t0:.0f} s)", file=sys.stderr)

    import pandas as pd

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    df = pd.DataFrame(rows).sort_values(["scenario", "planner"])
    df.to_csv(out, index=False)
    summary, errors = summarize(rows)
    md = summary.to_markdown(floatfmt=".1f") if hasattr(summary, "to_markdown") else summary.to_string()
    out.with_suffix(".md").write_text(md + "\n")
    meta = {"args": vars(args), "runs": len(rows), "errors": errors, "wall_s": time.time() - t0}
    out.with_suffix(".json").write_text(json.dumps(meta, indent=2))
    print(md)
    if errors:
        print(f"\n{errors} runs raised errors; see the 'error' column in {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
