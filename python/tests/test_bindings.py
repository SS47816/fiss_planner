"""Smoke tests of the Python bindings."""

import math

import fiss_planner as fp
import numpy as np
import pytest


def straight(length=300.0):
    x = np.arange(0.0, length + 1.0, 5.0)
    return fp.ReferenceLine(x, np.zeros_like(x))


def test_reference_line_roundtrip():
    ref = straight()
    assert ref.length == pytest.approx(300.0, abs=1e-6)
    p = ref.project(30.0, 1.5)
    assert (p.s, p.d) == pytest.approx((30.0, 1.5), abs=1e-6)
    fs = ref.to_frenet(fp.EgoState(10.0, -1.0, 0.1, 8.0))
    back = ref.to_cartesian(fs)
    assert (back.x, back.y, back.yaw, back.v) == pytest.approx((10.0, -1.0, 0.1, 8.0), abs=1e-6)
    pts = ref.sample(10.0)
    assert pts.shape == (31, 5)


@pytest.mark.parametrize("name", ["fop", "fop+", "fiss", "fiss+"])
def test_planners_avoid_obstacle(name):
    ref = straight()
    corridor = fp.Corridor.parallel_lanes(0.0, ref.length, 3.5, 1, 0)
    obstacles = [fp.predict_constant_velocity(1, 45.0, 0.0, 0.0, 0.0, 4.5, 1.9, 6.0, 0.1)]
    planner = fp.make_planner(fp.make_config(name))
    res = planner.plan(fp.FrenetState(10.0, 10.0), ref, corridor, obstacles, target_speed=12.0)
    assert res.success
    traj = res.trajectory
    assert traj.lane == 1
    assert isinstance(traj.x, np.ndarray) and len(traj.x) == len(traj)
    assert res.stats.num_generated > 0
    assert set(res.stats.rejections) >= {"collision", "off_road"}


def test_obstacle_from_arrays_and_box():
    t = np.linspace(0, 5, 51)
    ob = fp.Obstacle.from_arrays(3, 4.0, 2.0, t, 2.0 * t, np.zeros_like(t), np.zeros_like(t))
    box = ob.box_at(2.5)
    assert box.cx == pytest.approx(5.0)
    assert ob.box_at(10.0) is None


def test_config_yaml():
    cfg, vehicle = fp.parse_planner_config("planner: {type: fiss}\nvehicle: {length: 5.0}")
    assert cfg.type == fp.PlannerType.FISS
    assert vehicle.length == 5.0
    cfg.sampling.num_width = 7  # nested structs are mutable in place
    assert cfg.sampling.num_width == 7
    with pytest.raises(ValueError):
        fp.parse_planner_config("sampling: {bogus: 1}")


def two_lane_map():
    m = fp.RoadMap()
    for k in range(3):
        for lane, base in ((0, 1), (1, 11)):
            ln = fp.MapLane()
            ln.id = base + k
            xs = np.arange(60.0 * k, 60.0 * (k + 1) + 1e-9, 5.0)
            yc = 3.5 * lane
            ln.centerline = np.column_stack([xs, np.full_like(xs, yc)])
            ln.left = np.column_stack([xs, np.full_like(xs, yc + 1.75)])
            ln.right = np.column_stack([xs, np.full_like(xs, yc - 1.75)])
            if k < 2:
                ln.successors = [ln.id + 1]
            if lane == 0:
                ln.left_neighbor, ln.can_change_left = 11 + k, True
            else:
                ln.right_neighbor, ln.can_change_right = 1 + k, True
            m.add_lane(ln)
    m.finalize()
    return m


def test_local_planner_closed_loop_on_map():
    lp = fp.LocalPlanner(fp.LocalPlannerConfig())
    lp.set_map(two_lane_map(), [1, 11, 12, 13])
    ego = fp.EgoState(0.0, 0.0, 0.0, 10.0)
    for i in range(80):
        out = lp.step(0.1 * i, ego)
        assert out.success
        ego = out.trajectory.sample(0.1)
    assert ego.y == pytest.approx(3.5, abs=0.5)
    lp.request_planner_type(fp.PlannerType.FOP)
    lp.step(8.0, ego)
    assert lp.planner_type == fp.PlannerType.FOP


def test_route_corridor_bindings():
    rc = fp.build_route_corridor(two_lane_map(), [], fp.EgoState(10.0, 0.0, 0.0, 5.0))
    assert rc is not None and rc.ego_lane == 1
    offsets = sorted(lane.offset for lane in rc.corridor.lanes)
    assert offsets == [0, 1]
    lo, hi = rc.corridor.road_bounds(30.0)
    assert (lo, hi) == pytest.approx((-1.75, 5.25), abs=1e-3)
    assert math.isinf(rc.speed_limit)


def test_route_goal_and_smoothing_bindings():
    m = two_lane_map()
    ego = fp.EgoState(70.0, 0.0, 0.0, 10.0)
    cfg = fp.RouteCorridorConfig()
    assert (cfg.smoothing_length, cfg.smoothing_tolerance, cfg.urgency_time) == (20.0, 0.1, 10.0)
    # Route 1 -> 2 -> 3 -> 13: the change into lane 13 is due at the goal, not at the road's end.
    far = fp.build_route_corridor(m, [1, 2, 3, 13], ego, cfg)
    near = fp.build_route_corridor(m, [1, 2, 3, 13], ego, cfg, -1, (130.0, 3.5))
    ego_lane = {lane.offset: lane for lane in near.corridor.lanes}[0]
    assert ego_lane.urgency > {lane.offset: lane for lane in far.corridor.lanes}[0].urgency
    assert ego_lane.extra_cost > 0.0
    lp = fp.LocalPlanner(fp.LocalPlannerConfig())
    lp.set_map(m, [1, 2, 3, 13], goal=(130.0, 3.5))
    assert lp.step(0.0, ego).success
    x, y = fp.smooth_polyline([0.0, 10.0, 20.0], [0.0, 0.05, 0.0], 20.0, 0.1)
    assert len(x) == len(y) >= 21  # resampled every metre
    assert max(abs(v) for v in y) <= 0.05 + 1e-9


def test_controllers_and_bicycle():
    v = fp.VehicleParams()
    ref = straight()
    corridor = fp.Corridor.parallel_lanes(0.0, ref.length, 3.5, 1, 0)
    res = fp.make_planner(fp.make_config("fop")).plan(fp.FrenetState(10.0, 10.0), ref, corridor, [], 12.0)
    for ctrl in (fp.make_controller("stanley_pid", v), fp.make_controller("mpc", v)):
        car = fp.KinematicBicycle(v, fp.EgoState(10.0, 0.3, 0.0, 10.0))
        for i in range(60):
            car.step(ctrl.compute(car.state, res.trajectory, 0.05 * i), 0.05)
        assert abs(car.state.y) < 0.2, ctrl.name


def test_smoother_flag():
    cfg = fp.LocalPlannerConfig()
    cfg.smoother.enabled = True
    lp = fp.LocalPlanner(cfg)
    lp.set_reference_path([-20.0, 100.0, 300.0], [0.0, 0.0, 0.0])
    obs = [fp.predict_constant_velocity(1, 45.0, 0.0, 0.0, 0.0, 4.5, 1.9, 6.0, 0.1)]
    out = lp.step(0.0, fp.EgoState(10.0, 0.0, 0.0, 10.0), obs)
    assert out.success and out.smoothed == fp.has_optimizers()


def test_embedded_msg_definitions_match_package():
    from pathlib import Path

    from fiss_planner.bags.msg_definitions import MSG_DEFINITIONS

    msg_dir = Path(__file__).resolve().parents[2] / "msgs" / "fiss_planner_msgs" / "msg"
    files = {f.stem: f.read_text() for f in msg_dir.glob("*.msg")}
    assert files == MSG_DEFINITIONS


def test_keeping_distance_api():
    ref = straight()
    lane = fp.Corridor.parallel_lanes(0.0, ref.length, 3.5, 0, 0)
    lead = fp.predict_constant_velocity(7, 30.0, 0.0, 0.0, 8.0, 4.5, 1.9, 6.0, 0.1)
    cfg = fp.make_config("fiss_plus")
    assert cfg.safety.enabled and cfg.cost.follow_time > 0.0
    res = fp.make_planner(cfg).plan(fp.FrenetState(10.0, 8.0), ref, lane, [lead], target_speed=15.0)
    assert res.success
    assert res.stats.lead.id == 7
    assert res.stats.lead.gap == pytest.approx(30.0 - 10.0 - 4.5, abs=1e-6)
    assert res.stats.lead.speed == pytest.approx(8.0, abs=1e-6)
    assert res.trajectory.cost.proximity >= 0.0
    assert "safety_distance" in res.stats.rejections
    assert fp.Rejection.SAFETY_DISTANCE.name == "SAFETY_DISTANCE"

    blocked = [fp.predict_constant_velocity(9, 16.0, 0.0, 0.0, 0.0, 4.5, 1.9, 6.0, 0.1)]
    bad = fp.make_planner(cfg).plan(fp.FrenetState(0.0, 15.0), ref, lane, blocked, target_speed=15.0)
    assert not bad.success and bad.stats.blocking_id == 9


def test_lane_following_prediction():
    road = fp.RoadMap()
    lane = fp.MapLane()
    lane.id = 1
    xs = np.arange(0.0, 101.0, 5.0)
    lane.centerline = np.column_stack([xs, np.zeros_like(xs)])
    lane.left = np.column_stack([xs, np.full_like(xs, 1.75)])
    lane.right = np.column_stack([xs, np.full_like(xs, -1.75)])
    road.add_lane(lane)
    road.finalize()
    ob = fp.predict_lane_following(road, 3, 10.0, 0.5, 0.1, 10.0, 4.5, 1.9, 3.0, 0.1)
    last = ob.prediction[-1]
    assert last.x == pytest.approx(40.0, abs=0.5)  # along the lane, not along its 0.1 rad heading
    assert abs(last.y) < 0.1

    tracked = fp.Obstacle.from_arrays(4, 4.5, 1.9, [0.0], [10.0], [0.0], [0.0], False)
    tracked.prediction = [fp.ObstacleState(0.0, 10.0, 0.0, 0.0, 10.0)]
    out = fp.complete_predictions([tracked], "lane_following", road, 3.0, 0.1)
    assert len(out[0].prediction) == 31
    with pytest.raises(ValueError):
        fp.complete_predictions([tracked], "kalman")
