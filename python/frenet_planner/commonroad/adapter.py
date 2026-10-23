"""Conversion between CommonRoad scenarios and frenet_planner types."""

from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np
from commonroad.geometry.shape import Circle, Polygon, Rectangle, ShapeGroup
from commonroad.scenario.lanelet import LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import DynamicObstacle, Obstacle, StaticObstacle
from commonroad.scenario.traffic_light import TrafficLightState

from .. import _core as fp

_NO_CROSSING = {
    LineMarking.SOLID,
    LineMarking.SOLID_SOLID,
    LineMarking.BROAD_SOLID,
    LineMarking.CURB,
    LineMarking.LOWERED_CURB,
}
_LINE_TYPES = {
    LineMarking.DASHED: fp.LineType.DASHED,
    LineMarking.BROAD_DASHED: fp.LineType.DASHED,
    LineMarking.DASHED_DASHED: fp.LineType.DASHED,
    LineMarking.SOLID: fp.LineType.SOLID,
    LineMarking.SOLID_SOLID: fp.LineType.SOLID,
    LineMarking.BROAD_SOLID: fp.LineType.SOLID,
    LineMarking.SOLID_DASHED: fp.LineType.SOLID,
    LineMarking.DASHED_SOLID: fp.LineType.DASHED,
    LineMarking.CURB: fp.LineType.ROAD_EDGE,
    LineMarking.LOWERED_CURB: fp.LineType.ROAD_EDGE,
    LineMarking.NO_MARKING: fp.LineType.VIRTUAL,
}
_STOP_STATES = {TrafficLightState.RED, TrafficLightState.YELLOW, TrafficLightState.RED_YELLOW}


def _speed_limits(network: LaneletNetwork, country) -> dict[int, float]:
    """Speed limit per lanelet from traffic signs (missing -> inf)."""
    limits: dict[int, float] = {}
    if country is None:
        return limits
    try:
        from commonroad.scenario.traffic_sign_interpreter import TrafficSignInterpreter

        interp = TrafficSignInterpreter(country, network)
        for lanelet in network.lanelets:
            v = interp.speed_limit(frozenset({lanelet.lanelet_id}))
            if v is not None and v > 0:
                limits[lanelet.lanelet_id] = float(v)
    except Exception:  # noqa: BLE001 - signs are optional; unsupported countries are common
        pass
    return limits


def road_map_from_lanelet_network(network: LaneletNetwork, country=None) -> fp.RoadMap:
    """Build a RoadMap: lanelets become lanes; same-direction adjacent lanelets
    become neighbours, and lane changes are allowed unless the shared marking
    forbids crossing. Every lanelet stop line gets a stop line with the
    lanelet's id (inactive; see :func:`traffic_light_stop_lines`)."""
    limits = _speed_limits(network, country)
    road = fp.RoadMap()
    for ll in network.lanelets:
        lane = fp.MapLane()
        lane.id = ll.lanelet_id
        lane.centerline = np.ascontiguousarray(ll.center_vertices, dtype=float)
        lane.left = np.ascontiguousarray(ll.left_vertices, dtype=float)
        lane.right = np.ascontiguousarray(ll.right_vertices, dtype=float)
        lane.left_type = _LINE_TYPES.get(ll.line_marking_left_vertices, fp.LineType.UNKNOWN)
        lane.right_type = _LINE_TYPES.get(ll.line_marking_right_vertices, fp.LineType.UNKNOWN)
        if ll.adj_left is not None and ll.adj_left_same_direction:
            lane.left_neighbor = ll.adj_left
            lane.can_change_left = ll.line_marking_left_vertices not in _NO_CROSSING
        if ll.adj_right is not None and ll.adj_right_same_direction:
            lane.right_neighbor = ll.adj_right
            lane.can_change_right = ll.line_marking_right_vertices not in _NO_CROSSING
        lane.successors = list(ll.successor)
        lane.predecessors = list(ll.predecessor)
        lane.speed_limit = limits.get(ll.lanelet_id, math.inf)
        road.add_lane(lane)

        if ll.stop_line is not None and ll.stop_line.start is not None and ll.stop_line.end is not None:
            mid = 0.5 * (np.asarray(ll.stop_line.start) + np.asarray(ll.stop_line.end))
            cl = np.asarray(ll.center_vertices)
            seg = np.linalg.norm(np.diff(cl, axis=0), axis=1)
            s_along = np.concatenate([[0.0], np.cumsum(seg)])
            s = float(s_along[int(np.argmin(np.linalg.norm(cl - mid, axis=1)))])
            road.add_stop_line(fp.StopLine(ll.lanelet_id, ll.lanelet_id, s, False))
    road.finalize()
    return road


def traffic_light_stop_lines(network: LaneletNetwork, time_step: int) -> dict[int, bool]:
    """Active state of each lanelet's stop line at `time_step` (red/yellow lights)."""
    lights = {tl.traffic_light_id: tl for tl in network.traffic_lights}
    states: dict[int, bool] = {}
    for ll in network.lanelets:
        sl = ll.stop_line
        if sl is None:
            continue
        refs = set(sl.traffic_light_ref or ()) | set(ll.traffic_lights or ())
        active = False
        for ref in refs:
            tl = lights.get(ref)
            if tl is not None and tl.active and tl.get_state_at_time_step(time_step) in _STOP_STATES:
                active = True
        states[ll.lanelet_id] = active
    return states


def _footprint(shape) -> tuple[float, float, float, float, float]:
    """(length, width, dx, dy, dyaw) of an oriented box covering the shape, in the obstacle frame."""
    if isinstance(shape, Rectangle):
        c = np.asarray(shape.center, dtype=float)
        return shape.length, shape.width, float(c[0]), float(c[1]), float(shape.orientation)
    if isinstance(shape, Circle):
        c = np.asarray(shape.center, dtype=float)
        return 2 * shape.radius, 2 * shape.radius, float(c[0]), float(c[1]), 0.0
    if isinstance(shape, Polygon):
        v = np.asarray(shape.vertices, dtype=float)
    elif isinstance(shape, ShapeGroup):
        v = np.vstack([np.asarray(_footprint_vertices(s)) for s in shape.shapes])
    else:
        raise TypeError(f"unsupported obstacle shape {type(shape).__name__}")
    lo, hi = v.min(axis=0), v.max(axis=0)
    c = 0.5 * (lo + hi)
    return float(hi[0] - lo[0]), float(hi[1] - lo[1]), float(c[0]), float(c[1]), 0.0


def _footprint_vertices(shape) -> np.ndarray:
    length, width, dx, dy, dyaw = _footprint(shape)
    corners = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * [length / 2, width / 2]
    rot = np.array([[math.cos(dyaw), -math.sin(dyaw)], [math.sin(dyaw), math.cos(dyaw)]])
    return corners @ rot.T + [dx, dy]


def _box_pose(state, dx: float, dy: float, dyaw: float) -> tuple[float, float, float]:
    yaw = float(getattr(state, "orientation", 0.0) or 0.0)
    x, y = (float(v) for v in state.position)
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, yaw + dyaw


def obstacles_at(
    obstacles: Iterable[Obstacle],
    time_step: int,
    dt: float,
    horizon: float,
    prediction: str = "ground_truth",
    road_map: fp.RoadMap | None = None,
) -> list[fp.Obstacle]:
    """Planner obstacles at `time_step`, with predictions relative to that time.

    ``prediction="ground_truth"`` uses the recorded future states (as in the
    FISS+ paper); ``"constant_velocity"`` extrapolates the current state and
    ``"lane_following"`` moves it along its lane of `road_map`."""
    steps = int(round(horizon / dt))
    out: list[fp.Obstacle] = []
    for ob in obstacles:
        if isinstance(ob.obstacle_shape, (Rectangle, Circle, Polygon, ShapeGroup)):
            length, width, dx, dy, dyaw = _footprint(ob.obstacle_shape)
        else:
            continue
        if isinstance(ob, StaticObstacle):
            x, y, yaw = _box_pose(ob.initial_state, dx, dy, dyaw)
            o = fp.Obstacle.from_arrays(ob.obstacle_id, length, width, [0.0], [x], [y], [yaw], True)
            out.append(o)
            continue
        if not isinstance(ob, DynamicObstacle):
            continue
        now = ob.state_at_time(time_step)
        if now is None:
            continue
        if prediction in ("constant_velocity", "lane_following"):
            x, y, yaw = _box_pose(now, dx, dy, dyaw)
            v = float(getattr(now, "velocity", 0.0) or 0.0)
            if prediction == "lane_following" and road_map is not None:
                o = fp.predict_lane_following(
                    road_map, ob.obstacle_id, x, y, yaw, v, length, width, horizon, dt
                )
            else:
                o = fp.predict_constant_velocity(ob.obstacle_id, x, y, yaw, v, length, width, horizon, dt)
            o.is_static = False
            out.append(o)
            continue
        ts, xs, ys, yaws = [], [], [], []
        for j in range(steps + 1):
            st = ob.state_at_time(time_step + j)
            if st is None:
                break
            x, y, yaw = _box_pose(st, dx, dy, dyaw)
            ts.append(j * dt)
            xs.append(x)
            ys.append(y)
            yaws.append(yaw)
        out.append(fp.Obstacle.from_arrays(ob.obstacle_id, length, width, ts, xs, ys, yaws, False))
    return out


def vehicle_params_from_commonroad(
    params, max_accel: float = 4.0, max_decel: float = 8.0, max_lat_accel: float = 8.0
) -> fp.VehicleParams:
    """Ego limits from CommonRoad vehicle parameters (e.g. VW_VANAGON).

    The CommonRoad models carry physical limits (a_max = 11.5 m/s^2 for the
    Vanagon); planning uses more realistic longitudinal bounds. The lateral
    limit is a feasibility bound (~0.8 g friction): comfort is left to the
    cost, since recorded scenarios may start into turns at speed."""
    v = fp.VehicleParams()
    v.length = float(params.l)
    v.width = float(params.w)
    v.wheelbase = float(params.a + params.b)
    v.rear_axle_to_center = float(params.b)
    v.max_speed = float(params.longitudinal.v_max)
    v.max_accel = min(max_accel, float(params.longitudinal.a_max))
    v.max_decel = max_decel
    v.max_steer = float(params.steering.max)
    v.max_steer_rate = float(params.steering.v_max)
    v.max_lat_accel = max_lat_accel
    return v


def ego_state_from_commonroad(state) -> fp.EgoState:
    v = float(getattr(state, "velocity", 0.0) or 0.0)
    a = float(getattr(state, "acceleration", 0.0) or 0.0)
    yaw_rate = float(getattr(state, "yaw_rate", 0.0) or 0.0)
    kappa = yaw_rate / v if abs(v) > 1.0 else 0.0
    x, y = (float(c) for c in state.position)
    return fp.EgoState(x, y, float(state.orientation), v, a, kappa)
