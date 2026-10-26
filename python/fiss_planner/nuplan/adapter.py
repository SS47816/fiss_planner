"""Conversions between nuPlan (nuplan-devkit) and the planner.

nuPlan lanes and lane connectors become map lanes with the same (integer) ids;
adjacent lanes are neighbours with lane changes allowed. Every lane connector
gets an inactive stop line at its start, switched on while its traffic light
is red (see :func:`stop_line_states`). The route follows the mission's
roadblocks lane by lane, with the fewest lane changes.
"""

from __future__ import annotations

import heapq
import math

import numpy as np
from nuplan.common.actor_state.ego_state import EgoState as NuEgoState
from nuplan.common.actor_state.state_representation import Point2D, StateSE2, StateVector2D, TimePoint
from nuplan.common.actor_state.tracked_objects_types import TrackedObjectType
from nuplan.common.actor_state.vehicle_parameters import VehicleParameters
from nuplan.common.geometry.compute import principal_value
from nuplan.common.maps.abstract_map import AbstractMap
from nuplan.common.maps.maps_datatypes import SemanticMapLayer, TrafficLightStatusType

import fiss_planner as fp

LANE_LAYERS = [SemanticMapLayer.LANE, SemanticMapLayer.LANE_CONNECTOR]
BLOCK_LAYERS = [SemanticMapLayer.ROADBLOCK, SemanticMapLayer.ROADBLOCK_CONNECTOR]


def _xy(polyline) -> np.ndarray:
    return np.ascontiguousarray([(p.x, p.y) for p in polyline.discrete_path], dtype=float)


def _ids(objects) -> list[int]:
    return [int(o.id) for o in objects]


def map_lanes(
    map_api: AbstractMap, center: tuple[float, float], radius: float, route_roadblock_ids=()
) -> dict:
    """{lane id: nuPlan lane or lane connector} within `radius` of `center`
    and in the given roadblocks (or roadblock connectors)."""
    found = map_api.get_proximal_map_objects(Point2D(*center), radius, LANE_LAYERS)
    lanes = {int(o.id): o for layer in LANE_LAYERS for o in found[layer]}
    for rb_id in route_roadblock_ids:
        block = None
        for layer in BLOCK_LAYERS:
            block = map_api.get_map_object(str(rb_id), layer)
            if block is not None:
                break
        if block is not None:
            lanes.update({int(o.id): o for o in block.interior_edges})
    return lanes


def road_map_from_nuplan(lanes: dict) -> fp.RoadMap:
    """RoadMap of nuPlan lanes ({id: lane}, see :func:`map_lanes`). Links to
    lanes outside the set are dropped."""
    road = fp.RoadMap()
    for lid, obj in lanes.items():
        lane = fp.MapLane()
        lane.id = lid
        lane.centerline = _xy(obj.baseline_path)
        lane.left = _xy(obj.left_boundary)
        lane.right = _xy(obj.right_boundary)
        lane.successors = [i for i in _ids(obj.outgoing_edges) if i in lanes]
        lane.predecessors = [i for i in _ids(obj.incoming_edges) if i in lanes]
        if hasattr(obj, "adjacent_edges"):
            left, right = obj.adjacent_edges
            if left is not None and int(left.id) in lanes:
                lane.left_neighbor = int(left.id)
                lane.can_change_left = True
            if right is not None and int(right.id) in lanes:
                lane.right_neighbor = int(right.id)
                lane.can_change_right = True
        limit = obj.speed_limit_mps
        lane.speed_limit = float(limit) if limit else math.inf
        road.add_lane(lane)
        if obj.__class__.__name__.endswith("LaneConnector"):
            road.add_stop_line(fp.StopLine(lid, lid, 0.0, False))
    road.finalize()
    return road


def stop_line_states(road: fp.RoadMap, traffic_lights) -> dict[int, bool]:
    """Active state of every lane connector's stop line: on while its light is red."""
    red = {int(t.lane_connector_id) for t in traffic_lights or () if t.status == TrafficLightStatusType.RED}
    return {sl.id: sl.id in red for sl in road.stop_lines}


def _lane_costs(lanes, road: fp.RoadMap, x: float, y: float, yaw: float, candidates=None) -> dict[int, float]:
    """{lane id: distance + 5 * heading difference} of the lanes pointing the ego's way."""
    out = {}
    for lid in candidates if candidates is not None else lanes:
        lane = road.lane(lid)
        if lane is None or len(lane.centerline) < 2:
            continue
        cl = lane.centerline
        d = np.hypot(cl[:, 0] - x, cl[:, 1] - y)
        k2 = min(int(np.argmin(d)) + 1, len(cl) - 1)
        k1 = k2 - 1
        heading = math.atan2(cl[k2, 1] - cl[k1, 1], cl[k2, 0] - cl[k1, 0])
        dyaw = abs(principal_value(heading - yaw))
        if dyaw < 0.5 * math.pi:
            out[lid] = float(d.min()) + 5.0 * dyaw
    return out


def _block(map_api: AbstractMap, block_id: str):
    for layer in BLOCK_LAYERS:
        obj = map_api.get_map_object(str(block_id), layer)
        if obj is not None:
            return obj
    return None


def _ego_blocks(map_api: AbstractMap, x: float, y: float, yaw: float) -> list[str]:
    """Roadblocks (or connectors) of the lanes containing the ego that point its way, best aligned first."""
    point = Point2D(x, y)
    found = []
    for layer in LANE_LAYERS:
        objs = map_api.get_all_map_objects(point, layer)
        if not objs:
            near = map_api.get_proximal_map_objects(point, 3.0, [layer])[layer]
            objs = near
        for obj in objs:
            state = obj.baseline_path.get_nearest_pose_from_position(point)
            dyaw = abs(principal_value(state.heading - yaw))
            if dyaw < 0.5 * math.pi:
                found.append((dyaw, obj.get_roadblock_id()))
    return list(dict.fromkeys(b for _, b in sorted(found)))


def _connect(map_api: AbstractMap, start: str, targets: set, max_depth: int) -> list[str]:
    """Shortest chain of roadblocks from `start` to one of `targets` (both ends
    included), following roadblock successors; [] if none within `max_depth`."""
    parent, frontier = {start: None}, [start]
    for _ in range(max_depth):
        nxt = []
        for bid in frontier:
            if bid in targets:
                chain = []
                while bid is not None:
                    chain.append(bid)
                    bid = parent[bid]
                return chain[::-1]
            block = _block(map_api, bid)
            for succ in block.outgoing_edges if block is not None else ():
                if succ.id not in parent:
                    parent[succ.id] = bid
                    nxt.append(succ.id)
        frontier = nxt
    return []


def correct_route(
    map_api: AbstractMap, route_roadblock_ids, x: float, y: float, yaw: float, max_depth: int = 8
):
    """The mission's roadblocks, starting where the ego is: the route is cut
    before the ego's roadblock, or joined from it when the ego starts off the
    route (the shortest chain of successors), and gaps between consecutive
    roadblocks are bridged the same way."""
    route = [str(b) for b in route_roadblock_ids]
    fixed = [route[0]] if route else []
    for bid in route[1:]:
        prev = _block(map_api, fixed[-1])
        if prev is not None and bid not in {s.id for s in prev.outgoing_edges}:
            bridge = _connect(map_api, fixed[-1], {bid}, max_depth)
            fixed += bridge[1:-1]
        fixed.append(bid)
    index = {b: i for i, b in enumerate(fixed)}
    for start in _ego_blocks(map_api, x, y, yaw):
        if start in index:
            return fixed[index[start] :]
        chain = _connect(map_api, start, set(fixed), max_depth)
        if chain:
            return chain[:-1] + fixed[index[chain[-1]] :]
    return fixed


def route_lanes(
    lanes: dict, road: fp.RoadMap, route_roadblock_ids, x: float, y: float, yaw: float
) -> list[int]:
    """Lane ids from the ego's lane through the route's roadblocks to its last
    one, with the fewest lane changes (then the shortest); [] if the route
    cannot be reached."""
    order = {str(rb): i for i, rb in enumerate(route_roadblock_ids)}
    on_route = {lid for lid, obj in lanes.items() if obj.get_roadblock_id() in order}
    if not on_route:
        return []
    # Start from every lane near the ego (within 3 m of the best fit), so the
    # search can pick the one that leads into the route.
    near = _lane_costs(lanes, road, x, y, yaw, on_route) or _lane_costs(lanes, road, x, y, yaw)
    if not near:
        return []
    best = min(near.values())
    starts = {lid: c - best for lid, c in near.items() if c <= best + 3.0}
    allowed = on_route | set(starts)
    last = len(route_roadblock_ids) - 1
    goal_rank = max(order[lanes[lid].get_roadblock_id()] for lid in on_route)
    targets = {lid for lid in on_route if order[lanes[lid].get_roadblock_id()] == min(goal_rank, last)}
    queue = [(c, lid) for lid, c in starts.items()]
    heapq.heapify(queue)
    cost, parent = dict(starts), {lid: None for lid in starts}
    while queue:
        c, lid = heapq.heappop(queue)
        if c > cost[lid]:
            continue
        if lid in targets:
            path = []
            while lid is not None:
                path.append(lid)
                lid = parent[lid]
            return path[::-1]
        lane = road.lane(lid)
        steps = [(s, lane.length()) for s in lane.successors]
        steps += [(n, 1000.0) for n in (lane.left_neighbor, lane.right_neighbor) if n >= 0]
        for nxt, w in steps:
            if nxt in allowed and c + w < cost.get(nxt, math.inf):
                cost[nxt], parent[nxt] = c + w, lid
                heapq.heappush(queue, (c + w, nxt))
    return []


def vehicle_params(params: VehicleParameters, base: fp.VehicleParams | None = None) -> fp.VehicleParams:
    """Geometry of the nuPlan ego vehicle; limits come from `base`."""
    v = base if base is not None else fp.VehicleParams()
    v.length = params.length
    v.width = params.width
    v.wheelbase = params.wheel_base
    v.rear_axle_to_center = params.rear_axle_to_center
    return v


def ego_state(ego: NuEgoState) -> fp.EgoState:
    params = ego.car_footprint.vehicle_parameters
    c = ego.center
    dyn = ego.dynamic_car_state
    v = float(np.hypot(dyn.center_velocity_2d.x, dyn.center_velocity_2d.y))
    kappa = math.tan(ego.tire_steering_angle) / params.wheel_base
    return fp.EgoState(c.x, c.y, c.heading, v, dyn.center_acceleration_2d.x, kappa)


_VEHICLES = {TrackedObjectType.VEHICLE}


class ObstacleIds:
    """Stable integer ids for nuPlan track tokens."""

    def __init__(self):
        self._ids: dict[str, int] = {}

    def __call__(self, token: str) -> int:
        return self._ids.setdefault(token, len(self._ids) + 1)


def obstacles(
    tracked_objects,
    ids: ObstacleIds,
    road: fp.RoadMap,
    ego_xy: tuple[float, float],
    horizon: float,
    dt: float,
    max_range: float = 100.0,
    prediction: str = "lane_following",
) -> list[fp.Obstacle]:
    """Agents with predictions (vehicles along their lane, others at constant
    velocity) and static objects, within `max_range` of the ego."""
    out = []
    for obj in tracked_objects.tracked_objects:
        c = obj.box.center
        if math.hypot(c.x - ego_xy[0], c.y - ego_xy[1]) > max_range:
            continue
        oid = ids(obj.track_token)
        velocity = getattr(obj, "velocity", None)
        if velocity is None:  # static object
            ob = fp.predict_constant_velocity(
                oid, c.x, c.y, c.heading, 0.0, obj.box.length, obj.box.width, horizon, dt
            )
            ob.is_static = True
        else:
            speed = float(math.hypot(velocity.x, velocity.y))
            heading = math.atan2(velocity.y, velocity.x) if speed > 0.5 else c.heading
            size = (obj.box.length, obj.box.width)
            if obj.tracked_object_type in _VEHICLES and prediction == "lane_following":
                ob = fp.predict_lane_following(road, oid, c.x, c.y, heading, speed, *size, horizon, dt)
            else:
                ob = fp.predict_constant_velocity(oid, c.x, c.y, heading, speed, *size, horizon, dt)
        out.append(ob)
    return out


def trajectory_states(traj, ego: NuEgoState, t0_us: int) -> list[NuEgoState]:
    """nuPlan ego states along a planned trajectory (t relative to `t0_us`, the
    simulation time, which may precede the ego state's time stamp slightly).
    The trajectory's own state at `t_now` comes first (with stitching it is the
    previous plan's, so the tracker corrects the deviation from it); the
    measured state is used only if the trajectory starts later."""
    params = ego.car_footprint.vehicle_parameters
    t, x, y, yaw, v, a, kappa = (
        np.asarray(getattr(traj, k)) for k in ("t", "x", "y", "yaw", "v", "a", "kappa")
    )
    states = [] if len(t) and t[0] < 1e-3 else [ego]
    for i in range(len(t)):
        states.append(
            NuEgoState.build_from_center(
                center=StateSE2(float(x[i]), float(y[i]), float(yaw[i])),
                center_velocity_2d=StateVector2D(float(v[i]), 0.0),
                center_acceleration_2d=StateVector2D(float(a[i]), 0.0),
                tire_steering_angle=float(math.atan(kappa[i] * params.wheel_base)),
                time_point=TimePoint(t0_us + int(round(max(0.0, float(t[i])) * 1e6))),
                vehicle_parameters=params,
            )
        )
    return states
