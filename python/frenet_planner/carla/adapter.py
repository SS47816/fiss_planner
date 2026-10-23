"""CARLA <-> frenet_planner conversions.

CARLA uses a left-handed frame (x forward, y right, yaw clockwise). The
planner works in a right-handed frame, so every conversion mirrors y and
yaw: (x, y, yaw) -> (x, -y, -yaw).
"""

from __future__ import annotations

import math
from collections import defaultdict

import numpy as np

from .. import _core as fp


def to_planner(x: float, y: float, yaw_deg: float = 0.0) -> tuple[float, float, float]:
    return x, -y, -math.radians(yaw_deg)


def to_carla(x: float, y: float, yaw: float = 0.0) -> tuple[float, float, float]:
    return x, -y, -math.degrees(yaw)


def _lane_key(wp) -> tuple[int, int, int]:
    return (wp.road_id, wp.section_id, wp.lane_id)


def road_map_from_carla(carla_map, spacing: float = 2.0):
    """Build a RoadMap from CARLA waypoints.

    Every (road, section, lane) of driving type becomes a lane; neighbours are
    the same-direction driving lanes on either side, and lane changes follow
    the waypoints' ``lane_change`` flags (derived from the road markings).
    Returns (road_map, key_to_id) so that route waypoints can be mapped."""
    import carla

    groups: dict[tuple, list] = defaultdict(list)
    for wp in carla_map.generate_waypoints(spacing):
        if wp.lane_type == carla.LaneType.Driving:
            groups[_lane_key(wp)].append(wp)
    key_to_id = {k: i + 1 for i, k in enumerate(sorted(groups))}

    road = fp.RoadMap()
    for key, wps in groups.items():
        wps.sort(key=lambda w: w.s if w.lane_id < 0 else -w.s)  # driving direction
        # Close the lane at its exact end so consecutive lanes connect.
        end = wps[-1].next(spacing * 0.999)
        ends = [e for e in end if _lane_key(e) == key]
        pts = wps + ends[:1]
        if len(pts) < 2:
            continue
        center, left, right = [], [], []
        for w in pts:
            t = w.transform
            x, y, yaw = to_planner(t.location.x, t.location.y, t.rotation.yaw)
            hw = 0.5 * w.lane_width
            center.append((x, y))
            left.append((x - math.sin(yaw) * hw, y + math.cos(yaw) * hw))
            right.append((x + math.sin(yaw) * hw, y - math.cos(yaw) * hw))
        lane = fp.MapLane()
        lane.id = key_to_id[key]
        lane.centerline = np.asarray(center)
        lane.left = np.asarray(left)
        lane.right = np.asarray(right)

        mid = wps[len(wps) // 2]
        # CARLA "left" is the driver's left in its frame, which remains the
        # driver's left after mirroring.
        for side in ("left", "right"):
            nb = mid.get_left_lane() if side == "left" else mid.get_right_lane()
            if nb is None or nb.lane_type != carla.LaneType.Driving or (nb.lane_id * mid.lane_id) <= 0:
                continue
            nid = key_to_id.get(_lane_key(nb))
            if nid is None:
                continue
            allowed = mid.lane_change in (
                (carla.LaneChange.Left, carla.LaneChange.Both)
                if side == "left"
                else (carla.LaneChange.Right, carla.LaneChange.Both)
            )
            if side == "left":
                lane.left_neighbor, lane.can_change_left = nid, bool(allowed)
            else:
                lane.right_neighbor, lane.can_change_right = nid, bool(allowed)

        succ = set()
        for nxt in wps[-1].next(spacing * 1.5):
            k = _lane_key(nxt)
            if k != key and k in key_to_id:
                succ.add(key_to_id[k])
        lane.successors = sorted(succ)
        road.add_lane(lane)
    road.finalize()
    return road, key_to_id


def route_lane_ids(route_waypoints, key_to_id) -> list[int]:
    ids: list[int] = []
    for wp in route_waypoints:
        lid = key_to_id.get(_lane_key(wp))
        if lid is not None and (not ids or ids[-1] != lid):
            ids.append(lid)
    return ids


def vehicle_params(vehicle) -> fp.VehicleParams:
    """Geometry and steering limit of a CARLA vehicle actor."""
    pc = vehicle.get_physics_control()
    ext = vehicle.bounding_box.extent
    v = fp.VehicleParams()
    v.length = 2.0 * ext.x
    v.width = 2.0 * ext.y
    wheels = pc.wheels
    if len(wheels) >= 4:
        # Wheel positions are in world cm; measure along the vehicle heading.
        tf = vehicle.get_transform()
        fwd = tf.get_forward_vector()
        loc = tf.location
        proj = [
            ((w.position.x / 100.0 - loc.x) * fwd.x + (w.position.y / 100.0 - loc.y) * fwd.y) for w in wheels
        ]
        front = 0.5 * (proj[0] + proj[1])
        rear = 0.5 * (proj[2] + proj[3])
        center = vehicle.bounding_box.location.x
        v.wheelbase = max(1.5, front - rear)
        v.rear_axle_to_center = center - rear
        v.max_steer = math.radians(max(w.max_steer_angle for w in wheels[:2]))
    return v


def ego_state(vehicle) -> fp.EgoState:
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    acc = vehicle.get_acceleration()
    bb = vehicle.bounding_box.location
    fwd = tf.get_forward_vector()
    cx = tf.location.x + fwd.x * bb.x
    cy = tf.location.y + fwd.y * bb.x
    x, y, yaw = to_planner(cx, cy, tf.rotation.yaw)
    speed = math.hypot(vel.x, vel.y)
    a = acc.x * fwd.x + acc.y * fwd.y
    yaw_rate = -math.radians(vehicle.get_angular_velocity().z)
    kappa = yaw_rate / speed if speed > 1.0 else 0.0
    return fp.EgoState(x, y, yaw, speed, a, kappa)


def obstacles(
    world,
    ego_id: int,
    horizon: float,
    dt: float,
    max_range: float = 120.0,
    ego_xy=None,
    road_map=None,
    prediction: str = "lane_following",
    velocities=None,
):
    """Other vehicles and walkers with predictions: vehicles follow their lane on
    ``road_map`` (``prediction="lane_following"``), everything else keeps its
    velocity. `velocities` ({actor id: CARLA velocity vector}) replaces the
    simulator's report for actors moved kinematically (physics off), which CARLA
    reports as standing still."""
    out = []
    for actor in world.get_actors():
        is_vehicle = actor.type_id.startswith("vehicle.")
        if actor.id == ego_id or not (is_vehicle or actor.type_id.startswith("walker.")):
            continue
        tf = actor.get_transform()
        if (
            ego_xy is not None
            and math.hypot(tf.location.x - ego_xy[0], -tf.location.y - ego_xy[1]) > max_range
        ):
            continue
        ext = actor.bounding_box.extent
        vel = (velocities or {}).get(actor.id) or actor.get_velocity()
        x, y, yaw = to_planner(tf.location.x, tf.location.y, tf.rotation.yaw)
        speed = math.hypot(vel.x, vel.y)
        heading = math.atan2(-vel.y, vel.x) if speed > 0.5 else yaw
        size = (2 * ext.x + 0.2, 2 * ext.y + 0.2)
        if is_vehicle and road_map is not None and prediction == "lane_following":
            ob = fp.predict_lane_following(road_map, actor.id, x, y, heading, speed, *size, horizon, dt)
        else:
            ob = fp.predict_constant_velocity(actor.id, x, y, heading, speed, *size, horizon, dt)
        out.append(ob)
    return out
