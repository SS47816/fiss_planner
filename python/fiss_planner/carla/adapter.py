"""CARLA <-> fiss_planner conversions.

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


def _region_waypoints(carla_map, spacing: float, points: np.ndarray, radius: float) -> list:
    """Waypoints of the lanes passing within `radius` of `points` (CARLA x, y),
    walked from the map topology instead of sampling the whole map."""

    def dist(loc) -> float:
        return float(np.min(np.hypot(points[:, 0] - loc.x, points[:, 1] - loc.y)))

    out, seen = [], set()
    for a, b in carla_map.get_topology():
        reach = radius + a.transform.location.distance(b.transform.location)
        if min(dist(a.transform.location), dist(b.transform.location)) > reach:
            continue
        key = (_lane_key(a), round(a.s, 1))
        if key in seen:
            continue
        seen.add(key)
        lane = [a] + a.next_until_lane_end(spacing)
        if any(dist(w.transform.location) <= radius for w in lane):
            out.extend(lane)
    return out


def road_map_from_carla(carla_map, spacing: float = 2.0, region: tuple | None = None):
    """Build a RoadMap from CARLA waypoints.

    Every (road, section, lane) of driving type becomes a lane; neighbours are
    the same-direction driving lanes on either side, and lane changes follow
    the waypoints' ``lane_change`` flags (derived from the road markings).
    `region` = (points (N, 2) in CARLA x, y; radius [m]) keeps only the lanes
    passing within the radius of the points (for large towns).
    Returns (road_map, key_to_id) so that route waypoints can be mapped."""
    import carla

    groups: dict[tuple, list] = defaultdict(list)
    if region is None:
        wps = carla_map.generate_waypoints(spacing)
    else:
        wps = _region_waypoints(carla_map, spacing, *region)
    for wp in wps:
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
        lane.junction = any(w.is_junction for w in wps)

        mid = wps[len(wps) // 2]
        opposite = mid.get_left_lane()
        if (
            opposite is not None
            and opposite.lane_type == carla.LaneType.Driving
            and opposite.lane_id * mid.lane_id < 0
        ):
            lane.left_opposite = key_to_id.get(_lane_key(opposite), -1)
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


def traffic_light_stop_lines(road, key_to_id, lights, first_id: int = 1) -> dict[int, object]:
    """Add an inactive stop line for every stop waypoint of the traffic lights
    on a mapped lane; returns {stop line id: traffic light}."""
    out, sid = {}, first_id
    for tl in lights:
        for wp in tl.get_stop_waypoints():
            lid = key_to_id.get(_lane_key(wp))
            lane = road.lane(lid) if lid is not None else None
            if lane is None:
                continue
            x, y, _ = to_planner(wp.transform.location.x, wp.transform.location.y)
            cl = lane.centerline
            k = int(np.argmin(np.hypot(cl[:, 0] - x, cl[:, 1] - y)))
            s = float(np.sum(np.hypot(*np.diff(cl[: k + 1], axis=0).T))) if k > 0 else 0.0
            road.add_stop_line(fp.StopLine(sid, lid, s, False))
            out[sid] = tl
            sid += 1
    return out


def stop_sign_stop_lines(road, key_to_id, carla_map, signs, first_id: int, ahead: float = 3.5) -> list[int]:
    """Add a stop-sign stop line where each sign's trigger area meets the
    driving lanes it covers (same direction as the lane under its center),
    `ahead` [m] beyond the area's center along each branch (into a junction
    if need be): the planner stops a few meters short of a stop line, and the
    leaderboard counts a stop only with the vehicle's center within 4 m of
    that center.
    Returns the new stop line ids."""
    import carla

    ids, sid = [], first_id
    for sign in signs:
        center = sign.get_transform().transform(sign.trigger_volume.location)
        wp = carla_map.get_waypoint(center, lane_type=carla.LaneType.Driving)
        lanes, seen = [], set()
        for step in ("get_left_lane", "get_right_lane"):
            w = wp
            while w is not None and w.lane_type == carla.LaneType.Driving and w.lane_id * wp.lane_id > 0:
                if _lane_key(w) not in seen:
                    seen.add(_lane_key(w))
                    lanes.append(w)
                w = getattr(w, step)()
        for w in lanes:
            # `ahead` further along every branch the lane continues into
            for t in w.next(ahead) or [w]:
                lid = key_to_id.get(_lane_key(t))
                lane = road.lane(lid) if lid is not None else None
                if lane is None:
                    continue
                x, y, _ = to_planner(t.transform.location.x, t.transform.location.y)
                cl = lane.centerline
                k = int(np.argmin(np.hypot(cl[:, 0] - x, cl[:, 1] - y)))
                s_lane = float(np.sum(np.hypot(*np.diff(cl[: k + 1], axis=0).T))) if k > 0 else 0.0
                road.add_stop_line(fp.StopLine(sid, lid, s_lane, True, True))
                ids.append(sid)
                sid += 1
    return ids


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
        # Right after spawning, wheel positions can be stale: keep only a
        # consistent measurement, else axles symmetric about the center.
        wheelbase, rear_to_center = front - rear, center - rear
        if 1.5 <= wheelbase <= v.length and 0.0 <= rear_to_center <= 0.5 * v.length:
            v.wheelbase, v.rear_axle_to_center = wheelbase, rear_to_center
        else:
            v.wheelbase = 0.58 * v.length
            v.rear_axle_to_center = 0.5 * v.wheelbase
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
    ego_z: float | None = None,
):
    """Other vehicles, walkers and static props (standing) with predictions: vehicles follow their lane on
    ``road_map`` (``prediction="lane_following"``), everything else keeps its
    velocity. `velocities` ({actor id: CARLA velocity vector}) replaces the
    simulator's report for actors moved kinematically (physics off), which CARLA
    reports as standing still. With `ego_z`, actors more than 5 m above or
    below the ego are left out."""
    out = []
    for actor in world.get_actors():
        is_vehicle = actor.type_id.startswith("vehicle.")
        # Scenario props (cones, warning signs, ...); generic meshes are map scenery
        # and debris is driven over.
        is_prop = (
            actor.type_id.startswith("static.prop.")
            and actor.type_id != "static.prop.mesh"
            and "debris" not in actor.type_id
        )
        if actor.id == ego_id or not (is_vehicle or is_prop or actor.type_id.startswith("walker.")):
            continue
        tf = actor.get_transform()
        if (
            ego_xy is not None
            and math.hypot(tf.location.x - ego_xy[0], -tf.location.y - ego_xy[1]) > max_range
        ):
            continue
        if ego_z is not None and abs(tf.location.z - ego_z) > 5.0:
            continue  # e.g. scenario actors parked below the map until they are triggered
        ext = actor.bounding_box.extent
        x, y, yaw = to_planner(tf.location.x, tf.location.y, tf.rotation.yaw)
        if is_prop:  # cones, barriers, warning signs: standing, footprint at least 0.5 m
            size = (max(2 * ext.x, 0.5) + 0.2, max(2 * ext.y, 0.5) + 0.2)
            ob = fp.predict_constant_velocity(actor.id, x, y, yaw, 0.0, *size, horizon, dt)
            ob.is_static = True
            out.append(ob)
            continue
        vel = (velocities or {}).get(actor.id) or actor.get_velocity()
        speed = math.hypot(vel.x, vel.y)
        heading = math.atan2(-vel.y, vel.x) if speed > 0.5 else yaw
        size = (2 * ext.x + 0.2, 2 * ext.y + 0.2)
        if is_vehicle and road_map is not None and prediction == "lane_following":
            ob = fp.predict_lane_following(road_map, actor.id, x, y, heading, speed, *size, horizon, dt)
        else:
            ob = fp.predict_constant_velocity(actor.id, x, y, heading, speed, *size, horizon, dt)
        out.append(ob)
    return out
