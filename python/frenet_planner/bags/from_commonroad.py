"""Convert CommonRoad recordings into ROS 2 bags for planner replay.

A recorded vehicle of the scenario becomes the ego ("the logging car"): its
states are written as odometry, the lane chain it starts on as the
reference path, and all other traffic as frenet_planner_msgs/ObstacleArray
with constant-velocity predictions (what an online perception stack would
provide) or ground-truth futures. The bags can be replayed through the ROS 2
node in open loop (see ros/ros2/frenet_planner_ros2/scripts/bag_replay_eval.py)
and its plans compared with what the human driver actually did.

    python -m frenet_planner.bags.from_commonroad --scenarios data/cache/scenarios/NGSIM/US101 \\
        --out data/cache/bags --limit 10
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .msg_definitions import MSG_DEFINITIONS

T0_NS = 1_000_000_000_000  # bag start time (1000 s); stamps are T0 + k * dt


def make_typestore():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS2_JAZZY)
    types = {}
    for name, text in MSG_DEFINITIONS.items():
        types.update(get_types_from_msg(text, f"frenet_planner_msgs/msg/{name}"))
    ts.register(types)
    return ts


def _pick_ego(scenario, min_steps: int, min_speed: float):
    best, best_len = None, 0
    for ob in scenario.dynamic_obstacles:
        traj = getattr(ob.prediction, "trajectory", None)
        if traj is None:
            continue
        states = [ob.initial_state] + list(traj.state_list)
        speeds = [float(getattr(s, "velocity", 0.0) or 0.0) for s in states]
        if len(states) >= min_steps and np.mean(speeds) >= min_speed and len(states) > best_len:
            best, best_len = ob, len(states)
    return best


def _reference(scenario, states, extra: float = 60.0) -> np.ndarray:
    net = scenario.lanelet_network
    visited = set()
    for s in states:
        ids = net.find_lanelet_by_position([np.asarray(s.position)])[0]
        visited.update(ids)
    start = net.find_lanelet_by_position([np.asarray(states[0].position)])[0]
    if not start:
        raise ValueError("ego does not start on a lanelet")
    # Prefer a start lanelet the vehicle stays on the longest.
    chain = [start[0]]
    xy = np.asarray([s.position for s in states])
    travelled = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    length = 0.0
    while length < travelled + extra:
        ll = net.find_lanelet_by_id(chain[-1])
        length += float(np.sum(np.linalg.norm(np.diff(ll.center_vertices, axis=0), axis=1)))
        succ = [s for s in ll.successor if s in visited] or list(ll.successor)
        if not succ or succ[0] in chain:
            break
        chain.append(succ[0])
    pts = []
    for lid in chain:
        for p in net.find_lanelet_by_id(lid).center_vertices:
            if not pts or np.hypot(*(p - pts[-1])) > 0.05:
                pts.append(np.asarray(p, dtype=float))
    return np.asarray(pts)


def convert(
    path: str,
    out_dir: str,
    prediction: str = "constant_velocity",
    horizon: float = 6.0,
    min_steps: int = 80,
    min_speed: float = 3.0,
) -> dict | None:
    from commonroad.common.file_reader import CommonRoadFileReader
    from rosbags.rosbag2 import StoragePlugin, Writer

    from ..commonroad import adapter

    scenario, _ = CommonRoadFileReader(path).open()
    ego = _pick_ego(scenario, min_steps, min_speed)
    if ego is None:
        return None
    states = [ego.initial_state] + list(ego.prediction.trajectory.state_list)
    try:
        ref = _reference(scenario, states)
    except ValueError:
        return None
    others = [ob for ob in scenario.obstacles if ob.obstacle_id != ego.obstacle_id]
    dt = float(scenario.dt)

    ts = make_typestore()
    T = ts.types
    Header, Time = T["std_msgs/msg/Header"], T["builtin_interfaces/msg/Time"]
    Pose, Point, Quat = (
        T["geometry_msgs/msg/Pose"],
        T["geometry_msgs/msg/Point"],
        T["geometry_msgs/msg/Quaternion"],
    )

    def stamp(ns: int):
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def quat(yaw: float):
        return Quat(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))

    name = Path(path).stem
    bag_path = Path(out_dir) / name
    if bag_path.exists():
        import shutil

        shutil.rmtree(bag_path)
    bag_path.parent.mkdir(parents=True, exist_ok=True)
    cov36 = np.zeros(36, dtype=np.float64)
    with Writer(bag_path, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_odom = w.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=ts)
        c_path = w.add_connection("/planning/reference_path", "nav_msgs/msg/Path", typestore=ts)
        c_obs = w.add_connection(
            "/perception/obstacles", "frenet_planner_msgs/msg/ObstacleArray", typestore=ts
        )

        path_poses = [
            T["geometry_msgs/msg/PoseStamped"](
                header=Header(stamp=stamp(T0_NS), frame_id="map"),
                pose=Pose(position=Point(x=float(p[0]), y=float(p[1]), z=0.0), orientation=quat(0.0)),
            )
            for p in ref
        ]
        path_msg = T["nav_msgs/msg/Path"](header=Header(stamp=stamp(T0_NS), frame_id="map"), poses=path_poses)

        k0 = int(ego.initial_state.time_step)
        for i, st in enumerate(states):
            ns = T0_NS + int(round(i * dt * 1e9))
            if i % 10 == 0:  # 1 Hz reference path (late joiners)
                path_msg.header.stamp = stamp(ns)
                w.write(c_path, ns, ts.serialize_cdr(path_msg, "nav_msgs/msg/Path"))
            yaw = float(st.orientation)
            yaw_next = float(states[min(i + 1, len(states) - 1)].orientation)
            yaw_rate = (yaw_next - yaw) / dt if i + 1 < len(states) else 0.0
            v = float(getattr(st, "velocity", 0.0) or 0.0)
            odom = T["nav_msgs/msg/Odometry"](
                header=Header(stamp=stamp(ns), frame_id="map"),
                child_frame_id="base_link",
                pose=T["geometry_msgs/msg/PoseWithCovariance"](
                    pose=Pose(
                        position=Point(x=float(st.position[0]), y=float(st.position[1]), z=0.0),
                        orientation=quat(yaw),
                    ),
                    covariance=cov36,
                ),
                twist=T["geometry_msgs/msg/TwistWithCovariance"](
                    twist=T["geometry_msgs/msg/Twist"](
                        linear=T["geometry_msgs/msg/Vector3"](x=v, y=0.0, z=0.0),
                        angular=T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=yaw_rate),
                    ),
                    covariance=cov36,
                ),
            )
            w.write(c_odom, ns, ts.serialize_cdr(odom, "nav_msgs/msg/Odometry"))

            obs = adapter.obstacles_at(others, k0 + i, dt, horizon, prediction)
            arr = T["frenet_planner_msgs/msg/ObstacleArray"](
                header=Header(stamp=stamp(ns), frame_id="map"),
                obstacles=[
                    T["frenet_planner_msgs/msg/Obstacle"](
                        id=int(o.id),
                        length=float(o.length),
                        width=float(o.width),
                        is_static=bool(o.is_static),
                        prediction=[
                            T["frenet_planner_msgs/msg/ObstacleState"](
                                t=float(p.t), x=float(p.x), y=float(p.y), yaw=float(p.yaw), v=float(p.v)
                            )
                            for p in o.prediction
                        ],
                    )
                    for o in obs
                ],
            )
            w.write(c_obs, ns, ts.serialize_cdr(arr, "frenet_planner_msgs/msg/ObstacleArray"))
    meta = {
        "scenario": name,
        "ego_id": int(ego.obstacle_id),
        "steps": len(states),
        "dt": dt,
        "prediction": prediction,
        "obstacles": len(others),
        "bag": str(bag_path),
    }
    (bag_path / "frenet_meta.json").write_text(json.dumps(meta, indent=2))
    return meta


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", nargs="+", required=True)
    ap.add_argument("--out", default="data/cache/bags")
    ap.add_argument(
        "--prediction", choices=["constant_velocity", "ground_truth"], default="constant_velocity"
    )
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args(argv)
    files: list[str] = []
    for item in args.scenarios:
        p = Path(item)
        files += sorted(str(f) for f in p.rglob("*.xml")) if p.is_dir() else [str(p)]
    made = 0
    for f in files:
        if args.limit and made >= args.limit:
            break
        meta = convert(f, args.out, args.prediction)
        if meta:
            made += 1
            print(f"{meta['scenario']}: ego {meta['ego_id']}, {meta['steps']} steps -> {meta['bag']}")
    print(f"{made} bags written to {args.out}")


if __name__ == "__main__":
    main()
