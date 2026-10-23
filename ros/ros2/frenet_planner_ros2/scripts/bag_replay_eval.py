#!/usr/bin/env python3
"""Replay ROS 2 bags through the frenet_planner node and evaluate its plans.

For every bag and planner type this script starts the planner node (sim
time, planning on every odometry message, no stitching: the replay is open
loop), plays the bag with ``ros2 bag play --clock`` and records the node's
trajectories and statistics. Plans are compared with what the logged
vehicle actually did afterwards (average / final displacement over the
first seconds), together with success rate and latency.

    ros2 run frenet_planner_ros2 bag_replay_eval.py data/cache/bags/* \\
        --planners fop fop_plus fiss fiss_plus --out results/bag_eval.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
from frenet_planner_msgs.msg import PlannerStats, Trajectory
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message

QOS_OVERRIDES = """\
/planning/reference_path:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1
"""


def read_ego(bag: str) -> tuple[np.ndarray, np.ndarray]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/odom"]))
    t, xy = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, Odometry)
        t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        xy.append((m.pose.pose.position.x, m.pose.pose.position.y))
    return np.asarray(t), np.asarray(xy)


class Recorder(Node):
    def __init__(self):
        super().__init__(
            "bag_replay_recorder",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)],
        )
        self.trajectories: list[Trajectory] = []
        self.stats: list[PlannerStats] = []
        self.create_subscription(Trajectory, "/frenet_planner/trajectory", self.trajectories.append, 100)
        self.create_subscription(PlannerStats, "/frenet_planner/stats", self.stats.append, 100)


def displacement(traj: Trajectory, ego_t: np.ndarray, ego_xy: np.ndarray, horizon: float):
    t0 = traj.header.stamp.sec + 1e-9 * traj.header.stamp.nanosec
    errs, final = [], None
    for p in traj.points:
        if p.t <= 0.0 or p.t > horizon + 1e-6 or t0 + p.t > ego_t[-1]:
            continue
        hx = np.interp(t0 + p.t, ego_t, ego_xy[:, 0])
        hy = np.interp(t0 + p.t, ego_t, ego_xy[:, 1])
        errs.append(math.hypot(p.x - hx, p.y - hy))
        if abs(p.t - horizon) < 0.051:
            final = errs[-1]
    return (float(np.mean(errs)) if errs else None), final


def run_one(bag: str, planner: str, rate: float, horizon: float, qos_file: str, recorder: Recorder) -> dict:
    recorder.trajectories.clear()
    recorder.stats.clear()
    node = subprocess.Popen(
        [
            "ros2",
            "run",
            "frenet_planner_ros2",
            "planner_node",
            "--ros-args",
            "-r",
            "__node:=frenet_planner",
            "-r",
            "~/odom:=/odom",
            "-r",
            "~/obstacles:=/perception/obstacles",
            "-r",
            "~/reference_path:=/planning/reference_path",
            "-p",
            "use_sim_time:=true",
            "-p",
            "plan_on_odom:=true",
            "-p",
            f"planner.type:={planner}",
            "-p",
            "stitching.enabled:=false",
            "-p",
            "odom_to_center_offset:=0.0",
            "--log-level",
            "warn",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    time.sleep(1.5)
    play = subprocess.run(
        [
            "ros2",
            "bag",
            "play",
            bag,
            "--clock",
            "100",
            "-r",
            str(rate),
            "--qos-profile-overrides-path",
            qos_file,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(1.0)
    os.killpg(node.pid, signal.SIGINT)
    try:
        node.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(node.pid, signal.SIGKILL)

    ego_t, ego_xy = read_ego(bag)
    ades, fdes = [], []
    for tr in recorder.trajectories:
        if tr.emergency:
            continue
        ade, fde = displacement(tr, ego_t, ego_xy, horizon)
        if ade is not None:
            ades.append(ade)
        if fde is not None:
            fdes.append(fde)
    st = recorder.stats
    cyc = [s.cycle_ms for s in st]
    return {
        "bag": Path(bag).name,
        "planner": planner,
        "play_ok": play.returncode == 0,
        "cycles": len(st),
        "odom_msgs": int(len(ego_t)),
        "success_rate": float(np.mean([s.success for s in st])) if st else 0.0,
        "emergency_rate": float(np.mean([s.emergency for s in st])) if st else 0.0,
        "cycle_ms_mean": float(np.mean(cyc)) if cyc else math.nan,
        "cycle_ms_p95": float(np.percentile(cyc, 95)) if cyc else math.nan,
        "plan_ms_mean": float(np.mean([s.time_ms for s in st])) if st else math.nan,
        "generated_mean": float(np.mean([s.num_generated for s in st])) if st else math.nan,
        f"ade_{horizon:g}s": float(np.mean(ades)) if ades else math.nan,
        f"fde_{horizon:g}s": float(np.mean(fdes)) if fdes else math.nan,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--planners", nargs="+", default=["fop", "fop_plus", "fiss", "fiss_plus"])
    ap.add_argument("--rate", type=float, default=1.0, help="bag playback rate")
    ap.add_argument("--horizon", type=float, default=3.0, help="displacement horizon [s]")
    ap.add_argument("--out", default="results/bag_eval.json")
    args = ap.parse_args()

    rclpy.init()
    recorder = Recorder()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(QOS_OVERRIDES)
        qos_file = f.name
    rows = []
    bags = [b for b in args.bags if Path(b, "metadata.yaml").exists()]
    for bag in bags:
        for planner in args.planners:
            row = run_one(bag, planner, args.rate, args.horizon, qos_file, recorder)
            rows.append(row)
            print(
                json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()}),
                flush=True,
            )

    executor.shutdown()
    recorder.destroy_node()
    rclpy.shutdown()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rows, indent=2))
    keys = [
        "success_rate",
        "emergency_rate",
        "cycle_ms_mean",
        "cycle_ms_p95",
        "plan_ms_mean",
        "generated_mean",
        f"ade_{args.horizon:g}s",
        f"fde_{args.horizon:g}s",
    ]
    lines = ["| planner | bags | cycles | " + " | ".join(keys) + " |", "|" + "---|" * (len(keys) + 3)]
    for planner in args.planners:
        rs = [r for r in rows if r["planner"] == planner]
        vals = [np.nanmean([r[k] for r in rs]) for k in keys]
        lines.append(
            f"| {planner} | {len(rs)} | {sum(r['cycles'] for r in rs)} | "
            + " | ".join(f"{v:.3f}" for v in vals)
            + " |"
        )
    md = "\n".join(lines)
    out.with_suffix(".md").write_text(md + "\n")
    print(md)


if __name__ == "__main__":
    main()
