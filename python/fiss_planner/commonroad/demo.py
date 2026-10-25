"""Render closed-loop CommonRoad runs as animated GIFs.

    # single planner
    python -m fiss_planner.commonroad.demo data/cache/bench_set/USA_US101-22_3_T-1.xml \\
        --planner fiss_plus --gif docs/media/us101.gif
    # side-by-side comparison
    python -m fiss_planner.commonroad.demo data/cache/bench_set/ZAM_Zip-1_50_T-1.xml \\
        --panels fop fiss fiss_plus --gif docs/media/compare.gif
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .benchmark import Options, run_scenario

BG = "#14171c"
ROAD = "#2f3440"
MARK = "#7c8494"
EGO = "#3fa9ff"
OBST = "#ff5c5c"
PLAN = "#ffd54a"
GOAL = "#7ee787"
PLANNER_NAMES = {"fop": "FOP", "fop_plus": "FOP+", "fiss": "FISS", "fiss_plus": "FISS+"}


def _scene(path: str):
    from commonroad.common.file_reader import CommonRoadFileReader

    scenario, problems = CommonRoadFileReader(path).open()
    problem = list(problems.planning_problem_dict.values())[0]
    goal_shapes = []
    st = problem.goal.state_list[0]
    if st.has_value("position"):
        shape = st.position
        goal_shapes = [sh.shapely_object for sh in getattr(shape, "shapes", [shape])]
    return scenario, goal_shapes


def draw_frame(ax, scenario, goal_shapes, rec, step, title: str, view: float, aspect: float = 0.56) -> None:
    """Draw one planning cycle of a recorded run on `ax`."""
    import matplotlib.patches as patches
    from matplotlib.colors import LinearSegmentedColormap

    import fiss_planner as fp

    ex, ey, eyaw, ev = step["ego"]
    L, W = rec["vehicle"]
    ax.clear()
    ax.set_facecolor(BG)
    for ll in scenario.lanelet_network.lanelets:
        if np.min(np.linalg.norm(ll.center_vertices - [ex, ey], axis=1)) > view * 1.6:
            continue
        ax.fill(*np.vstack([ll.left_vertices, ll.right_vertices[::-1]]).T, color=ROAD, lw=0, zorder=1)
        ax.plot(*ll.left_vertices.T, color=MARK, lw=0.5, ls=(0, (4, 4)), zorder=2)
        ax.plot(*ll.right_vertices.T, color=MARK, lw=0.5, ls=(0, (4, 4)), zorder=2)
    for g in goal_shapes:
        ax.fill(*g.exterior.xy, color=GOAL, alpha=0.18, lw=0, zorder=2)
    costs = step["cand_costs"]
    cmap = LinearSegmentedColormap.from_list("cost", ["#4fd1c5", "#3b6f8f", "#4a5260"])
    if step["candidates"]:
        lo, hi = float(np.min(costs)), float(np.percentile(costs, 90))
        for c, cost in zip(step["candidates"], costs):
            c = np.asarray(c)
            w = np.clip((cost - lo) / (hi - lo), 0, 1) if hi > lo else 0.0
            ax.plot(c[:, 0], c[:, 1], color=cmap(w), lw=0.7, alpha=0.55, zorder=3)
    t = np.asarray(step["traj"])
    ax.plot(t[:, 0], t[:, 1], color=PLAN, lw=2.4, zorder=5)
    for corners in step["obstacles"]:
        ax.add_patch(patches.Polygon(corners, closed=True, fc=OBST, ec="#ffffff", lw=0.4, zorder=4))
    ego_box = fp.Box2d(ex, ey, eyaw, L, W)
    ax.add_patch(patches.Polygon(ego_box.corners(), closed=True, fc=EGO, ec="#ffffff", lw=0.8, zorder=6))
    # Window `view` meters wide, `aspect` times as tall, centered ahead of the ego.
    cx, cy = ex + 0.25 * view * np.cos(eyaw), ey + 0.25 * view * np.sin(eyaw)
    ax.set_xlim(cx - 0.5 * view, cx + 0.5 * view)
    ax.set_ylim(cy - 0.5 * aspect * view, cy + 0.5 * aspect * view)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_title(title, color="w", fontsize=10, loc="left")
    hud = (
        f"{ev * 3.6:3.0f} km/h · {step.get('generated', len(step['candidates'])):3d} trajectories · "
        f"{step.get('plan_ms', 0.0):4.2f} ms"
    )
    ax.set_title(hud, color="#c9d1d9", fontsize=9, loc="right", family="monospace")


def render(
    path: str,
    planners: list[str],
    gif: str,
    smoother: bool = True,
    view: float = 60.0,
    fps: int = 10,
    every: int = 1,
    width: float = 8.0,
) -> list[dict]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    scenario, goals = _scene(path)
    runs = []
    for p in planners:
        row, rec = run_scenario(
            path, Options(planner=p, smoother=smoother, keep_candidates=True), record=True
        )
        runs.append((p, row, rec))
    n = min(len(rec["steps"]) for _, _, rec in runs)
    # Several planners are stacked as wide rows so each shows its full candidate fan.
    rows = len(runs)
    fig_w, fig_h = (width, 4.6) if rows == 1 else (width * 1.5, 2.1 * rows + 0.2)
    aspect = (fig_h / rows * 0.86) / fig_w  # panel height / width, so the view fills each panel
    fig, axes = plt.subplots(rows, 1, figsize=(fig_w, fig_h), dpi=90, squeeze=False)
    fig.patch.set_facecolor(BG)
    images = []
    name = Path(path).stem
    for i in range(0, n, every):
        for ax, (p, _, rec) in zip(axes[:, 0], runs):
            step = rec["steps"][i]
            title = f"{PLANNER_NAMES.get(p, p)}" if rows > 1 else f"{name}  ·  {PLANNER_NAMES.get(p, p)}"
            draw_frame(ax, scenario, goals, rec, step, title, view, aspect)
        fig.subplots_adjust(left=0.01, right=0.99, top=0.93 if rows == 1 else 0.96, bottom=0.01, hspace=0.14)
        fig.canvas.draw()
        images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
    plt.close(fig)
    out = Path(gif)
    out.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        out, save_all=True, append_images=images[1:], duration=int(1000 / fps), loop=0, optimize=True
    )
    return [row for _, row, _ in runs]


def record_mcap(path: str, planner: str, mcap: str, smoother: bool = True) -> dict:
    """Run one scenario and write every planning step to an MCAP file for Foxglove."""
    from ..viz import SceneRecorder, obstacle_dict

    rec = SceneRecorder(mcap, start_time=0.0)
    state = {"lanes": False}

    def on_step(t, ego, obstacles, out, road):
        if not state["lanes"]:
            rec.lanes(((road.lane(i).left, road.lane(i).right) for i in road.lane_ids() if road.lane(i)), t)
            state["lanes"] = True
        obs = [d for d in (obstacle_dict(o) for o in obstacles) if d is not None]
        rec.frame(t, ego, vehicle, obs, out, planner)

    from . import adapter

    vehicle = adapter.vehicle_params_from_commonroad(_vanagon())
    try:
        row, _ = run_scenario(
            path, Options(planner=planner, smoother=smoother, keep_candidates=True), on_step=on_step
        )
    finally:
        rec.close()
    return row


def _vanagon():
    from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping

    return VehicleParameterMapping[Options.vehicle].value


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario")
    ap.add_argument("--planner", default="fiss_plus")
    ap.add_argument("--panels", nargs="+", help="render several planners side by side")
    ap.add_argument("--no-smoother", action="store_true")
    ap.add_argument("--gif", help="output GIF (gallery style)")
    ap.add_argument("--mcap", help="output MCAP for Foxglove (first planner)")
    ap.add_argument("--view", type=float, default=60.0)
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--fps", type=int, default=10)
    args = ap.parse_args(argv)
    if not args.gif and not args.mcap:
        ap.error("give --gif and/or --mcap")
    if args.mcap:
        row = record_mcap(args.scenario, (args.panels or [args.planner])[0], args.mcap, not args.no_smoother)
        print({k: row[k] for k in ("scenario", "planner", "status", "steps")}, "->", args.mcap)
        if not args.gif:
            return
    rows = render(
        args.scenario,
        args.panels or [args.planner],
        args.gif,
        not args.no_smoother,
        args.view,
        args.fps,
        args.every,
    )
    for row in rows:
        print(
            {k: row[k] for k in ("scenario", "planner", "status", "steps", "plan_ms_mean", "generated_mean")}
        )


if __name__ == "__main__":
    main()
