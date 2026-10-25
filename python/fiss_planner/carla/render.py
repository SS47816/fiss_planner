"""Bird's-eye GIF rendering of CARLA closed-loop runs (from planner data, no camera needed).

Uses the same visual language as the CommonRoad gallery: asphalt lanes, a teal
candidate fan (brighter = cheaper), the executed plan in amber, the ego in
blue and other traffic in red. The view is heading-up and follows the ego.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

BG = "#14171c"
ROAD = "#2f3440"
MARK = "#7c8494"
EGO = "#3fa9ff"
OBST = "#ff5c5c"
PLAN = "#ffd54a"


def render_gif(frames, road, vehicle, path: Path, view: float = 50.0, fps: int = 5) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.patches as patches
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from PIL import Image

    import fiss_planner as fp

    cmap = LinearSegmentedColormap.from_list("cost", ["#4fd1c5", "#3b6f8f", "#4a5260"])
    lanes = [road.lane(i) for i in road.lane_ids()]
    images = []
    fig, ax = plt.subplots(figsize=(4.4, 6.2), dpi=90)
    fig.patch.set_facecolor(BG)
    for ego, boxes, traj, cands in frames:
        c, s = np.cos(ego.yaw), np.sin(ego.yaw)

        def tf(pts, ego=ego, c=c, s=s):
            # Ego frame, heading up: x = -lateral, y = forward.
            pts = np.asarray(pts, dtype=float) - [ego.x, ego.y]
            fwd = pts[:, 0] * c + pts[:, 1] * s
            lat = -pts[:, 0] * s + pts[:, 1] * c
            return np.column_stack([-lat, fwd])

        ax.clear()
        ax.set_facecolor(BG)
        for ln in lanes:
            if ln is None or np.min(np.linalg.norm(ln.centerline - [ego.x, ego.y], axis=1)) > view * 1.5:
                continue
            ax.add_patch(
                patches.Polygon(tf(np.vstack([ln.left, ln.right[::-1]])), closed=True, fc=ROAD, lw=0)
            )
            for b in (ln.left, ln.right):
                q = tf(b)
                ax.plot(q[:, 0], q[:, 1], color=MARK, lw=0.5, ls=(0, (4, 4)))
        n = len(cands)
        for k, cand in enumerate(cands):  # candidates arrive sorted by cost
            q = tf(cand)
            ax.plot(q[:, 0], q[:, 1], color=cmap(k / max(1, n - 1)), lw=0.7, alpha=0.6)
        if traj is not None and len(traj):
            q = tf(traj)
            ax.plot(q[:, 0], q[:, 1], color=PLAN, lw=2.4)
        for b in boxes:
            if b is not None:
                ax.add_patch(patches.Polygon(tf(b.corners()), closed=True, fc=OBST, ec="w", lw=0.4))
        ego_box = fp.Box2d(ego.x, ego.y, ego.yaw, vehicle.length, vehicle.width)
        ax.add_patch(patches.Polygon(tf(ego_box.corners()), closed=True, fc=EGO, ec="w", lw=0.8))
        ax.set_xlim(-view * 0.35, view * 0.35)
        ax.set_ylim(-view * 0.2, view)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.set_title(f"{ego.v * 3.6:3.0f} km/h", color="w", fontsize=10, loc="left", family="monospace")
        fig.subplots_adjust(left=0.02, right=0.98, top=0.94, bottom=0.01)
        fig.canvas.draw()
        images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
    plt.close(fig)
    path.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        path, save_all=True, append_images=images[1:], duration=int(1000 / fps), loop=0, optimize=True
    )
