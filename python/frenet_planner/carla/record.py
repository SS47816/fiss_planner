"""Record CARLA closed-loop runs as chase-camera videos with a planner overlay.

The planned trajectory and the candidates of every planning cycle are projected
into a third-person camera image; a heading-up minimap and a HUD (planner,
speed, planning time, trajectories, manoeuvre) are drawn on top.

    $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen &
    python -m frenet_planner.carla.record --scenario cut_in --planner fiss_plus \\
        --mp4 results/carla_cut_in.mp4 --gif docs/media/carla_cut_in.gif
"""

from __future__ import annotations

import argparse
import math
import queue
from pathlib import Path

import numpy as np

import frenet_planner as fp

from . import adapter, closed_loop

PLANNER_NAMES = {"fop": "FOP", "fop_plus": "FOP+", "fiss": "FISS", "fiss_plus": "FISS+"}
AMBER = (242, 179, 61)
TEAL = (79, 209, 197)
RED = (255, 96, 96)
BLUE = (78, 163, 255)
INK = (230, 234, 238)
PANEL = (18, 22, 26)


class CameraRecorder:
    render = True  # keep CARLA rendering on

    def __init__(
        self, planner: str, width: int = 1280, height: int = 720, fov: float = 90.0, minimap: bool = True
    ):
        self.planner = PLANNER_NAMES.get(planner, planner)
        self.w, self.h, self.fov = width, height, fov
        self.minimap = minimap
        self.frames: list[np.ndarray] = []
        self.images: queue.Queue = queue.Queue()
        self.camera = None
        self.last_raw = None  # latest camera image without overlay
        self.out = None
        self.road = None
        self.vehicle = None
        f = width / (2.0 * math.tan(math.radians(fov) / 2.0))
        self.K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]])

    # --- hooks -------------------------------------------------------------
    def on_start(self, world, ego, vehicle, road) -> None:
        import carla

        self.vehicle, self.road = vehicle, road
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self.w))
        bp.set_attribute("image_size_y", str(self.h))
        bp.set_attribute("fov", str(self.fov))
        tf = carla.Transform(carla.Location(x=-7.5, z=3.0), carla.Rotation(pitch=-10.0))
        self.camera = world.spawn_actor(bp, tf, attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
        self.camera.listen(self.images.put)
        self.ego = ego

    def on_tick(self, t, ego_state, out, world, obstacles=None) -> None:
        if out is not None:
            self.out = out
        try:
            image = self.images.get(timeout=5.0)
        except queue.Empty:
            return
        frame = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(self.h, self.w, 4)[:, :, 2::-1].copy()
        self.last_raw = frame
        self.frames.append(self._overlay(frame, ego_state))

    def on_end(self) -> None:
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None

    # --- drawing -----------------------------------------------------------
    def _project(self, pts_xy: np.ndarray, z: float) -> np.ndarray:
        """Planner-frame points (x, y) at height z -> pixel coordinates (NaN when behind)."""
        world_to_cam = np.array(self.camera.get_transform().get_inverse_matrix())
        n = len(pts_xy)
        P = np.ones((4, n))
        P[0], P[1], P[2] = pts_xy[:, 0], -pts_xy[:, 1], z  # mirror back into CARLA's frame
        C = world_to_cam @ P
        cam = np.vstack([C[1], -C[2], C[0]])  # UE (x fwd, y right, z up) -> (right, down, fwd)
        uv = self.K @ cam
        with np.errstate(divide="ignore", invalid="ignore"):
            px = (uv[:2] / uv[2]).T
        px[cam[2] < 0.5] = np.nan
        return px

    def _overlay(self, frame: np.ndarray, ego) -> np.ndarray:
        from PIL import Image, ImageDraw, ImageFont

        img = Image.fromarray(frame).convert("RGBA")
        layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(layer)
        z = self.ego.get_transform().location.z + 0.08
        out = self.out

        def ahead(xy):  # drop points under the ego's own body
            return xy[np.hypot(xy[:, 0] - ego.x, xy[:, 1] - ego.y) > 3.5]

        if out is not None and self.camera is not None:
            for c in list(out.candidates)[:80]:
                px = self._project(ahead(np.column_stack([c.x, c.y])[::2]), z)
                pts = [tuple(p) for p in px if np.isfinite(p).all()]
                if len(pts) > 1:
                    d.line(pts, fill=TEAL + (90,), width=2)
            if len(out.trajectory) > 1:
                px = self._project(ahead(np.column_stack([out.trajectory.x, out.trajectory.y])), z)
                pts = [tuple(p) for p in px if np.isfinite(p).all()]
                if len(pts) > 1:
                    d.line(pts, fill=AMBER + (235,), width=7, joint="curve")
        img = Image.alpha_composite(img, layer)
        d = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype("DejaVuSansMono.ttf", 22)
            big = ImageFont.truetype("DejaVuSans-Bold.ttf", 30)
        except OSError:
            font = big = ImageFont.load_default()
        mode = "—"
        if out is not None:
            if out.emergency:
                mode = "EMERGENCY STOP"
            elif out.trajectory.lane > 0:
                mode = "lane change left"
            elif out.trajectory.lane < 0:
                mode = "lane change right"
            else:
                mode = "keep lane"
        d.rounded_rectangle((20, 20, 520, 166), radius=10, fill=PANEL + (200,))
        d.text((36, 30), self.planner, font=big, fill=AMBER)
        lines = [f"{ego.v * 3.6:5.1f} km/h   {mode}"]
        if out is not None:
            lines.append(f"{out.stats.num_generated:4d} trajectories   {out.stats.time_ms:5.2f} ms")
        for i, line in enumerate(lines):
            d.text((36, 76 + 32 * i), line, font=font, fill=INK)
        if self.minimap:
            img.paste(self._minimap(ego, out), (self.w - 300, 20))
        return np.asarray(img.convert("RGB"))

    def _minimap(self, ego, out, size: int = 280, span: float = 70.0):
        """Heading-up top view: lanes, traffic, candidates and plan."""
        from PIL import Image, ImageDraw

        img = Image.new("RGBA", (size, size), PANEL + (215,))
        d = ImageDraw.Draw(img)
        s = size / span
        c, sn = math.cos(ego.yaw), math.sin(ego.yaw)

        def tf(xy):
            xy = np.asarray(xy, dtype=float) - [ego.x, ego.y]
            fwd = xy[:, 0] * c + xy[:, 1] * sn
            lat = -xy[:, 0] * sn + xy[:, 1] * c
            return [(size / 2 - la * s, size * 0.75 - fw * s) for fw, la in zip(fwd, lat)]

        for lid in self.road.lane_ids():
            ln = self.road.lane(lid)
            if ln is None or np.min(np.linalg.norm(ln.centerline - [ego.x, ego.y], axis=1)) > span:
                continue
            d.polygon(tf(np.vstack([ln.left, ln.right[::-1]])), fill=(58, 66, 76, 255))
            d.line(tf(ln.left), fill=(120, 130, 142, 255), width=1)
            d.line(tf(ln.right), fill=(120, 130, 142, 255), width=1)
        if out is not None:
            for cand in list(out.candidates)[:80]:
                d.line(tf(np.column_stack([cand.x, cand.y])), fill=TEAL + (110,), width=1)
            if len(out.trajectory) > 1:
                d.line(
                    tf(np.column_stack([out.trajectory.x, out.trajectory.y])), fill=AMBER + (255,), width=3
                )
        world = self.ego.get_world()
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.id == self.ego.id:
                continue
            tr = actor.get_transform()
            x, y, yaw = adapter.to_planner(tr.location.x, tr.location.y, tr.rotation.yaw)
            if math.hypot(x - ego.x, y - ego.y) > span:
                continue
            ext = actor.bounding_box.extent
            box = fp.Box2d(x, y, yaw, 2 * ext.x, 2 * ext.y)
            d.polygon(tf(box.corners()), fill=RED + (255,))
        eb = fp.Box2d(ego.x, ego.y, ego.yaw, self.vehicle.length, self.vehicle.width)
        d.polygon(tf(eb.corners()), fill=BLUE + (255,))
        return img

    # --- output ------------------------------------------------------------
    def save(
        self,
        mp4: str | None,
        gif: str | None,
        gif_width: int = 560,
        gif_every: int = 2,
        gif_seconds: float | None = None,
    ) -> None:
        if mp4:
            import imageio.v2 as imageio

            Path(mp4).parent.mkdir(parents=True, exist_ok=True)
            imageio.mimsave(mp4, self.frames, fps=20, quality=6, macro_block_size=8)
        if gif:
            from PIL import Image

            frames = self.frames[::gif_every]
            if gif_seconds:
                frames = frames[: int(gif_seconds * 20 / gif_every)]
            h = int(self.h * gif_width / self.w)
            ims = [
                Image.fromarray(f).resize((gif_width, h), Image.LANCZOS).quantize(colors=128) for f in frames
            ]
            Path(gif).parent.mkdir(parents=True, exist_ok=True)
            ims[0].save(
                gif,
                save_all=True,
                append_images=ims[1:],
                duration=int(1000 * gif_every / 20),
                loop=0,
                optimize=True,
            )


class FoxgloveHook:
    """Runner hook writing each planning cycle to MCAP and/or a live Foxglove server
    (see `frenet_planner.viz.SceneRecorder`); adds JPEG frames of a `CameraRecorder`."""

    def __init__(
        self, planner: str, mcap: str | None = None, live: bool = False, camera=None, image_width=960
    ):
        from ..viz import SceneRecorder

        self.recorder = SceneRecorder(mcap, live=live)
        self.planner = PLANNER_NAMES.get(planner, planner)
        self.camera = camera
        self.image_width = image_width
        self.vehicle = None
        self._last_out = None

    def on_start(self, world, ego, vehicle, road) -> None:
        self.vehicle = vehicle
        self.recorder.lanes((road.lane(i).left, road.lane(i).right) for i in road.lane_ids() if road.lane(i))

    def on_tick(self, t, ego_state, out, world, obstacles=None) -> None:
        if out is None or out is self._last_out:
            return  # one frame per planning cycle
        self._last_out = out
        from ..viz import obstacle_dict

        jpeg = None
        if self.camera is not None and self.camera.last_raw is not None:
            import io

            from PIL import Image

            img = Image.fromarray(self.camera.last_raw)
            img = img.resize((self.image_width, int(img.height * self.image_width / img.width)))
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=80)
            jpeg = buf.getvalue()
        obs = [d for d in (obstacle_dict(o) for o in (obstacles or [])) if d is not None]
        self.recorder.frame(t, ego_state, self.vehicle, obs, out, self.planner, jpeg)

    def on_end(self) -> None:
        self.recorder.close()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="cut_in", choices=closed_loop.SCENARIOS)
    ap.add_argument("--planner", default="fiss_plus")
    ap.add_argument("--controller", default="mpc")
    ap.add_argument("--town", default="Town04")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--smoother", action="store_true")
    ap.add_argument("--mp4")
    ap.add_argument("--gif")
    ap.add_argument("--gif-start", type=float, default=0.0, help="seconds skipped at the start of the GIF")
    ap.add_argument("--gif-seconds", type=float, default=12.0)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--mcap", help="also write the run (scene, plans, stats, camera) as MCAP for Foxglove")
    ap.add_argument("--foxglove-live", action="store_true", help="stream to Foxglove on ws://localhost:8765")
    args = ap.parse_args(argv)

    rec = CameraRecorder(args.planner)
    hooks = [rec]
    if args.mcap or args.foxglove_live:
        hooks.append(FoxgloveHook(args.planner, args.mcap, args.foxglove_live, camera=rec))
    result = closed_loop.run(
        args.scenario,
        args.planner,
        args.controller,
        args.smoother,
        args.town,
        args.duration,
        port=args.port,
        hooks=hooks,
    )
    if args.mp4:
        rec.save(args.mp4, None)
    if args.gif:
        rec.frames = rec.frames[int(args.gif_start * 20) :]
        rec.save(None, args.gif, gif_seconds=args.gif_seconds)
    print(
        {
            k: v
            for k, v in result.__dict__.items()
            if k in ("scenario", "planner", "collisions", "plan_success_rate")
        },
        f"{len(rec.frames)} frames",
    )


if __name__ == "__main__":
    main()
