"""Record planner runs as MCAP files (or stream them live) for Foxglove.

Every planning cycle becomes one frame on these topics (frame ``map``, planner
coordinates):

- ``/map/lanes``       lane boundaries (logged once)
- ``/ego``             ego box and heading
- ``/obstacles``       obstacle boxes and predicted paths; the vehicle ahead is
                       yellow, the obstacle that blocked the cheapest candidate orange
- ``/plan/trajectory`` the executed plan
- ``/plan/candidates`` explored candidates, cheapest brightest (rejected ones faint)
- ``/plan/stats``      JSON: planner, mode, timings, counts, gap / headway / TTC, rejections
- ``/tf``              ``map -> ego`` (to follow the ego in the 3D panel)
- ``/camera/image``    JPEG frames, when a camera is recorded

Open the file in Foxglove (or the open-source Lichtblick) together with
``docs/foxglove/fiss_planner_layout.json``. Requires ``pip install foxglove-sdk``.
"""

from __future__ import annotations

import math
import time

import numpy as np

EGO = (0.25, 0.66, 1.0)
OBSTACLE = (1.0, 0.36, 0.36)
LEAD = (1.0, 0.84, 0.29)
BLOCKING = (1.0, 0.55, 0.1)
PLAN = (0.95, 0.70, 0.24)
CHEAP = np.array([0.31, 0.82, 0.77])
COSTLY = np.array([0.29, 0.32, 0.38])
LANE = (0.49, 0.52, 0.58)


def obstacle_dict(ob, t: float = 0.0) -> dict | None:
    """Plain description of an `fp.Obstacle` at time t (box and predicted path)."""
    box = ob.box_at(t)
    if box is None:
        return None
    path = [(p.x, p.y) for p in ob.prediction if p.t >= t]
    return {"id": ob.id, "box": (box.cx, box.cy, box.yaw, box.length, box.width), "path": path}


def _finite(x: float) -> float | None:
    return float(x) if x is not None and math.isfinite(x) else None


class SceneRecorder:
    """Writes frames to an MCAP file and/or a live Foxglove WebSocket server."""

    def __init__(
        self,
        mcap_path: str | None = None,
        live: bool = False,
        port: int = 8765,
        max_candidates: int = 60,
        start_time: float | None = None,
    ):
        import foxglove
        import foxglove.messages as m

        self.m = m
        self.foxglove = foxglove
        self.max_candidates = max_candidates
        self.t0 = time.time() if start_time is None else start_time  # wall-clock origin of the log
        self.writer = foxglove.open_mcap(mcap_path, allow_overwrite=True) if mcap_path else None
        self.server = foxglove.start_server(name="fiss_planner", port=port) if live else None
        self.frames = 0

    # --- primitives ----------------------------------------------------------
    def _stamp(self, t: float):
        ns = int(round((self.t0 + t) * 1e9))
        return ns, self.m.Timestamp(ns // 1_000_000_000, ns % 1_000_000_000)

    def _quat(self, yaw: float):
        return self.m.Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))

    def _pose(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0):
        return self.m.Pose(position=self.m.Vector3(x=x, y=y, z=z), orientation=self._quat(yaw))

    def _color(self, rgb, a: float = 1.0):
        return self.m.Color(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=a)

    def _line(self, xy, rgb, thickness: float, a: float = 1.0, z: float = 0.05, loop: bool = False):
        m = self.m
        kind = m.LinePrimitiveLineType.LineLoop if loop else m.LinePrimitiveLineType.LineStrip
        return m.LinePrimitive(
            type=kind,
            pose=self._pose(0.0, 0.0),
            thickness=thickness,
            scale_invariant=False,
            points=[m.Point3(x=float(x), y=float(y), z=z) for x, y in xy],
            color=self._color(rgb, a),
        )

    def _cube(self, box, rgb, height: float = 1.5, a: float = 0.9):
        cx, cy, yaw, length, width = box
        return self.m.CubePrimitive(
            pose=self._pose(cx, cy, height / 2, yaw),
            size=self.m.Vector3(x=length, y=width, z=height),
            color=self._color(rgb, a),
        )

    def _entity(self, stamp, eid: str, **prims):
        return self.m.SceneEntity(timestamp=stamp, frame_id="map", id=eid, **prims)

    def _scene(self, stamp, entities):
        m = self.m
        clear = m.SceneEntityDeletion(timestamp=stamp, type=m.SceneEntityDeletionType.All, id="")
        return m.SceneUpdate(deletions=[clear], entities=entities)

    def _log(self, topic: str, message, ns: int) -> None:
        self.foxglove.log(topic, message, log_time=ns)

    # --- content -------------------------------------------------------------
    def lanes(self, lanes, t: float = 0.0) -> None:
        """Static map: iterable of (left, right) boundary polylines (N x 2 arrays)."""
        ns, stamp = self._stamp(t)
        lines = []
        for left, right in lanes:
            for b in (left, right):
                if b is not None and len(b) > 1:
                    lines.append(self._line(np.asarray(b), LANE, 0.12, a=0.8, z=0.0))
        self._log("/map/lanes", self._scene(stamp, [self._entity(stamp, "lanes", lines=lines)]), ns)

    def frame(
        self,
        t: float,
        ego,
        vehicle,
        obstacles: list[dict],
        out=None,
        planner: str = "",
        image_jpeg: bytes | None = None,
        extra: dict | None = None,
    ) -> None:
        """One planning cycle. `ego` is an `fp.EgoState`, `obstacles` come from
        `obstacle_dict`, `out` is an `fp.LocalPlanOutput` (or None)."""
        m = self.m
        ns, stamp = self._stamp(t)
        ego_box = (ego.x, ego.y, ego.yaw, vehicle.length, vehicle.width)
        arrow = m.ArrowPrimitive(
            pose=self._pose(ego.x, ego.y, 1.6, ego.yaw),
            shaft_length=2.0,
            shaft_diameter=0.15,
            head_length=0.8,
            head_diameter=0.5,
            color=self._color((1.0, 1.0, 1.0)),
        )
        self._log(
            "/ego",
            self._scene(
                stamp, [self._entity(stamp, "ego", cubes=[self._cube(ego_box, EGO)], arrows=[arrow])]
            ),
            ns,
        )
        self._log(
            "/tf",
            m.FrameTransforms(
                transforms=[
                    m.FrameTransform(
                        timestamp=stamp,
                        parent_frame_id="map",
                        child_frame_id="ego",
                        translation=m.Vector3(x=ego.x, y=ego.y, z=0.0),
                        rotation=self._quat(ego.yaw),
                    )
                ]
            ),
            ns,
        )

        lead = out.stats.lead.id if out is not None else -1
        blocking = out.stats.blocking_id if out is not None else -1
        entities = []
        for ob in obstacles:
            color = BLOCKING if ob["id"] == blocking else LEAD if ob["id"] == lead else OBSTACLE
            prims = {"cubes": [self._cube(ob["box"], color)]}
            if len(ob["path"]) > 1:
                prims["lines"] = [self._line(ob["path"], color, 0.12, a=0.5)]
            entities.append(self._entity(stamp, f"obstacle_{ob['id']}", **prims))
        self._log("/obstacles", self._scene(stamp, entities), ns)

        if out is not None:
            plan = out.trajectory
            lines = (
                [self._line(np.column_stack([plan.x, plan.y]), PLAN, 0.45, z=0.15)] if len(plan) > 1 else []
            )
            self._log("/plan/trajectory", self._scene(stamp, [self._entity(stamp, "plan", lines=lines)]), ns)
            cands = sorted(out.candidates, key=lambda c: c.total_cost())[: self.max_candidates]
            cand_lines = []
            for k, c in enumerate(cands):
                if len(c) < 2 or not c.is_cartesian:
                    continue
                w = k / max(1, len(cands) - 1)
                rgb = (1 - w) * CHEAP + w * COSTLY
                alpha = 0.8 if c.rejection.name == "OK" else 0.25
                cand_lines.append(self._line(np.column_stack([c.x, c.y])[::2], rgb, 0.08, a=alpha, z=0.1))
            self._log(
                "/plan/candidates",
                self._scene(stamp, [self._entity(stamp, "candidates", lines=cand_lines)]),
                ns,
            )
            s = out.stats
            mode = (
                "emergency"
                if out.emergency
                else "fallback"
                if out.fallback
                else "degraded"
                if out.degraded
                else "plan"
            )
            stats = {
                "planner": planner,
                "mode": mode,
                "lane_change": int(plan.lane) if len(plan) else 0,
                "plan_ms": s.time_ms,
                "generated": s.num_generated,
                "validated": s.num_validated,
                "speed": ego.v,
                "target_speed": out.target_speed,
                "lead_id": s.lead.id,
                "lead_gap": _finite(s.lead.gap),
                "time_headway": _finite(s.lead.time_headway),
                "ttc": _finite(s.lead.ttc),
                "blocking_id": s.blocking_id,
                "rejections": dict(s.rejections),
            }
            if extra:
                stats.update(extra)
            self._log("/plan/stats", stats, ns)

        if image_jpeg is not None:
            self._log(
                "/camera/image",
                m.CompressedImage(timestamp=stamp, frame_id="camera", data=image_jpeg, format="jpeg"),
                ns,
            )
        self.frames += 1

    def close(self) -> None:
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        if self.server is not None:
            self.server.stop()
            self.server = None
