"""Export closed-loop runs for the interactive web demo (docs/demo).

    python -m fiss_planner.commonroad.export_web data/cache/bench_set/ZAM_Zip-1_50_T-1.xml ... \\
        --planners fop fiss fiss_plus --out docs/demo/data

Writes one JSON file per scenario plus an ``index.json`` listing them.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .benchmark import Options, run_scenario
from .demo import PLANNER_NAMES, _scene

MAX_CANDIDATES = 50


def _r(a, nd: int = 2):
    return np.round(np.asarray(a, dtype=float), nd).tolist()


def export(path: str, planners: list[str], every: int = 2, margin: float = 60.0) -> dict:
    scenario, goals = _scene(path)
    runs, xs, ys = {}, [], []
    for p in planners:
        row, rec = run_scenario(path, Options(planner=p, smoother=True, keep_candidates=True), record=True)
        frames = []
        for st in rec["steps"][::every]:
            ex, ey, eyaw, ev = st["ego"]
            xs.append(ex)
            ys.append(ey)
            order = np.argsort(st["cand_costs"])[:MAX_CANDIDATES] if st["candidates"] else []
            frames.append(
                {
                    "k": st["time_step"],
                    "ego": _r([ex, ey, eyaw, ev], 3),
                    "plan": _r(np.asarray(st["traj"])[::2]),
                    "obs": [_r(c) for c in st["obstacles"]],
                    "cands": [_r(np.asarray(st["candidates"][i])[::3]) for i in order],
                    "gen": int(st.get("generated", 0)),
                    "ms": round(float(st.get("plan_ms", 0.0)), 3),
                }
            )
        runs[p] = {
            "label": PLANNER_NAMES.get(p, p),
            "status": row["status"],
            "plan_ms_mean": round(row["plan_ms_mean"], 3),
            "generated_mean": round(row["generated_mean"], 1),
            "vehicle": list(rec["vehicle"]),
            "frames": frames,
        }
    x0, x1 = min(xs) - margin, max(xs) + margin
    y0, y1 = min(ys) - margin, max(ys) + margin
    lanes = []
    for ll in scenario.lanelet_network.lanelets:
        c = ll.center_vertices
        if c[:, 0].max() < x0 or c[:, 0].min() > x1 or c[:, 1].max() < y0 or c[:, 1].min() > y1:
            continue
        lanes.append({"l": _r(ll.left_vertices), "r": _r(ll.right_vertices)})
    return {
        "scenario": Path(path).stem,
        "dt": float(scenario.dt) * every,
        "lanes": lanes,
        "goal": [_r(np.asarray(g.exterior.coords)) for g in goals],
        "runs": runs,
    }


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="+")
    ap.add_argument("--planners", nargs="+", default=["fop", "fiss", "fiss_plus"])
    ap.add_argument("--every", type=int, default=2)
    ap.add_argument("--out", default="docs/demo/data")
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    index = []
    for s in args.scenarios:
        data = export(s, args.planners, args.every)
        name = data["scenario"]
        (out / f"{name}.json").write_text(json.dumps(data, separators=(",", ":")))
        index.append({"id": name, "planners": list(data["runs"])})
        size = (out / f"{name}.json").stat().st_size / 1e6
        print(f"{name}: {size:.2f} MB, " + ", ".join(f"{p}={r['status']}" for p, r in data["runs"].items()))
    (out / "index.json").write_text(json.dumps(index, indent=1))


if __name__ == "__main__":
    main()
