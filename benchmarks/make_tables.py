"""Render the result tables of docs/benchmarks.md from the files in benchmarks/results.

python benchmarks/make_tables.py            # prints Markdown
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

RESULTS = Path(__file__).resolve().parent / "results"
PLANNERS = ["fop", "fop_plus", "fiss", "fiss_plus"]


def _rows(name: str) -> list[dict]:
    with open(RESULTS / name) as f:
        return list(csv.DictReader(f))


def _pct(rows, pred) -> str:
    return f"{100.0 * sum(1 for r in rows if pred(r)) / max(1, len(rows)):.1f} %"


def _mean(rows, key) -> float:
    vals = [float(r[key]) for r in rows if r.get(key) not in ("", None) and math.isfinite(float(r[key]))]
    return statistics.fmean(vals) if vals else math.nan


def commonroad(name: str, title: str) -> str:
    by = defaultdict(list)
    for r in _rows(name):
        by[r["planner"]].append(r)
    out = [
        f"**{title}**",
        "",
        "| Planner | Success | Collision | Timeout | Plan time mean / p95 | Generated / validated "
        "| Headway < 1 s | Min TTC < 2 s |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for p in [p for base in PLANNERS for p in (base, base + "+qp") if p in by]:
        rows = by[p]
        outcomes = ("success", "collision", "timeout")
        status = {k: _pct(rows, lambda r, k=k: r["status"] == k) for k in outcomes}
        out.append(
            f"| {p.replace('+qp', ' + QP')} | {status['success']} | {status['collision']} "
            f"| {status['timeout']} "
            f"| {_mean(rows, 'plan_ms_mean'):.2f} / {_mean(rows, 'plan_ms_p95'):.2f} ms "
            f"| {_mean(rows, 'generated_mean'):.0f} / {_mean(rows, 'validated_mean'):.0f} "
            f"| {_mean(rows, 'time_below_1s_headway'):.2f} s "
            f"| {_pct(rows, lambda r: r.get('min_ttc') not in ('', None) and float(r['min_ttc']) < 2.0)} |"
        )
    return "\n".join(out)


def _fmt(v, unit="", digits=2) -> str:
    return "–" if v is None or not math.isfinite(v) else f"{v:.{digits}f}{unit}"


def carla(name: str, title: str, group=("scenario", "planner", "controller")) -> str:
    runs = json.loads((RESULTS / name).read_text())
    out = [
        f"**{title}**",
        "",
        "| Scenario | Planner | Controller | Collisions | Plan success | Emergency | Min TTC | Min headway "
        "| Headway < 1 s | Mean speed | Plan time | Lateral RMS |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in runs:
        out.append(
            f"| {r['scenario']} | {r['planner']} | {r['controller']} | {r['collisions']} "
            f"| {100 * r['plan_success_rate']:.1f} % | {100 * r['emergency_rate']:.1f} % "
            f"| {_fmt(r.get('min_ttc_s'), ' s')} | {_fmt(r.get('min_time_headway_s'), ' s')} "
            f"| {_fmt(r.get('time_below_1s_headway_s'), ' s', 1)} | {r['mean_speed']:.1f} m/s "
            f"| {r['plan_ms_mean']:.2f} ms | {_fmt(100 * r.get('lateral_rms_m', math.nan), ' cm', 1)} |"
        )
    return "\n".join(out)


def ablations(variants: list[tuple[str, str]]) -> str:
    out = [
        "| Variant (fiss_plus + QP) | Success | Collision | Timeout | Headway < 1 s | Min TTC < 2 s |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, label in variants:
        if not (RESULTS / name).exists():
            continue
        rows = [r for r in _rows(name) if r["planner"] == "fiss_plus+qp"]
        out.append(
            f"| {label} | {_pct(rows, lambda r: r['status'] == 'success')} "
            f"| {_pct(rows, lambda r: r['status'] == 'collision')} "
            f"| {_pct(rows, lambda r: r['status'] == 'timeout')} "
            f"| {_mean(rows, 'time_below_1s_headway'):.2f} s "
            f"| {_pct(rows, lambda r: r.get('min_ttc') not in ('', None) and float(r['min_ttc']) < 2.0)} |"
        )
    return "\n".join(out)


def traffic_seeds(name: str) -> str:
    runs = json.loads((RESULTS / name).read_text())
    by = defaultdict(list)
    for r in runs:
        by[r["configuration"]].append(r)

    def mean(rs, k, scale=1.0):
        vals = [r[k] for r in rs if r.get(k) is not None]
        return scale * statistics.fmean(vals) if vals else math.nan

    def median(rs, k):
        vals = [r[k] if r.get(k) is not None else math.inf for r in rs]
        return statistics.median(vals)

    out = [
        "| Configuration | Runs | Collisions | Plan success | Emergency | Fallback | Headway < 1 s "
        "| Median min TTC | Distance | Plan time |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for config, rs in by.items():
        out.append(
            f"| {config} | {len(rs)} | {sum(r['collisions'] for r in rs)} "
            f"| {mean(rs, 'plan_success_rate', 100):.1f} % | {mean(rs, 'emergency_rate', 100):.2f} % "
            f"| {mean(rs, 'fallback_rate', 100):.2f} % | {mean(rs, 'time_below_1s_headway_s'):.2f} s "
            f"| {_fmt(median(rs, 'min_ttc_s'), ' s')} | {mean(rs, 'distance_m'):.0f} m "
            f"| {mean(rs, 'plan_ms_mean'):.2f} ms |"
        )
    return "\n".join(out)


def main() -> None:
    sections = []
    if (RESULTS / "commonroad280.csv").exists():
        sections.append(
            commonroad("commonroad280.csv", "CommonRoad, 280 scenarios, ground-truth predictions")
        )
    sections.append(
        ablations(
            [
                ("commonroad280.csv", "default: adjacent lanes, ground-truth prediction"),
                ("commonroad280_no_keep_distance.csv", "without the safety envelope and proximity cost"),
                ("commonroad280_single_lane.csv", "ego lane only"),
                ("commonroad280_lane_following.csv", "lane-following prediction"),
                ("commonroad280_constant_velocity.csv", "constant-velocity prediction"),
                (
                    "commonroad280_cv_no_keep_distance.csv",
                    "constant-velocity prediction, no envelope / proximity",
                ),
            ]
        )
    )
    for name, title in [
        ("carla_planners.json", "CARLA, all planners"),
        ("carla_controllers.json", "CARLA, controllers"),
    ]:
        if (RESULTS / name).exists():
            sections.append(carla(name, title))
    if (RESULTS / "carla_traffic_seeds.json").exists():
        sections.append(traffic_seeds("carla_traffic_seeds.json"))
    print("\n\n".join(sections))


if __name__ == "__main__":
    main()
