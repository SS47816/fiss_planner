"""Summarize Bench2Drive runs (the evaluator's results.json): Driving Score,
Success Rate and infractions, as Bench2Drive's tools/merge_route_json.py
computes them (a success completes the route without infractions other than
minimum speed), averaged over the routes evaluated.

    python benchmarks/bench2drive/summarize.py results/b2d/<run> [...] [--routes]
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path


def records(run: str | Path) -> list[dict]:
    data = json.loads((Path(run) / "results.json").read_text())
    return data["_checkpoint"]["records"]


def success(rd: dict) -> bool:
    if rd["status"] not in ("Completed", "Perfect"):
        return False
    return all(not v for k, v in rd["infractions"].items() if k != "min_speed_infractions")


def summary(run: str | Path) -> dict:
    rds = records(run)
    n = max(1, len(rds))
    infractions = Counter()
    for rd in rds:
        for k, v in rd["infractions"].items():
            if v and k != "min_speed_infractions":
                infractions[k] += len(v)
    return {
        "run": str(run),
        "routes": len(rds),
        "driving_score": sum(rd["scores"]["score_composed"] for rd in rds) / n,
        "route_completion": sum(rd["scores"]["score_route"] for rd in rds) / n,
        "infraction_penalty": sum(rd["scores"]["score_penalty"] for rd in rds) / n,
        "success_%": 100.0 * sum(success(rd) for rd in rds) / n,
        "infractions": dict(infractions),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--routes", action="store_true", help="also list every route")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    rows = [summary(r) for r in args.runs]
    if args.json:
        print(json.dumps(rows))
        return
    for row in rows:
        print(
            f"{row['run']}: {row['routes']} routes, DS {row['driving_score']:.1f},"
            f" SR {row['success_%']:.1f} %, "
            f"RC {row['route_completion']:.1f}, penalty {row['infraction_penalty']:.2f}, {row['infractions']}"
        )
    if args.routes:
        for run in args.runs:
            for rd in records(run):
                inf = {k: len(v) for k, v in rd["infractions"].items() if v}
                print(
                    f"  {rd['route_id']:<45} {rd['status']:<28} DS {rd['scores']['score_composed']:6.1f} "
                    f"RC {rd['scores']['score_route']:6.1f} {inf}"
                )


if __name__ == "__main__":
    main()
