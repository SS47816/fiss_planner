"""Summarize nuPlan simulation runs: the challenge score and its metrics per run.

python benchmarks/nuplan/summarize.py $NUPLAN_EXP_ROOT/exp/<experiment>/<challenge>/<time> [...]
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path

import pandas as pd

METRICS = {
    "no_ego_at_fault_collisions": "collisions",
    "drivable_area_compliance": "drivable",
    "driving_direction_compliance": "direction",
    "ego_is_making_progress": "making_progress",
    "time_to_collision_within_bound": "ttc",
    "speed_limit_compliance": "speed_limit",
    "ego_progress_along_expert_route": "progress",
    "ego_is_comfortable": "comfort",
}


def load(run: str | Path) -> pd.DataFrame:
    """Per-scenario metrics of one run (rows with a log name)."""
    files = glob.glob(str(Path(run) / "aggregator_metric" / "*.parquet"))
    if not files:
        raise FileNotFoundError(f"no aggregator metrics under {run}")
    df = pd.read_parquet(files[0])
    return df[df["log_name"].notna()]


def summary(run: str | Path) -> dict:
    df = load(run)
    row = {"run": str(Path(run).relative_to(Path(run).parents[2])), "scenarios": len(df)}
    row["score"] = 100.0 * df["score"].mean()
    for col, name in METRICS.items():
        if col in df:
            row[name] = 100.0 * df[col].astype(float).mean()
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="simulation output directories")
    args = ap.parse_args()
    table = pd.DataFrame([summary(r) for r in args.runs])
    print(table.to_markdown(index=False, floatfmt=".1f"))


if __name__ == "__main__":
    main()
