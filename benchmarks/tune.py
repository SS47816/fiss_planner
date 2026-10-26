"""Tune a planner profile with Optuna on a benchmark's development split.

    python benchmarks/tune.py benchmarks/tuning/commonroad.yaml --trials 60 --jobs 16 \\
        --out results/tune/commonroad

The search-space file names the benchmark, the base profile, the tuning split,
an optional check split, settings fixed for every trial, the objective and the
searched settings by their profile keys (see benchmarks/tuning/). The first
trial evaluates the base profile, so the best settings score at least as well
on the tuning split. Test splits are refused: they are run once per release.

Writes to --out: best.yaml (a complete profile with its provenance; copy it to
python/fiss_planner/profiles/ after checking it), trials.csv (settings and
metrics per trial), study.json (summary, including base and best on the check
split) and the profile of every trial under trials/.
"""

from __future__ import annotations

import argparse
import datetime
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import fiss_planner as fp
import numpy as np
import optuna
import yaml
from fiss_planner.commonroad import benchmark as cr

TEST_SPLITS = {"bench_set"}


def _listing(path: str | Path, root: str | Path, limit: int = 0, seed: int = 0) -> list[str]:
    listing = Path(path)
    if listing.stem in TEST_SPLITS or "test" in listing.stem:
        sys.exit(f"refusing to tune on the test split '{listing}'")
    files = [str(Path(root) / rel) for rel in listing.read_text().split()]
    if limit:
        rng = np.random.default_rng(seed)
        files = sorted(rng.permutation(files)[:limit].tolist())
    return files


class CommonRoadEvaluator:
    """Closed-loop runs of one planner over a list of CommonRoad scenarios."""

    def __init__(self, space: dict, jobs: int):
        self.planner = space.get("planner", "fiss_plus")
        self.prediction = space.get("prediction", "ground_truth")
        self.pool = mp.get_context("spawn").Pool(jobs)

    def __call__(self, profile: Path, files: list[str]) -> dict:
        opts = cr.Options(planner=self.planner, profile=str(profile.resolve()), prediction=self.prediction)
        rows = list(self.pool.imap_unordered(cr.run_task, [(f, opts) for f in files], chunksize=1))
        n = len(rows)

        def pct(pred) -> float:
            return 100.0 * sum(1 for r in rows if pred(r)) / n

        jerk = [r["rms_jerk"] for r in rows if r.get("status") != "error"]
        return {
            "success_%": pct(lambda r: bool(r.get("success"))),
            "collision_%": pct(lambda r: bool(r.get("collision"))),
            "off_road_%": pct(lambda r: bool(r.get("off_road"))),
            "timeout_%": pct(lambda r: r.get("status") == "timeout"),
            "errors": sum(1 for r in rows if r.get("status") == "error"),
            "rms_jerk": float(np.mean(jerk)) if jerk else float("nan"),
        }

    def close(self) -> None:
        self.pool.close()
        self.pool.join()


EVALUATORS = {"commonroad": CommonRoadEvaluator}


def _suggest(trial: optuna.Trial, key: str, spec: dict):
    if "choices" in spec:
        return trial.suggest_categorical(key, spec["choices"])
    if spec.get("int"):
        return trial.suggest_int(key, int(spec["low"]), int(spec["high"]), log=spec.get("log", False))
    return trial.suggest_float(key, float(spec["low"]), float(spec["high"]), log=spec.get("log", False))


def _value(tree: dict, key: str):
    node = tree
    for part in key.split("."):
        node = node[part]
    return node


def _in_space(value, spec: dict) -> bool:
    if "choices" in spec:
        return value in spec["choices"]
    return float(spec["low"]) <= float(value) <= float(spec["high"])


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("space", help="search-space YAML (benchmarks/tuning/*.yaml)")
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--jobs", type=int, default=16, help="parallel simulations per trial")
    ap.add_argument("--seed", type=int, default=0, help="sampler seed")
    ap.add_argument("--limit", type=int, default=0, help="use N scenarios of each split (smoke runs)")
    ap.add_argument("--scenario-root", default="data/cache/scenarios", help="base of the split listings")
    ap.add_argument(
        "--storage", help="Optuna storage URL to keep or resume the study (e.g. sqlite:///tune.db)"
    )
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    space = yaml.safe_load(Path(args.space).read_text())
    params: dict = space["params"]
    weight = float(space.get("objective", {}).get("collision_weight", 2.0))
    files = _listing(space["split"], args.scenario_root, args.limit, args.seed)
    check = _listing(space["check"], args.scenario_root, args.limit, args.seed) if space.get("check") else []
    out = Path(args.out)
    (out / "trials").mkdir(parents=True, exist_ok=True)

    base_cfg, _ = fp.profiles.load(space.get("profile"))
    base_cfg, _ = fp.profiles.override(base_cfg, None, space.get("fixed", {}))
    base_tree = yaml.safe_load(fp.profiles.to_yaml(base_cfg))
    evaluate = EVALUATORS[space["benchmark"]](space, args.jobs)

    def score(m: dict) -> float:
        return m["success_%"] - weight * m["collision_%"]

    def objective(trial: optuna.Trial) -> float:
        values = {key: _suggest(trial, key, spec) for key, spec in params.items()}
        try:
            cfg, _ = fp.profiles.override(base_cfg, None, values)
        except ValueError as e:  # e.g. an inconsistent combination of settings
            trial.set_user_attr("invalid", str(e))
            raise optuna.TrialPruned() from e
        profile = out / "trials" / f"{trial.number:03d}.yaml"
        profile.write_text(fp.profiles.to_yaml(cfg))
        t0 = time.time()
        metrics = evaluate(profile, files)
        for k, v in metrics.items():
            trial.set_user_attr(k, v)
        trial.set_user_attr("wall_s", time.time() - t0)
        print(
            f"trial {trial.number}: score {score(metrics):.1f} "
            + " ".join(f"{k}={v:.2f}" for k, v in metrics.items()),
            file=sys.stderr,
            flush=True,
        )
        return score(metrics)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        direction="maximize",
        sampler=optuna.samplers.TPESampler(seed=args.seed),
        storage=args.storage,
        study_name=Path(args.out).name,
        load_if_exists=bool(args.storage),
    )
    base_values = {k: _value(base_tree, k) for k in params}
    outside = [k for k, spec in params.items() if not _in_space(base_values[k], spec)]
    if outside:
        print(f"base values outside the search space, not enqueued: {outside}", file=sys.stderr)
    elif not study.trials:
        study.enqueue_trial(base_values)
    t0 = time.time()
    try:
        study.optimize(objective, n_trials=args.trials)

        complete = [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE]
        best = study.best_trial
        base = next((t for t in complete if t.params == base_values), None)
        best_cfg, _ = fp.profiles.override(base_cfg, None, best.params)
        summary = {
            "space": args.space,
            "split": space["split"],
            "scenarios": len(files),
            "trials": len(study.trials),
            "seed": args.seed,
            "wall_s": time.time() - t0,
            "best": {"trial": best.number, "score": best.value, "params": best.params, **best.user_attrs},
            "base": {"score": base.value, **base.user_attrs} if base else None,
        }
        if check:
            best_path, base_path = out / "best_check.yaml", out / "base_check.yaml"
            best_path.write_text(fp.profiles.to_yaml(best_cfg))
            base_path.write_text(fp.profiles.to_yaml(base_cfg))
            for name, path in (("best", best_path), ("base", base_path)):
                m = evaluate(path, check)
                summary.setdefault("check", {"split": space["check"], "scenarios": len(check)})[name] = {
                    "score": score(m),
                    **m,
                }
    finally:
        evaluate.close()

    trials = study.trials_dataframe(attrs=("number", "value", "state", "params", "user_attrs"))
    trials.to_csv(out / "trials.csv", index=False)
    (out / "study.json").write_text(json.dumps(summary, indent=2))
    m = best.user_attrs
    header = [
        f"# {space['benchmark']} profile tuned with benchmarks/tune.py ({datetime.date.today()}):",
        f"# {len(study.trials)} trials (seed {args.seed}) on {space['split']} ({len(files)} scenarios),"
        f" planner {space.get('planner', 'fiss_plus')}.",
        f"# Tuning split: score {best.value:.1f} (success {m['success_%']:.1f} %,"
        f" collisions {m['collision_%']:.1f} %)" + (f"; base profile {base.value:.1f}." if base else "."),
    ]
    if check:
        c = summary["check"]
        header.append(
            f"# Check split {space['check']}: score {c['best']['score']:.1f}"
            f" (base profile {c['base']['score']:.1f})."
        )
    (out / "best.yaml").write_text("\n".join(header) + "\n" + fp.profiles.to_yaml(best_cfg))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
