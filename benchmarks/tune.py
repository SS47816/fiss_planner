"""Tune a planner profile with Optuna on a benchmark's development split.

    python benchmarks/tune.py benchmarks/tuning/commonroad.yaml --trials 60 --jobs 16 \\
        --out results/tune/commonroad

The search-space file names the benchmark (commonroad: scenario listings;
nuplan: scenario filters, run in the pixi nuplan environment), the base profile, the tuning split,
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
import os
import subprocess
import sys
import time
from pathlib import Path

import fiss_planner as fp
import numpy as np
import optuna
import yaml
from fiss_planner.commonroad import benchmark as cr


class CommonRoadEvaluator:
    """Closed-loop runs of one planner over a listing of CommonRoad scenarios.
    Score: success % minus `objective.collision_weight` times collision %."""

    test_splits = {"bench_set"}

    def __init__(self, space: dict, args: argparse.Namespace):
        self.planner = space.get("planner", "fiss_plus")
        self.prediction = space.get("prediction", "ground_truth")
        self.weight = float(space.get("objective", {}).get("collision_weight", 2.0))
        self.root, self.limit, self.seed = args.scenario_root, args.limit, args.seed
        self.pool = mp.get_context("spawn").Pool(args.jobs)

    def split(self, name: str) -> list[str]:
        listing = Path(name)
        if listing.stem in self.test_splits or "test" in listing.stem:
            sys.exit(f"refusing to tune on the test split '{listing}'")
        files = [str(Path(self.root) / rel) for rel in listing.read_text().split()]
        if self.limit:
            rng = np.random.default_rng(self.seed)
            files = sorted(rng.permutation(files)[: self.limit].tolist())
        return files

    def __call__(self, profile: Path, files: list[str]) -> dict:
        opts = cr.Options(planner=self.planner, profile=str(profile.resolve()), prediction=self.prediction)
        rows = list(self.pool.imap_unordered(cr.run_task, [(f, opts) for f in files], chunksize=1))
        n = len(rows)

        def pct(pred) -> float:
            return 100.0 * sum(1 for r in rows if pred(r)) / n

        jerk = [r["rms_jerk"] for r in rows if r.get("status") != "error"]
        return {
            "scenarios": n,
            "success_%": pct(lambda r: bool(r.get("success"))),
            "collision_%": pct(lambda r: bool(r.get("collision"))),
            "off_road_%": pct(lambda r: bool(r.get("off_road"))),
            "timeout_%": pct(lambda r: r.get("status") == "timeout"),
            "errors": sum(1 for r in rows if r.get("status") == "error"),
            "rms_jerk": float(np.mean(jerk)) if jerk else float("nan"),
        }

    def score(self, m: dict) -> float:
        return m["success_%"] - self.weight * m["collision_%"]

    def describe(self, m: dict) -> str:
        return f"success {m['success_%']:.1f} %, collisions {m['collision_%']:.1f} %"

    def close(self) -> None:
        self.pool.close()
        self.pool.join()


class NuplanEvaluator:
    """nuPlan closed-loop simulations (benchmarks/nuplan/simulate.sh in the pixi
    nuplan environment) of the planner on a scenario filter. Score: the
    challenge score (CLS-NR or CLS-R, 0-100)."""

    test_splits = {"val14_split", "reduced_val14_split", "test14_hard", "test14_random"}

    def __init__(self, space: dict, args: argparse.Namespace):
        self.challenge = space.get("challenge", "closed_loop_nonreactive_agents")
        self.jobs = args.jobs
        self.limit = args.limit
        self.root = Path(os.environ.get("NUPLAN_ROOT", "data/cache/nuplan"))
        self.pixi = os.environ.get("PIXI_EXE", "pixi")
        self.runs = 0

    def split(self, name: str) -> str:
        if name in self.test_splits or "test" in name or "val14" in name:
            sys.exit(f"refusing to tune on the test split '{name}'")
        return name

    def __call__(self, profile: Path, split: str) -> dict:
        self.runs += 1
        experiment = f"tune_{profile.parent.parent.name}_{profile.stem}_{self.runs}"
        cmd = [
            self.pixi,
            "run",
            "--frozen",
            "-e",
            "nuplan",
            "bash",
            "benchmarks/nuplan/simulate.sh",
            "fiss_planner",
            self.challenge,
            split,
            "worker=ray_distributed",
            f"worker.threads_per_node={self.jobs}",
            f"experiment_name={experiment}",
            f"planner.fiss_planner.profile={profile.resolve()}",
        ]
        if self.limit:
            cmd.append(f"scenario_filter.limit_total_scenarios={self.limit}")
        for _ in range(2):  # the simulator occasionally fails to start
            done = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
            if done:
                break
        if not done:
            raise RuntimeError(f"nuPlan simulation failed: {' '.join(cmd)}")
        run = str(sorted((self.root / "exp" / "exp" / experiment / self.challenge).iterdir())[-1])
        res = subprocess.run(
            [
                self.pixi,
                "run",
                "--frozen",
                "-e",
                "nuplan",
                "python",
                "benchmarks/nuplan/summarize.py",
                "--json",
                run,
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        m = json.loads(res.stdout.strip().splitlines()[-1])[0]
        m.pop("run", None)
        return m

    def score(self, m: dict) -> float:
        return m["score"]

    def describe(self, m: dict) -> str:
        return f"{self.challenge} score {m['score']:.1f}"

    def close(self) -> None:
        pass


EVALUATORS = {"commonroad": CommonRoadEvaluator, "nuplan": NuplanEvaluator}


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
    evaluate = EVALUATORS[space["benchmark"]](space, args)
    files = evaluate.split(space["split"])
    check = evaluate.split(space["check"]) if space.get("check") else None
    out = Path(args.out)
    (out / "trials").mkdir(parents=True, exist_ok=True)

    base_cfg, _ = fp.profiles.load(space.get("profile"))
    base_cfg, _ = fp.profiles.override(base_cfg, None, space.get("fixed", {}))
    base_tree = yaml.safe_load(fp.profiles.to_yaml(base_cfg))
    score = evaluate.score

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
        try:
            metrics = evaluate(profile, files)
        except RuntimeError as e:
            trial.set_user_attr("failed", str(e))
            raise optuna.TrialPruned() from e
        for k, v in metrics.items():
            trial.set_user_attr(k, v)
        trial.set_user_attr("wall_s", time.time() - t0)
        print(
            f"trial {trial.number}: score {score(metrics):.1f} "
            + " ".join(f"{k}={v:.2f}" for k, v in metrics.items() if isinstance(v, (int, float))),
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
            "scenarios": best.user_attrs.get("scenarios"),
            "trials": len(study.trials),
            "seed": args.seed,
            "wall_s": time.time() - t0,
            "best": {"trial": best.number, "score": best.value, "params": best.params, **best.user_attrs},
            "base": {"score": base.value, **base.user_attrs} if base else None,
        }
        if check is not None:
            best_path, base_path = out / "best_check.yaml", out / "base_check.yaml"
            best_path.write_text(fp.profiles.to_yaml(best_cfg))
            base_path.write_text(fp.profiles.to_yaml(base_cfg))
            for name, path in (("best", best_path), ("base", base_path)):
                m = evaluate(path, check)
                summary.setdefault("check", {"split": space["check"]})[name] = {
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
        f"# {len(study.trials)} trials (seed {args.seed}) on {space['split']}"
        f" ({m.get('scenarios')} scenarios),"
        f" planner {space.get('planner', 'fiss_plus')}.",
        f"# Tuning split: score {best.value:.1f} ({evaluate.describe(m)})"
        + (f"; base profile {base.value:.1f}." if base else "."),
    ]
    if check is not None:
        c = summary["check"]
        header.append(
            f"# Check split {space['check']}: score {c['best']['score']:.1f}"
            f" (base profile {c['base']['score']:.1f})."
        )
    (out / "best.yaml").write_text("\n".join(header) + "\n" + fp.profiles.to_yaml(best_cfg))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
