"""Download the pinned public CommonRoad scenarios used by the benchmark.

Source: https://gitlab.lrz.de/tum-cps/commonroad-scenarios (commit 2416dc2, 2020a format).
The 280-scenario benchmark set is listed in bench_set.txt (all NGSIM recordings
plus 150 hand-crafted scenarios drawn with seed 0); the development and check
splits used for tuning are in dev_set.txt and check_set.txt. The benchmark reads
the listings directly (--scenarios benchmarks/commonroad/dev_set.txt).

    python benchmarks/commonroad/fetch_scenarios.py --out data/cache/scenarios
    python benchmarks/commonroad/fetch_scenarios.py --out data/cache/scenarios \
        --bench-set data/cache/bench_set
"""

from __future__ import annotations

import argparse
import io
import shutil
import tarfile
import urllib.request
from pathlib import Path

COMMIT = "2416dc27faf3ee36b6799f42712da72a34a2c8fd"
URL = (
    "https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/archive/{c}/"
    "commonroad-scenarios-{c}.tar.gz?path=scenarios/recorded/{sub}"
)
SUBSETS = ["NGSIM", "hand-crafted"]


def fetch(subset: str, out: Path) -> int:
    with urllib.request.urlopen(URL.format(c=COMMIT, sub=subset), timeout=600) as r:
        data = r.read()
    n = 0
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
        for m in tar.getmembers():
            if not m.isfile() or not m.name.endswith(".xml"):
                continue
            rel = Path(*Path(m.name).parts[3:])  # strip <archive>/scenarios/recorded
            dest = out / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            with tar.extractfile(m) as src, open(dest, "wb") as dst:
                shutil.copyfileobj(src, dst)
            n += 1
    return n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="data/cache/scenarios")
    ap.add_argument("--subset", nargs="+", default=SUBSETS, choices=SUBSETS)
    ap.add_argument("--bench-set", help="also copy the 280 benchmark scenarios into this flat directory")
    args = ap.parse_args()
    out = Path(args.out)
    for sub in args.subset:
        print(f"{sub}: {fetch(sub, out)} scenarios")
    if args.bench_set:
        dst = Path(args.bench_set)
        dst.mkdir(parents=True, exist_ok=True)
        listing = Path(__file__).with_name("bench_set.txt").read_text().split()
        for rel in listing:
            shutil.copy(out / rel, dst / Path(rel).name)
        print(f"bench set: {len(listing)} scenarios in {dst}")


if __name__ == "__main__":
    main()
