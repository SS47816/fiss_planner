# CommonRoad scenarios used by CI

`ci_scenarios.tar.gz` contains the **same 20 scenarios** that the original CI
command selected with `--limit 20 --seed 1` from the pinned hand-crafted subset.
CI still evaluates all four planners and requires 80 result rows without errors.
The archive makes that smoke test independent of GitLab availability, including
on a cold runner. It does not replace the full benchmark, development or check
splits.

Source: [CommonRoad scenarios](https://gitlab.lrz.de/tum-cps/commonroad-scenarios),
commit `2416dc27faf3ee36b6799f42712da72a34a2c8fd`, directory
`scenarios/recorded/hand-crafted` (635 XML files). The files are stored unchanged.
`ci_scenarios.json` records the source, selection and SHA-256 hashes. The retained
upstream BSD license is in [LICENSE.scenarios.txt](LICENSE.scenarios.txt); it was
also available from the public [scenario mirror](https://github.com/LiaoZeguang/commonroad-scenarios/blob/master/LICENSE.txt)
while the original host was unreachable.

Prepare and verify the smoke data without network access:

```bash
python benchmarks/commonroad/prepare_ci_scenarios.py --out data/cache/ci_scenarios
pixi run -e bench python -m fiss_planner.commonroad.benchmark \
  --scenarios data/cache/ci_scenarios --limit 20 --seed 1 --jobs 4 --out results/ci.csv
```

To reproduce the selection from a fresh full download when upstream is available:

```python
from pathlib import Path
import numpy as np

files = sorted(Path("data/cache/scenarios/hand-crafted").rglob("*.xml"))
assert len(files) == 635
selected = sorted(np.random.default_rng(1).permutation([str(p) for p in files])[:20].tolist())
```

Compare the selected basenames and file hashes with `ci_scenarios.json` before
updating the snapshot. The archive has flat XML filenames and fixed tar metadata;
`prepare_ci_scenarios.py` checks the archive and every file before writing output.
Full datasets continue to use `fetch_scenarios.py`, which retries transient
transport and server failures up to three times and preserves the pinned commit.
