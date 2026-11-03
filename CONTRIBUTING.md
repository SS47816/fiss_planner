# Contributing

Thanks for helping! Bug reports with a minimal reproduction (a CommonRoad scenario, a bag, or a
few lines of Python) are the most useful thing you can send.

## Development setup

[pixi](https://pixi.sh) **0.82.1** provides every toolchain without root. The
manifest and CI pin this version. After dependency changes, run `pixi lock`, then
`pixi lock --check` and `pixi install --locked` before submitting the lockfile.
The explicit Ray package mapping prevents bundled dependencies from being
incorrectly identified as PyPI packages at Ray's version. See
[Pixi's mapping documentation](https://pixi.prefix.dev/dev/reference/pixi_manifest/#conda-pypi-map-optional).

| Task | Command |
|---|---|
| C++ core + tests | `pixi run test` |
| with ASan/UBSan | `pixi run test-asan` |
| micro-benchmarks | `pixi run bench-micro` |
| Python bindings + tests | `pixi run -e bench py-test` |
| CommonRoad benchmark | `pixi run -e bench cr-bench --scenarios <dir>` |
| ROS 2 build + tests | `pixi run -e jazzy ros2-test` |
| ROS 1 build + tests | `pixi run -e noetic ros1-test` |

## Guidelines

- The core (`core/`) must stay ROS-free and dependency-light (C++17 standard library,
  yaml-cpp, optional OSQP).
- Every bug fix comes with a regression test; every new planner feature with a test in
  `core/test/` and, if user-visible, a benchmark run (`benchmarks/`).
- Format with `pre-commit run --all-files` (clang-format, ruff).
- Commit messages follow [Conventional Commits](https://www.conventionalcommits.org).
- New planners: subclass `Planner` (see `core/src/planners.hpp`), register in `makePlanner`,
  add the type to `PlannerType`, and they are available in Python and both ROS wrappers.
