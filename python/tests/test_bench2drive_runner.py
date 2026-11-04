"""Exercise the route supervisor without CARLA or the leaderboard dependencies."""

import importlib.util
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[2] / "benchmarks/bench2drive/run_routes.py"
pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="CARLA supervisor uses Linux subreaping")
spec = importlib.util.spec_from_file_location("run_routes", SOURCE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def checkpoint(status="Completed", progress=None, entry="Finished", route_id="7"):
    return {
        "entry_status": entry,
        "_checkpoint": {
            "progress": [1, 1] if progress is None else progress,
            "records": [{"route_id": f"RouteScenario_{route_id}_rep0", "status": status}],
        },
    }


@pytest.mark.parametrize("status", ["Completed", "Perfect", "Failed", "Failed - Agent blocked"])
def test_completed_driving_results_are_terminal(tmp_path, status):
    path = tmp_path / "results.json"
    path.write_text(json.dumps(checkpoint(status)))
    assert runner.completed(path, "7")


@pytest.mark.parametrize(
    "value",
    [
        {},
        {"_checkpoint": None},
        checkpoint(progress=[0, 1]),
        checkpoint(entry="Crashed"),
        checkpoint(status="Started"),
        checkpoint(route_id="8"),
        checkpoint(entry="Invalid"),
    ],
)
def test_incomplete_checkpoints_are_retried(tmp_path, value):
    path = tmp_path / "results.json"
    path.write_text(json.dumps(value))
    assert not runner.completed(path, "7")
    path.write_text("{")
    assert not runner.completed(path, "7")


@pytest.fixture
def fake_runner(tmp_path):
    (tmp_path / "run_routes.py").write_text(SOURCE.read_text())
    (tmp_path / "run.sh").write_text('exec "$FAKE_PYTHON" "$FAKE_SCRIPT" "$@"\n')
    script = tmp_path / "fake.py"
    script.write_text("""
import json, os, signal, subprocess, sys, time
from pathlib import Path
out = Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
root = out.parent
count = root / "calls"
n = int(count.read_text()) + 1 if count.exists() else 1
count.write_text(str(n))
code = "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)"
child = subprocess.Popen([sys.executable, "-c", code], start_new_session=True)
(root / "child.pid").write_text(str(child.pid))
mode = os.environ["FAKE_MODE"]
if mode in ("timeout", "interrupt"):
    time.sleep(60)
if mode == "retry" and n == 1:
    result = {"entry_status": "Started", "_checkpoint": {"progress": [0,1], "records": []}}
else:
    record = {"route_id": "RouteScenario_7_rep0", "status": "Failed - Agent blocked"}
    result = {"entry_status": "Finished", "_checkpoint": {"progress": [1,1], "records": [record]}}
(out / "results.json").write_text(json.dumps(result))
""")
    env = dict(
        os.environ,
        FAKE_PYTHON=sys.executable,
        FAKE_SCRIPT=str(script),
        FAKE_MODE="done",
        ROUTE_TIMEOUT="5",
        PORT="23456",
    )
    out = tmp_path / "run"
    cmd = [sys.executable, str(tmp_path / "run_routes.py"), "validation", str(out), "bench2drive", "7"]
    return cmd, env, out


def assert_gone(pid):
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


@pytest.mark.parametrize(
    "mode,expected_calls,returncode", [("done", 1, 0), ("retry", 2, 0), ("timeout", 2, 1)]
)
def test_supervisor_retries_and_reaps_detached_children(fake_runner, mode, expected_calls, returncode):
    cmd, env, out = fake_runner
    env["FAKE_MODE"] = mode
    if mode == "timeout":
        env["ROUTE_TIMEOUT"] = "0.3"
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        result = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=15)
        assert result.returncode == returncode, result.stderr
        assert (out / "calls").read_text() == str(expected_calls)
        assert_gone(int((out / "child.pid").read_text()))
        assert unrelated.poll() is None
        if mode == "retry":
            assert len(list(out.parent.glob("run.incomplete/7-*/results.json"))) == 1
        if mode == "done":
            # A finished failed drive is skipped on the next invocation.
            assert subprocess.run(cmd, env=env, timeout=5).returncode == 0
            assert (out / "calls").read_text() == "1"
    finally:
        unrelated.terminate()
        unrelated.wait()


def test_interrupt_reaps_the_evaluator_and_detached_server(fake_runner):
    cmd, env, out = fake_runner
    env["FAKE_MODE"] = "interrupt"
    proc = subprocess.Popen(cmd, env=env)
    try:
        deadline = time.monotonic() + 5
        while not (out / "child.pid").exists():
            assert time.monotonic() < deadline
            time.sleep(0.02)
        proc.send_signal(signal.SIGTERM)
        assert proc.wait(timeout=8) == 143
        assert_gone(int((out / "child.pid").read_text()))
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
