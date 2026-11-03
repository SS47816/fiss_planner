"""Evaluate individual Bench2Drive routes with bounded retries and owned-process cleanup.

Linux subreaping also catches CARLA servers that start their own session and
outlive the evaluator. No process is selected by name or port.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import shutil
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path


def completed(path: Path, route_id: str) -> bool:
    """A terminal driving result is complete even when the route was failed."""
    try:
        result = json.loads(path.read_text())
        checkpoint = result["_checkpoint"]
        progress = checkpoint["progress"]
        records = checkpoint["records"]
        return (
            result.get("entry_status") == "Finished"
            and progress == [1, 1]
            and len(records) == 1
            and records[0]["route_id"] == f"RouteScenario_{route_id}_rep0"
            and (
                records[0]["status"] in ("Completed", "Perfect", "Failed")
                or records[0]["status"].startswith("Failed - ")
            )
        )
    except (OSError, ValueError, KeyError, IndexError, TypeError, AttributeError):
        return False


def descendants(pid: int) -> set[int]:
    """Read only this invocation's descendant tree, including adopted orphans."""
    found = set()
    try:
        children = Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
    except FileNotFoundError:
        return found
    for child in children:
        child_pid = int(child)
        found.add(child_pid)
        found.update(descendants(child_pid))
    return found


def cleanup() -> None:
    # Parent-first termination makes detached grandchildren adoptable here.
    # Re-scan until all adopted processes are reaped, then force stubborn ones.
    deadline = time.monotonic() + 2.0
    while True:
        owned = descendants(os.getpid())
        for pid in owned:
            try:
                os.kill(pid, signal.SIGTERM if time.monotonic() < deadline else signal.SIGKILL)
            except ProcessLookupError:
                pass
        while True:
            try:
                if os.waitpid(-1, os.WNOHANG)[0] == 0:
                    break
            except ChildProcessError:
                break
        if not descendants(os.getpid()):
            return
        time.sleep(0.02)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("routes")
    ap.add_argument("out", type=Path)
    ap.add_argument("profile", nargs="?", default="bench2drive")
    ap.add_argument("ids", nargs="*")
    args = ap.parse_args()
    # PR_SET_CHILD_SUBREAPER: CARLA uses setsid(), so a process group alone
    # cannot own all evaluator descendants. Fail before spawning if unsupported.
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), "cannot enable child subreaping")

    def interrupted(signum, _frame):
        raise SystemExit(128 + signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    root = Path(os.environ.get("PIXI_PROJECT_ROOT", Path(__file__).resolve().parents[2]))
    b2d = Path(os.environ.get("BENCH2DRIVE_ROOT", root / "data/cache/bench2drive/Bench2Drive"))
    ids = args.ids or [
        r.attrib["id"] for r in ET.parse(b2d / "leaderboard/data" / f"{args.routes}.xml").getroot()
    ]
    if not ids or any(not rid.isdigit() for rid in ids):
        ap.error("route ids must be nonempty numeric identifiers")
    timeout = float(os.environ.get("ROUTE_TIMEOUT", "900"))
    if timeout <= 0:
        ap.error("ROUTE_TIMEOUT must be positive")
    args.out.mkdir(parents=True, exist_ok=True)
    failed = False
    for rid in ids:
        directory = args.out / rid
        checkpoint = directory / "results.json"
        for attempt in range(1, 3):
            if completed(checkpoint, rid):
                break
            # Keep interrupted attempts for diagnosis instead of deleting them.
            if directory.exists():
                archive = args.out.parent / f"{args.out.name}.incomplete" / f"{rid}-{time.time_ns()}"
                archive.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(directory), str(archive))
            cmd = [
                "bash",
                str(Path(__file__).with_name("run.sh")),
                args.routes,
                str(directory.resolve()),
                args.profile,
                rid,
                os.environ.get("PORT", "20000"),
            ]
            with (args.out / f"{rid}.attempt-{attempt}-{time.time_ns()}.log").open("w") as log:
                try:
                    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        proc.wait(timeout=timeout)
                    except subprocess.TimeoutExpired:
                        pass
                finally:
                    # Cleanup must finish even when another interrupt arrives.
                    for sig in (signal.SIGINT, signal.SIGTERM):
                        signal.signal(sig, signal.SIG_IGN)
                    cleanup()
                    for sig in (signal.SIGINT, signal.SIGTERM):
                        signal.signal(sig, interrupted)
        ok = completed(checkpoint, rid)
        failed |= not ok
        print(f"{rid} {int(ok)}", flush=True)
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
