"""Verify and unpack the original 20-scenario CI selection without network access."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def prepare(out: Path) -> int:
    manifest = json.loads((ROOT / "ci_scenarios.json").read_text())
    data = (ROOT / "ci_scenarios.tar.gz").read_bytes()
    if hashlib.sha256(data).hexdigest() != manifest["archive_sha256"]:
        raise ValueError("CI scenario archive checksum mismatch")
    expected = manifest["files"]
    verified = {}
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        for member in archive.getmembers():
            if (
                not member.isfile()
                or member.name not in expected
                or member.name in verified
                or Path(member.name).name != member.name
            ):
                raise ValueError(f"unexpected CI scenario archive member: {member.name}")
            with archive.extractfile(member) as source:
                content = source.read()
            if hashlib.sha256(content).hexdigest() != expected[member.name]:
                raise ValueError(f"CI scenario checksum mismatch: {member.name}")
            verified[member.name] = content
    if set(verified) != set(expected) or len(verified) != 20:
        raise ValueError("CI scenario archive must contain the complete original 20-scenario selection")
    out.mkdir(parents=True, exist_ok=True)
    for name, content in verified.items():
        (out / name).write_bytes(content)
    return len(verified)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("data/cache/scenarios/hand-crafted"))
    args = parser.parse_args()
    print(f"Verified and prepared {prepare(args.out)} pinned CI scenarios")


if __name__ == "__main__":
    main()
