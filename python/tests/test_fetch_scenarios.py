"""Scenario downloads tolerate transient failures and retain pinned archive semantics."""

import importlib.util
import io
import tarfile
import urllib.error
from pathlib import Path
from unittest.mock import Mock

import pytest

SOURCE = Path(__file__).resolve().parents[2] / "benchmarks/commonroad/fetch_scenarios.py"
spec = importlib.util.spec_from_file_location("fetch_scenarios", SOURCE)
fetcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fetcher)


@pytest.mark.parametrize(
    "error",
    [
        urllib.error.URLError(OSError(101, "Network is unreachable")),
        urllib.error.HTTPError("https://example.test", 503, "Unavailable", {}, None),
        urllib.error.HTTPError("https://example.test", 429, "Too many requests", {}, None),
        TimeoutError("timed out"),
    ],
)
def test_download_recovers_from_transient_failures(monkeypatch, error):
    request = Mock(side_effect=[error, io.BytesIO(b"archive")])
    sleep = Mock()
    monkeypatch.setattr(fetcher.urllib.request, "urlopen", request)
    monkeypatch.setattr(fetcher.time, "sleep", sleep)
    assert fetcher.download("https://example.test/archive") == b"archive"
    assert request.call_count == 2
    request.assert_called_with("https://example.test/archive", timeout=60)
    sleep.assert_called_once_with(2)


def test_download_does_not_retry_a_missing_archive(monkeypatch):
    request = Mock(side_effect=urllib.error.HTTPError("https://example.test", 404, "Not Found", {}, None))
    sleep = Mock()
    monkeypatch.setattr(fetcher.urllib.request, "urlopen", request)
    monkeypatch.setattr(fetcher.time, "sleep", sleep)
    with pytest.raises(urllib.error.HTTPError):
        fetcher.download("https://example.test/archive")
    assert request.call_count == 1
    sleep.assert_not_called()


def test_download_stops_after_three_attempts(monkeypatch):
    request = Mock(side_effect=urllib.error.URLError("unreachable"))
    sleep = Mock()
    monkeypatch.setattr(fetcher.urllib.request, "urlopen", request)
    monkeypatch.setattr(fetcher.time, "sleep", sleep)
    with pytest.raises(urllib.error.URLError):
        fetcher.download("https://example.test/archive")
    assert request.call_count == 3
    assert [call.args[0] for call in sleep.call_args_list] == [2, 4]


def test_fetch_extracts_the_pinned_subset(monkeypatch, tmp_path):
    data = io.BytesIO()
    with tarfile.open(fileobj=data, mode="w:gz") as archive:
        for name, content in [("hand-crafted/example.xml", b"<scenario/>"), ("README.md", b"notes")]:
            member = tarfile.TarInfo(f"commonroad-scenarios-{fetcher.COMMIT}/scenarios/recorded/{name}")
            member.size = len(content)
            archive.addfile(member, io.BytesIO(content))
    download = Mock(return_value=data.getvalue())
    monkeypatch.setattr(fetcher, "download", download)
    assert fetcher.fetch("hand-crafted", tmp_path) == 1
    assert (tmp_path / "hand-crafted/example.xml").read_bytes() == b"<scenario/>"
    assert not (tmp_path / "README.md").exists()
    download.assert_called_once_with(fetcher.URL.format(c=fetcher.COMMIT, sub="hand-crafted"))


PREPARE_SOURCE = SOURCE.with_name("prepare_ci_scenarios.py")
prepare_spec = importlib.util.spec_from_file_location("prepare_ci_scenarios", PREPARE_SOURCE)
preparer = importlib.util.module_from_spec(prepare_spec)
prepare_spec.loader.exec_module(preparer)


def test_ci_snapshot_prepares_all_original_scenarios_offline(monkeypatch, tmp_path):
    import json

    request = Mock(side_effect=AssertionError("CI fixtures must not require network access"))
    monkeypatch.setattr(fetcher.urllib.request, "urlopen", request)
    manifest = json.loads((preparer.ROOT / "ci_scenarios.json").read_text())
    assert manifest["upstream_commit"] == fetcher.COMMIT
    assert manifest["selection"]["seed"] == 1
    assert manifest["selection"]["limit"] == 20
    assert preparer.prepare(tmp_path) == 20
    assert {p.name for p in tmp_path.glob("*.xml")} == set(manifest["files"])
    request.assert_not_called()


@pytest.mark.parametrize("corrupt_archive", [True, False])
def test_ci_snapshot_rejects_corruption_before_writing(monkeypatch, tmp_path, corrupt_archive):
    import json

    manifest = json.loads((preparer.ROOT / "ci_scenarios.json").read_text())
    data = (preparer.ROOT / "ci_scenarios.tar.gz").read_bytes()
    if corrupt_archive:
        data = data[:-1] + bytes([data[-1] ^ 1])
    else:
        first = next(iter(manifest["files"]))
        manifest["files"][first] = "0" * 64
    (tmp_path / "ci_scenarios.tar.gz").write_bytes(data)
    (tmp_path / "ci_scenarios.json").write_text(json.dumps(manifest))
    monkeypatch.setattr(preparer, "ROOT", tmp_path)
    out = tmp_path / "scenarios"
    with pytest.raises(ValueError, match="checksum mismatch"):
        preparer.prepare(out)
    assert not out.exists()
