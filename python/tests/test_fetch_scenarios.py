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
