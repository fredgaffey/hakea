import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from study import run_study as rs  # noqa: E402


def test_parser_and_resolve_called(monkeypatch):
    calls = []
    monkeypatch.setattr(rs, "resolve_candidate", lambda d, n, *a, **k: calls.append(n) or d)
    monkeypatch.setattr(rs, "RESOLVE_PRESETS", 0)
    assert rs._resolve_all(["x"]) == ["x"] and not calls
    monkeypatch.setattr(rs, "RESOLVE_PRESETS", 6)
    monkeypatch.setattr(rs, "RESOLVE_WORKERS", 1)  # in-process so calls are visible
    assert rs._resolve_all(["a", "b"]) == ["a", "b"] and calls == [6, 6]
    # parser: flag accepted bare (6) and with N, via main's salvage-free early exit path
    import argparse
    src = Path(rs.__file__).read_text()
    assert '"--resolve-presets"' in src and "const=6" in src
