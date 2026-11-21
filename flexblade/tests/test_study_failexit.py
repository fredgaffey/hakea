"""run_study.main must exit non-zero when a case raises."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "study"))
import run_study as rs  # noqa: E402


def test_failed_case_exits_nonzero(monkeypatch, tmp_path):
    def boom(*a, **k):
        raise RuntimeError("boom")
    monkeypatch.setattr(rs, "run_case", boom)
    monkeypatch.setattr(rs, "RESULTS", tmp_path)
    with pytest.raises(SystemExit) as e:
        rs.main(["--hosts", "local:1", "--sizes", "7", "--concepts", "tt_strap_steelx",
                 "--threads", "1", "--no-single", "--coax", "7:tt_strap_steelx"])
    assert e.value.code == 2
