"""Printed strip stop (strip_stop=True): stop position gives the design free length."""
import json
from pathlib import Path

import pytest

pytest.importorskip("manifold3d")

from flexblade.schema import Design  # noqa: E402
from flexblade import fittings as FT  # noqa: E402

ROOT = Path(__file__).resolve().parents[1] / "study/designs"


def test_stop_sets_free_length():
    D = Design.model_validate(json.loads((ROOT / "10in_easy_steelx.json").read_text()))
    prm = FT.strip_params(D)
    assert prm.stop and abs(prm.slot_t - (8 * 0.075 + 0.25)) < 1e-6
    # stop-to-stop minus the nominal cut length = modelled free length + the 0.3 mm long bias
    assert abs(FT.stop_free_length(D) - (prm.L + FT.STOP_BIAS)) < 0.05


def test_stop_default_off():
    D = Design.model_validate(json.loads((ROOT / "10in_tt_strap_steelx_v8.json").read_text()))
    assert not FT.strip_params(D).stop
