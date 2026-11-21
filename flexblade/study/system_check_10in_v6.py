"""Integrated SYSTEM check for 10" v6 (I33): system_check_10in_v5.py with the v6 flexure, same twin-washer
mount (v5c), arms (120/150/200 Hz), rev-avg governor (20 Hz), J_hub, I_t x1/x2.
Run from the repo root with PYTHONPATH=$PWD/flexblade:$PWD/flexblade/study:$PWD/flexblade/study/build.
-> study/results/system_check_10in_v6.json"""
import sys
import mounts_tilt as MTL
import system_check_10in_v5 as SC
import tilt_whirl as TW

TW.DES["10in_v6"] = ("10in_tt_strap_steelx_v6", TW.DES["10in_v5"][1])
MTL.SIZES["10in_v6"] = MTL.SIZES["10in_v5"]
SC.CFG["10in_v6"] = SC.CFG["10in_v5"]

if __name__ == "__main__":
    SC.main("10in_v6")
