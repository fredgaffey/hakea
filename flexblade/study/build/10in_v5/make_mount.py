"""10in v5c twin-washer mount (k_t sized per the tilt rule). Run from flexblade/: PYTHONPATH=. python study/build/10in_v5/make_mount.py"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
import mount_twin as M
P = dict(ri=4, ro=5.8, t=12, d=84)
if __name__ == "__main__":
    print(M.props(**P))
    M.build(Path(__file__).parent, **P)
