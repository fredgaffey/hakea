"""sa.py with the 5" throttle set: THRUST_FACTORS (0.7, 1.0, 1.25), continuous-eRPM nodes over
0.7-1.25 (SA_TRANGE=0.7,1.4 keeps the default range). Same argv/env as sa.py, e.g.
SA_JHUB=0.34e-6,0.91e-6,2.0e-6 SA_WORKERS=2 python sa_t125.py file <in.json> <out.json>"""
import functools
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from flexblade import robust  # noqa: E402

robust.THRUST_FACTORS = (0.7, 1.0, 1.25)
_tr = tuple(float(x) for x in os.environ.get("SA_TRANGE", "0.7,1.25").split(","))
robust.CQ_RPM_RANGE = _tr
robust.cont_points = functools.partial(robust.cont_points, trange=_tr)
sys.argv = [os.path.join(HERE, "sa.py")] + sys.argv[1:]
runpy.run_path(os.path.join(HERE, "sa.py"), run_name="__main__")
