"""Mount numbers: stiffness, modes, 1P amplitude, tilt, sag. Run from flexblade/ with PYTHONPATH=."""
import numpy as np
from flexblade.schema import Mount
from flexblade.stability import mount_stiffness, mount_modes

m = Mount(G_mpa=6.5, r_in_mm=4, r_out_mm=9, length_mm=10, infill=0.4)
km = mount_stiffness(m)
print("k_m N/m", km)
mr = 0.03
for fa in (190, 225, 260):
    mt = 0.25 + mr
    r = mount_modes(0.045 + mr, 0.25 - 0.045, mt * (2 * np.pi * fa) ** 2, 0.02, km, 0.15)
    print(fa, [round(x, 3) for x in r])
f0, z, mm = 140, 0.06, 0.09
for f1p in (190, 217, 250):
    r = f1p / f0
    x = 2e-6 / mm * r * r / np.hypot(1 - r * r, 2 * z * r)
    print("1P", f1p, "amp um (2 g.mm unbalance)", x * 1e6)
Ec = 3.4 * 6.5e6 * 0.4
kth = Ec * np.pi * (9e-3 ** 4 - 4e-3 ** 4) / 4 / 10e-3
print("ktheta", kth, "tilt deg @0.0975Nm", np.degrees(0.0975 / kth),
      "axial sag um @10N", 10 / (Ec * np.pi * 65e-6 / 10e-3) * 1e6)
