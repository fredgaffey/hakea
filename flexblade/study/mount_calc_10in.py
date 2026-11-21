"""10in mount numbers (v5): stiffness, modes, 1P amplitude, tilt, sag. Run from flexblade/ with PYTHONPATH=.
Masses: vehicle 1.697 kg, arm/airframe modal mass 0.25 x vehicle = 0.424 kg (stability.make_airframe default),
rotating-side mass on the mount = motor 3115 (~90 g) + carrier 12 g + hub assy 17.7 g + screws 4 g = 0.124 kg."""
import numpy as np
from flexblade.schema import Mount
from flexblade.stability import mount_stiffness, mount_modes
m = Mount(G_mpa=6.5, eta=0.15, r_in_mm=4, r_out_mm=9, length_mm=10, infill=0.4)
km = mount_stiffness(m)
print("k_m N/m", km)
mh, ma = 0.124, 0.424 - 0.124
for fa in (190, 225, 260):
    r = mount_modes(mh, ma, (mh + ma) * (2 * np.pi * fa) ** 2, 0.02, km, 0.15)
    print("f_arm", fa, [round(x, 4) for x in r])
f0, z = np.sqrt(km / mh) / 2 / np.pi, 0.07
for f1p in (150, 183, 230):
    rr = f1p / f0
    x = 2e-6 / mh * rr * rr / np.hypot(1 - rr * rr, 2 * z * rr)
    print("hub mode Hz", round(f0, 1), "1P", f1p, "amp um (2 g.mm unbalance)", round(x * 1e6, 1))
Ec = 3.4 * 6.5e6 * 0.4
kth = Ec * np.pi * (9e-3 ** 4 - 4e-3 ** 4) / 4 / 10e-3
print("ktheta", round(kth, 2), "tilt deg @0.0975Nm", round(np.degrees(0.0975 / kth), 2),
      "axial sag um @16.65 N (hover thrust)", round(16.65 / (Ec * np.pi * 65e-6 / 10e-3) * 1e6, 1))
