"""OpenFOAM spot-check CFD (OWNER: fea agent).

* airfoil2d.pitching_airfoil_case — 2D pitching section URANS (Theodorsen / aero.linearized check)
* rotor3d.rotor_sector_case       — 3D single-blade cyclic sector SRF hover (BEMT check)
* runner.run_case                 — run on the Mac (ssh, /tmp/flexblade-cfd), docker or locally
* post                            — .dat parsers, harmonic fit, Theodorsen reference, hover forces
"""
from .airfoil2d import PitchCase, pitching_airfoil_case, station_conditions  # noqa: F401
from .post import (harmonic_fit, hover_forces, pitching_response, read_dat,  # noqa: F401
                   theodorsen_pitch_cl)
from .rotor3d import RotorCase, rotor_sector_case  # noqa: F401
from .runner import docker_status, mac_status, run_case  # noqa: F401
