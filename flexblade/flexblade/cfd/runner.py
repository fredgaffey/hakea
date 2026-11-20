"""Run OpenFOAM cases on the Mac (ssh), in docker, or locally; pull back only small results.

Mac rules (from Fred): all remote files live under /tmp/flexblade-cfd/<case>/, which may be
deleted at any time. Each run recreates the remote case from scratch (rsync --delete of the
locally generated case), runs, pulls back postProcessing/ + log.* immediately, then deletes
the remote case dir. Nothing is assumed to persist on the Mac.
"""
from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import time
from pathlib import Path
from typing import Literal, Optional

MAC_HOST = os.environ.get("FLEXBLADE_MAC_HOST", "mac")
MAC_ROOT = "/tmp/flexblade-cfd"
# OpenFOAM.app (gerlero) installs an `openfoam` launcher (ESI etc/openfoam: `-c 'cmd'`).
MAC_OPENFOAM = os.environ.get("FLEXBLADE_MAC_OPENFOAM", "openfoam")
DOCKER_IMAGE = os.environ.get("FLEXBLADE_OF_IMAGE", "opencfd/openfoam-run:2512")
MIN_FREE_GB = 5.0

_SOURCE_DOCKER = ("for f in /usr/lib/openfoam/openfoam*/etc/bashrc /opt/openfoam*/etc/bashrc; "
                  "do [ -f \"$f\" ] && . \"$f\" && break; done")


def _sh(cmd: list[str] | str, timeout: Optional[float] = None, check: bool = True, **kw):
    shell = isinstance(cmd, str)
    r = subprocess.run(cmd, shell=shell, capture_output=True, text=True, timeout=timeout, **kw)
    if check and r.returncode != 0:
        raise RuntimeError(f"command failed ({r.returncode}): {cmd}\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
    return r


def mac_status(host: str = MAC_HOST) -> dict:
    """Read-only check of OpenFOAM availability and free disk on the Mac."""
    cmd = ("export PATH=/opt/homebrew/bin:/usr/local/bin:$PATH; "
           f"command -v {MAC_OPENFOAM} || echo NOOPENFOAM; ls -d /Applications/OpenFOAM*.app 2>/dev/null; "
           "df -k /tmp | tail -1; sysctl -n hw.ncpu")
    r = _sh(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host, cmd], timeout=60,
            check=False)
    lines = r.stdout.strip().splitlines()
    free_gb = None
    ncpu = None
    for ln in lines:
        parts = ln.split()
        if len(parts) >= 4 and parts[3].isdigit():
            free_gb = int(parts[3]) / 1024 ** 2
        if ln.strip().isdigit():
            ncpu = int(ln.strip())
    return dict(reachable=r.returncode == 0, openfoam="NOOPENFOAM" not in r.stdout,
                apps=[ln for ln in lines if ln.startswith("/Applications")],
                free_gb=free_gb, ncpu=ncpu, raw=r.stdout + r.stderr)


def docker_status(image: str = DOCKER_IMAGE) -> dict:
    r = _sh(["docker", "image", "inspect", image], check=False, timeout=30)
    return dict(image=image, present=r.returncode == 0)


def clean_case(case: Path, keep_mesh: bool = False) -> None:
    """Delete bulky outputs (processor*, time dirs, polyMesh) keeping postProcessing/logs."""
    case = Path(case)
    for p in case.glob("processor*"):
        shutil.rmtree(p, ignore_errors=True)
    for p in case.iterdir():
        if p.is_dir():
            try:
                t = float(p.name)
            except ValueError:
                continue
            if t != 0:
                shutil.rmtree(p, ignore_errors=True)
    if not keep_mesh:
        shutil.rmtree(case / "constant" / "polyMesh", ignore_errors=True)


def run_case(case_dir: str | Path, where: Literal["mac", "docker", "local"] = "mac",
             np: int = 4, timeout: float = 6 * 3600, host: str = MAC_HOST,
             image: str = DOCKER_IMAGE, keep: bool = False) -> dict:
    """Run ./Allrun <np> for a generated case; return dict(ok, where, seconds, log_tail, files).

    mac: rsync case → /tmp/flexblade-cfd/<name> (fresh), run via the OpenFOAM.app launcher,
         pull back postProcessing/ and log.*, delete the remote dir (always, even on failure).
    docker: run in `image` with the case bind-mounted (uid/gid preserved), then clean bulky
         outputs unless keep. local: OpenFOAM must already be sourced in the environment.
    """
    case = Path(case_dir).resolve()
    if not (case / "Allrun").exists():
        raise FileNotFoundError(f"{case}/Allrun missing; generate the case first")
    t0 = time.time()
    if where == "mac":
        st = mac_status(host)
        if not st["reachable"]:
            raise RuntimeError(f"ssh {host} failed: {st['raw']}")
        if not st["openfoam"]:
            raise RuntimeError("OpenFOAM not installed on the Mac "
                               "(brew install --cask gerlero/openfoam/openfoam)")
        if st["free_gb"] is not None and st["free_gb"] < MIN_FREE_GB:
            raise RuntimeError(f"only {st['free_gb']:.1f} GB free on Mac /tmp")
        remote = f"{MAC_ROOT}/{case.name}"
        rq = shlex.quote(remote)
        _sh(["ssh", host, f"rm -rf {rq} && mkdir -p {rq}"], timeout=60)
        try:
            _sh(["rsync", "-a", "--delete", f"{case}/", f"{host}:{remote}/"], timeout=600)
            inner = f"cd {rq} && ./Allrun {int(np)}"
            cmd = (f"export PATH=/opt/homebrew/bin:/usr/local/bin:$PATH; "
                   f"{MAC_OPENFOAM} -c {shlex.quote(inner)}")
            r = _sh(["ssh", host, cmd], timeout=timeout, check=False)
            ok = r.returncode == 0 and "FAILED" not in r.stdout
            _sh(["rsync", "-a", "--include=postProcessing/***", "--include=log.*",
                 "--exclude=*", f"{host}:{remote}/", f"{case}/"], timeout=600, check=False)
        finally:
            _sh(["ssh", host, f"rm -rf {rq}"], timeout=60, check=False)
    elif where == "docker":
        uid, gid = os.getuid(), os.getgid()
        inner = f"{_SOURCE_DOCKER}; cd /case && MPIRUN_FLAGS='--oversubscribe' ./Allrun {int(np)}"
        r = _sh(["docker", "run", "--rm", "-u", f"{uid}:{gid}", "-e", "HOME=/tmp",
                 "-v", f"{case}:/case", "-w", "/case", "--entrypoint", "bash", image, "-c", inner],
                timeout=timeout, check=False)
        ok = r.returncode == 0 and "FAILED" not in r.stdout
        if not keep:
            clean_case(case)
    elif where == "local":
        r = _sh(f"cd {shlex.quote(str(case))} && ./Allrun {int(np)}", timeout=timeout, check=False)
        ok = r.returncode == 0 and "FAILED" not in r.stdout
        if not keep:
            clean_case(case)
    else:
        raise ValueError(where)
    files = sorted(str(p.relative_to(case)) for p in (case / "postProcessing").rglob("*.dat")) \
        if (case / "postProcessing").exists() else []
    return dict(ok=ok, where=where, seconds=time.time() - t0, returncode=r.returncode,
                log_tail=(r.stdout + r.stderr)[-3000:], files=files)
