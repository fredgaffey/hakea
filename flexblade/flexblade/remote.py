"""Spread evaluations across machines (laptop + Mac over ssh).

Each host runs `flexblade worker` (JSON lines on stdin → JSON lines on stdout), which
evaluates jobs on a local process pool. `HostPool` keeps every host's slots busy
(work stealing), so a fast and a slow machine finish together.

Hosts spec: "local:31,mac:10"  (name:slots; "local" = this machine, other names are
ssh aliases). Remote hosts get the current package code via rsync before use, into a
temp dir that may be wiped at any time (recreated by `bootstrap`).

Job:    {"id": int, "design": {...Design json...}, "what": [...], "full": bool}
Result: {"id": int, "results": {...}} or {"id": int, "error": "..."}
"""
from __future__ import annotations

import json
import os
import queue
import shlex
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator

REMOTE_ROOT = "/tmp/flexblade-work"
REMOTE_PY = f"{REMOTE_ROOT}/venv/bin/python"
PKG_DIR = Path(__file__).resolve().parent          # .../flexblade/flexblade
REPO_URL = "https://github.com/fredgaffey/hakea.git"
DEPS = ("numpy scipy matplotlib numba pydantic neuralfoil aerosandbox scikit-fem meshio "
        "gmsh trimesh pytest")


# --------------------------------------------------------------------------- remote setup

def _ssh(host: str, cmd: str, timeout: float = 900, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host, cmd],
                          capture_output=True, text=True, timeout=timeout, check=check)


def bootstrap(host: str, python: str = "/opt/homebrew/bin/python3") -> None:
    """(Re)create the remote venv + package if the temp dir was wiped."""
    probe = _ssh(host, f"test -x {REMOTE_PY} && {REMOTE_PY} -c 'import flexblade' && echo ok",
                 check=False)
    if "ok" not in probe.stdout:
        _ssh(host, f"set -e; rm -rf {REMOTE_ROOT}; mkdir -p {REMOTE_ROOT}; cd {REMOTE_ROOT}; "
                   f"git clone -q --depth 1 -b flexblade {REPO_URL} hakea; "
                   f"{python} -m venv venv; venv/bin/pip install -q --upgrade pip; "
                   f"venv/bin/pip install -q {DEPS}; venv/bin/pip install -q -e hakea/flexblade",
             timeout=1800)
    sync(host)


def sync(host: str) -> None:
    """Push the local package source (incl. uncommitted edits) to the remote clone."""
    dst = f"{host}:{REMOTE_ROOT}/hakea/flexblade/flexblade/"
    subprocess.run(["rsync", "-a", "--delete", "--exclude", "__pycache__",
                    f"{PKG_DIR}/", dst], check=True, capture_output=True, timeout=120)


# --------------------------------------------------------------------------- worker side

def _eval_job(job: dict) -> dict:
    from . import api
    from .uicommon import strip_heavy
    try:
        r = api.evaluate(job["design"], job.get("what")).model_dump(mode="json")
        return {"id": job["id"], "results": r if job.get("full") else strip_heavy(r)}
    except Exception as e:  # noqa: BLE001
        return {"id": job["id"], "error": f"{type(e).__name__}: {e}"}


def worker_main(workers: int) -> None:
    """stdin JSON-lines jobs → stdout JSON-lines results, as they complete."""
    import multiprocessing as mp
    os.environ["FLEXBLADE_WORKER"] = "1"   # inherited by spawned pool processes
    lock = threading.Lock()
    out = sys.stdout

    def emit(rec):
        with lock:
            out.write(json.dumps(rec, default=str) + "\n")
            out.flush()

    with mp.get_context("spawn").Pool(workers) as pool:
        pending = []
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            job = json.loads(line)
            pending.append(pool.apply_async(_eval_job, (job,), callback=emit,
                                            error_callback=lambda e, j=job: emit(
                                                {"id": j["id"], "error": repr(e)})))
        for p in pending:
            p.wait()


# --------------------------------------------------------------------------- client side

def parse_hosts(spec: str) -> list[tuple[str, int]]:
    out = []
    for part in spec.split(","):
        name, _, n = part.strip().partition(":")
        out.append((name, int(n) if n else (os.cpu_count() or 2) - 1 if name == "local" else 8))
    return out


class HostPool:
    """Work-stealing pool across hosts. Use as a context manager.

    pool.map(fn_jobs) is not generic: it evaluates Designs. `imap(designs)` yields
    (index, record) in completion order; `map(designs)` returns records in order.
    """

    def __init__(self, hosts: str | list[tuple[str, int]] = "local", what: list[str] | None = None,
                 full: bool = False, setup: bool = True):
        self.hosts = parse_hosts(hosts) if isinstance(hosts, str) else hosts
        self.what, self.full = what, full
        self.procs: dict[str, subprocess.Popen] = {}
        if setup:
            for name, _ in self.hosts:
                if name != "local":
                    bootstrap(name)

    def __enter__(self):
        for name, slots in self.hosts:
            if name == "local":
                cmd = [sys.executable, "-m", "flexblade.remote", "--workers", str(slots)]
            else:
                cmd = ["ssh", "-o", "BatchMode=yes", name,
                       f"{REMOTE_PY} -m flexblade.remote --workers {slots}"]
            self.procs[name] = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                                stderr=subprocess.DEVNULL, text=True, bufsize=1)
        return self

    def __exit__(self, *exc):
        for p in self.procs.values():
            try:
                p.stdin.close()
            except Exception:  # noqa: BLE001
                pass
        for p in self.procs.values():
            try:
                p.wait(timeout=30)
            except subprocess.TimeoutExpired:
                p.kill()

    def imap(self, designs: Iterable[Any]) -> Iterator[tuple[int, dict]]:
        jobs: "queue.Queue[tuple[int, dict]]" = queue.Queue()
        designs = list(designs)
        for i, d in enumerate(designs):
            dj = d.model_dump(mode="json") if hasattr(d, "model_dump") else d
            jobs.put((i, dj))
        results: "queue.Queue[tuple[int, dict]]" = queue.Queue()

        def drive(name: str, slots: int, proc: subprocess.Popen):
            inflight: dict[int, dict] = {}
            lock = threading.Lock()
            done_reading = threading.Event()

            def reader():
                for line in proc.stdout:
                    rec = json.loads(line)
                    with lock:
                        inflight.pop(rec["id"], None)
                    rec["host"] = name
                    results.put((rec["id"], rec))
                    sem.release()
                done_reading.set()

            sem = threading.Semaphore(slots)
            threading.Thread(target=reader, daemon=True).start()
            while True:
                sem.acquire()
                if done_reading.is_set():
                    break
                try:
                    i, dj = jobs.get_nowait()
                except queue.Empty:
                    sem.release()
                    break
                with lock:
                    inflight[i] = dj
                try:
                    proc.stdin.write(json.dumps({"id": i, "design": dj, "what": self.what,
                                                 "full": self.full}) + "\n")
                    proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    break
            # wait for this host's in-flight jobs; if the host died, requeue them
            while True:
                with lock:
                    left = dict(inflight)
                if not left:
                    return
                if done_reading.wait(timeout=1.0):
                    for i, dj in left.items():
                        results.put((i, {"id": i, "error": f"host {name} died", "host": name}))
                    return

        threads = [threading.Thread(target=drive, args=(n, s, self.procs[n]), daemon=True)
                   for n, s in self.hosts]
        for t in threads:
            t.start()
        for _ in range(len(designs)):
            yield results.get()

    def map(self, designs: Iterable[Any]) -> list[dict]:
        designs = list(designs)
        out: list[dict | None] = [None] * len(designs)
        for i, rec in self.imap(designs):
            out[i] = rec
        return out  # type: ignore[return-value]


def main(argv: list[str] | None = None) -> None:
    import argparse
    ap = argparse.ArgumentParser(prog="flexblade.remote", description="JSON-lines evaluation worker")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args(argv)
    worker_main(a.workers)


if __name__ == "__main__":
    main()
