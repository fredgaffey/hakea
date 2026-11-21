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
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator

REMOTE_ROOT = "/tmp/flexblade-work"
REMOTE_PY = f"{REMOTE_ROOT}/venv/bin/python"
PKG_DIR = Path(__file__).resolve().parent          # .../flexblade/flexblade
REPO_URL = "https://github.com/fredgaffey/hakea.git"
DEPS = ("numpy scipy matplotlib numba pydantic neuralfoil aerosandbox scikit-fem meshio "
        "gmsh trimesh manifold3d pytest")


# --------------------------------------------------------------------------- remote setup

def _ssh(host: str, cmd: str, timeout: float = 900, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host, cmd],
                          capture_output=True, text=True, timeout=timeout, check=check,
                          stdin=subprocess.DEVNULL)


def bootstrap(host: str, python: str = "/opt/homebrew/bin/python3", tag: str | None = None) -> None:
    """(Re)create the remote venv + package if the temp dir was wiped."""
    probe = _ssh(host, f"test -x {REMOTE_PY} && {REMOTE_PY} -c 'import flexblade, manifold3d' && echo ok",
                 check=False)   # manifold3d: steelx fittings (rotor.fitting_mass); a host without it fails every eval fast and steals the queue
    if "ok" not in probe.stdout:
        _ssh(host, f"set -e; rm -rf {REMOTE_ROOT}; mkdir -p {REMOTE_ROOT}; cd {REMOTE_ROOT}; "
                   f"git clone -q --depth 1 -b flexblade {REPO_URL} hakea; "
                   f"{python} -m venv venv; venv/bin/pip install -q --upgrade pip; "
                   f"venv/bin/pip install -q {DEPS}; venv/bin/pip install -q -e hakea/flexblade",
             timeout=1800)
    sync(host, tag)


def remote_pkg_parent(tag: str | None = None) -> str:
    """Remote dir holding the `flexblade` package. tag=None: the shared clone (editable
    install); a tag gives a private copy (several worktrees/agents can share the Mac
    without overwriting each other's code), put first on PYTHONPATH."""
    return f"{REMOTE_ROOT}/hakea/flexblade" if not tag else f"{REMOTE_ROOT}/pkg-{tag}"


def sync(host: str, tag: str | None = None) -> None:
    """Push the local package source (incl. uncommitted edits) to the remote clone."""
    parent = remote_pkg_parent(tag)
    if tag:
        _ssh(host, f"mkdir -p {parent}/flexblade", timeout=60)
    dst = f"{host}:{parent}/flexblade/"
    subprocess.run(["rsync", "-a", "--delete", "--exclude", "__pycache__",
                    f"{PKG_DIR}/", dst], check=True, capture_output=True, timeout=120,
                   stdin=subprocess.DEVNULL)


# --------------------------------------------------------------------------- worker side

FORWARD_ENV = ("CQ_TOL",)   # robust-affecting env; SA_GATES is not used anywhere in flexblade


def forward_env() -> dict:
    return {k: os.environ[k] for k in FORWARD_ENV if os.environ.get(k)}


def _eval_job(job: dict) -> dict:
    from . import api
    for k in FORWARD_ENV:                  # call-time: driver's tolerance set wins over worker env
        if k in job.get("env", {}):
            os.environ[k] = job["env"][k]
        elif "env" in job:
            os.environ.pop(k, None)
    from .uicommon import strip_heavy
    try:
        r = api.evaluate(job["design"], job.get("what")).model_dump(mode="json")
        return {"id": job["id"], "results": r if job.get("full") else strip_heavy(r)}
    except Exception as e:  # noqa: BLE001
        return {"id": job["id"], "error": f"{type(e).__name__}: {e}"}


WATCHDOG_PERIOD = 10.0


def worker_main(workers: int) -> None:
    """stdin JSON-lines jobs → stdout JSON-lines results, as they complete."""
    import multiprocessing as mp
    # serial kernels in every evaluating process (inherited by the spawned pool): the host's
    # cores are filled by processes; threaded numba/BLAS on top oversubscribes (Mac load >100)
    os.environ["FLEXBLADE_WORKER"] = "1"
    os.environ["FLEXBLADE_CACHE"] = "off"     # host-side HostPool.map caches; workers must not
                                              # hammer the same SQLite file from N processes
    for k in ("NUMBA_NUM_THREADS", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
              "VECLIB_MAXIMUM_THREADS"):
        os.environ[k] = "1"
    lock = threading.Lock()
    out = sys.stdout

    last = [time.monotonic()]
    npend = [0]
    pool_ref: list = []

    def die(code=0):
        for p in pool_ref:
            try:
                p.terminate()
            except Exception:
                pass
        os._exit(code)

    def emit(rec):
        last[0] = time.monotonic()
        npend[0] = max(0, npend[0] - 1)
        try:
            with lock:
                out.write(json.dumps(rec, default=str) + "\n")
                out.flush()
        except (BrokenPipeError, OSError, ValueError):
            die(0)                                # driver gone: nobody to hear results

    try:
        idle_s = float(os.environ.get("FLEXBLADE_WORKER_IDLE_MIN", "360")) * 60
    except ValueError:
        idle_s = 1800.0
    def watchdog():
        while True:
            time.sleep(WATCHDOG_PERIOD)
            if os.getppid() == 1 or (npend[0] == 0 and time.monotonic() - last[0] > idle_s):
                die(0)

    with mp.get_context("spawn").Pool(workers) as pool:
        pool_ref.append(pool)
        threading.Thread(target=watchdog, daemon=True).start()
        pending = []
        while True:
            try:
                line = sys.stdin.readline()
            except (OSError, ValueError):
                break
            if not line:                          # EOF: driver closed / died
                break
            line = line.strip()
            if not line:
                continue
            try:
                job = json.loads(line)
            except ValueError:
                continue
            last[0] = time.monotonic()
            npend[0] += 1
            pending.append(pool.apply_async(_eval_job, (job,), callback=emit,
                                            error_callback=lambda e, j=job: emit(
                                                {"id": j["id"], "error": repr(e)})))
        # stdin EOF: normally the host closed it with nothing pending. If our parent died (host
        # killed → we are reparented to init), drop queued work instead of finishing it as an
        # orphan that oversubscribes the machine (seen 2026-11-21: 20 orphan workers for 75 min).
        ppid0 = os.getppid()
        for p in pending:
            while not p.ready():
                if os.getppid() != ppid0 or os.getppid() == 1:
                    pool.terminate()
                    return
                p.wait(1.0)


# --------------------------------------------------------------------------- client side

def parse_hosts(spec: str) -> list[tuple[str, int]]:
    out = []
    for part in spec.split(","):
        name, _, n = part.strip().partition(":")
        out.append((name, int(n) if n else (os.cpu_count() or 2) - 1 if name == "local" else 8))
    return out


class HostHealth:
    """Per-host result/error counts; quarantine a host with >= min_n results and >= frac errors."""

    def __init__(self, min_n: int = 20, frac: float = 0.5):
        self.min_n, self.frac = min_n, frac
        self.n: dict[str, int] = {}
        self.err: dict[str, int] = {}
        self.first: dict[str, str] = {}
        self.bad: set[str] = set()

    def record(self, host: str, rec: dict) -> bool:
        """Count one result; True exactly once, when this result quarantines the host."""
        self.n[host] = self.n.get(host, 0) + 1
        if "error" in rec:
            self.err[host] = self.err.get(host, 0) + 1
            self.first.setdefault(host, str(rec["error"]))
        if host not in self.bad and self.n[host] >= self.min_n and self.err.get(host, 0) >= self.frac * self.n[host]:
            self.bad.add(host)
            return True
        return False


class HostPool:
    """Work-stealing pool across hosts. Use as a context manager.

    pool.map(fn_jobs) is not generic: it evaluates Designs. `imap(designs)` yields
    (index, record) in completion order; `map(designs)` returns records in order.
    Safe to call repeatedly (e.g. once per optimiser generation) and from several
    threads: each host has one persistent reader thread that routes results to the
    call that submitted the job.
    """

    def __init__(self, hosts: str | list[tuple[str, int]] = "local", what: list[str] | None = None,
                 full: bool = False, setup: bool = True, tag: str | None = None):
        """tag: private remote code dir (default env FLEXBLADE_REMOTE_TAG; None = shared clone)."""
        self.hosts = parse_hosts(hosts) if isinstance(hosts, str) else hosts
        self.what, self.full = what, full
        self.tag = tag if tag is not None else (os.environ.get("FLEXBLADE_REMOTE_TAG") or None)
        self.procs: dict[str, subprocess.Popen] = {}
        self._sem: dict[str, threading.Semaphore] = {}
        self._dead: dict[str, threading.Event] = {}
        self._inflight: dict[int, tuple] = {}   # gid → host, results q, i
        self._lock = threading.Lock()
        self._wlock: dict[str, threading.Lock] = {}
        self._gid = 0
        self.health = HostHealth()
        if setup:
            for name, _ in self.hosts:
                if name != "local":
                    bootstrap(name, tag=self.tag)

    def __enter__(self):
        for name, slots in self.hosts:
            if name == "local":
                cmd = [sys.executable, "-m", "flexblade.remote", "--workers", str(slots)]
            else:
                slots = min(slots, 10)                    # shared Mac: ≤ 10 slots
                env = "FLEXBLADE_WORKER=1 NUMBA_NUM_THREADS=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 "
                env += "".join(f"{k}={shlex.quote(v)} " for k, v in forward_env().items())
                env += f"PYTHONPATH={remote_pkg_parent(self.tag)} " if self.tag else ""
                cmd = ["ssh", "-o", "BatchMode=yes", name,
                       f"cd {REMOTE_ROOT} && {env}{REMOTE_PY} -m flexblade.remote --workers {slots}"]
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, text=True, bufsize=1)
            self.procs[name] = proc
            self._sem[name] = threading.Semaphore(slots)
            self._dead[name] = threading.Event()
            self._wlock[name] = threading.Lock()
            threading.Thread(target=self._reader, args=(name, proc), daemon=True).start()
        return self

    def _reader(self, name: str, proc: subprocess.Popen) -> None:
        for line in proc.stdout:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            with self._lock:
                ent = self._inflight.pop(rec.get("id"), None)
            self._sem[name].release()
            if ent is None:
                continue
            _, q, i, jobs, dj = ent
            if self.health.record(name, rec):
                print(f"WARNING: host {name} quarantined ({self.health.err[name]}/{self.health.n[name]} "
                      f"errors); first error: {self.health.first[name]}", flush=True)
                self._dead[name].set()
                with self._lock:      # requeue its pending jobs on the other hosts
                    gs = [g for g, e in self._inflight.items() if e[0] == name]
                    pend = [self._inflight.pop(g) for g in gs]
                for e in pend + [ent]:
                    e[3].put((e[2], e[4]))
                continue
            if name in self.health.bad and "error" in rec:   # late error from quarantined host
                jobs.put((i, dj))
                continue
            rec["id"] = i
            rec["host"] = name
            q.put((i, rec))
        self._dead[name].set()
        with self._lock:      # host died: fail its in-flight jobs
            lost = [(g, e) for g, e in self._inflight.items() if e[0] == name]
            for g, _ in lost:
                self._inflight.pop(g)
        for _, (_, q, i, *_r) in lost:
            q.put((i, {"id": i, "error": f"host {name} died", "host": name}))

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

        def feed(name: str):
            sem, dead, proc = self._sem[name], self._dead[name], self.procs[name]
            while not dead.is_set():
                if not sem.acquire(timeout=1.0):
                    continue
                try:
                    i, dj = jobs.get_nowait()
                except queue.Empty:
                    sem.release()
                    return
                with self._lock:
                    self._gid += 1
                    gid = self._gid
                    self._inflight[gid] = (name, results, i, jobs, dj)
                try:
                    with self._wlock[name]:
                        proc.stdin.write(json.dumps({"id": gid, "design": dj, "what": self.what,
                                                     "full": self.full, "env": forward_env()}) + "\n")
                        proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    with self._lock:
                        self._inflight.pop(gid, None)
                    jobs.put((i, dj))            # let another host take it
                    dead.set()
                    return

        threads = [threading.Thread(target=feed, args=(n,), daemon=True) for n, _ in self.hosts]
        for t in threads:
            t.start()
        for _ in range(len(designs)):
            while True:
                try:
                    yield results.get(timeout=5.0)
                    break
                except queue.Empty:
                    if all(self._dead[n].is_set() for n, _ in self.hosts):
                        if self.health.bad >= {n for n, _ in self.hosts}:
                            raise RuntimeError("all hosts quarantined (see first-error warnings)")
                        raise RuntimeError("all hosts died")
                    # jobs left behind by a host that died while feeding → serve them
                    if not jobs.empty() and not any(t.is_alive() for t in threads):
                        threads = [threading.Thread(target=feed, args=(n,), daemon=True)
                                   for n, _ in self.hosts if not self._dead[n].is_set()]
                        for t in threads:
                            t.start()

    def map_designs(self, designs: Iterable[Any]) -> list[dict]:
        """Alias of `map`; marks this object as a Design mapper for optimize.run."""
        return self.map(designs)

    def map(self, designs: Iterable[Any]) -> list[dict]:
        """Host-side cache: look up, dispatch unique misses only, write back (errors not cached)."""
        from . import cache
        designs = list(designs)
        kind = f"remote:{','.join(self.what or [])}:{bool(self.full)}"
        out: list[dict | None] = [None] * len(designs)
        first: dict[str, int] = {}
        misses: list[Any] = []
        slots: list[list[int]] = []
        for i, d in enumerate(designs):
            hit = cache.get(d, kind)
            if hit is not None:
                out[i] = dict(hit, id=i)
                continue
            k = cache.make_key(d, kind)
            if k not in first:
                first[k] = len(misses)
                misses.append(d)
                slots.append([])
            slots[first[k]].append(i)
        for j, rec in self.imap(misses):
            if isinstance(rec, dict) and "error" not in rec:
                cache.put(misses[j], rec, kind)
            for i in slots[j]:
                out[i] = dict(rec, id=i)
        return out  # type: ignore[return-value]


def main(argv: list[str] | None = None) -> None:
    import argparse
    ap = argparse.ArgumentParser(prog="flexblade.remote", description="JSON-lines evaluation worker")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args(argv)
    worker_main(a.workers)


if __name__ == "__main__":
    main()
