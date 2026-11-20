"""Content-addressed evaluation cache (SQLite, WAL, zlib-compressed JSON).

Key = sha256(canonical design JSON [sorted keys, floats rounded to 12 significant digits]
+ evaluation kind + MODEL_VERSION). MODEL_VERSION = sha256 of all flexblade/*.py sources and
the data files, so any model change invalidates automatically; old versions stay in the table
until `prune(keep_versions=N)`.

File: ~/.cache/flexblade/evals.sqlite (env FLEXBLADE_CACHE=<path> overrides, =off disables).
Used by api.evaluate (full Results) and remote.HostPool.map (host side; workers need no DB).
Errors are never cached.

Note: DE samples continuous parameters, so exact repeats within one DE run are rare. The big
wins are re-runs, robust variants, resumed/restarted studies, verification and tests.

CLI: python -m flexblade.cache stats|prune [N]|clear
"""
from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import sys
import threading
import time
import zlib
from functools import lru_cache
from pathlib import Path

_PKG = Path(__file__).parent
_lock = threading.RLock()
_conn: dict[str, sqlite3.Connection] = {}
STATS = {"hits": 0, "misses": 0, "puts": 0}


def path() -> Path | None:
    v = os.environ.get("FLEXBLADE_CACHE")
    if v and v.lower() == "off":
        return None
    return Path(v).expanduser() if v else Path.home() / ".cache" / "flexblade" / "evals.sqlite"


@lru_cache(maxsize=1)
def model_version() -> str:
    h = hashlib.sha256()
    files = sorted(_PKG.rglob("*.py")) + sorted((_PKG / "data").rglob("*.json"))
    for f in files:
        if "__pycache__" in f.parts:
            continue
        h.update(str(f.relative_to(_PKG)).encode())
        h.update(f.read_bytes())
    return h.hexdigest()[:16]


def _round(o):
    if isinstance(o, float):
        return float(f"{o:.12g}") if o == o and abs(o) != float("inf") else repr(o)
    if isinstance(o, dict):
        return {k: _round(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_round(v) for v in o]
    return o


def make_key(design, kind: str = "") -> str:
    d = design.model_dump(mode="json") if hasattr(design, "model_dump") else design
    s = json.dumps(_round(d), sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(f"{s}|{kind}|{model_version()}".encode()).hexdigest()


def _db() -> sqlite3.Connection | None:
    p = path()
    if p is None:
        return None
    k = str(p)
    with _lock:
        c = _conn.get(k)
        if c is None:
            p.parent.mkdir(parents=True, exist_ok=True)
            c = sqlite3.connect(k, timeout=30, check_same_thread=False, isolation_level=None)
            c.execute("PRAGMA journal_mode=WAL")
            c.execute("PRAGMA synchronous=NORMAL")
            c.execute("CREATE TABLE IF NOT EXISTS evals (key TEXT PRIMARY KEY, version TEXT, "
                      "created REAL, data BLOB)")
            c.execute("CREATE INDEX IF NOT EXISTS evals_ver ON evals(version)")
            _conn[k] = c
        return c


def get(design, kind: str = ""):
    c = _db()
    if c is None:
        return None
    with _lock:
        row = c.execute("SELECT data FROM evals WHERE key=?", (make_key(design, kind),)).fetchone()
    if row is None:
        STATS["misses"] += 1
        return None
    STATS["hits"] += 1
    return json.loads(zlib.decompress(row[0]))


def put(design, result, kind: str = "") -> None:
    c = _db()
    if c is None or result is None:
        return
    blob = zlib.compress(json.dumps(result, default=str).encode(), 6)
    with _lock:
        c.execute("INSERT OR REPLACE INTO evals VALUES (?,?,?,?)",
                  (make_key(design, kind), model_version(), time.time(), blob))
    STATS["puts"] += 1


def prune(keep_versions: int = 1) -> int:
    c = _db()
    if c is None:
        return 0
    with _lock:
        vs = [r[0] for r in c.execute("SELECT version FROM evals GROUP BY version ORDER BY MAX(created) DESC")]
        keep = set(vs[:keep_versions]) | {model_version()}
        n = c.execute(f"DELETE FROM evals WHERE version NOT IN ({','.join('?' * len(keep))})",
                      tuple(keep)).rowcount
        c.execute("VACUUM")
    return n


def clear() -> None:
    c = _db()
    if c is not None:
        with _lock:
            c.execute("DELETE FROM evals")
            c.execute("VACUUM")


def stats() -> dict:
    p, c = path(), _db()
    if c is None:
        return {"enabled": False}
    per = dict(c.execute("SELECT version, COUNT(*) FROM evals GROUP BY version").fetchall())
    tot = STATS["hits"] + STATS["misses"]
    return {"enabled": True, "path": str(p), "size_bytes": p.stat().st_size if p.exists() else 0,
            "current_version": model_version(), "entries_per_version": per,
            "process": dict(STATS, hit_rate=(STATS["hits"] / tot if tot else None))}


def main(argv=None) -> None:
    a = list(sys.argv[1:] if argv is None else argv)
    cmd = a[0] if a else "stats"
    if cmd == "prune":
        print("pruned", prune(int(a[1]) if len(a) > 1 else 1))
    elif cmd == "clear":
        clear()
        print("cleared")
    elif cmd != "stats":
        sys.exit(__doc__)
    print(json.dumps(stats(), indent=1))


if __name__ == "__main__":
    main()
