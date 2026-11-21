import subprocess, sys, time


def test_worker_exits_on_stdin_eof():
    p = subprocess.Popen([sys.executable, "-c", "from flexblade.remote import worker_main; worker_main(1)"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    time.sleep(3)           # let the pool come up
    p.stdin.close()
    try:
        p.wait(timeout=5)
    finally:
        if p.poll() is None:
            p.kill()
    assert p.returncode is not None
