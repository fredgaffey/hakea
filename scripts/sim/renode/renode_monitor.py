"""Tiny client for the Renode monitor's telnet port.

Renode serves its command monitor on a TCP port when started with
--port N (ESCSim's generator: --monitor-port N). Every command is a line
of text; Renode answers and then prints a prompt like "(am32) ". This
class sends one command and returns everything up to the next prompt.
"""
import re
import socket
import time

PROMPT = re.compile(rb"\((?:monitor|am32[^)]*)\) $")


class Monitor:
    def __init__(self, port, host="127.0.0.1", connect_timeout=120.0):
        deadline = time.time() + connect_timeout
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=5)
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.5)
        self.buf = b""
        self.n = 0
        # a no-op round trip: waits for Renode to finish its startup
        # script and drains whatever it printed
        self.cmd("", timeout=connect_timeout)

    def _read_until_prompt(self, timeout):
        deadline = time.time() + timeout
        self.sock.settimeout(1.0)
        while True:
            # strip telnet negotiation (IAC ...) and ANSI colour codes
            clean = re.sub(rb"\xff[\xfb-\xfe].|\xff[\xf0-\xfa]", b"", self.buf)
            clean = re.sub(rb"\x1b\[[0-9;]*[A-Za-z]", b"", clean)
            if PROMPT.search(clean):
                self.buf = b""
                return clean.decode("utf-8", "replace")
            if time.time() > deadline:
                raise TimeoutError("no Renode prompt; got %r" % clean[-500:])
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("Renode closed the monitor")
            self.buf += chunk

    def cmd(self, line, timeout=600.0):
        """Run one monitor command and return its output text.

        A second command printing a unique marker follows it, and we read
        until that marker, so output never gets attributed to the wrong
        command (Renode also prints prompts for its own startup script).
        """
        self.n += 1
        marker = "@@END%d@@" % self.n
        self.sock.sendall(line.encode() + b"\n")
        self.sock.sendall(('python "print \'%s\'"' % marker).encode() + b"\n")
        text = self._read_until(marker.encode(), timeout)
        text = text.replace("\r", "")
        # keep what follows the echo of our command, up to the marker line
        i = text.rfind(line)
        if i >= 0:
            text = text[i + len(line):]
        lines = [l for l in text.split("\n")
                 if l.strip() and marker not in l
                 and not PROMPT.search(l.encode() + b" ")]
        return "\n".join(lines)

    def _read_until(self, token, timeout):
        deadline = time.time() + timeout
        self.sock.settimeout(1.0)
        while True:
            clean = re.sub(rb"\xff[\xfb-\xfe].|\xff[\xf0-\xfa]", b"", self.buf)
            clean = re.sub(rb"\x1b\[[0-9;]*[A-Za-z]", b"", clean)
            clean = clean.replace(b"\r", b"")
            # the marker appears once echoed (inside quotes) and once printed
            # on its own line; wait for the printed one
            m = re.search(rb"(^|\n)" + re.escape(token) + rb"\s*\n", clean)
            if m:
                self.buf = b""
                return clean[:m.start() + 1].decode("utf-8", "replace")
            if time.time() > deadline:
                raise TimeoutError("timed out; got %r" % clean[-500:])
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("Renode closed the monitor")
            self.buf += chunk

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
