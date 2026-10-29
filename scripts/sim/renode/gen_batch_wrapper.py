#!/usr/bin/env python3
"""ESCSim's Renode generator, but with a different bridge batch period.

ESCSim's generator hard-codes `batchUs: 20` for the bridge peripheral that
couples the emulated MCU to the motor physics: every 20 us of virtual time
it reads the bridge pins/timer and hands comparator edges back. This
wrapper (used by run_crosscheck.py --batch-us N) swaps in N without
editing ESCSim: it wraps generator.platform() and then runs generator.main()
with the same command-line arguments. BATCH_US comes from the environment.
"""
import os
import sys

import escsim.renode.generator as g

BATCH = int(os.environ["BATCH_US"])
_orig_platform = g.platform


def platform(cfg, sigrok=False):
    text = _orig_platform(cfg, sigrok)
    if "    batchUs: 20\n" not in text:
        raise SystemExit("gen_batch_wrapper: batchUs line not found in platform")
    return text.replace("    batchUs: 20\n", "    batchUs: %d\n" % BATCH)


g.platform = platform
sys.exit(g.main())
