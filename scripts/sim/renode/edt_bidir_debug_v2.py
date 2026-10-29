"""Reproducer for the v2 bidirectional-DShot blocker (see notes/sim/renode/v2).

Boots the PR #438 F421 ELF in Renode with bidirectional DShot300, waits for
armed, sends command 13, and prints firmware DShot state. Observed: decoding
stops after exactly 1.0 s of frames (dshot_goodcounts freezes, 100 bad), so
command 13 never registers. Env: B=batch us, FR=frame us, TB=True/False,
ELF=path. Monitor on TCP 34574. Kills Renode on exit.
"""
import os, sys, subprocess, signal, time
sys.path.insert(0, os.path.expanduser('~/vehicles/hakea/scripts/sim/renode'))
from renode_monitor import Monitor
from run_crosscheck_v2 import elf_symbols
H=os.path.expanduser('~/vehicles/hakea')
elf=os.environ.get('ELF', H+'/firmware/AM32_FOXEER_F421_head-pr438_bdbc75e.elf')
env=dict(os.environ, BATCH_US=os.environ.get('B','2'))
cmd=[sys.executable, H+'/scripts/sim/renode/gen_batch_wrapper.py','FOXEER_F421','--outdir',H+'/scripts/sim/renode/work_v2/edt_debug','--targets-file',H+'/am32-renode-pr438/Inc/targets.h','--elf',elf,'--eeprom',os.path.expanduser('~/Downloads/esc1_config.bin'),'--model',H+'/scripts/sim/models/hakea_calibrated_prop_24v3.json','--monitor-port','34574','--can-bus','-1','--run']
p=subprocess.Popen(cmd, stdout=open(H+'/scripts/sim/renode/work_v2/edt_debug.log','w'), stderr=subprocess.STDOUT, start_new_session=True, env=env)
try:
    m=Monitor(34574)
    m.cmd('python "execfile(\'%s\')"' % (H+'/scripts/sim/renode/hakea_stim_v2.py'))
    names={'armed','running','dshotcommand','command_count','last_command','dshot_goodcounts','dshot_badcounts','dshot_telemetry','dshot_extended_telemetry','newinput','input','EDT_ARMED','EDT_ARM_ENABLE','programming_mode','send_EDT_init'}
    m.cmd('python "set_syms(%r)"' % elf_symbols(elf, names))
    q=lambda: m.cmd('python "print [(n,_rd(n)) for n in sorted(HSYMS)]"')
    m.cmd('python "setup_dshot(300, %s, True)"' % os.environ.get('FR','250')); m.cmd('python "_thr.TelemetryBit = %s"' % os.environ.get('TB','False'))
    print(m.cmd('python "print wait_armed(6.0)"')); print(q())
    for v in [13]*5:
        m.cmd('python "run_idle(0.002, %d)"' % v); print(q())
    m.cmd('python "run_idle(0.05, 13)"'); print(q())
finally:
    os.killpg(p.pid, signal.SIGKILL)
