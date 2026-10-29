# Hakea

A single-rotor swashplateless drone. One motor provides lift and, by pulsing
its torque once per revolution, cyclic control through a hinged hub, so no
servos or swashplate are needed.

Inspired by Joshua Bird's one-motor drone
(<https://www.youtube.com/watch?v=Ndftcft6XhU>).

## Approach

- **Motor:** T-Motor 2307 1300 KV, 5" rotor, 6S.
- **ESC:** AM32, modified so a magnetic encoder (MT6835) drives commutation.
  This keeps the motor in sync under fast torque swings, where back-EMF
  sensing desyncs.
- **Flight controller:** Matek H743-SLIM V3 running a modified ArduPilot. It
  sends collective and cyclic (Mx, My) commands, and the ESC generates the
  once-per-revolution pulsing.

## Layout

| Path | Contents |
|---|---|
| `ardupilot-hakea/` | ArduPilot fork (submodule) |
| `am32-hakea/` | AM32 fork (submodule) |
| `scripts/` | Test and log-analysis scripts |
| `notes/` | Test results, wiring, decisions |
| `params/` | Known-good parameter sets |
| `logs/` | MAVProxy and flight-controller logs (large logs are not committed) |

## Getting started

```bash
git clone --recursive https://github.com/fredgaffey/hakea.git
cd hakea
git submodule update --init --recursive   # if cloned without --recursive
```

Build the flight-controller firmware:

```bash
cd ardupilot-hakea
./waf configure --board MatekH743-bdshot
./waf copter --upload
```
