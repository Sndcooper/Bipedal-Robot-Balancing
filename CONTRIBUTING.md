# Contributing to PAW 🐾

Thanks for building on PAW! Bug fixes, new features, ports to other boards, and papers that use it are all welcome.

## Where to work

- **Main code:** [`robot_control_suite/rcbfw/`](robot_control_suite/rcbfw/README.md). Please target this variant unless you have a specific reason not to.
- **Other variants** are earlier stages or bench tools. See [`robot_control_suite/README.md`](robot_control_suite/README.md).
- **Ideas and open problems** are in [`docs/PAW_Research_Notes.md`](docs/PAW_Research_Notes.md). The "Findings" and "Roadmap" sections are good first issues.

## Rules that prevent most breakage

1. **The firmware and GUI are a matched pair.** Any change to a command or telemetry field must update both `firmware/src/main.cpp` and `gui/serial_link.py` in the same PR.
2. **Multi-character commands go above single-letter ones** in `parseCommand()`. For example, `RVS` must be checked before `RV`, which must be checked before `R`.
3. **At most one Serial2 (AX-12) transaction per 100 Hz tick.** Only the bus arbiter in `loop()` may write to the bus.
4. **Nothing blocking in `loop()`**: no `delay()`, busy-waits, or unguarded prints. Every UART write is guarded with `availableForWrite()`.
5. **Don't weaken the safety paths** (tilt cutoff, RC failsafe, arm interlock) without discussing it in an issue first.
6. **Keep folder names short.** The Windows build path limit is 260 characters and is already tight (see the `bfrc` README).

## Before opening a PR

```powershell
pio run -d "robot_control_suite/rcbfw/firmware"          # must build
python -m py_compile robot_control_suite/rcbfw/gui/*.py   # must parse
```

- Test on hardware with the **wheels off the ground** first, and say in the PR what you tested.
- If you retuned gains, attach the session log (`gui/logs/`) or plots that justify the change.
- Keep one topic per PR and write commit messages in the imperative mood (for example, "Add DLPF config to setupMPU").

## Reporting bugs

Open an issue with the firmware variant, a description of what happened, and the relevant `gui/logs/*.log` excerpt. Logs are the fastest route to a fix.
