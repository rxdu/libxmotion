# TODO

Active near-term work. Status: `[ ]` open · `[~]` in progress · `[x]` done. Deferred and dormant items live in [docs/TODO.md](docs/TODO.md).

## Verification & CI (IHMC study "adopt now" items — docs/research/ihmc-open-robotics-software.md §6)

- [x] Allocation-testing category: malloc-hook counters around steady-state `Mppi::Plan()`, `WheeledShield::Filter()`, and MEKF/PID `Update()`; nightly `allocation` ctest label
- [x] Rewindability regression: run → rewind `SimLog` → re-simulate → diff-to-zero
- [ ] Self-hosted CUDA runner so the CUDA rollout backends build and test in CI

## Estimation

- [ ] Golden-log regressions: replay real IMU MCAP recordings against MEKF6/MEKF9 (blocked: pending hardware data — docs/typst/main.typ)

## Control / MPPI

- [ ] Per-sample counter-seeded noise streams on the CPU rollout backend (lifts the sampling Amdahl cap — docs/typst/main.typ § MPPI)
- [ ] On-device spline-knot sampling for the SRB CUDA program
- [ ] Jetson Orin deployment pass (unified memory: pinned staging maps zero-copy)
- [ ] Safety shield S5: quadruped GRF friction-cone projection, with the SRB shield (docs/control/safety_shield.md)

## Telemetry & visualization

- [ ] Telemetry evolution: hierarchical named variables + per-tick recording
- [ ] Viewer sessions over MCAP (quickviz roadmap)

## Docs & process

- [ ] Umbrella ADR: workspace-overlay dependency resolution
- [ ] Formalize (or retire) the W#/M# milestone labels in a committed roadmap doc — today they exist only in branch/commit names
