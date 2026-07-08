# Safety Shield

An output-stage command filter between a controller (`Mppi::Command()`, PID, teleop) and the actuator write. Sampling-based controllers treat constraints as soft penalties; the shield is the hard layer the technical note and the obstacle critic explicitly defer to ("safety-critical constraints need an output-stage shield on top"). It is a library stage, not a runtime node: the application calls `Filter()` once per control tick and forwards the result (ADR 0005 / algorithm-core composition).

## Scenarios (acceptance set)

These are the module's contract; each is implemented verbatim in `test_shield_scenarios.cpp`.

- **S1 — command spike.** The controller emits a step from 0 to well beyond the platform's acceleration ability. The issued command follows at the configured rate limit, per channel; the report marks the envelope active. No mode change.
- **S2 — invalid command.** The controller emits NaN. The shield holds the last safe command (`kHold`). If the fault clears within `hold_timeout`, operation resumes seamlessly (`kNormal`). If it persists, the shield ramps the held command to zero over `stop_ramp_time` (`kStopping`) and latches `kStopped`; only an explicit operator `Reset()` (with valid inputs) re-arms.
- **S3 — closing obstacle.** Commanded full speed toward an obstacle. The barrier filter attenuates the command as clearance shrinks so the platform never crosses the safety distance, and leaves the command untouched while far away (minimal intervention).
- **S4 — stale state.** The state estimate stops updating (`state_age` exceeds the configured maximum). Same ladder as S2: hold, then controlled stop. A state too old to trust is treated exactly like an invalid command.
- **S5 — quadruped GRF outside the friction cone** *(future, with the SRB shield)*: stance-foot forces are projected onto the cone before actuation.
- **S6 — e-stop from anywhere.** `TriggerEStop()` in any mode → `kStopped` (zero command) on the next tick, including mid-ramp. Guarded `Reset()` re-arms.

## Layers

1. **Command envelope** (`command_envelope.hpp`) — per-channel box clamps plus rate limits relative to the last *issued* command. Model-free, always on, exhaustively testable.
2. **Barrier filter** (`diff_drive_barrier.hpp`) — control-barrier-function constraint for planar circular obstacles using the look-ahead-point formulation for the unicycle (the CBF is relative-degree-1 in both `v` and `omega` at a point offset `l` ahead of the axle). Minimal command modification via a small QP (vendored QuadProg++, Goldfarb–Idnani): `min ||u − u_des||²` subject to `ḣ_i ≥ −α·h_i` per active obstacle plus the actuator box. A QP failure (e.g. started inside an obstacle) is reported and treated by the ladder as a fault — never silently passed through.
3. **Fallback ladder** (`fallback_ladder.hpp`) — `Normal → Hold → Stopping → Stopped` with an e-stop wildcard, built on the vendored `ctfsm` engine (its first in-tree consumer): the transition table is compile-time-verified, dispatch is allocation-free, and illegal events are refused rather than acted on.

`WheeledShield` (`wheeled_shield.hpp`) composes the three for a diff-drive platform and owns the per-tick sequencing.

## Semantics and assumptions

- **Units/frames:** commands are `[v (m/s), omega (rad/s)]`; state is `[x, y, theta]` in the world frame, `state_age` in seconds. All documented per header.
- **Validation at the boundary:** non-finite command or state, or `state_age > state_staleness_max`, is a fault. The hot path never throws; faults degrade through the ladder and are visible in the `ShieldReport` and telemetry.
- **Timing:** `Filter()` is deterministic and O(active obstacles); the QP solves a 2-variable problem with a bounded constraint set (`kMaxActiveObstacles` + box). Known limitation: QuadProg++ allocates internally per solve — acceptable at control rates (µs-scale, small problem), and the barrier only engages within its influence radius; replace with a preallocated solver if it ever shows in traces.
- **The shield reports everything it does:** `ShieldReport` per tick (mode, which layer engaged, minimum clearance) and telemetry (`control.shield.mode` gauge, `control.shield.faults` / `control.shield.barrier_active` counters). A shield that silently edits commands is an observability hole.
- **E-stop:** `TriggerEStop()` takes effect on the next `Filter()` call. It complements, never replaces, the hardware e-stop chain.
- **Recovery policy:** deliberately conservative — no automatic recovery from `kStopping`/`kStopped`; a human (or supervisor with equivalent authority) calls `Reset()`, which is guarded on inputs being valid again.

## References

- Ames et al., "Control Barrier Functions: Theory and Applications", ECC 2019 (the CBF-QP filter pattern).
- The look-ahead-point unicycle formulation is standard in CBF practice for differential-drive robots (relative-degree fix by output point offset).
