# Study: IHMC Open Robotics Software — design and engineering practices

**Date:** 2026-07-10
**Subject:** [ihmcrobotics/ihmc-open-robotics-software](https://github.com/ihmcrobotics/ihmc-open-robotics-software) (`develop` branch) — the field-proven Java humanoid stack (Atlas/Valkyrie at the DARPA Robotics Challenge; current reference robot `zulu`).
**Why studied:** the stack survived a decade of field deployment; its design, simulation-testing, and observability practices are language-independent and directly relevant to the XMotion validation-strategy and integration-test work. This document records the findings so decisions that reference them stay traceable.
**Sources:** main repo README and tree, `docs/Running Tests.md`, `.github/workflows/gradle-test-fast.yml` / `gradle-test-slow.yml`, module `build.gradle.kts` files, and the READMEs of `ihmc-yovariables`, `simulation-construction-set-2` (SCS2), `ihmc-commons`, `ihmc-build`, `ihmc-robot-data-logger`, `ihmc-realtime`.

## 1. Stack organization

The main repo is a Gradle composite build of ~25 subprojects: `ihmc-whole-body-controller`, `ihmc-common-walking-control-modules` (controller core: `WholeBodyControllerCore`, inverse-dynamics/virtual-model solvers), `ihmc-state-estimation`, `ihmc-sensor-processing`, `ihmc-footstep-planning`, `ihmc-path-planning`, `ihmc-perception`, `ihmc-communication`/`ihmc-interfaces` (ROS 2 message layer), `ihmc-avatar-interfaces` (robot-agnostic humanoid runtime: controller/estimator threads and tasks), `ihmc-simulation-toolkit`, and exactly one robot-specific module (`zulu`; historically Atlas/Valkyrie).

Foundations live in separate versioned repositories consumed as Maven artifacts: Euclid (geometry), Mecano (rigid-body dynamics), YoVariables, ihmc-commons, ihmc-realtime, SCS2. The dependency direction is strict: math/dynamics libraries → controller core → avatar-interfaces → robot module. Robot modules depend on everything; nothing depends on them.

Two build-system mechanisms stand out:

- **`source`-vs-pinned resolution** (ihmc-build plugin): a dependency declared as `api("us.ihmc:ihmc-footstep-planning:source")` resolves to a sibling source checkout when built inside a composite workspace (`compositeSearchHeight` in `gradle.properties`) and to a pinned published artifact otherwise. One declaration serves monorepo-style development and polyrepo consumption — no manual re-pinning during development.
- **Test code as a published artifact** (`extraSourceSets = ["test"]`): downstream robot modules depend on upstream *test* artifacts (e.g. `ihmc-common-walking-control-modules-test:source`) and instantiate abstract scenario suites against their own robot model.

## 2. Simulation-integration testing and test tiers

SCS2 is built around a **session** abstraction: the same GUI and variable buffer attach to a live simulation, a recorded log file, or a remote robot server (`scs2-definition` → `scs2-session` → `scs2-simulation` / `scs2-session-logger`; `scs2-shared-memory` holds the YoVariable buffer for playback). Controller code is identical between simulation and hardware because the controller runs behind sensor-reader/output-writer interfaces — the simulation output writer is just one implementation, and multiple physics backends are pluggable (`scs2-bullet-simulation`, `scs2-mujoco-simulation`).

Test tiers are JUnit 5 `@Tag`s selected via `gradle test -Pcategory=...` (the old `@ContinuousIntegration` duration annotations are gone; categories are now scenario-named):

- **PR CI** (`gradle-test-fast.yml`): `fast` per module, plus simulation suites on the reference robot — `controller-api`, `humanoid-flat-ground`, `humanoid-flat-ground-bullet` (the same scenario under a second physics engine), `humanoid-obstacle`, `humanoid-push-recovery`, `humanoid-rough-terrain`, `humanoid-toolbox`.
- **Nightly** (`gradle-test-slow.yml`, cron): `allocation-slow`, `controller-api-slow-*`, `humanoid-stairs-slow`, `footstep-planning-slow`, `gui-slow`, with sharded job suffixes (`-2`, `-3`, `-4`) for parallelism and `requires-self-hosted: true` for heavy/GPU jobs.

Determinism is a first-class regression: `DRCFlatGroundRewindabilityTest` runs a simulation, rewinds the variable buffer, re-simulates, and compares **every** registered variable, sitting alongside the walking tests in `ihmc-avatar-interfaces`.

## 3. YoVariable observability

YoVariables are named typed variables (`YoDouble/YoBoolean/YoInteger/YoLong/YoEnum`) registered in a hierarchical `YoRegistry` mirroring the module structure. Design goals (quoted from the README): "retrieve any control variable by name", "list the variables declared by an algorithm", "observe their value changing over time and have access to the variable history", tune parameters dynamically. Key elements:

- `YoBuffer`: in-memory per-tick history of every registered variable.
- `YoParameter`: read-only to the algorithm, writable by the tuning UI — live tuning with a safe direction of authority.
- Filtered variants (`AlphaFilteredYoVariable`, `RateLimitedYoVariable`, `GlitchFilteredYoBoolean`): even signal filters are inspectable state.
- `YoComposites` group variables into semantic units (tuple, quaternion); `YoGraphics` render visualizations directly from variables.
- On the robot, `ihmc-robot-data-logger` streams every control tick plus synchronized camera video to disk ("100 GB/hour easily"); logs open in the same SCS2 GUI used for simulation.

Net effect: every field incident is a scrubbable black-box recording with full controller internals, using the same tooling loop for development, testing, and field debugging. This is widely credited as a key ingredient of their DRC success.

## 4. Canonical scenarios and models

One reference robot instantiates abstract avatar test suites: flat-ground walking (forward / quick / side-stepping / step-in-place / pause), the DRC obstacle course, push recovery, rough-terrain walking, stairs, state-estimation end-to-end tests, controller-API tests. Test assertions are written against YoVariables (e.g. pelvis height) via `AvatarTestYoVariables` — not ad-hoc probes.

## 5. Real-time practices (in Java, of all things)

`ihmc-realtime` provides JNI POSIX RT threads on RT_PREEMPT and lock-free inter-thread queues. ihmc-commons provides recycling/preallocated collections and allocation-testing tools (built on a JVM allocation instrumenter). Allocation-freedom of the control loops is a **nightly CI category** (`allocation-slow`), not a code-review convention.

## 6. Transferable recommendations → XMotion mapping

| # | Practice (source) | XMotion mapping | Status / priority |
|---|---|---|---|
| 1 | Allocation testing as a CI category (`allocation-slow`, ihmc-commons instrumenter) | malloc-hook counter around steady-state `Mppi::Plan()`, `WheeledShield::Filter()`, MEKF/PID `Update()`; `allocation` ctest label in the nightly workflow | **adopt now** — proves anti-pattern #4 instead of asserting it in review |
| 2 | Rewindability regression (`DRCFlatGroundRewindabilityTest`) | run → rewind `SimLog` → re-simulate → diff recorded state | **adopt now** — extends existing determinism asserts |
| 3 | Scenario-named tags, PR-fast/nightly-slow, sharded jobs, self-hosted for heavy sim (`gradle-test-*.yml`) | ctest labels already split `integration`/`campaign`; adopt scenario names as labels; self-hosted runner answers the CUDA-in-CI question | partially in place (nav PR #69) |
| 4 | YoVariable registry + `YoParameter` + per-tick buffer | evolution of the xmBase telemetry plane: hierarchical named variables, parameter direction-of-authority (the MPPI tuner's atomics are an ad-hoc instance), per-tick recording; test assertions against named variables | fold into the estimation-round-2 / telemetry arc |
| 5 | Session abstraction: one viewer for live sim / log / remote robot (SCS2 sessions) | quickviz roadmap: viewer sessions over MCAP — "the log format is the interface" (matches the established converters-first insight-plane direction) | quickviz roadmap item |
| 6 | `source`-vs-pinned dependency resolution (ihmc-build) | CMake workspace-overlay option in the umbrella preferring sibling checkouts over vendored pins; would cut re-pin churn during multi-repo arcs | write up as umbrella ADR proposal before implementing |
| 7 | Test suites shipped as artifacts, abstract scenarios instantiated per robot (`extraSourceSets`, `Avatar*Test`) | template the integration scenarios over model/config; instantiate per platform when a second robot config exists | deferred until a second platform |
| 8 | Cross-engine simulation validation (`humanoid-flat-ground` vs `-bullet`) | run key scenarios under a second physics backend (e.g. MuJoCo) when one is integrated | deferred |

## 7. Caveats

- IHMC is a humanoid-centric framework with a runtime tier (avatar threads/tasks); XMotion deliberately rejected the runtime tier (algorithm core, ADR/decision history). The practices above transfer at the level of build, test, and observability infrastructure — not the framework architecture.
- Claims are based on repository inspection at the date above; module names and workflow files move. Re-verify paths before citing them in implementation PRs.
