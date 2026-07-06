<h1 align="center">
  <img src="docs/xmnavigation.svg" width="96" alt="xmNavigation"><br>
  xmNavigation&nbsp;·&nbsp;∇
</h1>

<p align="center"><b>Motion algorithms for mobile robots</b> — planning, control, state estimation and mapping.<br>
The ∇ centerpiece of the <a href="https://github.com/rxdu/xmotion">XMotion</a> family.</p>

---

`xmNavigation` is the motion-algorithms core of the **XMotion** product family: trajectory and path
planning, feedback and optimal control, state estimation, and mapping. It consolidates code that
was previously spread across several repositories (notably `librav` and `imtoolkit`) and, most
recently, lived in `libxmotion`. At the moment it is used mainly for study, research and
experimentation — not yet production-hardened.

> **Algorithm-centric.** `xmNavigation` is a pure algorithms library: it depends only on
> **[xmBase](https://github.com/rxdu/xmBase)** (foundation/common) and math libraries — no hardware
> dependencies. Composing algorithms with drivers into runnable robot applications happens in
> external application repos. See the umbrella's
> [transition ADR](https://github.com/rxdu/xmotion/blob/main/docs/adr/0002-repo-transition-plan.md).

## Repository structure

The source tree is organized around the five domain areas of a navigation stack, plus two
supporting tiers:

| Module               | Description                                                        |
|----------------------|--------------------------------------------------------------------|
| `src/estimation`     | state estimation (MEKF attitude/IMU filters)                       |
| `src/mapping`        | world representation (occupancy/PGM maps, point clouds)            |
| `src/decision`       | prediction & decision-making (markov models, reachability)         |
| `src/planning`       | motion planning (geometry, space decomposition, sampling, lattice) |
| `src/control`        | control laws & models (PID, FSM, dynamics models, kinematics)      |
| `src/types`          | shared navigation-stack vocabulary (the inter-area contract types) |
| `src/visualization`  | drawing for all of the above — the only module linking a renderer  |

Supporting folders: `cmake` (build configuration), `data` (test fixtures: maps, lookup tables),
`docs` (documentation), `python` (analysis scripts), `third_party` (vendored dependencies —
all permissively licensed).

## Build and run

#### Compiler requirements

* C++17

#### Install dependencies

```
$ sudo apt-get install -y libgl1-mesa-dev \
   libglfw3-dev libcairo2-dev libtbb-dev libboost-all-dev \
   libeigen3-dev libopencv-dev libyaml-cpp-dev \
   libncurses-dev libpcl-dev libglm-dev
```

#### Compile

```
$ mkdir build && cd build
$ cmake ..
$ make -j
```

**Note:** if you hit build issues, refer to the CI workflow under `.github/workflows/` for the
up-to-date steps.

#### Logging configuration

Logging goes through the xmBase telemetry API (`XM_*` macros). Without a telemetry SDK bound at
runtime, events at Warn severity and above go to stderr; the console binding's minimum severity can
be seeded with the `XM_LOG_LEVEL` environment variable.

## License

Apache-2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE). First-party code only; bundled
third-party components retain their own licenses.
