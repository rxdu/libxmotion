<h1 align="center">
  <img src="docs/xmnabla.svg" width="96" alt="xmNabla"><br>
  xmNabla&nbsp;·&nbsp;∇
</h1>

<p align="center"><b>Motion algorithms for mobile robots</b> — planning, control, state estimation and mapping.<br>
The ∇ centerpiece of the <a href="https://github.com/rxdu/xmotion">xMotion</a> family.</p>

---

`xmNabla` is the motion-algorithms core of the **xMotion** product family: trajectory and path
planning, feedback and optimal control, state estimation, and mapping. It consolidates code that
was previously spread across several repositories (notably `librav` and `imtoolkit`) and, most
recently, lived in `libxmotion`. At the moment it is used mainly for study, research and
experimentation — not yet production-hardened.

> **Transition status (Phase 1).** This repo was renamed from `libxmotion` and still vendors
> `src/common` and `src/driver`, which are being extracted into sibling components
> **[xmSigma](https://github.com/rxdu/xmSigma)** (foundation/common) and
> **[xmMu](https://github.com/rxdu/xmMu)** (host hardware drivers). Until the Phase-2 decoupling,
> `xmNabla` builds standalone exactly as before. See the umbrella's
> [transition ADR](https://github.com/rxdu/xmotion/blob/main/docs/adr/0002-repo-transition-plan.md).

## Repository structure

| Folder      | Description           |
|-------------|-----------------------|
| cmake       | cmake configuration   |
| data        | maps, results, logs   |
| docs        | documentation         |
| python      | Python code           |
| scripts     | bash scripts          |
| src         | C++ code              |
| third_party | third-party libraries |

## Build and run

#### Compiler requirements

* C++11

#### Install dependencies

```
$ sudo apt-get install -y libgl1-mesa-dev \
   libglfw3-dev libcairo2-dev libtbb-dev libasio-dev libboost-all-dev \
   libgsl-dev libeigen3-dev libtbb-dev libopencv-dev libyaml-cpp-dev \
   libncurses-dev libevdev-dev libmodbus-dev libpcl-dev libglm-dev
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

* `XLOG_LEVEL`: 0–6 (0: TRACE, 1: DEBUG, 2: INFO, 3: WARN, 4: ERROR, 5: FATAL, 6: OFF)
* `XLOG_ENABLE_LOGFILE`: 0 or 1
* `XLOG_FOLDER`: folder for log files (default `~/.xmotion/log`)

## License

Apache-2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE). First-party code only; bundled
third-party components retain their own licenses.
