# Backlog — deferred and dormant

Active near-term work is tracked in the root [TODO.md](../TODO.md). Items here are recorded but not scheduled; each is gated on a revival decision or an external trigger.

## Road-network revival (gates the state_lattice apps)

- Re-enable `state_lattice/apps` — `gen_lookup_table` depends on the retired `traffic_map` module (src/planning/state_lattice/CMakeLists.txt)
- Fix CSV loading in `LookupTable::LoadLookupTableFromFile` (src/planning/state_lattice/src/lookup_table.cpp)
- Make state_lattice data locations explicit configuration instead of path probing (src/planning/state_lattice/src/data_path.hpp)
- OSM-defined paths, traffic sim, and Monte-Carlo AV scenarios (carried over from the pre-reorg backlog; only meaningful after the revival)

## Decision

- Verify `prediction` threat-model behavior and re-enable it in the build (src/decision/CMakeLists.txt; static_threat_model.hpp / vehicle_threat.hpp carry "not sure if behavior is correct" markers)
- Record reachability Monte-Carlo samples through telemetry (XM_* / MCAP) (src/decision/reachability/src/monte_carlo_sim.cpp)

## Geometry

- Reimplement the stubbed polygon predicates (bounded-side, intersection, optimal convex partition) without CGAL (src/planning/geometry/src/polygon.cpp)

## Control

- Preallocated QP solver for the safety shield — only if QuadProg++ per-solve allocation ever shows in traces (docs/control/safety_shield.md)

## Deferred until a second platform exists

- Per-platform template integration scenarios
- Cross-engine simulation validation vs MuJoCo

---

The historical checklist previously in this file (LCM cleanup, RC-Car Kalman filter and LCM/FastRTPS sim interface, CGAL removal, kernel leak checks, uavcan/Gurobi items) was either completed or superseded by the estimation (MEKF6/9), simulation-tier, and telemetry arcs; see git history for the old list.
