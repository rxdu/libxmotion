# fsm (ctfsm) is a vendored family submodule, like planning/graph (libgraph):
# compile-time-verified FSM engine; tests/examples are upstream-gated.
# Install/export stays ON: exported consumers (shield) link ctfsm::ctfsm,
# so the target must belong to an installed export set.
set(CTFSM_INSTALL ON)
add_subdirectory(fsm)
add_subdirectory(models)
add_subdirectory(pid)
add_subdirectory(kinematics)
add_subdirectory(mppi)
add_subdirectory(shield)
