# fsm (ctfsm) is a vendored family submodule, like planning/graph (libgraph):
# compile-time-verified FSM engine; tests/examples are upstream-gated.
# Install/export stays ON: exported consumers (shield) link ctfsm::ctfsm,
# so the target must belong to an installed export set.
#
# Guard against double provision: an application that also consumes the
# standalone ctfsm (e.g. as its own submodule) defines `ctfsm` before adding
# xmNavigation. In that case reuse the parent's target — adding our vendored
# copy would clash on the `ctfsm` target name. (The parent is then responsible
# for exporting ctfsm; ours does so via CTFSM_INSTALL when we own it.)
if (NOT TARGET ctfsm)
  set(CTFSM_INSTALL ON)
  add_subdirectory(fsm)
endif ()
add_subdirectory(models)
add_subdirectory(pid)
add_subdirectory(kinematics)
add_subdirectory(mppi)
add_subdirectory(shield)
