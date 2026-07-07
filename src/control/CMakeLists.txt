# fsm (ctfsm) is a vendored family submodule, like planning/graph (libgraph):
# compile-time-verified FSM engine; tests/examples/install are upstream-gated
add_subdirectory(fsm)
add_subdirectory(model)
add_subdirectory(pid)
add_subdirectory(kinematics)
add_subdirectory(mppi)
