## Add source directories
# Modules. types/estimation/control/simulation are always built (the control +
# estimation foundation). The higher planning-stack tiers are opt-in (see the
# XMNAV_WITH_* options) so a control-only consumer configures just what it uses.
add_subdirectory(types)
if (XMNAV_WITH_MAPPING)
  add_subdirectory(mapping)
endif ()
if (XMNAV_WITH_DECISION)
  add_subdirectory(decision)
endif ()
add_subdirectory(estimation)
add_subdirectory(control)
if (XMNAV_WITH_PLANNING)
  add_subdirectory(planning)
endif ()
add_subdirectory(simulation)

if (ENABLE_VISUALIZATION)
  add_subdirectory(visualization)
endif ()
