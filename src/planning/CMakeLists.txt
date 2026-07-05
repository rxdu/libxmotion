## Add sub source directories
if (XMOTION_WERROR)
  # graph (libgraph) is a vendored submodule: keep warnings visible, never gate.
  get_directory_property(_xm_opts COMPILE_OPTIONS)
  list(REMOVE_ITEM _xm_opts -Werror)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_xm_opts}")
  add_subdirectory(graph)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_xm_opts};-Werror")
  unset(_xm_opts)
else ()
  add_subdirectory(graph)
endif ()
add_subdirectory(geometry)
add_subdirectory(decomp)

# under revision
add_subdirectory(sampling)

# add_subdirectory(state_lattice)
# add_subdirectory(decision)
