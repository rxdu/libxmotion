## Add source directories
# Modules
add_subdirectory(common)
#add_subdirectory(mapping)
add_subdirectory(estimation)
add_subdirectory(control)
add_subdirectory(planning)

if (ENABLE_VISUALIZATION)
  option(QUICKVIZ_DEV_MODE "Build quickviz in dev mode" ON)
  option(BUILD_QUICKVIZ_APP "Build quickviz application" ON)
  if (XMOTION_WERROR)
    # quickviz is vendored: keep -Wall/-Wextra visible but never gate on it.
    get_directory_property(_xm_opts COMPILE_OPTIONS)
    list(REMOVE_ITEM _xm_opts -Werror)
    set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_xm_opts}")
    add_subdirectory(quickviz)
    set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_xm_opts};-Werror")
    unset(_xm_opts)
  else ()
    add_subdirectory(quickviz)
  endif ()
endif ()
