## Dependency libraries
find_package(Eigen3 REQUIRED NO_MODULE)

## MPPI is a header-only templated controller (model/cost/sampler seams
## resolve at compile time so the rollout loop fully inlines).
add_library(mppi INTERFACE)
target_link_libraries(mppi INTERFACE xmotion::xmBase Eigen3::Eigen)
target_include_directories(mppi INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

## CUDA rollout backend (optional): float32 device rollouts implementing
## the same backend seam; the wheeled program reuses the model/critic cores.
if (XMNAV_WITH_CUDA)
  enable_language(CUDA)
  if (NOT DEFINED CMAKE_CUDA_STANDARD)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  endif ()
  add_library(mppi_cuda STATIC
      src/cuda_rollout_backend.cpp
      src/cuda_rollout_backend.cu)
  target_link_libraries(mppi_cuda PUBLIC mppi)
  # the host .cpp uses the CUDA runtime API directly
  target_include_directories(mppi_cuda PRIVATE
      ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
  # sm_75 = GTX 16xx/RTX 20xx (dev machine), sm_87 = Jetson Orin
  set_target_properties(mppi_cuda PROPERTIES
      CUDA_ARCHITECTURES "75;87"
      POSITION_INDEPENDENT_CODE ON)
  add_library(xmotion::mppi_cuda ALIAS mppi_cuda)

  install(TARGETS mppi_cuda
      EXPORT xmNavigationTargets
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include)
endif ()

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS mppi
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
