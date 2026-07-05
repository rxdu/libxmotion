include(CMakeFindDependencyMacro)

# Re-discover the dependencies that xmBase's public (imported) targets reference,
# so downstream find_package(xmBase) consumers get a usable xmotion::xmBase
# target. Consumers link xmotion::xmBase.
find_dependency(Threads REQUIRED)
find_dependency(Eigen3 REQUIRED NO_MODULE)   # xmotion::xmBase links Eigen3::Eigen

# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/xmBaseTargets.cmake")
