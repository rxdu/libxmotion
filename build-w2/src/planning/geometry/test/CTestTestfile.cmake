# CMake generated Testfile for 
# Source directory: /home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/src/planning/geometry/test
# Build directory: /home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/build-w2/src/planning/geometry/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
include("/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/build-w2/src/planning/geometry/test/utest_geometry[1]_include.cmake")
add_test(planning::geometry "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/build-w2/bin/utest_geometry")
set_tests_properties(planning::geometry PROPERTIES  _BACKTRACE_TRIPLES "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/src/planning/geometry/test/CMakeLists.txt;8;add_test;/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/src/planning/geometry/test/CMakeLists.txt;0;")
subdirs("devel")
