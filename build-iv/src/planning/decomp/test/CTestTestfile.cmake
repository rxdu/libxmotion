# CMake generated Testfile for 
# Source directory: /home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/src/planning/decomp/test
# Build directory: /home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/build-iv/src/planning/decomp/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
include("/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/build-iv/src/planning/decomp/test/utest_decomp[1]_include.cmake")
add_test(planning::decomp "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/build-iv/bin/utest_decomp")
set_tests_properties(planning::decomp PROPERTIES  _BACKTRACE_TRIPLES "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/src/planning/decomp/test/CMakeLists.txt;8;add_test;/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/src/planning/decomp/test/CMakeLists.txt;0;")
subdirs("devel")
