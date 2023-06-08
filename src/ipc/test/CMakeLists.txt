#add_executable(test_lcm_publisher lcm_publisher.cpp)
#target_link_libraries(test_lcm_publisher ${LCM_NAMESPACE}lcm ipc_lcmtypes-cpp)
#
#add_executable(test_lcm_subscriber lcm_subscriber.cpp)
#target_link_libraries(test_lcm_subscriber ${LCM_NAMESPACE}lcm ipc_lcmtypes-cpp)

find_package(CycloneDDS REQUIRED)

message(STATUS "Cyclone include: ${CycloneDDS_INCLUDE_DIRS}")

idlc_generate(TARGET HelloWorldData_lib FILES "HelloWorldData.idl" WARNINGS no-implicit-extensibility)
#idlc_generate(TARGET HelloWorldData_lib FILES "HelloWorldData.idl")

add_executable(test_dds_pub test_dds_pub.c)
target_link_libraries(test_dds_pub HelloWorldData_lib CycloneDDS::ddsc)

add_executable(test_dds_sub test_dds_sub.c)
target_link_libraries(test_dds_sub HelloWorldData_lib CycloneDDS::ddsc)