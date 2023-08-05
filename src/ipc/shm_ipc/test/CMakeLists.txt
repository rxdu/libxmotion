find_package(iceoryx_posh  2.0.3 REQUIRED)
find_package(iceoryx_hoofs 2.0.3 REQUIRED)

message(STATUS "Using iceoryx_posh version ${iceoryx_posh_VERSION}")
message(STATUS "Using iceoryx_hoofs version ${iceoryx_hoofs_VERSION}")

get_target_property(ICEORYX_CXX_STANDARD iceoryx_posh::iceoryx_posh CXX_STANDARD)
include(IceoryxPlatform)

add_executable(test_iox_publisher test_iox_publisher.cpp)
target_link_libraries(test_iox_publisher iceoryx_posh::iceoryx_posh iceoryx_hoofs::iceoryx_hoofs)
target_compile_options(test_iox_publisher PRIVATE ${ICEORYX_WARNINGS} ${ICEORYX_SANITIZER_FLAGS})

add_executable(test_iox_subscriber test_iox_subscriber.cpp)
target_link_libraries(test_iox_subscriber iceoryx_posh::iceoryx_posh iceoryx_hoofs::iceoryx_hoofs)
target_compile_options(test_iox_subscriber PRIVATE ${ICEORYX_WARNINGS} ${ICEORYX_SANITIZER_FLAGS})

set_target_properties(test_iox_publisher test_iox_subscriber
        PROPERTIES
        CXX_STANDARD_REQUIRED ON
        CXX_STANDARD ${ICEORYX_CXX_STANDARD}
        POSITION_INDEPENDENT_CODE ON
)