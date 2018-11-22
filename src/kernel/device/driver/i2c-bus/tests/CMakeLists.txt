# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(read_whoami read_whoami.cpp)
target_link_libraries(read_whoami i2cbus)

add_executable(test_i2cwrite test_i2cwrite.cpp)
target_link_libraries(test_i2cwrite i2cbus)