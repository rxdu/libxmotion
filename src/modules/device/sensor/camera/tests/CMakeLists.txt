# Dependency libraries
find_package(OpenCV REQUIRED)

# Add executables
add_executable(test_capture test_capture.c)
target_link_libraries(test_capture -lpthread  -lm ${OpenCV_LIBS})

add_executable(test_imx6sx_vadc imx6sx_vadc_pxp.c)
target_link_libraries(test_imx6sx_vadc -lpxp -lpthread -lm ${OpenCV_LIBS})
