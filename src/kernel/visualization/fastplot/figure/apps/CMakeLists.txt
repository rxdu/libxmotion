cmake_minimum_required(VERSION 2.8)

add_executable(figure_logscaling ./1-LogScaling/main.cpp )
target_link_libraries(figure_logscaling figure)

add_executable(figure_codingeffortcomparison ./2-CodingEffortComparison/main.cpp)
target_link_libraries(figure_codingeffortcomparison figure)

add_executable(figure_3dplot ./3-3dPlot/main.cpp)
target_link_libraries(figure_3dplot figure)

add_executable(figure_nonposlogdata ./4-NonPosLogData/main.cpp)
target_link_libraries(figure_nonposlogdata figure)

add_executable(figure_fplot ./5-FPlot/main.cpp)
target_link_libraries(figure_fplot figure)

add_executable(figure_autolinestyles ./6-AutoLineStyles/main.cpp)
target_link_libraries(figure_autolinestyles figure)

add_executable(figure_spy ./7-Spy/main.cpp)
target_link_libraries(figure_spy figure)

add_executable(figure_triplot ./8-Triplot/main.cpp)
target_link_libraries(figure_triplot figure)

add_executable(figure_bar ./9-Bar/main.cpp)
target_link_libraries(figure_bar figure)
