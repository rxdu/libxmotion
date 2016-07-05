cmake_minimum_required(VERSION 2.8.11)
project(srcl_3rd_party_lib)

## Check if g3log needs to be built
if(ENABLE_G3LOG)
	add_subdirectory(g3log_srcl)
endif(ENABLE_G3LOG)


## Check if OMPL can be built
if(Boost_FOUND)
	#message(STATUS "OMPL will be built.")
	#add_subdirectory(ompl-1.2.0)
elseif(Boost_FOUND)
	message(STATUS "Boost 1.54 or higher is not found. OMPL will not be built.")
endif(Boost_FOUND)

## Add sub source directories
#add_subdirectory(eigen)
add_subdirectory(octomap-1.8.0)
add_subdirectory(smp_karaman)
