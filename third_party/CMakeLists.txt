cmake_minimum_required(VERSION 2.8.11)
project(srcl_3rd_party_lib)

# Check if g3log needs to be built
if(ENABLE_G3LOG)
	add_subdirectory(g3log_srcl)
	message( "-- Enable logging with g3log -- Yes" )
	message( "-- Deafult log directory: srcl_ctrl/control/log " )
else(ENABLE_G3LOG)
	message( "-- Enable logging with g3log -- No" )
endif(ENABLE_G3LOG)

## Add sub source directories
#add_subdirectory(eigen)
add_subdirectory(octomap)
