# add_subdirectory(benchmark)

file( GLOB cfgs "*.cfg" )
file( GLOB worlds "*.world" )
file( GLOB incs "*.inc" )
file( GLOB scripts "*.sh" )

install(FILES ${cfgs} ${worlds} ${incs} ${scripts}
DESTINATION share/stage/worlds)

install (DIRECTORY bitmaps wifi DESTINATION share/stage/worlds)

# add_subdirectory(bitmaps)
# add_subdirectory(wifi)

#file(GLOB worlds "*.world")
#file(GLOB incs "*.inc")
#install(FILES ${worlds} ${incs}
#        DESTINATION share/stage/worlds
#)

