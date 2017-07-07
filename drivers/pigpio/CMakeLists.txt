############################################################################
#
# Copyright (c) 2017 LAOSAAC
#
############################################################################

include(../../cmake/df_common.cmake)

include_directories(../../framework/include)

get_filename_component(drivername ${CMAKE_CURRENT_SOURCE_DIR} NAME)
df_add_library(df_${drivername}
	pigpio.c
	command.c
	)

target_link_libraries(df_${drivername}
	rt)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
