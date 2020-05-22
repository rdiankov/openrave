###################################
# posture describer openrave plugin
###################################

set(POSTUREDESCRIBER_CLASS_NAME "posturedescriber")
set(POSTUREDESCRIBER_MODULE_NAME "posturedescriber")

configure_file(plugindefs.h.in
			   "${CMAKE_CURRENT_SOURCE_DIR}/plugindefs.h"
			   @ONLY
)

if( Boost_IOSTREAMS_FOUND )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DBoost_IOSTREAMS_FOUND")
endif()
if( OPT_IKFAST_FLOAT32 )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DOPENRAVE_IKFAST_FLOAT32")
endif()

file(GLOB posturedescriber_cppfiles "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB posturedescriber_headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
add_library(posturedescriber SHARED "${posturedescriber_cppfiles}")

if( Boost_IOSTREAMS_FOUND )
  target_link_libraries(posturedescriber libopenrave ${Boost_IOSTREAMS_LIBRARY} ${LAPACK_LIBRARIES})
else()
  target_link_libraries(posturedescriber libopenrave ${LAPACK_LIBRARIES})
endif()

target_link_libraries(posturedescriber PRIVATE boost_assertion_failed)
set_target_properties(posturedescriber PROPERTIES 
	COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -fvisibility=hidden"
	LINK_FLAGS "${PLUGIN_LINK_FLAGS}"
)

install(TARGETS posturedescriber DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
