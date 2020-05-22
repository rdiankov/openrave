###################################
# posture describer openrave plugin
###################################

set(POSTUREDESCRIBER_CLASS_NAME "posturedescriber")
set(POSTUREDESCRIBER_MODULE_NAME "posturedescriber")

configure_file(plugindefs.h.in
			   "${CMAKE_CURRENT_SOURCE_DIR}/plugindefs.h"
			   @ONLY
)

file(GLOB posturedescriber_cppfiles "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(posturedescriber SHARED "${posturedescriber_cppfiles}")

target_link_libraries(posturedescriber PUBLIC libopenrave PRIVATE boost_assertion_failed)
set_target_properties(posturedescriber PROPERTIES
	COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -fvisibility=hidden"
	LINK_FLAGS "${PLUGIN_LINK_FLAGS}"
)

install(TARGETS posturedescriber
	DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR}
	COMPONENT ${PLUGINS_BASE}
)
