###########################################
# ikfastsolvers openrave plugin
###########################################
if( Boost_IOSTREAMS_FOUND )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DBoost_IOSTREAMS_FOUND")
endif()
if( OPT_IKFAST_FLOAT32 )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DOPENRAVE_IKFAST_FLOAT32")
endif()

file(GLOB ik_files "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../python) # for ikfast.h
add_library(ikfastsolvers SHARED ikfastsolvers.cpp ikfastmodule.cpp ikfastsolver.cpp plugindefs.h ${CMAKE_CURRENT_SOURCE_DIR}/../../python/ikfast.h ${ik_files})
if( Boost_IOSTREAMS_FOUND )
  target_link_libraries(ikfastsolvers libopenrave ${Boost_IOSTREAMS_LIBRARY})
else()
  target_link_libraries(ikfastsolvers libopenrave)
endif()
set_target_properties(ikfastsolvers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS ikfastsolvers DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-ikfastsolvers)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-IKFASTSOLVERS_DISPLAY_NAME "Plugin interfacing to IKFast, the kinematics robot compiler" PARENT_SCOPE)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-IKFASTSOLVERS_DEPENDS ${COMPONENT_PREFIX}ikfast)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-ikfastsolvers PARENT_SCOPE)
