###########################################
# rmanipulation openrave plugin
###########################################
add_library(rmanipulation SHARED rmanipulation.cpp basemanipulation.cpp    plugindefs.h  taskmanipulation.cpp commonmanipulation.h  taskcaging.cpp  visualfeedback.cpp)

# check boost regex
if( Boost_REGEX_FOUND )
  message(STATUS "boost regex found")
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DHAVE_BOOST_REGEX")
  target_link_libraries(rmanipulation libopenrave ${Boost_REGEX_LIBRARY})
else()
  message(STATUS "failed to find boost regex, please install it")
  target_link_libraries(rmanipulation libopenrave)
endif()

set_target_properties(rmanipulation PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rmanipulation DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
