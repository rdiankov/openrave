###########################################
# mobyrave openrave plugin
###########################################
include(${CMAKE_CURRENT_SOURCE_DIR}/FindMoby.cmake)
if(MOBY_FOUND)
  message(STATUS "Building MobyRAVE")
  include_directories(${MOBY_INCLUDE_DIRS})
  link_directories(${OPENRAVE_LINK_DIRS} ${MOBY_INCLUDE_DIRS})
  add_library(mobyrave SHARED mobyrave.cpp mobyphysics.h mobyspace.h mobycontroller.h plugindefs.h)
  target_link_libraries(mobyrave libopenrave ${MOBY_LIBRARIES})
  target_link_libraries(mobyrave PRIVATE boost_assertion_failed)
  set_target_properties(mobyrave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")

  install(TARGETS mobyrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-mobyrave)
else()
  message(STATUS "Could not find Moby Physics engine. Please install from https://github.com/PositronicsLab/Moby ")
endif()
