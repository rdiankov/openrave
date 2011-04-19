###########################################
# rmanipulation openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(dualmanipulation SHARED dualmanipulation.cpp dualmanipulation.h    plugindefs.h  dualcommonmanipulation.h )

# check boost regex
if( Boost_REGEX_FOUND )
  message(STATUS "boost regex found")
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DHAVE_BOOST_REGEX")
  target_link_libraries(dualmanipulation libopenrave ${Boost_REGEX_LIBRARY})
else()
  message(STATUS "failed to find boost regex, please install it")
  target_link_libraries(dualmanipulation libopenrave)
endif()

add_dependencies(dualmanipulation libopenrave)
set_target_properties(dualmanipulation PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS dualmanipulation DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-dualmanipulation)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-DUALMANIPULATION_DESCRIPTION "Dual manipulation functionality." PARENT_SCOPE)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-DUALMANIPULATION_DEPENDS ${COMPONENT_PREFIX}base PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-dualmanipulation PARENT_SCOPE)
