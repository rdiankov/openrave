###########################################
# rmanipulation openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(dmanipulation SHARED dualmanipulationmain.cpp dualmanipulation.h    plugindefs.h  commonmanipulation.h )

# check boost regex
if( Boost_REGEX_FOUND )
  message(STATUS "boost regex found")
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DHAVE_BOOST_REGEX")
  target_link_libraries(dmanipulation libopenrave ${Boost_REGEX_LIBRARY})
else()
  message(STATUS "failed to find boost regex, please install it")
  target_link_libraries(dmanipulation libopenrave)
endif()

include_directories(dmanipulation "${CMAKE_SOURCE_DIR}")
add_dependencies(dmanipulation libopenrave)
set_target_properties(dmanipulation PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                       LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS dmanipulation DESTINATION share/openrave/plugins )
