###########################################
# basemanipulation openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(basemanipulation SHARED basemanipulationmain.cpp)

# check boost regex
if( Boost_REGEX_FOUND )
  message(STATUS "boost regex found")
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DHAVE_BOOST_REGEX")
  target_link_libraries(basemanipulation libopenrave ${Boost_REGEX_LIBRARY})
else()
  message(STATUS "failed to find boost regex, please install it")
  target_link_libraries(basemanipulation libopenrave)
endif()

include_directories(basemanipulation "${CMAKE_SOURCE_DIR}")
add_dependencies(basemanipulation libopenrave libopenrave_static)
set_target_properties(basemanipulation PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                       LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS basemanipulation DESTINATION share/openrave/plugins )
