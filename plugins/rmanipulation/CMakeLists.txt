###########################################
# rmanipulation openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(rmanipulation SHARED rmanipulation.cpp basemanipulation.h    plugindefs.h  taskmanipulation.h commonmanipulation.h  taskcaging.h  visualfeedback.h)

# check boost regex
if( Boost_REGEX_FOUND )
  message(STATUS "boost regex found")
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DHAVE_BOOST_REGEX")
  target_link_libraries(rmanipulation libopenrave ${Boost_REGEX_LIBRARY})
else()
  message(STATUS "failed to find boost regex, please install it")
  target_link_libraries(rmanipulation libopenrave)
endif()

add_dependencies(rmanipulation libopenrave)
set_target_properties(rmanipulation PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                       LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rmanipulation DESTINATION share/openrave/plugins )
