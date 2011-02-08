###########################################
# ikfastsolvers openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

if( Boost_IOSTREAMS_FOUND )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DBoost_IOSTREAMS_FOUND")
endif()

add_library(ikfastsolvers SHARED ikfastsolvers.cpp ikfastproblem.h plugindefs.h)
if( Boost_IOSTREAMS_FOUND )
  target_link_libraries(ikfastsolvers libopenrave ${Boost_IOSTREAMS_LIBRARY})
else()
  target_link_libraries(ikfastsolvers libopenrave)
endif()
add_dependencies(ikfastsolvers libopenrave)
set_target_properties(ikfastsolvers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                      LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS ikfastsolvers DESTINATION share/openrave/plugins )
