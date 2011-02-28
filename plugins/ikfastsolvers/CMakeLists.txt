###########################################
# ikfastsolvers openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

if( Boost_IOSTREAMS_FOUND )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DBoost_IOSTREAMS_FOUND")
endif()

set(ik_names barrettwam katana5d pa10 pr2_head pr2_head_torso pr2_leftarm pr2_leftarm_torso pr2_rightarm pr2_rightarm_torso puma schunk_lwa3)
set(ik_libraries)
foreach(ik_name ${ik_names})
  add_library(ik_${ik_name} STATIC ik_${ik_name}.cpp)
  set_target_properties(ik_${ik_name} PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DIKFAST_NO_MAIN -DIKFAST_NAMESPACE=${ik_name} -DIKFAST_REAL=double")
  set(ik_libraries ${ik_libraries} ik_${ik_name})
endforeach()

add_library(ikfastsolvers SHARED ikfastsolvers.cpp ikfastproblem.h plugindefs.h)
if( Boost_IOSTREAMS_FOUND )
  target_link_libraries(ikfastsolvers libopenrave ${ik_libraries} ${Boost_IOSTREAMS_LIBRARY})
else()
  target_link_libraries(ikfastsolvers libopenrave ${ik_libraries})
endif()
add_dependencies(ikfastsolvers libopenrave ${ik_libraries})
set_target_properties(ikfastsolvers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                      LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS ikfastsolvers DESTINATION share/openrave/plugins )
