###########################################
# ikfastsolvers openrave plugin
###########################################
if( Boost_IOSTREAMS_FOUND )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DBoost_IOSTREAMS_FOUND")
endif()

set(ik_names barrettwam katana5d katana5d_trans pa10 pr2_head pr2_head_torso pr2_leftarm pr2_leftarm_torso pr2_rightarm pr2_rightarm_torso puma schunk_lwa3)
set(ik_libraries)
foreach(ik_name ${ik_names})
  add_library(ik_${ik_name} STATIC ik_${ik_name}.cpp)
  set_target_properties(ik_${ik_name} PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DIKFAST_NAMESPACE=${ik_name} -DIKFAST_HEADER=\"\\\"plugindefs.h\\\"\"")
  set(ik_libraries ${ik_libraries} ik_${ik_name})
  add_dependencies(ik_${ik_name} libopenrave)
endforeach()

add_library(ikfastsolvers SHARED ikfastsolvers.cpp ikfastproblem.h plugindefs.h)
if( Boost_IOSTREAMS_FOUND )
  target_link_libraries(ikfastsolvers libopenrave ${ik_libraries} ${Boost_IOSTREAMS_LIBRARY})
else()
  target_link_libraries(ikfastsolvers libopenrave ${ik_libraries})
endif()
set_target_properties(ikfastsolvers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS ikfastsolvers DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-ikfastsolvers)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-IKFASTSOLVERS_DESCRIPTION "Interfaces to IKFast" PARENT_SCOPE)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-IKFASTSOLVERS_DEPENDS libboost${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}-iostreams ${COMPONENT_PREFIX}base PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-ikfastsolvers PARENT_SCOPE)
