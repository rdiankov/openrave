  add_library(openravepy3_configurationcache SHARED ../openravepy_configurationcache.cpp)
  target_include_directories(openravepy3_configurationcache PUBLIC ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS} ${OPENRAVEPY_INCLUDE_LOCAL_DIRS})
  target_link_libraries(openravepy3_configurationcache
    PRIVATE
      boost_assertion_failed
    PUBLIC
      ${OPENRAVEPY_CONFIGURATIONCACHE_LINKED_LIBRARIES}
      Python3::Module
      Python3::NumPy
      pybind11::module
    )

  set_target_properties(openravepy3_configurationcache
    PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS}"
  )
  if( WIN32 )
    set_target_properties(openravepy3_configurationcache PROPERTIES SUFFIX ".pyd")
  endif()
  set_target_properties(openravepy3_configurationcache PROPERTIES OUTPUT_NAME openravepy_configurationcache)
  install(TARGETS openravepy3_configurationcache DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
