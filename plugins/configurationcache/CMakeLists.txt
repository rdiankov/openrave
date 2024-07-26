###########################################
# configurationcache openrave plugin
###########################################
add_library(configurationcache SHARED cachechecker.cpp configurationcache.cpp configurationcachetree.cpp configurationjitterer.cpp workspaceconfigurationjitterer.cpp)
target_link_libraries(configurationcache PRIVATE boost_assertion_failed PUBLIC libopenrave ${LAPACK_LIBRARIES})
set_target_properties(configurationcache PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS configurationcache DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

if (OPT_PYTHON)
    add_library(openravepy_configurationcache SHARED openravepy_configurationcache.cpp)
    target_link_libraries(openravepy_configurationcache PRIVATE boost_assertion_failed PUBLIC configurationcache openravepy_int openravepy2_meta)
    set_target_properties(openravepy_configurationcache PROPERTIES PREFIX "")
    install(TARGETS openravepy_configurationcache DESTINATION ${OPENRAVEPY2_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
endif()

if (OPT_PYTHON3)
    add_library(openravepy3_configurationcache SHARED openravepy_configurationcache.cpp)
    target_link_libraries(openravepy3_configurationcache PRIVATE boost_assertion_failed PUBLIC configurationcache openravepy3_int openravepy3_meta)
    set_target_properties(openravepy3_configurationcache PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "python3"
        OUTPUT_NAME "openravepy_configurationcache"
    )
    install(TARGETS openravepy3_configurationcache DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
endif()
