add_library(rampoptimizer SHARED paraboliccommon.h paraboliccommon.cpp ramp.h ramp.cpp interpolator.h interpolator.cpp feasibilitychecker.h feasibilitychecker.cpp parabolicchecker.h parabolicchecker.cpp)
target_link_libraries(rampoptimizer PRIVATE boost_assertion_failed PUBLIC libopenrave)
set_target_properties(rampoptimizer PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")

install(TARGETS rampoptimizer DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

add_dependencies(rampoptimizer interfacehashes_target)		

if (OPT_PYTHON)
    add_library(openravepy_rampoptimizer SHARED openravepy_rampoptimizer.cpp)
    target_link_libraries(openravepy_rampoptimizer PRIVATE boost_assertion_failed PUBLIC rampoptimizer openravepy_int openravepy2_meta)
    set_target_properties(openravepy_rampoptimizer PROPERTIES PREFIX "")
    install(TARGETS openravepy_rampoptimizer DESTINATION ${OPENRAVEPY2_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
endif()

if (OPT_PYTHON3)
    add_library(openravepy3_rampoptimizer SHARED openravepy_rampoptimizer.cpp)
    target_link_libraries(openravepy3_rampoptimizer PRIVATE boost_assertion_failed PUBLIC rampoptimizer openravepy3_int openravepy3_meta)
    set_target_properties(openravepy3_rampoptimizer PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "python3"
        OUTPUT_NAME "openravepy_rampoptimizer"
    )
    install(TARGETS openravepy3_rampoptimizer DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
endif()
