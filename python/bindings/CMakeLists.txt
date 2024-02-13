if(NOT OPENRAVE_DISABLE_ASSERT_HANDLER)
  add_definitions("-DBOOST_ENABLE_ASSERT_HANDLER") # turns segfault into exception
  add_definitions("-DRAPIDJSON_ASSERT=BOOST_ASSERT") # turns segfault into exception
endif()

# all cpp files for python bindings
set(openravepy_int_cpp_files
    openravepy_int.cpp
    openravepy_kinbody.cpp
    openravepy_robot.cpp
    openravepy_global.cpp
    openravepy_collisionchecker.cpp
    openravepy_controller.cpp
    openravepy_ikparameterization.cpp
    openravepy_iksolver.cpp
    openravepy_module.cpp
    openravepy_physicsengine.cpp
    openravepy_planner.cpp
    openravepy_planningutils.cpp
    openravepy_sensor.cpp
    openravepy_sensorsystem.cpp
    openravepy_spacesampler.cpp
    openravepy_trajectory.cpp
    openravepy_viewer.cpp
    bindings.cpp
    docstrings.cpp
)

# Python version agnostic
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/openravepy_config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/openravepy/openravepy_config.h
    IMMEDIATE @ONLY
)

####################
##### PYTHON 2 #####
####################

if (OPT_PYTHON)
    ## openrave bindings
    add_library(openravepy_int SHARED ${openravepy_int_cpp_files})
    target_link_libraries(openravepy_int PRIVATE boost_assertion_failed PUBLIC openravepy2_meta)
    set_target_properties(openravepy_int PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS}")
    install(TARGETS openravepy_int EXPORT openrave-targets DESTINATION ${OPENRAVEPY2_VER_INSTALL_DIR})

    ## ann bindings
    if (ANN_FOUND)
        add_library(pyANN_int SHARED pyann.cpp bindings.cpp)
        include_directories(${ANN_INCLUDE_DIR})
        target_link_libraries(pyANN_int PRIVATE boost_assertion_failed ANN PUBLIC openravepy2_meta)
        set_target_properties(pyANN_int PROPERTIES PREFIX "" COMPILE_FLAGS "${ANN_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}")
        install(TARGETS pyANN_int EXPORT openrave-targets DESTINATION ${OPENRAVEPY2_VER_INSTALL_DIR})
    endif()

    ## convexdecomposition bindings
    if (CONVEXDECOMPOSITION_FOUND)
        add_library(convexdecompositionpy SHARED convexdecompositionpy.cpp bindings.cpp)
        include_directories(${CONVEXDECOMPOSITION_INCLUDE_DIR})
        target_link_libraries(convexdecompositionpy PRIVATE boost_assertion_failed convexdecomposition PUBLIC openravepy2_meta)
        set_target_properties(convexdecompositionpy PROPERTIES PREFIX "" COMPILE_FLAGS "${CONVEXDECOMPOSITION_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}" )
        install(TARGETS convexdecompositionpy EXPORT openrave-targets DESTINATION ${OPENRAVEPY2_VER_INSTALL_DIR})
    endif()
endif()

####################
##### PYTHON 3 #####
####################

if (OPT_PYTHON3)
    ## openrave bindings
    add_library(openravepy3_int SHARED ${openravepy_int_cpp_files})
    target_link_libraries(openravepy3_int PRIVATE boost_assertion_failed PUBLIC openravepy3_meta)
    target_compile_options(openravepy3_int PRIVATE "${OPENRAVEPY_COMPILE_FLAGS}")
    set_target_properties(openravepy3_int PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "python3"
        OUTPUT_NAME "openravepy_int"
    )
    install(TARGETS openravepy3_int EXPORT openrave-targets DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR})

    ## ann bindings
    if (ANN_FOUND)
        add_library(py3ANN_int SHARED pyann.cpp bindings.cpp)
        include_directories(${ANN_INCLUDE_DIR})
        target_link_libraries(py3ANN_int PRIVATE boost_assertion_failed ANN PUBLIC openravepy3_meta)
        target_compile_options(py3ANN_int PRIVATE "${ANN_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}")
        set_target_properties(py3ANN_int PROPERTIES
            PREFIX ""
            LIBRARY_OUTPUT_DIRECTORY "python3"
            OUTPUT_NAME "pyANN_int"
        )
        install(TARGETS py3ANN_int EXPORT openrave-targets DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR})
    endif()

    ## convexdecomposition bindings
    if (CONVEXDECOMPOSITION_FOUND)
        add_library(convexdecompositionpy3 SHARED convexdecompositionpy.cpp bindings.cpp)
        include_directories(${CONVEXDECOMPOSITION_INCLUDE_DIR})
        target_link_libraries(convexdecompositionpy3 PRIVATE boost_assertion_failed convexdecomposition PUBLIC openravepy3_meta)
        target_compile_options(convexdecompositionpy3 PRIVATE "${CONVEXDECOMPOSITION_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}")
        set_target_properties(convexdecompositionpy3 PROPERTIES
            PREFIX ""
            LIBRARY_OUTPUT_DIRECTORY "python3"
            OUTPUT_NAME "convexdecompositionpy"
        )
        install(TARGETS convexdecompositionpy3 EXPORT openrave-targets DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR})
    endif()
endif()

message(STATUS "Can successfully build OpenRAVE Python bindings; will install headers for python bindings of other modules")
file(GLOB openravepy_headers include/openravepy/*.h)
install(FILES
    ${openravepy_headers}
    ${CMAKE_CURRENT_BINARY_DIR}/include/openravepy/openravepy_config.h
    DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy
    COMPONENT ${COMPONENT_PREFIX}python
)
