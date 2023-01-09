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

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/openravepy_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/openravepy/openravepy_config.h IMMEDIATE @ONLY)

# Add a meta-target that sets the required dependencies, include paths, compile options and defintions for all OpenRAVE python modules.
# These values will be transitively inherited by any object that links against this target.
add_library(openravepy_meta INTERFACE)
target_include_directories(openravepy_meta INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${OPENRAVE_INCLUDE_LOCAL_DIRS}>
    $<BUILD_INTERFACE:${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${OPENRAVEPY_VER_INSTALL_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}>
)

set(PYBIND11_NOPYTHON ON) # Disable pybind11 internal FindPython, because we want to manage this ourselves
find_package(pybind11 2.9.2) # At least 2.9.2, the last version of pybind11 to support Python 2.
if (pybind11_FOUND)
    target_link_libraries(openravepy_meta INTERFACE rt Python::Module Python::NumPy pybind11::module libopenrave libopenrave-core)
    target_compile_options(openravepy_meta INTERFACE "-fvisibility=hidden") # pybind11 requires less visibility
    target_compile_definitions(openravepy_meta INTERFACE "OPENRAVE_CORE_DLL=1" "USE_PYBIND11_PYTHON_BINDINGS=1")
    set(USE_PYBIND11_PYTHON_BINDINGS ON)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/openravepy/pybind11 DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy/pybind11)
else()
    find_package(Boost REQUIRED COMPONENTS python thread)
    target_link_libraries(openravepy_meta INTERFACE rt Python::Module Python::NumPy Boost::python Boost::thread libopenrave libopenrave-core)
    target_compile_definitions(openravepy_meta INTERFACE "OPENRAVE_CORE_DLL=1")
    set(USE_PYBIND11_PYTHON_BINDINGS OFF)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/openravepy/boostpython DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy/boostpython)
endif()
install(TARGETS openravepy_meta EXPORT openrave-targets)

## openrave bindings
add_library(openravepy_int SHARED ${openravepy_int_cpp_files})
target_link_libraries(openravepy_int PRIVATE boost_assertion_failed PUBLIC openravepy_meta)
set_target_properties(openravepy_int PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS}")
install(TARGETS openravepy_int EXPORT openrave-targets DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR})

## ann bindings
if (ANN_FOUND)
    add_library(pyANN_int SHARED pyann.cpp bindings.cpp)
    include_directories(${ANN_INCLUDE_DIR})
    target_link_libraries(pyANN_int PRIVATE boost_assertion_failed ANN PUBLIC openravepy_meta)
    set_target_properties(pyANN_int PROPERTIES PREFIX "" COMPILE_FLAGS "${ANN_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}")
    install(TARGETS pyANN_int EXPORT openrave-targets DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR})
endif()

## convexdecomposition bindings
if (CONVEXDECOMPOSITION_FOUND)
    add_library(convexdecompositionpy SHARED convexdecompositionpy.cpp bindings.cpp)
    include_directories(${CONVEXDECOMPOSITION_INCLUDE_DIR})
    target_link_libraries(convexdecompositionpy PRIVATE boost_assertion_failed convexdecomposition PUBLIC openravepy_meta)
    set_target_properties(convexdecompositionpy PROPERTIES PREFIX "" COMPILE_FLAGS "${CONVEXDECOMPOSITION_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}" )
    install(TARGETS convexdecompositionpy EXPORT openrave-targets DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR})
endif()

file(GLOB openravepy_headers include/openravepy/*.h)
install(FILES ${openravepy_headers} ${CMAKE_CURRENT_BINARY_DIR}/include/openravepy/openravepy_config.h DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy)