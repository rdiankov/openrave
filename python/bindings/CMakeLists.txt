if(NOT OPENRAVE_DISABLE_ASSERT_HANDLER)
  add_definitions("-DBOOST_ENABLE_ASSERT_HANDLER") # turns segfault into exception
  add_definitions("-DRAPIDJSON_ASSERT=BOOST_ASSERT") # turns segfault into exception
endif()

# all cpp files for python bindings
set(openravepy_int_cpp_files openravepy_int.cpp openravepy_kinbody.cpp openravepy_robot.cpp openravepy_global.cpp
    openravepy_collisionchecker.cpp openravepy_controller.cpp openravepy_ikparameterization.cpp openravepy_iksolver.cpp
    openravepy_module.cpp openravepy_physicsengine.cpp openravepy_planner.cpp openravepy_planningutils.cpp
    openravepy_sensor.cpp openravepy_sensorsystem.cpp openravepy_spacesampler.cpp openravepy_trajectory.cpp
    openravepy_viewer.cpp bindings.cpp docstrings.cpp)

if( NOT(PYTHONLIBS_FOUND OR PYTHON_LIBRARIES ) )
  message(STATUS "failed to find development python headers please install them")
  message(STATUS "failed to build OpenRAVE python bindings")
  return()
endif()

set(SUCCESS_BUILD_PYTHON_BINDINGS ON)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/openravepy_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/openravepy/openravepy_config.h IMMEDIATE @ONLY)

if(USE_PYBIND11_PYTHON_BINDINGS)
  # check if all header files can be included
  set(PYTHON_INCLUDE_DIRS_BACKUP ${PYTHON_INCLUDE_DIRS}) # backup
  find_package(pybind11 REQUIRED) # this messed up ${PYTHON_INCLUDE_DIRS}
  set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS_BACKUP}) # restore
  add_definitions(${Boost_CFLAGS})
  set(CMAKE_REQUIRED_INCLUDES ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${REQUIRED_INCLUDES})
  set(CMAKE_REQUIRED_LIBRARIES ${PYTHON_LIBRARIES} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  set(CMAKE_REQUIRED_FLAGS ${Boost_CFLAGS} "-DCMAKE_BUILD_TYPE:STRING=Release")
  check_cxx_source_runs("
    // python
    #include <Python.h>
    #include <pyconfig.h>
    // numpy
    #include <numpy/arrayobject.h>
    #include <numpy/arrayscalars.h>
    // pybind11
    #include <pybind11/pybind11.h>
    #include <pybind11/stl.h>
    #include <pybind11/numpy.h>
    // #include <pybind11/eigen.h> // requires Eigen3 and include ${CMAKE_INSTALL_PREFIX}/include/eigen3
    int main() { return 0; }
  "
  HAVE_ALL_PYTHON_HEADERS)
  set(HAVE_ALL_PYTHON_HEADERS ${HAVE_ALL_PYTHON_HEADERS} PARENT_SCOPE)

  if(NOT HAVE_ALL_PYTHON_HEADERS)
    message(FATAL_ERROR "Should have installed pybind11; CMake will exit.")
  endif()
  
  if( HAVE_ALL_PYTHON_HEADERS )
    # include
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${PYTHON_INCLUDE_DIRS} ${OPENRAVE_INCLUDE_LOCAL_DIRS} ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS})
    add_definitions("-DUSE_PYBIND11_PYTHON_BINDINGS")
    set(OPENRAVEPY_COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS} -DOPENRAVE_CORE_DLL -fvisibility=hidden") # pybind11 requires less visibility
        
    # link
    set(OPENRAVEPY_LINKED_LIBRARIES "${STDC_LIBRARY} ${PYTHON_LIBRARIES} ${Boost_THREAD_LIBRARY}")
    if( CLOCK_GETTIME_FOUND )
      set(OPENRAVEPY_LINKED_LIBRARIES ${OPENRAVEPY_LINKED_LIBRARIES} rt)
    endif()

    ## openrave bindings
    add_library(openravepy_int SHARED ${openravepy_int_cpp_files})
    # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/Segfault_in_cxa_allocate_exception#Workaround)
    target_link_libraries(openravepy_int PUBLIC ${OPENRAVEPY_LINKED_LIBRARIES} libopenrave libopenrave-core PRIVATE boost_assertion_failed)
    add_dependencies(openravepy_int libopenrave libopenrave-core)
    set_target_properties(openravepy_int PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS}")
    if( WIN32 )
      set_target_properties(openravepy_int PROPERTIES SUFFIX ".pyd")
    elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      # on macOS, python cannot import libraries with .dylib extension
      set_target_properties(openravepy_int PROPERTIES SUFFIX ".so")
    endif()
    install(TARGETS openravepy_int DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
    
    file(GLOB openravepy_pybind11_headers include/openravepy/pybind11/*.h)
    install(FILES ${openravepy_pybind11_headers} DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy/pybind11 COMPONENT ${COMPONENT_PREFIX}python)
    
    ## ann bindings
    if( ANN_FOUND )
      add_library(pyANN_int SHARED pyann.cpp bindings.cpp)
      include_directories(${ANN_INCLUDE_DIR})
      # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/  Segfault_in_cxa_allocate_exception#Workaround)
      target_link_libraries(pyANN_int PUBLIC ${OPENRAVEPY_LINKED_LIBRARIES} ANN PRIVATE boost_assertion_failed)
      set_target_properties(pyANN_int PROPERTIES PREFIX "" COMPILE_FLAGS "${ANN_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}")
      add_dependencies(pyANN_int ANN)
      if( WIN32 )
        set_target_properties(pyANN_int PROPERTIES SUFFIX ".pyd")
        if( MSVC )
          # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
          # this is such an unbelievable hack, that it's disgusting
          set_target_properties(pyANN_int PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
        endif()
      elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set_target_properties(pyANN_int PROPERTIES SUFFIX ".so")
      endif()
      install(TARGETS pyANN_int DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
    endif()

    ## convexdecomposition bindings
    if( CONVEXDECOMPOSITION_FOUND )
      add_library(convexdecompositionpy SHARED convexdecompositionpy.cpp bindings.cpp)
      include_directories(${CONVEXDECOMPOSITION_INCLUDE_DIR})
      target_link_libraries(convexdecompositionpy PUBLIC ${OPENRAVEPY_LINKED_LIBRARIES} convexdecomposition PRIVATE boost_assertion_failed)
      set_target_properties(convexdecompositionpy PROPERTIES PREFIX "" COMPILE_FLAGS "${CONVEXDECOMPOSITION_CFLAGS} ${OPENRAVEPY_COMPILE_FLAGS}" )
      add_dependencies(convexdecompositionpy convexdecomposition)
      if( WIN32 )
        set_target_properties(convexdecompositionpy PROPERTIES SUFFIX ".pyd")
        if( MSVC )
          set_target_properties(convexdecompositionpy PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
        endif()
      elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set_target_properties(convexdecompositionpy PROPERTIES SUFFIX ".so")
      endif()
      install(TARGETS convexdecompositionpy DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
    endif()

    set(SUCCESS_BUILD_PYTHON_BINDINGS ON) # success
  else()
    message(STATUS "failed to find pybind11, please install")
    set(SUCCESS_BUILD_PYTHON_BINDINGS OFF)
  endif()

elseif( Boost_PYTHON_FOUND AND Boost_THREAD_FOUND )
  add_definitions(${Boost_CFLAGS})
  set(CMAKE_REQUIRED_INCLUDES ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${REQUIRED_INCLUDES})
  # Need to link Boost::System on macOS for symbols boost::system::system_category(), boost::system::generic_category()
  set(CMAKE_REQUIRED_LIBRARIES ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  set(CMAKE_REQUIRED_FLAGS ${Boost_CFLAGS} "-DCMAKE_BUILD_TYPE:STRING=Release")

  # check if all header files can be included
  check_cxx_source_runs("
  #include <Python.h>
  #include <boost/thread/thread.hpp>
  #include <boost/thread/mutex.hpp>
  #include <boost/thread/condition.hpp>
  #include <boost/shared_ptr.hpp>
  #define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
  #include <boost/python.hpp>
  #include <pyconfig.h>
  #include <numpy/arrayobject.h>
  int main()
  {
    return 0;
  }"
  HAVE_ALL_PYTHON_HEADERS)
  set(HAVE_ALL_PYTHON_HEADERS ${HAVE_ALL_PYTHON_HEADERS} PARENT_SCOPE)
  if( HAVE_ALL_PYTHON_HEADERS )
    message(STATUS "python and boost-python found")
    message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${PYTHON_INCLUDE_DIRS} ${OPENRAVE_INCLUDE_LOCAL_DIRS} ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS})

    # libraries to link
    set(OPENRAVEPY_LINKED_LIBRARIES "${STDC_LIBRARY} ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY}")
    if( CLOCK_GETTIME_FOUND )
      set(OPENRAVEPY_LINKED_LIBRARIES ${OPENRAVEPY_LINKED_LIBRARIES} rt)
    endif()

    set(extralibs)
    if( CLOCK_GETTIME_FOUND )
      set(extralibs ${extralibs} rt)
    endif()

    set(OPENRAVEPY_COMPILE_FLAGS "-DOPENRAVE_CORE_DLL")
    if( MSVC )
      check_cxx_compiler_flag("/bigobj" COMPILER_HAS_BIGOBJ)
      if( COMPILER_HAS_BIGOBJ )
        set(OPENRAVEPY_COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS} /bigobj")
      endif()
    endif()

    ## openrave bindings
    add_library(openravepy_int SHARED ${openravepy_int_cpp_files})
    target_link_libraries(openravepy_int PUBLIC ${OPENRAVEPY_LINKED_LIBRARIES} libopenrave libopenrave-core PRIVATE boost_assertion_failed)
    add_dependencies(openravepy_int libopenrave libopenrave-core)
    set_target_properties(openravepy_int PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS}")
    if( WIN32 )
      set_target_properties(openravepy_int PROPERTIES SUFFIX ".pyd")
    elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      set_target_properties(openravepy_int PROPERTIES SUFFIX ".so")
    endif()
    install(TARGETS openravepy_int DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)

    file(GLOB openravepy_boostpython_headers include/openravepy/boostpython/*.h)
    install(FILES ${openravepy_boostpython_headers} DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy/boostpython COMPONENT ${COMPONENT_PREFIX}python)
    
    ## ann bindings
    if( ANN_FOUND )
      add_library(pyANN_int SHARED pyann.cpp bindings.cpp)
      include_directories(${ANN_INCLUDE_DIR})
      target_link_libraries(pyANN_int PUBLIC ${OPENRAVEPY_LINKED_LIBRARIES} libopenrave ANN PRIVATE boost_assertion_failed)
      set_target_properties(pyANN_int PROPERTIES PREFIX "" COMPILE_FLAGS "${ANN_CFLAGS}" )
      add_dependencies(pyANN_int ANN)
      if( WIN32 )
        set_target_properties(pyANN_int PROPERTIES SUFFIX ".pyd")
        if( MSVC )
          set_target_properties(pyANN_int PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
        endif()
      elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        # on mac osx, python cannot import libraries with .dylib extension
        set_target_properties(pyANN_int PROPERTIES SUFFIX ".so")
      endif()
      install(TARGETS pyANN_int DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
    endif()

    ## convexdecomposition bindings
    if( CONVEXDECOMPOSITION_FOUND )
      add_library(convexdecompositionpy SHARED convexdecompositionpy.cpp bindings.cpp)
      include_directories(${CONVEXDECOMPOSITION_INCLUDE_DIR})
      target_link_libraries(convexdecompositionpy PUBLIC ${OPENRAVEPY_LINKED_LIBRARIES} libopenrave convexdecomposition PRIVATE boost_assertion_failed)
      set_target_properties(convexdecompositionpy PROPERTIES PREFIX "" COMPILE_FLAGS "${CONVEXDECOMPOSITION_CFLAGS}" )
      add_dependencies(convexdecompositionpy convexdecomposition)
      if( WIN32 )
        set_target_properties(convexdecompositionpy PROPERTIES SUFFIX ".pyd")
        if( MSVC )
          set_target_properties(convexdecompositionpy PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
        endif()
      elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set_target_properties(convexdecompositionpy PROPERTIES SUFFIX ".so")
      endif()
      install(TARGETS convexdecompositionpy DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
    endif()

    # success
    set(SUCCESS_BUILD_PYTHON_BINDINGS ON)
  else()
    message(STATUS "failed to use boost python libraries, check if python-numpy is installed.")
    set(SUCCESS_BUILD_PYTHON_BINDINGS OFF)
  endif()
else()
  message(STATUS "failed to find pybind11 or Boost.Python, please install either")
  set(SUCCESS_BUILD_PYTHON_BINDINGS OFF)
endif()

if(SUCCESS_BUILD_PYTHON_BINDINGS)
  message(STATUS "Can successfully build OpenRAVE python bindings; will install headers for python bindings of other modules")
  file(GLOB openravepy_headers include/openravepy/*.h)
  install(FILES ${openravepy_headers} ${CMAKE_CURRENT_BINARY_DIR}/include/openravepy/openravepy_config.h DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy COMPONENT ${COMPONENT_PREFIX}python)
else()
  message(STATUS "Failed to build OpenRAVE python bindings")
endif()
