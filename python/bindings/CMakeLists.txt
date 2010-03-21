if( Boost_PYTHON_FOUND AND Boost_THREAD_FOUND )
  find_package(PythonLibs)

  if( PYTHONLIBS_FOUND OR PYTHON_LIBRARIES )
    if( PYTHON_EXECUTABLE )
      # get the site-packages directory
      execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print get_python_lib()"
        OUTPUT_VARIABLE _python_sitepackage
        RESULT_VARIABLE _python_failed)
      if( ${_python_failed} EQUAL 0 )
        string(REGEX REPLACE "[\r\n]" "" _python_sitepackage "${_python_sitepackage}")
        set(PYTHON_INCLUDE_PATH ${PYTHON_INCLUDE_PATH} ${_python_sitepackage}/numpy/core/include)
      else()
        message(STATUS "failed to get python site-package directory")
      endif()
    endif()

    add_definitions(${Boost_CFLAGS})
    set(CMAKE_REQUIRED_INCLUDES ${PYTHON_INCLUDE_PATH} ${Boost_INCLUDE_DIR} )
    set(CMAKE_REQUIRED_LIBRARIES ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})
    set(CMAKE_REQUIRED_FLAGS ${Boost_CFLAGS})

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

    if( HAVE_ALL_PYTHON_HEADERS )
      message(STATUS "python and boost-python found")
      include_directories(${PYTHON_INCLUDE_PATH})
      include_directories("${CMAKE_SOURCE_DIR}")
      include_directories("${CMAKE_SOURCE_DIR}/src/libopenrave-core")

      ## openrave bindings
      add_library(openravepy_int SHARED openravepy_int.cpp bindings.cpp bindings.h)
      # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/Segfault_in_cxa_allocate_exception#Workaround)
      target_link_libraries(openravepy_int ${STDC_LIBRARY} libopenrave libopenrave-core ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})
      add_dependencies(openravepy_int libopenrave libopenrave-core)
      set_target_properties(openravepy_int PROPERTIES PREFIX "" COMPILE_FLAGS "-DRAVE_CORE_USEDLL")
      if( WIN32 )
        set_target_properties(openravepy_int PROPERTIES SUFFIX ".pyd")
      endif()
      install(TARGETS openravepy_int DESTINATION ${OPENRAVEPY_INSTALL_DIR})

      ## ann bindings
      if( ANN_FOUND )
        add_library(pyANN SHARED pyann.cpp bindings.cpp bindings.h)
        include_directories(${ANN_INCLUDE_DIR})
        # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/  Segfault_in_cxa_allocate_exception#Workaround)
        target_link_libraries(pyANN ${STDC_LIBRARY} ANN ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})
        set_target_properties(pyANN PROPERTIES PREFIX "" COMPILE_FLAGS "${ANN_CFLAGS}" )
        add_dependencies(pyANN ANN)
        if( WIN32 )
          set_target_properties(pyANN PROPERTIES SUFFIX ".pyd")
          if( MSVC )
            # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
            # this is such an unbelievable hack, that it's disgusting
            set_target_properties(pyANN PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
          endif()
        endif()
        install(TARGETS pyANN DESTINATION ${OPENRAVEPY_INSTALL_DIR})
      endif()

      ## convexdecomposition bindings
      if( CONVEXDECOMPOSITION_FOUND )
        add_library(convexdecompositionpy SHARED convexdecompositionpy.cpp bindings.cpp bindings.h)
        include_directories(${CONVEXDECOMPOSITION_INCLUDE_DIR})
        # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/  Segfault_in_cxa_allocate_exception#Workaround)
        target_link_libraries(convexdecompositionpy ${STDC_LIBRARY} convexdecomposition ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})
        set_target_properties(convexdecompositionpy PROPERTIES PREFIX "" COMPILE_FLAGS "${CONVEXDECOMPOSITION_CFLAGS}" )
        add_dependencies(convexdecompositionpy convexdecomposition)
        if( WIN32 )
          set_target_properties(convexdecompositionpy PROPERTIES SUFFIX ".pyd")
          if( MSVC )
            # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
            # this is such an unbelievable hack, that it's disgusting
            set_target_properties(convexdecompositionpy PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
          endif()
        endif()
        install(TARGETS convexdecompositionpy DESTINATION ${OPENRAVEPY_INSTALL_DIR})
      endif()
    else()
      message(STATUS "failed to use boost python libraries, check if python-numpy is installed.")
    endif()
  else()
    message(STATUS "failed to find development python headers please install them")
  endif()
else()
  message(STATUS "failed to find boost-python or boost-thread, please install them")
endif()
