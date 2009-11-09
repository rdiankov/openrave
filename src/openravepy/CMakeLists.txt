if( Boost_PYTHON_FOUND AND Boost_THREAD_FOUND )
  find_package(PythonLibs)

  if( PYTHONLIBS_FOUND OR PYTHON_LIBRARIES )
    set(CMAKE_REQUIRED_INCLUDES ${PYTHON_INCLUDE_PATH} ${Boost_INCLUDE_DIR})
    set(CMAKE_REQUIRED_LIBRARIES ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})

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

      add_library(openravepy SHARED openravepy.cpp)
      target_link_libraries(openravepy libopenrave libopenrave-core ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})
      add_dependencies(openravepy libopenrave libopenrave_static libopenrave-core libopenrave-core_static)
      set_target_properties(openravepy PROPERTIES PREFIX "")
      install(TARGETS openravepy DESTINATION share/openrave)
    else()
      message(STATUS "failed to use boost python libraries, check if python-numpy is installed.")
    endif()
  else()
    message(STATUS "failed to find development python headers please install them")
  endif()
else()
  message(STATUS "failed to find boost-python or boost-thread, please install them")
endif()
