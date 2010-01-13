###########################################
# oderave openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

include(${CMAKE_CURRENT_SOURCE_DIR}/FindODE.cmake)
if(ODE_LIBRARY_FOUND)
  link_directories(${OPENRAVE_LINK_DIRS} ${ODE_LINK_DIRS})

  # doesn't work
#  set(CMAKE_REQUIRED_INCLUDES ${ODE_INCLUDE_DIRS})
#  set(CMAKE_REQUIRED_FLAGS "${ODE_CXXFLAGS}")
#  set(CMAKE_REQUIRED_LIBRARIES ${ODE_LIBRARY})
#  message(STATUS "ode lib: ${ODE_LIBRARY}")
#  check_function_exists(dAllocateODEDataForThread ODE_HAVE_ALLOCATE_DATA_THREAD)
if( ODE_HAVE_ALLOCATE_DATA_THREAD )
  add_definitions("-DODE_HAVE_ALLOCATE_DATA_THREAD")
endif()

  include_directories(${CMAKE_SOURCE_DIR} ${ODE_INCLUDE_DIRS})
  add_library(oderave SHARED oderavemain.cpp odecollision.h odephysics.h odespace.h plugindefs.h)

  # test for ode version 0.10
  set(CMAKE_REQUIRED_INCLUDES "${ODE_INCLUDE_DIRS}")
  set(CMAKE_REQUIRED_FLAGS "${ODE_CXXFLAGS}")
  check_cxx_source_compiles("
    #include <ode/ode.h>

    typedef int dTriIndex;
    int main()
    {
      return 0;
    }"
    NEED_TRIINDEX)
  set(CMAKE_REQUIRED_FLAGS "")

  if( NEED_TRIINDEX )
    set(ODE_CXXFLAGS "${ODE_CXXFLAGS} -DNEED_DTRIINDEX_TYPEDEF")
  endif()

  message(STATUS "ODE found, building oderave plugin, precision=${ODE_PRECISION}")

  target_link_libraries(oderave libopenrave ${ODE_LIBRARY})
  add_dependencies(oderave libopenrave)
  set_target_properties(oderave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} ${ODE_CXXFLAGS}"
                                LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
  install(TARGETS oderave DESTINATION share/openrave/plugins )
else()
  message(STATUS "Could not find ODE. Please install ODE (http://www.ode.org)")
endif()

# restore the link dirs
link_directories(${OPENRAVE_LINK_DIRS})
