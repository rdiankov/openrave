cmake_minimum_required (VERSION 2.6.0)
project (cppexamples)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../..) # only used for cexamples!
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
link_directories(${OpenRAVE_LIBRARY_DIRS})

macro(build_openrave_executable name)
  add_executable(${name} ${name}.cpp)
  set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
  set_target_properties(${name} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
  target_link_libraries(${name} ${OpenRAVE_CORE_C_LIBRARIES})
  install(TARGETS ${name} DESTINATION . )
endmacro(build_openrave_executable)

build_openrave_executable(orc_bodyfunctions)
build_openrave_executable(orc_customgeometry)
build_openrave_executable(orc_rrtplanning)
