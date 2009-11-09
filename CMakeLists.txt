#############################################################################
# Project info
##############################################################################
cmake_minimum_required (VERSION 2.4.6) # need INSTALL(DIRECTORY ...)
project (openrave)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

#set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Define here the needed parameters
set (RAVE_CURRENT 0)
set (RAVE_REVISION 2)
set (RAVE_AGE 0)
set (RAVE_RELEASE ${RAVE_CURRENT}.${RAVE_REVISION}.${RAVE_AGE})
set (RAVE_SONAME ${RAVE_CURRENT})

# Differences between CMake 2.4 and 2.6
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)
   message(STATUS "Using cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" )
   # some developers may be using an cmake cvs version which didn't have set_property() yet
   # Tell them that a more recent version is required.
   if(NOT COMMAND SET_PROPERTY)
      message(FATAL_ERROR "You are using an old version of CMake from cvs, please update to CMake >= 2.6.0 or cvs at least from Feb 20th, 2008")
   endif(NOT COMMAND SET_PROPERTY)


   # CMake 2.6 gives errors if there are multiple targets with the same name
   # we use this for the target "buildtests", which is created for the unit tests
   # and which depends on the tests, so building "buildtests" builds all the tests
   # enabling this property disables this check in CMake
   # visual studio doesn't like it
   #set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)
    
   # CMP0002: in KDE4 we have multiple targets with the same name for the unit tests
   cmake_policy(SET CMP0002 OLD)
   # CMP0003: add the link paths to the link command as with cmake 2.4
   cmake_policy(SET CMP0003 OLD)
   # CMake 2.6 uses the full file names of the libraries when linking and so 
   # doesn't use -L anymore to list the link dirs. Now the dependencies created
   # export_library_dependencies() lists the in-project libraries without 
   # path, i.e. with only the logical name ("kdecore"), so they don't link
   # Setting this variable to true has the effect that the link dirs are
   # listed nevertheless also with CMake 2.6.
   set(CMAKE_LINK_OLD_PATHS TRUE)

endif("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)


# Differences between CMake 2.4 and 2.6
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)
   # CMP0003: add the link paths to the link command as with cmake 2.4
   cmake_policy(SET CMP0003 OLD)
   # CMake 2.6 uses the full file names of the libraries when linking and so 
   # doesn't use -L anymore to list the link dirs. Now the dependencies created
   # export_library_dependencies() lists the in-project libraries without 
   # path, i.e. with only the logical name ("kdecore"), so they don't link
   # Setting this variable to true has the effect that the link dirs are
   # listed nevertheless also with CMake 2.6.
   set(CMAKE_LINK_OLD_PATHS TRUE)
endif("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)

if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/localinstall" CACHE PATH "openrave install prefix" FORCE )
endif()

##############################################################################
# Sets the location of nonstandard shared libraries
##############################################################################

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# When building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

message(STATUS "installing to ${CMAKE_INSTALL_PREFIX}")

##############################################################################
# Custom CMake options
##############################################################################

option(ENABLE_VIDEORECORDING "Enable video recording" ON)
option(PLUGINS "Build the pluings" ON)
option(DOUBLE_PRECISION "Use double precision" OFF)

##############################################################################
# Required packages
##############################################################################

# Additional CMake modules for 3rd party library checks reside here
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules-cmake")

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)
include(FindPkgConfig)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g") # only add NDEBUG when generating super release modules
  set(CMAKE_CXX_FLAGS_DEBUG "-g -D_DEBUG")
endif()

set(OPENRAVE_LINK_DIRS "")

# to facilitate compilation, all visual studio libraries are included locally
set(Boost_ADDITIONAL_VERSIONS "1.40" "1.39" "1.38" "1.37.0" "1.37" "1.35.0" "1.34.1" "1.34.0" "1.34" "1.33.1" "1.33.0" "1.33")

if( MSVC )
  add_definitions( -D_CRT_SECURE_NO_WARNINGS)
  add_definitions( -D_CRT_SECURE_NO_DEPRECATE)

  # untar the source files
  add_custom_command(
    OUTPUT ${CMAKE_SOURCE_DIR}/inc
    COMMAND ${CMAKE_COMMAND} -E chdir
    ARGS "${CMAKE_SOURCE_DIR}" ${CMAKE_COMMAND} -E tar xzf "${CMAKE_SOURCE_DIR}/msvc_files.tgz"
    DEPENDS ${CMAKE_SOURCE_DIR}/msvc_files.tgz)
  add_custom_target(msvc_files ALL DEPENDS ${CMAKE_SOURCE_DIR}/inc)
  set(EXTRA_MSVC_DEPEND msvc_files)

  include_directories("${CMAKE_SOURCE_DIR}/inc")
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} "${CMAKE_SOURCE_DIR}/lib/win32")

  # manually set boost root, find_package errors due to tgz file
  if( NOT BOOST_ROOT )
    set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/inc")
  endif()

  set(Boost_FOUND 1)
  set(Boost_INCLUDE_DIRS ${BOOST_ROOT})
  set(Boost_LIBRARY_DIRS)
  set(Boost_REGEX_FOUND 0)
  set(Boost_FILESYSTEM_FOUND 0)
  set(Boost_SYSTEM_FOUND 0)
  set(Boost_PYTHON_FOUND 0)
  set(Boost_THREAD_FOUND 0)

  # make sure to install the library since it will be necessary for linking
  install(FILES ${CMAKE_SOURCE_DIR}/lib/win32/libxml2.lib DESTINATION lib)
  install(FILES ${CMAKE_SOURCE_DIR}/lib/win32/libxml2d.lib DESTINATION lib)
  install(FILES ${CMAKE_SOURCE_DIR}/inc/stdint.h DESTINATION include)
else()
  if( NOT $ENV{BOOST_INCLUDEDIR} STREQUAL "" )
    set(Boost_INCLUDE_DIR $ENV{BOOST_INCLUDEDIR})
  endif()
  if( NOT $ENV{BOOST_LIBRARYDIR} STREQUAL "" )
    set(Boost_LIBRARY_DIRS $ENV{BOOST_LIBRARYDIR})
  endif()
  find_package(Boost COMPONENTS regex filesystem system python thread)
endif()

message(STATUS "found boost version: ${Boost_VERSION}")

if( Boost_FOUND )
  include_directories(${Boost_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${Boost_LIBRARY_DIRS})
elseif(Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0")
  include_directories(${Boost_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${Boost_LIBRARY_DIRS})
else()
  message(FATAL_ERROR "Could not find boost libraries!")
endif()

set(OPENRAVE_BOOST_INCLUDE_DIRS)
foreach(idir ${Boost_INCLUDE_DIRS})
  set(OPENRAVE_BOOST_INCLUDE_DIRS "${OPENRAVE_BOOST_INCLUDE_DIRS} -I${idir}")
endforeach()

set(OPENRAVE_BOOST_LIB_DIRS)
foreach(ldir ${Boost_LIBRARY_DIRS})
  set(OPENRAVE_BOOST_LIB_DIRS "${OPENRAVE_BOOST_LIB_DIRS} -L${ldir}")
endforeach()


# generate the md5 sum for all OpenRAVE interfaces
add_subdirectory(cpp-gen-md5)

if( MSVC )
  add_dependencies(cpp-gen-md5 ${EXTRA_MSVC_DEPEND})
  set(CPPGENMD5 cpp-gen-md5)
else()
  set(CPPGENMD5 "${CMAKE_CURRENT_BINARY_DIR}/cpp-gen-md5/cpp-gen-md5")
endif()

add_custom_command(
  OUTPUT ${CMAKE_SOURCE_DIR}/rave/classhashes.h
  COMMAND ${CPPGENMD5}
  ARGS "${CMAKE_SOURCE_DIR}/rave/interface.h"
       "${CMAKE_SOURCE_DIR}/rave/collisionchecker.h" OPENRAVE_COLLISIONCHECKER_HASH
       "${CMAKE_SOURCE_DIR}/rave/robot.h" OPENRAVE_ROBOT_HASH
       "${CMAKE_SOURCE_DIR}/rave/planner.h" OPENRAVE_PLANNER_HASH
       "${CMAKE_SOURCE_DIR}/rave/kinbody.h" OPENRAVE_KINBODY_HASH
       "${CMAKE_SOURCE_DIR}/rave/sensorsystem.h" OPENRAVE_SENSORSYSTEM_HASH
       "${CMAKE_SOURCE_DIR}/rave/controller.h" OPENRAVE_CONTROLLER_HASH
       "${CMAKE_SOURCE_DIR}/rave/problems.h" OPENRAVE_PROBLEM_HASH
       "${CMAKE_SOURCE_DIR}/rave/iksolver.h" OPENRAVE_IKSOLVER_HASH
       "${CMAKE_SOURCE_DIR}/rave/physicsengine.h" OPENRAVE_PHYSICSENGINE_HASH
       "${CMAKE_SOURCE_DIR}/rave/sensor.h" OPENRAVE_SENSOR_HASH
       "${CMAKE_SOURCE_DIR}/rave/viewer.h" OPENRAVE_VIEWER_HASH
       "${CMAKE_SOURCE_DIR}/rave/trajectory.h" OPENRAVE_TRAJECTORY_HASH
       > "${CMAKE_SOURCE_DIR}/rave/classhashes.h"
  DEPENDS cpp-gen-md5
  ${CMAKE_SOURCE_DIR}/rave/collisionchecker.h
  ${CMAKE_SOURCE_DIR}/rave/robot.h
  ${CMAKE_SOURCE_DIR}/rave/planner.h
  ${CMAKE_SOURCE_DIR}/rave/kinbody.h
  ${CMAKE_SOURCE_DIR}/rave/sensorsystem.h
  ${CMAKE_SOURCE_DIR}/rave/controller.h
  ${CMAKE_SOURCE_DIR}/rave/problems.h
  ${CMAKE_SOURCE_DIR}/rave/iksolver.h
  ${CMAKE_SOURCE_DIR}/rave/physicsengine.h
  ${CMAKE_SOURCE_DIR}/rave/sensor.h
  ${CMAKE_SOURCE_DIR}/rave/viewer.h
  ${CMAKE_SOURCE_DIR}/rave/trajectory.h)

add_custom_target(classhashes_h ALL DEPENDS ${CMAKE_SOURCE_DIR}/rave/classhashes.h ${EXTRA_MSVC_DEPEND})

# always include coin3d (needed for file parsing)
find_package(Coin)
if( COIN_LIBRARY_FOUND )
  include_directories(${COIN_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COIN_LINK_DIRS})
  add_definitions("${COIN_CXXFLAGS} -DOPENRAVE_COIN3D")
  message(STATUS "Using Coin3D")
else()
  if( NOT COIN_LIBRARY_FOUND )
    message(STATUS "Could not find Coin3D. Please install Coin3D (http://www.coin3d.org)")
  endif()
endif()

#libXml2
find_package(LibXml2)

if( LIBXML2_FOUND )
  include_directories(${LIBXML2_INCLUDE_DIR})
  add_definitions(${LIBXML2_DEFINITIONS})
  message(STATUS "libxml2 found")
else()
  if( MSVC )
    set(LIBXML2_LIBRARIES optimized libxml2 debug libxml2d)
  else()
    message(FATAL_ERROR "Could not find libxml2")
  endif()
endif()

# zlib
check_library_exists(z deflate "/lib;/usr/lib;/usr/local/lib;/usr/pkg/lib" HAVE_ZLIB)

if(DOUBLE_PRECISION)
  set(OPENRAVE_PRECISION 1)
  message(STATUS "Using double precision")
else()
  set(OPENRAVE_PRECISION 0)
  message(STATUS "Using single precision")
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  check_cxx_source_runs("
  int main()
  {
    int a = 0;
    int*pa = &a;
    asm(\".intel_syntax\\\\n\"
	\"mov %%rax, %0\\\\n\"
    \"mov %%eax, [%%rax]\\\\n\"
    \".att_syntax\\\\n\"
    : : \"r\"(pa) : \"%rax\");
    return 0;
  }"
  IS_X86_64)

  if( IS_X86_64 )
    add_definitions("-D__x86_64__")
  endif()
else()
  set(IS_X86_64 0)
endif()

link_directories(${OPENRAVE_LINK_DIRS})

set(OPENRAVE_PLUGINS_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/openrave/plugins")
set(OPENRAVE_DATA_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/openrave")

# add ikfast program
set(IKFAST_EXECUTABLE ${CMAKE_SOURCE_DIR}/ikfast/ikfast.py)
execute_process(
  COMMAND ${IKFAST_EXECUTABLE} -h
  RESULT_VARIABLE _ikfast_success)

if( _ikfast_success MATCHES 0 )
  set(IKFAST_FOUND 1)
  message(STATUS "ikfast found (${IKFAST_EXECUTABLE})")
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" STRGREATER "2.4.3")
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/ikfast/sympy DESTINATION share/openrave)
  else()
    message(WARNING "Could not install sympy needed by ikfast (cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} needs to be >= 2.4.6)")
  endif()
  set(IKFAST_INSTALLED_EXECUTABLE ${CMAKE_INSTALL_PREFIX}/share/openrave/ikfast.py)
  install(PROGRAMS ${IKFAST_EXECUTABLE} DESTINATION share/openrave)
else()
  set(IKFAST_INSTALLED_EXECUTABLE "")
  set(IKFAST_FOUND 0)
  message(STATUS "failed to execute ${IKFAST_EXECUTABLE}")
endif()

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/defines.h.in ${CMAKE_CURRENT_SOURCE_DIR}/rave/defines.h)

add_subdirectory(src)
add_subdirectory(plugins)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/openrave-config.in"
  "${CMAKE_CURRENT_BINARY_DIR}/openrave-config"
  IMMEDIATE @ONLY)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/openrave-config DESTINATION bin)

if( WIN32 )
  # copy the dlls
  file(GLOB dll_files ${CMAKE_SOURCE_DIR}/bin/*.dll)
  install(FILES ${dll_files} DESTINATION bin)
endif()

file(GLOB rave_include_files ${CMAKE_CURRENT_SOURCE_DIR}/rave/*.h)
install(FILES ${rave_include_files} ${CMAKE_CURRENT_SOURCE_DIR}/rave/defines.h ${CMAKE_SOURCE_DIR}/rave/classhashes.h DESTINATION include/rave)

# add make uninstall capability
configure_file(
  "${CMAKE_SOURCE_DIR}/modules-cmake/cmake_uninstall.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
