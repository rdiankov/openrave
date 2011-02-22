#############################################################################
# Project info
##############################################################################
cmake_minimum_required (VERSION 2.4.6) # need INSTALL(DIRECTORY ...)
project (openrave)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

# Define here the needed parameters
set (OPENRAVE_VERSION_MAJOR 0)
set (OPENRAVE_VERSION_MINOR 2)
set (OPENRAVE_VERSION_PATCH 19)
set (OPENRAVE_VERSION ${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.${OPENRAVE_VERSION_PATCH})
set (OPENRAVE_SOVERSION ${OPENRAVE_VERSION_MAJOR})
message(STATUS "Compiling OpenRAVE Version ${OPENRAVE_VERSION}")

# Differences between CMake 2.4 and 2.6
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)
   message(STATUS "Using cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" )
   # some developers may be using an cmake cvs version which didn't have set_property() yet
   # Tell them that a more recent version is required.
   if(NOT COMMAND SET_PROPERTY)
      message(FATAL_ERROR "You are using an old version of CMake from cvs, please update to CMake >= 2.6.0 or cvs at least from Feb 20th, 2008")
   endif(NOT COMMAND SET_PROPERTY)
   # http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0002
   cmake_policy(SET CMP0002 NEW)
   # http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0003
   cmake_policy(SET CMP0003 NEW)
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

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" STRLESS "2.4.3")
  message(STATUS "cmake version too low, might not be able to install directories")
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

option(OPT_VIDEORECORDING "Enable video recording" ON)
option(OPT_PLUGINS "Build the pluings" ON)
option(OPT_DOUBLE_PRECISION "Use double precision" ON)
option(OPT_ACCURATEMATH "Use accurate and robust math to account for floating-point errors" ON)
option(OPT_PYTHON "Build python bindings" ON)
option(OPT_OCTAVE "Build octave bindings" ON)
option(OPT_MATLAB "Build matlab bindings" ON)
option(OPT_STATIC "Build static libraries" OFF)

##############################################################################
# Required packages
##############################################################################

# Additional CMake modules for 3rd party library checks reside here
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules-cmake")

message(STATUS "detected system processor: ${CMAKE_SYSTEM_PROCESSOR}")

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)
include(CheckTypeSize)
include(FindPkgConfig)

set(OPENRAVE_PLUGINS_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/openrave/plugins")
set(OPENRAVE_DATA_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/openrave")
set(OPENRAVE_PYTHON_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/openrave")
set(OPENRAVE_OCTAVE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/openrave/octave")
set(OPENRAVE_MATLAB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/openrave/matlab")
set(OPENRAVEPY_INSTALL_DIR "${OPENRAVE_PYTHON_INSTALL_DIR}/openravepy")

# very important to only test release since default is debug, and it usually reuquires special libraries
set(CMAKE_TRY_COMPILE_CONFIGURATION Release)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG -DBOOST_DISABLE_ASSERTS -D_SECURE_SCL=0") # this practically removes all checks making it a very dangerous options to play with
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -D_DEBUG")
endif()

if( MSVC )
  set(LINKER_HAS_RDYNAMIC 0)
  set(LINKER_HAS_BSYMBOLIC 0)
  set(LINKER_HAS_BSYMBOLIC_FUNCTIONS 0)
  set(LINKER_HAS_VISIBILITY 0)
  set(LINKER_HAS_VISIBILITY_INLINES_HIDDEN 0)
else()
  check_cxx_compiler_flag("-rdynamic" LINKER_HAS_RDYNAMIC)
  check_cxx_compiler_flag("-Bsymbolic" LINKER_HAS_BSYMBOLIC)
  check_cxx_compiler_flag("-Bsymbolic-functions" LINKER_HAS_BSYMBOLIC_FUNCTIONS)
  check_cxx_compiler_flag("-fvisibility=hidden" LINKER_HAS_VISIBILITY)
  check_cxx_compiler_flag("-fvisibility-inlines-hidden" LINKER_HAS_VISIBILITY_INLINES_HIDDEN)
endif()

if( UNIX OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set(STDC_LIBRARY stdc++)
else()
  set(STDC_LIBRARY)
endif()

if( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # apple doesn't have 64bit versions of file opening functions, so add them
  add_definitions("-Dfopen64=fopen -Dfseeko64=fseeko -Dfseek64=fseek -Dftell64=ftell -Dftello64=ftello")
endif()

set(OPENRAVE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
set(OPENRAVE_CORE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/libopenrave-core)
set(OPENRAVE_LINK_DIRS "")

# to facilitate compilation, all visual studio libraries are included locally
set(Boost_ADDITIONAL_VERSIONS "1.46" "1.45" "1.44" "1.43" "1.42" "1.41" "1.40" "1.39" "1.38" "1.37.0" "1.37" "1.35.0" "1.34.1" "1.34.0" "1.34" "1.33.1" "1.33.0" "1.33")

if( MSVC )
  add_definitions( -D_CRT_SECURE_NO_WARNINGS)
  add_definitions( -D_CRT_SECURE_NO_DEPRECATE)

  #gives linking problems
  #string(REGEX REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  #string(REGEX REPLACE "/MDd" "/MD" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")

  # allow extern "C" functions to throw exceptions
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHc- ")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /EHc- ")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /EHc- ")
  set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /EHc- ")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /EHc- ")

  # untar the source files
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/inc
    COMMAND ${CMAKE_COMMAND} -E chdir
    ARGS "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/msvc_files.tgz"
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/msvc_files.tgz)
  add_custom_target(msvc_files ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/inc)
  set(EXTRA_MSVC_DEPEND msvc_files)

  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/inc")
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/lib/win32")

  # manually set boost root, find_package errors due to tgz file
  if( NOT BOOST_ROOT )
    set(BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/inc")
  endif()

  set(Boost_FOUND 1)
  set(Boost_INCLUDE_DIRS ${BOOST_ROOT})
  set(Boost_LIBRARY_DIRS)
  set(Boost_REGEX_FOUND 0)
  set(Boost_FILESYSTEM_FOUND 1)
  set(Boost_IOSTREAMS_FOUND 1)
  set(Boost_SYSTEM_FOUND 1)
  set(Boost_PYTHON_FOUND 1)
  set(Boost_THREAD_FOUND 1)
  set(Boost_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc")
  set(Boost_CFLAGS "-DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
  # have to add to required flags for applications that will test linking with boost
  set(CMAKE_REQUIRED_FLAGS "-DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
  if( MSVC70 OR MSVC71 )
    set(Boost_THREAD_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_thread-vc71-mt-1_39.lib ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_date_time-vc71-mt-1_39.lib)
    set(Boost_SYSTEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_system-vc71-mt-1_39.lib)
    set(Boost_FILESYSTEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_filesystem-vc71-mt-1_39.lib)
    set(Boost_PYTHON_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_python-vc71-mt-1_39.lib)
    set(Boost_IOSTREAMS_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_iostreams-vc71-mt-1_39.lib)
  elseif( MSVC80 )
    set(Boost_THREAD_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_thread-vc80-mt-1_39.lib ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_date_time-vc80-mt-1_39.lib)
    set(Boost_SYSTEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_system-vc80-mt-1_39.lib)
    set(Boost_FILESYSTEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_filesystem-vc80-mt-1_39.lib)
    set(Boost_PYTHON_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_python-vc80-mt-1_39.lib)
    set(Boost_IOSTREAMS_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_iostreams-vc80-mt-1_39.lib)
  else()
    set(Boost_THREAD_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_thread-vc90-mt-1_39.lib ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_date_time-vc90-mt-1_39.lib)
    set(Boost_SYSTEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_system-vc90-mt-1_39.lib)
    set(Boost_FILESYSTEM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_filesystem-vc90-mt-1_39.lib)
    set(Boost_PYTHON_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_python-vc90-mt-1_39.lib)
    set(Boost_IOSTREAMS_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_iostreams-vc90-mt-1_39.lib)
  endif()

  # make sure to install the library since it will be necessary for linking
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/libxml2.lib DESTINATION lib)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/libxml2d.lib DESTINATION lib)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/inc/stdint.h DESTINATION include)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/inc/boost DESTINATION include)

  file(GLOB boost_lib_files ${CMAKE_CURRENT_SOURCE_DIR}/lib/win32/boost_*.lib)
  install(FILES ${boost_lib_files} DESTINATION lib)

else()
  if( NOT $ENV{BOOST_INCLUDEDIR} STREQUAL "" )
    set(Boost_INCLUDE_DIR $ENV{BOOST_INCLUDEDIR})
  endif()
  if( NOT $ENV{BOOST_LIBRARYDIR} STREQUAL "" )
    set(Boost_LIBRARY_DIRS $ENV{BOOST_LIBRARYDIR})
  endif()
  find_package(Boost COMPONENTS regex filesystem system python thread iostreams)
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

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(EXTRA_COMPILE_FLAGS "-fPIC")
else()
  set(EXTRA_COMPILE_FLAGS "")
endif()

# generate the md5 sum for all OpenRAVE interfaces
add_subdirectory(cpp-gen-md5)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/cpp-gen-md5")

if( MSVC )
  add_dependencies(cpp-gen-md5 ${EXTRA_MSVC_DEPEND})
  set(CPPGENMD5 cpp-gen-md5)
else()
  set(CPPGENMD5 "${CMAKE_CURRENT_BINARY_DIR}/cpp-gen-md5/cpp-gen-md5")
endif()

if(OPT_DOUBLE_PRECISION)
  set(OPENRAVE_PRECISION 1)
  message(STATUS "Using double precision")
else()
  set(OPENRAVE_PRECISION 0)
  message(STATUS "Using single precision")
endif()

set(interfacehashes_h ${CMAKE_CURRENT_BINARY_DIR}/rave/interfacehashes.h)
add_custom_command(
  OUTPUT ${interfacehashes_h}
  COMMAND ${CPPGENMD5}
  ARGS "${OPENRAVE_PRECISION}" "${CMAKE_CURRENT_SOURCE_DIR}/rave/interface.h"
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/collisionchecker.h" OPENRAVE_COLLISIONCHECKER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/robot.h" OPENRAVE_ROBOT_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/planner.h" OPENRAVE_PLANNER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/kinbody.h" OPENRAVE_KINBODY_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/sensorsystem.h" OPENRAVE_SENSORSYSTEM_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/controller.h" OPENRAVE_CONTROLLER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/problems.h" OPENRAVE_PROBLEM_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/iksolver.h" OPENRAVE_IKSOLVER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/physicsengine.h" OPENRAVE_PHYSICSENGINE_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/sensor.h" OPENRAVE_SENSOR_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/viewer.h" OPENRAVE_VIEWER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/trajectory.h" OPENRAVE_TRAJECTORY_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/environment.h" OPENRAVE_ENVIRONMENT_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/rave/plugininfo.h" OPENRAVE_PLUGININFO_HASH
       > "${interfacehashes_h}"
  DEPENDS cpp-gen-md5
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/collisionchecker.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/robot.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/planner.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/kinbody.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/sensorsystem.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/controller.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/problems.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/iksolver.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/physicsengine.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/sensor.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/viewer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/trajectory.h
  ${CMAKE_CURRENT_SOURCE_DIR}/rave/environment.h)

add_custom_target(interfacehashes_target ALL DEPENDS ${interfacehashes_h} ${EXTRA_MSVC_DEPEND})

# math libraries
find_package(GMP)
find_package(GMPXX)
find_package(MPFR)
find_package(MPFI)

if( OPT_ACCURATEMATH AND NOT MSVC )
  add_subdirectory(3rdparty/crlibm-1.0beta4)
  set(CRLIBM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/crlibm-1.0beta4")
  set(CRLIBM_FOUND 1)
  set(CRLIBM_LIBRARY crlibm)
else()
  set(CRLIBM_FOUND 0)
endif()

# always include coin3d (needed for file parsing)
find_package(Coin)
if( COIN_LIBRARY_FOUND )
  include_directories(${COIN_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COIN_LINK_DIRS})
  add_definitions("-DOPENRAVE_COIN3D")
  message(STATUS "Using Coin3D")
else()
  if( NOT COIN_LIBRARY_FOUND )
    message(STATUS "Could not find Coin3D. Please install Coin3D (http://www.coin3d.org)")
  endif()
  set(COIN_CXXFLAGS)
  set(COIN_LINK_FLAGS)
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

check_library_exists(rt clock_gettime "" CLOCK_GETTIME_FOUND)
if( CLOCK_GETTIME_FOUND )
  add_definitions(-DCLOCK_GETTIME_FOUND)
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
    set(LIBXML2_INCLUDE_DIR)
    set(LIBXML2_DEFINITIONS)
  else()
    message(FATAL_ERROR "Could not find libxml2")
  endif()
endif()

link_directories(${OPENRAVE_LINK_DIRS})

find_package(ZLIB)
if( NOT ZLIB_FOUND )
  message(STATUS "compiling zlib from souces")
  # compile from sources
  add_subdirectory(3rdparty/zlib)
  set(ZLIB_FOUND 1)
  set(ZLIB_LIBRARIES zlib)
  set(ZLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/zlib)
endif()

pkg_check_modules(COLLADA collada15dom>=2.3.0)

# require minizip regardless of collada presence
add_subdirectory(3rdparty/minizip)
set(MINIZIP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/minizip ${ZLIB_INCLUDE_DIR})

if( NOT COLLADA_FOUND AND ZLIB_FOUND AND Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
  pkg_check_modules(libpcrecpp libpcrecpp)
  if( libpcrecpp_FOUND )
    set(CMAKE_REQUIRED_INCLUDES ${libpcrecpp_INCLUDE_DIRS})
    check_include_file_cxx(pcrecpp.h HAVE_PCRECPP_H)
    set(CMAKE_REQUIRED_INCLUDES)
    if( NOT HAVE_PCRECPP_H )
      set(libpcrecpp_FOUND 0)
    endif()
  endif()

  if( NOT libpcrecpp_FOUND )
    message(STATUS "System pcre not found, using local from sources")
    # include the local pcre
    add_subdirectory(3rdparty/pcre-8.02)
    set(libpcrecpp_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/pcre-8.02)
    set(libpcrecpp_LIBRARY_DIRS)
    set(libpcrecpp_LIBRARIES pcrecpp_local)
    set(libpcrecpp_CFLAGS_OTHERS "-DPCRE_STATIC")
    set(libpcrecpp_LDFLAGS_OTHERS)
  endif()

  add_subdirectory(3rdparty/collada)
  set(COLLADA_FOUND 1)
  set(COLLADA_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/collada/include ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/collada/include/1.5)
  set(COLLADA_LIBRARY_DIRS)
  set(COLLADA_LIBRARIES collada15reader)
  set(COLLADA_CFLAGS_OTHER)
  set(COLLADA_LDFLAGS_OTHER)
else()
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COLLADA_LIBRARY_DIRS})
endif()
if( NOT COLLADA_FOUND )
  message(STATUS "no COLLADA support found")
endif()

link_directories(${OPENRAVE_LINK_DIRS})

## check qhull library
check_library_exists(qhull qh_new_qhull "" QHULL_FOUND_LIB)
check_include_file(qhull/qhull_a.h HAVE_QHULL_H)
if( QHULL_FOUND_LIB AND HAVE_QHULL_H )
  set(QHULL_FOUND 1)
  set(QHULL_INCLUDE_DIR)
else()
  # have to compile from sources
  message(STATUS "compiling local qhull library")
  add_subdirectory(3rdparty/qhull)
  set(QHULL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty")
  set(QHULL_FOUND 1)
endif()

## check ANN library
check_include_file_cxx(ANN/ANN.h HAVE_ANN_H)
if( HAVE_ANN_H )
  # _Z8annClosev is the gcc c++ mangled name of annClose
  check_library_exists(ANN _Z8annClosev "" ANN_FOUND_CXX_LIB)
  if( NOT ANN_FOUND_CXX_LIB )
    check_library_exists(ANN annClose "" ANN_FOUND_C_LIB)
  endif()
  if(ANN_FOUND_C_LIB OR ANN_FOUND_CXX_LIB)
    set(ANN_FOUND_LIB 1)
  else()
    set(ANN_FOUND_LIB 0)
  endif()
  if( NOT ANN_FOUND_LIB )
    message(STATUS "Found ANN headers but not library!")
  endif()
endif()

if( ANN_FOUND_LIB AND HAVE_ANN_H )
  set(ANN_FOUND 1)
  set(ANN_INCLUDE_DIR)
  set(ANN_CFLAGS)
else()
  # have to compile from sources
  message(STATUS "compiling local ann library")
  add_subdirectory(3rdparty/ann)
  set(ANN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ann/include")
  set(ANN_CFLAGS "-DANN_STATIC_LIBRARY")
  set(ANN_FOUND 1)
endif()

## check FLANN library
set(BUILD_MATLAB_BINDINGS false)
set(BUILD_PYTHON_BINDINGS true)
set(BUILD_C_BINDINGS true)
message(STATUS "compiling local flann library")
add_subdirectory(3rdparty/flann-1.6.6)
set(FLANN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/flann-1.6.6/src/cpp")
set(FLANN_FOUND 1) # use flann_cpp_s to link statically

# function expression parser library
add_subdirectory(3rdparty/fparser-4.3)
set(FPARSER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/fparser-4.3")
set(FPARSER_LIBRARY fparser)
set(FPARSER_FOUND 1)

message(STATUS "compiling local convexdecomposition library")
add_subdirectory(3rdparty/convexdecomposition)
set(CONVEXDECOMPOSITION_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/convexdecomposition")
if( UNIX )
  set(CONVEXDECOMPOSITION_CFLAGS "-DLINUX")
else()
  set(CONVEXDECOMPOSITION_CFLAGS "")
endif()
set(CONVEXDECOMPOSITION_FOUND 1)

## check python
find_package(PythonInterp)
if( NOT PYTHON_EXECUTABLE )
  # look specifically for 2.6
  FIND_PROGRAM(PYTHON_EXECUTABLE
    NAMES python2.6 python
    PATHS [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\2.6\\InstallPath])
endif()

if( PYTHON_EXECUTABLE )
  message(STATUS "python executable is ${PYTHON_EXECUTABLE}")
  set(IKFAST_INSTALLED_EXECUTABLE "${PYTHON_EXECUTABLE} \\\"${OPENRAVEPY_INSTALL_DIR}/ikfast.py\\\"")
else()
  message(STATUS "python executable not found")
  set(IKFAST_INSTALLED_EXECUTABLE "")
endif()

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/rave/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB rave_header_files ${CMAKE_CURRENT_SOURCE_DIR}/rave/*.h ${CMAKE_CURRENT_BINARY_DIR}/rave/config.h)

add_subdirectory(src)
add_subdirectory(octave_matlab)

# extract sympy
if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sympy/__init__.py" OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sympy/solvers/tests")
  message(STATUS "extracting sympy to ${CMAKE_CURRENT_SOURCE_DIR}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/sympy_0.6.7.tgz"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" STRGREATER "2.4.3")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/sympy" DESTINATION ${OPENRAVE_PYTHON_INSTALL_DIR} PATTERN ".svn" EXCLUDE)
else()
  message(WARNING "Could not install sympy needed by ikfast (cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} needs to be >= 2.4.6)")
endif()

if(OPT_PYTHON)
  add_subdirectory(python)
endif()

if(OPT_PLUGINS)
  add_subdirectory(plugins)
endif()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/openrave-config.in"
  "${CMAKE_CURRENT_BINARY_DIR}/openrave-config"
  IMMEDIATE @ONLY)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/openrave-config DESTINATION bin)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/openravebash DESTINATION share/openrave)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/openrave.pc.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/openrave.pc" @ONLY IMMEDIATE)

if(UNIX)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/openrave.pc DESTINATION lib/pkgconfig)
endif()


if( WIN32 )
  # copy the dlls
  file(GLOB dll_files ${CMAKE_CURRENT_SOURCE_DIR}/bin/*.dll)
  install(FILES ${dll_files} DESTINATION bin)
endif()

install(FILES ${rave_header_files} ${interfacehashes_h} DESTINATION include/rave)

# add make uninstall capability
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/modules-cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
