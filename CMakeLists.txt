cmake_minimum_required (VERSION 3.0)
project(OpenRAVE)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

# Define here the needed parameters
set (OPENRAVE_VERSION_MAJOR 0)
set (OPENRAVE_VERSION_MINOR 158)
set (OPENRAVE_VERSION_PATCH 0)
set (OPENRAVE_VERSION ${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.${OPENRAVE_VERSION_PATCH})
set (OPENRAVE_SOVERSION ${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR})
message(STATUS "Compiling OpenRAVE Version ${OPENRAVE_VERSION}, soversion=${OPENRAVE_SOVERSION}")

message(STATUS "Using cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" )
# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0002
cmake_policy(SET CMP0002 NEW)
# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0003
cmake_policy(SET CMP0003 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14) # C++14 for std::shared_timed_mutex
endif()
# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# When building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(LIB_SUFFIX CACHE STRING "suffix for the library directory need for x86-64 systems that use lib64 ")

# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Custom CMake options
option(OPT_VIDEORECORDING "Enable video recording" ON)
option(OPT_PLUGINS "Build the pluings" ON)
option(OPT_DOUBLE_PRECISION "Use double precision" ON)
option(OPT_ACCURATEMATH "Use accurate and robust math to account for floating-point errors" ON)
option(OPT_PYTHON "Build python bindings" ON)
option(OPT_PYTHON3 "Build python bindings" ON)
option(OPT_OCTAVE "Build octave bindings" ON)
option(OPT_MATLAB "Build matlab bindings" ON)
option(OPT_STATIC "Build static libraries" OFF)
option(OPT_COLLADA "Build with COLLADA support" ON)
option(OPT_BULLET "Use Bullet Collision/Physics if available" ON)
option(OPT_ODE_COLLISION "Use ODE collision checker if available" ON)
option(OPT_FCL_COLLISION "Use FCL collision checker if available" ON)
option(OPT_QTCOIN_VIEWER "Use QT/Coin Viewer if available" ON)
option(OPT_QTOSG_VIEWER "Use QT/OpenSceneGraph Viewer if available" ON)
option(OPT_BUILD_PACKAGES "Set to ON to generate CPack configuration files and packaging targets" OFF)
option(OPT_BUILD_PACKAGE_DEFAULT "Set to ON to generate a default openrave package that creates symlinks" ON)
option(OPT_IKFAST_FLOAT32 "Set to ON to allow loading of ikfast shared objects compiled with 32bit float (64bit double is always supported regardless of this option)" ON)
option(OPT_FLANN "Temporary switch to force building of flann" OFF)
option(OPT_CBINDINGS "Build the C-bindings libraries libopenrave_c and libopenrave-core_c" ON)
option(OPT_LOG4CXX "Use log4cxx for logging" ON)
option(OPT_MSGPACK "Support msgpack format" ON)
option(OPT_CURL "Use curl for downloading resources" ON)
option(OPT_INSTALL_3DMODELDATA "If ON, then install the 3D model data." ON)
option(OPT_EXTRA_ROBOTS "If ON, then install extra robots as part of 3D model data" ON)
option(OPT_ENVIRONMENT_RECURSIVE_LOCK "Use recursive mutex for the Environment. Defaults to ON." ON)
option(OPT_STATIC_PLUGINS "Statically compile in plugins" OFF)
option(OPT_PIC "Build position independent code" ON)
option(OPT_ENCRYPTION "Robot and KinBody encryption backed by GPG." ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ${OPT_PIC})

set(PACKAGE_VERSION "0" CACHE STRING "the package-specific version used for uploading the sources")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/modules-cmake")
set(CPACK_DEBIAN_PACKAGE_NAME openrave${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR})

if (OPT_ENVIRONMENT_RECURSIVE_LOCK)
  message(STATUS "using threads and synchronization primitives from std.")
  set(OPENRAVE_ENVIRONMENT_RECURSIVE_LOCK 1)
else()
  set(OPENRAVE_ENVIRONMENT_RECURSIVE_LOCK 0)
endif()

if(OPT_DOUBLE_PRECISION)
  set(OPENRAVE_PRECISION 1)
  set(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_DEBIAN_PACKAGE_NAME}-dp")
  message(STATUS "Using double precision")
else()
  set(OPENRAVE_PRECISION 0)
  set(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_DEBIAN_PACKAGE_NAME}-sp")
  message(STATUS "Using single precision")
endif()

if (OPT_STATIC_PLUGINS)
  set(OPT_PLUGINS OFF)
  set(OPENRAVE_STATIC_PLUGINS 1)
else()
  set(OPENRAVE_STATIC_PLUGINS 0)
endif()

set(COMPONENT_PREFIX "${CPACK_DEBIAN_PACKAGE_NAME}-")
string(TOUPPER ${COMPONENT_PREFIX} COMPONENT_PREFIX_UPPER)
set(CPACK_COMPONENTS_ALL ${COMPONENT_PREFIX}base ${COMPONENT_PREFIX}dev ${COMPONENT_PREFIX}data)

message(STATUS "detected system processor: ${CMAKE_SYSTEM_PROCESSOR}")

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)
include(CheckTypeSize)
include(InstallSymlink)
include(GNUInstallDirs)

find_package(PkgConfig) # pkg_check_modules

if( WIN32 )
  # does not support symlinks
  set(OPENRAVE_BIN_SUFFIX)
else()
  set(OPENRAVE_BIN_SUFFIX ${OPENRAVE_SOVERSION} CACHE STRING "Add the MAJOR.MINOR openrave version suffixes to all executable installed files")
endif()

set(OPENRAVE_SHARE_DIR "share/openrave-${OPENRAVE_SOVERSION}" CACHE PATH "Relative path for shared OpenRAVE data")
set(OPENRAVE_DATA_INSTALL_DIR "${OPENRAVE_SHARE_DIR}" CACHE PATH "Scene and robot files installation directory")
set(OPENRAVE_PLUGINS_INSTALL_DIR "lib${LIB_SUFFIX}/openrave${OPENRAVE_SOVERSION}-plugins" CACHE PATH "OpenRAVE plugins installation directory")
set(OPENRAVE_LOCALE_INSTALL_DIR "share/locale" CACHE PATH "Relative path for OpenRAVE translation files")
set(OPENRAVE_OCTAVE_INSTALL_DIR "${OPENRAVE_SHARE_DIR}/octave" CACHE PATH "OpenRAVE Octave bindings installation directory")
set(OPENRAVE_MATLAB_INSTALL_DIR "${OPENRAVE_SHARE_DIR}/matlab" CACHE PATH "OpenRAVE MATLAB bindings installation directory")
set(OPENRAVE_INCLUDE_INSTALL_DIR "openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}" CACHE STRING "Directory name for header files")
if( UNIX OR CYGWIN)
  # cache has to be STRING instead of PATH for path to remain relative
  set(BASH_COMPLETION_DIR CACHE STRING "Directory where bash completion files will be installed in, on linux it is /etc/bash_completion.d")# Additional CMake modules for 3rd party library checks reside here
endif()

set(OPENRAVE_PLUGINS_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_PLUGINS_INSTALL_DIR}" CACHE PATH "Absolute dir for plugins on target OS path, used to write to header file")
set(OPENRAVE_DATA_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_DATA_INSTALL_DIR}" CACHE PATH "Absolute dir for data on target OS path, used to write to header file")
set(OPENRAVE_LOCALE_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_LOCALE_INSTALL_DIR}" CACHE PATH "Absolute dir for translation files on target OS path, used to write to header file")

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}" "${OPENRAVE_PLUGINS_INSTALL_ABSOLUTE_DIR}")

set(CMAKE_TRY_COMPILE_CONFIGURATION Release CACHE STRING "very important to only test release since default is debug, and it usually reuquires special libraries")
set(OPENRAVE_INCLUDE_LOCAL_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OPENRAVEPY_INCLUDE_LOCAL_DIRS ${CMAKE_CURRENT_BINARY_DIR}/python/bindings/include ${CMAKE_CURRENT_SOURCE_DIR}/python/bindings/include)

message(STATUS "CMAKE_INSTALL_PREFIX          = ${CMAKE_INSTALL_PREFIX}")
message(STATUS "PYTHON_EXECUTABLE             = ${PYTHON_EXECUTABLE}")
message(STATUS "OPENRAVE_BIN_SUFFIX           = ${OPENRAVE_BIN_SUFFIX}")
message(STATUS "OPENRAVE_INCLUDE_LOCAL_DIRS   = ${OPENRAVE_INCLUDE_LOCAL_DIRS}")
message(STATUS "OPENRAVEPY_INCLUDE_LOCAL_DIRS = ${OPENRAVEPY_INCLUDE_LOCAL_DIRS}")

# need this in the very first include, otherwise openrave plugins can accidentaly compile with a system-installed openrave headers.
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${OPENRAVE_INCLUDE_LOCAL_DIRS}) # needed since plugins include openrave/openrave.h

set(REQUIRED_INCLUDES "" CACHE STRING "Includes to always force  when compiling")
include_directories(${REQUIRED_INCLUDES})

set(COMPILER_IS_CLANG FALSE)
if( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
  set(COMPILER_IS_CLANG TRUE)
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG)
  add_definitions("-fno-strict-aliasing -Wall -Werror=shadow")
  add_definitions("-Werror=return-type")
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-Werror=return-local-addr")
elseif( COMPILER_IS_CLANG )
  add_definitions("-Werror=return-stack-address")
endif()

set(OPENRAVE_EXPORT_CXXFLAGS)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG)
  set(CMAKE_CXX_FLAGS_OPTIMIZED "-O3 -DNDEBUG -DBOOST_DISABLE_ASSERTS -D_SECURE_SCL=0") # this practically removes all checks making it a very dangerous options to play with
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g3 -O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g3 -D_DEBUG")

  # the _GLIBCXX_DEBUG flag is very helpful in bounds checking for vector[] and iterators, however it requires all
  # libraries linking with openrave to use it
#  if( CMAKE_BUILD_TYPE STREQUAL "Debug")
#    set(OPENRAVE_EXPORT_CXXFLAGS "${OPENRAVE_EXPORT_CXXFLAGS} -D_GLIBCXX_DEBUG")
#  endif()
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

if( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # apple doesn't have 64bit versions of file opening functions, so add them
  add_definitions(-Dfopen64=fopen -Dfseeko64=fseeko -Dfseek64=fseek -Dftell64=ftell -Dftello64=ftello)
endif()

set(OPENRAVE_CORE_INCLUDE_LOCAL_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/libopenrave-core)
set(OPENRAVE_LINK_DIRS "")

if( MSVC )
  add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_DEPRECATE)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_DEPRECATE)

  #gives linking problems
  #string(REGEX REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  #string(REGEX REPLACE "/MDd" "/MD" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")

  # /EHc- allow extern "C" functions to throw exceptions
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHc- ")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /EHc- ")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /EHc- ")
  set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /EHc- ")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /EHc- ")

  # untar the source files
  set(EXTRA_MSVC_DEPEND msvc_boost msvc_collada msvc_libxml2 msvc_ode msvc_soqt)
  foreach(MSVC_LIBRARY ${EXTRA_MSVC_DEPEND})
    if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}" )
      execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}.tgz" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()

    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}
      COMMAND ${CMAKE_COMMAND} -E chdir
      ARGS "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}.tgz"
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}.tgz)
    add_custom_target(${MSVC_LIBRARY} ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY})
  endforeach()

  check_include_file(stdint.h HAVE_STDINT_H)
  if( NOT HAVE_STDINT_H )
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/msvc_include/stdint.h DESTINATION include/openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR} COMPONENT ${COMPONENT_PREFIX}dev)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/msvc_include)
  endif()
  
  # have to install dlls manually
  if( MSVC70 OR MSVC71 )
    set(MSVC_PREFIX "vc70")
  elseif( MSVC80 )
    set(MSVC_PREFIX "vc80")
  elseif( MSVC90 )
    set(MSVC_PREFIX "vc90")
  else()
    set(MSVC_PREFIX "vc100")
  endif()
  set(OPENRAVE_LIBRARY_SUFFIX "${OPENRAVE_SOVERSION}-${MSVC_PREFIX}-mt" CACHE STRING "Suffix to append to library names")
  
  # force multi-threaded DLL boost
  set(Boost_USE_MULTITHREAD ON)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_STATIC_RUNTIME OFF)
  set(Boost_CFLAGS "-DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
else()
  set(OPENRAVE_LIBRARY_SUFFIX "${OPENRAVE_SOVERSION}" CACHE STRING "Suffix to append to library names")
endif()

find_package(Boost REQUIRED COMPONENTS filesystem system thread iostreams)
set(OPENRAVE_BOOST_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
set(OPENRAVE_BOOST_LIB_DIRS ${Boost_LIBRARY_DIRS})

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG )
  set(NATIVE_COMPILE_FLAGS "" CACHE STRING "compilation flags to pass to tools that are executed at build-time. For example '-march=native -mtune=native'. By default this is empty.")
else()
  set(NATIVE_COMPILE_FLAGS "")
endif()

# generate the md5 sum for all OpenRAVE interfaces
add_subdirectory(cpp-gen-md5)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/cpp-gen-md5")

if( MSVC )
  if( EXTRA_MSVC_DEPEND )
    add_dependencies(cpp-gen-md5-native ${EXTRA_MSVC_DEPEND})
  endif()
  set(CPPGENMD5 cpp-gen-md5-native)
else()
  set(CPPGENMD5 "${CMAKE_CURRENT_BINARY_DIR}/cpp-gen-md5/cpp-gen-md5-native")
endif()

set(interfacehashes_h ${CMAKE_CURRENT_BINARY_DIR}/include/openrave/interfacehashes.h)
add_custom_command(
  OUTPUT ${interfacehashes_h}
  COMMAND ${CPPGENMD5}
  ARGS "${OPENRAVE_PRECISION}" "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/interface.h"
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/collisionchecker.h" OPENRAVE_COLLISIONCHECKER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/robot.h" OPENRAVE_ROBOT_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/planner.h" OPENRAVE_PLANNER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/kinbody.h" OPENRAVE_KINBODY_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/sensorsystem.h" OPENRAVE_SENSORSYSTEM_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/controller.h" OPENRAVE_CONTROLLER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/module.h" OPENRAVE_MODULE_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/iksolver.h" OPENRAVE_IKSOLVER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/physicsengine.h" OPENRAVE_PHYSICSENGINE_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/sensor.h" OPENRAVE_SENSOR_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/trajectory.h" OPENRAVE_TRAJECTORY_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/viewer.h" OPENRAVE_VIEWER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/spacesampler.h" OPENRAVE_SPACESAMPLER_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/environment.h" OPENRAVE_ENVIRONMENT_HASH
       "${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/plugininfo.h" OPENRAVE_PLUGININFO_HASH
       > "${interfacehashes_h}"
  DEPENDS cpp-gen-md5-native
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/interface.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/collisionchecker.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/robot.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/planner.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/kinbody.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/sensorsystem.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/controller.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/module.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/iksolver.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/physicsengine.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/sensor.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/trajectory.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/viewer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/spacesampler.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/environment.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/plugininfo.h)

add_custom_target(interfacehashes_target ALL DEPENDS ${interfacehashes_h} ${EXTRA_MSVC_DEPEND})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/openrave/interfacehashes.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}/openrave)

# math libraries
find_package(GMP)
find_package(GMPXX)
find_package(MPFR 2.4)
if( MPFR_FOUND )
  check_library_exists(${MPFR_LIBRARIES} mpfr_div_d "${MPFR_LIBRARIES_DIR}" MPFR_DIV_D_FOUND)
  if( NOT MPFR_DIV_D_FOUND )
    message(WARNING "MPFR verison too old, does not support mpfr_div_d function")
    set(MPFR_FOUND 0)
  endif()
endif()
find_package(MPFI)


if( OPT_ACCURATEMATH AND NOT MSVC AND NOT ${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64 )
  check_library_exists(crlibm crlibm_init "" CRLIBM_FOUND_LIB)
  check_include_file(crlibm.h HAVE_CRLIBM_H)
  if(NOT CRLIBM_FOUND_LIB OR NOT HAVE_CRLIBM_H)
    message(STATUS "Using local crlibm")
    add_subdirectory(3rdparty/crlibm-1.0beta4)
  endif()
endif()

#if( MSVC )
#  # MSVC has prepackaged assimp libraries
#  set(ASSIMP_PATH ${CMAKE_CURRENT_SOURCE_DIR}/msvc_assimp)
#endif()
find_package(ASSIMP)
if( NOT ASSIMP_FOUND )
  pkg_check_modules(ASSIMP assimp)
endif()

if( ASSIMP_FOUND )
  message(STATUS "assimp version ${ASSIMP_VERSION}, ${ASSIMP_PACKAGE_VERSION}, ${ASSIMP_INCLUDE_DIRS}")
  # For older versions of libassimp2, 
  # like the one in Ubuntu 12.04 
  set(CMAKE_REQUIRED_LIBRARIES assimp) 
  check_cxx_source_compiles(" 
  #include <assimp/Logger.h> 
  int main() 
  { 
      Assimp::Logger::WARN; 
      return 0; 
  }" 
  IS_ASSIMP_PRE_R896 
  )
  set(CMAKE_REQUIRED_LIBRARIES)
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
  endif()
endif()

check_library_exists(rt clock_gettime "" CLOCK_GETTIME_FOUND)
if( CLOCK_GETTIME_FOUND )
  add_definitions(-DCLOCK_GETTIME_FOUND)
endif()

find_package(RapidJSON)
if( RAPIDJSON_FOUND )
  include_directories(${RAPIDJSON_INCLUDES})
else()
  message(FATAL_ERROR "rapidjson library not found")
endif()

add_library(openrave-msgpack INTERFACE IMPORTED)
find_package(msgpack-cxx 6.0.0)
if (NOT msgpack-cxx_FOUND)
  pkg_check_modules(msgpack msgpack)
  target_link_libraries(openrave-msgpack INTERFACE ${msgpack_LIBRARIES})
else()
  message(STATUS "msgpack-cxx ${msgpack-cxx_VERSION}")
  target_link_libraries(openrave-msgpack INTERFACE msgpack-cxx)
endif()
if (msgpack-cxx_FOUND OR msgpack_FOUND)
  set(OPENRAVE_MSGPACK 1)
else()
  if (OPT_MSGPACK)
    message(FATAL_ERROR "msgpack feature was requested but msgpack libraries could not be found.")
  else()
    set(OPENRAVE_MSGPACK 0)
  endif()
endif()

find_package(LibXml2 REQUIRED)

add_library(openrave_gpgme IMPORTED INTERFACE)
if (OPT_ENCRYPTION)
  find_package(Gpgmepp 1.13 REQUIRED)
  target_link_libraries(openrave_gpgme INTERFACE Gpgmepp)
  target_compile_definitions(openrave_gpgme INTERFACE "OPENRAVE_ENCRYPTION=1")
endif()

# for qtcoin plugin.
# it is necessary to check here since we would like the qt version stored in the openrave-config file
if( WIN32 )
  if( NOT ENV{QTDIR} )
    # should fix this
    message(STATUS "Setting QTDIR=C:/Qt/4.7.1, msvc qt installer by default installs to C:/Qt")
    set(ENV{QTDIR} "C:/Qt/4.7.1")
  endif()
endif()

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL)
if (NOT (Qt5Gui_FOUND AND Qt5Core_FOUND AND Qt5Widgets_FOUND AND Qt5OpenGL_FOUND))
  message(STATUS "WARNING: Qt5 not found, is your QTDIR enviornment variable set?")
  find_package(Qt5Core)
  find_package(Qt5Gui)
  if( Qt5Core_FOUND AND Qt5Gui_FOUND )
    message(STATUS "Found Qt5")
  else()
    message(STATUS "Disabling QtCoin GUI plugin")
  endif()
endif ()

# always include libpcrecpp since we need it for URL parsing
pkg_check_modules(libpcrecpp libpcrecpp)
if( libpcrecpp_FOUND )
  set(CMAKE_REQUIRED_INCLUDES ${libpcrecpp_INCLUDE_DIRS} ${REQUIRED_INCLUDES})
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

  set(libpcrecpp_FOUND 1)
endif()

if( OPT_COLLADA )
  if( MSVC )
    # MSVC has prepackaged collada libraries
    set(COLLADA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/msvc_collada)
  endif()
  #find_package(COLLADA_DOM 2.4 COMPONENTS 1.5 1.4 PATHS ${COLLADA_PATH})
  find_package(COLLADA_DOM 2.3 COMPONENTS 1.5 PATHS ${COLLADA_PATH})

  if( COLLADA_DOM_FOUND )
    if( COLLADA_DOM_VERSION VERSION_LESS "2.5.0" )
      message(STATUS "collada version ${COLLADA_DOM_VERSION} does not support write to memory")
      set(COLLADA_SUPPORT_WRITE_MEMORY 0)
    else()
      message(STATUS "collada version ${COLLADA_DOM_VERSION} supports write to memory")
      set(COLLADA_SUPPORT_WRITE_MEMORY 1)
    endif()
  endif()
  
  find_package(ZLIB)
  if( NOT ZLIB_FOUND )
    message(STATUS "compiling zlib from souces")
    # compile from sources
    add_subdirectory(3rdparty/zlib)
  endif()

  pkg_check_modules(minizip minizip)
  if(minizip_FOUND)
    set(MINIZIP_INCLUDE_DIR ${minizip_INCLUDE_DIRS})
  else()
    add_subdirectory(3rdparty/minizip)
    set(MINIZIP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/minizip ${ZLIB_INCLUDE_DIR})
  endif()

  if( COLLADA_DOM_FOUND )
    if( MSVC )
      # have to copy the DLLs, but not the other stuff in the lib dir
      file(GLOB collada_dlls "${COLLADA_DOM_LIBRARY_DIRS}/*-${MSVC_PREFIX}-*.dll")
      install(FILES ${collada_dlls} DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base)
    endif()
  else()
    if( ZLIB_FOUND AND Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
      add_subdirectory(3rdparty/collada-2.4.0)
    else()
      set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COLLADA_DOM_LIBRARY_DIRS})
    endif()
  endif()
endif()

if( COLLADA_DOM_FOUND )
  # because several programs rely on linking with openrave-core, need to expose the link library
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COLLADA_DOM_LIBRARY_DIRS})
else()
  message(STATUS "no COLLADA support found")
endif()

link_directories(${OPENRAVE_LINK_DIRS})

# have to rely on the ivcon converter as a backup plan since assimp does not support iv/vrml
add_subdirectory(3rdparty/ivcon)
set(IVCON_FOUND 1)
set(IVCON_LIBRARY ivcon)
set(IVCON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ivcon")

## check qhull library
check_library_exists(qhull_r qh_new_qhull "" QHULL_REENTRANT_FOUND_LIB)
check_include_file(libqhull_r/libqhull_r.h HAVE_QHULL_REENTRANT_H)
if( QHULL_REENTRANT_FOUND_LIB AND HAVE_QHULL_REENTRANT_H )
    set(QHULL_FOUND 1)
    set(QHULL_USE_REENTRANT 1)
    set(QHULL_INCLUDE_DIR)
    message(STATUS "Use reentrant qhull")
else()
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

#if( OPT_FLANN )
#  find_package(FLANN)
#  if( NOT FLANN_FOUND)
#    ## check FLANN library
#    set(BUILD_MATLAB_BINDINGS false)
#    if( OPT_PYTHON AND PYTHON_EXECUTABLE )
#      set(BUILD_PYTHON_BINDINGS true)
#    else()
#      set(BUILD_PYTHON_BINDINGS false)
#    endif()
#    set(BUILD_C_BINDINGS true)
#    message(STATUS "compiling local flann library")
#    add_subdirectory(3rdparty/flann-1.6.6)
#    set(FLANN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/flann-1.6.6/src/cpp")
#    set(FLANN_FOUND 1) # use flann_cpp_s to link statically
#  endif()
#endif()

# function expression parser library
find_package(fparser 4.5)
if( fparser_FOUND )
    message(STATUS "Found fparser, version ${fparser_VERSION}")
    set(FPARSER_INCLUDE_DIRS "${fparser_INCLUDE_DIR}")
    set(FPARSER_LIBRARIES "${fparser_LIBRARY}")
    set(FPARSER_CXX_FLAGS "${fparser_CXX_FLAGS} -DOPENRAVE_FPARSER_SETEPSILON")
    set(FPARSER_LINK_FLAGS "${fparser_LINK_FLAGS}")
else()
  if( PKG_CONFIG_FOUND )
    pkg_search_module(FPARSER fparser)
  endif()
  if( NOT FPARSER_FOUND )
    message(STATUS "compiling local fparser library")
    add_subdirectory(3rdparty/fparser-4.5.2)
    set(FPARSER_CXX_FLAGS "${FPARSER_CXX_FLAGS} -DOPENRAVE_FPARSER_SETEPSILON")
  else()
    set(FPARSER_CXX_FLAGS "${FPARSER_CFLAGS}")
    set(FPARSER_LINK_FLAGS "${FPARSER_LDFLAGS}")
    message(STATUS "using fparser from pkg-config, cflags: ${FPARSER_CFLAGS}, link flags: ${FPARSER_LINK_FLAGS}")
    set(FPARSER_CXX_FLAGS "${FPARSER_CXX_FLAGS} -DOPENRAVE_FPARSER_SETEPSILON")
  endif()
endif()

# log4cxx logging lib
set(OPENRAVE_LOG4CXX 0)
set(OPENRAVE_LOG4CXX_INCLUDE_DIRS)
set(OPENRAVE_LOG4CXX_LIB_DIRS)
set(OPENRAVE_LOG4CXX_LIBRARY)
if( OPT_LOG4CXX )
  pkg_check_modules(LOG4CXX liblog4cxx)
  if( LOG4CXX_FOUND )
    if( MSVC )
      set(OPENRAVE_LOG4CXX_INCLUDE_DIRS "/I\"${LOG4CXX_INCLUDEDIR}\"")
      set(OPENRAVE_LOG4CXX_LIB_DIRS "/LIBPATH:\"${LOG4CXX_LIBDIR}\"")
    else()
      set(OPENRAVE_LOG4CXX_INCLUDE_DIRS "-I${LOG4CXX_INCLUDEDIR}")
      set(OPENRAVE_LOG4CXX_LIB_DIRS "-L${LOG4CXX_LIBDIR}")
    endif()
    set(OPENRAVE_LOG4CXX_LIBRARY "-l${LOG4CXX_LIBRARIES}")

    include_directories(${LOG4CXX_INCLUDEDIR})
    set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${LOG4CXX_LIBDIR})
    set(OPENRAVE_LOG4CXX 1)
  endif()
endif()

# libcurl for downloading resources
set(OPENRAVE_CURL 0)
set(OPENRAVE_CURL_LIBRARIES)
if( OPT_CURL )
  find_package(CURL)
  if ( CURL_FOUND )
    set(OPENRAVE_CURL_LIBRARIES ${CURL_LIBRARIES})
    set(OPENRAVE_CURL 1)
  endif()
endif()

link_directories(${OPENRAVE_LINK_DIRS})

message(STATUS "compiling local convexdecomposition library")
add_subdirectory(3rdparty/convexdecomposition)
set(CONVEXDECOMPOSITION_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/convexdecomposition")
if( UNIX )
  set(CONVEXDECOMPOSITION_CFLAGS "-DLINUX")
else()
  set(CONVEXDECOMPOSITION_CFLAGS "")
endif()
set(CONVEXDECOMPOSITION_FOUND 1)

if( Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
  add_definitions(-DHAVE_BOOST_FILESYSTEM)
endif()

add_subdirectory(src)
add_subdirectory(octave_matlab)
add_subdirectory(locale)

# Python version agnostic
if( NOT WIN32 AND OPENRAVE_BIN_SUFFIX )
  set(OPENRAVEPY_VER_NAME "_openravepy_${OPENRAVE_VERSION_MAJOR}_${OPENRAVE_VERSION_MINOR}")
else()
  set(OPENRAVEPY_VER_NAME "_openravepy_")
endif()

if (OPT_PYTHON)
  find_package(Python2 2.7 REQUIRED COMPONENTS Interpreter Development NumPy)
  message(STATUS "Python 2 version is ${Python2_VERSION}")
  execute_process(
    COMMAND ${Python2_EXECUTABLE} -c "import distutils.sysconfig as sysconf; import os; \
      print(os.path.relpath(sysconf.get_python_lib(1, prefix='${CMAKE_INSTALL_PREFIX}'), '${CMAKE_INSTALL_PREFIX}'))"
      OUTPUT_VARIABLE OPENRAVE_PYTHON2_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  message(STATUS "Python2 site-packages path is ${OPENRAVE_PYTHON2_INSTALL_DIR}")
  set(OPENRAVEPY2_INSTALL_DIR "${OPENRAVE_PYTHON2_INSTALL_DIR}/openravepy" CACHE PATH "OpenRAVE Python bindings installation directory")
  set(OPENRAVEPY2_VER_INSTALL_DIR "${OPENRAVEPY2_INSTALL_DIR}/${OPENRAVEPY_VER_NAME}") # used by openrave-config.cmake

  # Backward compat
  set(OPENRAVE_PYTHON_INSTALL_DIR ${OPENRAVE_PYTHON2_INSTALL_DIR})
  set(OPENRAVEPY_INSTALL_DIR      ${OPENRAVEPY2_INSTALL_DIR})
  set(OPENRAVEPY_VER_INSTALL_DIR  ${OPENRAVEPY2_VER_INSTALL_DIR})
  set(PYTHON_EXECUTABLE           ${Python2_EXECUTABLE})

  # get the sympy version
  execute_process(
    COMMAND ${Python2_EXECUTABLE} -c "import sympy; print(sympy.__version__)"
    OUTPUT_VARIABLE _sympy_version OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _sympy_version_failed
  )
  set(OPENRAVE_USE_LOCAL_SYMPY 1)
  if( ${_sympy_version_failed} EQUAL 0 )
    message(STATUS "Found sympy version '${_sympy_version}'")
    if( "${_sympy_version}" VERSION_GREATER "0.6.3" OR "${_sympy_version}" VERSION_EQUAL "0.6.3" )
      set(OPENRAVE_USE_LOCAL_SYMPY 0)
      if( "${_sympy_version}" VERSION_LESS "0.6.7" OR "${_sympy_version}" VERSION_EQUAL "0.6.7" )
        message(STATUS "Found sympy version 0.6.x, will patch this instead of installing local version")
      else()
        message(STATUS "Assuming sympy version 0.7.x or greater")
      endif()
    endif()
  else()
      message(STATUS "failed to find python sympy system installation")
  endif()

  if( OPENRAVE_USE_LOCAL_SYMPY )
    message(STATUS "System sympy (v=${_sympy_version}) is not right version, using local sympy")
    # extract sympy
    if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sympy/__init__.py" OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sympy/solvers/tests")
      message(STATUS "extracting sympy to ${CMAKE_CURRENT_SOURCE_DIR}")
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/sympy_0.7.1.tgz"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/sympy" DESTINATION ${OPENRAVE_PYTHON2_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python PATTERN ".svn" EXCLUDE)
  endif()

  add_library(openravepy2_meta INTERFACE)
  target_include_directories(openravepy2_meta INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/python/bindings/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/python/bindings/include>
    $<BUILD_INTERFACE:${OPENRAVE_INCLUDE_LOCAL_DIRS}>
    $<BUILD_INTERFACE:${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${OPENRAVEPY2_VER_INSTALL_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}>
  )
  install(TARGETS openravepy2_meta EXPORT openrave-targets)

  set(PYBIND11_NOPYTHON ON)
  find_package(pybind11 2.9.2) # 2.9.2 is the last version of pybind11 to support Python 2.

  if (pybind11_FOUND)
    target_link_libraries(openravepy2_meta INTERFACE rt Python2::Module Python2::NumPy pybind11::module libopenrave libopenrave-core)
    target_compile_options(openravepy2_meta INTERFACE "-fvisibility=hidden") # pybind11 requires less visibility
    target_compile_definitions(openravepy2_meta INTERFACE "OPENRAVE_CORE_DLL=1" "USE_PYBIND11_PYTHON_BINDINGS=1")
    set(USE_PYBIND11_PYTHON_BINDINGS ON)
  else()
    find_package(Boost REQUIRED COMPONENTS python thread)
    target_link_libraries(openravepy2_meta INTERFACE rt Python2::Module Python2::NumPy Boost::python Boost::thread libopenrave libopenrave-core)
    target_compile_definitions(openravepy2_meta INTERFACE "OPENRAVE_CORE_DLL=1")
    set(USE_PYBIND11_PYTHON_BINDINGS OFF)
  endif()
endif()

if (OPT_PYTHON3)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
  message(STATUS "Python 3 version is ${Python3_VERSION}")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import distutils.sysconfig as sysconf; import os; \
      print(os.path.relpath(sysconf.get_python_lib(1, prefix='${CMAKE_INSTALL_PREFIX}'), '${CMAKE_INSTALL_PREFIX}'))"
      OUTPUT_VARIABLE OPENRAVE_PYTHON3_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  message(STATUS "Python3 site-packages path is ${OPENRAVE_PYTHON3_INSTALL_DIR}")
  set(OPENRAVEPY3_INSTALL_DIR "${OPENRAVE_PYTHON3_INSTALL_DIR}/openravepy" CACHE PATH "OpenRAVE Python3 bindings installation directory")
  set(OPENRAVEPY3_VER_INSTALL_DIR "${OPENRAVEPY3_INSTALL_DIR}/${OPENRAVEPY_VER_NAME}") # used by openrave-config.cmake

  # Set only if Python2 is not enabled.
  if (NOT OPT_PYTHON)
    set(OPENRAVE_PYTHON_INSTALL_DIR ${OPENRAVE_PYTHON3_INSTALL_DIR})
    set(OPENRAVEPY_INSTALL_DIR      ${OPENRAVEPY3_INSTALL_DIR})
    set(OPENRAVEPY_VER_INSTALL_DIR  ${OPENRAVEPY3_VER_INSTALL_DIR})
    set(PYTHON_EXECUTABLE           ${Python3_EXECUTABLE})
  endif()

  add_library(openravepy3_meta INTERFACE)
  target_include_directories(openravepy3_meta INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/python/bindings/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/python/bindings/include>
    $<BUILD_INTERFACE:${OPENRAVE_INCLUDE_LOCAL_DIRS}>
    $<BUILD_INTERFACE:${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${OPENRAVEPY3_VER_INSTALL_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}>
  )
  install(TARGETS openravepy3_meta EXPORT openrave-targets)

  set(PYBIND11_NOPYTHON ON)
  find_package(pybind11 2.9.2) # 2.9.2 is the last version of pybind11 to support Python 2.

  if (pybind11_FOUND)
    target_link_libraries(openravepy3_meta INTERFACE rt Python3::Module Python3::NumPy pybind11::module libopenrave libopenrave-core)
    target_compile_options(openravepy3_meta INTERFACE "-fvisibility=hidden") # pybind11 requires less visibility
    target_compile_definitions(openravepy3_meta INTERFACE "OPENRAVE_CORE_DLL=1" "USE_PYBIND11_PYTHON_BINDINGS=1")
    set(USE_PYBIND11_PYTHON3_BINDINGS ON)
  else()
    find_package(Boost REQUIRED COMPONENTS python thread)
    target_link_libraries(openravepy3_meta INTERFACE rt Python3::Module Python3::NumPy Boost::python Boost::thread libopenrave libopenrave-core)
    target_compile_definitions(openravepy3_meta INTERFACE "OPENRAVE_CORE_DLL=1")
    set(USE_PYBIND11_PYTHON3_BINDINGS OFF)
  endif()
endif()

if (OPT_PYTHON OR OPT_PYTHON3)
  add_subdirectory(python)
  if (pybind11_FOUND)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/bindings/include/openravepy/pybind11 DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy)
  else()
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/bindings/include/openravepy/boostpython DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR}/openravepy)
  endif()
endif()

set(OPENRAVE_PYTHON_INSTALL_ABSOLUTE_DIR "${CMAKE_INSTALL_PREFIX}/${OPENRAVE_PYTHON_INSTALL_DIR}" CACHE PATH "Absolute dir for python on target OS path, used to write to header file")
message(STATUS "OPENRAVE_PYTHON_INSTALL_DIR   = ${OPENRAVE_PYTHON_INSTALL_DIR}")

# plugins have to be after python since they could use openravepy extensions
if(OPT_PLUGINS)
  add_subdirectory(plugins)
endif()

if( WIN32 )
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave-config.cpp.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cpp" IMMEDIATE @ONLY)
  add_executable(openrave-config "${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cpp")
  set_target_properties(openrave-config PROPERTIES OUTPUT_NAME openrave${OPENRAVE_BIN_SUFFIX}-config)
  install(TARGETS openrave-config DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base)
else()
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave-config.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_BIN_SUFFIX}-config" IMMEDIATE @ONLY)
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_BIN_SUFFIX}-config DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base)
endif()
if( OPT_BUILD_PACKAGE_DEFAULT AND OPENRAVE_BIN_SUFFIX )
  # create all the directories ahead of time, or otherwise WORKING_DIRECTORY will not work
  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory \${CMAKE_INSTALL_PREFIX}/bin COMMAND ${CMAKE_COMMAND} -E make_directory \${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig COMMAND ${CMAKE_COMMAND} -E make_directory \${CMAKE_INSTALL_PREFIX}/${BASH_COMPLETION_DIR})" COMPONENT openrave)
  InstallSymlink(openrave${OPENRAVE_BIN_SUFFIX}-config ${CMAKE_INSTALL_PREFIX}/bin/openrave-config)
endif()

# Write and install cmake export control files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/openrave-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cmake
  INSTALL_DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/openrave-${OPENRAVE_SOVERSION}
  PATH_VARS
    RAPIDJSON_INCLUDES
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/openrave-config-version.cmake
  VERSION "${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.${OPENRAVE_VERSION_PATCH}"
  COMPATIBILITY AnyNewerVersion
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/openrave-config-version.cmake" DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openrave-${OPENRAVE_SOVERSION})
install(EXPORT openrave-targets NAMESPACE OpenRAVE:: FILE openrave-targets.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openrave-${OPENRAVE_SOVERSION})

# Install header files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/openrave/     DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}/openrave)
if (OPT_CBINDINGS)
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/openrave_c/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}/openrave_c)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/openrave/config.h IMMEDIATE @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/openrave/config.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}/openrave)
install(FILES COPYING LICENSE.lgpl LICENSE.apache DESTINATION ${OPENRAVE_SHARE_DIR})

if( UNIX OR CYGWIN)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave_completion.bash.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave_completion.bash" @ONLY IMMEDIATE)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openrave_completion.bash" DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}base )
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_BIN_SUFFIX}.pc" @ONLY IMMEDIATE)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_BIN_SUFFIX}.pc" DESTINATION lib${LIB_SUFFIX}/pkgconfig COMPONENT ${COMPONENT_PREFIX}dev)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave-core.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_BIN_SUFFIX}-core.pc" @ONLY IMMEDIATE)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_BIN_SUFFIX}-core.pc" DESTINATION lib${LIB_SUFFIX}/pkgconfig COMPONENT ${COMPONENT_PREFIX}dev)
  if( OPT_BUILD_PACKAGE_DEFAULT AND OPENRAVE_BIN_SUFFIX )
    InstallSymlink(openrave${OPENRAVE_BIN_SUFFIX}.pc ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig/openrave.pc)

    InstallSymlink(openrave${OPENRAVE_BIN_SUFFIX}-core.pc ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig/openrave-core.pc)


    if( BASH_COMPLETION_DIR )
      get_filename_component(BASH_COMPLETION_ABSOLUTE_DIR ${CMAKE_INSTALL_PREFIX}/${BASH_COMPLETION_DIR} ABSOLUTE)

      get_filename_component(BASH_COMPLETION_SHARE_DIR ${CMAKE_INSTALL_PREFIX}/${OPENRAVE_SHARE_DIR} ABSOLUTE)

      file(RELATIVE_PATH BASH_COMPLETION_RELATIVE_DIR ${BASH_COMPLETION_ABSOLUTE_DIR} ${BASH_COMPLETION_SHARE_DIR})

      InstallSymlink(${BASH_COMPLETION_RELATIVE_DIR}/openrave_completion.bash ${BASH_COMPLETION_ABSOLUTE_DIR}/openrave_completion.bash)

    endif()
  endif()
  # don't need symlinks
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave.bash.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave.bash" IMMEDIATE @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/openrave.bash DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}base)
endif()

# add make uninstall capability
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/modules-cmake/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

if(CMAKE_CPACK_COMMAND AND UNIX AND OPT_BUILD_PACKAGES)
  # Packing information
  set(CPACK_PACKAGE_NAME OpenRAVE)
  set(CPACK_PACKAGE_CONTACT "OpenRAVE Testing (Testing and Releasing of OpenRAVE Packages) <openrave.testing@gmail.com>" CACHE STRING "Package maintainer and PGP signer.")
  set(CPACK_PACKAGE_VENDOR "http://openrave.org")
  set(CPACK_PACKAGE_DISPLAY_NAME "OpenRAVE ${OPENRAVE_VERSION}")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "- Open Robotics Automation Virtual Environment")
  if( OPT_DOUBLE_PRECISION )
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${CPACK_PACKAGE_DESCRIPTION_SUMMARY} using double precision")
  else()
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${CPACK_PACKAGE_DESCRIPTION_SUMMARY} using single precision")
  endif()
  set(CPACK_PACKAGE_VERSION "${OPENRAVE_VERSION}.${PACKAGE_VERSION}")
  set(CPACK_PACKAGE_VERSION_MAJOR ${OPENRAVE_VERSION_MAJOR})
  set(CPACK_PACKAGE_VERSION_MINOR ${OPENRAVE_VERSION_MINOR})
  set(CPACK_PACKAGE_VERSION_PATCH ${OPENRAVE_VERSION_PATCH})
  set(CPACK_PACKAGE_INSTALL_DIRECTORY "openrave${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}" CACHE STRING "CPack install directory")
  set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/docs/source/description.rst)
  set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/COPYING)

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}BASE_REQUIRED 1)
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}BASE_DISPLAY_NAME "core libraries and tools")
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}DATA_DISPLAY_NAME "basic robots, models, and scene files used in the examples")
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}DEV_DISPLAY_NAME "development files and examples")
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}DEV_DEPENDS ${COMPONENT_PREFIX}base libboost-thread-dev libboost-date-time-dev)

  # necessary to add a circular dependency to ${CPACK_DEBIAN_PACKAGE_NAME}?
  foreach(OPENRAVE_COMPONENT ${CPACK_COMPONENTS_ALL})
    string(TOUPPER ${OPENRAVE_COMPONENT} UPPER_COMPONENT)
    set( CPACK_COMPONENT_${UPPER_COMPONENT}_DEPENDS ${CPACK_COMPONENT_${UPPER_COMPONENT}_DEPENDS} openrave-minimal)
  endforeach()

  # base
  set(CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU debhelper cmake python-dev libxml2-dev libboost-dev ffmpeg zlib1g-dev python-numpy "python-sympy (>=0.6.3)" desktop-file-utils libboost-filesystem-dev libboost-system-dev libboost-python-dev libboost-thread-dev libboost-date-time-dev "octave3.2-headers|liboctave-dev" "collada-dom-dev (>=2.4.0)" pkg-config "fparser-dev (>=4.4.3)" libhdf5-serial-dev liblapack-dev gettext)
  # add once infinite precision computations are enabled:
  # libmpfr-dev libgmp3-dev libmpfi-dev libgmpxx4ldbl python-gmpy
  # plugins:
  set(CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU ${CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU} libboost-iostreams-dev libboost-regex-dev qtbase5-dev qtbase5-dev-tools libqhull-dev libavcodec-dev libavformat-dev libswscale-dev libsimage-dev libode-dev libsoqt4-dev "assimp-dev|libassimp-dev" libbullet-dev)# libopenscenegraph-dev)

  # debian
  set(CPACK_DEBIAN_PACKAGE_PRIORITY optional)
  set(CPACK_DEBIAN_PACKAGE_SECTION devel)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS openrave-minimal)
  set(CPACK_DEBIAN_PACKAGE_SUGGESTS cmake)
  set(CPACK_DEBIAN_PACKAGE_RECOMMENDS ${CPACK_COMPONENTS_ALL})
  set(CPACK_DEBIAN_CMAKE_OPTIONS "-DBASH_COMPLETION_DIR=../etc/bash_completion.d -DOPT_DOUBLE_PRECISION=${OPT_DOUBLE_PRECISION} -DOPT_BUILD_PACKAGE_DEFAULT=${OPT_BUILD_PACKAGE_DEFAULT}")
  set(CPACK_DEBIAN_PACKAGE_REMOVE_SOURCE_FILES )
  set(CPACK_DEBIAN_PACKAGE_SOURCE_COPY "${CMAKE_SOURCE_DIR}/release/copydebfiles.py")
  set(CPACK_DEBIAN_CHANGELOG "  * ChangeLog can be found at https://openrave.svn.sourceforge.net/svnroot/openrave/tags/${OPENRAVE_VERSION}/docs/source/changelog.rst\n\n")
  execute_process(COMMAND lsb_release -is
    OUTPUT_VARIABLE _lsb_distribution OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _lsb_release_failed)
  set(CPACK_DEBIAN_DISTRIBUTION_NAME ${_lsb_distribution} CACHE STRING "Name of the distrubiton")
  string(TOLOWER ${CPACK_DEBIAN_DISTRIBUTION_NAME} CPACK_DEBIAN_DISTRIBUTION_NAME)
  if( ${CPACK_DEBIAN_DISTRIBUTION_NAME} STREQUAL "ubuntu" )
    set(CPACK_DEBIAN_DISTRIBUTION_RELEASES lucid precise quantal raring saucy CACHE STRING "Release code-names of the distrubiton release")
  endif()
  if( OPT_BUILD_PACKAGE_DEFAULT )
    set(CPACK_COMPONENT_OPENRAVE_DEPENDS ${CPACK_DEBIAN_PACKAGE_NAME})
    set(CPACK_COMPONENT_OPENRAVE_DISPLAY_NAME "${CPACK_PACKAGE_DESCRIPTION_SUMMARY}")
    set(CPACK_COMPONENT_OPENRAVE-MINIMAL_DISPLAY_NAME "minimal shared files for all versions")
    set(CPACK_COMPONENT_OPENRAVE-MINIMAL_DESCRIPTION "Uninstall this package to remove all OpenRAVE installations")
    set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} openrave openrave-minimal openrave-python-minimal)
  endif()
  
  #TODO: extra robot CAD model files from: https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots

  set(DPUT_HOST "ppa:openrave/testing" CACHE STRING "PPA repository to upload the debian sources")
  include(CPack)
  include(DebSourcePPA)
endif()
