cmake_minimum_required (VERSION 2.6.0) # using import/export targets
project(OpenRAVE)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

# Define here the needed parameters
set (OPENRAVE_VERSION_MAJOR 0)
set (OPENRAVE_VERSION_MINOR 5)
set (OPENRAVE_VERSION_PATCH 0)
set (OPENRAVE_VERSION ${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.${OPENRAVE_VERSION_PATCH})
set (OPENRAVE_SOVERSION ${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR})
message(STATUS "Compiling OpenRAVE Version ${OPENRAVE_VERSION}, soversion=${OPENRAVE_SOVERSION}")

message(STATUS "Using cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" )
# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0002
cmake_policy(SET CMP0002 NEW)
# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0003
cmake_policy(SET CMP0003 NEW)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# When building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")

# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Custom CMake options
option(OPT_VIDEORECORDING "Enable video recording" ON)
option(OPT_PLUGINS "Build the pluings" ON)
option(OPT_DOUBLE_PRECISION "Use double precision" ON)
option(OPT_ACCURATEMATH "Use accurate and robust math to account for floating-point errors" ON)
option(OPT_PYTHON "Build python bindings" ON)
option(OPT_OCTAVE "Build octave bindings" ON)
option(OPT_MATLAB "Build matlab bindings" ON)
option(OPT_STATIC "Build static libraries" OFF)
option(OPT_COLLADA "Build with COLLADA support" ON)
option(OPT_BUILD_PACKAGES "Set to ON to generate CPack configuration files and packaging targets" OFF)
option(OPT_BUILD_PACKAGE_DEFAULT "Set to ON to generate a default openrave package that creates symlinks" ON)
option(OPT_FLANN "Temporary switch to force building of flann" OFF)

set(PACKAGE_VERSION "0" CACHE STRING "the package-specific version used for uploading the sources")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules-cmake")
set(CPACK_DEBIAN_PACKAGE_NAME openrave${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR})

if(OPT_DOUBLE_PRECISION)
  set(OPENRAVE_PRECISION 1)
  set(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_DEBIAN_PACKAGE_NAME}-dp")
  message(STATUS "Using double precision")
else()
  set(OPENRAVE_PRECISION 0)
  set(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_DEBIAN_PACKAGE_NAME}-sp")
  message(STATUS "Using single precision")
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

find_package(PkgConfig) # pkg_check_modules

set(OPENRAVE_PYTHON_INSTALL_DIR)

if( OPT_PYTHON )
  include(FindPythonInterp)
  ## check python
  find_package(PythonLibs) # using PYTHON_INCLUDE_PATH instead of PYTHON_INCLUDE_DIR
  if( NOT PYTHON_EXECUTABLE )
    # look specifically for 2.6
    find_program(PYTHON_EXECUTABLE NAMES python2.6 python PATHS [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\2.6\\InstallPath])
  endif()

  if( PYTHON_EXECUTABLE )
    # architecture independent
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print get_python_lib(0)"
      OUTPUT_VARIABLE _python_sitepackage OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _python_failed0)
    # architexture dependent
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print get_python_lib(1)"
      OUTPUT_VARIABLE _python_distpackage OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _python_failed1)
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; from os.path import relpath; print relpath(get_python_lib(1,prefix='${CMAKE_INSTALL_PREFIX}'),'${CMAKE_INSTALL_PREFIX}')"
      OUTPUT_VARIABLE OPENRAVE_PYTHON_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _python_failed2)

    if( ${_python_failed0} EQUAL 0 AND ${_python_failed1} EQUAL 0 AND ${_python_failed2} EQUAL 0 )
      if( NOT IS_DIRECTORY "${_python_distpackage}/numpy/core/include" )
        if( NOT IS_DIRECTORY "${_python_sitepackage}/numpy/core/include" )
          message(STATUS "cannot find python numpy dir!")
          set(PYTHON_EXECUTABLE)
        else()
          set(PYTHON_INCLUDE_PATH ${PYTHON_INCLUDE_PATH} ${_python_sitepackage_arch}/numpy/core/include)
        endif()
      else()
        set(PYTHON_INCLUDE_PATH ${PYTHON_INCLUDE_PATH} ${_python_distpackage}/numpy/core/include)
      endif()

      # get the major.minor python version
      execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "import sys; print('%d.%d'%sys.version_info[0:2])"
        OUTPUT_VARIABLE _python_version OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _python_failed)
      if( ${_python_failed} EQUAL 0 )
        string(REGEX REPLACE "[\r\n]" "" PYTHON_MAJORMINOR_VERSION "${_python_version}")
      else()
        message(STATUS "failed to get python version")
      endif()
    else()
      message(STATUS "failed to get python site-package directories via get_pytho_lib")
      set(PYTHON_EXECUTABLE)
    endif()
  endif()
endif()

if( WIN32 )
  # does not support symlinks
  set(OPENRAVE_BIN_SUFFIX)
else()
  set(OPENRAVE_BIN_SUFFIX ${OPENRAVE_SOVERSION} CACHE STRING "Add the MAJOR.MINOR openrave version suffixes to all executable installed files")
endif()

set(OPENRAVE_SHARE_DIR "share/openrave-${OPENRAVE_SOVERSION}")
set(OPENRAVE_DATA_INSTALL_DIR "${OPENRAVE_SHARE_DIR}" CACHE PATH "Scene and robot files installation directory")
set(OPENRAVE_PLUGINS_INSTALL_DIR "${OPENRAVE_SHARE_DIR}/plugins" CACHE PATH "OpenRAVE plugins installation directory")
set(OPENRAVEPY_INSTALL_DIR "${OPENRAVE_PYTHON_INSTALL_DIR}/openravepy" CACHE PATH "OpenRAVE Python bindings (openravepy) installation directory")
set(OPENRAVE_OCTAVE_INSTALL_DIR "${OPENRAVE_SHARE_DIR}/octave" CACHE PATH "OpenRAVE Octave bindings installation directory")
set(OPENRAVE_MATLAB_INSTALL_DIR "${OPENRAVE_SHARE_DIR}/matlab" CACHE PATH "OpenRAVE MATLAB bindings installation directory")

if( UNIX OR CYGWIN)
  # cache has to be STRING instead of PATH for path to remain relative
  set(BASH_COMPLETION_DIR CACHE STRING "Directory where bash completion files will be installed in, on linux it is /etc/bash_completion.d")# Additional CMake modules for 3rd party library checks reside here
endif()

message(STATUS "installing to ${CMAKE_INSTALL_PREFIX}, python subdir is ${OPENRAVE_PYTHON_INSTALL_DIR}, program suffix is '${OPENRAVE_BIN_SUFFIX}'")

set(CMAKE_TRY_COMPILE_CONFIGURATION Release CACHE STRING "very important to only test release since default is debug, and it usually reuquires special libraries")
set(REQUIRED_INCLUDES "" CACHE STRING "Includes to always force  when compiling")
include_directories(${REQUIRED_INCLUDES})

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif()

set(OPENRAVE_EXPORT_CXXFLAGS)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set(CMAKE_CXX_FLAGS_OPTIMIZED "-O2 -DNDEBUG -DBOOST_DISABLE_ASSERTS -D_SECURE_SCL=0") # this practically removes all checks making it a very dangerous options to play with
  set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG") 
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -D_DEBUG")

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

if( UNIX OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set(STDC_LIBRARY stdc++)
else()
  set(STDC_LIBRARY)
endif()

if( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # apple doesn't have 64bit versions of file opening functions, so add them
  add_definitions("-Dfopen64=fopen -Dfseeko64=fseeko -Dfseek64=fseek -Dftell64=ftell -Dftello64=ftello")
endif()

set(OPENRAVE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OPENRAVE_CORE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/libopenrave-core)
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
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}
      COMMAND ${CMAKE_COMMAND} -E chdir
      ARGS "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}.tgz"
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY}.tgz)
    add_custom_target(${MSVC_LIBRARY} ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${MSVC_LIBRARY})
  endforeach()

  check_include_file(stdint.h HAVE_STDINT_H)
  if( NOT HAVE_STDINT_H )
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/msvc_include/stdint.h DESTINATION include/openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}/openrave COMPONENT ${COMPONENT_PREFIX}dev)
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
  set(OPENRAVE_LIBRARY_SUFFIX "${OPENRAVE_SOVERSION}-${MSVC_PREFIX}-mt")
  
  # force multi-threaded DLL boost
  set(Boost_USE_MULTITHREAD ON)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_STATIC_RUNTIME OFF)
  set(Boost_CFLAGS "-DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
else()
  set(OPENRAVE_LIBRARY_SUFFIX "${OPENRAVE_SOVERSION}")
endif()

# only look for installed boost if not an old version of msvc
if(NOT MSVC71 AND NOT MSVC80)
  if( NOT $ENV{BOOST_INCLUDEDIR} STREQUAL "" )
    set(Boost_INCLUDE_DIR $ENV{BOOST_INCLUDEDIR})
  endif()
  if( NOT $ENV{BOOST_LIBRARYDIR} STREQUAL "" )
    set(Boost_LIBRARY_DIRS $ENV{BOOST_LIBRARYDIR})
  endif()
  set(Boost_ADDITIONAL_VERSIONS "1.46.1" "1.45" "1.44" "1.43" "1.42" "1.41" "1.40" "1.39" "1.38" "1.37.0" "1.37" "1.35.0" "1.34.1" "1.34.0" "1.34" "1.33.1" "1.33.0" "1.33" ${Boost_ADDITIONAL_VERSIONS})
  find_package(Boost COMPONENTS regex filesystem system python thread iostreams date_time)
else()
  # have to rely on local boost and a hack
  add_definitions(-DTEMP_SPIRIT_HACK)
endif()

if( Boost_FOUND )
  include_directories(${Boost_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${Boost_LIBRARY_DIRS})
elseif(Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0")
  include_directories(${Boost_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${Boost_LIBRARY_DIRS})
elseif( MSVC )
  # to facilitate compilation, visual studio libraries are included locally
  message(STATUS "using local boost libraries for MSVC")
  set(BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/msvc_boost")
  set(Boost_FOUND 1)
  set(Boost_INCLUDE_DIRS "${BOOST_ROOT}")
  set(Boost_INCLUDE_DIR "${BOOST_ROOT}")
  set(Boost_LIBRARY_DIRS "${BOOST_ROOT}/lib")
  set(Boost_REGEX_FOUND 0)
  set(Boost_FILESYSTEM_FOUND 1)
  set(Boost_IOSTREAMS_FOUND 1)
  set(Boost_SYSTEM_FOUND 1)
  set(Boost_PYTHON_FOUND 1)
  set(Boost_THREAD_FOUND 1)
  set(Boost_DATE_TIME_FOUND 1)
  set(Boost_SIGNALS_FOUND 1)
  set(Boost_VERSION "104400")
  # have to add to required flags for applications that will test linking with boost
  set(CMAKE_REQUIRED_FLAGS "-DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB")
  set(Boost_DATE_TIME_LIBRARY "${Boost_LIBRARY_DIRS}/boost_date_time-${MSVC_PREFIX}-mt-1_44.lib")
  set(Boost_THREAD_LIBRARY "${Boost_LIBRARY_DIRS}/boost_thread-${MSVC_PREFIX}-mt-1_44.lib")
  set(Boost_SYSTEM_LIBRARY "${Boost_LIBRARY_DIRS}/boost_system-${MSVC_PREFIX}-mt-1_44.lib")
  set(Boost_FILESYSTEM_LIBRARY "${Boost_LIBRARY_DIRS}/boost_filesystem-${MSVC_PREFIX}-mt-1_44.lib")
  set(Boost_PYTHON_LIBRARY "${Boost_LIBRARY_DIRS}/boost_python-${MSVC_PREFIX}-mt-1_44.lib")
  set(Boost_IOSTREAMS_LIBRARY "${Boost_LIBRARY_DIRS}/boost_iostreams-${MSVC_PREFIX}-mt-1_44.lib")
  install(DIRECTORY "${Boost_LIBRARY_DIRS}/" DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base FILES_MATCHING PATTERN "*-${MSVC_PREFIX}-*.dll")
  install(DIRECTORY "${Boost_LIBRARY_DIRS}/" DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}dev FILES_MATCHING PATTERN "*-${MSVC_PREFIX}-*.lib")
  install(DIRECTORY "${Boost_INCLUDE_DIR}/boost" DESTINATION include COMPONENT ${COMPONENT_PREFIX}dev)
  include_directories(${Boost_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${Boost_LIBRARY_DIRS})
else()
  message(FATAL_ERROR "Could not find boost libraries!")
endif()

message(STATUS "found boost version: ${Boost_VERSION}")
if( NOT Boost_THREAD_LIBRARY OR NOT Boost_DATE_TIME_LIBRARY )
  message(FATAL_ERROR "boost thread and date_time libraries are necessary")
endif()
if( OPT_PYTHON AND MSVC AND Boost_VERSION GREATER 104600 AND PYTHON_MAJORMINOR_VERSION STRGREATER "2.6")
  message(STATUS "Boost python library ${Boost_PYTHON_LIBRARY} might not be compiled for Python ${PYTHON_MAJORMINOR_VERSION}")
endif()
    
set(OPENRAVE_BOOST_INCLUDE_DIRS)
foreach(idir ${Boost_INCLUDE_DIRS})
  if( MSVC )
    set(OPENRAVE_BOOST_INCLUDE_DIRS "${OPENRAVE_BOOST_INCLUDE_DIRS} /I\"${idir}\"")
  else()
    set(OPENRAVE_BOOST_INCLUDE_DIRS "${OPENRAVE_BOOST_INCLUDE_DIRS} -I${idir}")
  endif()
endforeach()

set(OPENRAVE_BOOST_LIB_DIRS)
foreach(ldir ${Boost_LIBRARY_DIRS})
  if( MSVC )
    set(OPENRAVE_BOOST_LIB_DIRS "${OPENRAVE_BOOST_LIB_DIRS} /LIBPATH:\"${ldir}\"")
  else()
    set(OPENRAVE_BOOST_LIB_DIRS "${OPENRAVE_BOOST_LIB_DIRS} -L${ldir}")
  endif()
endforeach()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-fPIC) # this is a very important switch and some libraries seem now to have it....
  set(EXTRA_COMPILE_FLAGS "-fPIC")
else()
  set(EXTRA_COMPILE_FLAGS "")
endif()

# generate the md5 sum for all OpenRAVE interfaces
add_subdirectory(cpp-gen-md5)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/cpp-gen-md5")

if( MSVC )
  if( EXTRA_MSVC_DEPEND )
    add_dependencies(cpp-gen-md5 ${EXTRA_MSVC_DEPEND})
  endif()
  set(CPPGENMD5 cpp-gen-md5)
else()
  set(CPPGENMD5 "${CMAKE_CURRENT_BINARY_DIR}/cpp-gen-md5/cpp-gen-md5")
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
  DEPENDS cpp-gen-md5
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

# math libraries
find_package(GMP)
find_package(GMPXX)
find_package(MPFR)
find_package(MPFI)

if( OPT_ACCURATEMATH AND NOT MSVC )
  check_library_exists(crlibm crlibm_init "" CRLIBM_FOUND_LIB)
  check_include_file(crlibm.h HAVE_CRLIBM_H)
  if(NOT CRLIBM_FOUND_LIB OR NOT HAVE_CRLIBM_H)
    message(STATUS "Using local crlibm")
    add_subdirectory(3rdparty/crlibm-1.0beta4)
    set(CRLIBM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/crlibm-1.0beta4")
  endif()
  set(CRLIBM_FOUND 1)
  set(CRLIBM_LIBRARY crlibm)
else()
  set(CRLIBM_FOUND 0)
endif()

pkg_check_modules(ASSIMP assimp)

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

find_package(LibXml2)

if( LIBXML2_FOUND )
  include_directories(${LIBXML2_INCLUDE_DIR})
  add_definitions(${LIBXML2_DEFINITIONS})
  message(STATUS "libxml2 found")
else()
  if( MSVC )
    set(LIBXML2_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/msvc_libxml2/lib/libxml2-${MSVC_PREFIX}-mt.lib)
    set(LIBXML2_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/msvc_libxml2/include)
    set(LIBXML2_DEFINITIONS)
    set(LIBXML2_FOUND 1)
    include_directories(${LIBXML2_INCLUDE_DIR})
    # have to copy the DLLs, but not the other stuff in the lib dir
    file(GLOB libxml_dlls "${CMAKE_CURRENT_SOURCE_DIR}/msvc_libxml2/lib/*-${MSVC_PREFIX}-*.dll")
    install(FILES ${libxml_dlls} DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base)
  else()
    message(FATAL_ERROR "Could not find libxml2")
  endif()
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

find_package(Qt4 COMPONENTS QtCore QtGui)
if (NOT QT_FOUND)
  message(STATUS "WARNING: Qt4 not found, is your QTDIR enviornment variable set? Disabling QtCoin GUI plugin")
endif ()

link_directories(${OPENRAVE_LINK_DIRS})

if( OPT_COLLADA )
  if( MSVC )
    # MSVC has prepackaged collada libraries
    set(COLLADA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/msvc_collada)
  else()
    # this is temporary since collada libraries changed (11/10/20)
    set(COLLADA_DOM_LIBRARIES "" CACHE INTERNAL "collada libraries directories" FORCE)
    set(COLLADA_DOM_DIR "" CACHE PATH "The directory containing a CMake configuration file for COLLADA_DOM." FORCE)
    set(COLLADA_PATH "" CACHE PATH "collada root path" FORCE)
  endif()
  find_package(COLLADA_DOM 2.3 COMPONENTS 1.5 PATHS ${COLLADA_PATH})
  
  find_package(ZLIB)
  if( NOT ZLIB_FOUND )
    message(STATUS "compiling zlib from souces")
    # compile from sources
    add_subdirectory(3rdparty/zlib)
    set(ZLIB_FOUND 1)
    set(ZLIB_LIBRARIES zlib)
    set(ZLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/zlib)
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
        set(libpcrecpp_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/pcre-8.02)
        set(libpcrecpp_LIBRARY_DIRS)
        set(libpcrecpp_LIBRARIES pcrecpp_local)
        set(libpcrecpp_CFLAGS_OTHERS "-DPCRE_STATIC")
        set(libpcrecpp_LDFLAGS_OTHERS)
      endif()

      add_subdirectory(3rdparty/collada)
      set(COLLADA_DOM_FOUND 1)
      set(COLLADA_DOM_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/collada/include ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/collada/include/1.5)
      set(COLLADA_DOM_LIBRARY_DIRS)
      set(COLLADA_DOM_LIBRARIES collada15reader)
      set(COLLADA_DOM_CFLAGS_OTHER)
      set(COLLADA_DOM_LDFLAGS_OTHER)
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

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/openrave/config.h IMMEDIATE @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

add_subdirectory(src)
add_subdirectory(octave_matlab)

if(OPT_PYTHON AND PYTHON_EXECUTABLE)
  set(OPENRAVE_USE_LOCAL_SYMPY 1)
  if( PYTHON_EXECUTABLE )
    set(SYMPY_MINIMUM_VERSION "0.6.3")
    set(SYMPY_MAXIMUM_VERSION "0.6.7")
    # check if sympy substitution is ok
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "import sympy, sys;  x=sympy.Symbol('x'); sys.exit((x**3).subs(x**2,sympy.Symbol('y')) != x**3)"
      OUTPUT_VARIABLE _sympy_check OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _sympy_check_failed)
    if( ${_sympy_check_failed} EQUAL 0 )
      set(OPENRAVE_USE_LOCAL_SYMPY 0)
    else()
      # get the sympy version
      execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "import sympy; print sympy.__version__"
        OUTPUT_VARIABLE _sympy_version OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _sympy_version_failed)
      if( ${_sympy_version_failed} EQUAL 0 )
        string(REGEX REPLACE "[\r\n]" "" _sympy_version "${_sympy_version}")
        if( "${_sympy_version}" VERSION_GREATER "${SYMPY_MINIMUM_VERSION}" OR "${_sympy_version}" VERSION_EQUAL "${SYMPY_MINIMUM_VERSION}" )
          if( "${_sympy_version}" VERSION_LESS "${SYMPY_MAXIMUM_VERSION}" OR "${_sympy_version}" VERSION_EQUAL "${SYMPY_MAXIMUM_VERSION}" )
            message(STATUS "Found sympy version ${_sympy_version} >= ${SYMPY_MINIMUM_VERSION}, will patch this instead of installing local version (could have been previously installed by openrave?)")
            set(OPENRAVE_USE_LOCAL_SYMPY 0)
          endif()
        endif()
      else()
        message(STATUS "failed to find python sympy system installation")
      endif()
    endif()
  endif()

  if( OPENRAVE_USE_LOCAL_SYMPY )
    message(STATUS "System sympy (v=${_sympy_version}) is not right version, using local sympy")
    # extract sympy
    if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sympy/__init__.py" OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sympy/solvers/tests")
      message(STATUS "extracting sympy to ${CMAKE_CURRENT_SOURCE_DIR}")
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/sympy_0.6.7.tgz"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/sympy" DESTINATION ${OPENRAVE_PYTHON_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python PATTERN ".svn" EXCLUDE)
  endif()

  add_subdirectory(python)
endif()

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
  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink openrave${OPENRAVE_BIN_SUFFIX}-config openrave-config WORKING_DIRECTORY \${CMAKE_INSTALL_PREFIX}/bin)" COMPONENT openrave)
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave-config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cmake" IMMEDIATE @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave-config-version.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave-config-version.cmake" IMMEDIATE @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openrave-config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/openrave-config-version.cmake" DESTINATION "lib${LIB_SUFFIX}/cmake/openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}" COMPONENT ${COMPONENT_PREFIX}dev)

if( UNIX OR CYGWIN)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave_completion.bash.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave_completion.bash" @ONLY IMMEDIATE)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openrave_completion.bash" DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}base )
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/openrave.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.pc" @ONLY IMMEDIATE)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openrave${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.pc" DESTINATION lib${LIB_SUFFIX}/pkgconfig COMPONENT ${COMPONENT_PREFIX}dev)
  if( OPT_BUILD_PACKAGE_DEFAULT AND OPENRAVE_BIN_SUFFIX )
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink openrave${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}.pc openrave.pc WORKING_DIRECTORY \${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig)" COMPONENT openrave)
    if( BASH_COMPLETION_DIR )
      install(CODE "get_filename_component(BASH_COMPLETION_ABSOLUTE_DIR \${CMAKE_INSTALL_PREFIX}/${BASH_COMPLETION_DIR} ABSOLUTE)" COMPONENT openrave)
      install(CODE "get_filename_component(BASH_COMPLETION_SHARE_DIR \${CMAKE_INSTALL_PREFIX}/${OPENRAVE_SHARE_DIR} ABSOLUTE)" COMPONENT openrave)
      install(CODE "file(RELATIVE_PATH BASH_COMPLETION_RELATIVE_DIR \${BASH_COMPLETION_ABSOLUTE_DIR} \${BASH_COMPLETION_SHARE_DIR})" COMPONENT openrave)
      install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \${BASH_COMPLETION_RELATIVE_DIR}/openrave_completion.bash openrave_completion.bash WORKING_DIRECTORY \${BASH_COMPLETION_ABSOLUTE_DIR})" COMPONENT openrave)
    endif()
  endif()
endif()

file(GLOB rave_header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/openrave/*.h)
install(FILES ${rave_header_files} ${interfacehashes_h} ${CMAKE_CURRENT_BINARY_DIR}/include/openrave/config.h DESTINATION include/openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}/openrave COMPONENT ${COMPONENT_PREFIX}dev)
install(FILES rave/rave.h rave/plugin.h DESTINATION include/openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}/rave COMPONENT ${COMPONENT_PREFIX}dev) # deprecated
install(FILES COPYING LICENSE.lgpl LICENSE.apache DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}base)

#install(EXPORT openrave-targets DESTINATION lib/cmake/openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR})

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
  set(CPACK_PACKAGE_INSTALL_DIRECTORY "openrave${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR}")
  set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/docs/en/description.rst)
  set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/COPYING)

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}BASE_REQUIRED 1)
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}BASE_DISPLAY_NAME "core libraries and tools")
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}DATA_DISPLAY_NAME "basic robots, models, and scene files used in the examples")
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}DEV_DISPLAY_NAME "development files and examples")
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}DEV_DEPENDS ${COMPONENT_PREFIX}base libboost-thread-dev libboost-date-time-dev)

  # add a circular dependency to ${CPACK_DEBIAN_PACKAGE_NAME}
  foreach(OPENRAVE_COMPONENT ${CPACK_COMPONENTS_ALL})
    string(TOUPPER ${OPENRAVE_COMPONENT} UPPER_COMPONENT)
    set( CPACK_COMPONENT_${UPPER_COMPONENT}_DEPENDS ${CPACK_COMPONENT_${UPPER_COMPONENT}_DEPENDS} ${CPACK_DEBIAN_PACKAGE_NAME} openrave-minimal)
  endforeach()

  # base
  set(CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU debhelper cmake python-dev libxml2-dev libboost-dev ffmpeg zlib1g-dev python-numpy "python-sympy (>=0.6.3)" desktop-file-utils libboost-filesystem-dev libboost-system-dev libboost-python-dev libboost-thread-dev libboost-date-time-dev octave3.2-headers "libcollada15dom-dev (>=2.3.1)" pkg-config)
  # developer tools for devs
  set(CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU ${CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU} python-scipy ipython)
  # add once infinite precision computations are enabled:
  # libmpfr-dev libgmp3-dev libmpfi-dev libgmpxx4ldbl python-gmpy
  # plugins:
  set(CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU ${CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU} libboost-iostreams-dev libboost-regex-dev libqt4-dev qt4-dev-tools libqhull-dev libavcodec-dev libavformat-dev libswscale-dev libsimage-dev libode-dev libsoqt4-dev assimp-dev libbullet-dev)

  # debian
  set(CPACK_DEBIAN_PACKAGE_PRIORITY optional)
  set(CPACK_DEBIAN_PACKAGE_SECTION devel)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS openrave-minimal)
  set(CPACK_DEBIAN_CMAKE_OPTIONS "-DBASH_COMPLETION_DIR=../etc/bash_completion.d -DOPT_DOUBLE_PRECISION=${OPT_DOUBLE_PRECISION} -DOPT_BUILD_PACKAGE_DEFAULT=${OPT_BUILD_PACKAGE_DEFAULT}")
  set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "ipython, python-qt4")
  foreach(COMP ${CPACK_COMPONENTS_ALL})
    set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "${CPACK_DEBIAN_PACKAGE_RECOMMENDS}, ${COMP}")
  endforeach()
  set(CPACK_DEBIAN_PACKAGE_REMOVE_SOURCE_FILES docs release sandbox test sympy msvc_boost.tgz msvc_collada.tgz msvc_libxml2.tgz msvc_ode.tgz msvc_soqt.tgz msvc_include sympy_0.6.7.tgz sympy_0.6.7_patches.diff 3rdparty/pcre-8.02 3rdparty/qhull 3rdparty/zlib 3rdparty/flann-1.6.6 3rdparty/collada)
  set(CPACK_DEBIAN_PACKAGE_SOURCE_COPY svn export --force)
  set(CPACK_DEBIAN_CHANGELOG "  * ChangeLog can be found at https://openrave.svn.sourceforge.net/svnroot/openrave/tags/${OPENRAVE_VERSION}/docs/en/changelog.rst\n\n")
  execute_process(COMMAND lsb_release -is
    OUTPUT_VARIABLE _lsb_distribution OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _lsb_release_failed)
  set(CPACK_DEBIAN_DISTRIBUTION_NAME ${_lsb_distribution} CACHE STRING "Name of the distrubiton")
  string(TOLOWER ${CPACK_DEBIAN_DISTRIBUTION_NAME} CPACK_DEBIAN_DISTRIBUTION_NAME)
  if( ${CPACK_DEBIAN_DISTRIBUTION_NAME} STREQUAL "ubuntu" )
    set(CPACK_DEBIAN_DISTRIBUTION_RELEASES lucid maverick natty oneiric CACHE STRING "Release code-names of the distrubiton release")
  endif()
  if( OPT_BUILD_PACKAGE_DEFAULT )
    set(CPACK_COMPONENT_OPENRAVE_RECOMMENDS ${CPACK_DEBIAN_PACKAGE_NAME})
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
