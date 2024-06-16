# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0005
cmake_policy(SET CMP0005 NEW)

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckFunctionExists)
include(CheckLibraryExists)
include(CheckTypeSize)

check_type_size("float" HAVE_FLOAT_SIZE BUILTIN_TYPES_ONLY)
if( HAVE_FLOAT_SIZE EQUAL 4 )
  set(FP_SUPPORT_FLOAT_TYPE 1 CACHE STRING "support float type")
endif()

check_type_size("long int" HAVE_LONG_INT_SIZE BUILTIN_TYPES_ONLY)
if( HAVE_LONG_INT_SIZE EQUAL 8 )
  set(FP_SUPPORT_LONG_INT_TYPE 1 CACHE STRING "support long int type")
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set(CMAKE_REQUIRED_LIBRARIES m)
endif()

# check if alloca exists
check_include_file(alloca.h HAVE_ALLOCA_H)
set(FP_USE_THREAD_SAFE_EVAL 1 CACHE STRING "use thread-safety")
if( HAVE_ALLOCA_H )
  set(FP_USE_THREAD_SAFE_EVAL_WITH_ALLOCA 1 CACHE STRING "use thread-safety with alloca")
endif()

set(FP_SUPPORT_OPTIMIZER 1 CACHE STRING "support optimized computations")
set(FP_SUPPORT_COMPLEX_DOUBLE_TYPE 1 CACHE STRING "support complex double types")
set(FP_SUPPORT_COMPLEX_FLOAT_TYPE 1 CACHE STRING "support complex float types")

check_function_exists(strtof FP_USE_STRTOF)
check_function_exists(strtold FP_USE_STRTOLD)
check_type_size("long double" HAVE_LONG_DOUBLE_SIZE BUILTIN_TYPES_ONLY)
if( HAVE_LONG_DOUBLE_SIZE EQUAL 16 )
  set(FP_SUPPORT_LONG_DOUBLE_TYPE 1 CACHE STRING "support long double")
endif()
if( HAVE_LONG_DOUBLE_SIZE EQUAL 16 )
  set(FP_SUPPORT_COMPLEX_LONG_DOUBLE_TYPE 1 CACHE STRING "support complex long double")
endif()

set(fparser_SOURCES fparser.cc ) # fpoptimizer.cc

set(fparser_LIBRARIES)
set(fparser_LIBRARIES_DIR)

set(FP_SUPPORT_MPFR_FLOAT_TYPE ${MPFR_FOUND} CACHE STRING "use mpfr float")
if( FP_SUPPORT_MPFR_FLOAT_TYPE )
  set(fparser_SOURCES ${fparser_SOURCES} mpfr/MpfrFloat.cc)
  set(fparser_LIBRARIES ${fparser_LIBRARIES} ${MPFR_LIBRARIES})
  set(fparser_LIBRARIES_DIR ${fparser_LIBRARIES_DIR} ${MPFR_LIBRARIES_DIR})
endif()

set(FP_SUPPORT_GMP_INT_TYPE ${GMP_FOUND} CACHE STRING "use gmp int")
if( FP_SUPPORT_GMP_INT_TYPE )
  set(fparser_SOURCES ${fparser_SOURCES} mpfr/GmpInt.cc)
  set(fparser_LIBRARIES ${fparser_LIBRARIES} ${GMP_LIBRARIES})
  set(fparser_LIBRARIES_DIR ${fparser_LIBRARIES_DIR} ${GMP_LIBRARIES_DIR})
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/fpconfig.hh.in ${CMAKE_CURRENT_SOURCE_DIR}/fpconfig.hh)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${fparser_LIBRARIES_DIR})
add_library(fparser STATIC ${fparser_SOURCES})
target_link_libraries(fparser ${fparser_LIBRARIES})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/fpconfig.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/fparser.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/fparser_gmpint.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/fparser_mpfr.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser)
if( FP_SUPPORT_MPFR_FLOAT_TYPE )
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mpfr/MpfrFloat.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser/mpfr)
endif()
if( FP_SUPPORT_GMP_INT_TYPE )
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mpfr/GmpInt.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser/mpfr)
endif()
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/extrasrc/fpaux.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/extrasrc/fptypes.hh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/fparser)

set(FPARSER_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}/include" PARENT_SCOPE)
set(FPARSER_LIBRARY_DIRS PARENT_SCOPE)
set(FPARSER_LIBRARIES fparser PARENT_SCOPE)
set(FPARSER_CXX_FLAGS PARENT_SCOPE)
set(FPARSER_LINK_FLAGS PARENT_SCOPE)
set(FPARSER_FOUND 1 PARENT_SCOPE)
