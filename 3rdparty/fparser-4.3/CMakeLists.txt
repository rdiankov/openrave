# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0005
cmake_policy(SET CMP0005 NEW)
set(FP_SUPPORT_FLOAT_TYPE 1) # since openrave has a float option

set(CMAKE_REQUIRED_LIBRARIES m)
#look for asinh(), acosh(), atanh(), exp2() and log2().
check_function_exists(asinh HAS_ASINH)
check_function_exists(acosh HAS_ACOSH)
check_function_exists(atanh HAS_ATANH)
check_function_exists(exp2 HAS_EXP2)
check_function_exists(log2 HAS_LOG2)
# make things faster?
if( HAS_ASINH AND HAS_ACOSH AND HAS_ATANH AND HAS_EXP2 AND HAS_LOG2 )
  set(FP_SUPPORT_TR1_MATH_FUNCS 1)
else()
  set(FP_SUPPORT_TR1_MATH_FUNCS 0)
endif()

# check if alloca exists
check_include_file(alloca.h HAVE_ALLOCA_H)
if( HAVE_ALLOCA_H )
  set(FP_USE_THREAD_SAFE_EVAL_WITH_ALLOCA 1)
else()
  set(FP_USE_THREAD_SAFE_EVAL 1)
endif()

# don't include optimizer since openrave really doesn't use that many complex equations
set(FP_NO_SUPPORT_OPTIMIZER 1)

check_function_exists(strtof FP_USE_STRTOF)
check_function_exists(strtold HAS_STRTOLD)
check_type_size("long double" HAVE_LONG_DOUBLE BUILTIN_TYPES_ONLY)
if( HAVE_LONG_DOUBLE EQUAL 16 AND HAS_STRTOLD )
  set(FP_SUPPORT_LONG_DOUBLE_TYPE 1)
else()
  set(FP_SUPPORT_LONG_DOUBLE_TYPE 0)
endif()

# custom flag
set(FP_USE_BOOST_FUNCTION 1)
set(FP_EPSILON "1e-14")
set(FP_EVAL_MAX_REC_LEVEL 1000)

#should be faster at a cost
#set(FP_NO_EVALUATION_CHECKS 1)

set(fparser_SOURCES fparser.cc ) # fpoptimizer.cc

set(fparser_LIBRARIES)
set(fparser_LIBRARIES_DIR)

if( MPFR_FOUND )
  set(fparser_SOURCES ${fparser_SOURCES} mpfr/MpfrFloat.cc)
  set(fparser_LIBRARIES ${fparser_LIBRARIES} ${MPFR_LIBRARIES})
  set(fparser_LIBRARIES_DIR ${fparser_LIBRARIES_DIR} ${MPFR_LIBRARIES_DIR})
  set(FP_SUPPORT_MPFR_FLOAT_TYPE 1)
else()
  set(FP_SUPPORT_MPFR_FLOAT_TYPE 0)
endif()

if( GMP_FOUND )
  set(fparser_SOURCES ${fparser_SOURCES} mpfr/GmpInt.cc)
  set(fparser_LIBRARIES ${fparser_LIBRARIES} ${GMP_LIBRARIES})
  set(fparser_LIBRARIES_DIR ${fparser_LIBRARIES_DIR} ${GMP_LIBRARIES_DIR})
  set(FP_SUPPORT_GMP_INT_TYPE 1)
else()
  set(FP_SUPPORT_GMP_INT_TYPE 0)
endif()

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/fpconfig.hh.in ${CMAKE_CURRENT_SOURCE_DIR}/fpconfig.hh)

link_directories(${fparser_LIBRARIES_DIR})
add_library(fparser STATIC ${fparser_SOURCES})
set_target_properties(fparser PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")
target_link_libraries(fparser ${fparser_LIBRARIES})
