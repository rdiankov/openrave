add_definitions("-DFP_SUPPORT_FLOAT_TYPE") # since openrave has a float option
add_definitions("-DFP_SUPPORT_TR1_MATH_FUNCS") # make things faster?

# check if alloca exists
check_include_file(alloca.h HAVE_ALLOCA_H)
if( HAVE_ALLOCA_H )
  add_definitions("-DFP_USE_THREAD_SAFE_EVAL_WITH_ALLOCA")
else()
  add_definitions("-DFP_USE_THREAD_SAFE_EVAL")
endif()

# don't include optimizer since openrave really doesn't use that many complex equations
add_definitions("-DFP_NO_SUPPORT_OPTIMIZER")

set(fparser_SOURCES fparser.cc ) # fpoptimizer.cc

set(fparser_LIBRARIES)
set(fparser_LIBRARIES_DIR)
if( MPFR_FOUND )
  set(fparser_SOURCES ${fparser_SOURCES} mpfr/MpfrFloat.cc)
  add_definitions("-DFP_SUPPORT_MPFR_FLOAT_TYPE")
  set(fparser_LIBRARIES ${fparser_LIBRARIES} ${MPFR_LIBRARIES})
  set(fparser_LIBRARIES_DIR ${fparser_LIBRARIES_DIR} ${MPFR_LIBRARIES_DIR})
endif()
if( GMP_FOUND )
  set(fparser_SOURCES ${fparser_SOURCES} mpfr/GmpInt.cc)
  add_definitions("-DFP_SUPPORT_GMP_INT_TYPE")
  set(fparser_LIBRARIES ${fparser_LIBRARIES} ${GMP_LIBRARIES})
  set(fparser_LIBRARIES_DIR ${fparser_LIBRARIES_DIR} ${GMP_LIBRARIES_DIR})
endif()
link_directories(${fparser_LIBRARIES_DIR})
add_library(fparser STATIC ${fparser_SOURCES})
set_target_properties(fparser PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")
target_link_libraries(fparser ${fparser_LIBRARIES})
