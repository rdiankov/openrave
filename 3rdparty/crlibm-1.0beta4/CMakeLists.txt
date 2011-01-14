# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0005
cmake_policy(SET CMP0005 NEW)
add_definitions(-DPACKAGE=\"crlibm\")
add_definitions(-DVERSION=\"1.0beta4\")

# Compile the filib-compatible interval functions
# BUILD_INTERVAL_FUNCTIONS

# architecture- and system-specific FPU control header file
check_include_file(fpu_control.h HAS_FPU_CONTROL_H)
if( HAS_FPU_CONTROL_H )
  add_definitions("-DCRLIBM_HAS_FPU_CONTROL=1")
endif()

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )
  add_definitions("-DCRLIBM_TYPECPU_AMD64=1")
elseif( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "i486" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "i686" )
  add_definitions("-DCRLIBM_TYPECPU_X86=1")
elseif( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "ppc" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "powerpc" )
  add_definitions("-DCRLIBM_TYPECPU_POWERPC=1")
  add_definitions("-DWORDS_BIGENDIAN=1")
elseif( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "sparc" )
  add_definitions("-DCRLIBM_TYPECPU_SPARC=1")
endif()

if( BSD )
  add_definitions("-DCRLIBM_TYPEOS_BSD=1")
endif()
if( CYGWIN )
  add_definitions("-DCRLIBM_TYPEOS_CYGWIN")
endif()
if( HPUX )
  add_definitions("-DCRLIBM_TYPEOS_HPUX")
endif()

check_include_file(fenv.h HAVE_FENV_H)
if(HAVE_FENV_H)
  add_definitions("-DHAVE_FENV_H=1")
endif()

check_include_file(inttypes.h HAVE_INTTYPES_H)
if(HAVE_INTTYPES_H)
  add_definitions("-DHAVE_INTTYPES_H=1")
endif()

# Size of an SCS digit
add_definitions("-DSCS_NB_BITS=30")

# Number of digits in the SCS structure
add_definitions("-DSCS_NB_WORDS=8")

check_type_size("long double" LONG_DOUBLE BUILTIN_TYPES_ONLY)
set(source_dir .)
if( HAS_FPU_CONTROL_H AND HAVE_LONG_DOUBLE AND LONG_DOUBLE EQUAL 16 )
  message(STATUS "compiling with double extended (long double) floating point")
  set(crlibm_SOURCES
    ${source_dir}/crlibm_private.c
    ${source_dir}/triple-double.c
    ${source_dir}/exp-td.c
    ${source_dir}/exp-td-standalone.c
    ${source_dir}/expm1-standalone.c
    ${source_dir}/expm1.c
    ${source_dir}/log-de.c
    ${source_dir}/log1p.c
    ${source_dir}/log10-td.c
    ${source_dir}/log2-td.c
    ${source_dir}/rem_pio2_accurate.c
    ${source_dir}/trigo_fast.c
    ${source_dir}/trigo_accurate.c
    ${source_dir}/trigpi.c
    ${source_dir}/asincos.c
    ${source_dir}/pow.c
    ${source_dir}/atan_fast.c
    ${source_dir}/atan_accurate.c
    ${source_dir}/csh_fast.c
    ${source_dir}/scs_lib/addition_scs.c
    ${source_dir}/scs_lib/double2scs.c
    ${source_dir}/scs_lib/print_scs.c
    ${source_dir}/scs_lib/scs2mpfr.c
    ${source_dir}/scs_lib/zero_scs.c
    ${source_dir}/scs_lib/division_scs.c
    ${source_dir}/scs_lib/multiplication_scs.c
    ${source_dir}/scs_lib/scs2double.c
    ${source_dir}/scs_lib/scs_private.c
    )
else()
  set(crlibm_SOURCES
    ${source_dir}/crlibm_private.c
    ${source_dir}/triple-double.c
    ${source_dir}/exp-td.c
    ${source_dir}/exp-td-standalone.c
    ${source_dir}/expm1-standalone.c
    ${source_dir}/expm1.c
    ${source_dir}/log.c
    ${source_dir}/log1p.c
    ${source_dir}/rem_pio2_accurate.c
    ${source_dir}/trigo_fast.c 
    ${source_dir}/trigo_accurate.c
    ${source_dir}/trigpi.c
    ${source_dir}/asincos.c
    ${source_dir}/pow.c
    ${source_dir}/atan_fast.c
    ${source_dir}/atan_accurate.c
    ${source_dir}/csh_fast.c
    ${source_dir}/scs_lib/addition_scs.c
    ${source_dir}/scs_lib/double2scs.c
    ${source_dir}/scs_lib/print_scs.c
    ${source_dir}/scs_lib/scs2mpfr.c
    ${source_dir}/scs_lib/zero_scs.c
    ${source_dir}/scs_lib/division_scs.c
    ${source_dir}/scs_lib/multiplication_scs.c
    ${source_dir}/scs_lib/scs2double.c
    ${source_dir}/scs_lib/scs_private.c
    )
endif()

set(crlibm_SOURCES ${crlibm_SOURCES}
  ${source_dir}/asincos.h        ${source_dir}/crlibm.h          ${source_dir}/double-extended.h  ${source_dir}/log-de.h  ${source_dir}/log10-td.h  ${source_dir}/rem_pio2_accurate.h  ${source_dir}/trigpi.h
  ${source_dir}/atan_accurate.h  ${source_dir}/crlibm_private.h  ${source_dir}/exp-td.h           ${source_dir}/log-td.h  ${source_dir}/log2-td.h   ${source_dir}/trigo_accurate.h     ${source_dir}/triple-double.h
  ${source_dir}/atan_fast.h      ${source_dir}/csh_fast.h        ${source_dir}/expm1.h            ${source_dir}/log.h     ${source_dir}/pow.h       ${source_dir}/trigo_fast.h)

add_library(crlibm STATIC ${crlibm_SOURCES})
set_target_properties(crlibm PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")
