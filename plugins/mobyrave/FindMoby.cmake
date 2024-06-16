# Find MOBY header and library.
#
# This module defines the following uncached variables:
# MOBY_FOUND, if false, do not try to use MOBY.
# MOBY_INCLUDE_DIRS, where to find MOBY/MOBY_a.h.
# MOBY_LIBRARIES, the libraries to link against to use the MOBY library
# MOBY_LIBRARY_DIRS, the directory where the MOBY library is found.
find_path(
MOBY_INCLUDE_DIR
NAMES Moby/EventDrivenSimulator.h
PATHS /usr/local/include /usr/include
)
if( MOBY_INCLUDE_DIR )
find_library(
MOBY_LIBRARY
NAMES libMoby.so libMoby.dylib libMoby.a
PATHS /usr/local/lib /usr/lib
)
if( MOBY_LIBRARY )
set(MOBY_LIBRARY_DIR "")
get_filename_component(MOBY_LIBRARY_DIRS ${MOBY_LIBRARY} PATH)
# Set uncached variables as per standard.
set(MOBY_FOUND ON)
set(MOBY_INCLUDE_DIRS ${MOBY_INCLUDE_DIR})
set(MOBY_LIBRARIES ${MOBY_LIBRARY})
endif(MOBY_LIBRARY)
else(MOBY_INCLUDE_DIR)
message(STATUS "FindMoby: Could not find EventDrivenSimulator.h")
endif(MOBY_INCLUDE_DIR)
if(MOBY_FOUND)
if(NOT MOBY_FIND_QUIETLY)
message(STATUS "FindMoby: Found both EventDrivenSimulator.h and libMoby")
endif(NOT MOBY_FIND_QUIETLY)
else(MOBY_FOUND)
if(MOBY_FIND_REQUIRED)
message(FATAL_ERROR "FindMoby: Could not find EventDrivenSimulator.h and/or libMoby")
endif(MOBY_FIND_REQUIRED)
endif(MOBY_FOUND)