include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DBOOST_ALL_NO_LIB) # remove auto-linking
add_library(boost_filesystem_local STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/filesystem/operations.cpp)
set_target_properties(boost_filesystem_local PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")

add_library(boost_system_local STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/system/error_code.cpp)
set_target_properties(boost_system_local PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")
