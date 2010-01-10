file(GLOB ann_files ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
add_library(ann STATIC ${ann_files})
set_target_properties(ann PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")
