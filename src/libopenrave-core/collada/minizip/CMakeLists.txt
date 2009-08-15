include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB minizip_files ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
add_library(minizip STATIC ${minizip_files})
set_target_properties(minizip PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")
