file(GLOB minizip_files ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
add_library(zlib STATIC ${minizip_files})
set_target_properties(zlib PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")

