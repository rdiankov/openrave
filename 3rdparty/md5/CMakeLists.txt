project(openrave-md5 LANGUAGES C)

add_library(openrave-md5 STATIC md5.h md5.c)
target_include_directories(openrave-md5 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Native variant for cpp-gen-md5
add_library(openrave-md5-native STATIC md5.h md5.c)
target_include_directories(openrave-md5-native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(openrave-md5-native PROPERTIES
    COMPILE_FLAGS "${NATIVE_COMPILE_FLAGS}"
)