add_library(openrave-md5 STATIC md5.h md5.c)
target_include_directories(openrave-md5 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
