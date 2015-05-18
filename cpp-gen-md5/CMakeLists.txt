# build sources, moc'd sources, and rcc'd sources
add_library(openrave-md5 STATIC md5.c)
set_target_properties(openrave-md5 PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")

add_library(openrave-md5-native STATIC md5.c)
set_target_properties(openrave-md5-native PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS} ${NATIVE_COMPILE_FLAGS}")

# has to be native since it is a build-time tool
add_executable(cpp-gen-md5-native cpp_lexer.cpp cpp_lexer_token.cpp md5.c cpp-gen-md5.cpp)
set_target_properties(cpp-gen-md5-native PROPERTIES COMPILE_FLAGS "${NATIVE_COMPILE_FLAGS}")
target_link_libraries(cpp-gen-md5-native openrave-md5-native ${STDC_LIBRARY})
add_dependencies(cpp-gen-md5-native openrave-md5-native)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set_target_properties(cpp-gen-md5-native PROPERTIES COMPILE_FLAGS "-trigraphs -ftemplate-depth-500 ${NATIVE_COMPILE_FLAGS}")
endif()
