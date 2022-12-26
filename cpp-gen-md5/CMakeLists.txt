# has to be native since it is a build-time tool
add_executable(cpp-gen-md5-native cpp_lexer.cpp cpp_lexer_token.cpp cpp-gen-md5.cpp)
set_target_properties(cpp-gen-md5-native PROPERTIES COMPILE_FLAGS "${NATIVE_COMPILE_FLAGS}")
target_link_libraries(cpp-gen-md5-native openrave-md5-native)
add_dependencies(cpp-gen-md5-native openrave-md5-native)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set_target_properties(cpp-gen-md5-native PROPERTIES COMPILE_FLAGS "-trigraphs -ftemplate-depth-500 ${NATIVE_COMPILE_FLAGS}")
endif()
