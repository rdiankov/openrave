# build sources, moc'd sources, and rcc'd sources
add_library(openrave-md5 STATIC md5.cpp)
set_target_properties(openrave-md5 PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")

add_executable(cpp-gen-md5 cpp_lexer.cpp cpp_lexer_token.cpp md5.cpp cpp-gen-md5.cpp)
target_link_libraries(cpp-gen-md5 openrave-md5)
add_dependencies(cpp-gen-md5 openrave-md5)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  set_target_properties(cpp-gen-md5 PROPERTIES COMPILE_FLAGS "-trigraphs -ftemplate-depth-500")
endif()
