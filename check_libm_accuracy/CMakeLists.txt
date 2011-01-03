# check the accuracy of the current math library with crlibm
add_executable(check_libm_accuracy check_libm_accuracy.cpp)
set_target_properties(check_libm_accuracy PROPERTIES COMPILE_FLAGS "-I${CRLIBM_INCLUDE_DIR}")
target_link_libraries(check_libm_accuracy ${CRLIBM_LIBRARY})
set(libm_accuracy_results "${CMAKE_SOURCE_DIR}/src/libopenrave/libm_accuracy_results.h")
add_custom_command(TARGET check_libm_accuracy POST_BUILD
  OUTPUT ${libm_accuracy_result}
  COMMAND check_libm_accuracy ARGS ${libm_accuracy_results}
  COMMENT "Checking accuracy between libm and crlibm")
