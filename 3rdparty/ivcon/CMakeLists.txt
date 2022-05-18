include_directories(${OPENRAVE_INCLUDE_LOCAL_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_library(ivcon STATIC ivcon.cpp)
target_link_libraries(ivcon ${Boost_THREAD_LIBRARY})
