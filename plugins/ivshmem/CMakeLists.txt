find_package(libccd REQUIRED)
find_package(fcl REQUIRED)
message(STATUS "libccd version ${libccd_VERSION}")
message(STATUS "fcl version ${fcl_VERSION}")

set(ivshmem_src
    ivshmem.cpp # Entrypoint of the plugin
    ivshmem_server.cpp
    ivshmem_server.hpp
    ivshmem_interface.cpp
    ivshmem_interface.hpp

    fd.hpp
    triple_buffered_shared_memory.cpp
    serialization.cpp

    fclspace.cpp
    fclmanagercache.cpp
    #fclcollision.cpp
)

add_library(ivshmem SHARED ${ivshmem_src})
target_include_directories(ivshmem PRIVATE ${FCL_INCLUDEDIR})
target_link_directories(ivshmem PRIVATE ${FCL_LIBRARY_DIRS})
target_link_libraries(ivshmem PRIVATE boost_assertion_failed PUBLIC rt libopenrave fcl::fcl_static libccd::ccd_static)
set_target_properties(ivshmem PROPERTIES
    COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
    LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}"
)

install(TARGETS ivshmem DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-ivshmem)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-IVSHMEM_DISPLAY_NAME "Plugin for collision computation through inter-VM shared memory" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-ivshmem PARENT_SCOPE)
