if (NOT OPENRAVE_STATIC_PLUGINS)
    # If static-plugins are not specified, then create an empty library.
    # This is so that linking against static_plugins becomes a no-op,
    # and we can link against it regardless of the state of the OPENRAVE_STATIC_PLUGINS flag.
    add_library(static_plugins INTERFACE)
    return()
endif()

find_package(PkgConfig QUIET)
set(OPENRAVE_PLUGIN_DIR ${CMAKE_SOURCE_DIR}/plugins)

message(STATUS "Statically compiling in plugins:")

### Base robots
add_library(baserobots_static STATIC
    ${OPENRAVE_PLUGIN_DIR}/baserobots/baserobots.cpp
    ${OPENRAVE_PLUGIN_DIR}/baserobots/collisionmaprobot.cpp
    ${OPENRAVE_PLUGIN_DIR}/baserobots/conveyor.cpp)
target_compile_definitions(baserobots_static PRIVATE "OPENRAVE_STATIC_PLUGINS=1")
message(STATUS "\t... Base robots")

### Base controllers
add_library(basecontrollers_static STATIC
    ${OPENRAVE_PLUGIN_DIR}/basecontrollers/basecontrollers.cpp
    ${OPENRAVE_PLUGIN_DIR}/basecontrollers/redirectcontroller.cpp
    ${OPENRAVE_PLUGIN_DIR}/basecontrollers/idealcontroller.cpp
    ${OPENRAVE_PLUGIN_DIR}/basecontrollers/idealvelocitycontroller.cpp)
target_compile_definitions(basecontrollers_static PRIVATE "OPENRAVE_STATIC_PLUGINS=1")
message(STATUS "\t... Base controllers")

### Base samplers
add_library(basesamplers_static STATIC
    ${OPENRAVE_PLUGIN_DIR}/basesamplers/basesamplers.cpp
    ${OPENRAVE_PLUGIN_DIR}/basesamplers/halton.cpp
    ${OPENRAVE_PLUGIN_DIR}/basesamplers/robotconfiguration.cpp
    ${OPENRAVE_PLUGIN_DIR}/basesamplers/bodyconfiguration.cpp)
target_compile_definitions(basesamplers_static PRIVATE "OPENRAVE_STATIC_PLUGINS=1")
message(STATUS "\t... Base samplers")

### Base sensors
add_library(basesensors_static STATIC ${OPENRAVE_PLUGIN_DIR}/basesensors/basesensors.cpp)
target_compile_definitions(basesensors_static PRIVATE "OPENRAVE_STATIC_PLUGINS=1")
message(STATUS "\t... Base sensors")

### FCL
pkg_check_modules(FCL fcl)
if (NOT FCL_FOUND)
    message(FATAL_ERROR "FCL is a required plugin when linking plugins statically.")
endif()
add_library(fclrave_static STATIC ${OPENRAVE_PLUGIN_DIR}/fclrave/fclrave.cpp)
target_compile_definitions(fclrave_static PRIVATE
    $<$<BOOL:${NARROW_COLLISION_CACHING}>:"NARROW_COLLISION_CACHING">
    $<$<BOOL:${FCL_USE_STATISTICS}>:"FCLUSESTATISTICS">
    $<$<BOOL:${FCL_STATISTICS_DISPLAY_CONTINUOUSLY}>:"FCL_STATISTICS_DISPLAY_CONTINUOUSLY">
    $<$<BOOL:${FCLRAVE_USE_COLLISION_STATISTICS}>:"FCLRAVE_USE_COLLISION_STATISTICS">
    $<$<BOOL:${FCLRAVE_DEBUG_COLLISION_OBJECTS}>:"FCLRAVE_DEBUG_COLLISION_OBJECTS">
    $<$<BOOL:${FCL_HAS_REPLACEOBJECT}>:"FCL_HAS_REPLACEOBJECT">
    $<$<BOOL:${FCL_SUPPORT_BULK_UPDATE}>:"FCL_SUPPORT_BULK_UPDATE">
)
target_link_libraries(fclrave_static PRIVATE ${FCL_LIBRARIES})
target_compile_definitions(fclrave_static PRIVATE "OPENRAVE_STATIC_PLUGINS=1")
message(STATUS "\t... FCL")

add_dependencies(baserobots_static      interfacehashes_target)
add_dependencies(basecontrollers_static interfacehashes_target)
add_dependencies(basesamplers_static    interfacehashes_target)
add_dependencies(basesensors_static     interfacehashes_target)
add_dependencies(fclrave_static         interfacehashes_target)

add_library(static_plugins INTERFACE)
target_include_directories(static_plugins INTERFACE
    ${OPENRAVE_PLUGIN_DIR}/baserobots
    ${OPENRAVE_PLUGIN_DIR}/basecontrollers
    ${OPENRAVE_PLUGIN_DIR}/basesamplers
    ${OPENRAVE_PLUGIN_DIR}/basesensors
    ${OPENRAVE_PLUGIN_DIR}/fclrave
)
target_link_libraries(static_plugins INTERFACE
    baserobots_static
    basecontrollers_static
    basesamplers_static
    basesensors_static
    fclrave_static
)
