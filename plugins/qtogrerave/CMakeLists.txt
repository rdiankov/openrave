
# set(QTOGRE_RCCS
#     qtogreviewer.qrc)

find_package(OGRE)
find_package(OpenGL)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

if (NOT(Qt5Core_FOUND AND Qt5Gui_FOUND AND OPENGL_FOUND AND OGRE_RenderSystem_GL3Plus_FOUND))
    message(STATUS "qtogrerave is skipped")
endif()
message(STATUS "Build qtogrerave")

set(QTOGRE_SRCS
	ogrehandle.cpp
    qtogrerave.cpp
    qtogrewindow.cpp
    qtogreviewer.cpp)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

include_directories(${OGRE_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${Qt5_INCLUDE_DIRS})
link_directories(${OPENRAVE_LINK_DIRS})

add_library(qtogrerave SHARED ${QTOGRE_SRCS})
target_link_libraries(qtogrerave PUBLIC ${OGRE_LIBRARIES} ${BOOST_LIBRARIES} libopenrave Qt5::Core Qt5::Gui Qt5::Widgets OgreHlmsPbs
                                 PRIVATE boost_assertion_failed)

set_target_properties(qtogrerave PROPERTIES COMPILE_FLAGS "${QTOGRE_COMPILE_FLAGS} ${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${QTOGRE_LINK_FLAGS} ${PLUGIN_LINK_FLAGS}")
install(TARGETS qtogrerave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-qtogrerave)

set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-QTOGRERAVE_DISPLAY_NAME "Plugin for a Qt/Ogre viewer" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-qtogrerave PARENT_SCOPE)