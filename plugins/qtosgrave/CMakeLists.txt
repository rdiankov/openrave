#############################
# qtosgrave openrave plugin #
#############################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL)
if(NOT OPENGL_FOUND)
  message(WARNING "OpenGL not found")
endif()

#include(${CMAKE_SOURCE_DIR}/modules-cmake/FindOSG.cmake)
find_package(OpenSceneGraph 3.4 COMPONENTS osgDB osgGA osgFX osgText osgViewer osgManipulator)

if (Qt5Gui_FOUND AND Qt5Core_FOUND AND Qt5Widgets_FOUND AND Qt5OpenGL_FOUND AND OPENGL_FOUND AND OPENSCENEGRAPH_FOUND)
  message(STATUS "Detected Qt/OSG GUI, making plugin")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
  #set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} "/usr/lib" "/usr/share/qt4/lib")
  
  link_directories(${OPENRAVE_LINK_DIRS})

  set(QTOSG_SRCS objecttree.cpp osgcartoon.cpp osgviewerwidget.cpp osgviewerwidget.h osgpick.cpp osgpick.h qtreemodel.cpp qtreemodel.h osgrenderitem.cpp osgrenderitem.h qtreeitem.cpp qtreeitem.h qtosgviewer.cpp qtosgrave.cpp osgskybox.cpp osgskybox.h osglodlabel.cpp osglodlabel.h)
  set(QTOSG_MOCS qtosgviewer.h qtreemodel.h)
  set(QTOSG_RCCS qtosgviewer.qrc)

  # generate rules for building source files from the resources
  qt5_add_resources(QTOSG_RCC_SRCS ${QTOSG_RCCS})
  qt5_wrap_cpp(QTOSG_MOC_SRCS ${QTOSG_MOCS})
  
  check_include_file(X11/Xlib.h HAVE_X11_XLIB_H)

  set(QTOSG_COMPILE_FLAGS)
  set(QTOSG_LINK_FLAGS)
  if( HAVE_X11_XLIB_H )
    set(QTOSG_COMPILE_FLAGS "${QTOSG_COMPILE_FLAGS} -DHAVE_X11_XLIB_H")
  endif()

  set (extLIBS ${PTHREADS_LIBRARY} OpenGL::OpenGL Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
  set (extLIBS ${extLIBS} ${OPENSCENEGRAPH_LIBRARIES})
  #set (extLIBS ${extLIBS} ${OSGDB_LIBRARY_DEBUG} ${OSGQT_LIBRARY_DEBUG} ${OSGGA_LIBRARY_DEBUG} ${OSGFX_LIBRARY_DEBUG} ${OSGTEXT_LIBRARY_DEBUG} ${OSGVIEWER_LIBRARY_DEBUG} ${OSGMANIPULATOR_LIBRARY_DEBUG})
  
  if( MSVC )
    set (extLIBS ${extLIBS} winmm imm32 ws2_32)
  endif()

  # Debug.
  if (NOT QT_QTOPENGL_FOUND )
      message(STATUS "QTOPENGL library NOT FOUND!!!")
  endif()
  
  # Debug.
  message(STATUS "Link libraries: ${extLIBS}")

  # build sources, moc'd sources, and rcc'd sources

  add_library(qtosgrave SHARED ${QTOSG_SRCS} ${QTOSG_MOC_SRCS} ${QTOSG_RCC_SRCS})
  target_link_libraries(qtosgrave PRIVATE boost_assertion_failed PUBLIC libopenrave ${extLIBS})

  #if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  #  # have to turn off optimizations or else qtcoin crashes!!
  #  # (probably something to do with this being  a plugin and SoQt relies on static variables?)
  #  # another possiblity is that openrave-core also links to Coin3D...
  #  set(QTOSG_COMPILE_FLAGS "${QTOSG_COMPILE_FLAGS} -O0")
  #endif()

  set_target_properties(qtosgrave PROPERTIES COMPILE_FLAGS "${QTOSG_COMPILE_FLAGS} ${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${QTOSG_LINK_FLAGS} ${PLUGIN_LINK_FLAGS}")
  install(TARGETS qtosgrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-qtosgrave)
  
  #if( MSVC )
  #  # have to install the DLLs
  #  install(DIRECTORY "${SOQT_LINK_DIRS}/" DESTINATION bin COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave FILES_MATCHING PATTERN "*-${MSVC_PREFIX}-*.dll")
  #endif()

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-QTOSGRAVE_DISPLAY_NAME "Plugin for a Qt/OpenSceneGraph viewer" PARENT_SCOPE)
  set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-qtosgrave PARENT_SCOPE)
else()
    message(STATUS "Could not find OpenSceneGraph v3+. Please install OSG (http://www.openscenegraph.org/projects/osg)")
endif()
