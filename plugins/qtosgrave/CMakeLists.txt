#############################
# qtosgrave openrave plugin #
#############################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

find_package(OpenGL)
if(NOT OPENGL_FOUND)
  message(STATUS "WARNING: OpenGL not found")
endif()

#include(${CMAKE_SOURCE_DIR}/modules-cmake/FindOSG.cmake)
find_package(OpenSceneGraph 3.0 COMPONENTS osgDB osgQt osgGA osgText osgViewer osgManipulator osgShadow)

if (QT_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND AND OPENGL_FOUND AND OPENSCENEGRAPH_FOUND)
  message(STATUS "Detected Qt/OSG GUI, making plugin")

  include_directories(${QT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR})
  message(STATUS "QT dirs: " ${QT_INCLUDE_DIR} " OPENGL: " ${QT_QTOPENGL_INCLUDE_DIR})
  include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
  #set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} "/usr/lib" "/usr/share/qt4/lib")
  
  link_directories(${OPENRAVE_LINK_DIRS})

  set(QTOSG_SRCS objecttree.cpp treemodel.cpp Item.cpp treeitem.cpp qtosgviewer.cpp qtosgravemain.cpp osgviewerQtContext.h osgpick.cpp)
  set(QTOSG_MOCS qtosgviewer.h treemodel.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
  set(QTOSG_RCCS qtosgviewer.qrc)

  # generate rules for building source files from the resources
  qt4_add_resources(QTOSG_RCC_SRCS ${QTOSG_RCCS})

  include( ${QT_USE_FILE} )
  qt4_wrap_cpp(QTOSG_MOC_SRCS ${QTOSG_MOCS})
  
  check_include_file(X11/Xlib.h HAVE_X11_XLIB_H)

  set(QTOSG_COMPILE_FLAGS)
  set(QTOSG_LINK_FLAGS)
  if( HAVE_X11_XLIB_H )
    set(QTOSG_COMPILE_FLAGS "${QTOSG_COMPILE_FLAGS} -DHAVE_X11_XLIB_H")
  endif()

  set (extLIBS ${QT_LIBRARIES} ${LIBXML2_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${PTHREADS_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${OPENGL_LIBRARIES})
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
  target_link_libraries (qtosgrave ${extLIBS} libopenrave)

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
