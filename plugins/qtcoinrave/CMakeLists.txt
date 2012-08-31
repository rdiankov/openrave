############################
# qtcoinrave openrave plugin
############################
find_package(Coin)
find_package(SoQt4)
if (NOT SOQT_LIBRARY_FOUND)
  message(STATUS "WARNING: SoQt4 not found, disabling QtCoin GUI plugin. Please install SoQt (http://www.coin3d.org)")
endif ()

find_package(OpenGL)
if(NOT OPENGL_FOUND)
  message(STATUS "WARNING: OpenGL not found. This library is not necessary, but it is good to link with")
endif()

if (QT_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND AND SOQT_LIBRARY_FOUND AND COIN_LIBRARY_FOUND)
  message(STATUS "Detected SoQt/Coin3D GUI, making plugin")
  include_directories(${OPENGL_INCLUDE_DIR} ${SOQT_INCLUDE_DIRS} ${COIN_INCLUDE_DIRS})
  link_directories(${OPENRAVE_LINK_DIRS} ${SOQT_LINK_DIRS} ${COIN_LINK_DIRS})

  # generate rules for building source files from the resources
  # BOOST_TT_HAS_OPERATOR_HPP_INCLUDED: https://bugreports.qt-project.org/browse/QTBUG-22829
  set(QTCOIN_MOCS qtcoinviewer.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_OUTPUT_FILES ${QTCOIN_MOCS})

  check_include_file(X11/Xlib.h HAVE_X11_XLIB_H)

   # build sources, moc'd sources, and rcc'd sources
  add_library(qtcoinrave SHARED ivselector.cpp item.cpp qtcoinviewer.cpp qtcoinrave.cpp ${MOC_OUTPUT_FILES} item.h ivselector.h qtcoin.h qtcoinviewer.h qtcameraviewer.h ivmodelloader.cpp)

  set(QTCOIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}")
  if( HAVE_X11_XLIB_H )
    set(QTCOIN_COMPILE_FLAGS "${QTCOIN_COMPILE_FLAGS} -DHAVE_X11_XLIB_H")
  endif()
  set(QTCOIN_LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${COIN_LINK_FLAGS} ${SOQT_LINK_FLAGS}")

  set (extLIBS ${QT_LIBRARIES} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} ${SOQT_LIBRARY} ${OPENGL_LIBRARIES})
  if( MSVC )
    set(extLIBS ${extLIBS} winmm imm32 ws2_32)
  endif()

  target_link_libraries (qtcoinrave ${extLIBS} libopenrave)
  set_target_properties(qtcoinrave PROPERTIES COMPILE_FLAGS "${QTCOIN_COMPILE_FLAGS} ${SOQT_CXXFLAGS} ${COIN_CXXFLAGS}" LINK_FLAGS "${QTCOIN_LINK_FLAGS}")
  install(TARGETS qtcoinrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave)
  
  if( MSVC )
    # have to install the DLLs
    install(DIRECTORY "${SOQT_LINK_DIRS}/" DESTINATION bin COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave FILES_MATCHING PATTERN "*-${MSVC_PREFIX}-*.dll")
  endif()

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-QTCOINRAVE_DISPLAY_NAME "Plugin for a Qt/Coin3d/SoQt viewer" PARENT_SCOPE)
  set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave PARENT_SCOPE)
  
else()
  message(STATUS "WARNING: Could not find Coin3D/SoQt/Qt libraries, will not compile qtcoinrave viewer.")
endif()
