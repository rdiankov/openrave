############################
# qtcoinrave openrave plugin
############################
find_package(Coin QUIET)
find_package(SoQt QUIET)

set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL)
if(NOT OPENGL_FOUND)
  message(WARNING "OpenGL not found. This library is not necessary, but it is good to link with")
endif()

find_package(Qt5 COMPONENTS Core Gui) # Declarative)
if (NOT Qt5Gui_FOUND)
  message(WARNING "Qt5 not found, is your QTDIR enviornment variable set?")
endif()

if (Qt5Core_FOUND AND Qt5Gui_FOUND AND SoQt_FOUND AND Coin_FOUND)
  message(STATUS "Detected SoQt/Coin3D GUI, making plugin")
  include_directories(${OPENGL_INCLUDE_DIR} ${SOQT_INCLUDE_DIRS} ${COIN_INCLUDE_DIRS})
  link_directories(${OPENRAVE_LINK_DIRS} ${SOQT_LINK_DIRS} ${COIN_LINK_DIRS})

  # generate rules for building source files from the resources
  set(QTCOIN_MOCS qtcoinviewer.h)
  #include(${QT_USE_FILE})
  qt5_wrap_cpp(MOC_OUTPUT_FILES ${QTCOIN_MOCS})

  check_include_file(X11/Xlib.h HAVE_X11_XLIB_H)

   # build sources, moc'd sources, and rcc'd sources
  add_library(qtcoinrave SHARED ivselector.cpp item.cpp qtcoinviewer.cpp qtcoinrave.cpp ${MOC_OUTPUT_FILES} item.h ivselector.h qtcoin.h qtcoinviewer.h qtcameraviewer.h ivmodelloader.cpp)

  set(QTCOIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DQT_DISABLE_DEPRECATED_BEFORE=0x000000 -Wno-shadow")
  if( HAVE_X11_XLIB_H )
    set(QTCOIN_COMPILE_FLAGS "${QTCOIN_COMPILE_FLAGS} -DHAVE_X11_XLIB_H")
  endif()
  set(QTCOIN_LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${COIN_LINK_FLAGS} ${SOQT_LINK_FLAGS}")

  set (extLIBS ${QT_LIBRARIES} ${COIN_LIBRARY} ${SOQT_LIBRARY} ${OPENGL_LIBRARIES})
  if( MSVC )
    set(extLIBS ${extLIBS} winmm imm32 ws2_32)
  endif()

  target_link_libraries(qtcoinrave PRIVATE boost_assertion_failed PUBLIC libopenrave ${extLIBS})
  set_target_properties(qtcoinrave PROPERTIES COMPILE_FLAGS "${QTCOIN_COMPILE_FLAGS} ${SOQT_CXXFLAGS} ${COIN_CXXFLAGS}" LINK_FLAGS "${QTCOIN_LINK_FLAGS}")
  install(TARGETS qtcoinrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave)
  
  if( MSVC )
    # have to install the DLLs
    install(DIRECTORY "${SOQT_LINK_DIRS}/" DESTINATION bin COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave FILES_MATCHING PATTERN "*-${MSVC_PREFIX}-*.dll")
  endif()

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-QTCOINRAVE_DISPLAY_NAME "Plugin for a Qt/Coin3d/SoQt viewer" PARENT_SCOPE)
  set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave PARENT_SCOPE)
  
else()
  message(WARNING "Could not find Coin3D/SoQt/Qt libraries, will not compile qtcoinrave viewer.")
endif()
