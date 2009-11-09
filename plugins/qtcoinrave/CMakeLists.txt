############################
# qtcoinrave openrave plugin
############################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

if( MSVC )
  set(QT_MOC_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/moc.exe")
  set(QT_FOUND 1)
else()
  find_package(Qt4)
  if (NOT QT_FOUND)
    message(STATUS "WARNING: Qt4 not found, disabling QtCoin GUI plugin")
  endif ()
endif()

find_package(SoQt4)
if (NOT SOQT_LIBRARY_FOUND)
  message(STATUS "WARNING: SoQt4 not found, disabling QtCoin GUI plugin. Please install SoQt (http://www.coin3d.org)")
endif ()

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

if (QT_FOUND AND SOQT_LIBRARY_FOUND)

  message(STATUS "Detected SoQt/Coin3D GUI, making plugin")

  include_directories(${CMAKE_SOURCE_DIR})
  include_directories(${QT_INCLUDE_DIRS})
  include_directories(${SOQT_INCLUDE_DIRS})

  add_definitions(${SOQT_CXXFLAGS})
  link_directories(${OPENRAVE_LINK_DIRS} ${SOQT_LINK_DIRS})

  # generate rules for building source files from the resources
  set(QTCOIN_MOCS qtcoinviewer.h)

  if( MSVC )
    set(MOC_OUTPUT_FILES "${CMAKE_CURRENT_BINARY_DIR}/moc_qtcoinviewer.cpp")
    add_custom_command(
      OUTPUT ${MOC_OUTPUT_FILES}
      COMMAND ${QT_MOC_EXECUTABLE}
      ARGS -I${CMAKE_SOURCE_DIR}/inc/qt/include -I${CMAKE_SOURCE_DIR}/inc/qt/include/qt -I${CMAKE_SOURCE_DIR}/inc/qt/include/qtgui -I${CMAKE_SOURCE_DIR}/inc/qt/include/qtcore -I${CMAKE_SOURCE_DIR}/inc/qt/include/qtopengl -o ${MOC_OUTPUT_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/qtcoinviewer.h
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/qtcoinviewer.h
      )
  else()
    include( ${QT_USE_FILE} )
    qt4_wrap_cpp(MOC_OUTPUT_FILES ${QTCOIN_MOCS})
    #QT4_ADD_RESOURCES(QTCOIN_RCC_SRCS ${QTCOIN_RCCS})
  endif()

   # build sources, moc'd sources, and rcc'd sources
  add_library(qtcoinrave SHARED aviUtil.cpp IvSelector.cpp Item.cpp qtcoinviewer.cpp qtcoinravemain.cpp ${MOC_OUTPUT_FILES})

  if( ENABLE_VIDEORECORDING )
    pkg_check_modules(FFMPEG libavformat libavcodec)

    if (FFMPEG_FOUND)
      message(STATUS "enabling FFMPEG support")
      add_definitions(${FFMPEG_CFLAGS})
      add_definitions(-DENABLE_FFMPEG)

      pkg_check_modules(FFMPEG_SWSCALE libswscale)

      set(CMAKE_REQUIRED_INCLUDES ${FFMPEG_libavformat_INCLUDEDIR} ${FFMPEG_libavcodec_INCLUDEDIR} ${FFMPEG_libswscale_INCLUDEDIR})
      check_include_file(libswscale/swscale.h HAVE_NEW_SWSCALE_H)
      check_include_file(libavformat/avformat.h HAVE_NEW_AVFORMAT_H)
      check_include_file(libavcodec/avcodec.h HAVE_NEW_AVCODEC_H)

      if( FFMPEG_SWSCALE_FOUND AND HAVE_NEW_SWSCALE_H AND HAVE_NEW_AVFORMAT_H AND HAVE_NEW_AVCODEC_H )
        set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_SWSCALE_LIBRARIES})
        add_definitions(-DHAVE_NEW_FFMPEG)
      endif()
    elseif(MSVC)
      set (FFMPEG_LIBRARIES vfw32)
    else ()
      message(STATUS "WARNING: FFMPEG not found, support disabled")
    endif ()
  else()
    set(FFMPEG_LIBRARIES)
  endif()

  set (extLIBS ${QT_LIBRARIES} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} ${SOQT_LIBRARY} ${PTHREADS_LIBRARY} ${FFMPEG_LIBRARIES})

  set(QTCOIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}")
  set(QTCOIN_LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${COIN_LINK_FLAGS} ${SOQT_LINK_FLAGS}")

  if( MSVC )
    set(extLIBS ${extLIBS} winmm imm32 ws2_32)
    # soqt libs not compiled correct so multiply defined symbols
    set(QTCOIN_LINK_FLAGS "${QTCOIN_LINK_FLAGS} /FORCE:MULTIPLE")
  endif()

  target_link_libraries (qtcoinrave ${extLIBS} libopenrave)
  include_directories(${CMAKE_SOURCE_DIR} ${FFMPEG_libavformat_INCLUDEDIR} ${FFMPEG_libavcodec_INCLUDEDIR} ${FFMPEG_libswscale_INCLUDEDIR})
  add_dependencies(qtcoinrave libopenrave libopenrave_static)
  set_target_properties(qtcoinrave PROPERTIES COMPILE_FLAGS "${QTCOIN_COMPILE_FLAGS}"
                                      LINK_FLAGS "${QTCOIN_LINK_FLAGS}")
  install(TARGETS qtcoinrave DESTINATION share/openrave/plugins)

else()
    message(SEND_ERROR "ERROR: Could not find SoQt/Qt.")
endif()
