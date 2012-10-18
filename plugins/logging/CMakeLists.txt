###########################################
# logging openrave plugin
###########################################
set(logging_SOURCES logging.cpp plugindefs.h)
set(ENABLE_VIDEORECORDING)

if( OPT_VIDEORECORDING )
  pkg_check_modules(FFMPEG libavformat libavcodec)
  if (NOT MSVC AND FFMPEG_FOUND)
    message(STATUS "enabling FFMPEG support")
    pkg_check_modules(FFMPEG_SWSCALE libswscale)

    set(CMAKE_REQUIRED_INCLUDES ${FFMPEG_libavformat_INCLUDEDIR} ${FFMPEG_libavcodec_INCLUDEDIR} ${FFMPEG_libswscale_INCLUDEDIR})
    check_include_file(libswscale/swscale.h HAVE_NEW_SWSCALE_H)
    check_include_file(libavformat/avformat.h HAVE_NEW_AVFORMAT_H)
    check_include_file(libavcodec/avcodec.h HAVE_NEW_AVCODEC_H)

    check_include_file(ffmpeg/avformat.h HAVE_OLD_AVFORMAT_H)
    check_include_file(ffmpeg/avcodec.h HAVE_OLD_AVCODEC_H)

    if( HAVE_NEW_SWSCALE_H AND HAVE_NEW_AVFORMAT_H AND HAVE_NEW_AVCODEC_H )
      if( FFMPEG_SWSCALE_FOUND )
        set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_SWSCALE_LIBRARIES})
        set(ENABLE_VIDEORECORDING 1)
        add_definitions("${FFMPEG_CFLAGS} -DHAVE_NEW_FFMPEG")
      else()
        check_library_exists(swscale sws_scale "" SWSCALE_FOUND_LIB)
        if( SWSCALE_FOUND_LIB )
          set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} swscale)
          set(ENABLE_VIDEORECORDING 1)
          add_definitions("${FFMPEG_CFLAGS} -DHAVE_NEW_FFMPEG")
        else()
          message(STATUS "could not find swscale library files!")
        endif()
      endif()
    elseif(HAVE_OLD_AVFORMAT_H AND HAVE_OLD_AVCODEC_H)
      add_definitions(${FFMPEG_CFLAGS})
      set(ENABLE_VIDEORECORDING 1)
    endif()
    include_directories(${FFMPEG_libavformat_INCLUDEDIR} ${FFMPEG_libavcodec_INCLUDEDIR} ${FFMPEG_libswscale_INCLUDEDIR})

    set(CMAKE_REQUIRED_INCLUDES ${FFMPEG_libavformat_INCLUDEDIR} ${FFMPEG_libavcodec_INCLUDEDIR} ${FFMPEG_libswscale_INCLUDEDIR})
    set(CMAKE_REQUIRED_FLAGS "${FFMPEG_CFLAGS} ")
    check_cxx_source_compiles("
      #define __STDC_CONSTANT_MACROS
      #include <stdint.h>
      #include <libavformat/avformat.h>
      #include <libavcodec/avcodec.h>
      int main()
      {
	return AVMEDIA_TYPE_VIDEO;
      }"
      HAS_AVMEDIA_TYPE_VIDEO)
    set(CMAKE_REQUIRED_FLAGS)
    set(CMAKE_REQUIRED_INCLUDES)
    if( HAS_AVMEDIA_TYPE_VIDEO )
      add_definitions(-DHAS_AVMEDIA_TYPE_VIDEO)
    endif()

  elseif(MSVC)
    set (FFMPEG_LIBRARIES vfw32)
    set(ENABLE_VIDEORECORDING 1)
  else ()
    message(STATUS "WARNING: FFMPEG not found, support disabled")
  endif ()
else()
  set(FFMPEG_LIBRARIES)
endif()

if( ENABLE_VIDEORECORDING )
  add_definitions(-DENABLE_VIDEORECORDING)
  set(logging_SOURCES ${logging_SOURCES} viewerrecorder.cpp)
endif()

add_library(logging SHARED ${logging_SOURCES})
target_link_libraries(logging libopenrave ${FFMPEG_LIBRARIES})
set_target_properties(logging PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS logging DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-LOGGING_DISPLAY_NAME "OpenRAVE Logging, includes video recorders" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-logging PARENT_SCOPE)
