#
# Try to find GLEW library and include path.
# Once done this will define
#
# GLEW_FOUND
# GLEW_INCLUDE_DIRS
# GLEW_LIBRARIES
#

include(LibFindMacros)
libfind_package(GLEW OpenGL)

if (WIN32)
  if( NOT GLEW_FOUND )
    # always force
    FIND_PATH( GLEW_INCLUDE_DIR GL/glew.h
	  	$ENV{PROGRAMFILES}/GLEW/include
		${PROJECT_SOURCE_DIR}/src/nvgl/glew/include
		DOC "The directory where GL/glew.h resides")
    FIND_LIBRARY( GLEW_LIBRARY
		NAMES glew GLEW glew32 glew32s
		PATHS
		$ENV{PROGRAMFILES}/GLEW/lib
		${PROJECT_SOURCE_DIR}/src/nvgl/glew/bin
		${PROJECT_SOURCE_DIR}/src/nvgl/glew/lib
		DOC "The GLEW library")
     FIND_LIBRARY( OPENGL_LIBRARY
		NAMES opengl32 OPENGL opengl32
		PATHS
		$ENV{PROGRAMFILES}/lib
		DOC "The OpenGL library")
    if( NOT GLEW_LIBRARY OR NOT OPENGL_LIBRARY )
      set(GLEW_LIBRARY glew32 opengl32 glu32)
      message(STATUS "glew library not found, forcing ${GLEW_LIBRARY}")
    else()
      set(GLEW_LIBRARY ${GLEW_LIBRARY} ${OPENGL_LIBRARY})
    endif()
    set(GLEW_FOUND 1)
  endif()
else (WIN32)
	FIND_PATH( GLEW_INCLUDE_DIR GL/glew.h
		/usr/include
		/usr/local/include
		/sw/include
		/opt/local/include
		DOC "The directory where glew.h resides")
	FIND_LIBRARY( GLEW_LIBRARY
		NAMES GLEW glew
		PATHS
		/usr/lib64
		/usr/lib
		/usr/local/lib64
		/usr/local/lib
		/sw/lib
		/opt/local/lib
		DOC "The GLEW library")
ENDIF (WIN32)

MARK_AS_ADVANCED( GLEW_FOUND
	 GLEW_INCLUDE_PATH)

set(GLEW_PROCESS_INCLUDES GLEW_INCLUDE_DIR OPENGL_INCLUDE_DIR)
set(GLEW_PROCESS_LIBS GLEW_LIBRARY OPENGL_LIBRARIES)
LIBFIND_PROCESS(GLEW)
