# Locate Collada
# This module defines:
# COLLADA_INCLUDE_DIR, where to find the headers
#
# COLLADA_LIBRARY, COLLADA_LIBRARY_DEBUG
# COLLADA_FOUND, if false, do not try to link to Collada dynamically
#
# COLLADA_LIBRARY_STATIC, COLLADA_LIBRARY_STATIC_DEBUG
# COLLADA_STATIC_FOUND, if false, do not try to link to Collada statically
#
# $COLLADA_DIR is an environment variable that would
# correspond to the ./configure --prefix=$COLLADA_DIR
#
# Created by Robert Osfield. 

SET(COLLADA_DOM_ROOT "$ENV{COLLADA_DIR}/dom" CACHE PATH "Location of Collada DOM directory")

IF(APPLE)
    SET(COLLADA_BUILDNAME "mac")
ELSEIF(MINGW)
    SET(COLLADA_BUILDNAME "mingw")
ELSEIF(MSVC90)
    SET(COLLADA_BUILDNAME "vc9")
ELSE(APPLE)
    SET(COLLADA_BUILDNAME "vc8")
ENDIF(APPLE)


FIND_PATH(COLLADA_INCLUDE_DIR dae.h
    ${COLLADA_DOM_ROOT}/include
    $ENV{COLLADA_DIR}/include
    $ENV{COLLADA_DIR}
    $ENV{OSGDIR}/include
    $ENV{OSGDIR}
    $ENV{OSG_ROOT}/include
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/include
    /usr/local/include/colladadom
    /usr/include/
    /usr/include/colladadom
    /sw/include # Fink
    /opt/local/include # DarwinPorts
    /opt/csw/include # Blastwave
    /opt/include
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/include
    /usr/freeware/include
)

FIND_LIBRARY(COLLADA_DYNAMIC_LIBRARY 
    NAMES collada_dom collada15dom Collada15Dom libcollada15dom21 libcollada15dom22
    PATHS
    ${COLLADA_DOM_ROOT}/build/${COLLADA_BUILDNAME}-1.5
    $ENV{COLLADA_DIR}/build/${COLLADA_BUILDNAME}-1.5
    $ENV{COLLADA_DIR}/lib
    $ENV{COLLADA_DIR}/lib-dbg
    $ENV{COLLADA_DIR}
    $ENV{OSGDIR}/lib
    $ENV{OSGDIR}
    $ENV{OSG_ROOT}/lib
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
    /usr/freeware/lib64
)

FIND_LIBRARY(COLLADA_DYNAMIC_LIBRARY_DEBUG 
    NAMES collada_dom-d collada15dom-d Collada15Dom-d libcollada15dom21-d libcollada15dom22-d
    PATHS
    ${COLLADA_DOM_ROOT}/build/${COLLADA_BUILDNAME}-1.5-d
    $ENV{COLLADA_DIR}/build/${COLLADA_BUILDNAME}-1.5-d
    $ENV{COLLADA_DIR}/lib
    $ENV{COLLADA_DIR}/lib-dbg
    $ENV{COLLADA_DIR}
    $ENV{OSGDIR}/lib
    $ENV{OSGDIR}
    $ENV{OSG_ROOT}/lib
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
    /usr/freeware/lib64
)

FIND_LIBRARY(COLLADA_STATIC_LIBRARY 
    NAMES libcollada15dom21-s  libcollada15dom22-s collada15dom-s
    PATHS
    ${COLLADA_DOM_ROOT}/build/${COLLADA_BUILDNAME}-1.5
    $ENV{COLLADA_DIR}/build/${COLLADA_BUILDNAME}-1.5
    $ENV{COLLADA_DIR}/lib
    $ENV{COLLADA_DIR}/lib-dbg
    $ENV{COLLADA_DIR}
    $ENV{OSGDIR}/lib
    $ENV{OSGDIR}
    $ENV{OSG_ROOT}/lib
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
    /usr/freeware/lib64
)

FIND_LIBRARY(COLLADA_STATIC_LIBRARY_DEBUG 
    NAMES collada_dom-sd collada15dom-sd libcollada15dom21-sd libcollada15dom22-sd
    PATHS
    ${COLLADA_DOM_ROOT}/build/${COLLADA_BUILDNAME}-1.5-d
    $ENV{COLLADA_DIR}/build/${COLLADA_BUILDNAME}-1.5-d
    $ENV{COLLADA_DIR}/lib
    $ENV{COLLADA_DIR}/lib-dbg
    $ENV{COLLADA_DIR}
    $ENV{OSGDIR}/lib
    $ENV{OSGDIR}
    $ENV{OSG_ROOT}/lib
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
    /usr/freeware/lib64
)

    # find extra libraries that the static linking requires

    FIND_PACKAGE(LibXml2)
    IF (LIBXML2_FOUND)
        SET(COLLADA_LIBXML_LIBRARY ${LIBXML2_LIBRARIES})
    ELSE(LIBXML2_FOUND)
        IF(WIN32)
            FIND_LIBRARY(COLLADA_LIBXML_LIBRARY
                NAMES libxml2
                PATHS
                ${COLLADA_DOM_ROOT}/external-libs/libxml2/win32/lib
                ${COLLADA_DOM_ROOT}/external-libs/libxml2/mingw/lib
            )
        ENDIF(WIN32)
    ENDIF(LIBXML2_FOUND)
    
    FIND_PACKAGE(ZLIB)
    IF (ZLIB_FOUND)
        SET(COLLADA_ZLIB_LIBRARY ${ZLIB_LIBRARY})
    ELSE(ZLIB_FOUND)
        IF(WIN32)
            FIND_LIBRARY(COLLADA_ZLIB_LIBRARY
                NAMES zlib
                PATHS
                ${COLLADA_DOM_ROOT}/external-libs/libxml2/win32/lib
                ${COLLADA_DOM_ROOT}/external-libs/libxml2/mingw/lib
            )
        ENDIF(WIN32)
    ENDIF(ZLIB_FOUND)

    FIND_LIBRARY(COLLADA_PCRECPP_LIBRARY
        NAMES pcrecpp
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mac
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mingw
    )

    FIND_LIBRARY(COLLADA_PCRECPP_LIBRARY_DEBUG
        NAMES pcrecpp-d
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mac
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mingw
    )

		message(STATUS "COLLADA_ZLIB_LIBRARY: ${COLLADA_ZLIB_LIBRARY}")
		message(STATUS "COLLADA_PCRECPP_LIBRARY: ${COLLADA_PCRECPP_LIBRARY}")
		
    FIND_LIBRARY(COLLADA_PCRE_LIBRARY
        NAMES pcre
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mac
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mingw
    )

    FIND_LIBRARY(COLLADA_PCRE_LIBRARY_DEBUG
        NAMES pcre-d
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mac
        ${COLLADA_DOM_ROOT}/external-libs/pcre/lib/mingw
    )

		message(STATUS "COLLADA_PCRE_LIBRARY: ${COLLADA_PCRE_LIBRARY}")

    FIND_LIBRARY(COLLADA_MINIZIP_LIBRARY
        NAMES minizip
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/minizip/win32/lib
        ${COLLADA_DOM_ROOT}/external-libs/minizip/mac
    )

    FIND_LIBRARY(COLLADA_MINIZIP_LIBRARY_DEBUG
        NAMES minizip-d
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/minizip/win32/lib
        ${COLLADA_DOM_ROOT}/external-libs/minizip/mac
    )

		message(STATUS "COLLADA_MINIZIP_LIBRARY: ${COLLADA_MINIZIP_LIBRARY}")
		
    FIND_LIBRARY(COLLADA_BOOST_FILESYSTEM_LIBRARY
        NAMES libboost_filesystem boost_filesystem
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/mingw
    )

    FIND_LIBRARY(COLLADA_BOOST_FILESYSTEM_LIBRARY_DEBUG
        NAMES libboost_filesystem-d boost_filesystem-d
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/mingw
    )

		message(STATUS "COLLADA_BOOST_FILESYSTEM_LIBRARY: ${COLLADA_BOOST_FILESYSTEM_LIBRARY}")

    FIND_LIBRARY(COLLADA_BOOST_SYSTEM_LIBRARY
        NAMES libboost_system boost_system
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/mingw
    )

    FIND_LIBRARY(COLLADA_BOOST_SYSTEM_LIBRARY_DEBUG
        NAMES libboost_system-d boost_system-d
        PATHS
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/${COLLADA_BUILDNAME}
        ${COLLADA_DOM_ROOT}/external-libs/boost/lib/mingw
    )


SET(COLLADA_FOUND "NO")
IF(COLLADA_DYNAMIC_LIBRARY OR COLLADA_STATIC_LIBRARY)
  	message(STATUS "COLLADA_DYNAMIC_LIBRARY ${COLLADA_DYNAMIC_LIBRARY}")
		message(STATUS "COLLADA_STATIC_LIBRARY ${COLLADA_STATIC_LIBRARY}")
    IF   (COLLADA_INCLUDE_DIR)
        SET(COLLADA_FOUND "YES")
        SET(COLLADA_INCLUDE_DIR ${COLLADA_INCLUDE_DIR} ${COLLADA_INCLUDE_DIR}/1.5)
        
        message(STATUS "-------<<<>-----------libpcrecpp_LIBRARIES:: ${libpcrecpp_LIBRARIES}")
				message(STATUS "-------<<<>-----------ZLIB_LIBRARIES:: ${ZLIB_LIBRARIES}")
				message(STATUS "-------<<<>-----------Boost_FILESYSTEM_LIBRARY:: ${Boost_FILESYSTEM_LIBRARY}")
				message(STATUS "-------<<<>-----------Boost_SYSTEM_LIBRARY:: ${Boost_SYSTEM_LIBRARY}")
				
    ENDIF(COLLADA_INCLUDE_DIR)

    IF( COLLADA_DYNAMIC_LIBRARY )
      SET(COLLADA_LIBRARY ${COLLADA_DYNAMIC_LIBRARY})
    ELSE( COLLADA_DYNAMIC_LIBRARY )
      SET(COLLADA_LIBRARY ${COLLADA_STATIC_LIBRARY})
    ENDIF( COLLADA_DYNAMIC_LIBRARY )
    
    SET(COLLADA_LIBS minizip ${libpcrecpp_LIBRARIES} ${ZLIB_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    set (coreLIBS ${coreLIBS} ${COLLADA_LIBS})
    
ENDIF(COLLADA_DYNAMIC_LIBRARY OR COLLADA_STATIC_LIBRARY)
