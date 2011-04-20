##
# Copyright (c) 2010 Daniel Pfeifer <daniel@pfeifer-mail.de>
##

find_program(DEBUILD_EXECUTABLE debuild)
find_program(DPUT_EXECUTABLE dput)

if(NOT DEBUILD_EXECUTABLE OR NOT DPUT_EXECUTABLE)
  return()
endif(NOT DEBUILD_EXECUTABLE OR NOT DPUT_EXECUTABLE)

# DEBIAN/control
# debian policy enforce lower case for package name
# Package: (mandatory)
IF(NOT CPACK_DEBIAN_PACKAGE_NAME)
  STRING(TOLOWER "${CPACK_PACKAGE_NAME}" CPACK_DEBIAN_PACKAGE_NAME)
ENDIF(NOT CPACK_DEBIAN_PACKAGE_NAME)

# Section: (recommended)
IF(NOT CPACK_DEBIAN_PACKAGE_SECTION)
  SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
ENDIF(NOT CPACK_DEBIAN_PACKAGE_SECTION)

# Priority: (recommended)
IF(NOT CPACK_DEBIAN_PACKAGE_PRIORITY)
  SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
ENDIF(NOT CPACK_DEBIAN_PACKAGE_PRIORITY)

file(STRINGS ${CPACK_PACKAGE_DESCRIPTION_FILE} DESC_LINES)
foreach(LINE ${DESC_LINES})
  set(DEB_LONG_DESCRIPTION "${DEB_LONG_DESCRIPTION} ${LINE}\n")
endforeach(LINE ${DESC_LINES})

file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/Debian)
set(DEBIAN_SOURCE_DIR ${CMAKE_BINARY_DIR}/Debian/${CPACK_DEBIAN_PACKAGE_NAME}-src)
file(MAKE_DIRECTORY ${DEBIAN_SOURCE_DIR})

if( CPACK_DEBIAN_PACKAGE_SOURCE_COPY )
  execute_process(COMMAND ${CPACK_DEBIAN_PACKAGE_SOURCE_COPY} "${CMAKE_SOURCE_DIR}" "${DEBIAN_SOURCE_DIR}")
else()
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR} ${DEBIAN_SOURCE_DIR})
  execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory ${DEBIAN_SOURCE_DIR}/.git)
  execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory ${DEBIAN_SOURCE_DIR}/.svn)
endif()

# remove unnecessary folders
foreach(REMOVE_DIR ${CPACK_DEBIAN_PACKAGE_REMOVE_SOURCE_FILES})
  file(REMOVE_RECURSE ${DEBIAN_SOURCE_DIR}/${REMOVE_DIR})
endforeach()

file(MAKE_DIRECTORY ${DEBIAN_SOURCE_DIR}/debian)

##############################################################################
# debian/control
set(DEBIAN_CONTROL ${DEBIAN_SOURCE_DIR}/debian/control)
file(WRITE ${DEBIAN_CONTROL}
  "Source: ${CPACK_DEBIAN_PACKAGE_NAME}\n"
  "Section: ${CPACK_DEBIAN_PACKAGE_SECTION}\n"
  "Priority: ${CPACK_DEBIAN_PACKAGE_PRIORITY}\n"
  "Maintainer: ${CPACK_PACKAGE_CONTACT}\n"
  "Build-Depends: "
  )

foreach(DEP ${CPACK_DEBIAN_BUILD_DEPENDS})
  file(APPEND ${DEBIAN_CONTROL} "${DEP}, ")
endforeach(DEP ${CPACK_DEBIAN_BUILD_DEPENDS})

file(APPEND ${DEBIAN_CONTROL} "\n"
  "Standards-Version: 3.8.4\n"
  "Homepage: ${CPACK_PACKAGE_VENDOR}\n"
  "\n"
  "Package: ${CPACK_DEBIAN_PACKAGE_NAME}\n"
  "Architecture: any\n"
  "Suggests: ${CPACK_DEBIAN_BUILD_SUGGESTS}\n"
  "Depends: "
)

foreach(DEP ${CPACK_DEBIAN_PACKAGE_DEPENDS})
  file(APPEND ${DEBIAN_CONTROL} "${DEP}, ")
endforeach(DEP ${CPACK_DEBIAN_PACKAGE_DEPENDS})  

file(APPEND ${DEBIAN_CONTROL} "\n"
  "Description: ${CPACK_PACKAGE_DESCRIPTION_SUMMARY}\n"
  "${DEB_LONG_DESCRIPTION}"
  )

foreach(COMPONENT ${CPACK_COMPONENTS_ALL})
  string(TOUPPER ${COMPONENT} UPPER_COMPONENT)
  set(DEPENDS "\${shlibs:Depends}, \${misc:Depends}")
  foreach(DEP ${CPACK_COMPONENT_${UPPER_COMPONENT}_DEPENDS})
    set(DEPENDS "${DEPENDS}, ${DEP}")
  endforeach(DEP ${CPACK_COMPONENT_${UPPER_COMPONENT}_DEPENDS})
  file(APPEND ${DEBIAN_CONTROL} "\n"
    "Package: ${COMPONENT}\n"
    "Architecture: any\n"
    "Depends: ${DEPENDS}\n"
    "Description: ${CPACK_PACKAGE_DESCRIPTION_SUMMARY}"
    ": ${CPACK_COMPONENT_${UPPER_COMPONENT}_DISPLAY_NAME}\n"
    "${DEB_LONG_DESCRIPTION}"
    " .\n"
    " ${CPACK_COMPONENT_${UPPER_COMPONENT}_DESCRIPTION}\n"
    )
endforeach(COMPONENT ${CPACK_COMPONENTS_ALL})

##############################################################################
# debian/copyright
set(DEBIAN_COPYRIGHT ${DEBIAN_SOURCE_DIR}/debian/copyright)
execute_process(COMMAND ${CMAKE_COMMAND} -E
  copy ${CPACK_RESOURCE_FILE_LICENSE} ${DEBIAN_COPYRIGHT}
  )

##############################################################################
# debian/rules
set(DEBIAN_RULES ${DEBIAN_SOURCE_DIR}/debian/rules)
file(WRITE ${DEBIAN_RULES}
  "#!/usr/bin/make -f\n"
  "\n"
  "BUILDDIR = build_dir\n"
  "\n"
  "build:\n"
  "	mkdir $(BUILDDIR)\n"
  "	cd $(BUILDDIR); cmake -DCMAKE_INSTALL_PREFIX=/usr ..\n"
  "	$(MAKE) -C $(BUILDDIR) preinstall\n"
  "	touch build\n"
  "\n"
  "binary: binary-indep binary-arch\n"
  "\n"
  "binary-indep: build\n"
  "\n"
  "binary-arch: build\n"
  "	cd $(BUILDDIR); cmake -DCOMPONENT=Unspecified -DCMAKE_INSTALL_PREFIX=../debian/tmp/usr -P cmake_install.cmake; $(MAKE) install\n"
  "	mkdir -p debian/tmp/DEBIAN\n"
  "	dpkg-gencontrol -p${CPACK_DEBIAN_PACKAGE_NAME}\n"
  "	dpkg --build debian/tmp ..\n"
  )

foreach(COMPONENT ${CPACK_COMPONENTS_ALL})
  set(PATH debian/tmp_${COMPONENT})
  set(PACKAGE ${CPACK_DEBIAN_PACKAGE_NAME}-${COMPONENT})
  file(APPEND ${DEBIAN_RULES}
    "	cd $(BUILDDIR); cmake -DCOMPONENT=${COMPONENT} -DCMAKE_INSTALL_PREFIX=../${PATH}/usr -P cmake_install.cmake; $(MAKE) install\n"
    "	mkdir -p ${PATH}/DEBIAN\n"
    "	dpkg-gencontrol -p${PACKAGE} -P${PATH}\n"
    "	dpkg --build ${PATH} ..\n"
    )
endforeach(COMPONENT ${CPACK_COMPONENTS_ALL})

file(APPEND ${DEBIAN_RULES}
  "\n"
  "clean:\n"
  "	rm -f build\n"
  "	rm -rf $(BUILDDIR)\n"
  "\n"
  ".PHONY: binary binary-arch binary-indep clean\n"
  )

execute_process(COMMAND chmod +x ${DEBIAN_RULES})

##############################################################################
# debian/compat
file(WRITE ${DEBIAN_SOURCE_DIR}/debian/compat "7")

##############################################################################
# debian/source/format
file(WRITE ${DEBIAN_SOURCE_DIR}/debian/source/format "3.0 (native)")

##############################################################################
# debian/changelog
set(DEBIAN_CHANGELOG ${DEBIAN_SOURCE_DIR}/debian/changelog)
execute_process(COMMAND date -R  OUTPUT_VARIABLE DATE_TIME)
file(WRITE ${DEBIAN_CHANGELOG}
  "${CPACK_DEBIAN_PACKAGE_NAME} (${CPACK_PACKAGE_VERSION}) ${CPACK_DEBIAN_DISTRIBUTION_CODENAME}; urgency=low\n\n"
  "  * Package built with CMake\n\n"
  "  * ChangeLog can be found at https://openrave.svn.sourceforge.net/svnroot/openrave/tags/${OPENRAVE_VERSION}/docs/en/changelog.rst\n\n"
  " -- ${CPACK_PACKAGE_CONTACT}  ${DATE_TIME}"
  )

##############################################################################
# debuild -S
set(DEB_SOURCE_CHANGES
  ${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_source.changes
  )

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/Debian/${DEB_SOURCE_CHANGES}
  COMMAND ${DEBUILD_EXECUTABLE} -S
  WORKING_DIRECTORY ${DEBIAN_SOURCE_DIR}
  )

##############################################################################
# dput ppa:your-lp-id/ppa <source.changes>
add_custom_target(dput ${DPUT_EXECUTABLE} ${DPUT_HOST} ${DEB_SOURCE_CHANGES}
  DEPENDS ${CMAKE_BINARY_DIR}/Debian/${DEB_SOURCE_CHANGES}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Debian
  )
