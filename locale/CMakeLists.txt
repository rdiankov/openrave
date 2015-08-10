set(LANGUAGES
    en_US
    ja_JP
)

file(GLOB DOMAINS RELATIVE ${CMAKE_SOURCE_DIR}/locale/domains ${CMAKE_SOURCE_DIR}/locale/domains/*)

find_program(GETTEXT_MSGFMT_EXECUTABLE msgfmt)
if(NOT GETTEXT_MSGFMT_EXECUTABLE)
    message(FATAL_ERROR "msgfmt not found")
endif(NOT GETTEXT_MSGFMT_EXECUTABLE)

foreach(LANG ${LANGUAGES})
    foreach(DOMAIN ${DOMAINS})

        add_custom_command(
            OUTPUT ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES
            COMMAND ${CMAKE_COMMAND} -E make_directory
                    ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES
        )

        add_custom_command(
            OUTPUT ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES/${DOMAIN}.mo
            COMMAND
                ${GETTEXT_MSGFMT_EXECUTABLE} --verbose --check-format --check-domain --output-file=${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES/${DOMAIN}.mo ${DOMAIN}.po
            DEPENDS
                ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES
                ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES/${DOMAIN}.po
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES
        )

        install(FILES ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES/${DOMAIN}.mo
                DESTINATION ${OPENRAVE_LOCALE_INSTALL_ABSOLUTE_DIR}/${LANG}/LC_MESSAGES)

        set(locale-deps ${locale-deps} ${CMAKE_SOURCE_DIR}/locale/${LANG}/LC_MESSAGES/${DOMAIN}.mo)

    endforeach(DOMAIN ${DOMAINS})
endforeach(LANG ${LANGUAGES})

add_custom_target(locale ALL DEPENDS ${locale-deps})
