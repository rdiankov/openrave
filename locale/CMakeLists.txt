set(LANGUAGES
    en_US
    ja_JP
)

find_program(GETTEXT_MSGFMT_EXECUTABLE msgfmt)
if(NOT GETTEXT_MSGFMT_EXECUTABLE)
    message(WARNING "msgfmt is not available, skipping locale compilation")

else(NOT GETTEXT_MSGFMT_EXECUTABLE)

    foreach(LANG ${LANGUAGES})
        file(
            GLOB DOMAINS
            RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/${LANG}/LC_MESSAGES
            ${CMAKE_CURRENT_SOURCE_DIR}/${LANG}/LC_MESSAGES/*.po
        )
        foreach(DOMAIN ${DOMAINS})
            get_filename_component(DOMAIN ${DOMAIN} NAME_WE)

            add_custom_command(
                OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${LANG}/LC_MESSAGES
                COMMAND ${CMAKE_COMMAND} -E make_directory
                    ${CMAKE_CURRENT_BINARY_DIR}/${LANG}/LC_MESSAGES
            )

            add_custom_command(
                OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${LANG}/LC_MESSAGES/${DOMAIN}.mo
                COMMAND ${GETTEXT_MSGFMT_EXECUTABLE}
                    --verbose --check-format --check-domain
                    --output-file=${CMAKE_CURRENT_BINARY_DIR}/${LANG}/LC_MESSAGES/${DOMAIN}.mo
                    ${CMAKE_CURRENT_SOURCE_DIR}/${LANG}/LC_MESSAGES/${DOMAIN}.po
                DEPENDS
                    ${CMAKE_CURRENT_BINARY_DIR}/${LANG}/LC_MESSAGES
                    ${CMAKE_CURRENT_SOURCE_DIR}/${LANG}/LC_MESSAGES/${DOMAIN}.po
            )

            install(
                FILES ${CMAKE_CURRENT_BINARY_DIR}/${LANG}/LC_MESSAGES/${DOMAIN}.mo
                DESTINATION ${OPENRAVE_LOCALE_INSTALL_DIR}/${LANG}/LC_MESSAGES)

            set(locale-deps ${locale-deps} ${CMAKE_CURRENT_BINARY_DIR}/${LANG}/LC_MESSAGES/${DOMAIN}.mo)

        endforeach(DOMAIN ${DOMAINS})
    endforeach(LANG ${LANGUAGES})

    add_custom_target(locale ALL DEPENDS ${locale-deps})

endif(NOT GETTEXT_MSGFMT_EXECUTABLE)
