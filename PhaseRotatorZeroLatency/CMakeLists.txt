cmake_minimum_required(VERSION 3.19)

project(PhaseRotatorZeroLatency)

build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    SOURCE_FILES
        PhaseRotatorZeroLatency.cpp
)

# ---------------------------------------------------------------------------
# macOS SEM bundle safety fix
# ---------------------------------------------------------------------------
# sem_example/plugin_helper.cmake already marks the XML as a bundle Resource.
# This explicit POST_BUILD copy is intentional redundancy: it guarantees that
# the final .sem bundle really contains:
#
#   <module>.sem/Contents/Resources/<module>.xml
#
# This is especially important when the Mac SEM is later copied to the Windows
# SynthEdit "modules_mac" folder. Windows cannot execute the Mach-O factory, so
# SynthEdit must be able to read the XML metadata directly from the bundle.
if(APPLE)
    set(MODULE_XML
        "${CMAKE_CURRENT_SOURCE_DIR}/PhaseRotatorZeroLatency.xml"
    )

    add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD

        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_BUNDLE_DIR:${PROJECT_NAME}>/Contents/Resources"

        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MODULE_XML}"
            "$<TARGET_BUNDLE_DIR:${PROJECT_NAME}>/Contents/Resources/PhaseRotatorZeroLatency.xml"

        # Fail the GitHub/macOS build if the XML did not make it into the SEM.
        COMMAND /bin/sh -c
            "test -f '$<TARGET_BUNDLE_DIR:${PROJECT_NAME}>/Contents/Resources/PhaseRotatorZeroLatency.xml' || { echo 'ERROR: SEM bundle is missing XML resource'; exit 1; }"

        # Also verify that the actual Mac binary exports MP_GetFactory.
        COMMAND /bin/sh -c
            "nm -gU '$<TARGET_FILE:${PROJECT_NAME}>' | grep -q 'MP_GetFactory' || { echo 'ERROR: SEM binary has no MP_GetFactory export'; exit 1; }"

        COMMENT
            "Verifying PhaseRotatorZeroLatency.sem XML resource and factory export"
        VERBATIM
    )
endif()
