project(StrongestHarmonicEQ LANGUAGES CXX)

set(MODULE_SOURCES
    StrongestHarmonicEQ.cpp
    StrongestPeakEngine.h
    StrongestHarmonicEQ.xml
)

if(WIN32)
    list(APPEND MODULE_SOURCES
        StrongestHarmonicEQ.rc
        resource.h
    )
endif()

add_library(${PROJECT_NAME} MODULE
    ${MODULE_SOURCES}
    "${SE_SDK_FOLDER}/mp_sdk_common.cpp"
    "${SE_SDK_FOLDER}/mp_sdk_audio.cpp"
    "${SE_SDK_FOLDER}/mp_sdk_common.h"
    "${SE_SDK_FOLDER}/mp_sdk_audio.h"
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        "${SE_SDK_FOLDER}"
        "${SE_SHARED_FOLDER}"
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        SE_TARGET_SEM
        _UNICODE
        UNICODE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        $<$<CONFIG:Debug>:_DEBUG>
        $<$<CONFIG:Release>:NDEBUG>
)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /fp:precise /GS)
    set_property(
        TARGET ${PROJECT_NAME}
        PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
endif()

if(WIN32)
    target_link_options(${PROJECT_NAME} PRIVATE "/SUBSYSTEM:WINDOWS")
endif()

# This is the important part: legacy SynthEdit module extension.
set_target_properties(${PROJECT_NAME} PROPERTIES
    PREFIX ""
    SUFFIX ".sem"
)

if(WIN32 AND SE_COPY_TO_SEM_FOLDER)
    add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${PROJECT_NAME}>"
            "C:/Program Files/Common Files/SynthEdit/modules/StrongestHarmonicEQ.sem"
        COMMENT "Copying StrongestHarmonicEQ.sem to SynthEdit modules folder"
        VERBATIM
    )
endif()
