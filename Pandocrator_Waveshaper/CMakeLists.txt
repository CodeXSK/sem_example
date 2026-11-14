cmake_minimum_required(VERSION 3.19)

project(WaveshaperSEM LANGUAGES CXX)
if(WIN32)
    enable_language(RC)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

# Optional:
# cmake -S . -B build -A x64 -DSE_SDK_ROOT="C:/path/to/SynthEdit_SDK"
set(SE_SDK_ROOT "" CACHE PATH "Path to a local SynthEdit_SDK checkout")

if(SE_SDK_ROOT)
    set(SE_SDK_DIR "${SE_SDK_ROOT}/se_sdk3")
else()
    include(FetchContent)
    FetchContent_Declare(
        synthedit_sdk
        GIT_REPOSITORY https://github.com/JeffMcClintock/SynthEdit_SDK.git
        GIT_TAG master
        # Match the official SynthEdit example: fetch the repository without
        # configuring/building all example modules in its root CMake project.
        SOURCE_SUBDIR se_sdk3
    )
    FetchContent_MakeAvailable(synthedit_sdk)
    set(SE_SDK_DIR "${synthedit_sdk_SOURCE_DIR}/se_sdk3")
endif()

if(NOT EXISTS "${SE_SDK_DIR}/mp_sdk_audio.h")
    message(FATAL_ERROR
        "SynthEdit SDK3 was not found. Set SE_SDK_ROOT to the folder that contains se_sdk3.")
endif()

add_library(Waveshaper MODULE
    Waveshaper.cpp
    WaveshaperDSP.h
    Waveshaper.xml
    "${SE_SDK_DIR}/mp_sdk_common.cpp"
    "${SE_SDK_DIR}/mp_sdk_audio.cpp"
)

if(WIN32)
    target_sources(Waveshaper PRIVATE Waveshaper.rc resource.h)
elseif(APPLE)
    set_target_properties(Waveshaper PROPERTIES BUNDLE TRUE BUNDLE_EXTENSION "sem")
    set_source_files_properties(Waveshaper.xml PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
endif()

target_include_directories(Waveshaper PRIVATE
    "${SE_SDK_DIR}"
)

target_compile_definitions(Waveshaper PRIVATE
    SE_TARGET_SEM
    $<$<CONFIG:Debug>:_DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
)

if(WIN32)
    target_compile_definitions(Waveshaper PRIVATE UNICODE _UNICODE)
endif()

if(MSVC)
    target_link_options(Waveshaper PRIVATE /SUBSYSTEM:WINDOWS)
    set_property(
        TARGET Waveshaper
        PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
endif()

set_target_properties(Waveshaper PROPERTIES
    PREFIX ""
    SUFFIX ".sem"
)
