cmake_minimum_required(VERSION 3.19)
project(AdaptiveDCBlockerStandalone LANGUAGES CXX RC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

# Optional: -DSE_SDK_ROOT=C:/code/SynthEdit_SDK
set(SE_SDK_ROOT "" CACHE PATH "Path to checkout containing se_sdk3/")
if(SE_SDK_ROOT)
    set(SE_SDK_DIR "${SE_SDK_ROOT}/se_sdk3")
else()
    include(FetchContent)
    FetchContent_Declare(
        synthedit_sdk
        GIT_REPOSITORY https://github.com/JeffMcClintock/SynthEdit_SDK.git
        GIT_TAG origin/master
        SOURCE_SUBDIR se_sdk3
    )
    FetchContent_MakeAvailable(synthedit_sdk)
    set(SE_SDK_DIR "${synthedit_sdk_SOURCE_DIR}/se_sdk3")
endif()

if(NOT EXISTS "${SE_SDK_DIR}/mp_sdk_audio.h")
    message(FATAL_ERROR "SynthEdit SDK3 missing. Supply -DSE_SDK_ROOT=<SynthEdit_SDK>")
endif()

add_library(AdaptiveDCBlocker MODULE
    AdaptiveDCBlocker.cpp
    AdaptiveDCBlockerDSP.h
    AdaptiveDCBlocker.xml
    AdaptiveDCBlocker.rc
    resource.h
    "${SE_SDK_DIR}/mp_sdk_common.cpp"
    "${SE_SDK_DIR}/mp_sdk_audio.cpp"
)
target_include_directories(AdaptiveDCBlocker PRIVATE "${SE_SDK_DIR}")
target_compile_definitions(AdaptiveDCBlocker PRIVATE SE_TARGET_SEM UNICODE _UNICODE
    $<$<CONFIG:Debug>:_DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
)
if(MSVC)
    target_link_options(AdaptiveDCBlocker PRIVATE /SUBSYSTEM:WINDOWS)
    set_property(TARGET AdaptiveDCBlocker PROPERTY MSVC_RUNTIME_LIBRARY
        "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()
set_target_properties(AdaptiveDCBlocker PROPERTIES PREFIX "" SUFFIX ".sem")
