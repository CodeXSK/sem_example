cmake_minimum_required(VERSION 3.19)

project(StrongestHarmonicEQ)

build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    SOURCE_FILES
        StrongestHarmonicEQ.cpp
        StrongestPeakEngine.h
)

# Jeff's current sem_example sets /fp:fast /GS- globally on MSVC.
# This module explicitly contains NaN/Inf safety handling, therefore use
# predictable IEEE special-value behavior and restore stack protection.
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /fp:precise /GS)
endif()

# Defensive override in case this folder is reused under a parent project that
# enables fast-math for Clang/GCC.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(${PROJECT_NAME} PRIVATE -fno-fast-math)
endif()
