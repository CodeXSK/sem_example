cmake_minimum_required(VERSION 3.19)
project(AdaptiveDCBlocker)

# Copy this folder to your existing JeffMcClintock/sem_example fork,
# then add: add_subdirectory(AdaptiveDCBlocker)
# in the ROOT CMakeLists.txt of that repository.
build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    SOURCE_FILES
        AdaptiveDCBlocker.cpp
        AdaptiveDCBlockerDSP.h
)
