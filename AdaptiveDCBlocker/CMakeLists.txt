cmake_minimum_required(VERSION 3.19)
project(AdaptiveDCBlocker)

# Add add_subdirectory(AdaptiveDCBlocker) to the parent sem_example CMakeLists.txt
build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    SOURCE_FILES
        AdaptiveDCBlocker.cpp
)
