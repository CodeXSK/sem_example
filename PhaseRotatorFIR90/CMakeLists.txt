cmake_minimum_required(VERSION 3.19)

project(PhaseRotatorFIR90)

build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    SOURCE_FILES
        PhaseRotatorFIR90.cpp
)
