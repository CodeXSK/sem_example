cmake_minimum_required(VERSION 3.19)
project(ImpulseResponseScopeV1)

build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    HAS_GUI
    SOURCE_FILES
        ImpulseResponseScopeV1.cpp
        ImpulseResponseScopeV1Gui.cpp
)
