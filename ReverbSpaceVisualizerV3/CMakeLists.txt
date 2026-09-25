cmake_minimum_required(VERSION 3.19)

project(ReverbSpaceVisualizerV3)

# Deliberately build BOTH processor and GUI. This guarantees a normal DSP
# factory (same mechanism as the working FDN example) plus the GUI factory.
build_gmpi_plugin(
    PROJECT_NAME ${PROJECT_NAME}
    HAS_DSP
    HAS_GUI
    SOURCE_FILES
        ReverbSpaceVisualizerV3.cpp
        ReverbSpaceVisualizerV3Gui.cpp
)
