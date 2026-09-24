# Intended for Jeff McClintock's sem_example / SynthEdit code-skeleton setup.
# Put this folder where the parent project already defines BUILD_GMPI_PLUGIN().

BUILD_GMPI_PLUGIN(
    PROJECT_NAME ReverbSpace
    HAS_GUI
    SOURCE_FILES
        ReverbSpaceGui.cpp
        ReverbSpaceGui.h
)
