# Build the DualOscOS plugin
add_library(DualOscOS MODULE
    DualOscOS.cpp
    wavetables.h
)

# platform-specific linking
if(APPLE)
    target_link_libraries(DualOscOS "-undefined dynamic_lookup")
endif()