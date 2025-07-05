# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Initialise the Pico SDK
pico_sdk_init()

# Add library cpp files
add_library(LiquidCrystal_I2C INTERFACE)
target_sources(LiquidCrystal_I2C INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/LiquidCrystal_I2C.cpp
)

# Add include directory
target_include_directories(LiquidCrystal_I2C INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
target_link_libraries(LiquidCrystal_I2C INTERFACE pico_stdlib hardware_pio)