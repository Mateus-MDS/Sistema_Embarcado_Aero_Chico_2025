# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(main C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories( ${CMAKE_SOURCE_DIR}/Lib) # Inclui os files .h na pasta lib

# Add executable. Default name is the project name, version 0.1k
add_executable(main main.c Lib/GPS_neo_6.c)

pico_set_program_name(main "main")
pico_set_program_version(main "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(main 0)
pico_enable_stdio_usb(main 1)

# Add the standard library to the build
target_link_libraries(main
        pico_stdlib
        )

# Add the standard include files to the build
target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(main PRIVATE
    RUNNING_AS_main=1
)

# Add any user requested libraries
target_link_libraries(main 
        
        )

pico_add_extra_outputs(main)

# add url via pico_set_program_url
#example_auto_set_url(main)
