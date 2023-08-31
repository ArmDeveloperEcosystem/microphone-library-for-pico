cmake_minimum_required(VERSION 3.12)

# initialize pico_sdk from GIT
# (note this can come from environment, CMake cache etc)
set(ENV{PICO_SDK_PATH} "/home/ttb/raspberry-pi-pico/pico/pico-sdk")

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_microphone)

# initialize the Pico SDK
pico_sdk_init()

add_library(pico_pdm_microphone INTERFACE)
target_sources(pico_pdm_microphone INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/pdm_microphone.c
    ${CMAKE_CURRENT_LIST_DIR}/src/OpenPDM2PCM/OpenPDMFilter.c
)
target_include_directories(pico_pdm_microphone INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)
pico_generate_pio_header(pico_pdm_microphone ${CMAKE_CURRENT_LIST_DIR}/src/pdm_microphone.pio)
target_link_libraries(pico_pdm_microphone INTERFACE pico_stdlib hardware_dma hardware_pio)


add_library(pico_i2s_microphone INTERFACE)
target_sources(pico_i2s_microphone INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/i2s_microphone.c
)
target_include_directories(pico_i2s_microphone INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
    examples/usb_microphone
)
target_link_libraries(pico_i2s_microphone INTERFACE pico_stdlib hardware_dma hardware_pio)


add_library(pico_analog_microphone INTERFACE)
target_sources(pico_analog_microphone INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/analog_microphone.c
)
target_include_directories(pico_analog_microphone INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)
target_link_libraries(pico_analog_microphone INTERFACE pico_stdlib hardware_adc hardware_dma)

add_subdirectory("examples/hello_pdm_microphone")
add_subdirectory("examples/hello_analog_microphone")
add_subdirectory("examples/usb_microphone")
add_subdirectory("examples/hello_i2s_microphone")
