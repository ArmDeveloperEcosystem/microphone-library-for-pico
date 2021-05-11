cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(pico_microphone_usb
    main.c
    usb_descriptors.c
)

target_include_directories(pico_microphone_usb PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(pico_microphone_usb PRIVATE tinyusb_device tinyusb_board pico_microphone)

# # enable usb output, disable uart output
# pico_enable_stdio_usb(pico_microphone_hello_pdm 1)
# pico_enable_stdio_uart(pico_microphone_hello_pdm 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_microphone_usb)
