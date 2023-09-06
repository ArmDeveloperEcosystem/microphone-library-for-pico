cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(hello_i2s_microphone
    main.c
    usb_descriptors.c
    usb_microphone.c
)

target_include_directories(hello_i2s_microphone PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(hello_i2s_microphone PRIVATE tinyusb_device tinyusb_board pico_i2s_microphone)

# enable usb output, disable uart output
# pico_enable_stdio_usb(hello_i2s_microphone 1)
pico_enable_stdio_uart(hello_i2s_microphone 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello_i2s_microphone)
