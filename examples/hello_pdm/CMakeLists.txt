cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(pico_microphone_hello_pdm
    main.c
)

target_link_libraries(pico_microphone_hello_pdm pico_pdm_microphone)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_microphone_hello_pdm 1)
pico_enable_stdio_uart(pico_microphone_hello_pdm 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_microphone_hello_pdm)
