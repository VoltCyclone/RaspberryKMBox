# KMBox Bridge - Advanced Autopilot Firmware
# RP2350 bridge with color tracking and PIO UART communication

cmake_minimum_required(VERSION 3.13)

# Set board type for Adafruit Feather RP2350
set(PICO_BOARD adafruit_feather_rp2350)
set(PICO_PLATFORM rp2350-arm-s)

# Import Pico SDK
set(PICO_SDK_PATH $ENV{HOME}/.pico-sdk/sdk/2.2.0-fresh)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(kmbox_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(kmbox_bridge
    main.c
    tracker.c
)

# Generate PIO headers
pico_generate_pio_header(kmbox_bridge ${CMAKE_CURRENT_LIST_DIR}/uart_tx.pio)
pico_generate_pio_header(kmbox_bridge ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio)

target_link_libraries(kmbox_bridge
    pico_stdlib
    hardware_pio
    hardware_clocks
)

# Enable USB CDC output (this is our PC interface)
pico_enable_stdio_usb(kmbox_bridge 1)
pico_enable_stdio_uart(kmbox_bridge 0)

# Generate UF2 file
pico_add_extra_outputs(kmbox_bridge)

# Binary info
pico_set_program_name(kmbox_bridge "KMBox Bridge Autopilot")
pico_set_program_description(kmbox_bridge "Advanced autopilot with color tracking and PIO UART")
