# KMBox Bridge - Advanced Autopilot Firmware
# RP2350 bridge with color tracking and Hardware UART communication
# 
# Architecture change: TX/RX wires are now crossed at hardware level,
# allowing direct hardware UART usage instead of PIO-based pin swapping.
# This frees up PIO state machines for timing/latency tracking.
#
# Optional: 1.8" ST7735 TFT display support via SPI1

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
    hw_uart.c
    latency_tracker.c
    tft_display.c
    core1_translator.c
    makcu_translator.c
    ferrum_translator.c
)

# Add kmbox-commands library for shared utilities
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/kmbox-commands kmbox-commands)

# Note: PIO programs removed for UART (now using hardware UART)
# timestamp.pio is available but not currently generated
# pico_generate_pio_header(kmbox_bridge ${CMAKE_CURRENT_LIST_DIR}/timestamp.pio)

target_link_libraries(kmbox_bridge
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_dma
    hardware_uart
    hardware_spi
    hardware_pwm
    pico_multicore
    kmbox_commands
)

# Enable USB CDC output (this is our PC interface)
pico_enable_stdio_usb(kmbox_bridge 1)
pico_enable_stdio_uart(kmbox_bridge 0)

# Generate UF2 file
pico_add_extra_outputs(kmbox_bridge)

# Binary info
pico_set_program_name(kmbox_bridge "KMBox Bridge Autopilot")
pico_set_program_description(kmbox_bridge "Advanced autopilot with TFT display and Hardware UART")
