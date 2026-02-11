# KMBox Bridge - Advanced Autopilot Firmware
# RP2350 bridge with color tracking and Hardware UART communication
# 
# Supports:
#   - Adafruit Feather RP2350 + ST7735 1.8" TFT (128x160)
#   - Adafruit Metro RP2350 + ILI9341 Arduino TFT shield (240x320)

cmake_minimum_required(VERSION 3.13)

# Board selection: metro (default, ILI9341 TFT) or feather (ST7735 TFT)
set(PICO_BOARD adafruit_metro_rp2350 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s)

# Custom board headers (for adafruit_metro_rp2350)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../boards)

# Import Pico SDK
set(PICO_SDK_PATH $ENV{HOME}/.pico-sdk/sdk/2.2.0-fresh)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(kmbox_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Optimization flags for 240MHz Cortex-M33 with hardware FPU
# -ffast-math: Enable VFMA (fused multiply-add) on M33 FPV5 FPU
# -funroll-loops: Unroll tight pixel processing loops in tracker
# -ffunction-sections -fdata-sections: Dead code elimination
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")

# Detect board and configure TFT driver + pins
if(PICO_BOARD STREQUAL "adafruit_metro_rp2350")
    set(TFT_DRIVER_NAME "ILI9341")
    set(BRIDGE_TFT_DRIVER 3)   # TFT_DRIVER_ILI9341
    # Metro Arduino header SPI (ICSP) = SPI1: SCK=30, MOSI=31, MISO=28
    set(BRIDGE_TFT_SPI_DEV "spi1")
    set(BRIDGE_TFT_SCK  30)
    set(BRIDGE_TFT_MOSI 31)
    set(BRIDGE_TFT_MISO 28)
    set(BRIDGE_TFT_CS   10)    # D10
    set(BRIDGE_TFT_DC    9)    # D9
    set(BRIDGE_TFT_RST   8)    # D8
    set(BRIDGE_TFT_BL    3)    # D3 (D7 is MODE_BUTTON)
    # FT6206 capacitive touch over I2C (Arduino A4=SDA, A5=SCL)
    set(BRIDGE_TOUCH_SDA 20)   # I2C0 SDA (Arduino A4)
    set(BRIDGE_TOUCH_SCL 21)   # I2C0 SCL (Arduino A5)
    set(BRIDGE_MODE_BTN  7)    # D7
    message(STATUS "Bridge: Metro RP2350 + ILI9341 240x320 TFT + FT6206 touch")
else()
    set(TFT_DRIVER_NAME "ST7735")
    set(BRIDGE_TFT_DRIVER 1)   # TFT_DRIVER_ST7735
    # Feather SPI0 default pins
    set(BRIDGE_TFT_SPI_DEV "spi0")
    set(BRIDGE_TFT_SCK  22)    # PICO_DEFAULT_SPI_SCK_PIN
    set(BRIDGE_TFT_MOSI 23)    # PICO_DEFAULT_SPI_TX_PIN
    set(BRIDGE_TFT_MISO -1)    # Not used
    set(BRIDGE_TFT_CS    9)
    set(BRIDGE_TFT_DC   10)
    set(BRIDGE_TFT_RST   6)
    set(BRIDGE_TFT_BL    7)
    set(BRIDGE_MODE_BTN  7)    # Shared with BL on Feather
    message(STATUS "Bridge: Feather RP2350 + ST7735 128x160 TFT")
endif()

add_executable(kmbox_bridge
    main.c
    tracker.c
    hw_uart.c
    latency_tracker.c
    core1_translator.c
    makcu_translator.c
    ferrum_translator.c
    protocol_luts.c
    neopixel_dma.c
    tft_display.c
    tft.c
    st7735.c
    ili9341.c
    font.c
    ft6206_touch.c
)

# Generate PIO header for WS2812 NeoPixel from SDK
pico_generate_pio_header(kmbox_bridge ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio)

# Generate PIO header for TFT LUT (if hardware acceleration is enabled)
pico_generate_pio_header(kmbox_bridge ${CMAKE_CURRENT_LIST_DIR}/lut.pio)

# Add kmbox-commands library for shared utilities
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/kmbox-commands kmbox-commands)

target_link_libraries(kmbox_bridge
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_dma
    hardware_uart
    hardware_spi
    hardware_i2c
    hardware_pwm
    hardware_adc
    pico_multicore
    kmbox_commands
)
target_link_options(kmbox_bridge PRIVATE -Wl,--gc-sections)

# Add custom TinyUSB configuration and board-specific TFT pin definitions
target_compile_definitions(kmbox_bridge PRIVATE
    CFG_TUSB_CONFIG_FILE="tusb_config.h"
    USBD_MANUFACTURER="Hurricane"
    USBD_PRODUCT="KMBox\ Bridge\ ${TFT_DRIVER_NAME}"
    USBD_DESC_STR_MAX=32
    BRIDGE_TFT_DRIVER=${BRIDGE_TFT_DRIVER}
    BRIDGE_TFT_SCK_PIN=${BRIDGE_TFT_SCK}
    BRIDGE_TFT_MOSI_PIN=${BRIDGE_TFT_MOSI}
    BRIDGE_TFT_CS_PIN=${BRIDGE_TFT_CS}
    BRIDGE_TFT_DC_PIN=${BRIDGE_TFT_DC}
    BRIDGE_TFT_RST_PIN=${BRIDGE_TFT_RST}
    BRIDGE_TFT_BL_PIN=${BRIDGE_TFT_BL}
    BRIDGE_MODE_BTN_PIN=${BRIDGE_MODE_BTN}
)

# Add FT6206 I2C touch pin definitions for ILI9341 displays
if(DEFINED BRIDGE_TOUCH_SDA)
    target_compile_definitions(kmbox_bridge PRIVATE
        BRIDGE_TOUCH_SDA_PIN=${BRIDGE_TOUCH_SDA}
        BRIDGE_TOUCH_SCL_PIN=${BRIDGE_TOUCH_SCL}
    )
endif()

# Enable USB CDC output (this is our PC interface)
pico_enable_stdio_usb(kmbox_bridge 1)
pico_enable_stdio_uart(kmbox_bridge 0)

# Generate UF2 file
pico_add_extra_outputs(kmbox_bridge)

# Binary info
pico_set_program_name(kmbox_bridge "KMBox Bridge Autopilot")
pico_set_program_description(kmbox_bridge "Advanced autopilot with TFT display and Hardware UART")
