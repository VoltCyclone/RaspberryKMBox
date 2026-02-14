# KMBox Bridge Firmware
# Supports two board configurations:
#   - Adafruit Metro RP2350: Full build with TFT, tracker, Makcu
#   - Adafruit Feather RP2350: Minimal latency bridge (CDC -> Wire Protocol -> KMBox)

cmake_minimum_required(VERSION 3.13)

# Board selection: metro (default) or feather
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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")

# ============================================================================
# Board-specific configuration
# ============================================================================

if(PICO_BOARD STREQUAL "adafruit_feather_rp2350")
    # Feather: minimal bridge, no TFT/tracker/Makcu
    set(BRIDGE_IS_FEATHER TRUE)
    set(ENABLE_TFT_DISPLAY 0)
    set(ENABLE_COLOR_TRACKER 0)
    set(ENABLE_MAKCU_PROTOCOL 0)
    message(STATUS "Bridge: Feather RP2350 — minimal latency (Wire Protocol v2)")
elseif(PICO_BOARD STREQUAL "adafruit_metro_rp2350")
    # Metro: full build with TFT + tracker + Makcu
    set(BRIDGE_IS_FEATHER FALSE)
    set(ENABLE_TFT_DISPLAY 1)
    set(ENABLE_COLOR_TRACKER 1)
    set(ENABLE_MAKCU_PROTOCOL 1)
    set(TFT_DRIVER_NAME "ILI9341")
    set(BRIDGE_TFT_DRIVER 3)   # TFT_DRIVER_ILI9341
    set(BRIDGE_TFT_SPI_DEV "spi1")
    set(BRIDGE_TFT_SCK  30)
    set(BRIDGE_TFT_MOSI 31)
    set(BRIDGE_TFT_MISO 28)
    set(BRIDGE_TFT_CS   10)
    set(BRIDGE_TFT_DC    9)
    set(BRIDGE_TFT_RST   8)
    set(BRIDGE_TFT_BL    3)
    set(BRIDGE_TOUCH_SDA 20)
    set(BRIDGE_TOUCH_SCL 21)
    set(BRIDGE_MODE_BTN  7)
    message(STATUS "Bridge: Metro RP2350 + ILI9341 240x320 TFT + FT6206 touch")
else()
    # Other boards: default to minimal (no TFT)
    set(BRIDGE_IS_FEATHER FALSE)
    set(ENABLE_TFT_DISPLAY 0)
    set(ENABLE_COLOR_TRACKER 0)
    set(ENABLE_MAKCU_PROTOCOL 0)
    message(STATUS "Bridge: ${PICO_BOARD} — minimal (no TFT)")
endif()

# ============================================================================
# Source files
# ============================================================================

# Core sources shared by all configurations
set(BRIDGE_SOURCES
    hw_uart.c
    latency_tracker.c
    ferrum_translator.c
    neopixel_dma.c
)

if(BRIDGE_IS_FEATHER)
    # Feather: clean minimal main
    list(APPEND BRIDGE_SOURCES main_feather.c)
else()
    # Metro/other: full main with compile-gated features
    list(APPEND BRIDGE_SOURCES
        main.c
        protocol_luts.c
    )
endif()

# TFT display sources (Metro only)
if(ENABLE_TFT_DISPLAY)
    list(APPEND BRIDGE_SOURCES
        tft_display.c
        tft.c
        st7735.c
        ili9341.c
        font.c
        ft6206_touch.c
    )
endif()

# Color tracker (Metro only)
if(ENABLE_COLOR_TRACKER)
    list(APPEND BRIDGE_SOURCES tracker.c)
endif()

# Makcu protocol (Metro only)
if(ENABLE_MAKCU_PROTOCOL)
    list(APPEND BRIDGE_SOURCES
        core1_translator.c
        makcu_translator.c
    )
endif()

add_executable(kmbox_bridge ${BRIDGE_SOURCES})

# Generate PIO header for WS2812 NeoPixel from SDK
pico_generate_pio_header(kmbox_bridge ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio)

# PIO header for TFT LUT (Metro only, but always generate if file exists)
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lut.pio)
    pico_generate_pio_header(kmbox_bridge ${CMAKE_CURRENT_LIST_DIR}/lut.pio)
endif()

# Add shared libraries
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/kmbox-commands kmbox-commands)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/wire-protocol wire-protocol)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/peri-clock peri-clock)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/dma-uart dma-uart)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/led-utils led-utils)

# ============================================================================
# Libraries
# ============================================================================

# Core libraries (all configurations)
set(BRIDGE_LIBS
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_dma
    hardware_uart
    hardware_adc
    kmbox_commands
    wire_protocol
    peri_clock
    dma_uart
    led_utils
)

# TFT needs SPI, I2C, PWM, multicore
if(ENABLE_TFT_DISPLAY)
    list(APPEND BRIDGE_LIBS
        hardware_spi
        hardware_i2c
        hardware_pwm
    )
endif()

# Makcu needs multicore
if(ENABLE_MAKCU_PROTOCOL)
    list(APPEND BRIDGE_LIBS pico_multicore)
endif()

target_link_libraries(kmbox_bridge ${BRIDGE_LIBS})
target_link_options(kmbox_bridge PRIVATE -Wl,--gc-sections)

# ============================================================================
# Compile definitions
# ============================================================================

target_compile_definitions(kmbox_bridge PRIVATE
    CFG_TUSB_CONFIG_FILE="tusb_config.h"
    ENABLE_TFT_DISPLAY=${ENABLE_TFT_DISPLAY}
    ENABLE_COLOR_TRACKER=${ENABLE_COLOR_TRACKER}
    ENABLE_MAKCU_PROTOCOL=${ENABLE_MAKCU_PROTOCOL}
)

if(BRIDGE_IS_FEATHER)
    target_compile_definitions(kmbox_bridge PRIVATE
        USBD_MANUFACTURER="Hurricane"
        USBD_PRODUCT="KMBox\ Feather\ Bridge"
        USBD_DESC_STR_MAX=32
    )
else()
    target_compile_definitions(kmbox_bridge PRIVATE
        USBD_MANUFACTURER="Hurricane"
        USBD_PRODUCT="KMBox\ Bridge\ ${TFT_DRIVER_NAME}"
        USBD_DESC_STR_MAX=32
    )
endif()

# TFT pin definitions (Metro only)
if(ENABLE_TFT_DISPLAY)
    target_compile_definitions(kmbox_bridge PRIVATE
        BRIDGE_TFT_DRIVER=${BRIDGE_TFT_DRIVER}
        BRIDGE_TFT_SCK_PIN=${BRIDGE_TFT_SCK}
        BRIDGE_TFT_MOSI_PIN=${BRIDGE_TFT_MOSI}
        BRIDGE_TFT_CS_PIN=${BRIDGE_TFT_CS}
        BRIDGE_TFT_DC_PIN=${BRIDGE_TFT_DC}
        BRIDGE_TFT_RST_PIN=${BRIDGE_TFT_RST}
        BRIDGE_TFT_BL_PIN=${BRIDGE_TFT_BL}
        BRIDGE_MODE_BTN_PIN=${BRIDGE_MODE_BTN}
    )
    if(DEFINED BRIDGE_TOUCH_SDA)
        target_compile_definitions(kmbox_bridge PRIVATE
            BRIDGE_TOUCH_SDA_PIN=${BRIDGE_TOUCH_SDA}
            BRIDGE_TOUCH_SCL_PIN=${BRIDGE_TOUCH_SCL}
        )
    endif()
endif()

# Enable USB CDC output (this is our PC interface)
pico_enable_stdio_usb(kmbox_bridge 1)
pico_enable_stdio_uart(kmbox_bridge 0)

# Generate UF2 file
pico_add_extra_outputs(kmbox_bridge)

# Binary info
if(BRIDGE_IS_FEATHER)
    pico_set_program_name(kmbox_bridge "KMBox Feather Bridge")
    pico_set_program_description(kmbox_bridge "Minimal latency bridge with Wire Protocol v2")
else()
    pico_set_program_name(kmbox_bridge "KMBox Bridge Autopilot")
    pico_set_program_description(kmbox_bridge "Advanced autopilot with TFT display and Hardware UART")
endif()
