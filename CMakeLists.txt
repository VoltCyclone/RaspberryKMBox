# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(PICO_SDK_PATH $ENV{HOME}/.pico-sdk/sdk/2.2.0-fresh)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Board selection: metro (Adafruit Metro RP2350) or pico2 (RP2350 only)
set(PICO_BOARD adafruit_metro_rp2350 CACHE STRING "Board type")

# Tell the SDK where to find custom board headers (e.g. adafruit_metro_rp2350.h)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)

# Detect board type and set appropriate pin configurations
if(PICO_BOARD STREQUAL "adafruit_metro_rp2350")
    set(USB_HOST_DP_PIN 32 CACHE STRING "USB Host D+ pin")
    set(USB_HOST_DM_PIN 33 CACHE STRING "USB Host D- pin")
    set(USB_HOST_5V_PIN 29 CACHE STRING "USB Host 5V power pin")
    set(LED_PIN 23 CACHE STRING "Status LED pin")
    set(NEOPIXEL_PIN 25 CACHE STRING "NeoPixel data pin")
    set(NEOPIXEL_POWER_PIN 255 CACHE STRING "NeoPixel power pin (255=not used)")
    message(STATUS "Configuring for Adafruit Metro RP2350 - USB Host on GPIO32/33")

    # Check for "broken metro" pin remap (D0-D7 unavailable)
    # Available pins: D8-D11, D22-D23 (D12-D19 are HSTX) + A0-A5 (GPIO 41-46)
    # No UART0 needed — commands arrive via SPI from FPGA bridge, not serial.
    # UART0 defaults to GPIO 0/1 (broken/disconnected) which is harmless.
    if(METRO_BROKEN_PINS)
        # SPI slave (FPGA): D10/D11 for MOSI/SCK, D22/D23 for CS/MISO
        set(KM_SPI_MOSI 10 CACHE STRING "SPI MOSI pin (D10)")
        set(KM_SPI_SCK 11 CACHE STRING "SPI SCK pin (D11, must be MOSI+1)")
        set(KM_SPI_CS 22 CACHE STRING "SPI CS pin (D22)")
        set(KM_SPI_MISO 23 CACHE STRING "SPI MISO pin (D23)")
        # Side channel UART1: GPIO 8/9 (same as default)
        set(SC_UART_TX 8 CACHE STRING "Side channel UART1 TX pin (D8)")
        set(SC_UART_RX 9 CACHE STRING "Side channel UART1 RX pin (D9)")
        # Button: on-board, GPIO 7
        set(BUTTON_PIN 7 CACHE STRING "Button pin (on-board)")
        # USB Host stays on GPIO 32/33 (separate pads on Metro)
        message(STATUS "  BROKEN-METRO mode: D8-D11, D22-D23")
        message(STATUS "  SPI = GPIO 10-11/22-23, Side channel = GPIO 8/9, Button = GPIO 7")
    endif()

elseif(PICO_BOARD STREQUAL "seeed_xiao_rp2350")
    # XIAO RP2350: front header only (no back pads)
    # Available GPIOs: 0,1,2,3,4,5,6,7,26,27,28 (front header)
    # Internal: 22 (RGB), 23 (RGB pwr), 25 (user LED)
    # NOTE: GPIO 8 not exposed - UART1 TX remapped to GPIO 4
    #       GPIO 18 not exposed - USB 5V remapped to GPIO 6
    set(USB_HOST_DP_PIN 16 CACHE STRING "USB Host D+ pin")
    set(USB_HOST_DM_PIN 17 CACHE STRING "USB Host D- pin")
    set(USB_HOST_5V_PIN 6 CACHE STRING "USB Host 5V power pin (D4, front header)")
    set(LED_PIN 25 CACHE STRING "Status LED pin (internal user LED)")
    set(NEOPIXEL_PIN 22 CACHE STRING "NeoPixel data pin (internal RGB LED)")
    set(NEOPIXEL_POWER_PIN 23 CACHE STRING "NeoPixel power pin (internal RGB power)")
    # Side channel UART1: GPIO 4 TX (UART1-capable), GPIO 9 RX (back pad needed!)
    # Without back pads, side channel is NOT available on XIAO front-header-only
    message(STATUS "Configuring for Seeed XIAO RP2350 - USB Host on GPIO16/17 (back pads)")
    message(WARNING "XIAO RP2350 front-header-only: USB Host (GPIO 16/17) and")
    message(WARNING "  side channel UART1 (GPIO 8/9) require back pad soldering!")
    message(WARNING "  Consider using a board with more exposed pins.")

else()
    # Default: Pico/Pico2 with common GPIO16/17 for PIO USB
    set(USB_HOST_DP_PIN 16 CACHE STRING "USB Host D+ pin")
    set(USB_HOST_DM_PIN 17 CACHE STRING "USB Host D- pin")
    set(USB_HOST_5V_PIN 18 CACHE STRING "USB Host 5V power pin")
    set(LED_PIN 25 CACHE STRING "Status LED pin")
    set(NEOPIXEL_PIN 21 CACHE STRING "NeoPixel data pin")
    set(NEOPIXEL_POWER_PIN 20 CACHE STRING "NeoPixel power pin")
    message(STATUS "Configuring for ${PICO_BOARD} - USB Host on GPIO16/17")
endif()

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PIOKMbox C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(PIOKMbox
    PIOKMbox.c
    led_control.c
    usb_hid.c
    watchdog.c
    state_management.c
    kmbox_serial_handler.c
    smooth_injection.c
    humanization_fpu.c
    dcp_helpers.S
    spi_slave.c
    side_channel.c
)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(PIOKMbox ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
pico_generate_pio_header(PIOKMbox ${CMAKE_CURRENT_LIST_DIR}/spi_slave.pio)

pico_set_program_name(PIOKMbox "PIOKMbox")
pico_set_program_version(PIOKMbox "0.1")

# Modify the below lines to enable/disable output over UART/USB
# UART0 is used for KMBox serial input via RP2350 USB Bridge
# USB CDC is disabled to avoid breaking HID enumeration
# Debug output can be sent through the KMBox interface if needed
pico_enable_stdio_uart(PIOKMbox 1)
pico_enable_stdio_usb(PIOKMbox 0)

# Optimization flags for release performance at 240MHz
# -O3: Aggressive inlining and vectorization
# -ffast-math: Enable FPU VFMA (fused multiply-add) instructions on Cortex-M33
# -funroll-loops: Unroll tight inner loops (helps DMA/UART processing)
# -ffunction-sections -fdata-sections: Enable dead code elimination with --gc-sections
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")
target_link_options(PIOKMbox PRIVATE -Wl,--gc-sections)

# Add the standard library to the build
target_link_libraries(PIOKMbox
        pico_stdlib
        tinyusb_device
        tinyusb_host
        tinyusb_board
        hardware_pio
        hardware_dma
        hardware_watchdog
        hardware_uart
        hardware_spi
        hardware_irq
        hardware_adc
        pico_unique_id
        pico_multicore
        hardware_dcp
        pico_flash
        pico_rand
        pico_time
        m)

# Add PIO USB library
add_subdirectory(lib/Pico-PIO-USB)

# Add KMBox Commands library
add_subdirectory(lib/kmbox-commands)

# PIO USB library support (now enabled)
message(STATUS "PIO USB library support enabled")
# Add compile definitions for TinyUSB and pin configurations
target_compile_definitions(PIOKMbox PRIVATE
        CFG_TUSB_CONFIG_FILE="tusb_config.h"
        BOARD_TUH_RHPORT=1
        BOARD_TUD_RHPORT=0
        PIO_USB_USE_TINYUSB
        BUILD_CONFIG=BUILD_CONFIG_DEVELOPMENT
        PIO_USB_AVAILABLE=1
        CFG_TUH_RPI_PIO_USB=1
        CPU_FREQ=240000
        PIN_USB_HOST_DP=${USB_HOST_DP_PIN}
        PIN_USB_HOST_DM=${USB_HOST_DM_PIN}
        PIN_USB_5V=${USB_HOST_5V_PIN}
        PIN_LED=${LED_PIN}
        PIN_NEOPIXEL=${NEOPIXEL_PIN}
        NEOPIXEL_POWER=${NEOPIXEL_POWER_PIN}
)

# Apply pin remapping compile definitions for broken Metro or XIAO boards
if(METRO_BROKEN_PINS)
    target_compile_definitions(PIOKMbox PRIVATE
        KM_SPI_MOSI_PIN=${KM_SPI_MOSI}
        KM_SPI_SCK_PIN=${KM_SPI_SCK}
        KM_SPI_CS_PIN=${KM_SPI_CS}
        KM_SPI_MISO_PIN=${KM_SPI_MISO}
        SC_UART_TX_PIN=${SC_UART_TX}
        SC_UART_RX_PIN=${SC_UART_RX}
        PIN_BUTTON=${BUTTON_PIN}
    )
endif()

# Link PIO USB library
target_link_libraries(PIOKMbox pico_pio_usb)

# Link KMBox Commands library
target_link_libraries(PIOKMbox kmbox_commands)

# Add PIO USB HCD implementation directly
target_sources(PIOKMbox PRIVATE
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
 )

# Add the standard include files to the build
target_include_directories(PIOKMbox PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

pico_add_extra_outputs(PIOKMbox)
