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
set(PICO_BOARD adafruit_feather_rp2040_usb_host CACHE STRING "Board type")

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
    humanization_lut.c
)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(PIOKMbox ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

pico_set_program_name(PIOKMbox "PIOKMbox")
pico_set_program_version(PIOKMbox "0.1")

# Modify the below lines to enable/disable output over UART/USB
# UART0 is used for KMBox serial input via RP2350 USB Bridge
# USB CDC is disabled to avoid breaking HID enumeration
# Debug output can be sent through the KMBox interface if needed
pico_enable_stdio_uart(PIOKMbox 1)
pico_enable_stdio_usb(PIOKMbox 0)

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
        PIN_USB_HOST_DP=16
        PIN_USB_HOST_DM=17
)

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
