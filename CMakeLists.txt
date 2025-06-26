# PIOKMbox - Raspberry Pi Pico USB HID Project
# CMake configuration file

cmake_minimum_required(VERSION 3.13)

# ==== Build Configuration ====
# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type: Debug or Release" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Set compiler standards and options
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add optimization flags for Release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2)
    add_compile_definitions(NDEBUG)
endif()

# ==== SDK Configuration ====
# Initialise pico_sdk from installed location
# DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ==== Board Configuration ====
# Define supported boards
set(SUPPORTED_BOARDS "pico;adafruit_feather_rp2040_usb_host;pico2;adafruit_feather_rp2350")

# Set board type based on TARGET_BOARD variable or default to pico (RP2040)
if(NOT DEFINED TARGET_BOARD)
    set(TARGET_BOARD "pico" CACHE STRING "Target board type (${SUPPORTED_BOARDS})")
endif()

# Validate board type
if(TARGET_BOARD STREQUAL "adafruit_feather_rp2350" OR TARGET_BOARD STREQUAL "pico2")
    set(PICO_BOARD pico2 CACHE STRING "Board type")
    set(TARGET_CHIP "RP2350")
    message(STATUS "Building for ${TARGET_CHIP} (Pico 2)")
elseif(TARGET_BOARD STREQUAL "adafruit_feather_rp2040_usb_host" OR TARGET_BOARD STREQUAL "pico")
    set(PICO_BOARD pico CACHE STRING "Board type")
    set(TARGET_CHIP "RP2040")
    message(STATUS "Building for ${TARGET_CHIP} (Pico)")
else()
    message(FATAL_ERROR "Unsupported board type: ${TARGET_BOARD}. Supported: ${SUPPORTED_BOARDS}")
endif()

# ==== SDK Initialization ====
# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Initialize project
project(PIOKMbox C CXX ASM)

# Check if SDK is available
if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Pico SDK path not found. Please set PICO_SDK_PATH.")
endif()

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ==== Project Configuration ====
# Project metadata
set(PROJECT_VERSION "0.1")

# ==== Source Files ====
# Define source files
set(PROJECT_SOURCES
    PIOKMbox.c
    led_control.c
    usb_hid.c
    usb_hid_init.c
    usb_hid_device.c
    usb_hid_host.c
    usb_hid_reports.c
    usb_hid_descriptors.c
    usb_hid_strings.c
    usb_hid_callbacks.c
    watchdog.c
    init_state_machine.c
    state_management.c
    dma_handlers.c
    dma_manager.c
    usb_locks.c
)

# Add RP2350-specific source files
if(TARGET_CHIP STREQUAL "RP2350")
    list(APPEND PROJECT_SOURCES
        rp2350_hw_accel.c
        rp2350_tuh_task.c
        rp2350_dma_handler.c
    )
    message(STATUS "Added RP2350 hardware acceleration source files")
endif()

# Add executable
add_executable(PIOKMbox ${PROJECT_SOURCES})

# Now set program properties after the target is defined
pico_set_program_name(PIOKMbox "PIOKMbox")
pico_set_program_version(PIOKMbox "${PROJECT_VERSION}")

# ==== I/O Configuration ====
# Configure stdio output (UART/USB)
option(ENABLE_UART_OUTPUT "Enable UART output" ON)
option(ENABLE_USB_OUTPUT "Enable USB output" OFF)
pico_enable_stdio_uart(PIOKMbox ${ENABLE_UART_OUTPUT})
pico_enable_stdio_usb(PIOKMbox ${ENABLE_USB_OUTPUT})

# Generate PIO header file
pico_generate_pio_header(PIOKMbox ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

# ==== Include Directories ====
target_include_directories(PIOKMbox PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# ==== PIO USB Configuration ====
# Add PIO USB library
add_subdirectory(lib/pio_usb)
message(STATUS "PIO USB library support enabled")

# Add PIO USB HCD implementation directly
target_sources(PIOKMbox PRIVATE
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
)

# ==== Compile Definitions ====
# Common definitions for all boards
set(COMMON_DEFINITIONS
    CFG_TUSB_CONFIG_FILE="tusb_config.h"
    BOARD_TUH_RHPORT=1
    BOARD_TUD_RHPORT=0
    PIO_USB_USE_TINYUSB
    BUILD_CONFIG=BUILD_CONFIG_DEVELOPMENT
    PIO_USB_AVAILABLE=1
    CFG_TUH_RPI_PIO_USB=1
    USE_DMA_MANAGER=1
)


# Apply common definitions
target_compile_definitions(PIOKMbox PRIVATE ${COMMON_DEFINITIONS})

# Add board-specific definitions
if(TARGET_CHIP STREQUAL "RP2350")
    target_compile_definitions(PIOKMbox PRIVATE
        TARGET_RP2350=1
        PICO_RP2350=1
        PIN_USB_HOST_DP=10
        PIN_USB_HOST_DM=11
        USE_HARDWARE_ACCELERATION=1
        RP2350=1
    )
    message(STATUS "Added RP2350-specific definitions")
else()
    target_compile_definitions(PIOKMbox PRIVATE
        TARGET_RP2040=1
        PICO_RP2040=1
    )
    message(STATUS "Added RP2040-specific definitions")
endif()

# ==== Libraries ====
# Define all required libraries
set(PROJECT_LIBRARIES
    pico_stdlib
    tinyusb_device
    tinyusb_host
    tinyusb_board
    hardware_pio
    hardware_dma
    hardware_watchdog
    pico_unique_id
    pico_multicore
    pico_pio_usb
    m
)

# Link libraries
target_link_libraries(PIOKMbox ${PROJECT_LIBRARIES})

# ==== Build Outputs ====
# Generate additional output files (.uf2, .hex, etc.)
pico_add_extra_outputs(PIOKMbox)
