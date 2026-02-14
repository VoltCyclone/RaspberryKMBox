cmake_minimum_required(VERSION 3.13)

# ============================================================================
# pico-ice-sdk pre-init  (must come before pico_sdk_init)
# Sets PICO_BOARD=pico2_ice and adds board header search paths.
# ============================================================================
set(PICO_ICE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../pico-ice-sdk"
    CACHE PATH "Path to pico-ice-sdk checkout (sibling of RaspberryKMBox)")

include(${PICO_ICE_SDK_DIR}/cmake/preinit_pico_ice_sdk.cmake)

# Force pico2_ice board
set(PICO_BOARD pico2_ice CACHE STRING "Target board" FORCE)

# ============================================================================
# Pico SDK
# ============================================================================
set(PICO_SDK_PATH "$ENV{HOME}/.pico-sdk/sdk/2.2.0-fresh"
    CACHE PATH "Path to Raspberry Pi Pico SDK")
include(${CMAKE_CURRENT_SOURCE_DIR}/../pico_sdk_import.cmake)

project(kmbox_fpga_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")

# ============================================================================
# pico-ice-sdk libraries
# ============================================================================
add_subdirectory(${PICO_ICE_SDK_DIR} pico-ice-sdk)

# ============================================================================
# Verilog Synthesis  (iCE40 UP5K bitstream)
# ============================================================================
set(RTL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rtl)
set(RTL_SOURCES
    ${RTL_DIR}/top.v
    ${RTL_DIR}/bridge_engine_spi.v
    ${RTL_DIR}/spi_target.v
    ${RTL_DIR}/spi_master.v
)
set(PCF_FILE ${RTL_DIR}/pico2_ice_bridge.pcf)

# Bitstream outputs (in binary build dir)
set(SYNTH_JSON  ${CMAKE_CURRENT_BINARY_DIR}/bitstream.json)
set(SYNTH_ASC   ${CMAKE_CURRENT_BINARY_DIR}/bitstream.asc)
set(SYNTH_BIN   ${CMAKE_CURRENT_BINARY_DIR}/bitstream.bin)
set(BITSTREAM_H ${CMAKE_CURRENT_BINARY_DIR}/bitstream.h)

add_custom_command(
    OUTPUT  ${SYNTH_JSON}
    DEPENDS ${RTL_SOURCES}
    COMMAND yosys -p "synth_ice40 -abc9 -top top -json ${SYNTH_JSON}" ${RTL_SOURCES}
    COMMENT "[YOSYS] Synthesizing iCE40 bitstream..."
)

add_custom_command(
    OUTPUT  ${SYNTH_ASC}
    DEPENDS ${SYNTH_JSON} ${PCF_FILE}
    COMMAND nextpnr-ice40
        --up5k --package sg48 --freq 48
        --opt-timing
        --seed 33
        --top top
        --pcf ${PCF_FILE}
        --json ${SYNTH_JSON}
        --asc ${SYNTH_ASC}
    COMMENT "[NEXTPNR] Place & route..."
)

add_custom_command(
    OUTPUT  ${SYNTH_BIN}
    DEPENDS ${SYNTH_ASC}
    COMMAND icepack ${SYNTH_ASC} ${SYNTH_BIN}
    COMMENT "[ICEPACK] Generating binary bitstream..."
)

add_custom_command(
    OUTPUT  ${BITSTREAM_H}
    DEPENDS ${SYNTH_BIN}
    COMMAND python3
        ${CMAKE_CURRENT_SOURCE_DIR}/bin2header.py
        ${SYNTH_BIN} > ${BITSTREAM_H}
    COMMENT "[BIN2HDR] Generating bitstream.h..."
)

add_custom_target(bitstream_header DEPENDS ${BITSTREAM_H})

# ============================================================================
# TFT Display (optional — disable with -DDISABLE_TFT=ON)
# ============================================================================
option(DISABLE_TFT "Disable TFT display and touch" ON)

# ============================================================================
# RP2350 Firmware
# ============================================================================
set(CORE_SOURCES
    main.c
    fpga_spi.c
    side_channel.c
    tracker.c
    usb_descriptors.c
)

if(NOT DISABLE_TFT)
    set(TFT_SOURCES
        tft.c
        tft_display.c
        ili9341.c
        font.c
        ft6206_touch.c
    )
else()
    set(TFT_SOURCES "")
endif()

add_executable(${CMAKE_PROJECT_NAME}
    ${CORE_SOURCES}
    ${TFT_SOURCES}
)

# Bitstream header must be built before the firmware
add_dependencies(${CMAKE_PROJECT_NAME} bitstream_header)

# Generate PIO header from fpga_spi.pio
pico_generate_pio_header(${CMAKE_PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/fpga_spi.pio
)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..     # for bridge_protocol.h
    ${CMAKE_CURRENT_BINARY_DIR}      # for bitstream.h
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    pico_ice_sdk
    pico_ice_usb
    hardware_spi
    hardware_pio
    hardware_uart
    hardware_adc
    hardware_dma
)

if(NOT DISABLE_TFT)
    target_link_libraries(${CMAKE_PROJECT_NAME}
        hardware_pwm
        hardware_i2c
    )
endif()

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--gc-sections)

if(DISABLE_TFT)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE DISABLE_TFT=1)
else()
    # TFT compile definitions (pin config)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        BRIDGE_TFT_DRIVER=3
        BRIDGE_TFT_SCK_PIN=38
        BRIDGE_TFT_MOSI_PIN=35
        BRIDGE_TFT_MISO_PIN=36
        BRIDGE_TFT_CS_PIN=37
        BRIDGE_TFT_DC_PIN=34
        BRIDGE_TFT_RST_PIN=39
        BRIDGE_TFT_BL_PIN=33
        BRIDGE_TOUCH_SDA_PIN=26
        BRIDGE_TOUCH_SCL_PIN=27
    )
endif()

# Disable pico-sdk stdio (we use pico_ice_usb CDC)
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 0)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

# Generate UF2 output
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
