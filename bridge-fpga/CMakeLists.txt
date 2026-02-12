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
    ${RTL_DIR}/bridge_engine.v
    ${RTL_DIR}/spi_target.v
    ${RTL_DIR}/uart_tx.v
    ${RTL_DIR}/uart_rx.v
    ${RTL_DIR}/sync_fifo.v
)
set(PCF_FILE ${RTL_DIR}/pico2_ice_bridge.pcf)
set(REGS_HDR ${RTL_DIR}/bridge_regs.vh)

# Bitstream outputs (in binary build dir)
set(SYNTH_JSON  ${CMAKE_CURRENT_BINARY_DIR}/bitstream.json)
set(SYNTH_ASC   ${CMAKE_CURRENT_BINARY_DIR}/bitstream.asc)
set(SYNTH_BIN   ${CMAKE_CURRENT_BINARY_DIR}/bitstream.bin)
set(BITSTREAM_H ${CMAKE_CURRENT_BINARY_DIR}/bitstream.h)

add_custom_command(
    OUTPUT  ${SYNTH_JSON}
    DEPENDS ${RTL_SOURCES} ${REGS_HDR}
    COMMAND yosys -p "synth_ice40 -abc9 -top top -json ${SYNTH_JSON}" ${RTL_SOURCES}
    COMMENT "[YOSYS] Synthesizing iCE40 bitstream..."
)

add_custom_command(
    OUTPUT  ${SYNTH_ASC}
    DEPENDS ${SYNTH_JSON} ${PCF_FILE}
    COMMAND nextpnr-ice40
        --up5k --package sg48 --freq 48
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
# RP2350 Firmware
# ============================================================================
add_executable(${CMAKE_PROJECT_NAME}
    main.c
    fpga_spi.c
    tracker.c
    usb_descriptors.c
)

# Bitstream header must be built before the firmware
add_dependencies(${CMAKE_PROJECT_NAME} bitstream_header)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}   # for bitstream.h
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    pico_ice_sdk
    pico_ice_usb
    hardware_spi
    hardware_adc
)

# Disable pico-sdk stdio (we use pico_ice_usb CDC)
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 0)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

# Generate UF2 output
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
