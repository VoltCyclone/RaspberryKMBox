cmake_minimum_required(VERSION 3.13)

# Bridge Protocol Library
# High-performance binary protocol for bridge ↔ KMBox UART communication

add_library(bridge_protocol INTERFACE)

target_sources(bridge_protocol INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bridge_handler.c
    ${CMAKE_CURRENT_LIST_DIR}/bridge_pio.c
)

target_include_directories(bridge_protocol INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Generate PIO headers for optimized UART
# These will be generated at build time
if(NOT TARGET pioasm)
    # PIO assembler not available, skip PIO optimization
    message(WARNING "PIO assembler not found - bridge protocol will use standard UART")
else()
    pico_generate_pio_header(bridge_protocol ${CMAKE_CURRENT_LIST_DIR}/bridge_uart_tx.pio)
    pico_generate_pio_header(bridge_protocol ${CMAKE_CURRENT_LIST_DIR}/bridge_uart_rx.pio)
endif()

# No dependencies - pure protocol implementation
