# KMBox Commands Library CMakeLists.txt

add_library(kmbox_commands STATIC
    kmbox_commands.c
)

target_include_directories(kmbox_commands PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with pico_stdlib for time functions
target_link_libraries(kmbox_commands
    pico_stdlib
    hid_defs
)