/**
 * Makcu API Binary Protocol Definitions
 * 
 * This header defines the binary protocol format used by Makcu devices.
 * The bridge can receive Makcu commands and translate them to KMBox protocol.
 * 
 * Reference: https://www.makcu.com/en/api#binary-protocol-format
 * 
 * Frame Format:
 * RX (HOST → DEVICE): [0x50] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...]
 * TX (DEVICE → HOST): [0x50] [CMD] [LEN_LO] [LEN_HI] [status:u8 or PAYLOAD]
 */

#ifndef MAKCU_PROTOCOL_H
#define MAKCU_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// Frame constants
#define MAKCU_FRAME_START       0x50
#define MAKCU_STATUS_OK         0x00
#define MAKCU_STATUS_ERR        0x01

// Maximum payload size
#define MAKCU_MAX_PAYLOAD       256

// Mouse Commands (0x01-0x19)
#define MAKCU_CMD_AXIS_STREAM       0x01  // Stream x/y/wheel axis
#define MAKCU_CMD_BUTTONS_STREAM    0x02  // Stream button states
#define MAKCU_CMD_CATCH             0x03  // Catch mode
#define MAKCU_CMD_CLICK             0x04  // Schedule clicks
#define MAKCU_CMD_GETPOS            0x05  // Get mouse position
#define MAKCU_CMD_INVERT_X          0x06  // Invert X axis
#define MAKCU_CMD_INVERT_Y          0x07  // Invert Y axis
#define MAKCU_CMD_LEFT_BUTTON       0x08  // Left button control
#define MAKCU_CMD_LOCK              0x09  // Lock button/axis
#define MAKCU_CMD_MIDDLE_BUTTON     0x0A  // Middle button control
#define MAKCU_CMD_MO                0x0B  // Raw mouse frame
#define MAKCU_CMD_MOUSE_STREAM      0x0C  // Stream full mouse data
#define MAKCU_CMD_MOVE              0x0D  // Relative move with bezier
#define MAKCU_CMD_MOVETO            0x0E  // Absolute move with bezier
#define MAKCU_CMD_PAN               0x0F  // Horizontal scroll
#define MAKCU_CMD_REMAP_BUTTON      0x10  // Remap mouse buttons
#define MAKCU_CMD_RIGHT_BUTTON      0x11  // Right button control
#define MAKCU_CMD_SIDE1_BUTTON      0x12  // Side1 button control
#define MAKCU_CMD_SIDE2_BUTTON      0x13  // Side2 button control
#define MAKCU_CMD_SILENT            0x14  // Silent click
#define MAKCU_CMD_SWAP_XY           0x15  // Swap X and Y axes
#define MAKCU_CMD_TILT              0x16  // Tilt/z-axis
#define MAKCU_CMD_TURBO             0x17  // Turbo mode
#define MAKCU_CMD_WHEEL             0x18  // Scroll wheel
#define MAKCU_CMD_REMAP_AXIS        0x19  // Remap mouse axes

// Keyboard Commands (0xA1-0xAA)
#define MAKCU_CMD_KB_DISABLE        0xA1  // Disable keys
#define MAKCU_CMD_KB_DOWN           0xA2  // Key down
#define MAKCU_CMD_KB_INIT           0xA3  // Clear keyboard state
#define MAKCU_CMD_KB_ISDOWN         0xA4  // Query if key down
#define MAKCU_CMD_KB_STREAM         0xA5  // Stream keyboard
#define MAKCU_CMD_KB_MASK           0xA6  // Mask key
#define MAKCU_CMD_KB_PRESS          0xA7  // Tap key
#define MAKCU_CMD_KB_REMAP          0xA8  // Remap key
#define MAKCU_CMD_KB_STRING         0xA9  // Type string
#define MAKCU_CMD_KB_UP             0xAA  // Key up

// Misc Commands (0xB1-0xBF)
#define MAKCU_CMD_BAUD              0xB1  // Set/get baud rate
#define MAKCU_CMD_BYPASS            0xB2  // Bypass mode
#define MAKCU_CMD_DEVICE            0xB3  // Get device type
#define MAKCU_CMD_ECHO              0xB4  // Toggle echo
#define MAKCU_CMD_FAULT             0xB5  // Get fault info
#define MAKCU_CMD_HS                0xB7  // USB high-speed
#define MAKCU_CMD_INFO              0xB8  // System info
#define MAKCU_CMD_LED               0xB9  // LED control
#define MAKCU_CMD_LOG               0xBA  // Log level
#define MAKCU_CMD_REBOOT            0xBB  // Reboot device
#define MAKCU_CMD_RELEASE           0xBC  // Auto-release timer
#define MAKCU_CMD_SCREEN            0xBD  // Virtual screen size
#define MAKCU_CMD_SERIAL            0xBE  // Serial number
#define MAKCU_CMD_VERSION           0xBF  // Firmware version

// Command structures

// Mouse move (0x0D) - relative with optional bezier
typedef struct __attribute__((packed)) {
    int16_t dx;
    int16_t dy;
    uint8_t segments;
    int8_t cx1;
    int8_t cy1;
} makcu_move_t;

// Mouse moveto (0x0E) - absolute with optional bezier
typedef struct __attribute__((packed)) {
    int16_t x;
    int16_t y;
    uint8_t segments;
    int16_t cx1;
    int16_t cy1;
    int16_t cx2;
    int16_t cy2;
} makcu_moveto_t;

// Mouse raw frame (0x0B)
typedef struct __attribute__((packed)) {
    uint8_t buttons;
    int16_t x;
    int16_t y;
    int8_t wheel;
    int8_t pan;
    int8_t tilt;
} makcu_mo_t;

// Click scheduling (0x04)
typedef struct __attribute__((packed)) {
    uint8_t button;
    uint8_t count;
    uint8_t delay_ms;
} makcu_click_t;

// Keyboard press (0xA7)
typedef struct __attribute__((packed)) {
    uint8_t key;
    uint8_t hold_ms;
    uint8_t rand_ms;
} makcu_kb_press_t;

// Keyboard remap (0xA8)
typedef struct __attribute__((packed)) {
    uint8_t source;
    uint8_t target;
} makcu_kb_remap_t;

// Screen dimensions (0xBD)
typedef struct __attribute__((packed)) {
    int16_t width;
    int16_t height;
} makcu_screen_t;

// Makcu frame header
typedef struct __attribute__((packed)) {
    uint8_t start;      // 0x50
    uint8_t cmd;
    uint16_t len;       // little-endian payload length
} makcu_frame_header_t;

// API mode selection
typedef enum {
    API_MODE_KMBOX,     // Native KMBox protocol (optimized)
    API_MODE_MAKCU,     // Makcu binary protocol (translated)
    API_MODE_FERRUM     // Ferrum KM API text protocol
} api_mode_t;

// Helper functions
static inline bool is_makcu_frame_start(uint8_t byte) {
    return byte == MAKCU_FRAME_START;
}

static inline bool is_makcu_mouse_cmd(uint8_t cmd) {
    return (cmd >= 0x01 && cmd <= 0x19);
}

static inline bool is_makcu_keyboard_cmd(uint8_t cmd) {
    return (cmd >= 0xA1 && cmd <= 0xAA);
}

static inline bool is_makcu_misc_cmd(uint8_t cmd) {
    return (cmd >= 0xB1 && cmd <= 0xBF);
}

#endif // MAKCU_PROTOCOL_H
