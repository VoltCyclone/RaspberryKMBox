/**
 * High-Performance Bridge Protocol
 * 
 * Optimized binary protocol for bridge → KMBox UART communication
 * Designed for minimal overhead and maximum throughput at 2 Mbaud
 * 
 * All multi-byte values are little-endian
 */

#ifndef BRIDGE_PROTOCOL_H
#define BRIDGE_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Protocol Constants
// ============================================================================

#define BRIDGE_SYNC_BYTE 0xBD  // Bridge protocol sync marker

// Command IDs (single byte)
typedef enum {
    BRIDGE_CMD_MOUSE_MOVE = 0x01,      // x:i16, y:i16 (5 bytes total)
    BRIDGE_CMD_MOUSE_WHEEL = 0x02,     // wheel:i8 (2 bytes total)
    BRIDGE_CMD_BUTTON_SET = 0x03,      // mask:u8, state:u8 (3 bytes total)
    BRIDGE_CMD_MOUSE_MOVE_WHEEL = 0x04,// x:i16, y:i16, wheel:i8 (6 bytes total)
    BRIDGE_CMD_PING = 0xFE,            // Keepalive (1 byte total)
    BRIDGE_CMD_RESET = 0xFF            // Reset state (1 byte total)
} bridge_cmd_t;

// Response codes
typedef enum {
    BRIDGE_RESP_ACK = 0xA0,            // Command accepted
    BRIDGE_RESP_NACK = 0xA1,           // Command rejected
    BRIDGE_RESP_PONG = 0xA2            // Ping response
} bridge_resp_t;

// Button masks (matching HID standard)
#define BRIDGE_BTN_LEFT    0x01
#define BRIDGE_BTN_RIGHT   0x02
#define BRIDGE_BTN_MIDDLE  0x04
#define BRIDGE_BTN_SIDE1   0x08  // Back
#define BRIDGE_BTN_SIDE2   0x10  // Forward

// ============================================================================
// Packet Structures
// ============================================================================

// All packets start with sync byte
// Packet format: [SYNC][CMD][PAYLOAD...]
// No checksum needed - UART has hardware CRC, and we retry on errors

typedef struct __attribute__((packed)) {
    uint8_t sync;   // BRIDGE_SYNC_BYTE
    uint8_t cmd;    // bridge_cmd_t
    int16_t x;
    int16_t y;
} bridge_pkt_mouse_move_t;  // 6 bytes

typedef struct __attribute__((packed)) {
    uint8_t sync;   // BRIDGE_SYNC_BYTE
    uint8_t cmd;    // bridge_cmd_t
    int8_t wheel;
} bridge_pkt_mouse_wheel_t;  // 3 bytes

typedef struct __attribute__((packed)) {
    uint8_t sync;   // BRIDGE_SYNC_BYTE
    uint8_t cmd;    // bridge_cmd_t
    uint8_t mask;   // Button mask
    uint8_t state;  // 0=release, 1=press
} bridge_pkt_button_set_t;  // 4 bytes

typedef struct __attribute__((packed)) {
    uint8_t sync;   // BRIDGE_SYNC_BYTE
    uint8_t cmd;    // bridge_cmd_t
    int16_t x;
    int16_t y;
    int8_t wheel;
} bridge_pkt_mouse_move_wheel_t;  // 7 bytes

typedef struct __attribute__((packed)) {
    uint8_t sync;   // BRIDGE_SYNC_BYTE
    uint8_t cmd;    // bridge_cmd_t
} bridge_pkt_simple_t;  // 2 bytes (for PING/RESET)

// Union for easy packet access
typedef union {
    uint8_t raw[7];  // Max packet size
    struct {
        uint8_t sync;
        uint8_t cmd;
        uint8_t payload[5];
    } generic;
    bridge_pkt_mouse_move_t mouse_move;
    bridge_pkt_mouse_wheel_t mouse_wheel;
    bridge_pkt_button_set_t button_set;
    bridge_pkt_mouse_move_wheel_t mouse_move_wheel;
    bridge_pkt_simple_t simple;
} bridge_packet_t;

// ============================================================================
// Builder Functions (Bridge Side)
// ============================================================================

static inline size_t bridge_build_mouse_move(uint8_t* buf, int16_t x, int16_t y) {
    buf[0] = BRIDGE_SYNC_BYTE;
    buf[1] = BRIDGE_CMD_MOUSE_MOVE;
    *((int16_t*)(buf + 2)) = x;
    *((int16_t*)(buf + 4)) = y;
    return 6;
}

static inline size_t bridge_build_mouse_wheel(uint8_t* buf, int8_t wheel) {
    buf[0] = BRIDGE_SYNC_BYTE;
    buf[1] = BRIDGE_CMD_MOUSE_WHEEL;
    buf[2] = wheel;
    return 3;
}

static inline size_t bridge_build_button_set(uint8_t* buf, uint8_t mask, uint8_t state) {
    buf[0] = BRIDGE_SYNC_BYTE;
    buf[1] = BRIDGE_CMD_BUTTON_SET;
    buf[2] = mask;
    buf[3] = state;
    return 4;
}

static inline size_t bridge_build_mouse_move_wheel(uint8_t* buf, int16_t x, int16_t y, int8_t wheel) {
    buf[0] = BRIDGE_SYNC_BYTE;
    buf[1] = BRIDGE_CMD_MOUSE_MOVE_WHEEL;
    *((int16_t*)(buf + 2)) = x;
    *((int16_t*)(buf + 4)) = y;
    buf[6] = wheel;
    return 7;
}

static inline size_t bridge_build_ping(uint8_t* buf) {
    buf[0] = BRIDGE_SYNC_BYTE;
    buf[1] = BRIDGE_CMD_PING;
    return 2;
}

static inline size_t bridge_build_reset(uint8_t* buf) {
    buf[0] = BRIDGE_SYNC_BYTE;
    buf[1] = BRIDGE_CMD_RESET;
    return 2;
}

// ============================================================================
// Performance Comparison
// ============================================================================
//
// Bridge Protocol:
// - Move: [0xBD, 0x01, x_lo, x_hi, y_lo, y_hi] = 6 bytes (25% smaller)
// - Button: [0xBD, 0x03, mask, state] = 4 bytes (50% smaller)
// - Wheel: [0xBD, 0x02, wheel] = 3 bytes (62.5% smaller)
// - Move+Wheel: [0xBD, 0x04, x_lo, x_hi, y_lo, y_hi, wheel] = 7 bytes (vs 16 bytes = 56% smaller)

#endif // BRIDGE_PROTOCOL_H
