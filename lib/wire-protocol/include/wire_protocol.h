/*
 * Wire Protocol v2 — Minimal Latency Bridge <-> KMBox UART Protocol
 *
 * Shared header included by both KMBox firmware and bridge firmware.
 *
 * Design principles:
 *   - No sync byte: command byte IS the framing (length via LUT)
 *   - Variable-length packets: common case (small move) = 3 bytes
 *   - Fire-and-forget: no ACK on hot path
 *   - 0x0A skipped: avoids '\n' collision with text protocol fallback
 *   - LUT-based parsing: peek first byte, look up length, consume atomically
 *
 * At 3 Mbaud: 1 byte = 3.33us, so MOVE8 (3B) = ~10us vs old 8B = ~27us
 */

#ifndef WIRE_PROTOCOL_H
#define WIRE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Command Bytes
// ============================================================================

#define WIRE_MOVE8       0x01  // [cmd][x:i8][y:i8]                    = 3 bytes
#define WIRE_MOVE16      0x02  // [cmd][xl][xh][yl][yh]                = 5 bytes
#define WIRE_BUTTONS     0x03  // [cmd][state:u8]                       = 2 bytes
#define WIRE_WHEEL       0x04  // [cmd][delta:i8]                       = 2 bytes
#define WIRE_MOVE_ALL    0x05  // [cmd][xl][xh][yl][yh][btn][whl]      = 7 bytes
#define WIRE_MOVE8_BTN   0x06  // [cmd][x:i8][y:i8][btn][whl]          = 5 bytes
#define WIRE_SMOOTH16    0x07  // [cmd][xl][xh][yl][yh][mode]          = 6 bytes
#define WIRE_KEYDOWN     0x08  // [cmd][keycode][modifiers]             = 3 bytes
#define WIRE_KEYUP       0x09  // [cmd][keycode]                        = 2 bytes
// 0x0A deliberately skipped — conflicts with '\n' in text protocol
#define WIRE_CONFIG      0x0B  // [cmd][param][value]                   = 3 bytes
#define WIRE_SMOOTH_CFG  0x0C  // [cmd][max_per_frame][vel_match]       = 3 bytes
#define WIRE_INFO_REQ    0x0D  // [cmd]                                 = 1 byte
#define WIRE_SMOOTH_CLR  0x0E  // [cmd]                                 = 1 byte
#define WIRE_CYCLE_HUM   0x0F  // [cmd]                                 = 1 byte
#define WIRE_PING        0xFE  // [cmd]                                 = 1 byte
#define WIRE_RESPONSE    0xFF  // [cmd][status][d0][d1][d2][d3]         = 6 bytes

#define WIRE_MAX_PACKET  7     // Maximum packet size in bytes

// Button masks (match HID standard)
#define WIRE_BTN_LEFT    0x01
#define WIRE_BTN_RIGHT   0x02
#define WIRE_BTN_MIDDLE  0x04
#define WIRE_BTN_BACK    0x08
#define WIRE_BTN_FORWARD 0x10

// Smooth injection modes
#define WIRE_INJECT_IMMEDIATE        0
#define WIRE_INJECT_SMOOTH           1
#define WIRE_INJECT_VELOCITY_MATCHED 2
#define WIRE_INJECT_MICRO            3

// Config parameters (for WIRE_CONFIG command)
#define WIRE_CFG_BAUD              0x01
#define WIRE_CFG_HUMANIZATION      0x02

// Response status codes
#define WIRE_RESP_OK       0x00
#define WIRE_RESP_INFO     0x0C  // Info response (humanization, queue, etc.)
#define WIRE_RESP_PONG     0xFE

// ============================================================================
// Command-Byte to Packet-Length LUT
// ============================================================================
// Index = command byte, value = total packet length including command byte.
// 0 = not a valid wire command (fall through to text/legacy parsing).

static const uint8_t wire_cmd_len[256] = {
    [0x00]           = 0,
    [WIRE_MOVE8]     = 3,
    [WIRE_MOVE16]    = 5,
    [WIRE_BUTTONS]   = 2,
    [WIRE_WHEEL]     = 2,
    [WIRE_MOVE_ALL]  = 7,
    [WIRE_MOVE8_BTN] = 5,
    [WIRE_SMOOTH16]  = 6,
    [WIRE_KEYDOWN]   = 3,
    [WIRE_KEYUP]     = 2,
    // 0x0A = 0 (reserved, '\n')
    [WIRE_CONFIG]     = 3,
    [WIRE_SMOOTH_CFG] = 3,
    [WIRE_INFO_REQ]   = 1,
    [WIRE_SMOOTH_CLR] = 1,
    [WIRE_CYCLE_HUM]  = 1,
    // 0x10-0xFD = 0 (invalid)
    [WIRE_PING]       = 1,
    [WIRE_RESPONSE]   = 6,
};

// ============================================================================
// Inline Packet Builders (Bridge side — produce wire packets for TX)
// ============================================================================

// Small mouse move (most common, 80%+ of traffic)
static inline size_t wire_build_move8(uint8_t *buf, int8_t x, int8_t y) {
    buf[0] = WIRE_MOVE8;
    buf[1] = (uint8_t)x;
    buf[2] = (uint8_t)y;
    return 3;
}

// Large mouse move (when delta exceeds ±127)
static inline size_t wire_build_move16(uint8_t *buf, int16_t x, int16_t y) {
    buf[0] = WIRE_MOVE16;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    return 5;
}

// Auto-select: MOVE8 if both axes fit in i8, else MOVE16
static inline size_t wire_build_move(uint8_t *buf, int16_t x, int16_t y) {
    if (x >= -128 && x <= 127 && y >= -128 && y <= 127) {
        return wire_build_move8(buf, (int8_t)x, (int8_t)y);
    }
    return wire_build_move16(buf, x, y);
}

// Button state update
static inline size_t wire_build_buttons(uint8_t *buf, uint8_t state) {
    buf[0] = WIRE_BUTTONS;
    buf[1] = state;
    return 2;
}

// Scroll wheel
static inline size_t wire_build_wheel(uint8_t *buf, int8_t delta) {
    buf[0] = WIRE_WHEEL;
    buf[1] = (uint8_t)delta;
    return 2;
}

// Combined move + buttons + wheel (16-bit axes)
static inline size_t wire_build_move_all(uint8_t *buf, int16_t x, int16_t y,
                                          uint8_t buttons, int8_t wheel) {
    buf[0] = WIRE_MOVE_ALL;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = buttons;
    buf[6] = (uint8_t)wheel;
    return 7;
}

// Combined move + buttons + wheel (8-bit axes)
static inline size_t wire_build_move8_btn(uint8_t *buf, int8_t x, int8_t y,
                                           uint8_t buttons, int8_t wheel) {
    buf[0] = WIRE_MOVE8_BTN;
    buf[1] = (uint8_t)x;
    buf[2] = (uint8_t)y;
    buf[3] = buttons;
    buf[4] = (uint8_t)wheel;
    return 5;
}

// Smooth (humanized) move
static inline size_t wire_build_smooth16(uint8_t *buf, int16_t x, int16_t y,
                                          uint8_t mode) {
    buf[0] = WIRE_SMOOTH16;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = mode;
    return 6;
}

// Key press
static inline size_t wire_build_keydown(uint8_t *buf, uint8_t keycode,
                                         uint8_t modifiers) {
    buf[0] = WIRE_KEYDOWN;
    buf[1] = keycode;
    buf[2] = modifiers;
    return 3;
}

// Key release
static inline size_t wire_build_keyup(uint8_t *buf, uint8_t keycode) {
    buf[0] = WIRE_KEYUP;
    buf[1] = keycode;
    return 2;
}

// Configuration change
static inline size_t wire_build_config(uint8_t *buf, uint8_t param, uint8_t value) {
    buf[0] = WIRE_CONFIG;
    buf[1] = param;
    buf[2] = value;
    return 3;
}

// Smooth injection config
static inline size_t wire_build_smooth_cfg(uint8_t *buf, uint8_t max_per_frame,
                                            uint8_t vel_match) {
    buf[0] = WIRE_SMOOTH_CFG;
    buf[1] = max_per_frame;
    buf[2] = vel_match;
    return 3;
}

// Info request
static inline size_t wire_build_info_req(uint8_t *buf) {
    buf[0] = WIRE_INFO_REQ;
    return 1;
}

// Clear smooth queue
static inline size_t wire_build_smooth_clr(uint8_t *buf) {
    buf[0] = WIRE_SMOOTH_CLR;
    return 1;
}

// Cycle humanization mode
static inline size_t wire_build_cycle_hum(uint8_t *buf) {
    buf[0] = WIRE_CYCLE_HUM;
    return 1;
}

// Keepalive ping
static inline size_t wire_build_ping(uint8_t *buf) {
    buf[0] = WIRE_PING;
    return 1;
}

// Response packet (KMBox -> Bridge)
static inline size_t wire_build_response(uint8_t *buf, uint8_t status,
                                          uint8_t d0, uint8_t d1,
                                          uint8_t d2, uint8_t d3) {
    buf[0] = WIRE_RESPONSE;
    buf[1] = status;
    buf[2] = d0;
    buf[3] = d1;
    buf[4] = d2;
    buf[5] = d3;
    return 6;
}

// ============================================================================
// Parser Helper (KMBox side — peek from DMA ring buffer)
// ============================================================================

// Returns packet length if cmd_byte is a valid wire command, 0 otherwise.
// Caller checks rx_available() >= returned length before consuming.
static inline uint8_t wire_get_packet_len(uint8_t cmd_byte) {
    return wire_cmd_len[cmd_byte];
}

#endif // WIRE_PROTOCOL_H
