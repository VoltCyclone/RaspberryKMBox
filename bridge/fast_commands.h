/**
 * Fast Binary Command Definitions for Bridge → KMBox UART
 *
 * All bridge translators (Ferrum, Makcu, tracker) should produce these
 * 8-byte packets instead of text strings.  The KMBox already has an
 * optimized binary parser that handles them with zero string parsing.
 *
 * Key command IDs:
 *   0x01  FAST_CMD_MOUSE_MOVE   — direct accumulator (buttons + wheel + move)
 *   0x02  FAST_CMD_MOUSE_CLICK  — button click with repeat count
 *   0x07  FAST_CMD_SMOOTH_MOVE  — smooth injection queue (humanized)
 */

#ifndef BRIDGE_FAST_COMMANDS_H
#define BRIDGE_FAST_COMMANDS_H

#include <stdint.h>
#include <string.h>

// Command IDs (must match defines.h on KMBox side)
#define FAST_CMD_MOUSE_MOVE     0x01
#define FAST_CMD_MOUSE_CLICK    0x02
#define FAST_CMD_SMOOTH_MOVE    0x07
#define FAST_CMD_SMOOTH_CONFIG  0x08
#define FAST_CMD_SMOOTH_CLEAR   0x09
#define FAST_CMD_CYCLE_HUMAN    0x0F
#define FAST_CMD_PING           0xFE

// Smooth injection modes (must match inject_mode_t on KMBox side)
#define INJECT_MODE_IMMEDIATE       0
#define INJECT_MODE_SMOOTH          1
#define INJECT_MODE_VELOCITY_MATCHED 2
#define INJECT_MODE_MICRO           3

#define FAST_CMD_PACKET_SIZE    8

// ============================================================================
// Inline packet builders — produce 8-byte packets, return 8 always
// ============================================================================

/**
 * Build FAST_CMD_SMOOTH_MOVE (0x07) packet.
 * KMBox routes this through smooth_inject_movement() which applies
 * humanization (easing, subdivision, tremor, overshoot) automatically.
 * Use for: km.move(), tracker aim commands, makcu MOVE.
 */
static inline size_t fast_build_smooth_move(uint8_t *buf, int16_t x, int16_t y, uint8_t mode) {
    buf[0] = FAST_CMD_SMOOTH_MOVE;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = mode;
    buf[6] = 0;
    buf[7] = 0;
    return FAST_CMD_PACKET_SIZE;
}

/**
 * Build FAST_CMD_MOUSE_MOVE (0x01) packet.
 * KMBox routes this through kmbox_add_mouse_movement() (direct accumulator)
 * plus optional buttons + wheel.  No smooth queue / humanization subdivision.
 * Use for: button state changes, wheel, raw passthrough moves.
 */
static inline size_t fast_build_mouse_move(uint8_t *buf, int16_t x, int16_t y,
                                           uint8_t buttons, int8_t wheel) {
    buf[0] = FAST_CMD_MOUSE_MOVE;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = buttons;
    buf[6] = (uint8_t)wheel;
    buf[7] = 0;
    return FAST_CMD_PACKET_SIZE;
}

/**
 * Build FAST_CMD_MOUSE_CLICK (0x02) packet.
 * KMBox generates press + release pairs internally.
 */
static inline size_t fast_build_mouse_click(uint8_t *buf, uint8_t button, uint8_t count) {
    buf[0] = FAST_CMD_MOUSE_CLICK;
    buf[1] = button;
    buf[2] = count;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
    return FAST_CMD_PACKET_SIZE;
}

// Button masks (match HID standard)
#define FAST_BTN_LEFT       0x01
#define FAST_BTN_RIGHT      0x02
#define FAST_BTN_MIDDLE     0x04
#define FAST_BTN_BACK       0x08
#define FAST_BTN_FORWARD    0x10

#endif // BRIDGE_FAST_COMMANDS_H
