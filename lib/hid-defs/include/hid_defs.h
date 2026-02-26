/**
 * Shared HID Definitions
 *
 * Canonical HID mouse button bit masks used by both KMBox firmware,
 * bridge firmware, wire protocol, and fast command protocol.
 *
 * Values follow the USB HID Usage Table, Button Page (0x09).
 */

#ifndef HID_DEFS_H
#define HID_DEFS_H

// HID mouse button bit masks
#define HID_BTN_LEFT    0x01
#define HID_BTN_RIGHT   0x02
#define HID_BTN_MIDDLE  0x04
#define HID_BTN_BACK    0x08
#define HID_BTN_FORWARD 0x10

#endif // HID_DEFS_H
