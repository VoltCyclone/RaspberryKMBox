/**
 * Fast Binary Command Definitions for Bridge -> KMBox UART
 *
 * Thin wrapper — all definitions now live in the shared library
 * lib/fast-protocol/include/fast_protocol.h.  This file exists so
 * existing bridge #include "fast_commands.h" directives continue to work.
 */

#ifndef BRIDGE_FAST_COMMANDS_H
#define BRIDGE_FAST_COMMANDS_H

#include "fast_protocol.h"

// Legacy button mask aliases — prefer HID_BTN_* from hid_defs.h in new code
#define FAST_BTN_LEFT       HID_BTN_LEFT
#define FAST_BTN_RIGHT      HID_BTN_RIGHT
#define FAST_BTN_MIDDLE     HID_BTN_MIDDLE
#define FAST_BTN_BACK       HID_BTN_BACK
#define FAST_BTN_FORWARD    HID_BTN_FORWARD

#endif // BRIDGE_FAST_COMMANDS_H
