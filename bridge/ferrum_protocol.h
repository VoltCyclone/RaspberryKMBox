/**
 * Ferrum KM API Protocol Definitions
 * 
 * Text-based protocol: km.command(arg1, arg2, ...)\n
 * Response: >>> after each command
 */

#ifndef FERRUM_PROTOCOL_H
#define FERRUM_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// Protocol constants
#define FERRUM_MAX_LINE 256
#define FERRUM_RESPONSE ">>>\r\n"
#define FERRUM_RESPONSE_LEN 5

// Command prefixes
#define FERRUM_PREFIX "km."
#define FERRUM_PREFIX_LEN 3

// Mouse commands
#define FERRUM_CMD_MOVE "move"
#define FERRUM_CMD_LEFT "left"
#define FERRUM_CMD_RIGHT "right"
#define FERRUM_CMD_MIDDLE "middle"
#define FERRUM_CMD_SIDE1 "side1"
#define FERRUM_CMD_SIDE2 "side2"
#define FERRUM_CMD_WHEEL "wheel"

// Keyboard commands
#define FERRUM_CMD_DOWN "down"
#define FERRUM_CMD_UP "up"
#define FERRUM_CMD_PRESS "press"
#define FERRUM_CMD_MULTIDOWN "multidown"
#define FERRUM_CMD_MULTIUP "multiup"
#define FERRUM_CMD_MULTIPRESS "multipress"

#endif // FERRUM_PROTOCOL_H
