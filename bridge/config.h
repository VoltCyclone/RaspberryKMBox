/**
 * Configuration for KMBox Bridge Autopilot
 * 
 * This header defines all tunable parameters for the color tracking
 * and mouse control system running on the RP2350 bridge.
 * 
 * UART Architecture:
 * Both devices use UART0 for communication with crossed wiring:
 * This eliminates the need for PIO-based UART and frees up state machines.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"

// Hardware Configuration
#define UART_TX_PIN          PICO_DEFAULT_UART_TX_PIN 
#define UART_RX_PIN          PICO_DEFAULT_UART_RX_PIN 
#define UART_BAUD            2000000  // Baud rate for RP2350<->RP2040 communication (2 Mbaud)
#define LED_PIN              PICO_DEFAULT_LED_PIN
#define WS2812_PIN           PICO_DEFAULT_WS2812_PIN

// Mode button pin (from CMake or default)
#ifdef BRIDGE_MODE_BTN_PIN
#define MODE_BUTTON_PIN      BRIDGE_MODE_BTN_PIN
#else
#define MODE_BUTTON_PIN      7        // API mode toggle button (same as KMBox reset button)
#endif

// ROI (Region of Interest) Configuration
#define ROI_DEFAULT_SIZE     48       // Default ROI size (48x48 pixels)
#define ROI_MAX_SIZE         64       // Maximum supported ROI size

// Color Thresholds (for red target detection)
#define DEFAULT_R_MIN        180      // Minimum red value (0-255)
#define DEFAULT_R_MAX        255      // Maximum red value (0-255)
#define DEFAULT_G_MAX        80       // Maximum green value (0-255)
#define DEFAULT_B_MAX        80       // Maximum blue value (0-255)

// Tracking Parameters
#define DEFAULT_GAIN_X       1.0f     // Horizontal sensitivity multiplier
#define DEFAULT_GAIN_Y       1.0f     // Vertical sensitivity multiplier
#define DEFAULT_DEADZONE     2        // Deadzone in pixels (ignore small movements)
#define DEFAULT_MIN_BLOB     10       // Minimum blob size in pixels to track

// Smoothing Configuration
#define SMOOTHING_FACTOR     0.7f     // Exponential moving average (0.0-1.0)

// Mouse Movement Limits
#define MAX_MOUSE_DELTA      127      // Maximum mouse delta per frame

// NeoPixel Status Colors (RGB format)
#define LED_COLOR_BOOTING       0, 0, 255      // Blue - System booting
#define LED_COLOR_IDLE          255, 0, 0      // Red - Waiting for frames
#define LED_COLOR_CDC_CONNECTED 0, 255, 255    // Cyan - CDC connected, no frames yet
#define LED_COLOR_TRACKING      0, 255, 0      // Green - Actively tracking
#define LED_COLOR_DISABLED      255, 255, 0    // Yellow - Tracking disabled
#define LED_COLOR_ACTIVITY      0, 0, 255      // Blue - Brief activity pulse
#define LED_COLOR_ERROR         255, 0, 0      // Red - Error state

// Protocol Commands (PC -> Bridge via USB CDC)
#define CMD_CONFIG           'C'      // Update configuration
#define CMD_ENABLE           'E'      // Enable/disable tracking
#define CMD_STATUS           'S'      // Request status

// Protocol Commands (Bridge -> RP2040 via PIO UART)
#define CMD_MOUSE_MOVE       0x01     // Mouse move command
#define CMD_MOUSE_CLICK      0x02     // Mouse click command

// Status Messages (RP2040 -> Bridge via UART RX)
#define STATUS_MSG_PREFIX    ">>>"    // Prefix for status messages
#define STATUS_BUFFER_SIZE   256      // Maximum status message length

// Frame Protocol (PC -> Bridge via USB CDC)
#define FRAME_MAGIC_0        'F'      // Frame header magic bytes
#define FRAME_MAGIC_1        'R'

// Frame Buffer Configuration
#define FRAME_BUFFER_SIZE    (ROI_MAX_SIZE * ROI_MAX_SIZE * 3)

#endif // CONFIG_H