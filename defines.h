/*
 * Consolidated Defines for PIOKMbox
 * 
 * This header consolidates and organizes all #define constants used throughout
 * the PIOKMbox project, eliminating duplicates and providing clear categorization.
 */

#ifndef DEFINES_H
#define DEFINES_H

//--------------------------------------------------------------------+
// Performance Tuning Options
//--------------------------------------------------------------------+

// Main loop time sampling frequency
// Lower = more frequent time checks (more overhead, better responsiveness)
// Higher = less frequent time checks (less overhead, slightly delayed task execution)
// Valid values: 16, 32, 64, 128
#ifndef MAIN_LOOP_TIME_SAMPLE_INTERVAL
#define MAIN_LOOP_TIME_SAMPLE_INTERVAL  32
#endif

// RP2350 DSP instructions for fixed-point math (Cortex-M33 SMULL)
// Always enabled - RP2040 support has been dropped
// Note: SDK's pico_float/pico_double already use DCP hardware acceleration
// automatically on RP2350 (LIB_PICO_DOUBLE_PICO=1, LIB_PICO_FLOAT_PICO_VFP=1)
#define ENABLE_DSP_FIXED_POINT  1

//--------------------------------------------------------------------+
// HARDWARE CONFIGURATION
//--------------------------------------------------------------------+
// RP2350 overclocked to 240MHz for real-time tracking
// NOTE: This increases power draw and may be unstable on some boards.
#define DEFAULT_CPU_FREQ 240000

#ifndef CPU_FREQ
#define CPU_FREQ DEFAULT_CPU_FREQ
#endif

// Pin definitions - set via CMake compile definitions based on board type
// Override these via CMakeLists.txt for different boards:
//   - Adafruit Metro RP2350:   D+=32, D-=33, 5V=29, LED=23, NeoPixel=25
//   - Adafruit Feather RP2040: D+=16, D-=17, 5V=18, LED=13, NeoPixel=21
//   - Pico/Pico2 default:      D+=16, D-=17, 5V=18, LED=25, NeoPixel=21

#ifndef PIN_USB_HOST_DP
#define PIN_USB_HOST_DP         (16u)   // PIO USB Host D+ pin (default)
#endif
#ifndef PIN_USB_HOST_DM
#define PIN_USB_HOST_DM         (17u)   // PIO USB Host D- pin (default, must be D+ + 1)
#endif
#ifndef PIN_USB_5V
#define PIN_USB_5V              (18u)   // Power pin for USB host
#endif
#ifndef PIN_LED
#define PIN_LED                 (13u)   // Status LED pin
#endif
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL            (21u)   // Neopixel data pin
#endif
#ifndef NEOPIXEL_POWER
#define NEOPIXEL_POWER          (20u)   // Neopixel power pin (255 = not used/always on)
#endif

#define PIN_BUTTON              (7u)    // Reset button pin

// UART configuration for KMBox serial communication with RP2350 Bridge
// Physical connection (crossed wiring):
// The RP2350 bridge provides USB CDC interface to PC and translates to/from KMBox
#define BRIDGE_UART_TX_PIN      PICO_DEFAULT_UART_TX_PIN   
#define BRIDGE_UART_RX_PIN      PICO_DEFAULT_UART_RX_PIN  
#define KMBOX_UART              uart0    // UART0 instance
#define KMBOX_UART_TX_PIN       PICO_DEFAULT_UART_TX_PIN    // UART0 TX (to Bridge RX)
#define KMBOX_UART_RX_PIN       PICO_DEFAULT_UART_RX_PIN    // UART0 RX (from Bridge TX)
#define KMBOX_UART_BAUDRATE     2000000  // Baud rate (must match bridge) - 2 Mbaud for max throughput
#define KMBOX_UART_FIFO_SIZE    32       // UART FIFO size for buffering

//--------------------------------------------------------------------+
// FAST BINARY COMMAND PROTOCOL
//--------------------------------------------------------------------+
// Ultra-fast binary protocol for minimal latency mouse/keyboard control
// At 2 Mbps: 8 bytes takes only ~40µs vs ~700µs at 115200
//
// Memory-aligned 8-byte packets enable single-cycle struct access
// Clock-synchronized processing for deterministic timing
//
// Command format (8 bytes fixed, 4-byte aligned):
//   Byte 0: Command type (FAST_CMD_*)
//   Byte 1-7: Command-specific payload
//
// Mouse move:    [0x01][X_lo][X_hi][Y_lo][Y_hi][Buttons][Wheel][0x00]
// Mouse click:   [0x02][Button][Count][0x00][0x00][0x00][0x00][0x00]
// Key press:     [0x03][Keycode][Modifiers][0x00][0x00][0x00][0x00][0x00]
// Key combo:     [0x04][Key1][Key2][Key3][Key4][Modifiers][0x00][0x00]
// Multi-move:    [0x05][X1][Y1][X2][Y2][X3][Y3][Flags] - 3 moves in 1 packet!
// Smooth move:   [0x07][X_lo][X_hi][Y_lo][Y_hi][Mode][0x00][0x00]
// Smooth config: [0x08][MaxPerFrame][VelMatch][0x00][0x00][0x00][0x00][0x00]
// Smooth clear:  [0x09][0x00][0x00][0x00][0x00][0x00][0x00][0x00]
// Timed move:    [0x0A][X_lo][X_hi][Y_lo][Y_hi][Time_lo][Time_hi][Mode]
// Sync:          [0x0B][SeqNum][Time0][Time1][Time2][Time3][0x00][0x00]

#define FAST_CMD_MOUSE_MOVE     0x01    // Relative mouse movement + buttons + wheel
#define FAST_CMD_MOUSE_CLICK    0x02    // Click with button and count
#define FAST_CMD_KEY_PRESS      0x03    // Single key press/release
#define FAST_CMD_KEY_COMBO      0x04    // Multi-key combo (up to 4 keys)
#define FAST_CMD_MULTI_MOVE     0x05    // 3 mouse moves in one packet
#define FAST_CMD_MOUSE_ABS      0x06    // Absolute mouse position
#define FAST_CMD_SMOOTH_MOVE    0x07    // Smooth injection with mode selection
#define FAST_CMD_SMOOTH_CONFIG  0x08    // Configure smooth injection parameters
#define FAST_CMD_SMOOTH_CLEAR   0x09    // Clear smooth injection queue
#define FAST_CMD_TIMED_MOVE     0x0A    // Movement with timestamp for sync
#define FAST_CMD_SYNC           0x0B    // Clock synchronization packet
#define FAST_CMD_INFO           0x0C    // Request info (humanization, inject mode, etc)
                                        // Response byte 7 bitfield: [0]=jitter_en [1]=vel_match [2:4]=queue_depth_3bit [5:7]=reserved
#define FAST_CMD_INFO_EXT       0x0E    // Extended stats: [0x0E][queue_count][queue_cap][overshoot%][total_inj_lo][total_inj_hi][overflows_lo][overflows_hi]
#define FAST_CMD_PING           0xFE    // Fast ping (response: 0xFF)
#define FAST_CMD_RESPONSE       0xFF    // Response/ACK

#define FAST_CMD_PACKET_SIZE    8       // Fixed 8-byte packet size (4-byte aligned)
#define FAST_CMD_SYNC_BYTE      0xAA    // Optional sync byte for framing
#define FAST_CMD_QUEUE_SIZE     16      // Number of packets in aligned ring buffer

// Clock synchronization timing
// NOTE: Actual HID frame rate depends on connected mouse's bInterval (1-8ms typical)
// Gaming mice: 1ms (1000Hz), Standard mice: 8ms (125Hz)
// These are defaults/fallbacks
#define FAST_CMD_FRAME_US_DEFAULT  1000 // Default 1ms for gaming mice (1000Hz)
#define FAST_CMD_JITTER_US         100  // Acceptable timing jitter
#define FAST_CMD_SYNC_INTERVAL     1000 // Sync every 1000 packets (~1 second)

// Button bit flags for FAST_CMD_MOUSE_MOVE
#define FAST_BTN_LEFT           0x01
#define FAST_BTN_RIGHT          0x02
#define FAST_BTN_MIDDLE         0x04
#define FAST_BTN_BACK           0x08
#define FAST_BTN_FORWARD        0x10

//--------------------------------------------------------------------+
// Memory-Aligned Binary Command Structures (packed, 8-byte aligned)
//--------------------------------------------------------------------+
// These structures allow direct memory casting from the UART buffer
// for zero-copy command processing. The compiler will generate
// optimal single-instruction loads for 16/32-bit fields.

// Generic 8-byte packet (for alignment)
typedef union __attribute__((packed, aligned(4))) {
    uint8_t  bytes[8];
    uint32_t words[2];  // For fast 32-bit copy operations
    uint64_t qword;     // For single-instruction 64-bit copy (if available)
} fast_packet_t;

// Mouse move command: 0x01
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x01
    int16_t  x;         // X movement (little-endian)
    int16_t  y;         // Y movement (little-endian)
    uint8_t  buttons;   // Button bit flags
    int8_t   wheel;     // Wheel movement
    uint8_t  _pad;      // Padding to 8 bytes
} fast_cmd_move_t;

// Mouse click command: 0x02
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x02
    uint8_t  button;    // Button number (0=L, 1=R, 2=M, etc)
    uint8_t  count;     // Click count
    uint8_t  _pad[5];   // Padding to 8 bytes
} fast_cmd_click_t;

// Key press command: 0x03
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x03
    uint8_t  keycode;   // USB HID keycode
    uint8_t  modifiers; // Modifier flags
    uint8_t  _pad[5];   // Padding to 8 bytes
} fast_cmd_key_t;

// Key combo command: 0x04
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x04
    uint8_t  keys[4];   // Up to 4 keycodes
    uint8_t  modifiers; // Modifier flags
    uint8_t  _pad[2];   // Padding to 8 bytes
} fast_cmd_combo_t;

// Multi-move command: 0x05 (3 moves in one packet!)
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x05
    int8_t   x1, y1;    // Move 1
    int8_t   x2, y2;    // Move 2
    int8_t   x3, y3;    // Move 3
    uint8_t  flags;     // Reserved flags
} fast_cmd_multi_t;

// Smooth move command: 0x07
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x07
    int16_t  x;         // X movement
    int16_t  y;         // Y movement
    uint8_t  mode;      // Injection mode (0-3)
    uint8_t  _pad[2];   // Padding to 8 bytes
} fast_cmd_smooth_t;

// Smooth config command: 0x08
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;           // 0x08
    uint8_t  max_per_frame; // Max pixels per frame (1-127)
    uint8_t  vel_match;     // Velocity matching enable
    uint8_t  _pad[5];       // Padding to 8 bytes
} fast_cmd_config_t;

// Timed move command: 0x0A (for clock-synchronized injection)
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x0A
    int16_t  x;         // X movement
    int16_t  y;         // Y movement
    uint16_t time_us;   // Execution time offset (microseconds from sync)
    uint8_t  mode;      // Injection mode
} fast_cmd_timed_t;

// Sync command: 0x0B (clock synchronization)
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x0B
    uint8_t  seq_num;   // Sequence number for ordering
    uint32_t timestamp; // PC timestamp in microseconds (for RTT calc)
    uint8_t  _pad[2];   // Padding to 8 bytes
} fast_cmd_sync_t;

// Ping command: 0xFE
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0xFE
    uint8_t  _pad[7];   // Padding to 8 bytes
} fast_cmd_ping_t;

// Response command: 0xFF
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0xFF
    uint8_t  status;    // Response status
    uint32_t timestamp; // Device timestamp for sync
    uint8_t  _pad[2];   // Padding to 8 bytes
} fast_cmd_response_t;

// Union for type-punned access (cast packet buffer directly)
typedef union __attribute__((packed, aligned(4))) {
    fast_packet_t      raw;
    fast_cmd_move_t    move;
    fast_cmd_click_t   click;
    fast_cmd_key_t     key;
    fast_cmd_combo_t   combo;
    fast_cmd_multi_t   multi;
    fast_cmd_smooth_t  smooth;
    fast_cmd_config_t  config;
    fast_cmd_timed_t   timed;
    fast_cmd_sync_t    sync;
    fast_cmd_ping_t    ping;
    fast_cmd_response_t response;
} fast_cmd_union_t;

// Compile-time validation
_Static_assert(sizeof(fast_cmd_move_t) == 8, "fast_cmd_move_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_click_t) == 8, "fast_cmd_click_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_key_t) == 8, "fast_cmd_key_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_combo_t) == 8, "fast_cmd_combo_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_multi_t) == 8, "fast_cmd_multi_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_smooth_t) == 8, "fast_cmd_smooth_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_config_t) == 8, "fast_cmd_config_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_timed_t) == 8, "fast_cmd_timed_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_sync_t) == 8, "fast_cmd_sync_t must be 8 bytes");
_Static_assert(sizeof(fast_cmd_union_t) == 8, "fast_cmd_union_t must be 8 bytes");
_Static_assert(sizeof(fast_packet_t) == 8, "fast_packet_t must be 8 bytes");

#define DEBUG_OUTPUT_USB_CDC    0        // Always disable debug output over USB CDC

// USB port configuration
#define USB_DEVICE_PORT         0       // On-board USB controller port (device mode)
#define USB_HOST_PORT           1       // PIO USB controller port (host mode)
#define USB_DM_PIN_OFFSET       1       // DM pin offset from DP pin (DM = DP + 1)

// Core assignment
#define USB_DEVICE_CORE         0       // Core 0 handles USB device tasks
#define USB_HOST_CORE           1       // Core 1 handles USB host tasks

//--------------------------------------------------------------------+
// TIMING CONSTANTS
//--------------------------------------------------------------------+

// Boot and initialization timing
#define COLD_BOOT_STABILIZATION_MS      2000    // Initial cold boot delay
#define USB_DEVICE_STABILIZATION_MS     2000    // USB device init timeout
#define CORE1_INIT_DELAY_MS             100     // Initial delay before core1 USB host initialization
#define CORE1_EXTRA_INIT_DELAY_MS       500     // Additional core1 init delay
#define USB_STACK_READY_DELAY_MS        1000    // Delay before USB stack ready
#define FINAL_STABILIZATION_DELAY_MS    3000    // Pre-power-enable delay
#define POWER_ENABLE_DELAY_MS           1000    // Delay between power enables
#define DEVICE_READY_TIMEOUT_MS         3000    // USB device ready timeout

// Retry and recovery timing
#define ERROR_RETRY_DELAY_MS            2000    // Delay between USB host initialization retry attempts
#define USB_INIT_MAX_RETRIES            8       // Max USB init attempts
#define USB_INIT_BASE_RETRY_DELAY_MS    ERROR_RETRY_DELAY_MS // Base retry delay
#define USB_INIT_PROGRESSIVE_DELAY_MS   500     // Additional delay per retry
#define FALLBACK_HEARTBEAT_INTERVAL_MS  2000    // Fallback mode heartbeat
#define PERIODIC_REINIT_ATTEMPTS        30      // Reinit every N heartbeats

// Button timing
#define BUTTON_HOLD_TRIGGER_MS          3000    // Hold time for USB reset
#define BUTTON_DEBOUNCE_MS              10      // Button polling interval
#define USB_RESET_COOLDOWN_MS           2000    // Post-reset cooldown

// Main loop task timing
#define HID_DEVICE_TASK_INTERVAL_MS     8       // 8ms = ~125 FPS for ultra-smooth operation
#define WATCHDOG_TASK_INTERVAL_MS       100     // Watchdog update frequency
#define WATCHDOG_INIT_DELAY_MS          8       // HID device task frequency
#define VISUAL_TASK_INTERVAL_MS         50      // LED/neopixel update frequency
#define ERROR_CHECK_INTERVAL_MS         1000    // USB error check frequency
#define CORE1_HEARTBEAT_CHECK_LOOPS     10000   // Core1 heartbeat check frequency

// Performance tuning constants
#define MAIN_LOOP_TIME_SAMPLE_INTERVAL  32      // Sample time every N loops for performance
#define CORE1_HEARTBEAT_MULTIPLIER      4       // Reduce Core1 heartbeat frequency by this factor

// LED timing
#define LED_BLINK_MOUNTED_MS            250     // Fast blink when USB device mounted
#define LED_BLINK_UNMOUNTED_MS          1000    // Medium blink when USB device unmounted
#define LED_BLINK_SUSPENDED_MS          2500    // Slow blink when USB device suspended
#define LED_BLINK_RESUMED_MS            250     // Fast blink when USB device resumed
#define DEFAULT_BLINK_INTERVAL_MS       250     // Default LED blink interval

// Neopixel timing
#define STATUS_UPDATE_INTERVAL_MS       100     // Neopixel status update interval
#define BOOT_TIMEOUT_MS                 3000    // Boot status timeout
#define BREATHING_CYCLE_MS              2000    // Breathing effect cycle time
#define BREATHING_HALF_CYCLE_MS         (BREATHING_CYCLE_MS / 2)
#define POWER_STABILIZATION_DELAY_MS    10      // Power stabilization delay
#define ACTIVITY_FLASH_DURATION_MS      150     // Activity flash duration

// Reporting intervals
#define DEBUG_INTERVAL                  10000   // Print debug every 10000 reports
#define TASK_COUNT_REPORT_INTERVAL      500000  // Print status every N task iterations
#define WATCHDOG_STATUS_REPORT_INTERVAL_MS 60000 // Watchdog status reporting

//--------------------------------------------------------------------+
// WATCHDOG CONFIGURATION
//--------------------------------------------------------------------+

#define WATCHDOG_HEARTBEAT_INTERVAL_MS  1000    // Watchdog heartbeat interval
#define WATCHDOG_HARDWARE_TIMEOUT_MS    90000   // Hardware watchdog timeout
#define WATCHDOG_CORE_TIMEOUT_MS        30000   // Inter-core heartbeat timeout
#define WATCHDOG_UPDATE_INTERVAL_MS     5000    // How often to update hardware watchdog
#define WATCHDOG_ENABLE_HARDWARE        1       // Enable hardware watchdog
#define WATCHDOG_ENABLE_INTER_CORE      1       // Enable inter-core monitoring
#define WATCHDOG_ENABLE_DEBUG           0       // Disable debug output for cold boot

//--------------------------------------------------------------------+
// USB CONFIGURATION
//--------------------------------------------------------------------+

// USB descriptor constants
#define USB_BCD_VERSION                 0x0200  // USB 2.0
#define USB_VENDOR_ID                   0x9981  // Vendor ID
#define USB_PRODUCT_ID                  0x4001  // Product ID
#define USB_DEVICE_VERSION              0x0100  // Device version 1.0
#define USB_NUM_CONFIGURATIONS          0x01    // Number of configurations
#define USB_CONFIG_POWER_MA             100     // Power consumption in mA

// String descriptor indices
#define STRING_DESC_LANGUAGE_IDX        0x00    // Language descriptor index
#define STRING_DESC_MANUFACTURER_IDX    0x01    // Manufacturer string index
#define STRING_DESC_PRODUCT_IDX         0x02    // Product string index
#define STRING_DESC_SERIAL_IDX          0x03    // Serial number string index

// USB device class codes
#define USB_DEVICE_CLASS_NONE           0x00    // Device class: defined at interface level
#define USB_DEVICE_SUBCLASS_NONE        0x00    // Device subclass: none
#define USB_DEVICE_PROTOCOL_NONE        0x00    // Device protocol: none

// Configuration descriptor values
#define USB_CONFIG_INDEX                1       // Configuration index (1-based)
#define USB_INTERFACE_STRING_NONE       0       // No string descriptor for interface

// USB reset constants
#define USB_RESET_TIMEOUT_MS            5000    // Timeout for USB reset operations
#define USB_RESET_RETRY_DELAY_MS        100     // Delay between reset retry attempts
#define USB_RESET_MAX_RETRIES           3       // Maximum number of reset retries
#define USB_ERROR_CHECK_INTERVAL_MS     1000    // How often to check for USB errors
#define USB_STACK_ERROR_THRESHOLD       50      // Number of consecutive errors before reset

// USB descriptor configuration
#define CONFIG_TOTAL_LEN                (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define EPNUM_HID                       HID_ENDPOINT_ADDRESS

//--------------------------------------------------------------------+
// HID CONFIGURATION
//--------------------------------------------------------------------+

// HID endpoint configuration
#define HID_ENDPOINT_ADDRESS            0x81    // HID IN endpoint address (device mode)
#define HID_POLLING_INTERVAL_MS         1       // HID polling interval in ms

// USB endpoint allocation to prevent conflicts
// Device stack (controller 0) uses endpoints 0x00-0x8F
// Host stack (controller 1) has separate endpoint space
#define USB_DEVICE_CTRL_EP              0x00    // Device control endpoint
#define USB_HOST_CTRL_EP                0x00    // Host control endpoint (separate controller)

// HID report structure
#define HID_KEYBOARD_KEYCODE_COUNT      6       // Number of simultaneous keycodes supported
#define HID_CONSUMER_CONTROL_SIZE       2       // Consumer control report size in bytes

// Activity tracking
#define KEYBOARD_ACTIVITY_THROTTLE      50      // Trigger keyboard activity flash every 50 reports
#define MOUSE_ACTIVITY_THROTTLE         100     // Trigger mouse activity flash every 100 reports

//--------------------------------------------------------------------+
// MOUSE CONFIGURATION
//--------------------------------------------------------------------+

// Mouse coordinate bounds
#define MOUSE_COORD_MIN                 -127    // Minimum mouse coordinate value
#define MOUSE_COORD_MAX                 127     // Maximum mouse coordinate value
#define MOUSE_NO_MOVEMENT               0       // No mouse movement value
#define MOUSE_BUTTON_MOVEMENT_DELTA     -5      // Y-axis movement when button pressed

// Mouse button masks
#define MOUSE_BUTTON_NONE               0x00    // No mouse buttons pressed


// STRING DESCRIPTOR PROCESSING
//--------------------------------------------------------------------+

// Language codes
#define USB_LANGUAGE_ENGLISH_US_BYTE1   0x09    // English (US) language code byte 1
#define USB_LANGUAGE_ENGLISH_US_BYTE2   0x04    // English (US) language code byte 2

// String processing constants
#define MAX_STRING_DESCRIPTOR_CHARS     31      // Maximum characters in string descriptor
#define STRING_DESC_HEADER_SIZE         2       // String descriptor header size
#define STRING_DESC_TYPE_SHIFT          8       // Bit shift for string descriptor type
#define STRING_DESC_LENGTH_MULTIPLIER   2       // UTF-16 uses 2 bytes per character
#define STRING_DESC_FIRST_CHAR_OFFSET   1       // Offset to first character in descriptor
#define STRING_DESC_CHAR_COUNT_INIT     1       // Initial character count for language descriptor

//--------------------------------------------------------------------+
// NEOPIXEL CONFIGURATION
//--------------------------------------------------------------------+

// System status colors (RGB format)
#define COLOR_OFF                       0x000000
#define COLOR_BOOTING                   0x0000FF  // Blue
#define COLOR_USB_DEVICE_ONLY           0x00FF00  // Green
#define COLOR_USB_HOST_ONLY             0xFF8000  // Orange
#define COLOR_BOTH_ACTIVE               0x00FFFF  // Cyan
#define COLOR_MOUSE_CONNECTED           0xFF00FF  // Magenta
#define COLOR_KEYBOARD_CONNECTED        0xFFFF00  // Yellow
#define COLOR_BOTH_HID_CONNECTED        0xFF4080  // Pink
#define COLOR_ERROR                     0xFF0000  // Red
#define COLOR_SUSPENDED                 0x800080  // Purple
#define COLOR_CAPS_LOCK_ON              0xFFA500  // Orange flash
#define COLOR_USB_RESET_PENDING         0xFF6600  // Orange-red for USB reset pending
#define COLOR_USB_RESET_SUCCESS         0x00FF00  // Green flash for successful reset
#define COLOR_USB_RESET_FAILED          0xFF0000  // Red flash for failed reset

// Activity colors
#define COLOR_ACTIVITY_FLASH            0xFFFFFF  // White flash for activity
#define COLOR_MOUSE_ACTIVITY            0xFF00FF  // Magenta flash for mouse
#define COLOR_KEYBOARD_ACTIVITY         0xFFFF00  // Yellow flash for keyboard
#define COLOR_USB_CONNECTION            0x00FF80  // Bright green flash for USB connection
#define COLOR_USB_DISCONNECTION         0xFF8000  // Orange flash for USB disconnection

// Bridge Serial Connection Colors
#define COLOR_BRIDGE_WAITING            0x0080FF  // Light blue - waiting for connection
#define COLOR_BRIDGE_CONNECTING         0xFFFF00  // Yellow - handshake in progress
#define COLOR_BRIDGE_CONNECTED          0x00FF00  // Green - connected and ready
#define COLOR_BRIDGE_ACTIVE             0x00FFFF  // Cyan - actively receiving commands
#define COLOR_BRIDGE_DISCONNECTED       0xFF4000  // Orange-red - connection lost

// Humanization mode colors (for button mode switching)
#define COLOR_HUMANIZATION_OFF          0xFF0000  // Red - no humanization
#define COLOR_HUMANIZATION_LOW          0xFFFF00  // Yellow - minimal humanization
#define COLOR_HUMANIZATION_MEDIUM       0x00FF00  // Green - balanced (default)
#define COLOR_HUMANIZATION_HIGH         0x00FFFF  // Cyan - maximum humanization

// Brightness constants
#define MIN_BRIGHTNESS                  0.0f
#define MAX_BRIGHTNESS                  1.0f
#define BREATHING_MIN_BRIGHTNESS        0.2f
#define BREATHING_MAX_BRIGHTNESS        0.8f

// WS2812 configuration
#define WS2812_FREQUENCY_HZ             800000
#define WS2812_RGB_SHIFT                8u

// Rainbow movement configuration
// Degrees of hue change per unit of mouse movement (signed)
#define RAINBOW_MOVE_SCALE_DEG_PER_UNIT 2.0f
// Automatic slow rotation applied when idle (degrees per millisecond)
#define RAINBOW_AUTO_SPEED_DEG_PER_MS   0.12f

//--------------------------------------------------------------------+
// BUFFER AND ARRAY CONSTANTS
//--------------------------------------------------------------------+

#define MIN_BUFFER_SIZE                 1       // Minimum buffer size for HID reports
#define BUFFER_FIRST_ELEMENT_INDEX      0       // Index of first element in buffer/array
#define KEYCODE_ASCII_SHIFT_INDEX       1       // Index for shifted character in keycode2ascii array
#define KEYCODE_ASCII_NORMAL_INDEX      0       // Index for normal character in keycode2ascii array
#define LOOP_START_INDEX                0       // Standard loop start index
#define ARRAY_FIRST_INDEX               0       // First index in arrays

//--------------------------------------------------------------------+
// MANUFACTURER/PRODUCT STRINGS
//--------------------------------------------------------------------+

#define MANUFACTURER_STRING             "Hurricane"
#define PRODUCT_STRING                  "PIOKM Box"

//--------------------------------------------------------------------+
// BUILD CONFIGURATION
//--------------------------------------------------------------------+

// Build configuration presets
#define BUILD_CONFIG_DEVELOPMENT        1
#define BUILD_CONFIG_PRODUCTION         2
#define BUILD_CONFIG_TESTING            3
#define BUILD_CONFIG_DEBUG              4

#ifndef BUILD_CONFIG
#define BUILD_CONFIG                    BUILD_CONFIG_DEVELOPMENT
#endif

//--------------------------------------------------------------------+
// HARDWARE REVISION CONFIGURATION
//--------------------------------------------------------------------+

#ifndef HARDWARE_REVISION
#define HARDWARE_REVISION               1
#endif

#if HARDWARE_REVISION >= 2
#define REQUIRES_EXTENDED_BOOT_DELAY    0
#define USB_POWER_STABILIZATION_MS      500
#else
#define REQUIRES_EXTENDED_BOOT_DELAY    1
#define USB_POWER_STABILIZATION_MS      2000
#endif

//--------------------------------------------------------------------+
// FEATURE ENABLE/DISABLE FLAGS
//--------------------------------------------------------------------+

#ifndef ENABLE_HID_STATISTICS
#define ENABLE_HID_STATISTICS           1
#endif

#ifndef ENABLE_WATCHDOG_REPORTING
#define ENABLE_WATCHDOG_REPORTING       1
#endif

#ifndef ENABLE_NEOPIXEL_STATUS
#define ENABLE_NEOPIXEL_STATUS          1
#endif

#ifndef ENABLE_BUTTON_RESET
#define ENABLE_BUTTON_RESET             1
#endif

#define ENABLE_PERIODIC_REINIT          1
#define ENABLE_FALLBACK_MODE            1

//--------------------------------------------------------------------+
// LOGGING CONFIGURATION
//--------------------------------------------------------------------+

#if BUILD_CONFIG == BUILD_CONFIG_PRODUCTION
    #define ENABLE_VERBOSE_LOGGING      0
    #define ENABLE_INIT_LOGGING         0
    #define ENABLE_ERROR_LOGGING        0
    #define ENABLE_STATS_LOGGING        0
#elif BUILD_CONFIG == BUILD_CONFIG_TESTING
    #define ENABLE_VERBOSE_LOGGING      0
    #define ENABLE_INIT_LOGGING         0
    #define ENABLE_ERROR_LOGGING        0
    #define ENABLE_STATS_LOGGING        0
#else // Development and Debug
    #define ENABLE_VERBOSE_LOGGING      0
    #define ENABLE_INIT_LOGGING         0
    #define ENABLE_ERROR_LOGGING        0
    #define ENABLE_STATS_LOGGING        0
#endif

//--------------------------------------------------------------------+
// CONDITIONAL COMPILATION MACROS
//--------------------------------------------------------------------+

#if ENABLE_VERBOSE_LOGGING
    #define LOG_VERBOSE(fmt, ...) printf("[VERBOSE] " fmt "\n", ##__VA_ARGS__)
#else
    #define LOG_VERBOSE(fmt, ...) ((void)0)
#endif

#if ENABLE_INIT_LOGGING
    #define LOG_INIT(fmt, ...) printf("[INIT] " fmt "\n", ##__VA_ARGS__)
#else
    #define LOG_INIT(fmt, ...) ((void)0)
#endif

#if ENABLE_ERROR_LOGGING
    #define LOG_ERROR(fmt, ...) printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#else
    #define LOG_ERROR(fmt, ...) ((void)0)
#endif

#endif // DEFINES_H