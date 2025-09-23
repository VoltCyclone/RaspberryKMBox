/*
 * Consolidated Defines for PIOKMbox
 * 
 * This header consolidates and organizes all #define constants used throughout
 * the PIOKMbox project, eliminating duplicates and providing clear categorization.
 */

#ifndef DEFINES_H
#define DEFINES_H

//--------------------------------------------------------------------+
// HARDWARE CONFIGURATION
//--------------------------------------------------------------------+
#if PICO_PLATFORM == rp2040
#define DEFAULT_CPU_FREQ 240000
#elif PICO_PLATFORM == rp2350
#define DEFAULT_CPU_FREQ 120000
#endif

#ifndef CPU_FREQ
#define CPU_FREQ DEFAULT_CPU_FREQ
#endif

// Pin definitions
#ifndef PIN_USB_HOST_DP
#define PIN_USB_HOST_DP         (16u)   // PIO USB Host D+ pin
#endif
#ifndef PIN_USB_HOST_DM
#define PIN_USB_HOST_DM         (17u)   // PIO USB Host D- pin
#endif
#define PIN_BUTTON              (7u)    // Reset button pin
#define PIN_USB_5V              (18u)   // Power pin for USB host
#define PIN_LED                 (13u)   // Status LED pin
#define PIN_NEOPIXEL            (21u)   // Neopixel data pin
#define NEOPIXEL_POWER          (20u)   // Neopixel power pin

// UART configuration for KMBox serial input
#define KMBOX_UART              uart1    // Use UART1 for KMBox (UART0 is for debug)
#define KMBOX_UART_TX_PIN       (5u)     // GPIO5 for UART1 TX
#define KMBOX_UART_RX_PIN       (6u)     // GPIO6 for UART1 RX
#define KMBOX_UART_BAUDRATE     115200   // Standard baud rate for KMBox
#define KMBOX_UART_FIFO_SIZE    32       // UART FIFO size for buffering

// UART0 stdio (debug) configuration
#ifndef STDIO_UART_BAUDRATE
#define STDIO_UART_BAUDRATE     115200   // High baud for fast non-blocking debug prints
#endif

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

// HID report descriptor length is always patched at runtime to match the
// constructed runtime HID report descriptor which mirrors the attached host
// mouse. No compile-time static length is used.

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

//--------------------------------------------------------------------+
// SERIAL STRING CONFIGURATION
//--------------------------------------------------------------------+

#define SERIAL_STRING_LENGTH            16      // 16 hex characters for unique ID
#define SERIAL_STRING_BUFFER_SIZE       17      // 16 chars + null terminator
#define SERIAL_HEX_CHARS_PER_BYTE       2       // 2 hex characters per byte
#define SERIAL_SNPRINTF_BUFFER_SIZE     3       // Buffer size for snprintf formatting

//--------------------------------------------------------------------+
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
    #define ENABLE_INIT_LOGGING         1
    #define ENABLE_ERROR_LOGGING        1
    #define ENABLE_STATS_LOGGING        1
#elif BUILD_CONFIG == BUILD_CONFIG_TESTING
    #define ENABLE_VERBOSE_LOGGING      1
    #define ENABLE_INIT_LOGGING         1
    #define ENABLE_ERROR_LOGGING        1
    #define ENABLE_STATS_LOGGING        0
#else // Development and Debug
    #define ENABLE_VERBOSE_LOGGING      1
    #define ENABLE_INIT_LOGGING         1
    #define ENABLE_ERROR_LOGGING        1
    #define ENABLE_STATS_LOGGING        1
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

#endif // DEFINES_CONSOLIDATED_H