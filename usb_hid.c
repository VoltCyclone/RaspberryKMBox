/*
 * Hurricane PIOKMBox Firmware
 */

#include "usb_hid.h"
#include "defines.h"
#include "led_control.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "hardware/timer.h"     // For time_us_32() in hid_device_task
#include "kmbox_serial_handler.h" // Include the header for serial handling
#include "state_management.h"   // Include the header for state management
#include "watchdog.h"           // Include the header for watchdog management
#include "smooth_injection.h"   // Include the header for smooth mouse injection
#include "humanization_fpu.h"   // Include for tremor generation
#include <string.h>             // For strcpy, strlen, memset
#include <math.h>                // For sqrtf

uint16_t attached_vid = 0;
uint16_t attached_pid = 0;
bool attached_has_serial = false;

// Dynamic string descriptor storage
static char attached_manufacturer[64] = "";
static char attached_product[64] = "";
static char attached_serial[32] = "";
static bool string_descriptors_fetched = false;

#define LANGUAGE_ID 0x0409  // English (US)

// UTF-16LE string descriptor -> UTF-8 (ASCII-only) conversion.
// Uses the descriptor's bLength to avoid reading past valid data.
static void utf16_to_utf8(uint16_t *utf16_buf, size_t utf16_buf_bytes, char *utf8_buf, size_t utf8_len)
{
    if (!utf16_buf || !utf8_buf || utf8_len == 0)
        return;

    // String descriptor format: [bLength (1B)][bDescriptorType (1B)][UTF-16LE code units...]
    // Determine actual descriptor length from first byte
    const uint8_t *raw = (const uint8_t *)utf16_buf;

    // Cap length by provided buffer size to be safe
    uint8_t bLength = raw[0];
    if (bLength > utf16_buf_bytes)
    {
        bLength = (uint8_t)utf16_buf_bytes;
    }

    // Compute number of 16-bit code units
    size_t code_units = 0;
    if (bLength >= 2)
    {
        code_units = (size_t)(bLength - 2) / 2;
    }

    size_t utf8_pos = 0;
    
    // Unroll loop by 4 for better performance (most strings are short)
    size_t i = 0;
    size_t utf16_buf_len = utf16_buf_bytes / 2;  // Total UTF-16 code units available
    while (i + 3 < code_units && utf8_pos + 3 < utf8_len - 1 && (1 + i + 3) < utf16_buf_len) {
        uint16_t u0 = utf16_buf[1 + i];
        uint16_t u1 = utf16_buf[1 + i + 1];
        uint16_t u2 = utf16_buf[1 + i + 2];
        uint16_t u3 = utf16_buf[1 + i + 3];
        
        // Early exit on NUL
        if (u0 == 0) break;
        utf8_buf[utf8_pos++] = (u0 <= 0x7F) ? (char)u0 : '?';
        
        if (u1 == 0) break;
        utf8_buf[utf8_pos++] = (u1 <= 0x7F) ? (char)u1 : '?';
        
        if (u2 == 0) break;
        utf8_buf[utf8_pos++] = (u2 <= 0x7F) ? (char)u2 : '?';
        
        if (u3 == 0) break;
        utf8_buf[utf8_pos++] = (u3 <= 0x7F) ? (char)u3 : '?';
        
        i += 4;
    }
    
    // Handle remaining code units
    for (; i < code_units && utf8_pos < utf8_len - 1; i++)
    {
        uint16_t u = utf16_buf[1 + i];
        if (u == 0) break;
        utf8_buf[utf8_pos++] = (u <= 0x7F) ? (char)u : '?';
    }

    utf8_buf[utf8_pos] = '\0';
}

// Function to set the VID and PID of the attached device
void set_attached_device_vid_pid(uint16_t vid, uint16_t pid) {
    // Only update and re-enumerate if VID/PID has actually changed
    if (attached_vid != vid || attached_pid != pid) {
        attached_vid = vid;
        attached_pid = pid;
        attached_has_serial = false; // Default to no serial number unless device has one
        
        // Force USB re-enumeration to update descriptor
        force_usb_reenumeration();
    }
}

void force_usb_reenumeration() {
    // Disconnect from USB host
    tud_disconnect();
    
    // Wait for host to recognize disconnection (500ms for Windows/macOS)
    // Feed watchdog during long wait to prevent reset
    for (int i = 0; i < 50; i++) {
        sleep_ms(10);
        watchdog_core0_heartbeat();
    }
    
    // Reconnect with new descriptor
    tud_connect();
    
    // Wait for reconnection (250ms for stability)
    // Feed watchdog during wait
    for (int i = 0; i < 25; i++) {
        sleep_ms(10);
        watchdog_core0_heartbeat();
    }
}

//--------------------------------------------------------------------+
// USB Descriptor Cloning Infrastructure
//--------------------------------------------------------------------+

// Forward declarations
static void build_runtime_hid_report_with_mouse(const uint8_t *mouse_desc, size_t mouse_len);
static void rebuild_configuration_descriptor(void);
static void parse_host_config_descriptor(const uint8_t *cfg_desc, uint16_t cfg_len);
static void reset_device_string_descriptors(void);  // Defined after all global state

// Captured host device descriptor fields for cloning
static struct {
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    bool     valid;
} host_device_info = { .bcdUSB = 0x0200, .bMaxPacketSize0 = 64, .bcdDevice = 0x0100, .valid = false };

// Captured host config descriptor fields for cloning
static struct {
    uint8_t  bmAttributes;     // Self-powered, remote wakeup
    uint8_t  bMaxPower;        // In 2mA units
    uint8_t  bInterfaceProtocol; // Boot mouse protocol etc
    uint8_t  bInterfaceSubClass;
    uint16_t wMaxPacketSize;   // Endpoint max packet size
    uint8_t  bInterval;        // Polling interval
    bool     valid;
} host_config_info = { .bmAttributes = TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, .bMaxPower = USB_CONFIG_POWER_MA / 2, .bInterfaceProtocol = HID_ITF_PROTOCOL_NONE, .bInterfaceSubClass = 0, .wMaxPacketSize = CFG_TUD_HID_EP_BUFSIZE, .bInterval = HID_POLLING_INTERVAL_MS, .valid = false };

// Runtime configuration descriptor buffer (mutable so we can patch it)
static uint8_t desc_configuration_runtime[TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN];
static bool desc_config_runtime_valid = false;

// Function to fetch string descriptors from attached device
static void fetch_device_string_descriptors(uint8_t dev_addr) {
    // Reset string descriptors
    memset(attached_manufacturer, 0, sizeof(attached_manufacturer));
    memset(attached_product, 0, sizeof(attached_product));
    memset(attached_serial, 0, sizeof(attached_serial));
    string_descriptors_fetched = false;
    
    // Temporary buffers for UTF-16 strings
    uint16_t temp_manufacturer[32];
    uint16_t temp_product[48];
    uint16_t temp_serial[16];

    // Ensure buffers are zeroed to avoid stray data being interpreted
    memset(temp_manufacturer, 0, sizeof(temp_manufacturer));
    memset(temp_product, 0, sizeof(temp_product));
    memset(temp_serial, 0, sizeof(temp_serial));
    
    // Get manufacturer string
    if (tuh_descriptor_get_manufacturer_string_sync(dev_addr, LANGUAGE_ID, temp_manufacturer, sizeof(temp_manufacturer)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_manufacturer, sizeof(temp_manufacturer), attached_manufacturer, sizeof(attached_manufacturer));
        kmbox_send_status(attached_manufacturer);
    } else {
        strcpy(attached_manufacturer, MANUFACTURER_STRING);  // Fallback
    }
    
    // Get product string
    if (tuh_descriptor_get_product_string_sync(dev_addr, LANGUAGE_ID, temp_product, sizeof(temp_product)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_product, sizeof(temp_product), attached_product, sizeof(attached_product));
        kmbox_send_status(attached_product);
    } else {
        strcpy(attached_product, PRODUCT_STRING);  // Fallback
    }
    
    // Get serial string (optional)
    if (tuh_descriptor_get_serial_string_sync(dev_addr, LANGUAGE_ID, temp_serial, sizeof(temp_serial)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_serial, sizeof(temp_serial), attached_serial, sizeof(attached_serial));
        char serial_msg[64];
        snprintf(serial_msg, sizeof(serial_msg), "Serial: %s", attached_serial);
        kmbox_send_status(serial_msg);
        attached_has_serial = (strlen(attached_serial) > 0);
    } else {
        attached_has_serial = false;
    }
    
    string_descriptors_fetched = true;
}

// reset_device_string_descriptors() — defined after all global state declarations
// (see forward declaration above)

// Function to get the VID of the attached device
uint16_t get_attached_vid(void) {
    return attached_vid;
}

// Function to get the PID of the attached device
uint16_t get_attached_pid(void) {
    return attached_pid;
}

// Function to get attached device manufacturer string
const char* get_attached_manufacturer(void) {
    return attached_manufacturer;
}

// Function to get attached device product string
const char* get_attached_product(void) {
    return attached_product;
}

// Function to get dynamic serial string
const char* get_dynamic_serial_string() {
    static char dynamic_serial[64];
    if (attached_vid && attached_pid) {
        snprintf(dynamic_serial, sizeof(dynamic_serial), "PIOKMbox_%04X_%04X", attached_vid, attached_pid);
        return dynamic_serial;
    }
    return "PIOKMbox_v1.0";
}

// Error tracking structure with better organization
typedef struct
{
    uint32_t device_errors;
    uint32_t host_errors;
    uint32_t consecutive_device_errors;
    uint32_t consecutive_host_errors;
    uint32_t last_error_check_time;
    bool device_error_state;
    bool host_error_state;
} usb_error_tracker_t;

// Device connection state with better encapsulation
typedef struct
{
    bool mouse_connected;
    bool keyboard_connected;
    uint8_t mouse_dev_addr;
    uint8_t keyboard_dev_addr;
} device_connection_state_t;

// Device mode state
static bool caps_lock_state = false;

static device_connection_state_t connection_state = {0};

// Error tracking - now properly typed
static usb_error_tracker_t usb_error_tracker = {0};

// USB stack initialization tracking
static bool usb_device_initialized = false;
static bool usb_host_initialized = false;

// Track the last button byte sent to the host across ALL report paths
// (physical mouse forwarding, kmbox injection, smooth injection).
// Used by hid_device_task() to detect button-only changes that need
// an immediate report — a real mouse sends button changes on the very
// next poll.
static volatile uint8_t last_sent_buttons = 0;
// Track whether the previous cycle had activity, so we can send one
// final zero-delta "stop" report on the active→idle edge.
static volatile bool was_active = false;

// Device management helpers
static void handle_device_disconnection(uint8_t dev_addr);
static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol);

// Report processing helpers
static bool process_keyboard_report_internal(const hid_keyboard_report_t *report);
static bool process_mouse_report_internal(const hid_mouse_report_t *report);

// Humanization helpers
static void apply_output_humanization(int16_t *x, int16_t *y, int16_t injected_x, int16_t injected_y);
static inline int8_t clamp_i8(int32_t val);

// --- Runtime HID descriptor mirroring storage & helpers ---
// Gaming mice (Razer, Logitech, SteelSeries) can have very large HID
// report descriptors — 500-1000+ bytes with multiple collections for
// mouse, keyboard macros, and vendor-specific features.
#define HID_DESC_BUF_SIZE 1024

static const uint8_t desc_hid_keyboard[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD))};

// 16-bit mouse descriptor for gaming mice (G703, G Pro Wireless, etc.)
// Matches the G703 Lightspeed structure: 16-bit buttons, 16-bit X/Y, 8-bit wheel/pan
static const uint8_t desc_hid_mouse_16bit[] = {
    HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP      )                   ,
    HID_USAGE      ( HID_USAGE_DESKTOP_MOUSE     )                   ,
    HID_COLLECTION ( HID_COLLECTION_APPLICATION  )                   ,
      HID_REPORT_ID( REPORT_ID_MOUSE )
      HID_USAGE      ( HID_USAGE_DESKTOP_POINTER )                   ,
      HID_COLLECTION ( HID_COLLECTION_PHYSICAL   )                   ,
        // Buttons: 16 bits (to match high-end gaming mice)
        HID_USAGE_PAGE  ( HID_USAGE_PAGE_BUTTON  )                   ,
        HID_USAGE_MIN   ( 1                      )                   ,
        HID_USAGE_MAX   ( 16                     )                   ,
        HID_LOGICAL_MIN ( 0                      )                   ,
        HID_LOGICAL_MAX ( 1                      )                   ,
        HID_REPORT_COUNT( 16                     )                   ,
        HID_REPORT_SIZE ( 1                      )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE )   ,
        // X, Y: 16-bit relative (-32767 to +32767)
        HID_USAGE_PAGE  ( HID_USAGE_PAGE_DESKTOP )                   ,
        HID_USAGE       ( HID_USAGE_DESKTOP_X    )                   ,
        HID_USAGE       ( HID_USAGE_DESKTOP_Y    )                   ,
        HID_LOGICAL_MIN_N ( -32767, 2            )                   ,
        HID_LOGICAL_MAX_N ( 32767, 2             )                   ,
        HID_REPORT_COUNT( 2                      )                   ,
        HID_REPORT_SIZE ( 16                     )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )   ,
        // Wheel: 8-bit relative
        HID_USAGE       ( HID_USAGE_DESKTOP_WHEEL )                  ,
        HID_LOGICAL_MIN ( 0x81                   )                   ,
        HID_LOGICAL_MAX ( 0x7F                   )                   ,
        HID_REPORT_COUNT( 1                      )                   ,
        HID_REPORT_SIZE ( 8                      )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )   ,
        // AC Pan: 8-bit relative (horizontal scroll)
        HID_USAGE_PAGE  ( HID_USAGE_PAGE_CONSUMER )                  ,
        HID_USAGE_N     ( HID_USAGE_CONSUMER_AC_PAN, 2 )             ,
        HID_LOGICAL_MIN ( 0x81                   )                   ,
        HID_LOGICAL_MAX ( 0x7F                   )                   ,
        HID_REPORT_COUNT( 1                      )                   ,
        HID_REPORT_SIZE ( 8                      )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )   ,
      HID_COLLECTION_END                                             ,
    HID_COLLECTION_END
};

static const uint8_t desc_hid_consumer[] = {
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(REPORT_ID_CONSUMER_CONTROL))};

// Static fallback concatenated descriptor (used by config descriptor sizeof)
const uint8_t desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE)),
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(REPORT_ID_CONSUMER_CONTROL))};

static uint8_t desc_hid_report_runtime[HID_DESC_BUF_SIZE];
static size_t desc_hid_runtime_len = 0;
static bool desc_hid_runtime_valid = false;
static bool using_16bit_output_override = false;  // True when we override to 16-bit descriptor

static uint8_t host_mouse_desc[HID_DESC_BUF_SIZE];
static size_t host_mouse_desc_len = 0;
static bool host_mouse_has_report_id = false;
static uint8_t host_mouse_report_id = 0;

// --- Raw report forwarding for gaming mice ---
// When we clone the host mouse's HID report descriptor, we must send reports
// in the exact same binary format.  We parse the host descriptor to discover
// the byte layout (offsets & widths of buttons, X, Y, wheel) and then forward
// incoming reports with kmbox/smooth deltas injected in-place.
//
// Layout populated by parse_mouse_report_layout() during tuh_hid_mount_cb.
typedef struct {
    // Total expected report size (excluding report-ID prefix byte)
    uint8_t  report_size;

    // Button byte
    uint8_t  buttons_offset;
    uint8_t  buttons_bits;     // typically 5 or 8

    // X axis
    uint8_t  x_offset;
    bool     x_is_16bit;
    uint16_t x_bit_offset;
    uint8_t  x_bits;
    uint8_t  x_start_byte;
    uint8_t  x_bit_in_byte;

    // Y axis
    uint8_t  y_offset;
    bool     y_is_16bit;
    uint16_t y_bit_offset;
    uint8_t  y_bits;
    uint8_t  y_start_byte;
    uint8_t  y_bit_in_byte;

    // Wheel (vertical scroll)
    uint8_t  wheel_offset;
    bool     has_wheel;

    // Horizontal scroll / pan
    uint8_t  pan_offset;
    bool     has_pan;

    // Report ID for the mouse collection (0 = no report IDs in descriptor)
    uint8_t  mouse_report_id;
    bool     has_report_id;

    bool     valid;            // true once successfully parsed
} mouse_report_layout_t;

static mouse_report_layout_t host_mouse_layout = { .valid = false };

static mouse_report_layout_t output_mouse_layout_16bit = {
    .report_size = 8,
    .buttons_offset = 0,
    .buttons_bits = 16,
    .x_offset = 2,
    .x_is_16bit = true,
    .x_bit_offset = 16,
    .x_bits = 16,
    .x_start_byte = 2,
    .x_bit_in_byte = 0,
    .y_offset = 4,
    .y_is_16bit = true,
    .y_bit_offset = 32,
    .y_bits = 16,
    .y_start_byte = 4,
    .y_bit_in_byte = 0,
    .wheel_offset = 6,
    .has_wheel = true,
    .pan_offset = 7,
    .has_pan = true,
    .mouse_report_id = REPORT_ID_MOUSE,
    .has_report_id = true,
    .valid = true
};

// Track which dev_addr we've already cloned device/config descriptors for,
// so we only do it once for multi-interface composite devices (e.g. Razer
// Basilisk V3 has 4 HID interfaces, each triggers tuh_hid_mount_cb).
static uint8_t cloned_dev_addr = 0;

// Runtime report IDs — may be remapped to avoid conflicts with host mouse descriptor
static uint8_t runtime_kbd_report_id = REPORT_ID_KEYBOARD;
static uint8_t runtime_consumer_report_id = REPORT_ID_CONSUMER_CONTROL;

// Function to reset string descriptors and cloned state when device is disconnected
static void reset_device_string_descriptors(void) {
    memset(attached_manufacturer, 0, sizeof(attached_manufacturer));
    memset(attached_product, 0, sizeof(attached_product));
    memset(attached_serial, 0, sizeof(attached_serial));
    string_descriptors_fetched = false;
    attached_has_serial = false;
    
    // Reset cloned descriptor state
    host_device_info.valid = false;
    host_config_info.valid = false;
    host_mouse_layout.valid = false;
    host_mouse_desc_len = 0;
    host_mouse_has_report_id = false;
    host_mouse_report_id = 0;
    cloned_dev_addr = 0;
    
    // Reset runtime report IDs to defaults
    runtime_kbd_report_id = REPORT_ID_KEYBOARD;
    runtime_consumer_report_id = REPORT_ID_CONSUMER_CONTROL;
    
    // Rebuild config descriptor with defaults
    build_runtime_hid_report_with_mouse(NULL, 0);
    rebuild_configuration_descriptor();
}

//--------------------------------------------------------------------+
// Inline Helper Functions
//--------------------------------------------------------------------+

static __force_inline int8_t clamp_i8(int32_t val) {
    if (val > 127) return 127;
    if (val < -128) return -128;
    return (int8_t)val;
}

//--------------------------------------------------------------------+
// Final Stage Humanization (Applied proportionally to injected movement)
//--------------------------------------------------------------------+

/**
 * Apply humanization tremor to final HID output movement.
 * 
 * KEY DESIGN: Human mouse movement is already human — it doesn't need
 * additional tremor/jitter. Tremor is only applied proportionally to the
 * synthetic (injected) fraction of the total movement. This keeps the
 * mouse feeling natural and responsive during normal use while still
 * humanizing injected/bot movement.
 * 
 * Blend logic:
 *   - Pure physical movement (injected == 0): no tremor applied
 *   - Mixed (physical + injected): tremor scaled by injected fraction
 *   - Pure injected (no physical): full tremor applied
 * 
 * @param x Pointer to total X movement (modified in place)
 * @param y Pointer to total Y movement (modified in place)
 * @param injected_x The injected/synthetic X component (smooth queue + kmbox serial)
 * @param injected_y The injected/synthetic Y component (smooth queue + kmbox serial)
 */
static void apply_output_humanization(int16_t *x, int16_t *y, int16_t injected_x, int16_t injected_y) {
    // Skip if humanization is completely disabled
    humanization_mode_t mode = smooth_get_humanization_mode();
    if (mode == HUMANIZATION_OFF) {
        return;
    }
    
    // Get humanization parameters from smooth injection state
    int32_t jitter_amount_fp;
    bool jitter_enabled;
    smooth_get_humanization_params(&jitter_amount_fp, &jitter_enabled);
    
    if (!jitter_enabled) {
        return;
    }
    
    // --- Calculate blend ratio: how much of this movement is synthetic? ---
    // If there's no injected component, the user is just moving their mouse.
    // Human movement is already human — don't add tremor to it.
    float inject_mag = sqrtf((float)injected_x * injected_x + (float)injected_y * injected_y);
    
    // No injected movement — nothing to humanize. Do NOT apply tremor.
    // This prevents phantom tremor after injection queue drains (the "shaking" bug).
    if (inject_mag < 0.5f) {
        return;
    }
    
    float total_mag  = sqrtf((float)(*x) * (*x) + (float)(*y) * (*y));
    
    // No movement at all — skip (don't generate idle tremor on pure physical idle)
    if (total_mag < 0.5f && inject_mag < 0.5f) {
        return;
    }
    
    // Blend ratio: 0.0 = pure physical, 1.0 = pure injected
    float blend;
    if (total_mag < 0.5f) {
        // Total is ~zero but inject is non-zero (rare edge: physical cancelled inject)
        blend = 1.0f;
    } else {
        blend = inject_mag / total_mag;
        // Clamp to [0, 1] — inject_mag can exceed total_mag if they oppose
        if (blend > 1.0f) blend = 1.0f;
    }
    
    // If movement is almost entirely physical, skip tremor entirely
    // This threshold avoids wasting FPU cycles for negligible tremor
    if (blend < 0.05f) {
        return;
    }
    
    // --- Calculate tremor magnitude ---
    float magnitude = total_mag;
    
    // Mode-dependent intensity scaling
    float mode_scale = 1.0f;
    switch (mode) {
        case HUMANIZATION_MICRO:
            mode_scale = 0.5f;
            break;
        case HUMANIZATION_FULL:
            mode_scale = 1.0f;
            break;
        default:
            break;
    }
    
    // Calculate tremor scale with blend factor
    float movement_scale = humanization_jitter_scale(magnitude);
    float base_jitter = (float)jitter_amount_fp / 65536.0f;  // Convert from 16.16 fixed-point
    float tremor_scale = base_jitter * movement_scale * mode_scale * blend;
    
    // Get runtime tremor (layered oscillators + noise)
    float tremor_x, tremor_y;
    humanization_get_tremor(tremor_scale, &tremor_x, &tremor_y);
    
    if (magnitude > 2.0f) {
        // Moving cursor: perpendicular + parallel decomposition
        float fx = (float)(*x);
        float fy = (float)(*y);
        float norm_x = fx / magnitude;
        float norm_y = fy / magnitude;
        
        // Perpendicular component (tremor_y) - primary humanization signal
        float perp_dx = -norm_y * tremor_y;
        float perp_dy = norm_x * tremor_y;
        
        // Parallel component (tremor_x) - speed variation (smaller)
        float para_dx = norm_x * tremor_x * 0.3f;
        float para_dy = norm_y * tremor_x * 0.3f;
        
        // Apply tremor to output (with rounding)
        *x = (int16_t)(*x + (int16_t)(perp_dx + para_dx + 0.5f));
        *y = (int16_t)(*y + (int16_t)(perp_dy + para_dy + 0.5f));
    } else {
        // Small/idle movement with injection active: apply tremor as raw X/Y
        *x += (int16_t)(tremor_x + 0.5f);
        *y += (int16_t)(tremor_y + 0.5f);
    }
}

// Minimal HID descriptor parser — extracts report field layout for the mouse
// collection so we know where to inject deltas in raw reports.
// This is intentionally simple and handles the common gaming mouse patterns:
//   buttons (1-3 bytes), X (8/12/16 bit), Y (8/12/16 bit), wheel, pan.
static void parse_mouse_report_layout(const uint8_t *desc, size_t len,
                                       mouse_report_layout_t *layout)
{
    memset(layout, 0, sizeof(*layout));
    layout->valid = false;

    if (!desc || len < 4) return;

    // HID descriptor state machine — minimal implementation
    bool in_mouse_collection = false;
    uint16_t usage_page = 0;  // 16-bit: vendor pages (0xFF00) truncate in uint8_t
    uint8_t usage = 0;
    uint32_t bit_offset = 0;        // current bit position in the report
    uint32_t mouse_bit_max = 0;     // track highest bit offset within mouse collection
    uint8_t report_size_bits = 0;   // current REPORT_SIZE
    uint8_t report_count = 0;       // current REPORT_COUNT
    int collection_depth = 0;
    int mouse_collection_depth = -1;
    uint8_t current_report_id = 0;  // current Report ID context
    uint8_t mouse_report_id = 0;    // Report ID that contains the mouse collection
    bool found_mouse_report_id = false;

    // Track what usages we've seen before each INPUT item
    #define MAX_USAGES 16
    uint8_t usage_stack[MAX_USAGES];
    uint8_t usage_stack_count = 0;
    bool has_usage_range = false;
    uint8_t usage_min = 0;

    size_t i = 0;
    while (i < len) {
        uint8_t item = desc[i];
        uint8_t item_size = item & 0x03;
        if (item_size == 3) item_size = 4; // size=3 means 4 bytes

        if (i + 1 + item_size > len) break;

        uint32_t value = 0;
        for (uint8_t b = 0; b < item_size; b++) {
            value |= (uint32_t)desc[i + 1 + b] << (b * 8);
        }

        uint8_t item_tag = item & 0xFC; // tag + type

        switch (item_tag) {
            case 0x04: // Usage Page (Global)
                usage_page = (uint16_t)value;
                break;
            case 0x08: // Usage (Local)
                if (usage_stack_count < MAX_USAGES) {
                    usage_stack[usage_stack_count++] = (uint8_t)value;
                }
                usage = (uint8_t)value;
                break;
            case 0x18: // Usage Minimum (Local)
                has_usage_range = true;
                usage_min = (uint8_t)value;
                break;
            case 0x28: // Usage Maximum (Local)
                break;
            case 0xA0: // Collection
                collection_depth++;
                if (usage_page == HID_USAGE_PAGE_DESKTOP && usage == HID_USAGE_DESKTOP_MOUSE) {
                    in_mouse_collection = true;
                    mouse_collection_depth = collection_depth;
                    // Record which Report ID contains the mouse collection
                    mouse_report_id = current_report_id;
                    found_mouse_report_id = (current_report_id != 0);
                }
                break;
            case 0xC0: // End Collection
                if (in_mouse_collection && collection_depth == mouse_collection_depth) {
                    // Exiting the mouse collection — record max bit offset for size calc
                    if (bit_offset > mouse_bit_max) {
                        mouse_bit_max = bit_offset;
                    }
                    in_mouse_collection = false;
                    mouse_collection_depth = -1;
                }
                collection_depth--;
                break;
            case 0x74: // Report Size (Global)
                report_size_bits = (uint8_t)value;
                break;
            case 0x94: // Report Count (Global)
                report_count = (uint8_t)value;
                break;
            case 0x80: // Input (Main)
            {
                if (in_mouse_collection) {
                    bool is_constant = (value & 0x01); // bit 0: constant vs data
                    uint32_t total_bits = (uint32_t)report_size_bits * report_count;

                    if (!is_constant) {
                        // Determine what this input field is based on usage context
                        bool is_desktop_range = (has_usage_range && usage_page == HID_USAGE_PAGE_DESKTOP);
                        
                        if (usage_page == HID_USAGE_PAGE_BUTTON || (has_usage_range && !is_desktop_range)) {
                            // Button field
                            layout->buttons_offset = bit_offset / 8;
                            layout->buttons_bits = report_count;
                        } else if (usage_page == HID_USAGE_PAGE_DESKTOP) {
                            // Process usages (explicit stack or range-based)
                            for (uint8_t u = 0; u < report_count; u++) {
                                uint8_t cur_usage = 0;
                                
                                if (has_usage_range) {
                                    cur_usage = usage_min + u;
                                } else if (u < usage_stack_count) {
                                    cur_usage = usage_stack[u];
                                } else {
                                    // No more usages in stack vs report count
                                    break;
                                }

                                uint32_t field_bit_offset = bit_offset + (u * report_size_bits);
                                uint8_t byte_off = field_bit_offset / 8;

                                if (cur_usage == HID_USAGE_DESKTOP_X) {
                                    layout->x_offset = byte_off;
                                    layout->x_is_16bit = (report_size_bits >= 16);
                                    layout->x_bit_offset = (uint16_t)field_bit_offset;
                                    layout->x_bits = report_size_bits;
                                    layout->x_start_byte = byte_off;
                                    layout->x_bit_in_byte = field_bit_offset % 8;
                                } else if (cur_usage == HID_USAGE_DESKTOP_Y) {
                                    layout->y_offset = byte_off;
                                    layout->y_is_16bit = (report_size_bits >= 16);
                                    layout->y_bit_offset = (uint16_t)field_bit_offset;
                                    layout->y_bits = report_size_bits;
                                    layout->y_start_byte = byte_off;
                                    layout->y_bit_in_byte = field_bit_offset % 8;
                                } else if (cur_usage == HID_USAGE_DESKTOP_WHEEL) {
                                    layout->wheel_offset = byte_off;
                                    layout->has_wheel = true;
                                }
                            }
                            
                            if (!has_usage_range && usage_stack_count <= 1 && usage == HID_USAGE_DESKTOP_X && report_count >= 2) {
                                uint8_t x_byte_off = bit_offset / 8;
                                uint8_t y_byte_off = (bit_offset + report_size_bits) / 8;
                                layout->x_offset = x_byte_off;
                                layout->x_is_16bit = (report_size_bits >= 16);
                                layout->x_bit_offset = (uint16_t)bit_offset;
                                layout->x_bits = report_size_bits;
                                layout->x_start_byte = x_byte_off;
                                layout->x_bit_in_byte = bit_offset % 8;
                                layout->y_offset = y_byte_off;
                                layout->y_is_16bit = (report_size_bits >= 16);
                                layout->y_bit_offset = (uint16_t)(bit_offset + report_size_bits);
                                layout->y_bits = report_size_bits;
                                layout->y_start_byte = y_byte_off;
                                layout->y_bit_in_byte = (bit_offset + report_size_bits) % 8;
                            }
                        } else if (usage_page == HID_USAGE_PAGE_CONSUMER) {
                            // AC Pan (horizontal scroll)
                            if (usage_stack_count > 0 && usage_stack[0] == 0x38) { // AC Pan = 0x238
                                layout->pan_offset = bit_offset / 8;
                                layout->has_pan = true;
                            }
                        }
                    }

                    bit_offset += total_bits;
                }

                // Clear local state after Main item
                usage_stack_count = 0;
                has_usage_range = false;
                break;
            }
            case 0x84: // Report ID (Global)
                // Record the current Report ID context before processing
                // If we were in the mouse collection, save its final bit offset
                if (in_mouse_collection && bit_offset > mouse_bit_max) {
                    mouse_bit_max = bit_offset;
                }
                current_report_id = (uint8_t)value;
                // If we're already inside the mouse collection, update the
                // mouse report ID.  Some receivers (e.g. Logitech LIGHTSPEED)
                // place the Report ID *after* the Collection(Application) tag
                // for the mouse, so the initial value captured at collection
                // open time may be stale (e.g. keyboard's Report ID 1).
                if (in_mouse_collection) {
                    mouse_report_id = current_report_id;
                    found_mouse_report_id = (current_report_id != 0);
                }
                // Reset bit offset for new report ID section
                // (report ID byte is NOT counted in the bit offset of field data)
                bit_offset = 0;
                break;
        }

        i += 1 + item_size;
    }

    // Use mouse-specific bit count for report size, not the final global bit_offset
    // which may include data from non-mouse report IDs after the mouse collection
    if (in_mouse_collection && bit_offset > mouse_bit_max) {
        mouse_bit_max = bit_offset; // Still in mouse collection at end of descriptor
    }
    layout->report_size = (mouse_bit_max + 7) / 8;
    
    // Store the discovered report ID
    layout->mouse_report_id = mouse_report_id;
    layout->has_report_id = found_mouse_report_id;
    
    // A valid mouse report MUST have at least X and Y axes defined.
    // Otherwise we risk using a broken layout (offsets=0) and reading garbage.
    if (layout->report_size < 3 || layout->x_bits == 0 || layout->y_bits == 0) {
        // Invalid or incomplete layout - better to fallback to legacy mode
        layout->valid = false;
        return;
    }
    layout->valid = true;
}

// Per-instance HID usage tracking for non-boot-protocol devices (e.g. Logitech receivers)
// These devices report HID_ITF_PROTOCOL_NONE but we can detect mouse/keyboard
// usage from parsing the HID report descriptor.
#define MAX_HID_INSTANCES 16  // CFG_TUH_HID * CFG_TUH_DEVICE_MAX
typedef struct {
    uint8_t dev_addr;
    uint8_t instance;
    uint8_t effective_protocol;  // HID_ITF_PROTOCOL_MOUSE/KEYBOARD/NONE
    bool    has_report_id;
    uint8_t mouse_report_id;     // Report ID for mouse reports (0 if no report IDs)
    uint8_t keyboard_report_id;  // Report ID for keyboard reports
    bool    active;
} hid_instance_info_t;

static hid_instance_info_t hid_instances[MAX_HID_INSTANCES];

static hid_instance_info_t* find_hid_instance(uint8_t dev_addr, uint8_t instance) {
    for (int i = 0; i < MAX_HID_INSTANCES; i++) {
        if (hid_instances[i].active && 
            hid_instances[i].dev_addr == dev_addr && 
            hid_instances[i].instance == instance) {
            return &hid_instances[i];
        }
    }
    return NULL;
}

static hid_instance_info_t* alloc_hid_instance(uint8_t dev_addr, uint8_t instance) {
    // First check if already exists
    hid_instance_info_t* existing = find_hid_instance(dev_addr, instance);
    if (existing) return existing;
    
    // Find empty slot
    for (int i = 0; i < MAX_HID_INSTANCES; i++) {
        if (!hid_instances[i].active) {
            memset(&hid_instances[i], 0, sizeof(hid_instance_info_t));
            hid_instances[i].dev_addr = dev_addr;
            hid_instances[i].instance = instance;
            hid_instances[i].active = true;
            return &hid_instances[i];
        }
    }
    return NULL;
}

static void free_hid_instances_for_device(uint8_t dev_addr) {
    for (int i = 0; i < MAX_HID_INSTANCES; i++) {
        if (hid_instances[i].active && hid_instances[i].dev_addr == dev_addr) {
            hid_instances[i].active = false;
        }
    }
}

// Detect mouse/keyboard usage from HID report descriptor
// Returns effective protocol: HID_ITF_PROTOCOL_MOUSE, HID_ITF_PROTOCOL_KEYBOARD, or HID_ITF_PROTOCOL_NONE
static uint8_t detect_usage_from_report_descriptor(const uint8_t *desc_report, uint16_t desc_len,
                                                    hid_instance_info_t *info) {
    if (!desc_report || desc_len == 0) return HID_ITF_PROTOCOL_NONE;
    
    // Use TinyUSB's built-in parser
    tuh_hid_report_info_t report_info[8];
    uint8_t report_count = tuh_hid_parse_report_descriptor(report_info, 8, desc_report, desc_len);
    
    uint8_t detected_protocol = HID_ITF_PROTOCOL_NONE;
    
    for (uint8_t i = 0; i < report_count; i++) {
        if (report_info[i].usage_page == HID_USAGE_PAGE_DESKTOP) {
            if (report_info[i].usage == HID_USAGE_DESKTOP_MOUSE) {
                detected_protocol = HID_ITF_PROTOCOL_MOUSE;
                if (info) {
                    info->has_report_id = (report_info[i].report_id != 0);
                    info->mouse_report_id = report_info[i].report_id;
                }
                // Mouse usage detected
                break;  // Mouse takes priority
            } else if (report_info[i].usage == HID_USAGE_DESKTOP_KEYBOARD) {
                detected_protocol = HID_ITF_PROTOCOL_KEYBOARD;
                if (info) {
                    info->has_report_id = (report_info[i].report_id != 0);
                    info->keyboard_report_id = report_info[i].report_id;
                }
                // Keyboard usage detected
                // Don't break - keep looking for mouse
            }
        }
    }
    
    return detected_protocol;
}

// Strip vendor-specific HID collections (Logitech HID++ Report IDs 0x10/0x11,
// Usage Page >= 0xFF00) from a cloned HID report descriptor.  Including these
// in the proxy output descriptor causes host drivers to install vendor filters
// (e.g. Logitech HID++ driver) that fight the proxy for the device.
//
// Walks the descriptor item-by-item.  When entering a top-level Collection
// under a vendor Usage Page, skips all items until the matching End Collection.
static size_t strip_vendor_collections(const uint8_t *src, size_t src_len,
                                        uint8_t *dst, size_t dst_max)
{
    if (!src || src_len == 0 || !dst || dst_max == 0) return 0;

    size_t out = 0;
    size_t i = 0;
    uint16_t cur_usage_page = 0;
    int skip_depth = 0;

    while (i < src_len) {
        uint8_t item = src[i];
        uint8_t item_size = item & 0x03;
        if (item_size == 3) item_size = 4;
        uint8_t total_len = 1 + item_size;
        if (i + total_len > src_len) break;

        uint32_t value = 0;
        for (uint8_t b = 0; b < item_size; b++)
            value |= (uint32_t)src[i + 1 + b] << (b * 8);

        uint8_t item_tag = item & 0xFC;

        if (skip_depth > 0) {
            if (item_tag == 0xA0) skip_depth++;
            else if (item_tag == 0xC0) skip_depth--;
            i += total_len;
            continue;
        }

        if (item_tag == 0x04) // Usage Page (Global)
            cur_usage_page = (uint16_t)value;

        if (item_tag == 0xA0 && cur_usage_page >= 0xFF00) {
            // Entering vendor collection — skip it entirely
            skip_depth = 1;
            i += total_len;
            continue;
        }

        if (out + total_len <= dst_max) {
            memcpy(&dst[out], &src[i], total_len);
            out += total_len;
        } else {
            break;
        }
        i += total_len;
    }
    return out;
}

static void build_runtime_hid_report_with_mouse(const uint8_t *mouse_desc, size_t mouse_len)
{
    // Build the composite HID report descriptor:
    //   Keyboard (report ID N) + Mouse (report ID M) + Consumer Control (report ID P)
    //
    // CRITICAL CONSTRAINT (USB HID §5.6): When ANY top-level collection uses a
    // Report ID, ALL collections in the descriptor MUST have a Report ID.
    // Our keyboard and consumer descriptors always have Report IDs, so we MUST
    // ensure the mouse portion also has one.
    //
    // For gaming mice whose descriptors already contain Report IDs, we use the
    // descriptor as-is and remap keyboard/consumer to avoid conflicts.
    //
    // For boot-protocol mice (or mice whose descriptors lack Report IDs), we
    // inject REPORT_ID_MOUSE (2) into the mouse descriptor so it's valid in the
    // composite.
    
    // --- Scan mouse descriptor for existing Report IDs ---
    bool used_ids[256];
    memset(used_ids, 0, sizeof(used_ids));
    bool mouse_has_report_ids = false;
    
    if (mouse_desc != NULL && mouse_len > 0) {
        for (size_t i = 0; i + 1 < mouse_len; i++) {
            if (mouse_desc[i] == 0x85) { // Report ID tag (1-byte value)
                used_ids[mouse_desc[i + 1]] = true;
                mouse_has_report_ids = true;
            }
        }
    }
    
    // --- If mouse has no Report IDs, we'll inject REPORT_ID_MOUSE ---
    // Build a modified copy with Report ID inserted after the first
    // Collection(Application) tag [0xA1, 0x01].
    uint8_t mouse_desc_patched[HID_DESC_BUF_SIZE];
    size_t mouse_len_patched = 0;
    const uint8_t *mouse_desc_final = mouse_desc;
    size_t mouse_len_final = mouse_len;
    
    if (mouse_desc != NULL && mouse_len > 0 && !mouse_has_report_ids) {
        // Inject Report ID after first Collection(Application)
        bool injected = false;
        size_t dst = 0;
        for (size_t src = 0; src < mouse_len && dst < sizeof(mouse_desc_patched) - 2; src++) {
            mouse_desc_patched[dst++] = mouse_desc[src];
            
            // Look for Collection(Application): 0xA1 0x01
            if (!injected && src + 1 < mouse_len &&
                mouse_desc[src] == 0xA1 && mouse_desc[src + 1] == 0x01) {
                // Copy the 0x01 value byte
                src++;
                mouse_desc_patched[dst++] = mouse_desc[src];
                // Inject Report ID(REPORT_ID_MOUSE)
                mouse_desc_patched[dst++] = 0x85; // Report ID tag
                mouse_desc_patched[dst++] = REPORT_ID_MOUSE;
                injected = true;
            }
        }
        
        if (injected) {
            mouse_len_patched = dst;
            mouse_desc_final = mouse_desc_patched;
            mouse_len_final = mouse_len_patched;
            used_ids[REPORT_ID_MOUSE] = true;
        }
    }
    
    // --- Determine safe Report IDs for keyboard and consumer ---
    uint8_t kbd_report_id = REPORT_ID_KEYBOARD;      // default 1
    uint8_t consumer_report_id = REPORT_ID_CONSUMER_CONTROL; // default 3
    
    // If default keyboard ID conflicts, find an unused one starting from 0x10
    if (used_ids[kbd_report_id]) {
        for (uint8_t id = 0x10; id < 0xFE; id++) {
            if (!used_ids[id]) { kbd_report_id = id; break; }
        }
    }
    used_ids[kbd_report_id] = true; // Mark it used
    
    // Same for consumer
    if (used_ids[consumer_report_id]) {
        for (uint8_t id = 0x11; id < 0xFE; id++) {
            if (!used_ids[id]) { consumer_report_id = id; break; }
        }
    }
    
    // Build remapped keyboard and consumer descriptors
    uint8_t kbd_desc[sizeof(desc_hid_keyboard)];
    memcpy(kbd_desc, desc_hid_keyboard, sizeof(desc_hid_keyboard));
    for (size_t i = 0; i + 1 < sizeof(kbd_desc); i++) {
        if (kbd_desc[i] == 0x85 && kbd_desc[i+1] == REPORT_ID_KEYBOARD) {
            kbd_desc[i+1] = kbd_report_id;
            break;
        }
    }
    
    uint8_t consumer_desc[sizeof(desc_hid_consumer)];
    memcpy(consumer_desc, desc_hid_consumer, sizeof(desc_hid_consumer));
    for (size_t i = 0; i + 1 < sizeof(consumer_desc); i++) {
        if (consumer_desc[i] == 0x85 && consumer_desc[i+1] == REPORT_ID_CONSUMER_CONTROL) {
            consumer_desc[i+1] = consumer_report_id;
            break;
        }
    }
    
    size_t pos = 0;

    // Copy keyboard descriptor
    if (pos + sizeof(kbd_desc) >= HID_DESC_BUF_SIZE)
        return;
    memcpy(&desc_hid_report_runtime[pos], kbd_desc, sizeof(kbd_desc));
    pos += sizeof(kbd_desc);

    // Copy mouse descriptor (possibly patched with injected Report ID)
    if (mouse_desc_final != NULL && mouse_len_final > 0)
    {
        // CRITICAL FIX: For 16-bit gaming mice (G703, G Pro Wireless, etc.),
        // macOS may expose an 8-bit HID descriptor even though the mouse reports
        // 16-bit X/Y values. If we clone the 8-bit descriptor, the 16-bit reports
        // get truncated. Solution: If the parsed layout shows 16-bit X/Y fields,
        // override the cloned descriptor with our own 16-bit descriptor.
        bool use_16bit_override = (host_mouse_layout.valid && 
                                   host_mouse_layout.x_bits == 16 && 
                                   host_mouse_layout.y_bits == 16);
        
        if (use_16bit_override) {
            // Use our 16-bit mouse descriptor instead of the cloned 8-bit one
            size_t dlen = sizeof(desc_hid_mouse_16bit);
            if (pos + dlen >= HID_DESC_BUF_SIZE)
                return;
            memcpy(&desc_hid_report_runtime[pos], desc_hid_mouse_16bit, dlen);
            pos += dlen;
            using_16bit_output_override = true;
        } else {
            // Use the cloned descriptor from the host mouse
            if (pos + mouse_len_final >= HID_DESC_BUF_SIZE)
                return;
            memcpy(&desc_hid_report_runtime[pos], mouse_desc_final, mouse_len_final);
            pos += mouse_len_final;
            using_16bit_output_override = false;
        }
    }
    else
    {
        // Fallback: Use 16-bit mouse descriptor for Lightspeed and other high-res mice
        // This path is triggered when host_mouse_desc_len == 0 (e.g., Logitech Lightspeed)
        size_t dlen = sizeof(desc_hid_mouse_16bit);
        if (pos + dlen >= HID_DESC_BUF_SIZE)
            return;
        memcpy(&desc_hid_report_runtime[pos], desc_hid_mouse_16bit, dlen);
        pos += dlen;
        using_16bit_output_override = true;
    }

    // Copy consumer control descriptor
    if (pos + sizeof(consumer_desc) >= HID_DESC_BUF_SIZE)
        return;
    memcpy(&desc_hid_report_runtime[pos], consumer_desc, sizeof(consumer_desc));
    pos += sizeof(consumer_desc);

    // Zero-fill remainder
    if (pos < HID_DESC_BUF_SIZE) {
        memset(&desc_hid_report_runtime[pos], 0, HID_DESC_BUF_SIZE - pos);
    }

    // Set actual content length so host reads only valid descriptor data
    desc_hid_runtime_len = pos;
    desc_hid_runtime_valid = true;
    
    // Store the runtime IDs for use by all report-sending paths
    runtime_kbd_report_id = kbd_report_id;
    runtime_consumer_report_id = consumer_report_id;
}

bool usb_hid_init(void)
{
    gpio_init(PIN_BUTTON);

    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_BUTTON);

    // Initialize connection state
    memset(&connection_state, 0, sizeof(connection_state));

    // Initialize per-instance HID tracking
    memset(hid_instances, 0, sizeof(hid_instances));

    // Build default runtime HID report descriptor (keyboard + default mouse + consumer)
    build_runtime_hid_report_with_mouse(NULL, 0);

    // Build initial runtime config descriptor with defaults
    rebuild_configuration_descriptor();

    (void)0; // suppressed init log
    return true;
}



bool usb_host_enable_power(void)
{
    #ifdef PIN_USB_5V
    gpio_put(PIN_USB_5V, 1); // Enable USB power
    #endif
    sleep_ms(100);           // Allow power to stabilize
    return true;
}

void usb_device_mark_initialized(void)
{
    usb_device_initialized = true;
}

void usb_host_mark_initialized(void)
{
    usb_host_initialized = true;
}

//--------------------------------------------------------------------+
// IMPROVED STATE MANAGEMENT
//--------------------------------------------------------------------+

bool get_caps_lock_state(void)
{
    return caps_lock_state;
}

bool is_mouse_connected(void)
{
    return connection_state.mouse_connected;
}

bool is_keyboard_connected(void)
{
    return connection_state.keyboard_connected;
}

static void handle_device_disconnection(uint8_t dev_addr)
{
    // Only reset connection flags for the specific device that was disconnected
    if (dev_addr == connection_state.mouse_dev_addr)
    {
        connection_state.mouse_connected = false;
        connection_state.mouse_dev_addr = 0;
    }

    if (dev_addr == connection_state.keyboard_dev_addr)
    {
        connection_state.keyboard_connected = false;
        connection_state.keyboard_dev_addr = 0;
    }
}

static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol)
{
    // Validate input parameters
    if (dev_addr == 0)
    {
        // Invalid device address - avoid heavy logging in hot path
        return;
    }

    // Track connected device types and store device addresses.
    // Use LED activity to indicate connection instead of console logging.
    switch (itf_protocol)
    {
    case HID_ITF_PROTOCOL_MOUSE:
        connection_state.mouse_connected = true;
        connection_state.mouse_dev_addr = dev_addr;
        neopixel_trigger_activity_flash_color(COLOR_MOUSE_ACTIVITY); // Flash magenta for mouse connection
        kmbox_send_status("Mouse connected");
        break;

    case HID_ITF_PROTOCOL_KEYBOARD:
        connection_state.keyboard_connected = true;
        connection_state.keyboard_dev_addr = dev_addr;
        neopixel_trigger_activity_flash_color(COLOR_KEYBOARD_ACTIVITY); // Flash yellow for keyboard connection
        kmbox_send_status("Keyboard connected");
        break;

    default:
        // Unknown HID protocol: silently ignore logging in the hot path
        kmbox_send_status("Unknown HID device connected");
        break;
    }

    // Update LED status instead of verbose console output
    neopixel_update_status();
}

static bool __not_in_flash_func(process_keyboard_report_internal)(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // CRITICAL FIX: Check readiness before attempting to send
    // If endpoint is busy, return true anyway to avoid blocking the HID report pipeline
    // The keyboard state will be sent with the next available opportunity
    if (!tud_hid_ready())
    {
        return true;  // Endpoint busy, continue processing without blocking
    }

    // Fast path: send report immediately if endpoint is ready
    bool success = tud_hid_report(runtime_kbd_report_id, report, sizeof(hid_keyboard_report_t));
    if (success)
    {
        // Skip error counter reset for performance
        return true;
    }
    else
    {
        return false;
    }
}

// Helper: write a signed axis value into a raw report buffer at an arbitrary
// bit offset and bit width.  Handles 8, 12, 16, and any other width correctly,
// including packed layouts where X and Y share a byte (e.g. 12-bit Logitech).
static inline void __not_in_flash_func(write_axis_bits)(uint8_t *buf, uint8_t buf_len,
                                                         uint8_t start_byte, uint8_t bit_in_byte,
                                                         uint8_t nbits, bool is_16bit,
                                                         int16_t value)
{
    if (nbits > 0 && nbits != 8 && nbits != 16) {
        int32_t max_val = (1 << (nbits - 1)) - 1;
        int32_t min_val = -(1 << (nbits - 1));
        int32_t cv = value;
        if (cv > max_val) cv = max_val;
        if (cv < min_val) cv = min_val;
        uint32_t uval = (uint32_t)cv & ((1u << nbits) - 1);
        if (start_byte + 2 < buf_len) {
            uint32_t mask = ((1u << nbits) - 1) << bit_in_byte;
            uint32_t raw32 = (uint32_t)buf[start_byte]
                           | ((uint32_t)buf[start_byte + 1] << 8)
                           | ((uint32_t)buf[start_byte + 2] << 16);
            raw32 = (raw32 & ~mask) | ((uval << bit_in_byte) & mask);
            buf[start_byte]     = (uint8_t)(raw32 & 0xFF);
            buf[start_byte + 1] = (uint8_t)((raw32 >> 8) & 0xFF);
            buf[start_byte + 2] = (uint8_t)((raw32 >> 16) & 0xFF);
        }
    } else if (is_16bit) {
        if (start_byte + 1 < buf_len) {
            buf[start_byte]     = (uint8_t)(value & 0xFF);
            buf[start_byte + 1] = (uint8_t)((value >> 8) & 0xFF);
        }
    } else {
        int8_t cv = (value > 127) ? 127 : ((value < -128) ? -128 : (int8_t)value);
        if (start_byte < buf_len)
            buf[start_byte] = (uint8_t)cv;
    }
}

// Build a complete raw mouse report from individual values using parsed layout.
static inline void build_raw_mouse_report(uint8_t *buf, uint8_t sz,
                                           const mouse_report_layout_t *L,
                                           uint8_t buttons, int16_t x, int16_t y,
                                           int8_t wheel, int8_t pan)
{
    memset(buf, 0, sz);
    
    // Buttons (handle both 8-bit and 16-bit button fields)
    if (L->buttons_offset < sz) {
        if (L->buttons_bits > 8 && L->buttons_offset + 1 < sz) {
            // 16-bit button field: write two bytes
            buf[L->buttons_offset] = buttons;
            buf[L->buttons_offset + 1] = 0;  // High byte (buttons 9-16, typically unused)
        } else {
            // 8-bit or less
            buf[L->buttons_offset] = buttons;
        }
    }
    
    write_axis_bits(buf, sz, L->x_start_byte, L->x_bit_in_byte, L->x_bits, L->x_is_16bit, x);
    write_axis_bits(buf, sz, L->y_start_byte, L->y_bit_in_byte, L->y_bits, L->y_is_16bit, y);
    if (L->has_wheel && L->wheel_offset < sz)
        buf[L->wheel_offset] = (uint8_t)wheel;
    if (L->has_pan && L->pan_offset < sz)
        buf[L->pan_offset] = (uint8_t)pan;
}

/**
 * Core1 accumulate-only mouse report handler.
 *
 * ARCHITECTURE NOTE: TinyUSB device API (tud_hid_report, etc.) is NOT
 * thread-safe.  Core0 runs tud_task() in the main loop, so ALL device
 * API calls must happen on Core0.  This function — called from Core1's
 * tuh_hid_report_received_cb — ONLY extracts physical mouse data and
 * writes it into the shared accumulators (spinlock-protected).  Core0's
 * hid_device_task() drains the accumulators and sends the USB report.
 */
static void __not_in_flash_func(forward_raw_mouse_report)(const uint8_t *raw, uint16_t raw_len)
{
    const mouse_report_layout_t *L = &host_mouse_layout;

    // --- Extract physical movement from the raw report ---
    int16_t phys_x = 0, phys_y = 0;
    int8_t  phys_wheel = 0, phys_pan = 0;
    uint8_t phys_buttons = 0;

    // Extract buttons (handle both 8-bit and 16-bit button fields)
    if (L->buttons_offset < raw_len) {
        if (L->buttons_bits > 8 && L->buttons_offset + 1 < raw_len) {
            uint16_t buttons16 = raw[L->buttons_offset] | (raw[L->buttons_offset + 1] << 8);
            uint16_t mask = (L->buttons_bits >= 16) ? 0xFFFF : ((1u << L->buttons_bits) - 1);
            phys_buttons = (uint8_t)(buttons16 & mask);
        } else {
            uint8_t mask = (L->buttons_bits >= 8) ? 0xFF : ((1u << L->buttons_bits) - 1);
            phys_buttons = raw[L->buttons_offset] & mask;
        }
    }

    // X axis extraction (all bit-width variants)
    if ((L->x_bits > 0 && L->x_bits != 8 && L->x_bits != 16) || (L->x_is_16bit && (L->x_bit_offset % 8 != 0))) {
        uint8_t nbits = L->x_bits;
        uint8_t start_byte = L->x_start_byte;
        uint8_t bit_in_byte = L->x_bit_in_byte;
        uint8_t end_byte = (L->x_bit_offset + nbits - 1) / 8;
        if (end_byte < raw_len) {
            uint32_t raw32 = (uint32_t)raw[start_byte];
            if (start_byte + 1 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 1] << 8);
            if (start_byte + 2 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 2] << 16);
            raw32 >>= bit_in_byte;
            raw32 &= (1u << nbits) - 1;
            if (raw32 & (1u << (nbits - 1)))
                raw32 |= ~((1u << nbits) - 1);
            phys_x = (int16_t)(int32_t)raw32;
        }
    } else if (L->x_is_16bit) {
        if (L->x_offset + 1 < raw_len)
            phys_x = (int16_t)(raw[L->x_offset] | (raw[L->x_offset + 1] << 8));
    } else if (L->x_bits == 8) {
        if (L->x_offset < raw_len)
            phys_x = (int8_t)raw[L->x_offset];
    }

    // Y axis extraction
    if ((L->y_bits > 0 && L->y_bits != 8 && L->y_bits != 16) || (L->y_is_16bit && (L->y_bit_offset % 8 != 0))) {
        uint8_t nbits = L->y_bits;
        uint8_t start_byte = L->y_start_byte;
        uint8_t bit_in_byte = L->y_bit_in_byte;
        uint8_t end_byte = (L->y_bit_offset + nbits - 1) / 8;
        if (end_byte < raw_len) {
            uint32_t raw32 = (uint32_t)raw[start_byte];
            if (start_byte + 1 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 1] << 8);
            if (start_byte + 2 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 2] << 16);
            raw32 >>= bit_in_byte;
            raw32 &= (1u << nbits) - 1;
            if (raw32 & (1u << (nbits - 1)))
                raw32 |= ~((1u << nbits) - 1);
            phys_y = (int16_t)(int32_t)raw32;
        }
    } else if (L->y_is_16bit) {
        if (L->y_offset + 1 < raw_len)
            phys_y = (int16_t)(raw[L->y_offset] | (raw[L->y_offset + 1] << 8));
    } else if (L->y_bits == 8) {
        if (L->y_offset < raw_len)
            phys_y = (int8_t)raw[L->y_offset];
    }

    if (L->has_wheel && L->wheel_offset < raw_len) {
        phys_wheel = (int8_t)raw[L->wheel_offset];
    }
    if (L->has_pan && L->pan_offset < raw_len) {
        phys_pan = (int8_t)raw[L->pan_offset];
    }

    // --- Accumulate into shared state (spinlock-protected inside each call) ---
    kmbox_update_physical_buttons(phys_buttons & 0x1F);

    if (phys_x != 0 || phys_y != 0) {
        int16_t tx, ty;
        kmbox_transform_movement(phys_x, phys_y, &tx, &ty);
        smooth_record_physical_movement(tx, ty);
        kmbox_add_mouse_movement(tx, ty);
    }
    if (phys_wheel != 0) kmbox_add_wheel_movement(phys_wheel);
    if (phys_pan != 0)   kmbox_add_pan_movement(phys_pan);

    // Signal Core0 that fresh physical data is available.
    // hid_device_task() on Core0 will drain accumulators and send the report.
    was_active = true;
}

/**
 * Accumulate-only mouse report handler.
 *
 * ARCHITECTURE: This function is called from both Core0 (fast command click
 * handler) and Core1 (legacy boot-protocol mouse path).  It MUST NOT call
 * any tud_* device API.  It only accumulates into the shared kmbox state;
 * Core0's hid_device_task() will drain and send.
 */
static bool __not_in_flash_func(process_mouse_report_internal)(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // Fast button validation using bitwise AND
    uint8_t valid_buttons = report->buttons & 0x1F;

    // Update physical button states in kmbox (for lock functionality)
    kmbox_update_physical_buttons(valid_buttons);

    // Record physical movement for velocity tracking (smooth injection)
    if (report->x != 0 || report->y != 0)
    {
        int16_t transformed_x, transformed_y;
        kmbox_transform_movement(report->x, report->y, &transformed_x, &transformed_y);
        
        smooth_record_physical_movement(transformed_x, transformed_y);
        kmbox_add_mouse_movement(transformed_x, transformed_y);
    }

    // Add physical wheel movement
    if (report->wheel != 0)
    {
        kmbox_add_wheel_movement(report->wheel);
    }

    // Signal Core0 that data is available
    was_active = true;
    return true;
}

void __not_in_flash_func(process_kbd_report)(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if (++activity_counter % KEYBOARD_ACTIVITY_THROTTLE == 0)
    {
        neopixel_trigger_activity_flash_color(COLOR_KEYBOARD_ACTIVITY);
    }

    // Fast forward the report
    if (process_keyboard_report_internal(report))
    {
        // Report processed successfully
    }
}

void __not_in_flash_func(process_mouse_report)(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if (++activity_counter % MOUSE_ACTIVITY_THROTTLE == 0)
    {
        neopixel_trigger_activity_flash_color(0x000000FF); // Blue for mouse activity
    }

    // Fast forward the report
    if (process_mouse_report_internal(report))
    {
        // Report processed successfully  
    }

    // TEMPORARILY DISABLED: Rainbow effect for debugging UART
    // If the report contains movement, advance the rainbow hue based on movement
    // if (report->x != 0 || report->y != 0)
    // {
    //     neopixel_rainbow_on_movement(report->x, report->y);
    // }
}

bool find_key_in_report(const hid_keyboard_report_t *report, uint8_t keycode)
{
    if (report == NULL)
    {
        return false;
    }

    // Unrolled loop - HID_KEYBOARD_KEYCODE_COUNT is always 6
    // Avoids branch overhead on RP2350 Cortex-M33 pipeline
    return (report->keycode[0] == keycode) |
           (report->keycode[1] == keycode) |
           (report->keycode[2] == keycode) |
           (report->keycode[3] == keycode) |
           (report->keycode[4] == keycode) |
           (report->keycode[5] == keycode);
}

void hid_device_task(void)
{
    // Use cheap microsecond timer for 1ms precision polling
    static uint32_t start_us = 0;
    uint32_t current_us = time_us_32();
    uint32_t elapsed_us = current_us - start_us;

    if (elapsed_us < (HID_DEVICE_TASK_INTERVAL_MS * 1000u))
    {
        return; // Not enough time elapsed
    }
    start_us = current_us;

    // Remote wakeup handling — wake on button press OR pending injection data.
    // Without this, USB suspend blocks all smooth injection output until the
    // physical mouse moves (which prevents suspend in the first place).
    if (tud_suspended())
    {
        bool has_data = !gpio_get(PIN_BUTTON) ||
                        smooth_has_pending() ||
                        kmbox_has_pending_movement();
        if (has_data) {
            tud_remote_wakeup();
        }
        return;
    }

    // Only send reports when USB device is properly mounted and ready
    if (!tud_mounted() || !tud_ready())
    {
        return;
    }

    // --- Track last-sent button state for change detection ---
    // Real mice send a report immediately on button state changes.
    // A bridge-injected click with no movement must trigger a report
    // on the very next poll — otherwise the delay is detectably wrong.
    // (last_sent_buttons / was_active are file-scope for cross-path sync)

    // ARCHITECTURE: Core0 is the ONLY core that calls tud_hid_report().
    // Core1 (physical mouse callbacks) accumulates into kmbox accumulators
    // and sets was_active=true.  We drain everything here.
    if (tud_hid_ready())
    {
        bool has_kmbox = kmbox_has_pending_movement();
        bool has_smooth = smooth_has_pending();
        // Cheaply read current button byte without draining accumulators
        uint8_t current_buttons = kmbox_get_current_buttons();
        bool buttons_changed = (current_buttons != last_sent_buttons);

        bool has_pending = has_kmbox || has_smooth || buttons_changed;

        if (!has_pending) goto check_idle;

        // Drain accumulators (spinlock-protected, safe from both cores)
        uint8_t buttons;
        int16_t x, y;
        int8_t wheel, pan;
        kmbox_get_mouse_report_16(&buttons, &x, &y, &wheel, &pan);

        // Process smooth injection
        int16_t smooth_x = 0, smooth_y = 0;
        if (has_smooth) {
            int8_t sx8 = 0, sy8 = 0;
            smooth_process_frame(&sx8, &sy8);
            smooth_x = sx8;
            smooth_y = sy8;
            x += smooth_x;
            y += smooth_y;
        }

        // Apply output-stage humanization (tremor).
        // Physical mouse movement is already human; only humanize the
        // synthetic (smooth injection + bridge) portion.
        if (!connection_state.mouse_connected) {
            // No physical mouse — everything is synthetic, full humanization
            apply_output_humanization(&x, &y, x, y);
        } else {
            // Physical mouse connected — only humanize the smooth injection part
            if (smooth_x != 0 || smooth_y != 0) {
                apply_output_humanization(&x, &y, smooth_x, smooth_y);
            }
        }

        // Send if there's any movement, wheel, OR button state to report.
        if (x != 0 || y != 0 || wheel != 0 || buttons != 0 || buttons_changed) {
            // Build raw report matching host mouse descriptor when available.
            if (using_16bit_output_override) {
                uint8_t raw[16];
                build_raw_mouse_report(raw, sizeof(raw), &output_mouse_layout_16bit,
                                       buttons, x, y, wheel, pan);
                tud_hid_report(REPORT_ID_MOUSE, raw, output_mouse_layout_16bit.report_size);
            } else if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
                uint8_t raw[64];
                uint8_t sz = host_mouse_layout.report_size;
                if (sz > sizeof(raw)) sz = sizeof(raw);

                build_raw_mouse_report(raw, sz, &host_mouse_layout,
                                       buttons, x, y, wheel, pan);

                uint8_t rid = host_mouse_layout.has_report_id ? host_mouse_layout.mouse_report_id : REPORT_ID_MOUSE;
                tud_hid_report(rid, raw, sz);
            } else {
                // Clamp to int8 for standard HID mouse report
                int8_t cx = (x > 127) ? 127 : ((x < -128) ? -128 : (int8_t)x);
                int8_t cy = (y > 127) ? 127 : ((y < -128) ? -128 : (int8_t)y);
                tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, cx, cy, wheel, pan);
            }
            last_sent_buttons = buttons;
            was_active = true;
            return;
        }
    }
check_idle:

    // --- Active → idle edge: send one final zero-delta stop report ---
    // Real mice send a last report with zero deltas (confirming the stop)
    // before they begin NAKing idle polls.  Mirror that behavior here.
    if (was_active && tud_hid_ready())
    {
        uint8_t current_buttons = kmbox_get_current_buttons();
        if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
            uint8_t raw[64];
            uint8_t sz = host_mouse_layout.report_size;
            if (sz > sizeof(raw)) sz = sizeof(raw);

            build_raw_mouse_report(raw, sz, &host_mouse_layout,
                                   current_buttons, 0, 0, 0, 0);

            uint8_t rid = host_mouse_layout.has_report_id ? host_mouse_layout.mouse_report_id : REPORT_ID_MOUSE;
            tud_hid_report(rid, raw, sz);
        } else {
            tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons, 0, 0, 0, 0);
        }
        last_sent_buttons = current_buttons;
        was_active = false;
        return;
    }

    // --- Truly idle: NAK naturally (no report sent) ---
    // When the mouse is idle, real mice NAK interrupt IN polls.
    // Sending continuous zero reports when the real device wouldn't is
    // itself a fingerprint.  So we intentionally do nothing here when
    // a physical mouse is connected.

    // Only send fallback reports when NO devices are connected at all
    // (standalone mode without a physical mouse/keyboard attached).
    if (!connection_state.mouse_connected && !connection_state.keyboard_connected)
    {
        send_hid_report(REPORT_ID_MOUSE);
    }
}

void send_hid_report(uint8_t report_id)
{
    // Multiple checks to prevent endpoint allocation conflicts
    if (!tud_mounted() || !tud_ready())
    {
        return; // Prevent endpoint conflicts by not sending reports when device isn't ready
    }

    // Additional check to ensure device stack is stable
    static uint32_t last_mount_check = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_mount_check > 1000)
    { // Check every second
        if (!tud_mounted())
        {
            return; // Device was unmounted, don't send reports
        }
        last_mount_check = current_time;
    }

    switch (report_id)
    {
    case REPORT_ID_KEYBOARD:
        if (!connection_state.keyboard_connected)
        {
            // Check device readiness before each report
            if (tud_hid_ready())
            {
                // Use static array to avoid stack allocation overhead
                static const uint8_t empty_keycode[HID_KEYBOARD_KEYCODE_COUNT] = {0};
                tud_hid_keyboard_report(runtime_kbd_report_id, 0, empty_keycode);
            }
        }
        break;

    case REPORT_ID_MOUSE:
        // Only send button-based mouse movement if no mouse is connected
        if (!connection_state.mouse_connected)
        {
            // Check device readiness before each report
            if (tud_hid_ready())
            {
                static bool prev_button_state = true; // true = not pressed (active low)
                bool current_button_state = gpio_get(PIN_BUTTON);

                if (!current_button_state)
                { // button pressed (active low)
                    // Mouse move up (negative Y direction)
                    tud_hid_mouse_report(REPORT_ID_MOUSE, MOUSE_BUTTON_NONE,
                                         MOUSE_NO_MOVEMENT, MOUSE_BUTTON_MOVEMENT_DELTA,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT);
                }
                else if (prev_button_state != current_button_state)
                {
                    // Send stop movement when button is released
                    tud_hid_mouse_report(REPORT_ID_MOUSE, MOUSE_BUTTON_NONE,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT);
                }

                prev_button_state = current_button_state;
            }
        }
        break;

    case REPORT_ID_CONSUMER_CONTROL:
    {
        // CRITICAL: Check device readiness before each report
        if (tud_hid_ready())
        {
            static const uint16_t empty_key = 0;
            tud_hid_report(runtime_consumer_report_id, &empty_key, HID_CONSUMER_CONTROL_SIZE);
        }
        break;
    }

    default:
        break;
    }
}

void hid_host_task(void)
{
    // This function can be called from core0 if needed for additional host processing
    // The main host task runs on core1 in PIOKMbox.c
}

// Device callbacks with improved error handling
void tud_mount_cb(void)
{
    led_set_blink_interval(LED_BLINK_MOUNTED_MS);
    neopixel_update_status();
}

void tud_umount_cb(void)
{
    led_set_blink_interval(LED_BLINK_UNMOUNTED_MS);

    // Track device unmount as potential error condition
    usb_error_tracker.consecutive_device_errors++;

    neopixel_update_status();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    led_set_blink_interval(LED_BLINK_SUSPENDED_MS);
    neopixel_update_status();
}

void tud_resume_cb(void)
{
    led_set_blink_interval(LED_BLINK_RESUMED_MS);
    neopixel_update_status();
}

// Host callbacks with improved error handling
void tuh_mount_cb(uint8_t dev_addr)
{
    (void)dev_addr;
    neopixel_trigger_activity_flash_color(COLOR_USB_CONNECTION);
    neopixel_update_status();
}

void tuh_umount_cb(uint8_t dev_addr)
{

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    static uint32_t last_unmount_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // If unmounts happen in quick succession, count as consecutive errors
    if (current_time - last_unmount_time < 5000)
    {
        usb_error_tracker.consecutive_host_errors++;
    }
    else
    {
        // Reset error count for normal disconnections
        usb_error_tracker.consecutive_host_errors = 0;
    }
    last_unmount_time = current_time;

    // Trigger visual feedback
    neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
    neopixel_update_status();
}

// HID host callbacks with improved validation
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, const uint8_t *desc_report, uint16_t desc_len)
{
    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    // === DEVICE-LEVEL CLONING (once per physical device) ===
    // Composite devices (e.g. Razer Basilisk V3 = 4 HID interfaces) trigger
    // tuh_hid_mount_cb once per interface. Device/config descriptors and strings
    // are device-level, so we only need to fetch them once.
    bool first_interface_for_device = (cloned_dev_addr != dev_addr);
    
    if (first_interface_for_device) {
        cloned_dev_addr = dev_addr;
        
        // Fetch string descriptors from the attached device
        fetch_device_string_descriptors(dev_addr);
        
        // Capture full device descriptor for identity cloning
        tusb_desc_device_t host_dev_desc;
        if (tuh_descriptor_get_device_sync(dev_addr, &host_dev_desc, sizeof(host_dev_desc)) == XFER_RESULT_SUCCESS) {
            host_device_info.bcdUSB          = host_dev_desc.bcdUSB;
            host_device_info.bDeviceClass    = host_dev_desc.bDeviceClass;
            host_device_info.bDeviceSubClass = host_dev_desc.bDeviceSubClass;
            host_device_info.bDeviceProtocol = host_dev_desc.bDeviceProtocol;
            host_device_info.bMaxPacketSize0 = host_dev_desc.bMaxPacketSize0;
            host_device_info.bcdDevice       = host_dev_desc.bcdDevice;
            host_device_info.valid           = true;
        }
        
        // Capture configuration descriptor (contains all interfaces + endpoints)
        uint8_t cfg_buf[256];
        if (tuh_descriptor_get_configuration_sync(dev_addr, 0, cfg_buf, sizeof(cfg_buf)) == XFER_RESULT_SUCCESS) {
            parse_host_config_descriptor(cfg_buf, sizeof(cfg_buf));
        }
    }

    // === DETERMINE EFFECTIVE PROTOCOL ===
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    hid_instance_info_t *inst_info = alloc_hid_instance(dev_addr, instance);
    uint8_t effective_protocol = itf_protocol;
    
    if (itf_protocol == HID_ITF_PROTOCOL_NONE && desc_report != NULL && desc_len > 0) {
        // Non-boot protocol device — detect usage from report descriptor
        effective_protocol = detect_usage_from_report_descriptor(desc_report, desc_len, inst_info);
    }
    
    if (inst_info) {
        inst_info->effective_protocol = effective_protocol;
    }

    // === MOUSE INTERFACE: Capture HID report descriptor ===
    // CRITICAL: Only capture the mouse descriptor from the mouse interface!
    // Composite devices have multiple interfaces with different descriptors.
    // e.g. Razer Basilisk V3:
    //   Interface 0: Boot Mouse (protocol=2), 79 byte desc — THIS IS THE MOUSE
    //   Interface 1: Keyboard (protocol=1), 159 byte desc — macro keys
    //   Interface 2: Keyboard (protocol=1), 61 byte desc — more keys
    //   Interface 3: Vendor (protocol=0), 22 byte desc — lighting control
    // We MUST NOT let interfaces 1-3 overwrite the mouse descriptor from interface 0.
    bool is_mouse_interface = (effective_protocol == HID_ITF_PROTOCOL_MOUSE);
    
    if (is_mouse_interface && desc_report != NULL && desc_len > 0) {
        // Capture this interface's HID report descriptor as the mouse descriptor.
        // Logitech Unifying receivers include HID++ vendor collections (Report IDs
        // 0x10/0x11, Usage Page 0xFF00) alongside the mouse collection.  Strip
        // these before storing — they confuse host HID drivers and can trigger
        // installation of vendor filter drivers that fight the proxy.
        size_t copy_len = desc_len;
        if (copy_len > sizeof(host_mouse_desc))
            copy_len = sizeof(host_mouse_desc);
        host_mouse_desc_len = strip_vendor_collections(desc_report, copy_len,
                                                        host_mouse_desc, sizeof(host_mouse_desc));

        // Parse the mouse report layout to discover field offsets for raw forwarding
        parse_mouse_report_layout(host_mouse_desc, host_mouse_desc_len, &host_mouse_layout);
        
        // CRITICAL FIX: Detect OS descriptor mismatches (macOS/Windows sometimes expose
        // 8-bit descriptors for 16-bit mice). Check if descriptor claims 8-bit X/Y but
        // report is large enough for 16-bit data (8+ bytes = 2 btn + 2 X + 2 Y + wheel + pan).
        if (host_mouse_layout.valid && 
            host_mouse_layout.x_bits == 8 && 
            host_mouse_layout.y_bits == 8 &&
            host_mouse_layout.report_size >= 8 &&
            host_mouse_layout.buttons_bits >= 8) {
            // Descriptor says 8-bit but structure suggests 16-bit
            // Override to 16-bit layout (common with Logitech Lightspeed, etc.)
            host_mouse_layout.x_bits = 16;
            host_mouse_layout.y_bits = 16;
            host_mouse_layout.x_is_16bit = true;
            host_mouse_layout.y_is_16bit = true;
            host_mouse_layout.report_size = 8;
        }
        
        // Use the layout-parsed report ID (extracted from the mouse collection context)
        // instead of blindly scanning for the first 0x85 tag
        if (host_mouse_layout.valid && host_mouse_layout.has_report_id) {
            host_mouse_has_report_id = true;
            host_mouse_report_id = host_mouse_layout.mouse_report_id;
        } else {
            // Fallback: scan for any Report ID tag in the descriptor
            host_mouse_has_report_id = false;
            host_mouse_report_id = 0;
            for (size_t i = 0; i + 1 < host_mouse_desc_len; ++i) {
                if (host_mouse_desc[i] == 0x85) {
                    host_mouse_has_report_id = true;
                    host_mouse_report_id = host_mouse_desc[i + 1];
                    break;
                }
            }
        }
        
        // Also update instance info with the correct mouse report ID
        if (inst_info) {
            inst_info->has_report_id = host_mouse_has_report_id;
            inst_info->mouse_report_id = host_mouse_report_id;
        }

        // Build runtime HID report descriptor (keyboard + mouse + consumer)
        build_runtime_hid_report_with_mouse(host_mouse_desc, host_mouse_desc_len);
        rebuild_configuration_descriptor();
        
        // CRITICAL: set_attached_device_vid_pid() triggers force_usb_reenumeration()
        // which disconnects/reconnects the device stack. This MUST happen AFTER all
        // descriptors are fully rebuilt. It also only triggers if VID/PID changed.
        set_attached_device_vid_pid(vid, pid);
    }
    else if (first_interface_for_device && !is_mouse_interface) {
        // First interface mounted but it's not the mouse — still need to set VID/PID
        // so we present the correct identity. The mouse descriptor will be captured
        // when the mouse interface mounts (if it exists).
        // Don't overwrite host_mouse_desc or host_mouse_layout here!
        
        // If we haven't seen a mouse interface yet for this device, build
        // descriptors with defaults. They'll be rebuilt when mouse mounts.
        if (host_mouse_desc_len == 0) {
            build_runtime_hid_report_with_mouse(NULL, 0);
            rebuild_configuration_descriptor();
            set_attached_device_vid_pid(vid, pid);
        }
    }

    // Handle HID device connection using effective protocol
    handle_hid_device_connection(dev_addr, effective_protocol);

    // Start receiving reports — but only from interfaces we actually want.
    //
    // Composite gaming mice (e.g. Razer Basilisk V3) expose multiple HID
    // interfaces on a single dev_addr:
    //   Interface 0: Boot Mouse (protocol=2)  — WE WANT THIS
    //   Interface 1: Keyboard (protocol=1)    — macro keys, NOT a real keyboard
    //   Interface 2: Keyboard (protocol=1)    — media keys, NOT a real keyboard
    //   Interface 3: Vendor (protocol=0)      — lighting control
    //
    // If we receive reports from the non-mouse interfaces, their periodic
    // status/idle reports get forwarded as keyboard input, causing garbage
    // keypresses (e.g. '#' flood).
    //
    // Rule: Only receive from mouse interfaces, OR from keyboard/vendor
    // interfaces on a DIFFERENT device (i.e. a standalone keyboard).
    bool should_receive = false;
    
    if (is_mouse_interface) {
        // Always receive from mouse interfaces
        should_receive = true;
    } else if (dev_addr != connection_state.mouse_dev_addr) {
        // This interface is on a different physical device than the mouse,
        // so it's a standalone keyboard — receive its reports
        should_receive = true;
    }
    // else: non-mouse interface on the same device as the mouse → skip

    if (should_receive) {
        if (!tuh_hid_receive_report(dev_addr, instance)) {
            neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
        } else {
            neopixel_update_status();
        }
    } else {
        // Skip receiving from non-mouse interfaces on the same device
    }
    neopixel_update_status();
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    (void)instance;

    // Free per-instance tracking for this device
    free_hid_instances_for_device(dev_addr);

    // Reset string descriptors when device is disconnected
    reset_device_string_descriptors();

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    // Trigger visual feedback
    neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
    neopixel_update_status();
}

// Helper: parse a mouse report from raw bytes, handling variable report sizes.
// When we have a parsed host mouse layout (gaming mice), forward the raw bytes
// with kmbox/smooth deltas injected.  Otherwise fall back to the legacy path
// that re-encodes through hid_mouse_report_t.
static void __not_in_flash_func(parse_and_forward_mouse_report)(const uint8_t *data, uint16_t data_len)
{
    if (data_len < 3) return;

    // --- RUNTIME DETECTION: 16-bit mouse with wrong descriptor ---
    // macOS/Windows may expose an 8-bit descriptor for 16-bit mice (G703, etc.)
    // Detect by comparing actual report size vs descriptor-claimed size
    static bool checked_override = false;
    if (!checked_override && host_mouse_layout.valid && data_len >= 8) {
        checked_override = true;
        if (host_mouse_layout.x_bits == 8 && host_mouse_layout.y_bits == 8) {
            // Descriptor says 8-bit but we're receiving 8+ byte reports
            // Force 16-bit override
            host_mouse_layout.x_bits = 16;
            host_mouse_layout.y_bits = 16;
            host_mouse_layout.x_is_16bit = true;
            host_mouse_layout.y_is_16bit = true;
            host_mouse_layout.report_size = 8;
            
            // Rebuild descriptors with 16-bit override
            build_runtime_hid_report_with_mouse(host_mouse_desc, host_mouse_desc_len);
            rebuild_configuration_descriptor();
        }
    }

    // --- RAW FORWARDING PATH (gaming mice with cloned descriptor) ---
    if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
        forward_raw_mouse_report(data, data_len);
        return;
    }

    // --- LEGACY FALLBACK PATH (boot-protocol / unknown mice) ---
    hid_mouse_report_t mouse_report_local;
    
    if (data_len == 8) {
        // 16-bit coordinate mouse  - check if this is G703-style or legacy
        // G703: bytes 0-1=buttons(16bit), 2-3=X(16bit), 4-5=Y(16bit), 6=wheel, 7=pan
        // Legacy: bytes 0=buttons(8bit), 1=X(8bit), 2=Y(8bit), 3=wheel, 4=pan, 5-7=X(16bit)/Y(16bit)
        // Heuristic: if byte 1 looks like button data (mostly zeros), use G703 layout
        bool is_g703_layout = (data[1] == 0);  // Byte 1 is high button bits (usually 0)
        
        if (is_g703_layout) {
            // G703 Lightspeed layout: 16-bit buttons at 0-1, 16-bit X at 2-3, 16-bit Y at 4-5
            mouse_report_local.buttons = data[0];  // Only low 8 button bits
            
            int16_t x16 = (int16_t)(data[2] | (data[3] << 8));
            int16_t y16 = (int16_t)(data[4] | (data[5] << 8));
            
            mouse_report_local.x = (x16 > 127) ? 127 : (x16 < -128) ? -128 : (int8_t)x16;
            mouse_report_local.y = (y16 > 127) ? 127 : (y16 < -128) ? -128 : (int8_t)y16;
            mouse_report_local.wheel = (int8_t)data[6];
            mouse_report_local.pan = (int8_t)data[7];
        } else {
            // Legacy layout: assume X at bytes 4-5, Y at bytes 6-7
            mouse_report_local.buttons = data[0];
            
            int16_t x16 = (int16_t)(data[4] | (data[5] << 8)) >> 2;
            int16_t y16 = (int16_t)(data[6] | (data[7] << 8)) >> 2;
            
            mouse_report_local.x = (x16 > 127) ? 127 : (x16 < -128) ? -128 : (int8_t)x16;
            mouse_report_local.y = (y16 > 127) ? 127 : (y16 < -128) ? -128 : (int8_t)y16;
            
            // Wheel detection
            mouse_report_local.wheel = (data[1] != 0) ? (int8_t)data[1] : 
                                       (data[2] != 0) ? (int8_t)data[2] : (int8_t)data[3];
            mouse_report_local.pan = 0;
        }
    } else {
        // Standard format - can cast directly if length allows
        if (data_len >= sizeof(hid_mouse_report_t)) {
            process_mouse_report((const hid_mouse_report_t*)data);
            return;
        }
        
        // Partial report - build manually
        mouse_report_local.buttons = data[0];
        mouse_report_local.x = (int8_t)data[1];
        mouse_report_local.y = (int8_t)data[2];
        mouse_report_local.wheel = (data_len >= 4) ? (int8_t)data[3] : 0;
        mouse_report_local.pan = (data_len >= 5) ? (int8_t)data[4] : 0;
    }

    process_mouse_report(&mouse_report_local);
}

void __not_in_flash_func(tuh_hid_report_received_cb)(uint8_t dev_addr, uint8_t instance, const uint8_t *report, uint16_t len)
{
    if (report == NULL || len == 0)
    {
        tuh_hid_receive_report(dev_addr, instance);
        return;
    }

    // Look up effective protocol from our per-instance tracking
    // This handles non-boot-protocol devices (Logitech, gaming mice, etc.)
    uint8_t effective_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    hid_instance_info_t *inst_info = find_hid_instance(dev_addr, instance);
    bool has_report_id = false;
    uint8_t mouse_report_id = 0;
    
    if (inst_info) {
        effective_protocol = inst_info->effective_protocol;
        has_report_id = inst_info->has_report_id;
        mouse_report_id = inst_info->mouse_report_id;
    }

    // Direct processing without extra copying for better performance
    // SAFETY: Only forward keyboard reports from standalone keyboard devices,
    // not from keyboard interfaces on composite mouse devices (e.g. Razer
    // Basilisk V3 exposes macro/media key interfaces that send garbage data).
    switch (effective_protocol)
    {
    case HID_ITF_PROTOCOL_KEYBOARD:
        {
            // Only forward if this is a standalone keyboard (different device than mouse)
            if (dev_addr == connection_state.mouse_dev_addr) {
                // This keyboard interface is on the same device as the mouse —
                // it's a composite gaming mouse's macro/media keys, skip it
                break;
            }
            
            const uint8_t *kbd_data = report;
            uint16_t kbd_len = len;
            
            // Strip report ID prefix if present
            if (has_report_id && kbd_len > 0) {
                kbd_data++;
                kbd_len--;
            }
            
            if (kbd_len >= (int)sizeof(hid_keyboard_report_t))
            {
                process_kbd_report((const hid_keyboard_report_t*)kbd_data);
            }
        }
        break;

    case HID_ITF_PROTOCOL_MOUSE:
        {
            const uint8_t *mouse_data = report;
            uint16_t mouse_len = len;
            
            // For composite/report-ID devices, the first byte is the report ID
            // We need to strip it before parsing the mouse data
            if (has_report_id && mouse_len > 0) {
                uint8_t received_id = mouse_data[0];
                // Only process if this report ID matches the mouse report ID
                if (received_id != mouse_report_id) {
                    // Not a mouse report from this composite device - skip
                    break;
                }
                mouse_data++;
                mouse_len--;
            }
            
            parse_and_forward_mouse_report(mouse_data, mouse_len);
        }
        break;

    default:
        // Unknown HID protocol - ignore
        break;
    }

    // Continue to request reports
    tuh_hid_receive_report(dev_addr, instance);
}

// HID device callbacks with improved validation
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize)
{
    (void)instance;

    if (report_type == HID_REPORT_TYPE_OUTPUT && report_id == runtime_kbd_report_id)
    {
        // Validate buffer
        if (buffer == NULL || bufsize < MIN_BUFFER_SIZE)
        {
            return;
        }

        uint8_t const kbd_leds = buffer[0];
        bool new_caps_state = (kbd_leds & KEYBOARD_LED_CAPSLOCK) != 0;

        if (new_caps_state != caps_lock_state)
        {
            caps_lock_state = new_caps_state;
            // Indicate caps lock change with LED flash instead of console logging
            neopixel_trigger_caps_lock_flash();
        }
    }
}

void tud_hid_report_complete_cb(uint8_t instance, const uint8_t *report, uint16_t len)
{
    (void)instance;
    (void)len;
    (void)report;
}

bool usb_device_stack_reset(void)
{
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if device stack was previously initialized
    if (!usb_device_initialized)
    {
        return true;
    }

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.device_errors = 0;
    usb_error_tracker.consecutive_device_errors = 0;
    usb_error_tracker.device_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    (void)elapsed_time; // suppressed timing log

    return true; // Return success to prevent repeated attempts
}

bool usb_host_stack_reset(void)
{
#if PIO_USB_AVAILABLE
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if host stack was previously initialized
    if (!usb_host_initialized)
    {
        return true;
    }

    // Reset connection tracking without reinitializing stacks
    memset(&connection_state, 0, sizeof(connection_state));

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.host_errors = 0;
    usb_error_tracker.consecutive_host_errors = 0;
    usb_error_tracker.host_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    (void)elapsed_time; // suppressed timing log

    return true; // Return success to prevent repeated attempts
#else
    (void)0; // suppressed log
    return false;
#endif
}

bool usb_stacks_reset(void)
{
    neopixel_trigger_usb_reset_pending();

    const bool device_success = usb_device_stack_reset();
    const bool host_success = usb_host_stack_reset();
    const bool overall_success = device_success && host_success;

    if (overall_success)
    {
        neopixel_trigger_usb_reset_success();
    }
    else
    {
        neopixel_trigger_usb_reset_failed();
    }

    return overall_success;
}

void usb_stack_error_check(void)
{
    const uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Only check errors at specified intervals
    if (current_time - usb_error_tracker.last_error_check_time < USB_ERROR_CHECK_INTERVAL_MS)
    {
        return;
    }
    usb_error_tracker.last_error_check_time = current_time;

    const bool device_healthy = tud_ready(); // Remove mount requirement as device can be functional without being mounted
    if (!device_healthy)
    {
        usb_error_tracker.consecutive_device_errors++;
    }
    else
    {
        usb_error_tracker.consecutive_device_errors = 0;
        usb_error_tracker.device_error_state = false;
    }

    // Trigger reset if error threshold is exceeded
    if (usb_error_tracker.consecutive_device_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.device_error_state)
        {
            usb_error_tracker.device_error_state = true;
        }
    }

#if PIO_USB_AVAILABLE
    if (usb_error_tracker.consecutive_host_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.host_error_state)
        {
            usb_error_tracker.host_error_state = true;
        }
    }
#endif
}

// Device Descriptors
// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor — clones host mouse fields when available
uint8_t const * tud_descriptor_device_cb(void)
{
    static tusb_desc_device_t desc_device;

    desc_device = (tusb_desc_device_t) {
        .bLength            = sizeof(tusb_desc_device_t),
        .bDescriptorType    = TUSB_DESC_DEVICE,
        // Clone ALL device descriptor fields from the host mouse to look identical
        .bcdUSB             = host_device_info.valid ? host_device_info.bcdUSB : 0x0200,
        .bDeviceClass       = host_device_info.valid ? host_device_info.bDeviceClass : 0x00,
        .bDeviceSubClass    = host_device_info.valid ? host_device_info.bDeviceSubClass : 0x00,
        .bDeviceProtocol    = host_device_info.valid ? host_device_info.bDeviceProtocol : 0x00,
        .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,  // Must match our controller's actual EP0 size

        // Clone VID/PID from host device
        .idVendor           = (get_attached_vid() != 0) ? get_attached_vid() : USB_VENDOR_ID,
        .idProduct          = (get_attached_pid() != 0) ? get_attached_pid() : USB_PRODUCT_ID,
        // Clone device revision from host
        .bcdDevice          = host_device_info.valid ? host_device_info.bcdDevice : 0x0100,

        .iManufacturer      = 0x01,
        .iProduct           = 0x02,
        .iSerialNumber      = attached_has_serial ? 0x03 : 0x00,

        .bNumConfigurations = 0x01
    };

    return (uint8_t const *)&desc_device;
}

// HID Report Descriptor
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    if (desc_hid_runtime_valid)
    {
        return desc_hid_report_runtime;
    }
    // Fallback to static concatenation if runtime descriptor not ready
    return desc_hid_report_runtime; // still points to buffer (may contain defaults)
}

// Configuration Descriptor - now dynamic for cloning
enum
{
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

// Offsets within the configuration descriptor for fields we patch
#define CFG_DESC_OFFSET_BMATTRIBUTES    7
#define CFG_DESC_OFFSET_BMAXPOWER       8
#define HID_ITF_OFFSET_SUBCLASS         (TUD_CONFIG_DESC_LEN + 6)  // bInterfaceSubClass
#define HID_ITF_OFFSET_PROTOCOL         (TUD_CONFIG_DESC_LEN + 7)  // bInterfaceProtocol
#define HID_DESC_OFFSET_REPORT_LEN_LO   (TUD_CONFIG_DESC_LEN + 9 + 7)  // wDescriptorLength low byte
#define HID_DESC_OFFSET_REPORT_LEN_HI   (TUD_CONFIG_DESC_LEN + 9 + 8)  // wDescriptorLength high byte
#define HID_EP_OFFSET_MAXPACKET_LO      (TUD_CONFIG_DESC_LEN + 9 + 9 + 4) // wMaxPacketSize low byte
#define HID_EP_OFFSET_MAXPACKET_HI      (TUD_CONFIG_DESC_LEN + 9 + 9 + 5) // wMaxPacketSize high byte
#define HID_EP_OFFSET_INTERVAL          (TUD_CONFIG_DESC_LEN + 9 + 9 + 6) // bInterval

// Build the configuration descriptor from current runtime state
static void rebuild_configuration_descriptor(void) {
    // Determine actual report descriptor length
    size_t report_desc_len = desc_hid_runtime_valid ? desc_hid_runtime_len : sizeof(desc_hid_report);
    
    // Clone ALL configuration descriptor fields from the host mouse.
    // The goal is to present as the exact same device to the downstream PC.
    uint8_t cfg_attributes = TU_BIT(7) | (host_config_info.valid ? host_config_info.bmAttributes : TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP);
    uint8_t cfg_max_power = host_config_info.valid ? host_config_info.bMaxPower : (USB_CONFIG_POWER_MA / 2);
    uint8_t itf_sub_class = host_config_info.valid ? host_config_info.bInterfaceSubClass : 0;
    uint8_t itf_protocol = host_config_info.valid ? host_config_info.bInterfaceProtocol : HID_ITF_PROTOCOL_NONE;
    // CRITICAL: wMaxPacketSize must be our actual EP buffer size, NOT the host mouse's.
    // We re-encode reports through tud_hid_mouse_report() / tud_hid_keyboard_report(),
    // and our keyboard report (1 + 8 = 9 bytes) may exceed a small mouse EP size (e.g. 8).
    // The downstream PC uses this to allocate its receive buffer — it must fit our largest report.
    uint16_t ep_max_packet = CFG_TUD_HID_EP_BUFSIZE;
    uint8_t ep_interval = host_config_info.valid ? host_config_info.bInterval : HID_POLLING_INTERVAL_MS;
    
    // Build using the TinyUSB macros as a base template, then patch
    uint8_t template[] = {
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USB_CONFIG_POWER_MA),
        TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, HID_POLLING_INTERVAL_MS)
    };
    
    _Static_assert(sizeof(template) == TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN, "config descriptor size mismatch");
    memcpy(desc_configuration_runtime, template, sizeof(template));
    
    // Patch config descriptor fields
    desc_configuration_runtime[CFG_DESC_OFFSET_BMATTRIBUTES] = cfg_attributes;
    desc_configuration_runtime[CFG_DESC_OFFSET_BMAXPOWER] = cfg_max_power;
    
    // Patch HID interface descriptor fields
    desc_configuration_runtime[HID_ITF_OFFSET_SUBCLASS] = itf_sub_class;
    desc_configuration_runtime[HID_ITF_OFFSET_PROTOCOL] = itf_protocol;
    
    // Patch HID report descriptor length (critical — must match what tud_hid_descriptor_report_cb returns)
    desc_configuration_runtime[HID_DESC_OFFSET_REPORT_LEN_LO] = (uint8_t)(report_desc_len & 0xFF);
    desc_configuration_runtime[HID_DESC_OFFSET_REPORT_LEN_HI] = (uint8_t)((report_desc_len >> 8) & 0xFF);
    
    // Patch endpoint descriptor fields  
    desc_configuration_runtime[HID_EP_OFFSET_MAXPACKET_LO] = (uint8_t)(ep_max_packet & 0xFF);
    desc_configuration_runtime[HID_EP_OFFSET_MAXPACKET_HI] = (uint8_t)((ep_max_packet >> 8) & 0xFF);
    desc_configuration_runtime[HID_EP_OFFSET_INTERVAL] = ep_interval;
    
    desc_config_runtime_valid = true;
}

// Parse host configuration descriptor to extract endpoint size, interval, power, etc.
static void parse_host_config_descriptor(const uint8_t *cfg_desc, uint16_t cfg_len) {
    if (!cfg_desc || cfg_len < TUD_CONFIG_DESC_LEN) return;
    
    // Extract config-level fields
    host_config_info.bmAttributes = cfg_desc[7] & 0x7F; // Mask off reserved bit 7 (we add it back)
    host_config_info.bMaxPower = cfg_desc[8];
    
    // Walk the descriptor chain to find HID interface + endpoint
    uint16_t offset = 0;
    bool found_hid_interface = false;
    
    while (offset + 1 < cfg_len) {
        uint8_t desc_length = cfg_desc[offset];
        uint8_t desc_type = cfg_desc[offset + 1];
        
        if (desc_length == 0) break; // Prevent infinite loop on malformed descriptors
        
        // Interface descriptor
        if (desc_type == TUSB_DESC_INTERFACE && desc_length >= 9 && offset + 8 < cfg_len) {
            uint8_t itf_class = cfg_desc[offset + 5];
            if (itf_class == TUSB_CLASS_HID) {
                host_config_info.bInterfaceSubClass = cfg_desc[offset + 6];
                host_config_info.bInterfaceProtocol = cfg_desc[offset + 7];
                found_hid_interface = true;
            }
        }
        
        // Endpoint descriptor (IN endpoint after HID interface)
        if (found_hid_interface && desc_type == TUSB_DESC_ENDPOINT && desc_length >= 7 && offset + 6 < cfg_len) {
            uint8_t ep_addr = cfg_desc[offset + 2];
            uint8_t ep_attr = cfg_desc[offset + 3];
            
            // Only capture IN interrupt endpoint (direction bit 7 set, transfer type = interrupt)
            if ((ep_addr & 0x80) && (ep_attr & 0x03) == TUSB_XFER_INTERRUPT) {
                host_config_info.wMaxPacketSize = cfg_desc[offset + 4] | (cfg_desc[offset + 5] << 8);
                host_config_info.bInterval = cfg_desc[offset + 6];
                host_config_info.valid = true;
                break; // Found what we need
            }
        }
        
        offset += desc_length;
    }
}

// Static fallback (used only for initial sizeof reference)
uint8_t const desc_configuration[] =
    {
        // Config number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USB_CONFIG_POWER_MA),

        // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
        TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, HID_POLLING_INTERVAL_MS)};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Returns the runtime (cloned) config descriptor if available
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index; // for multiple configurations
    if (desc_config_runtime_valid) {
        return desc_configuration_runtime;
    }
    return desc_configuration;
}

// String Descriptors
char const *string_desc_arr[] =
    {
        (const char[]){USB_LANGUAGE_ENGLISH_US_BYTE1, USB_LANGUAGE_ENGLISH_US_BYTE2}, // 0: is supported language is English (0x0409)
        MANUFACTURER_STRING,                                                          // 1: Manufacturer
        PRODUCT_STRING,                                                               // 2: Product
        "PIOKMbox_fallback"                                                           // 3: Fallback serial (unused - dynamic serial used instead)
};

static uint16_t _desc_str[MAX_STRING_DESCRIPTOR_CHARS + 1];

// Convert ASCII string into UTF-16
static uint8_t convert_string_to_utf16(const char *str, uint16_t *desc_str)
{
    if (str == NULL || desc_str == NULL)
    {
        return 0;
    }

    // Cap at max char
    uint8_t chr_count = strlen(str);
    if (chr_count > MAX_STRING_DESCRIPTOR_CHARS)
    {
        chr_count = MAX_STRING_DESCRIPTOR_CHARS;
    }

    // Convert ASCII string into UTF-16
    for (uint8_t i = 0; i < chr_count; i++)
    {
        desc_str[STRING_DESC_FIRST_CHAR_OFFSET + i] = str[i];
    }

    return chr_count;
}

// Invoked when received GET STRING DESCRIPTOR request
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    uint8_t chr_count;

    if (index == BUFFER_FIRST_ELEMENT_INDEX)
    {
        memcpy(&_desc_str[STRING_DESC_FIRST_CHAR_OFFSET],
               string_desc_arr[BUFFER_FIRST_ELEMENT_INDEX],
               STRING_DESC_HEADER_SIZE);
        chr_count = STRING_DESC_CHAR_COUNT_INIT;
    }
    else
    {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[BUFFER_FIRST_ELEMENT_INDEX])))
        {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        if (str == NULL)
        {
            return NULL;
        }

        // Use dynamic string descriptors if available
        if (string_descriptors_fetched) {
            switch (index) {
                case STRING_DESC_MANUFACTURER_IDX:
                    str = attached_manufacturer;
                    break;
                case STRING_DESC_PRODUCT_IDX:
                    str = attached_product;
                    break;
                case STRING_DESC_SERIAL_IDX:
                    if (attached_has_serial && strlen(attached_serial) > 0) {
                        str = attached_serial;
                    } else {
                        str = get_dynamic_serial_string();
                    }
                    break;
                default:
                    // Use default for other indices
                    break;
            }
        } else {
            // Fallback to default strings if not fetched yet
            if (index == STRING_DESC_SERIAL_IDX) {
                str = get_dynamic_serial_string();
            }
        }

        // Convert ASCII string to UTF-16 and get character count
        chr_count = convert_string_to_utf16(str, _desc_str);
    }

    // first byte is length (including header), second byte is string type
    _desc_str[BUFFER_FIRST_ELEMENT_INDEX] = (TUSB_DESC_STRING << STRING_DESC_TYPE_SHIFT) |
                                            (STRING_DESC_LENGTH_MULTIPLIER * chr_count + STRING_DESC_HEADER_SIZE);

    return _desc_str;
}