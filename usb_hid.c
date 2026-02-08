/*
 * Hurricane PIOKMBox Firmware
 */

#include "usb_hid.h"
#include "defines.h"
#include "led_control.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "kmbox_serial_handler.h" // Include the header for serial handling
#include "state_management.h"   // Include the header for state management
#include "watchdog.h"           // Include the header for watchdog management
#include "smooth_injection.h"   // Include the header for smooth mouse injection
#include <string.h>             // For strcpy, strlen, memset

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

// Device management helpers
static void handle_device_disconnection(uint8_t dev_addr);
static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol);

// Report processing helpers
static bool process_keyboard_report_internal(const hid_keyboard_report_t *report);
static bool process_mouse_report_internal(const hid_mouse_report_t *report);

// --- Runtime HID descriptor mirroring storage & helpers ---
// Gaming mice (Razer, Logitech, SteelSeries) can have very large HID
// report descriptors — 500-1000+ bytes with multiple collections for
// mouse, keyboard macros, and vendor-specific features.
#define HID_DESC_BUF_SIZE 1024

static const uint8_t desc_hid_keyboard[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD))};

static const uint8_t desc_hid_mouse_default[] = {
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE))};

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
    uint8_t  x_offset;         // byte offset from start of report data
    bool     x_is_16bit;       // true for gaming mice, false for boot mice

    // Y axis
    uint8_t  y_offset;
    bool     y_is_16bit;

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
    uint8_t usage_page = 0;
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
                usage_page = (uint8_t)value;
                break;
            case 0x08: // Usage (Local)
                if (usage_stack_count < MAX_USAGES) {
                    usage_stack[usage_stack_count++] = (uint8_t)value;
                }
                usage = (uint8_t)value;
                break;
            case 0x18: // Usage Minimum (Local)
                has_usage_range = true;
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
                        if (usage_page == HID_USAGE_PAGE_BUTTON || has_usage_range) {
                            // Button field
                            layout->buttons_offset = bit_offset / 8;
                            layout->buttons_bits = report_count;
                        } else if (usage_page == HID_USAGE_PAGE_DESKTOP) {
                            // Process each usage in the stack
                            for (uint8_t u = 0; u < usage_stack_count && u < report_count; u++) {
                                uint8_t cur_usage = usage_stack[u];
                                uint32_t field_bit_offset = bit_offset + (u * report_size_bits);
                                uint8_t byte_off = field_bit_offset / 8;

                                if (cur_usage == HID_USAGE_DESKTOP_X) {
                                    layout->x_offset = byte_off;
                                    layout->x_is_16bit = (report_size_bits >= 16);
                                } else if (cur_usage == HID_USAGE_DESKTOP_Y) {
                                    layout->y_offset = byte_off;
                                    layout->y_is_16bit = (report_size_bits >= 16);
                                } else if (cur_usage == HID_USAGE_DESKTOP_WHEEL) {
                                    layout->wheel_offset = byte_off;
                                    layout->has_wheel = true;
                                }
                            }
                            // Also handle X,Y packed in a single INPUT with report_count=2
                            if (usage_stack_count == 0 && usage == HID_USAGE_DESKTOP_X && report_count >= 2) {
                                layout->x_offset = bit_offset / 8;
                                layout->x_is_16bit = (report_size_bits >= 16);
                                layout->y_offset = (bit_offset + report_size_bits) / 8;
                                layout->y_is_16bit = (report_size_bits >= 16);
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
    
    if (layout->report_size < 3) {
        // Too small to be a valid mouse report
        return;
    }
    layout->valid = true;

    // Debug output
    char msg[128];
    snprintf(msg, sizeof(msg), "Layout: sz=%u rid=%u btn@%u x@%u(%s) y@%u(%s) whl@%u pan@%u",
             layout->report_size,
             layout->mouse_report_id,
             layout->buttons_offset,
             layout->x_offset, layout->x_is_16bit ? "16" : "8",
             layout->y_offset, layout->y_is_16bit ? "16" : "8",
             layout->has_wheel ? layout->wheel_offset : 0xFF,
             layout->has_pan ? layout->pan_offset : 0xFF);
    kmbox_send_status(msg);
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
                char msg[64];
                snprintf(msg, sizeof(msg), "Detected MOUSE usage (report_id=%u)", report_info[i].report_id);
                kmbox_send_status(msg);
                break;  // Mouse takes priority
            } else if (report_info[i].usage == HID_USAGE_DESKTOP_KEYBOARD) {
                detected_protocol = HID_ITF_PROTOCOL_KEYBOARD;
                if (info) {
                    info->has_report_id = (report_info[i].report_id != 0);
                    info->keyboard_report_id = report_info[i].report_id;
                }
                char msg[64];
                snprintf(msg, sizeof(msg), "Detected KEYBOARD usage (report_id=%u)", report_info[i].report_id);
                kmbox_send_status(msg);
                // Don't break - keep looking for mouse
            }
        }
    }
    
    return detected_protocol;
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
            
            char msg[64];
            snprintf(msg, sizeof(msg), "Injected Report ID %u into mouse desc (%zu->%zu bytes)",
                     REPORT_ID_MOUSE, mouse_len, mouse_len_patched);
            kmbox_send_status(msg);
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
        if (pos + mouse_len_final >= HID_DESC_BUF_SIZE)
            return;
        memcpy(&desc_hid_report_runtime[pos], mouse_desc_final, mouse_len_final);
        pos += mouse_len_final;
    }
    else
    {
        // Fallback to standard boot-protocol mouse descriptor (report ID 2)
        size_t dlen = sizeof(desc_hid_mouse_default);
        if (pos + dlen >= HID_DESC_BUF_SIZE)
            return;
        memcpy(&desc_hid_report_runtime[pos], desc_hid_mouse_default, dlen);
        pos += dlen;
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
    
    char msg[96];
    snprintf(msg, sizeof(msg), "HID desc: %zu bytes, kbd_id=%u mouse_rid=%s consumer_id=%u",
             pos, kbd_report_id,
             mouse_has_report_ids ? "native" : "injected",
             consumer_report_id);
    kmbox_send_status(msg);
    
    if (kbd_report_id != REPORT_ID_KEYBOARD || consumer_report_id != REPORT_ID_CONSUMER_CONTROL) {
        snprintf(msg, sizeof(msg), "Remapped IDs: kbd=%u consumer=%u (was %u/%u)",
                 kbd_report_id, consumer_report_id, REPORT_ID_KEYBOARD, REPORT_ID_CONSUMER_CONTROL);
        kmbox_send_status(msg);
    }
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

// Helper: forward a raw mouse report with kmbox/smooth deltas injected.
// When we have parsed the host mouse's HID descriptor layout we can
// modify the raw bytes in-place so the downstream PC gets a report that
// exactly matches the cloned descriptor.  Falls back to the legacy
// hid_mouse_report_t path when the layout is unknown.
static bool __not_in_flash_func(forward_raw_mouse_report)(const uint8_t *raw, uint16_t raw_len)
{
    // We need a mutable copy to inject deltas
    uint8_t buf[64];
    if (raw_len > sizeof(buf)) raw_len = sizeof(buf);
    memcpy(buf, raw, raw_len);

    const mouse_report_layout_t *L = &host_mouse_layout;

    // --- Extract physical movement from the raw report ---
    int16_t phys_x = 0, phys_y = 0;
    int8_t  phys_wheel = 0;
    uint8_t phys_buttons = 0;

    if (L->buttons_offset < raw_len) {
        phys_buttons = buf[L->buttons_offset] & ((1u << L->buttons_bits) - 1);
    }

    if (L->x_is_16bit) {
        if (L->x_offset + 1 < raw_len)
            phys_x = (int16_t)(buf[L->x_offset] | (buf[L->x_offset + 1] << 8));
    } else {
        if (L->x_offset < raw_len)
            phys_x = (int8_t)buf[L->x_offset];
    }

    if (L->y_is_16bit) {
        if (L->y_offset + 1 < raw_len)
            phys_y = (int16_t)(buf[L->y_offset] | (buf[L->y_offset + 1] << 8));
    } else {
        if (L->y_offset < raw_len)
            phys_y = (int8_t)buf[L->y_offset];
    }

    if (L->has_wheel && L->wheel_offset < raw_len) {
        phys_wheel = (int8_t)buf[L->wheel_offset];
    }

    // --- Feed physical data into the kmbox accumulator / smooth system ---
    kmbox_update_physical_buttons(phys_buttons & 0x1F);

    if (phys_x != 0 || phys_y != 0) {
        int16_t tx, ty;
        kmbox_transform_movement(phys_x, phys_y, &tx, &ty);
        smooth_record_physical_movement(tx, ty);
        kmbox_add_mouse_movement(tx, ty);
    }
    if (phys_wheel != 0) {
        kmbox_add_wheel_movement(phys_wheel);
    }

    // --- Get combined movement (physical + bridge/kmbox pending) ---
    uint8_t btn_send;
    int8_t  mx8, my8, mwheel, mpan;
    kmbox_get_mouse_report(&btn_send, &mx8, &my8, &mwheel, &mpan);

    // Widen to 16-bit for accumulation
    int16_t mx = mx8, my = my8;

    // Blend smooth injection
    int8_t smooth_x = 0, smooth_y = 0;
    if (smooth_has_pending()) {
        smooth_process_frame(&smooth_x, &smooth_y);
        mx += smooth_x;
        my += smooth_y;
    }

    // --- Check endpoint readiness ---
    if (!tud_hid_ready()) {
        // Re-accumulate raw (pre-smooth) movement to avoid losing it
        int16_t raw_mx = mx - smooth_x;
        int16_t raw_my = my - smooth_y;
        if (raw_mx != 0 || raw_my != 0)
            kmbox_add_mouse_movement((int16_t)raw_mx, (int16_t)raw_my);
        if (mwheel != 0)
            kmbox_add_wheel_movement(mwheel);
        return true;
    }

    // --- Patch the raw report buffer with combined values ---
    // Buttons
    if (L->buttons_offset < raw_len) {
        // Preserve any high bits (extra buttons) from the physical report
        uint8_t btn_mask = (uint8_t)((1u << L->buttons_bits) - 1);
        buf[L->buttons_offset] = (buf[L->buttons_offset] & ~btn_mask) | (btn_send & btn_mask);
    }

    // X axis
    if (L->x_is_16bit) {
        // Clamp to int16 range
        int32_t cx = mx;
        if (cx > 32767) cx = 32767;
        if (cx < -32768) cx = -32768;
        if (L->x_offset + 1 < raw_len) {
            buf[L->x_offset]     = (uint8_t)(cx & 0xFF);
            buf[L->x_offset + 1] = (uint8_t)((cx >> 8) & 0xFF);
        }
    } else {
        int16_t cx = mx;
        if (cx > 127) cx = 127;
        if (cx < -128) cx = -128;
        if (L->x_offset < raw_len)
            buf[L->x_offset] = (uint8_t)(int8_t)cx;
    }

    // Y axis
    if (L->y_is_16bit) {
        int32_t cy = my;
        if (cy > 32767) cy = 32767;
        if (cy < -32768) cy = -32768;
        if (L->y_offset + 1 < raw_len) {
            buf[L->y_offset]     = (uint8_t)(cy & 0xFF);
            buf[L->y_offset + 1] = (uint8_t)((cy >> 8) & 0xFF);
        }
    } else {
        int16_t cy = my;
        if (cy > 127) cy = 127;
        if (cy < -128) cy = -128;
        if (L->y_offset < raw_len)
            buf[L->y_offset] = (uint8_t)(int8_t)cy;
    }

    // Wheel
    if (L->has_wheel && L->wheel_offset < raw_len) {
        int16_t cw = mwheel;
        if (cw > 127) cw = 127;
        if (cw < -128) cw = -128;
        buf[L->wheel_offset] = (uint8_t)(int8_t)cw;
    }

    // Pan
    if (L->has_pan && L->pan_offset < raw_len) {
        int16_t cp = mpan;
        if (cp > 127) cp = 127;
        if (cp < -128) cp = -128;
        buf[L->pan_offset] = (uint8_t)(int8_t)cp;
    }

    // Send the raw report using the mouse report ID from the parsed layout.
    // When the host mouse descriptor uses Report IDs, we must forward with
    // the same ID.  When it doesn't, use our default REPORT_ID_MOUSE.
    uint8_t rid = L->has_report_id ? L->mouse_report_id : REPORT_ID_MOUSE;
    return tud_hid_report(rid, buf, raw_len);
}

static bool __not_in_flash_func(process_mouse_report_internal)(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // Fast button validation using bitwise AND
    uint8_t valid_buttons = report->buttons & 0x1F; // Keep first 5 bits (L/R/M/S1/S2 buttons)

    // Update physical button states in kmbox (for lock functionality)
    kmbox_update_physical_buttons(valid_buttons);

    // Record physical movement for velocity tracking (smooth injection)
    // Apply transform to physical mouse movement if enabled
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

    // Always get and send the combined movement (physical + any pending bridge commands)
    // This ensures bridge movements get sent COMBINED with physical movements
    // rather than waiting for a separate send opportunity
    uint8_t buttons_to_send;
    int8_t x, y, wheel, pan;
    kmbox_get_mouse_report(&buttons_to_send, &x, &y, &wheel, &pan);

    // Process smooth injection queue and blend with physical movement
    // This provides sub-pixel precision and velocity-aware blending
    int8_t smooth_x = 0, smooth_y = 0;
    if (smooth_has_pending())
    {
        smooth_process_frame(&smooth_x, &smooth_y);
        // Add smooth injection to the physical/kmbox movement
        // Clamp the total to int8_t range
        int16_t total_x = (int16_t)x + (int16_t)smooth_x;
        int16_t total_y = (int16_t)y + (int16_t)smooth_y;
        x = (total_x > 127) ? 127 : ((total_x < -128) ? -128 : (int8_t)total_x);
        y = (total_y > 127) ? 127 : ((total_y < -128) ? -128 : (int8_t)total_y);
    }

    // Save pre-smooth values for re-accumulation if endpoint is busy
    int8_t raw_x = x - smooth_x;
    int8_t raw_y = y - smooth_y;
    int8_t final_x = x;
    int8_t final_y = y;
    int8_t final_wheel = wheel;

    // Check readiness right before send to minimize latency
    // If not ready, re-accumulate ONLY the raw kmbox values (before smooth injection)
    // to avoid doubling the smooth injection delta on next cycle
    if (!tud_hid_ready())
    {
        // Endpoint busy - put back only the raw consumed movement (pre-smooth)
        // We must NOT re-add the smooth injection delta or it compounds each miss
        if (raw_x != 0 || raw_y != 0) {
            kmbox_add_mouse_movement(raw_x, raw_y);
        }
        if (wheel != 0) {
            kmbox_add_wheel_movement(wheel);
        }
        return true;
    }

    // Send the report
    bool success = tud_hid_mouse_report(REPORT_ID_MOUSE, buttons_to_send, final_x, final_y, final_wheel, pan);
    return success;
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
    // Optimized polling: 16ms for better performance (60 FPS equivalent)
    static uint32_t start_ms = 0;
    uint32_t current_ms = to_ms_since_boot(get_absolute_time());

    if (current_ms - start_ms < HID_DEVICE_TASK_INTERVAL_MS)
    {
        return; // Not enough time elapsed
    }
    start_ms = current_ms;

    // Remote wakeup handling
    if (tud_suspended() && !gpio_get(PIN_BUTTON))
    {
        // Wake up host if we are in suspend mode and button is pressed
        tud_remote_wakeup();
        return;
    }

    // Only send reports when USB device is properly mounted and ready
    if (!tud_mounted() || !tud_ready())
    {
        return;
    }

    // Flush pending bridge/smooth movements from Core0
    // CRITICAL: When a physical mouse IS connected, Core1 handles physical movement
    // via process_mouse_report_internal() which drains kmbox accumulators.
    // We must NOT drain accumulators here when mouse is connected or we race Core1
    // and steal its movement data. Only drain smooth injection standalone when
    // mouse is connected but idle.
    if (tud_hid_ready())
    {
        bool has_kmbox = kmbox_has_pending_movement();
        bool has_smooth = smooth_has_pending();
        
        if (!has_kmbox && !has_smooth) goto skip_flush;
        
        if (!connection_state.mouse_connected)
        {
            // No physical mouse — Core0 is the only sender, safe to drain everything
            uint8_t buttons;
            int8_t x, y, wheel, pan;
            kmbox_get_mouse_report(&buttons, &x, &y, &wheel, &pan);
            
            int8_t smooth_x = 0, smooth_y = 0;
            if (has_smooth) {
                smooth_process_frame(&smooth_x, &smooth_y);
                int16_t total_x = (int16_t)x + (int16_t)smooth_x;
                int16_t total_y = (int16_t)y + (int16_t)smooth_y;
                x = (total_x > 127) ? 127 : ((total_x < -128) ? -128 : (int8_t)total_x);
                y = (total_y > 127) ? 127 : ((total_y < -128) ? -128 : (int8_t)total_y);
            }
            
            if (x != 0 || y != 0 || wheel != 0 || buttons != 0) {
                // When we have a cloned gaming mouse layout, build a raw report
                // that matches the descriptor format.  Otherwise use the standard API.
                if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
                    uint8_t raw[64];
                    memset(raw, 0, sizeof(raw));
                    uint8_t sz = host_mouse_layout.report_size;
                    if (sz > sizeof(raw)) sz = sizeof(raw);

                    const mouse_report_layout_t *L = &host_mouse_layout;
                    if (L->buttons_offset < sz)
                        raw[L->buttons_offset] = buttons;
                    if (L->x_is_16bit) {
                        if (L->x_offset + 1 < sz) { raw[L->x_offset] = (uint8_t)x; raw[L->x_offset+1] = (x < 0) ? 0xFF : 0x00; }
                    } else {
                        if (L->x_offset < sz) raw[L->x_offset] = (uint8_t)x;
                    }
                    if (L->y_is_16bit) {
                        if (L->y_offset + 1 < sz) { raw[L->y_offset] = (uint8_t)y; raw[L->y_offset+1] = (y < 0) ? 0xFF : 0x00; }
                    } else {
                        if (L->y_offset < sz) raw[L->y_offset] = (uint8_t)y;
                    }
                    if (L->has_wheel && L->wheel_offset < sz)
                        raw[L->wheel_offset] = (uint8_t)wheel;
                    if (L->has_pan && L->pan_offset < sz)
                        raw[L->pan_offset] = (uint8_t)pan;

                    uint8_t rid = L->has_report_id ? L->mouse_report_id : REPORT_ID_MOUSE;
                    tud_hid_report(rid, raw, sz);
                } else {
                    tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, x, y, wheel, pan);
                }
                return;
            }
        }
        else if (has_smooth)
        {
            // Physical mouse IS connected — only send standalone smooth injection.
            // Do NOT drain kmbox accumulators (Core1 owns those).
            int8_t smooth_x = 0, smooth_y = 0;
            smooth_process_frame(&smooth_x, &smooth_y);
            if (smooth_x != 0 || smooth_y != 0) {
                if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
                    uint8_t raw[64];
                    memset(raw, 0, sizeof(raw));
                    uint8_t sz = host_mouse_layout.report_size;
                    if (sz > sizeof(raw)) sz = sizeof(raw);

                    const mouse_report_layout_t *L = &host_mouse_layout;
                    if (L->x_is_16bit) {
                        if (L->x_offset + 1 < sz) { raw[L->x_offset] = (uint8_t)smooth_x; raw[L->x_offset+1] = (smooth_x < 0) ? 0xFF : 0x00; }
                    } else {
                        if (L->x_offset < sz) raw[L->x_offset] = (uint8_t)smooth_x;
                    }
                    if (L->y_is_16bit) {
                        if (L->y_offset + 1 < sz) { raw[L->y_offset] = (uint8_t)smooth_y; raw[L->y_offset+1] = (smooth_y < 0) ? 0xFF : 0x00; }
                    } else {
                        if (L->y_offset < sz) raw[L->y_offset] = (uint8_t)smooth_y;
                    }

                    uint8_t rid = L->has_report_id ? L->mouse_report_id : REPORT_ID_MOUSE;
                    tud_hid_report(rid, raw, sz);
                } else {
                    tud_hid_mouse_report(REPORT_ID_MOUSE, 0, smooth_x, smooth_y, 0, 0);
                }
                return;
            }
        }
    }
skip_flush:

    // Only send reports when devices are not connected
    if (!connection_state.mouse_connected && !connection_state.keyboard_connected)
    {
        send_hid_report(REPORT_ID_MOUSE);
    }
    // Note: No need to send periodic empty consumer control reports
    // when devices are connected — HID connections are maintained by
    // the regular mouse/keyboard reports from the physical devices.
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
    
    // Send status to bridge
    char status_msg[128];
    snprintf(status_msg, sizeof(status_msg), "HID Mount: VID=%04X PID=%04X inst=%u desc=%u",
             vid, pid, instance, desc_len);
    kmbox_send_status(status_msg);

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
            
            snprintf(status_msg, sizeof(status_msg), "Clone DEV: USB%04X bcd=%04X ep0=%u",
                     host_dev_desc.bcdUSB, host_dev_desc.bcdDevice, host_dev_desc.bMaxPacketSize0);
            kmbox_send_status(status_msg);
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
        
        snprintf(status_msg, sizeof(status_msg), "Protocol: itf=%u effective=%u inst=%u",
                 itf_protocol, effective_protocol, instance);
        kmbox_send_status(status_msg);
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
        // Capture this interface's HID report descriptor as the mouse descriptor
        size_t copy_len = desc_len;
        if (copy_len > sizeof(host_mouse_desc))
            copy_len = sizeof(host_mouse_desc);
        memcpy(host_mouse_desc, desc_report, copy_len);
        host_mouse_desc_len = copy_len;

        // Parse the mouse report layout to discover field offsets for raw forwarding
        parse_mouse_report_layout(host_mouse_desc, host_mouse_desc_len, &host_mouse_layout);
        
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

        snprintf(status_msg, sizeof(status_msg), "Mouse desc: %zuB rid=%u layout=%s",
                 host_mouse_desc_len, host_mouse_report_id,
                 host_mouse_layout.valid ? "OK" : "FAIL");
        kmbox_send_status(status_msg);
        
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
        char skip_msg[64];
        snprintf(skip_msg, sizeof(skip_msg), "Skip recv inst=%u proto=%u (composite non-mouse)",
                 instance, effective_protocol);
        kmbox_send_status(skip_msg);
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

    // --- RAW FORWARDING PATH (gaming mice with cloned descriptor) ---
    if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
        forward_raw_mouse_report(data, data_len);
        return;
    }

    // --- LEGACY FALLBACK PATH (boot-protocol / unknown mice) ---
    hid_mouse_report_t mouse_report_local;
    
    if (data_len == 8) {
        // 16-bit coordinate mouse - direct extraction
        mouse_report_local.buttons = data[0];
        
        // Extract and scale 16-bit coordinates inline
        int16_t x16 = (int16_t)(data[4] | (data[5] << 8)) >> 2;
        int16_t y16 = (int16_t)(data[6] | (data[7] << 8)) >> 2;
        
        // Clamp inline
        mouse_report_local.x = (x16 > 127) ? 127 : (x16 < -128) ? -128 : (int8_t)x16;
        mouse_report_local.y = (y16 > 127) ? 127 : (y16 < -128) ? -128 : (int8_t)y16;
        
        // Wheel detection
        mouse_report_local.wheel = (data[1] != 0) ? (int8_t)data[1] : 
                                   (data[2] != 0) ? (int8_t)data[2] : (int8_t)data[3];
        mouse_report_local.pan = 0;
        
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
                
                char msg[96];
                snprintf(msg, sizeof(msg), "Clone EP: pkt=%u interval=%ums power=%umA",
                         host_config_info.wMaxPacketSize, host_config_info.bInterval, host_config_info.bMaxPower * 2);
                kmbox_send_status(msg);
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