/*
 * USB Descriptors - KMBox FPGA Bridge (pico2-ice)
 *
 * Provides:
 *   CDC 0 - KMBox bridge data (PC client ↔ RP2350 ↔ FPGA ↔ KMBox)
 *   DFU   - iCE40 firmware update (flash + CRAM alternates)
 */

#include "ice_usb.h"

enum {
    ITF_NUM_CDC0, ITF_NUM_CDC0_DATA,
    ITF_NUM_DFU,
    ITF_NUM_TOTAL
};

uint8_t const tud_desc_configuration[CONFIG_TOTAL_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 500 /* mA */),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC0, STRID_CDC + 0, EPIN + 1, 8, EPOUT + 2, EPIN + 2, 64),
    TUD_DFU_DESCRIPTOR(ITF_NUM_DFU, CFG_TUD_DFU_ALT, STRID_DFU, DFU_ATTR_CAN_DOWNLOAD, 1000, CFG_TUD_DFU_XFER_BUFSIZE),
};

char const *tud_string_desc[STRID_NUM_TOTAL] = {
    [STRID_LANGID]          = USB_LANG_EN,
    [STRID_MANUFACTURER]    = "RaspberryKMBox",
    [STRID_PRODUCT]         = "KMBox FPGA Bridge",
    [STRID_SERIAL_NUMBER]   = usb_serial_number,
    [STRID_VENDOR]          = "pico2-ice",
    [STRID_CDC + 0]         = "KMBox Bridge Data",
    [STRID_DFU + 0]         = "iCE40 DFU (Flash)",
    [STRID_DFU + 1]         = "iCE40 DFU (CRAM)",
};
