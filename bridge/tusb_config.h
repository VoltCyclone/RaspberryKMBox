/*
 * TinyUSB Configuration - KMBox Bridge Optimized
 * 
 * Custom configuration for high-throughput CDC communication
 * - Increased buffer sizes for better USB performance
 * - Optimized for 240MHz RP2350 operation
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by compiler flags for flexibility
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

// Enable Device stack
#define CFG_TUD_ENABLED 1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUD_MAX_SPEED BOARD_TUD_MAX_SPEED

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
// CDC - optimized for high throughput
#define CFG_TUD_CDC 1

// CDC FIFO sizes - INCREASED for better performance
// Default is 64, we use 2048 for burst transfers
#define CFG_TUD_CDC_RX_BUFSIZE 2048
#define CFG_TUD_CDC_TX_BUFSIZE 2048

// CDC Endpoint transfer sizes
#define CFG_TUD_CDC_EP_BUFSIZE 64

// Disable unused classes to save resources
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_USBTMC 0
#define CFG_TUD_DFU_RUNTIME 0
#define CFG_TUD_NET 0
#define CFG_TUD_BTH 0
#define CFG_TUD_ECM_RNDIS 0
#define CFG_TUD_NCM 0

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
