/*
 * TinyUSB Configuration - KMBox FPGA Bridge (pico2-ice)
 *
 * Single CDC interface for PC ↔ RP2350 communication.
 * The FPGA handles all UART traffic; the RP2350 only needs
 * one CDC port for the PC-side client.
 *
 * Uses pico_ice_usb infrastructure (DFU for bitstream updates).
 */
#pragma once

// pico-ice-sdk
#include "boards.h"
#include "ice_flash.h"

// RHPort number used for device (port 0 for pico-ice)
#define BOARD_DEVICE_RHPORT_NUM     0

// Device mode
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE

// Full speed (RP2350 USB)
#define BOARD_DEVICE_RHPORT_SPEED   OPT_MODE_FULL_SPEED

// Enable Device stack
#define CFG_TUD_ENABLED             1
#define CFG_TUD_MAX_SPEED           OPT_MODE_FULL_SPEED

// Endpoint 0 size
#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE      64
#endif

// ------------- Class Configuration ------------- //

// 1 CDC interface for PC communication
#define CFG_TUD_CDC                 1

// DFU with 2 alternates (flash + CRAM)
#define CFG_TUD_DFU                 1
#define CFG_TUD_DFU_ALT             2
#define CFG_TUD_DFU_XFER_BUFSIZE    ICE_FLASH_PAGE_SIZE

// Disable unused classes
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

// CDC FIFO sizes - large for frame data throughput
#define CFG_TUD_CDC_RX_BUFSIZE      2048
#define CFG_TUD_CDC_TX_BUFSIZE      2048
