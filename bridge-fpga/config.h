/**
 * Configuration for KMBox FPGA Bridge
 *
 * Tunable parameters for the pico2-ice based bridge.
 * FPGA handles UART ↔ KMBox; RP2350 handles USB CDC + tracking.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"

// ============================================================================
// ROI (Region of Interest) Configuration
// ============================================================================
#define BRIDGE_FPGA_ROI_DEFAULT    48    // Default ROI size (48x48 pixels)
#define BRIDGE_FPGA_ROI_MAX        64    // Maximum supported ROI size
#define BRIDGE_FPGA_FRAME_BUF_SIZE (BRIDGE_FPGA_ROI_MAX * BRIDGE_FPGA_ROI_MAX * 3)

// ============================================================================
// Color Tracking Defaults
// ============================================================================
#define DEFAULT_R_MIN        180
#define DEFAULT_R_MAX        255
#define DEFAULT_G_MAX        80
#define DEFAULT_B_MAX        80
#define DEFAULT_GAIN_X       1.0f
#define DEFAULT_GAIN_Y       1.0f
#define DEFAULT_DEADZONE     2
#define DEFAULT_MIN_BLOB     10

// ============================================================================
// Smoothing
// ============================================================================
#define SMOOTHING_FACTOR     0.7f
#define MAX_MOUSE_DELTA      127

// ============================================================================
// FPGA Communication Timing
// ============================================================================
#define FPGA_STATUS_POLL_MS     100   // Poll FPGA status registers
#define FPGA_RX_DRAIN_MS        10    // Drain FPGA RX FIFO
#define FPGA_TX_TIMEOUT_US      50    // Max wait for TX ready

// ============================================================================
// CDC Protocol
// ============================================================================
#define CMD_CONFIG           'C'
#define CMD_ENABLE           'E'
#define CMD_STATUS           'S'

#endif // CONFIG_H
