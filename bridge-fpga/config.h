/**
 * Configuration for KMBox FPGA Bridge (pico2-ice)
 *
 * Pin assignments and tunable parameters for the RP2350B + iCE40 UP5K
 * FPGA bridge with ILI9341 TFT display and FT6206 touch.
 *
 * Architecture:
 *   RP2350 ←(SPI 12M)→ iCE40 FPGA ←(SPI 12M)→ KMBox
 *            +(SPI0 TFT, optional)
 *            +(I2C1 Touch, optional)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"

// ============================================================================
// FPGA Communication (PIO2 SPI via on-board CRAM traces to iCE40)
// Pin assignments match fpga_spi.h — defined here for reference only.
// The fpga_spi driver uses the defines from fpga_spi.h directly.
// ============================================================================
// GPIO 7 → iCE40 pin 14: SPI MOSI
// GPIO 6 → iCE40 pin 15: SPI SCK
// GPIO 5 → iCE40 pin 16: SPI CS_N
// GPIO 4 ← iCE40 pin 17: SPI MISO
// SPI clock: 12 MHz (via PIO2 SM0)

// ============================================================================
// Side Channel UART (direct to KMBox, bypasses FPGA)
// ============================================================================
// GPIO 20 → KMBox UART1 RX (GPIO 9): Bridge TX
// GPIO 25 ← KMBox UART1 TX (GPIO 8): Bridge RX
// 921600 baud, protocol: [0xCC][CMD][LEN][payload][CHK]
// Pin assignments defined in side_channel.h

// ============================================================================
// Pico RGB LED (RP2350, active-low, GPIO 0/1/9)
// ============================================================================
// GPIO 0 = Green, GPIO 1 = Red, GPIO 9 = Blue
// Separate physical LED from the FPGA RGB LED (iCE40 pins 39/40/41).
// Pico LED:  Green=firmware running, Blue=CDC connected, Red=KMBox disconnected
// FPGA LED:  Green=KMBox SPI connected, Red=disconnected blink, Blue=SPI activity
// FPGA DONE LED: GPIO 40 (CDONE) — hardware, lights when bitstream loaded
// Pin assignments defined in pico2_ice.h board header (ICE_LED_*_PIN)

// ============================================================================
// TFT Display (ILI9341 on SPI0)
// ============================================================================
#define TFT_SPI_SCK_GPIO        38
#define TFT_SPI_MOSI_GPIO       35
#define TFT_SPI_MISO_GPIO       36      // Reserved (for future SPI reads)
#define TFT_CS_GPIO             37
#define TFT_DC_GPIO             34
#define TFT_RST_GPIO            39
#define TFT_BL_GPIO             33

// ============================================================================
// FT6206 Capacitive Touch (I2C1)
// ============================================================================
#define TOUCH_SDA_GPIO          26
#define TOUCH_SCL_GPIO          27

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
// Timing
// ============================================================================
#define STATUS_POLL_MS       100     // Check connection status
#define TFT_RENDER_MS        100     // 10 FPS display update

// ============================================================================
// CDC Protocol
// ============================================================================
#define CMD_CONFIG           'C'
#define CMD_ENABLE           'E'
#define CMD_STATUS           'S'

#endif // CONFIG_H
