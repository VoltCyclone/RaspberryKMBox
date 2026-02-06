/*
 * Board header for Adafruit Metro RP2350 (Product #6003)
 * Based on official CircuitPython board definitions and Adafruit pinout docs.
 *
 * Key specs:
 *   - RP2350B (QFN-80, 48 GPIO) — NOT RP2350A
 *   - 16 MB QSPI flash (W25Q128JVxQ)
 *   - USB Host breakout pads: D+ = GPIO32, D- = GPIO33, 5V = GPIO29
 *   - NeoPixel on GPIO25 (3.3V powered, no separate power pin)
 *   - Red LED on GPIO23
 *   - UART0 TX=GPIO0, RX=GPIO1
 *   - I2C0 SDA=GPIO20, SCL=GPIO21
 *   - SPI SCK=GPIO30, MOSI=GPIO31, MISO=GPIO28
 *   - 8 MB PSRAM on GPIO47 (chip select)
 *   - HSTX/DVI on GPIO12-19
 *   - A2 silicon (RP2350-E9 erratum)
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

#ifndef _BOARDS_ADAFRUIT_METRO_RP2350_H
#define _BOARDS_ADAFRUIT_METRO_RP2350_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

// On some samples, the xosc can take longer to stabilize than is usual
#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

// For board detection
#define ADAFRUIT_METRO_RP2350

// --- RP2350 VARIANT ---
// Metro RP2350 uses the RP2350B (QFN-80, 48 GPIO).
#define PICO_RP2350A 0

// --- BOARD-SPECIFIC PINS ---

// USB Host breakout pads
#define ADAFRUIT_METRO_RP2350_USB_HOST_DP_PIN   32
#define ADAFRUIT_METRO_RP2350_USB_HOST_DM_PIN   33
#define ADAFRUIT_METRO_RP2350_USB_HOST_5V_PIN   29

// NeoPixel (WS2812) — 3.3V powered, no separate power control pin
#define ADAFRUIT_METRO_RP2350_NEOPIXEL_PIN      25

// PSRAM chip select
#define ADAFRUIT_METRO_RP2350_PSRAM_CS_PIN      47

// HSTX / DVI pins
#define ADAFRUIT_METRO_RP2350_DVI_CKN_PIN       12
#define ADAFRUIT_METRO_RP2350_DVI_CKP_PIN       13
#define ADAFRUIT_METRO_RP2350_DVI_D0N_PIN       14
#define ADAFRUIT_METRO_RP2350_DVI_D0P_PIN       15
#define ADAFRUIT_METRO_RP2350_DVI_D1N_PIN       16
#define ADAFRUIT_METRO_RP2350_DVI_D1P_PIN       17
#define ADAFRUIT_METRO_RP2350_DVI_D2N_PIN       18
#define ADAFRUIT_METRO_RP2350_DVI_D2P_PIN       19

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 0
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 1
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 23
#endif

// --- RGB (NeoPixel) LED ---
#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN 25
#endif

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 20
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 21
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 30
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 31
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 28
#endif

// --- PIO USB ---
#define PICO_DEFAULT_PIO_USB_DP_PIN ADAFRUIT_METRO_RP2350_USB_HOST_DP_PIN

// --- FLASH ---
// Winbond W25Q128JVxQ (16 MB)
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (16 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

// --- A2 SILICON SUPPORT ---
pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
