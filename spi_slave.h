/**
 * SPI Slave - FPGA-clocked packet reception for KMBox
 *
 * Receives 8-byte KMBox fast binary command packets from the iCE40 FPGA
 * SPI master over a dedicated SPI link. Uses PIO2 for clock-domain
 * crossing (FPGA provides 12 MHz SCK) and DMA for zero-CPU-overhead
 * byte collection.
 *
 * The FPGA relays commands from the bridge RP2350 without protocol
 * translation — packets arrive in the same format that
 * process_fast_command() already handles.
 *
 * Pin constraint: SCK GPIO must be MOSI GPIO + 1 (adjacent pins)
 * for the PIO wait instruction to address both via relative offsets.
 *
 * Synchronization: The PIO naturally self-aligns between transactions
 * (SCK idles low → PIO blocks at "wait 1 pin 1" with empty ISR).
 * No CS_N GPIO IRQ is needed — DMA runs continuously with copy-and-restart.
 *
 * The UART path (kmbox_serial_task) continues to work independently
 * for direct PC connections — this is a parallel input path.
 */

#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Pin mapping (Pico 2 defaults — configurable via CMake defines)
//
// CONSTRAINT: SCK must be MOSI + 1 for PIO addressing.
// CS_N has pull-up bias (idle high when disconnected), no IRQ needed.
// MISO is reserved for future bidirectional support.
// ============================================================================

#ifndef KM_SPI_MOSI_PIN
#define KM_SPI_MOSI_PIN     2       // GPIO 2: MOSI from FPGA
#endif

#ifndef KM_SPI_SCK_PIN
#define KM_SPI_SCK_PIN      3       // GPIO 3: SCK from FPGA (must be MOSI + 1)
#endif

#ifndef KM_SPI_CS_PIN
#define KM_SPI_CS_PIN       4       // GPIO 4: CS_N from FPGA
#endif

#ifndef KM_SPI_MISO_PIN
#define KM_SPI_MISO_PIN     5       // GPIO 5: MISO to FPGA (optional, reserved)
#endif

// PIO allocation: PIO2 is free (PIO0 = neopixel, PIO1 = USB host)
#define KM_SPI_PIO          pio2
#define KM_SPI_SM           0

/**
 * Initialize the SPI slave receiver.
 *
 * Loads the PIO program into PIO2 SM0, configures DMA for continuous
 * 8-byte transfers with copy-and-restart on completion.
 *
 * Call after USB and UART are initialized (in main setup).
 */
void spi_slave_init(void);

/**
 * Process any received SPI packets.
 *
 * Call from the main loop alongside kmbox_serial_task().
 * If a complete 8-byte packet has been received via DMA,
 * forwards it to process_fast_command() for execution.
 *
 * Non-blocking: returns immediately if no packet is available.
 */
void spi_slave_task(void);

#endif // SPI_SLAVE_H
