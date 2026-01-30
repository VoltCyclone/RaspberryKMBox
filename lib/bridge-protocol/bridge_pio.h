/**
 * Bridge Protocol PIO Optimization
 * 
 * Hardware-accelerated packet framing using PIO state machines
 * for both TX and RX sides of the bridge protocol.
 */

#ifndef BRIDGE_PIO_H
#define BRIDGE_PIO_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "bridge_protocol.h"

// PIO-optimized packet buffer
#define BRIDGE_PIO_PACKET_BUFFER_SIZE 8

typedef struct {
    PIO pio;
    uint sm_tx;
    uint sm_rx;
    uint offset_tx;
    uint offset_rx;
    int dma_tx_chan;
    int dma_rx_chan;
    uint32_t baud_rate;
} bridge_pio_config_t;

// Initialize PIO-based bridge protocol (replaces standard UART)
// Returns true if successful
bool bridge_pio_init(
    bridge_pio_config_t* config,
    PIO pio,
    uint pin_tx,
    uint pin_rx,
    uint32_t baud_rate
);

// Send packet via PIO+DMA (zero-copy)
// Automatically prepends sync byte and handles framing
bool bridge_pio_send_packet(
    bridge_pio_config_t* config,
    const uint8_t* packet_data,  // Command byte + payload (no sync)
    size_t length                // 1-6 bytes (cmd + payload)
);

// Check if TX is ready for next packet
bool bridge_pio_tx_ready(bridge_pio_config_t* config);

// Get received packet from RX FIFO
// Returns true if packet available
bool bridge_pio_receive_packet(
    bridge_pio_config_t* config,
    bridge_packet_t* packet_out
);

// Get number of packets in RX queue
uint32_t bridge_pio_rx_available(bridge_pio_config_t* config);

// Performance monitoring
typedef struct {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t sync_errors;
    uint32_t overruns;
    uint32_t tx_stalls;
} bridge_pio_stats_t;

void bridge_pio_get_stats(bridge_pio_config_t* config, bridge_pio_stats_t* stats);
void bridge_pio_reset_stats(bridge_pio_config_t* config);

#endif // BRIDGE_PIO_H
