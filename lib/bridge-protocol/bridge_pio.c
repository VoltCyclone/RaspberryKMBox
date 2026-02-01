/**
 * Bridge Protocol PIO+DMA Implementation
 * 
 * High-performance packet transmission using:
 * - PIO for hardware UART + sync byte injection
 * - DMA for zero-copy transfers
 * - Hardware packet framing
 */

#include "bridge_pio.h"
#include "bridge_uart_tx.pio.h"
#include "bridge_uart_rx.pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <string.h>

// Statistics
static bridge_pio_stats_t stats = {0};

bool bridge_pio_init(
    bridge_pio_config_t* config,
    PIO pio,
    uint pin_tx,
    uint pin_rx,
    uint32_t baud_rate
) {
    config->pio = pio;
    config->baud_rate = baud_rate;
    
    // Load TX program
    if (!pio_can_add_program(pio, &bridge_uart_tx_program)) {
        return false;
    }
    config->offset_tx = pio_add_program(pio, &bridge_uart_tx_program);
    config->sm_tx = pio_claim_unused_sm(pio, false);
    if (config->sm_tx == -1) {
        return false;
    }
    
    // Load RX program
    if (!pio_can_add_program(pio, &bridge_uart_rx_program)) {
        pio_remove_program(pio, &bridge_uart_tx_program, config->offset_tx);
        return false;
    }
    config->offset_rx = pio_add_program(pio, &bridge_uart_rx_program);
    config->sm_rx = pio_claim_unused_sm(pio, false);
    if (config->sm_rx == -1) {
        pio_remove_program(pio, &bridge_uart_tx_program, config->offset_tx);
        pio_remove_program(pio, &bridge_uart_rx_program, config->offset_rx);
        return false;
    }
    
    // Initialize PIO programs
    bridge_uart_tx_program_init(pio, config->sm_tx, config->offset_tx, pin_tx, baud_rate);
    bridge_uart_rx_program_init(pio, config->sm_rx, config->offset_rx, pin_rx, baud_rate);
    
    // Claim DMA channels
    config->dma_tx_chan = dma_claim_unused_channel(false);
    config->dma_rx_chan = dma_claim_unused_channel(false);
    
    if (config->dma_tx_chan < 0 || config->dma_rx_chan < 0) {
        if (config->dma_tx_chan >= 0) dma_channel_unclaim(config->dma_tx_chan);
        if (config->dma_rx_chan >= 0) dma_channel_unclaim(config->dma_rx_chan);
        pio_remove_program(pio, &bridge_uart_tx_program, config->offset_tx);
        pio_remove_program(pio, &bridge_uart_rx_program, config->offset_rx);
        return false;
    }
    
    // Configure TX DMA: Memory -> PIO TX FIFO
    dma_channel_config tx_config = dma_channel_get_default_config(config->dma_tx_chan);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, pio_get_dreq(pio, config->sm_tx, true));
    
    dma_channel_configure(
        config->dma_tx_chan,
        &tx_config,
        &pio->txf[config->sm_tx],  // Write to TX FIFO
        NULL,                       // Read address set per transfer
        0,                          // Transfer count set per transfer
        false                       // Don't start yet
    );
    
    // Configure RX DMA: PIO RX FIFO -> Memory  
    dma_channel_config rx_config = dma_channel_get_default_config(config->dma_rx_chan);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, pio_get_dreq(pio, config->sm_rx, false));
    
    dma_channel_configure(
        config->dma_rx_chan,
        &rx_config,
        NULL,                       // Write address set per transfer
        &pio->rxf[config->sm_rx],  // Read from RX FIFO
        0,                          // Transfer count set per transfer
        false                       // Don't start yet
    );
    
    memset(&stats, 0, sizeof(stats));
    
    return true;
}

bool bridge_pio_send_packet(
    bridge_pio_config_t* config,
    const uint8_t* packet_data,
    size_t length
) {
    if (!bridge_pio_tx_ready(config)) {
        stats.tx_stalls++;
        return false;
    }
    
    // PIO automatically prepends sync byte (0xBD)
    // Just send: [CMD][PAYLOAD...]
    
    dma_channel_set_read_addr(config->dma_tx_chan, packet_data, false);
    dma_channel_set_trans_count(config->dma_tx_chan, length, true);  // Start transfer
    
    stats.packets_sent++;
    return true;
}

bool bridge_pio_tx_ready(bridge_pio_config_t* config) {
    return !dma_channel_is_busy(config->dma_tx_chan) &&
           !bridge_uart_tx_full(config->pio, config->sm_tx);
}

bool bridge_pio_receive_packet(
    bridge_pio_config_t* config,
    bridge_packet_t* packet_out
) {
    // Check if sync byte available
    if (!bridge_uart_rx_available(config->pio, config->sm_rx)) {
        return false;
    }
    
    // Read sync byte
    uint8_t sync = bridge_uart_rx_getc(config->pio, config->sm_rx);
    if (sync != BRIDGE_SYNC_BYTE) {
        stats.sync_errors++;
        return false;
    }
    
    // Read command byte
    while (!bridge_uart_rx_available(config->pio, config->sm_rx));
    uint8_t cmd = bridge_uart_rx_getc(config->pio, config->sm_rx);
    
    packet_out->generic.sync = sync;
    packet_out->generic.cmd = cmd;
    
    // Determine payload length based on command
    size_t payload_len = 0;
    switch (cmd) {
        case BRIDGE_CMD_MOUSE_MOVE:
            payload_len = 4;  // x:i16, y:i16
            break;
        case BRIDGE_CMD_MOUSE_WHEEL:
            payload_len = 1;  // wheel:i8
            break;
        case BRIDGE_CMD_BUTTON_SET:
            payload_len = 2;  // mask:u8, state:u8
            break;
        case BRIDGE_CMD_MOUSE_MOVE_WHEEL:
            payload_len = 5;  // x:i16, y:i16, wheel:i8
            break;
        case BRIDGE_CMD_PING:
        case BRIDGE_CMD_RESET:
            payload_len = 0;  // No payload
            break;
        default:
            stats.sync_errors++;
            return false;
    }
    
    // Read payload bytes
    for (size_t i = 0; i < payload_len; i++) {
        while (!bridge_uart_rx_available(config->pio, config->sm_rx));
        packet_out->generic.payload[i] = bridge_uart_rx_getc(config->pio, config->sm_rx);
    }
    
    stats.packets_received++;
    return true;
}

uint32_t bridge_pio_rx_available(bridge_pio_config_t* config) {
    return pio_sm_get_rx_fifo_level(config->pio, config->sm_rx);
}

void bridge_pio_get_stats(bridge_pio_config_t* config, bridge_pio_stats_t* stats_out) {
    *stats_out = stats;
    
    // Check for overruns
    if (pio_sm_is_rx_fifo_full(config->pio, config->sm_rx)) {
        stats.overruns++;
    }
}

void bridge_pio_reset_stats(bridge_pio_config_t* config) {
    memset(&stats, 0, sizeof(stats));
}
