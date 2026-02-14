/**
 * DMA Ring Buffer Helpers for UART RX
 *
 * Shared inline utilities for reading from a DMA-driven circular receive
 * buffer.  The DMA channel writes continuously into a power-of-2 aligned
 * buffer using ring mode; these helpers inspect the DMA write pointer to
 * determine how many bytes are available and read them out.
 *
 * Used by both the KMBox serial handler and the bridge HW UART driver.
 */

#ifndef DMA_UART_H
#define DMA_UART_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "hardware/dma.h"

/** Get the current DMA write position in the ring buffer. */
static inline uint32_t dma_ring_write_pos(int dma_chan,
                                           const uint8_t *buf,
                                           uint32_t mask) {
    if (dma_chan < 0) return 0;
    uintptr_t wa = (uintptr_t)dma_channel_hw_addr(dma_chan)->write_addr;
    return (uint32_t)((wa - (uintptr_t)buf) & mask);
}

/** Number of unread bytes in the ring buffer. */
static inline uint32_t dma_ring_available(uint32_t read_pos,
                                           uint32_t write_pos,
                                           uint32_t mask) {
    return (write_pos - read_pos) & mask;
}

/** Peek at a byte at `offset` from the current read position. */
static inline uint8_t dma_ring_peek(const uint8_t *buf,
                                     uint32_t read_pos,
                                     uint32_t offset,
                                     uint32_t mask) {
    return buf[(read_pos + offset) & mask];
}

/**
 * Copy up to `maxlen` bytes out of the ring buffer into `dest`.
 * Updates `*read_pos` to reflect consumed bytes.
 * Returns number of bytes actually copied.
 */
static inline size_t dma_ring_read(const uint8_t *buf,
                                    uint32_t *read_pos,
                                    uint32_t write_pos,
                                    uint32_t mask,
                                    uint8_t *dest,
                                    size_t maxlen) {
    uint32_t avail = dma_ring_available(*read_pos, write_pos, mask);
    size_t to_read = (avail < maxlen) ? avail : maxlen;
    if (to_read == 0) return 0;

    uint32_t rp   = *read_pos;
    uint32_t size = mask + 1;
    size_t first  = size - rp;
    if (first > to_read) first = to_read;

    memcpy(dest, &buf[rp], first);
    if (to_read > first) {
        memcpy(dest + first, buf, to_read - first);
    }

    *read_pos = (rp + (uint32_t)to_read) & mask;
    return to_read;
}

#endif // DMA_UART_H
