/**
 * Latency Tracker - High-Precision Command Timing
 * 
 * Uses PIO for sub-microsecond latency measurement between
 * command reception and execution. Provides statistics for
 * system tuning and performance monitoring.
 * 
 * Now that PIO is freed from UART duties, we can use it for
 * precise timing that doesn't depend on CPU load.
 */

#ifndef LATENCY_TRACKER_H
#define LATENCY_TRACKER_H

#include <stdint.h>
#include <stdbool.h>

// Latency statistics structure
typedef struct {
    uint32_t min_us;        // Minimum latency in microseconds
    uint32_t max_us;        // Maximum latency in microseconds
    uint32_t avg_us;        // Average latency (rolling)
    uint32_t last_us;       // Most recent measurement
    uint32_t sample_count;  // Number of samples
    uint32_t jitter_us;     // Standard deviation estimate
} latency_stats_t;

/**
 * Initialize the latency tracker
 * Uses PIO for high-precision timing
 * 
 * @return true if successful
 */
bool latency_tracker_init(void);

/**
 * Mark the start of a timed operation
 * Call this when a command is received
 * 
 * @return Timestamp token to pass to end_timing()
 */
uint32_t latency_start_timing(void);

/**
 * Mark the end of a timed operation
 * Call this when command processing completes
 * 
 * @param start_token Token from start_timing()
 */
void latency_end_timing(uint32_t start_token);

/**
 * Get current latency statistics
 * 
 * @param stats Pointer to stats structure to fill
 */
void latency_get_stats(latency_stats_t *stats);

/**
 * Reset latency statistics
 */
void latency_reset_stats(void);

/**
 * Get raw timer value (for custom timing)
 * 
 * @return Current timer count
 */
uint32_t latency_get_raw_time(void);

/**
 * Convert timer ticks to microseconds
 * 
 * @param ticks Timer ticks
 * @return Microseconds
 */
uint32_t latency_ticks_to_us(uint32_t ticks);

#endif // LATENCY_TRACKER_H
