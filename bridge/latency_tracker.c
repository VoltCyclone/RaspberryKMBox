/**
 * Latency Tracker Implementation
 * 
 * Uses SDK hardware timer for high-precision latency measurement.
 * Simple and efficient approach using time_us_32().
 */

#include "latency_tracker.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"

static bool initialized = false;

// Rolling statistics
static latency_stats_t stats = {
    .min_us = UINT32_MAX,
    .max_us = 0,
    .avg_us = 0,
    .last_us = 0,
    .sample_count = 0,
    .jitter_us = 0
};

// For rolling average calculation
static uint64_t sum_us = 0;
static uint64_t sum_sq_us = 0;

bool latency_tracker_init(void) {
    if (initialized) {
        return true;
    }
    
    // Using SDK hardware timer for microsecond precision
    // time_us_32() wraps every ~71 minutes, fine for latency measurement
    
    initialized = true;
    return true;
}

uint32_t latency_start_timing(void) {
    // Use hardware timer for high precision
    // time_us_32() wraps every ~71 minutes, fine for latency measurement
    return time_us_32();
}

void latency_end_timing(uint32_t start_token) {
    uint32_t end_time = time_us_32();
    
    // Handle wrap-around (rare but possible)
    uint32_t elapsed_us;
    if (end_time >= start_token) {
        elapsed_us = end_time - start_token;
    } else {
        elapsed_us = (UINT32_MAX - start_token) + end_time + 1;
    }
    
    // Update statistics===
    stats.last_us = elapsed_us;
    stats.sample_count++;
    
    if (elapsed_us < stats.min_us) {
        stats.min_us = elapsed_us;
    }
    if (elapsed_us > stats.max_us) {
        stats.max_us = elapsed_us;
    }
    
    // Rolling average (with overflow protection)
    if (stats.sample_count < 1000000) {
        sum_us += elapsed_us;
        sum_sq_us += (uint64_t)elapsed_us * elapsed_us;
        stats.avg_us = (uint32_t)(sum_us / stats.sample_count);
        
        // Simple jitter estimate: sqrt(variance)
        if (stats.sample_count > 1) {
            uint64_t variance = (sum_sq_us / stats.sample_count) - 
                               ((uint64_t)stats.avg_us * stats.avg_us);
            // Integer square root approximation
            uint32_t jitter = 0;
            uint64_t bit = 1ULL << 30;
            while (bit > variance) bit >>= 2;
            while (bit != 0) {
                if (variance >= jitter + bit) {
                    variance -= jitter + bit;
                    jitter = (jitter >> 1) + bit;
                } else {
                    jitter >>= 1;
                }
                bit >>= 2;
            }
            stats.jitter_us = jitter;
        }
    } else {
        // Reset sums to prevent overflow after many samples
        sum_us = stats.avg_us;
        sum_sq_us = (uint64_t)stats.avg_us * stats.avg_us;
        stats.sample_count = 1;
    }
}

void latency_get_stats(latency_stats_t *out_stats) {
    if (out_stats) {
        *out_stats = stats;
        
        // Handle case where no samples collected
        if (stats.sample_count == 0) {
            out_stats->min_us = 0;
        }
    }
}

void latency_reset_stats(void) {
    stats.min_us = UINT32_MAX;
    stats.max_us = 0;
    stats.avg_us = 0;
    stats.last_us = 0;
    stats.sample_count = 0;
    stats.jitter_us = 0;
    sum_us = 0;
    sum_sq_us = 0;
}

uint32_t latency_get_raw_time(void) {
    return time_us_32();
}

uint32_t latency_ticks_to_us(uint32_t ticks) {
    // For hardware timer, ticks are already in microseconds
    return ticks;
}
