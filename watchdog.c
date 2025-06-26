/*
 * Hurricane PIOKMBox Firmware
 */


#include "watchdog.h"
#include "defines.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

//--------------------------------------------------------------------+
// INTERNAL STATE
//--------------------------------------------------------------------+

static watchdog_status_t g_watchdog_status;
static bool g_watchdog_initialized = false;
static bool g_watchdog_started = false;
static bool g_debug_enabled = WATCHDOG_ENABLE_DEBUG;
static uint32_t g_last_hardware_update_ms = 0;

// Inter-core communication for heartbeats
static volatile uint32_t g_core0_heartbeat_timestamp = 0;
static volatile uint32_t g_core1_heartbeat_timestamp = 0;

//--------------------------------------------------------------------+
// INTERNAL FUNCTIONS
//--------------------------------------------------------------------+

/**
 * Get current time in milliseconds since boot
 */
static uint32_t get_time_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

/**
 * Update hardware watchdog timer
 */
static void update_hardware_watchdog(void) {
    if (WATCHDOG_ENABLE_HARDWARE) {
        watchdog_update();
        g_watchdog_status.hardware_updates++;
        g_last_hardware_update_ms = get_time_ms();
        
        if (g_debug_enabled) {
            printf("Watchdog: Hardware watchdog updated (count: %lu)\n", 
                   g_watchdog_status.hardware_updates);
        }
    }
}

/**
 * Check if a core is responsive based on its last heartbeat
 */
static bool is_core_responsive(uint32_t last_heartbeat_ms, uint32_t current_time_ms) {
    if (last_heartbeat_ms == 0) {
        // Core hasn't sent any heartbeats yet
        return false;
    }
    
    uint32_t time_since_heartbeat = current_time_ms - last_heartbeat_ms;
    return time_since_heartbeat <= WATCHDOG_CORE_TIMEOUT_MS;
}

/**
 * Handle timeout warning
 */
static void handle_timeout_warning(int core_num, uint32_t time_since_heartbeat) {
    g_watchdog_status.timeout_warnings++;
    
    printf("WATCHDOG WARNING: Core %d unresponsive for %lu ms (timeout: %d ms)\n",
           core_num, time_since_heartbeat, WATCHDOG_CORE_TIMEOUT_MS);
    
    if (time_since_heartbeat > WATCHDOG_CORE_TIMEOUT_MS * 2) {
        printf("WATCHDOG CRITICAL: Core %d severely unresponsive, system reset imminent!\n", core_num);
    }
}

/**
 * Check inter-core health and update status
 */
static void check_inter_core_health(void) {
    if (!WATCHDOG_ENABLE_INTER_CORE) {
        return;
    }
    
    uint32_t current_time = get_time_ms();
    
    // Update heartbeat times from volatile variables
    g_watchdog_status.core0_last_heartbeat_ms = g_core0_heartbeat_timestamp;
    g_watchdog_status.core1_last_heartbeat_ms = g_core1_heartbeat_timestamp;
    
    // Check core 0 responsiveness
    bool core0_was_responsive = g_watchdog_status.core0_responsive;
    g_watchdog_status.core0_responsive = is_core_responsive(
        g_watchdog_status.core0_last_heartbeat_ms, current_time);
    
    if (!g_watchdog_status.core0_responsive && core0_was_responsive) {
        uint32_t time_since = current_time - g_watchdog_status.core0_last_heartbeat_ms;
        handle_timeout_warning(0, time_since);
    }
    
    // Check core 1 responsiveness
    bool core1_was_responsive = g_watchdog_status.core1_responsive;
    g_watchdog_status.core1_responsive = is_core_responsive(
        g_watchdog_status.core1_last_heartbeat_ms, current_time);
    
    if (!g_watchdog_status.core1_responsive && core1_was_responsive) {
        uint32_t time_since = current_time - g_watchdog_status.core1_last_heartbeat_ms;
        handle_timeout_warning(1, time_since);
    }
    
    // Update overall system health
    g_watchdog_status.system_healthy = 
        g_watchdog_status.core0_responsive && g_watchdog_status.core1_responsive;
    
    // If system is unhealthy for too long, force reset
    if (!g_watchdog_status.system_healthy) {
        static uint32_t unhealthy_start_time = 0;
        if (unhealthy_start_time == 0) {
            unhealthy_start_time = current_time;
        } else if (current_time - unhealthy_start_time > WATCHDOG_CORE_TIMEOUT_MS * 2) {
            printf("WATCHDOG FATAL: System unhealthy for too long, forcing reset!\n");
            watchdog_force_reset();
        }
    } else {
        // (No action needed - timer is reset by healthy state)
    }
}

//--------------------------------------------------------------------+
// PUBLIC API IMPLEMENTATION
//--------------------------------------------------------------------+

void watchdog_init(void) {
    if (g_watchdog_initialized) {
        printf("Watchdog: Already initialized\n");
        return;
    }
    
    // Clear status structure
    memset(&g_watchdog_status, 0, sizeof(g_watchdog_status));
    
    // Initialize timestamps
    g_core0_heartbeat_timestamp = 0;
    g_core1_heartbeat_timestamp = 0;
    g_last_hardware_update_ms = 0;
    
    // Set initial status
    g_watchdog_status.system_healthy = false;
    g_watchdog_status.core0_responsive = false;
    g_watchdog_status.core1_responsive = false;
    
    g_watchdog_initialized = true;
    
    if (g_debug_enabled) {
        printf("Watchdog: Initialized successfully\n");
        printf("Watchdog: Hardware timeout: %d ms\n", WATCHDOG_HARDWARE_TIMEOUT_MS);
        printf("Watchdog: Core timeout: %d ms\n", WATCHDOG_CORE_TIMEOUT_MS);
        printf("Watchdog: Update interval: %d ms\n", WATCHDOG_UPDATE_INTERVAL_MS);
        printf("Watchdog: Heartbeat interval: %d ms\n", WATCHDOG_HEARTBEAT_INTERVAL_MS);
    }
}

void watchdog_start(void) {
    if (!g_watchdog_initialized) {
        printf("Watchdog: ERROR - Not initialized, call watchdog_init() first\n");
        return;
    }
    
    if (g_watchdog_started) {
        printf("Watchdog: Already started\n");
        return;
    }
    
    // Extended delay to ensure system is fully stable before starting watchdog
    // This is critical for cold boot scenarios where initialization takes longer
    printf("Watchdog: Waiting for extended system stabilization (cold boot safety)...\n");
    printf("Extended stabilization progress:\n");
    for (int i = 0; i < 30; i++) {
        printf("Stabilization: %d/30 (%.1fs remaining)\n", i+1, (30-i-1) * 0.1f);
        watchdog_core0_heartbeat();  // Send heartbeat during wait
        // Blink LED during extended stabilization - very slow blink
        gpio_put(PIN_LED, (i % 4 < 2) ? 1 : 0);  // 2 on, 2 off pattern
        sleep_ms(100);
    }
    gpio_put(PIN_LED, 1);  // LED on after stabilization
    printf("Extended stabilization complete\n");
    
    if (WATCHDOG_ENABLE_HARDWARE) {
        // Enable hardware watchdog with specified timeout
        printf("Watchdog: Enabling hardware watchdog with %d ms timeout...\n",
               WATCHDOG_HARDWARE_TIMEOUT_MS);
        
        // Try enabling watchdog with retry logic for cold boot robustness
        bool watchdog_enabled = false;
        for (int attempt = 0; attempt < 3; attempt++) {
            watchdog_enable(WATCHDOG_HARDWARE_TIMEOUT_MS, true);
            sleep_ms(100);  // Small delay to let it settle
            watchdog_enabled = true;  // Assume success since watchdog_enable doesn't return status
            printf("Watchdog: Hardware watchdog enabled (attempt %d)\n", attempt + 1);
            break;
        }
        
        if (!watchdog_enabled) {
            printf("Watchdog: WARNING - Failed to enable hardware watchdog\n");
        }
    }
    
    g_watchdog_started = true;
    g_last_hardware_update_ms = get_time_ms();
    
    // Send initial heartbeats to establish baseline with delay
    watchdog_core0_heartbeat();
    sleep_ms(100);
    
    // Give cores time to start sending regular heartbeats
    printf("Watchdog: Allowing cores to establish heartbeat rhythm...\n");
    for (int i = 0; i < 20; i++) {
        printf("Heartbeat rhythm establishment: %d/20 (%.1fs remaining)\n", i+1, (20-i-1) * 0.1f);
        watchdog_core0_heartbeat();  // Send heartbeat during wait
        // Blink LED during heartbeat establishment - medium blink
        gpio_put(PIN_LED, (i % 3 < 1) ? 1 : 0);  // 1 on, 2 off pattern
        sleep_ms(100);
    }
    gpio_put(PIN_LED, 1);  // LED on after heartbeat establishment
    printf("Heartbeat rhythm established\n");
    
    if (g_debug_enabled) {
        printf("Watchdog: Started successfully with enhanced cold boot protection\n");
    } else {
        printf("Watchdog: Started successfully\n");
    }
}

void watchdog_stop(void) {
    if (!g_watchdog_started) {
        printf("Watchdog: Not started, nothing to stop\n");
        return;
    }
    
    if (WATCHDOG_ENABLE_HARDWARE) {
        // We can only update it to prevent reset
        printf("Watchdog: WARNING - Hardware watchdog cannot be disabled on RP2040\n");
        printf("Watchdog: Continuing to update hardware watchdog to prevent reset\n");
    }
    
    g_watchdog_started = false;
    
    if (g_debug_enabled) {
        printf("Watchdog: Stopped (hardware watchdog still active)\n");
    }
}

void watchdog_core0_heartbeat(void) {
    if (!g_watchdog_initialized) {
        return;
    }
    
    uint32_t current_time = get_time_ms();
    g_core0_heartbeat_timestamp = current_time;
    g_watchdog_status.core0_heartbeat_count++;
    
    if (g_debug_enabled && (g_watchdog_status.core0_heartbeat_count % 10 == 0)) {
        printf("Watchdog: Core 0 heartbeat #%lu at %lu ms\n", 
               g_watchdog_status.core0_heartbeat_count, current_time);
    }
}

void watchdog_core1_heartbeat(void) {
    if (!g_watchdog_initialized) {
        return;
    }
    
    uint32_t current_time = get_time_ms();
    g_core1_heartbeat_timestamp = current_time;
    g_watchdog_status.core1_heartbeat_count++;
    
    if (g_debug_enabled && (g_watchdog_status.core1_heartbeat_count % 10 == 0)) {
        printf("Watchdog: Core 1 heartbeat #%lu at %lu ms\n", 
               g_watchdog_status.core1_heartbeat_count, current_time);
    }
}

void watchdog_task(void) {
    if (!g_watchdog_initialized || !g_watchdog_started) {
        return;
    }
    
    uint32_t current_time = get_time_ms();
    
    // Update hardware watchdog at regular intervals
    if (current_time - g_last_hardware_update_ms >= WATCHDOG_UPDATE_INTERVAL_MS) {
        update_hardware_watchdog();
    }
    
    // Check inter-core health
    check_inter_core_health();
}

watchdog_status_t watchdog_get_status(void) {
    return g_watchdog_status;
}

bool watchdog_is_system_healthy(void) {
    return g_watchdog_initialized && g_watchdog_status.system_healthy;
}

void watchdog_force_reset(void) {
    printf("Watchdog: FORCING IMMEDIATE SYSTEM RESET!\n");
    printf("Watchdog: Core 0 heartbeats: %lu, Core 1 heartbeats: %lu\n",
           g_watchdog_status.core0_heartbeat_count, g_watchdog_status.core1_heartbeat_count);
    printf("Watchdog: Timeout warnings: %lu\n", g_watchdog_status.timeout_warnings);
    
    // Force immediate reset by causing hardware watchdog timeout
    if (WATCHDOG_ENABLE_HARDWARE) {
        // Stop updating the hardware watchdog and wait for reset
        while (true) {
            tight_loop_contents();
        }
    } else {
        // If hardware watchdog is disabled, use software reset
        watchdog_enable(1, true);  // 1ms timeout
        while (true) {
            tight_loop_contents();
        }
    }
}

void watchdog_set_debug(bool enable) {
    g_debug_enabled = enable;
    printf("Watchdog: Debug output %s\n", enable ? "enabled" : "disabled");
}