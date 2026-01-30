/**
 * Color Tracking Module for KMBox Bridge
 * 
 * Performs color-based blob detection and calculates centroid-based
 * mouse movement deltas from RGB frame data.
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>
#include <stdbool.h>

// Tracker configuration structure
typedef struct {
    uint8_t r_min;          // Minimum red threshold
    uint8_t r_max;          // Maximum red threshold
    uint8_t g_max;          // Maximum green threshold
    uint8_t b_max;          // Maximum blue threshold
    float gain_x;           // Horizontal sensitivity
    float gain_y;           // Vertical sensitivity
    uint8_t deadzone;       // Deadzone in pixels
    uint8_t min_blob_size;  // Minimum blob size to track
} tracker_config_t;

// Tracker result structure
typedef struct {
    bool valid;             // True if a valid target was detected
    int16_t dx;             // Mouse delta X (smoothed, gain-adjusted)
    int16_t dy;             // Mouse delta Y (smoothed, gain-adjusted)
    int16_t cx;             // Raw centroid X offset from center
    int16_t cy;             // Raw centroid Y offset from center
    uint32_t blob_size;     // Number of pixels in detected blob
} tracker_result_t;

// Statistics structure for status reporting
typedef struct {
    int16_t last_dx;        // Last output delta X
    int16_t last_dy;        // Last output delta Y
    uint32_t blob_size;     // Last blob size
    uint16_t fps;           // Current processing FPS
} tracker_stats_t;

/**
 * Initialize the tracker with default configuration
 */
void tracker_init(void);

/**
 * Update tracker configuration
 * @param cfg Pointer to new configuration
 */
void tracker_set_config(const tracker_config_t* cfg);

/**
 * Get current tracker configuration
 * @param cfg Pointer to receive current configuration
 */
void tracker_get_config(tracker_config_t* cfg);

/**
 * Enable or disable tracking
 * @param enabled True to enable, false to disable
 */
void tracker_set_enabled(bool enabled);

/**
 * Check if tracking is currently enabled
 * @return True if enabled, false if disabled
 */
bool tracker_is_enabled(void);

/**
 * Process a frame and calculate mouse movement
 * @param rgb Pointer to RGB888 frame buffer (row-major, width*height*3 bytes)
 * @param w Frame width in pixels
 * @param h Frame height in pixels
 * @param result Pointer to receive tracking result
 */
void tracker_process_frame(const uint8_t* rgb, uint16_t w, uint16_t h, tracker_result_t* result);

/**
 * Get current tracking statistics
 * @param stats Pointer to receive statistics
 */
void tracker_get_stats(tracker_stats_t* stats);

/**
 * Update FPS statistic
 * @param fps Current frames per second
 */
void tracker_update_fps(uint16_t fps);

#endif // TRACKER_H
