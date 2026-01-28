/**
 * Color Tracking Module Implementation
 */

#include "tracker.h"
#include "config.h"
#include <string.h>

static tracker_config_t config;
static tracker_stats_t stats;
static bool enabled = true;

// Smoothing state
static float smooth_dx = 0.0f;
static float smooth_dy = 0.0f;

void tracker_init(void) {
    config.r_min = DEFAULT_R_MIN;
    config.r_max = DEFAULT_R_MAX;
    config.g_max = DEFAULT_G_MAX;
    config.b_max = DEFAULT_B_MAX;
    config.gain_x = DEFAULT_GAIN_X;
    config.gain_y = DEFAULT_GAIN_Y;
    config.deadzone = DEFAULT_DEADZONE;
    config.min_blob_size = DEFAULT_MIN_BLOB;
    
    memset(&stats, 0, sizeof(stats));
    smooth_dx = 0.0f;
    smooth_dy = 0.0f;
    enabled = true;
}

void tracker_set_config(const tracker_config_t* cfg) {
    if (cfg) {
        config = *cfg;
    }
}

void tracker_get_config(tracker_config_t* cfg) {
    if (cfg) {
        *cfg = config;
    }
}

void tracker_set_enabled(bool en) {
    enabled = en;
    if (!en) {
        // Reset smoothing when disabled
        smooth_dx = 0.0f;
        smooth_dy = 0.0f;
    }
}

bool tracker_is_enabled(void) {
    return enabled;
}

void tracker_process_frame(const uint8_t* rgb, uint16_t w, uint16_t h, tracker_result_t* result) {
    result->valid = false;
    result->dx = 0;
    result->dy = 0;
    result->cx = 0;
    result->cy = 0;
    result->blob_size = 0;
    
    if (!rgb || w == 0 || h == 0) {
        return;
    }
    
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    uint32_t count = 0;
    
    // Scan for matching pixels
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            uint32_t idx = (y * w + x) * 3;
            uint8_t r = rgb[idx + 0];
            uint8_t g = rgb[idx + 1];
            uint8_t b = rgb[idx + 2];
            
            // Color match: high red, low green, low blue
            if (r >= config.r_min && r <= config.r_max &&
                g <= config.g_max && b <= config.b_max) {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }
    
    result->blob_size = count;
    stats.blob_size = count;
    
    if (count < config.min_blob_size) {
        // Not enough pixels - no valid target
        // Decay smoothing towards zero
        smooth_dx *= SMOOTHING_FACTOR;
        smooth_dy *= SMOOTHING_FACTOR;
        return;
    }
    
    // Calculate centroid offset from ROI center
    int16_t center_x = w / 2;
    int16_t center_y = h / 2;
    int16_t cx = (int16_t)(sum_x / count) - center_x;
    int16_t cy = (int16_t)(sum_y / count) - center_y;
    
    result->cx = cx;
    result->cy = cy;
    
    // Apply deadzone
    if (cx > -config.deadzone && cx < config.deadzone) {
        cx = 0;
    }
    if (cy > -config.deadzone && cy < config.deadzone) {
        cy = 0;
    }
    
    // Apply gain
    float raw_dx = cx * config.gain_x;
    float raw_dy = cy * config.gain_y;
    
    // Apply smoothing (exponential moving average)
    smooth_dx = smooth_dx * SMOOTHING_FACTOR + raw_dx * (1.0f - SMOOTHING_FACTOR);
    smooth_dy = smooth_dy * SMOOTHING_FACTOR + raw_dy * (1.0f - SMOOTHING_FACTOR);
    
    // Clamp to valid range
    int16_t dx = (int16_t)smooth_dx;
    int16_t dy = (int16_t)smooth_dy;
    
    if (dx > MAX_MOUSE_DELTA) dx = MAX_MOUSE_DELTA;
    if (dx < -MAX_MOUSE_DELTA) dx = -MAX_MOUSE_DELTA;
    if (dy > MAX_MOUSE_DELTA) dy = MAX_MOUSE_DELTA;
    if (dy < -MAX_MOUSE_DELTA) dy = -MAX_MOUSE_DELTA;
    
    result->dx = dx;
    result->dy = dy;
    result->valid = true;
    
    stats.last_dx = dx;
    stats.last_dy = dy;
}

void tracker_get_stats(tracker_stats_t* s) {
    if (s) {
        *s = stats;
    }
}

void tracker_update_fps(uint16_t fps) {
    stats.fps = fps;
}
