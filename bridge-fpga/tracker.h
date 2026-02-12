/**
 * Color Tracking Module for KMBox FPGA Bridge
 *
 * Identical to the standard bridge tracker - performs color-based blob
 * detection and calculates centroid-based mouse movement deltas.
 *
 * This is shared code; changes here should be mirrored in bridge/tracker.h
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t r_min;
    uint8_t r_max;
    uint8_t g_max;
    uint8_t b_max;
    float gain_x;
    float gain_y;
    uint8_t deadzone;
    uint8_t min_blob_size;
} tracker_config_t;

typedef struct {
    bool valid;
    int16_t dx;
    int16_t dy;
    int16_t cx;
    int16_t cy;
    uint32_t blob_size;
} tracker_result_t;

typedef struct {
    int16_t last_dx;
    int16_t last_dy;
    uint32_t blob_size;
    uint16_t fps;
} tracker_stats_t;

void tracker_init(void);
void tracker_set_config(const tracker_config_t* cfg);
void tracker_get_config(tracker_config_t* cfg);
void tracker_set_enabled(bool enabled);
bool tracker_is_enabled(void);
void tracker_process_frame(const uint8_t* rgb, uint16_t w, uint16_t h, tracker_result_t* result);
void tracker_get_stats(tracker_stats_t* stats);
void tracker_update_fps(uint16_t fps);

#endif // TRACKER_H
