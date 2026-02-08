/**
 * DCP Hardware Acceleration Helpers for RP2350
 *
 * Uses the Double Co-Processor for higher precision math.
 */

#ifndef DCP_HELPERS_H
#define DCP_HELPERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

float dcp_adc_to_temp(uint16_t raw_adc);
uint8_t dcp_scale_channel(uint8_t val, float brightness);
uint32_t dcp_apply_brightness_rgb(uint32_t color, float brightness);

#ifdef __cplusplus
}
#endif

#endif // DCP_HELPERS_H
