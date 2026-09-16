#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief US EPA AQI category, in increasing severity. */
typedef enum {
    AQI_GOOD,
    AQI_MODERATE,
    AQI_SENSITIVE,      // "Unhealthy for Sensitive Groups"
    AQI_UNHEALTHY,
    AQI_VERY_UNHEALTHY,
    AQI_HAZARDOUS,
    AQI_CATEGORY_COUNT,
} aqi_category_t;

/**
 * @brief US EPA AQI for a PM2.5 concentration.
 *
 * Uses the breakpoints from the 2024 revision of the PM NAAQS, which lowered
 * the Good/Moderate boundary from 12.0 to 9.0 ug/m3.
 *
 * @param pm25_ugm3  24-hour PM2.5 in ug/m3. A live reading is not what the
 *                   index is defined over, so treat the result as indicative.
 * @return 0..500, saturating at the top of the scale
 */
int aqi_from_pm25(float pm25_ugm3);

/** @brief Band the index falls in. */
aqi_category_t aqi_category(int aqi);

/** @brief Short label, at most 16 characters so it fits the panel. */
const char *aqi_category_name(aqi_category_t category);

/** @brief The category's official EPA colour, for the LED strip. */
void aqi_category_rgb(aqi_category_t category, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
