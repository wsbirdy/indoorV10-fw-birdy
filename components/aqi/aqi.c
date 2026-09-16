#include "aqi.h"

/*
 * EPA PM2.5 breakpoints, 2024 revision. Concentrations are truncated to one
 * decimal before the lookup, which is what makes the gaps between rows
 * (9.0 -> 9.1) exact rather than approximate.
 */
static const struct {
    float c_lo, c_hi;
    int i_lo, i_hi;
} PM25_BREAKPOINTS[] = {
    {   0.0f,   9.0f,   0,  50 },
    {   9.1f,  35.4f,  51, 100 },
    {  35.5f,  55.4f, 101, 150 },
    {  55.5f, 125.4f, 151, 200 },
    { 125.5f, 225.4f, 201, 300 },
    { 225.5f, 325.4f, 301, 500 },
};

#define BREAKPOINT_COUNT (sizeof(PM25_BREAKPOINTS) / sizeof(PM25_BREAKPOINTS[0]))

int aqi_from_pm25(float pm25_ugm3)
{
    if (pm25_ugm3 <= 0.0f) {
        return 0;
    }

    // EPA rule: truncate to one decimal place before the table lookup.
    const float c = (float)((int)(pm25_ugm3 * 10.0f)) / 10.0f;

    for (unsigned i = 0; i < BREAKPOINT_COUNT; i++) {
        if (c <= PM25_BREAKPOINTS[i].c_hi) {
            const float c_lo = PM25_BREAKPOINTS[i].c_lo;
            const int i_lo = PM25_BREAKPOINTS[i].i_lo;
            const int i_hi = PM25_BREAKPOINTS[i].i_hi;
            const float span = PM25_BREAKPOINTS[i].c_hi - c_lo;
            const float above = (c > c_lo) ? (c - c_lo) : 0.0f;

            return i_lo + (int)(((float)(i_hi - i_lo) * above) / span + 0.5f);
        }
    }

    return 500; // off the top of the EPA table
}

aqi_category_t aqi_category(int aqi)
{
    if (aqi <= 50) {
        return AQI_GOOD;
    } else if (aqi <= 100) {
        return AQI_MODERATE;
    } else if (aqi <= 150) {
        return AQI_SENSITIVE;
    } else if (aqi <= 200) {
        return AQI_UNHEALTHY;
    } else if (aqi <= 300) {
        return AQI_VERY_UNHEALTHY;
    }
    return AQI_HAZARDOUS;
}

const char *aqi_category_name(aqi_category_t category)
{
    switch (category) {
    case AQI_GOOD:              return "GOOD";
    case AQI_MODERATE:          return "MODERATE";
    case AQI_SENSITIVE:         return "SENSITIVE GRP";
    case AQI_UNHEALTHY:         return "UNHEALTHY";
    case AQI_VERY_UNHEALTHY:    return "VERY UNHEALTHY";
    case AQI_HAZARDOUS:         return "HAZARDOUS";
    default:                    return "?";
    }
}

void aqi_category_rgb(aqi_category_t category, uint8_t *r, uint8_t *g, uint8_t *b)
{
    /* The EPA's published category colours. */
    static const uint8_t RGB[AQI_CATEGORY_COUNT][3] = {
        {   0, 228,   0 },  // green
        { 255, 255,   0 },  // yellow
        { 255, 126,   0 },  // orange
        { 255,   0,   0 },  // red
        { 143,  63, 151 },  // purple
        { 126,   0,  35 },  // maroon
    };

    if (category >= AQI_CATEGORY_COUNT) {
        category = AQI_HAZARDOUS;
    }

    *r = RGB[category][0];
    *g = RGB[category][1];
    *b = RGB[category][2];
}
