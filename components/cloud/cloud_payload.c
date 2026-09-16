#include <math.h>
#include <string.h>

#include "cloud_payload.h"

#include "cJSON.h"

/*
 * Property names and precision are the AirGradient server contract. They are
 * spelled once, here, so nothing else in the firmware has to know them.
 *
 * Note the two that read oddly: "pm02" is PM2.5, not PM0.2, and "light" is the
 * charging/panel voltage rather than an illuminance. Both are inherited names.
 */
#define PROP_SIGNAL     "wifi"
#define PROP_BOOT       "boot"
#define PROP_CO2        "rco2"
#define PROP_TEMP       "atmp"
#define PROP_RHUM       "rhum"
#define PROP_PM01       "pm01"
#define PROP_PM25       "pm02"
#define PROP_PM10       "pm10"
#define PROP_TVOC_RAW   "tvocRaw"
#define PROP_NOX_RAW    "noxRaw"

/* Board-specific extras, outside the AirGradient contract. See Kconfig. */
#define PROP_PM04       "pm04"
#define PROP_PRESSURE   "pressure"
#define PROP_ALTITUDE   "altitude"
#define PROP_PART_SIZE  "typicalSize"

/* Decimal places, per field family. */
#define DEC_INT         0
#define DEC_PM_MASS     1
#define DEC_TEMP_HUM    2
#define DEC_PRESSURE    1
#define DEC_PART_SIZE   2

/*
 * Round half away from zero. Zero decimals yields an integer-valued double,
 * which cJSON then prints without a decimal point -- that is what keeps the
 * integer-typed properties looking like integers on the wire.
 */
static double round_to(double value, int decimals)
{
    switch (decimals) {
    case 0:  return round(value);
    case 1:  return round(value * 10.0) / 10.0;
    case 2:  return round(value * 100.0) / 100.0;
    case 3:  return round(value * 1000.0) / 1000.0;
    default: return value;
    }
}

/*
 * A driver that reports a NaN has failed in a way its validity flag did not
 * catch; cJSON would serialize it as `null` and the backend would have to cope.
 * Screening here keeps that out of the body.
 */
static void add_float(cJSON *obj, const char *name, float value, int decimals)
{
    if (!isfinite(value)) {
        return;
    }
    cJSON_AddNumberToObject(obj, name, round_to((double)value, decimals));
}

static void add_uint(cJSON *obj, const char *name, uint32_t value)
{
    cJSON_AddNumberToObject(obj, name, (double)value);
}

static void serialize_temp_hum(cJSON *obj, const air_monitor_device *dev)
{
    if (!dev->sht40_valid) {
        return;
    }
    add_float(obj, PROP_TEMP, dev->temperature_c, DEC_TEMP_HUM);
    add_float(obj, PROP_RHUM, dev->humidity_pct, DEC_TEMP_HUM);
}

static void serialize_pm(cJSON *obj, const air_monitor_device *dev)
{
    if (!dev->pm_valid) {
        return;
    }

    // The bound module decides which bins exist: the PMS5003 has no PM4.0 and
    // no size estimate, so those keys simply do not appear on a PMS5003 board.
    const pm_data_t *pm = &dev->pm;

    if (pm_has(pm, PM_FIELD_PM1_0)) {
        add_float(obj, PROP_PM01, pm->pm1_0_ugm3, DEC_PM_MASS);
    }
    if (pm_has(pm, PM_FIELD_PM2_5)) {
        add_float(obj, PROP_PM25, pm->pm2_5_ugm3, DEC_PM_MASS);
    }
    if (pm_has(pm, PM_FIELD_PM10)) {
        add_float(obj, PROP_PM10, pm->pm10_ugm3, DEC_PM_MASS);
    }

#if CONFIG_CLOUD_PAYLOAD_EXTRA_FIELDS
    if (pm_has(pm, PM_FIELD_PM4_0)) {
        add_float(obj, PROP_PM04, pm->pm4_0_ugm3, DEC_PM_MASS);
    }
    if (pm_has(pm, PM_FIELD_TYPICAL_SIZE)) {
        add_float(obj, PROP_PART_SIZE, pm->typical_size_um, DEC_PART_SIZE);
    }
#endif
}

static void serialize_gas(cJSON *obj, const air_monitor_device *dev)
{
    if (dev->co2_valid) {
        add_uint(obj, PROP_CO2, dev->co2_ppm);
    }

    // Only the raw ticks: this build has no gas index algorithm, so the
    // tvocIndex/noxIndex properties have nothing to carry and stay absent.
    // Conditioning counts as "no reading" -- the NOx pixel is meaningless
    // until it completes.
    if (dev->sgp41_valid && !dev->sgp41_conditioning) {
        add_uint(obj, PROP_TVOC_RAW, dev->voc_raw);
        add_uint(obj, PROP_NOX_RAW, dev->nox_raw);
    }
}

#if CONFIG_CLOUD_PAYLOAD_EXTRA_FIELDS
static void serialize_barometric(cJSON *obj, const air_monitor_device *dev)
{
    if (!dev->spl07_valid) {
        return;
    }
    // Reported in hPa: the raw pascals would need six significant digits to
    // stay useful, and no dashboard plots barometric pressure in Pa.
    add_float(obj, PROP_PRESSURE, dev->pressure_pa / 100.0f, DEC_PRESSURE);
    add_float(obj, PROP_ALTITUDE, dev->altitude_m, DEC_INT);
}
#endif

size_t cloud_payload_build(char *out, size_t out_size, const air_monitor_device *dev,
                           int rssi, uint32_t boot_minutes)
{
    if (!out || out_size == 0 || !dev) {
        return 0;
    }

    cJSON *doc = cJSON_CreateObject();
    if (!doc) {
        out[0] = '\0';
        return 0;
    }

    // The two metadata properties are unconditional: they are what makes an
    // otherwise empty post still worth sending, because they prove the device
    // is alive and say how well it can hear the AP.
    const bool metadata_ok =
        cJSON_AddNumberToObject(doc, PROP_SIGNAL, (double)rssi) != NULL &&
        cJSON_AddNumberToObject(doc, PROP_BOOT, (double)boot_minutes) != NULL;

    if (!metadata_ok) {
        cJSON_Delete(doc);
        out[0] = '\0';
        return 0;
    }

    serialize_gas(doc, dev);
    serialize_temp_hum(doc, dev);
    serialize_pm(doc, dev);
#if CONFIG_CLOUD_PAYLOAD_EXTRA_FIELDS
    serialize_barometric(doc, dev);
#endif

    const bool printed = cJSON_PrintPreallocated(doc, out, (int)out_size, /*fmt=*/0);
    cJSON_Delete(doc);

    if (!printed) {
        out[0] = '\0';
        return 0;
    }
    return strlen(out);
}
