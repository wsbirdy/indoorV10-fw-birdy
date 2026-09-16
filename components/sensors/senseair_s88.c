#include "co2_drivers.h"
#include "senseair_modbus.h"

/*
 * Senseair S88.
 *
 * Reached at 0xFE, the address the original driver on this board used and the
 * one it was known to work at. Unlike the S12 this variant has no datasheet on
 * hand, so its firmware type constant is unknown and its probe accepts any
 * module that answers -- which is why it must stay last in the table in co2.c.
 *
 * Two things to fix once the S88 datasheet is available: its default address
 * (0xFE is the Senseair production/test address, not normally a part's real
 * one) and S88_FW_TYPE below. The probe logs the firmware type it reads, so a
 * module on the bench will tell you the value.
 */

#define S88_SLAVE       SENSEAIR_SLAVE_TEST

/* 0 = accept any module that answers; see above. */
#define S88_FW_TYPE     0x0000

static esp_err_t s88_probe(void)
{
    return senseair_modbus_probe_variant(S88_SLAVE, S88_FW_TYPE, "S88");
}

static esp_err_t s88_read_co2(uint16_t *ppm)
{
    return senseair_modbus_read_concentration(S88_SLAVE, SENSEAIR_REG_CO2, ppm);
}

static esp_err_t s88_read_status(uint16_t *status)
{
    return senseair_modbus_read_input_reg(S88_SLAVE, SENSEAIR_REG_STATUS, status);
}

const co2_driver_t senseair_s88_driver = {
    .name        = "Senseair S88",
    .part        = CO2_PART_S88,
    .probe       = s88_probe,
    .read_co2    = s88_read_co2,
    .read_status = s88_read_status,
};
