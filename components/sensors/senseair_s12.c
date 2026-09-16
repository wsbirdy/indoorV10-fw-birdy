#include "co2_drivers.h"
#include "senseair_modbus.h"

/*
 * Senseair S12 CO2, per TDE15927 rev 3.
 *
 * Verified against the datasheet: Modbus RTU 9600 8N1 no parity, function code
 * 0x04 for input registers, ErrorStatus at IR1 and the filtered pressure
 * compensated concentration at IR4. Command turnaround is at most 180 ms, well
 * inside the transport's response timeout.
 */

/* Ships at 0x68 (HR20 initial value). Change here if the module was readdressed. */
#define S12_SLAVE       SENSEAIR_SLAVE_S12_DEFAULT

/* IR24 firmware type: "For S12 CO2, fixed at 0x00C2" (TDE15927 section 2.1). */
#define S12_FW_TYPE     0x00C2

static esp_err_t s12_probe(void)
{
    return senseair_modbus_probe_variant(S12_SLAVE, S12_FW_TYPE, "S12");
}

static esp_err_t s12_read_co2(uint16_t *ppm)
{
    return senseair_modbus_read_concentration(S12_SLAVE, SENSEAIR_REG_CO2, ppm);
}

static esp_err_t s12_read_status(uint16_t *status)
{
    return senseair_modbus_read_input_reg(S12_SLAVE, SENSEAIR_REG_STATUS, status);
}

const co2_driver_t senseair_s12_driver = {
    .name        = "Senseair S12",
    .part        = CO2_PART_S12,
    .probe       = s12_probe,
    .read_co2    = s12_read_co2,
    .read_status = s12_read_status,
};
