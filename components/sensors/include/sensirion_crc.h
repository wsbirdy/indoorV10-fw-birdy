#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CRC-8 as used by Sensirion parts (poly 0x31, init 0xFF, no final XOR).
 *
 * Shared by the SHT40 and SGP41 drivers.
 */
uint8_t sensirion_crc8(const uint8_t *data, size_t len);

/** @brief Check a 3-byte group laid out as [MSB, LSB, CRC]. */
bool sensirion_crc8_ok(const uint8_t *word_with_crc);

#ifdef __cplusplus
}
#endif
