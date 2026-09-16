#include "sensirion_crc.h"

uint8_t sensirion_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

bool sensirion_crc8_ok(const uint8_t *word_with_crc)
{
    return sensirion_crc8(word_with_crc, 2) == word_with_crc[2];
}
