#include "crc.h"

uint8_t crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ 0x07;
        else crc <<= 1;
    }
    return crc;
}
