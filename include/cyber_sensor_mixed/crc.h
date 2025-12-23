#pragma once

#include <cstdint>

namespace CyberSensor {

constexpr uint16_t CRC16_POLY = 0x1021;
constexpr uint32_t CRC32_POLY = 0x04c11db7;

inline uint8_t reverse_8(uint8_t data)
{
    data = (data & 0xf0) >> 4 | (data & 0x0f) << 4;
    data = (data & 0xcc) >> 2 | (data & 0x33) << 2;
    data = (data & 0xaa) >> 1 | (data & 0x55) << 1;
    return data;
}

inline uint32_t reverse_32(uint32_t data)
{
    data = ((data >> 1) & 0x55555555) | ((data & 0x55555555) << 1);
    data = ((data >> 2) & 0x33333333) | ((data & 0x33333333) << 2);
    data = ((data >> 4) & 0x0f0f0f0f) | ((data & 0x0f0f0f0f) << 4);
    data = ((data >> 8) & 0x00ff00ff) | ((data & 0x00ff00ff) << 8);
    data = ((data >> 16) & 0x0000ffff) | ((data & 0x0000ffff) << 16);
    return data;
}

inline uint16_t crc16_maker(const uint8_t *data, int length)
{
    uint16_t crc = 0xffff;

    for (int i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ CRC16_POLY) : (crc << 1);
        }
    }

    return crc;
}

inline uint32_t crc32_maker(const uint8_t *data, int length)
{
    uint32_t crc = 0xffffffff;

    for (int i = 0; i < length; ++i) {
        uint8_t byte = reverse_8(data[i]);
        crc ^= static_cast<uint32_t>(byte) << 24;

        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x80000000) ? ((crc << 1) ^ CRC32_POLY) : (crc << 1);
        }
    }

    crc = reverse_32(crc);
    crc ^= 0xffffffff;

    return crc;
}

}
