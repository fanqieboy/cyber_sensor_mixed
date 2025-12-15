#pragma once

#include "cyber_sensor_fixed/data_structs.h"

#include <cstdint>
#include <iostream>
#include <cstring>

namespace CyberSensor {

inline bool findTimeSyncType(const uint8_t *data, size_t len)
{
    if (len < sizeof(CmdHeader)) {
        std::cout << "cmd header length error" << std::endl;
        return false;
    }

    size_t offset = sizeof(CmdHeader);
    size_t total_length = len;

    while (offset < total_length) {
        if (offset + sizeof(ParaInfoHeader) > total_length) {
            std::cout << "total length error" << std::endl;
            break;
        }

        const ParaInfoHeader *header = reinterpret_cast<const ParaInfoHeader*>(data + offset);

        uint16_t key = header->key;
        uint16_t length = header->length;

        if (offset + sizeof(ParaInfoHeader) + length > total_length) {
            std::cout << "total length error" << std::endl;
            break;
        }

        if (key == 0x800c && length == 1) {
            uint8_t value = data[offset + sizeof(ParaInfoHeader)];

            if (value) {
                std::cout << "sync on" << std::endl;
                return true;
            }

            std::cout << "sync off" << std::endl;
            return false;
        }

        offset += sizeof(ParaInfoHeader) + length;
    }

    return false;
}

}