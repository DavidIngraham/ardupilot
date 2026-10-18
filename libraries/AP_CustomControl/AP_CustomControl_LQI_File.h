#pragma once
// AI-assisted. Bounded, versioned gain-only interchange; no allocation or file I/O here.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

namespace MicroAGU_LQI_File {
static constexpr size_t SIZE = 280;
struct Gains {
    float k[3][19];
    float aw[3][3];
};
static_assert(sizeof(Gains) == 264, "Gain layout requires IEEE binary32 without padding");

inline uint32_t crc32(const void *buffer, size_t size, uint32_t previous = 0)
{
    uint32_t crc = ~previous;
    const auto *bytes = static_cast<const uint8_t *>(buffer);
    for (size_t i = 0; i < size; i++) {
        crc ^= bytes[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

inline uint32_t u32(const uint8_t *p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

inline bool decode(const uint8_t *data, size_t size, uint32_t context, const float (&scales)[3], Gains &out)
{
    if (size != SIZE || memcmp(data, "AGULQI01", 8) != 0 || u32(data+8) != context ||
        u32(data+12) != crc32(data+16, SIZE-16)) {
        return false;
    }
    Gains candidate;
    // Decode explicitly to avoid alignment assumptions and endian-dependent file reads.
    for (size_t i = 0; i < 66; i++) {
        const uint32_t word = u32(data+16+4*i);
        float value;
        memcpy(&value, &word, sizeof(value));
        if (!isfinite(value)) {
            return false;
        }
        if (i < 57) {
            candidate.k[i/19][i%19] = value;
        } else {
            candidate.aw[(i-57)/3][(i-57)%3] = value;
        }
    }
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            float value = 0;
            for (uint8_t k = 0; k < 3; k++) {
                value += candidate.aw[i][k] * scales[k] * candidate.k[k][16+j];
            }
            if (!isfinite(value) || fabsf(value - (i == j ? 1.0f : 0.0f)) > 0.002f) {
                return false;
            }
        }
    }
    out = candidate;
    return true;
}
} // namespace MicroAGU_LQI_File
