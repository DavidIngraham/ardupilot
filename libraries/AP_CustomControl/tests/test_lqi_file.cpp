// AI-assisted: fixed-size decoder rejects damaged, incompatible and nonfinite files.
#include <AP_gtest.h>
#include "../AP_CustomControl_LQI_File.h"
using namespace MicroAGU_LQI_File;

TEST(LQIFile, ValidAndCorrupt)
{
    uint8_t data[SIZE] {};
    memcpy(data, "AGULQI01", 8);
    const uint32_t context = 123;
    memcpy(data+8, &context, 4);
    Gains input {};
    const float scales[3] = {0.1f, 0.1f, 0.1f};
    for (uint8_t i = 0; i < 3; i++) {
        input.k[i][16+i] = 1;
        input.aw[i][i] = 10;
    }
    memcpy(data+16, &input, sizeof(input));
    uint32_t crc = crc32(data+16, sizeof(input));
    memcpy(data+12, &crc, 4);
    Gains output {};
    ASSERT_TRUE(decode(data, sizeof(data), context, scales, output));
    EXPECT_EQ(output.aw[1][1], 10);
    EXPECT_FALSE(decode(data, sizeof(data)-1, context, scales, output));
    EXPECT_FALSE(decode(data, sizeof(data), context+1, scales, output));
    data[16] ^= 1;
    EXPECT_FALSE(decode(data, sizeof(data), context, scales, output));
    EXPECT_EQ(output.aw[1][1], 10);
    const uint32_t nan = 0x7fc00000;
    memcpy(data+16, &nan, 4);
    crc = crc32(data+16, sizeof(input));
    memcpy(data+12, &crc, 4);
    EXPECT_FALSE(decode(data, sizeof(data), context, scales, output));
}
AP_GTEST_MAIN()
