#ifndef BEACON_RESULT_UTILS_H
#define BEACON_RESULT_UTILS_H

#include "beacon_image.h"

namespace BeaconResultUtils
{
inline int boundedCount(unsigned char count, int maxCount)
{
    return count < maxCount ? count : maxCount;
}

inline int validCircleCount(const beacon_circle_t* circles, int count)
{
    int validCount = 0;
    for (int i = 0; i < count; ++i)
    {
        if (circles[i].valid != 0)
        {
            ++validCount;
        }
    }
    return validCount;
}

inline int validRectCount(const beacon_rect_t* rects, int count)
{
    int validCount = 0;
    for (int i = 0; i < count; ++i)
    {
        if (rects[i].valid != 0)
        {
            ++validCount;
        }
    }
    return validCount;
}

inline int legacyCircleCount(const beacon_result_t& result)
{
    return validCircleCount(result.circles, boundedCount(result.count, BEACON_MAX_CIRCLE_COUNT));
}

inline int nativeBeaconCount(const beacon_result_t& result)
{
    return validCircleCount(result.beacons, boundedCount(result.beacon_count, BEACON_MAX_BEACON_COUNT));
}

inline int carLampCount(const beacon_result_t& result)
{
    return validRectCount(result.car_lamps,
                          boundedCount(result.car_lamp_count, BEACON_MAX_CAR_LAMP_COUNT));
}

inline int temporalBeaconCount(const beacon_result_t& result)
{
    return validCircleCount(result.temporal_beacons,
                            boundedCount(result.temporal_beacon_count, BEACON_MAX_BEACON_COUNT));
}

inline int temporalCarLampCount(const beacon_result_t& result)
{
    return validRectCount(result.temporal_car_lamps,
                          boundedCount(result.temporal_car_lamp_count, BEACON_MAX_CAR_LAMP_COUNT));
}

inline bool usesLegacyBeacons(const beacon_result_t& result)
{
    return nativeBeaconCount(result) == 0 &&
           carLampCount(result) == 0 &&
           legacyCircleCount(result) > 0;
}

inline int beaconCount(const beacon_result_t& result)
{
    return usesLegacyBeacons(result) ? legacyCircleCount(result) : nativeBeaconCount(result);
}

inline int totalTargetCount(const beacon_result_t& result)
{
    return beaconCount(result) + carLampCount(result);
}
}

#endif
