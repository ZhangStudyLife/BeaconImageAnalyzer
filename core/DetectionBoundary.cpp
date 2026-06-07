#include "DetectionBoundary.h"

#include <QtGlobal>
#include <cstring>

namespace
{
constexpr DetectionBoundary FrontBoundary = {
    2U,
    { 6.0f, -13.0f / 8742.0f, 13.0f / 8742.0f, 0.0f },
    1.0f,
    -93.000f,
    94.000f,
    DetectionBoundaryType::Front
};

constexpr DetectionBoundary RearBoundary = {
    2U,
    { 6.0f, -13.0f / 8742.0f, 13.0f / 8742.0f, 0.0f },
    1.0f,
    -93.000f,
    94.000f,
    DetectionBoundaryType::Rear
};

int roundFloatToInt(float value)
{
    if (value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }
    return (int)(value - 0.5f);
}

void copyLegacyCircles(beacon_result_t* result)
{
    result->count = result->beacon_count;
    for (int i = 0; i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        if (i < result->beacon_count)
        {
            result->circles[i] = result->beacons[i];
        }
        else
        {
            std::memset(&result->circles[i], 0, sizeof(result->circles[i]));
        }
    }
}

float boundaryY(const DetectionBoundary& boundary, float x)
{
    if (x < boundary.xMin)
    {
        x = boundary.xMin;
    }
    else if (x > boundary.xMax)
    {
        x = boundary.xMax;
    }

    const float x2 = x * x;
    const float x3 = x2 * x;
    float y = boundary.c[0] + boundary.c[1] * x;
    if (boundary.degree >= 2U)
    {
        y += boundary.c[2] * x2;
    }
    if (boundary.degree >= 3U)
    {
        y += boundary.c[3] * x3;
    }
    return y;
}
}

DetectionBoundaryType DetectionBoundaryRules::typeForCameraIndex(int cameraIndex)
{
    if (cameraIndex == 0)
    {
        return DetectionBoundaryType::Front;
    }
    if (cameraIndex == 2)
    {
        return DetectionBoundaryType::Rear;
    }
    return DetectionBoundaryType::None;
}

const DetectionBoundary* DetectionBoundaryRules::boundaryForType(DetectionBoundaryType type)
{
    if (type == DetectionBoundaryType::Front)
    {
        return &FrontBoundary;
    }
    if (type == DetectionBoundaryType::Rear)
    {
        return &RearBoundary;
    }
    return nullptr;
}

const DetectionBoundary* DetectionBoundaryRules::boundaryForCameraIndex(int cameraIndex)
{
    return boundaryForType(typeForCameraIndex(cameraIndex));
}

QColor DetectionBoundaryRules::colorForType(DetectionBoundaryType type)
{
    if (type == DetectionBoundaryType::Front)
    {
        return QColor(255, 255, 0);
    }
    if (type == DetectionBoundaryType::Rear)
    {
        return QColor(0, 255, 255);
    }
    return QColor();
}

bool DetectionBoundaryRules::keepPoint(const DetectionBoundary* boundary, float x, float y)
{
    if (boundary == nullptr)
    {
        return true;
    }

    const float testY = -y;
    return (testY - boundaryY(*boundary, x)) * boundary->keepSign >= 0.0f;
}

QPointF DetectionBoundaryRules::imagePointForX(const DetectionBoundary& boundary, int pixelX, bool* valid)
{
    const float imageX = BEACON_IMAGE_TARGET_PIXEL_X - (float)pixelX;
    const float imageY = boundaryY(boundary, imageX);
    const int pixelY = roundFloatToInt(BEACON_IMAGE_TARGET_PIXEL_Y - imageY);
    const bool pointValid = pixelY >= 0 && pixelY < BEACON_IMAGE_H;
    if (valid != nullptr)
    {
        *valid = pointValid;
    }
    return QPointF((qreal)pixelX, (qreal)pixelY);
}

beacon_result_t DetectionBoundaryRules::apply(const beacon_result_t& result, const DetectionBoundary* boundary)
{
    if (boundary == nullptr)
    {
        return result;
    }

    beacon_result_t filtered;
    std::memset(&filtered, 0, sizeof(filtered));

    for (int i = 0; i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        const beacon_circle_t& beacon = result.beacons[i];
        if (beacon.valid == 0)
        {
            continue;
        }
        if (!keepPoint(boundary, beacon.x, beacon.y))
        {
            continue;
        }
        if (filtered.beacon_count >= BEACON_MAX_CIRCLE_COUNT)
        {
            break;
        }
        filtered.beacons[filtered.beacon_count] = beacon;
        ++filtered.beacon_count;
    }

    for (int i = 0; i < BEACON_MAX_CAR_LAMP_COUNT; ++i)
    {
        const beacon_rect_t& lamp = result.car_lamps[i];
        if (lamp.valid == 0)
        {
            continue;
        }
        if (!keepPoint(boundary, lamp.cx, lamp.cy))
        {
            continue;
        }
        if (filtered.car_lamp_count >= BEACON_MAX_CAR_LAMP_COUNT)
        {
            break;
        }
        filtered.car_lamps[filtered.car_lamp_count] = lamp;
        ++filtered.car_lamp_count;
    }

    copyLegacyCircles(&filtered);
    return filtered;
}
