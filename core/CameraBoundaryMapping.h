#ifndef CAMERA_BOUNDARY_MAPPING_H
#define CAMERA_BOUNDARY_MAPPING_H

#include "ImageResult.h"

#include <QPointF>

class CameraBoundaryMapping
{
public:
    static QPointF frontBoundaryToCenter(float frontX)
    {
        return QPointF(frontBoundaryCenterX(frontX), frontBoundaryCenterY(frontX));
    }

    static QPointF rearBoundaryToCenter(float rearX)
    {
        return QPointF(rearBoundaryCenterX(rearX), rearBoundaryCenterY(rearX));
    }

    static float frontBoundaryCenterX(float frontX)
    {
        return frontX;
    }

    static float frontBoundaryCenterY(float)
    {
        return FrontBoundaryCenterY;
    }

    static float rearBoundaryCenterX(float rearX)
    {
        return CenterImageMaxX - rearX;
    }

    static float rearBoundaryCenterY(float)
    {
        return RearBoundaryCenterY;
    }

private:
    static constexpr float CenterImageMaxX = (float)(BEACON_IMAGE_W - 1);
    static constexpr float FrontBoundaryCenterY = 3.0f;
    static constexpr float RearBoundaryCenterY = 102.0f;
};

#endif
