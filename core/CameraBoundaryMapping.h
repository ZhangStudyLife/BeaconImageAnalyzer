#ifndef CAMERA_BOUNDARY_MAPPING_H
#define CAMERA_BOUNDARY_MAPPING_H

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
        return rearPointToCenter(rearX, rearCropBoundaryY(rearX));
    }

    static float frontBoundaryCenterX(float frontX)
    {
        return (((-1.61349009851e-6f * frontX) + 0.000590180490985f) * frontX +
                1.02352403654f) * frontX - 6.08595519353f;
    }

    static float frontBoundaryCenterY(float frontX)
    {
        return (((-2.13888497740e-5f * frontX) + 0.00622926255210f) * frontX -
                0.497913843925f) * frontX + 11.5273738193f;
    }

    static float rearBoundaryCenterX(float rearX)
    {
        return (float)rearBoundaryToCenter(rearX).x();
    }

    static float rearBoundaryCenterY(float rearX)
    {
        return (float)rearBoundaryToCenter(rearX).y();
    }

private:
    struct RearBoundaryMapNode
    {
        float sourceX;
        float sourceY;
        float centerX;
        float centerY;
    };

    static float rearCropBoundaryY(float rearX)
    {
        return ((-7.0f / 4371.0f) * rearX * rearX) + ((1309.0f / 4371.0f) * rearX) + 32.0f;
    }

    static QPointF rearPointToCenter(float rearX, float rearY)
    {
        static constexpr RearBoundaryMapNode nodes[] = {
            {21.0f, 34.0f, 172.0f, 99.0f},
            {30.0f, 35.0f, 164.0f, 98.0f},
            {31.0f, 40.0f, 168.0f, 76.0f},
            {37.0f, 39.0f, 157.0f, 98.0f},
            {100.0f, 41.0f, 85.0f, 107.0f},
            {166.0f, 47.0f, 17.0f, 88.0f},
        };

        float weightSum = 0.0f;
        float centerXSum = 0.0f;
        float centerYSum = 0.0f;

        for (const RearBoundaryMapNode& node : nodes)
        {
            const float dx = rearX - node.sourceX;
            const float dy = rearY - node.sourceY;
            const float d2 = dx * dx + dy * dy;
            if (d2 < 1.0e-4f)
            {
                return QPointF(node.centerX, node.centerY);
            }

            const float weight = 1.0f / (d2 * d2);
            weightSum += weight;
            centerXSum += node.centerX * weight;
            centerYSum += node.centerY * weight;
        }

        return QPointF(centerXSum / weightSum, centerYSum / weightSum);
    }
};

#endif
