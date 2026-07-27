#ifndef VEHICLE_MOTION_UTILS_H
#define VEHICLE_MOTION_UTILS_H

#include "JustFloatLog.h"

#include <QPointF>
#include <QtMath>

#include <cmath>

namespace VehicleMotionUtils
{
constexpr int FrontCameraIndex = 0;
constexpr int CenterCameraIndex = 1;
constexpr int BackCameraIndex = 2;

inline QPointF mapPointToCenter(int cameraIndex, float x, float y)
{
    const double x2 = (double)x * x;
    const double xy = (double)x * y;
    const double y2 = (double)y * y;
    if (cameraIndex == FrontCameraIndex)
    {
        return QPointF(-3.224193 + 1.123975 * x + 0.003353 * y +
                           0.000073 * x2 - 0.004078 * xy - 0.000302 * y2,
                       -60.512112 + 0.030475 * x + 0.772429 * y +
                           0.004336 * x2 - 0.000232 * xy + 0.004678 * y2);
    }
    if (cameraIndex == BackCameraIndex)
    {
        return QPointF(-10.828701 - 1.119896 * x + 0.059751 * y -
                           0.000063 * x2 + 0.004186 * xy - 0.000850 * y2,
                       58.428997 - 0.026951 * x - 0.718077 * y -
                           0.004166 * x2 + 0.000106 * xy - 0.004593 * y2);
    }
    return QPointF(x, y);
}

inline float mapCarLampAngleToCenter(int cameraIndex, float angleDeg)
{
    if (cameraIndex == FrontCameraIndex)
    {
        return 0.098951f - 0.145981f * angleDeg;
    }
    if (cameraIndex == BackCameraIndex)
    {
        return 1.206762f - 0.084711f * angleDeg;
    }
    return angleDeg;
}

inline QPointF velocityToCenter(float strafeMps, float forwardMps, float fusedAngleDeg)
{
    if (!std::isfinite((double)strafeMps) ||
        !std::isfinite((double)forwardMps) ||
        !std::isfinite((double)fusedAngleDeg))
    {
        return QPointF();
    }
    const double radians = qDegreesToRadians((double)fusedAngleDeg);
    const double cosAngle = std::cos(radians);
    const double sinAngle = std::sin(radians);
    return QPointF(strafeMps * cosAngle + forwardMps * sinAngle,
                   strafeMps * sinAngle - forwardMps * cosAngle);
}

class CarLampFusion
{
public:
    void reset()
    {
        m_state = {};
        m_holdTicks = 0;
    }

    JustFloatFusedCarLamp update(const JustFloatLogRow& row)
    {
        int validCount = 0;
        double xSum = 0.0;
        double ySum = 0.0;
        double angleSum = 0.0;
        for (int camera = FrontCameraIndex; camera <= BackCameraIndex; ++camera)
        {
            const JustFloatCarLamp& lamp = row.cameras[camera].carLamp;
            if (!lamp.valid)
            {
                continue;
            }
            const QPointF center = mapPointToCenter(camera, lamp.cx, lamp.cy);
            xSum += center.x();
            ySum += center.y();
            angleSum += mapCarLampAngleToCenter(camera, lamp.angle);
            ++validCount;
        }

        if (validCount == 0)
        {
            if (!m_state.valid)
            {
                m_state = {};
                return m_state;
            }
            if (m_holdTicks < 20)
            {
                ++m_holdTicks;
                return m_state;
            }
            reset();
            return m_state;
        }

        const float cx = static_cast<float>(xSum / validCount);
        const float cy = static_cast<float>(ySum / validCount);
        const float angle = static_cast<float>(angleSum / validCount);
        m_holdTicks = 0;
        if (!m_state.valid)
        {
            m_state = {true, cx, cy, angle};
            return m_state;
        }

        double dx = 0.5 * ((double)cx - m_state.cx);
        double dy = 0.5 * ((double)cy - m_state.cy);
        const double step = std::hypot(dx, dy);
        if (step > 6.0)
        {
            const double scale = 6.0 / step;
            dx *= scale;
            dy *= scale;
        }
        m_state.cx += static_cast<float>(dx);
        m_state.cy += static_cast<float>(dy);
        m_state.angle += 0.5f * (angle - m_state.angle);
        return m_state;
    }

private:
    JustFloatFusedCarLamp m_state;
    int m_holdTicks = 0;
};
}

#endif
