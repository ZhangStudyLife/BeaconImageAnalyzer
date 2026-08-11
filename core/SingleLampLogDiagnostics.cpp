#include "SingleLampLogDiagnostics.h"

#include <QStringList>
#include <QtGlobal>

#include <array>
#include <cmath>

namespace
{
constexpr double ImageHalfWidth = 94.0;
constexpr double ImageHalfHeight = 60.0;
constexpr double CenterHalfWidth = 140.0;
constexpr double CenterHalfHeight = 110.0;

using Coefficients = std::array<double, 6>;

const Coefficients CenterToFrontX = {
    4.598873713, 1.112662896, 0.03961500727,
    -0.0009593892413, 0.003638791921, 0.0001953756646
};
const Coefficients CenterToFrontY = {
    57.76555674, -0.04623574104, 0.7418767472,
    -0.003921243931, 0.00009845932112, -0.003075629357
};
const Coefficients CenterToBackX = {
    -11.82017882, -1.112148871, 0.05965474006,
    0.001312940857, 0.003580014712, -0.0005696456455
};
const Coefficients CenterToBackY = {
    58.42283122, -0.04773654331, -0.7173750546,
    -0.003842275563, -0.0004617931, -0.005021741498
};

double evaluate(const Coefficients& coefficients, const QPointF& point)
{
    const double x = point.x();
    const double y = point.y();
    return coefficients[0] + coefficients[1] * x + coefficients[2] * y
           + coefficients[3] * x * x + coefficients[4] * x * y
           + coefficients[5] * y * y;
}

bool pointInRange(const QPointF& point, double halfWidth, double halfHeight)
{
    return std::isfinite(point.x()) && std::isfinite(point.y())
           && point.x() >= -halfWidth && point.x() <= halfWidth
           && point.y() >= -halfHeight && point.y() <= halfHeight;
}
}

bool SingleLampLogDecoder::decode(const JustFloatLogRow& row,
                                  SingleLampLogDiagnostics* output,
                                  QString* errorMessage)
{
    if (output == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("诊断输出为空");
        }
        return false;
    }
    if (row.layout != JustFloatLogLayout::SingleLampRoiV1)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("当前日志不是 SingleLampRoiV1 布局");
        }
        return false;
    }

    SingleLampLogDiagnostics decoded;
    const JustFloatSingleLampRoiFrame& source = row.singleLampRoi;
    decoded.rollDeg = row.roll;
    decoded.pitchDeg = row.pitch;
    decoded.heightMm = source.heightMm;

    for (int camera = 0; camera < 3; ++camera)
    {
        const JustFloatSingleLampShape& sourceShape = source.lampShapes[camera];
        SingleLampShapeDiagnostic& shape = decoded.lampShapes[static_cast<std::size_t>(camera)];
        shape.valid = sourceShape.valid;
        shape.width = sourceShape.width;
        shape.length = sourceShape.length;
        shape.angleDeg = sourceShape.angle;
        shape.nearestBeaconDistanceValid = sourceShape.nearestBeaconDistanceValid;
        shape.nearestBeaconDistancePx = sourceShape.nearestBeaconDistance;
    }

    decoded.track.valid = source.trackGeometry.valid;
    decoded.track.center = QPointF(source.trackGeometry.centerX, source.trackGeometry.centerY);
    decoded.track.centerRoiHalfSize = source.trackGeometry.centerRoiHalfSize;

    decoded.crossCheck.state = source.crossCheck.state;
    decoded.crossCheck.supportCameraMask = source.crossCheck.supportCameraMask;
    decoded.crossCheck.roiValidMask = source.crossCheck.roiValidMask;
    decoded.crossCheck.roiHitMask = source.crossCheck.roiHitMask;
    decoded.crossCheck.conflictCameraMask = source.crossCheck.conflictCameraMask;
    decoded.crossCheck.fullFrameFallbackMask = source.crossCheck.fullFrameFallbackMask;
    decoded.crossCheck.measuredCameraMask = source.crossCheck.measuredCameraMask;
    decoded.crossCheck.projectionEnabled = source.crossCheck.projectionEnabled;
    decoded.crossCheck.manualMark = source.crossCheck.manuallyMarked;
    decoded.crossCheck.actualRoiMode = source.crossCheck.roiMode;

    for (int camera = 0; camera < 3; ++camera)
    {
        decoded.frameStamps[static_cast<std::size_t>(camera)].sequenceLow7 =
            source.sourceFrames[camera].sequenceLow7;
        decoded.frameStamps[static_cast<std::size_t>(camera)].valid =
            source.sourceFrames[camera].valid;
    }

    decoded.relativeYawValid = source.relativeYawValid;
    decoded.relativeYawDeg = source.relativeYawDeg;
    decoded.maxFrameSkewMs = source.maxSkewMs;

    *output = decoded;
    return true;
}

bool SingleLampLogDecoder::projectCenterToCamera(int cameraIndex,
                                                 const QPointF& center,
                                                 QPointF* cameraPoint)
{
    if (cameraPoint == nullptr || cameraIndex < 0 || cameraIndex > 2
        || !pointInRange(center, CenterHalfWidth, CenterHalfHeight))
    {
        return false;
    }

    QPointF projected;
    if (cameraIndex == 1)
    {
        projected = center;
    }
    else if (cameraIndex == 0)
    {
        projected = QPointF(evaluate(CenterToFrontX, center),
                            evaluate(CenterToFrontY, center));
    }
    else
    {
        projected = QPointF(evaluate(CenterToBackX, center),
                            evaluate(CenterToBackY, center));
    }
    if (!pointInRange(projected, ImageHalfWidth, ImageHalfHeight))
    {
        return false;
    }
    *cameraPoint = projected;
    return true;
}

QString SingleLampLogDecoder::trackStateName(quint8 state)
{
    switch (state)
    {
    case 0: return QStringLiteral("SEARCH");
    case 1: return QStringLiteral("ACQUIRE");
    case 2: return QStringLiteral("TRACKED");
    case 3: return QStringLiteral("COAST");
    case 4: return QStringLiteral("LOST");
    default: return QStringLiteral("UNKNOWN(%1)").arg(state);
    }
}

QString SingleLampLogDecoder::cameraMaskText(quint8 mask)
{
    QStringList names;
    if ((mask & 0x01U) != 0U) { names.push_back(QStringLiteral("前")); }
    if ((mask & 0x02U) != 0U) { names.push_back(QStringLiteral("下")); }
    if ((mask & 0x04U) != 0U) { names.push_back(QStringLiteral("后")); }
    return names.isEmpty() ? QStringLiteral("无") : names.join(QLatin1Char('/'));
}
