#ifndef SINGLE_LAMP_LOG_DIAGNOSTICS_H
#define SINGLE_LAMP_LOG_DIAGNOSTICS_H

#include "JustFloatLog.h"

#include <QPointF>

#include <array>

struct SingleLampShapeDiagnostic
{
    bool valid = false;
    float width = 0.0f;
    float length = 0.0f;
    float angleDeg = 0.0f;
    float nearestBeaconDistancePx = 0.0f;
    bool nearestBeaconDistanceValid = false;
};

struct SingleLampTrackGeometryDiagnostic
{
    bool valid = false;
    QPointF center;
    float centerRoiHalfSize = 0.0f;
};

struct SingleLampCrossCheckDiagnostic
{
    quint8 state = 0;
    quint8 supportCameraMask = 0;
    quint8 roiValidMask = 0;
    quint8 roiHitMask = 0;
    quint8 conflictCameraMask = 0;
    quint8 fullFrameFallbackMask = 0;
    quint8 measuredCameraMask = 0;
    bool projectionEnabled = false;
    bool manualMark = false;
    bool actualRoiMode = false;
};

struct SingleLampFrameStampDiagnostic
{
    quint8 sequenceLow7 = 0;
    bool valid = false;
};

struct SingleLampLogDiagnostics
{
    std::array<SingleLampShapeDiagnostic, 3> lampShapes;
    SingleLampTrackGeometryDiagnostic track;
    SingleLampCrossCheckDiagnostic crossCheck;
    std::array<SingleLampFrameStampDiagnostic, 3> frameStamps;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float heightMm = 0.0f;
    float relativeYawDeg = 0.0f;
    float maxFrameSkewMs = 0.0f;
    bool relativeYawValid = false;
};

class SingleLampLogDecoder
{
public:
    static bool decode(const JustFloatLogRow& row,
                       SingleLampLogDiagnostics* output,
                       QString* errorMessage = nullptr);

    static bool projectCenterToCamera(int cameraIndex,
                                      const QPointF& center,
                                      QPointF* cameraPoint);
    static QString trackStateName(quint8 state);
    static QString cameraMaskText(quint8 mask);
};

#endif
