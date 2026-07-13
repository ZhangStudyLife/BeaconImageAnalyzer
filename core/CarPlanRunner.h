#ifndef CAR_PLAN_RUNNER_H
#define CAR_PLAN_RUNNER_H

#include "JustFloatLog.h"

#include <QLibrary>
#include <QString>

struct CarPlanResult
{
    bool available = false;
    bool valid = false;
    int camera = 0;
    int beaconIndex = 0;
    float targetStrafeMps = 0.0f;
    float targetForwardMps = 0.0f;
    float distPx = 0.0f;
    float along = 0.0f;
    float perp = 0.0f;
};

class CarPlanRunner
{
public:
    CarPlanRunner() = default;
    ~CarPlanRunner();

    bool loadSourcePath(const QString& path, const QString& buildDir, QString* errorMessage = nullptr);
    bool isLoaded() const;
    QString sourcePath() const;
    void reset() const;
    bool update(const JustFloatLogRow& row, CarPlanResult* result) const;

private:
    struct HostBeacon
    {
        unsigned char valid;
        float x;
        float y;
        float area;
    };

    struct HostCarLamp
    {
        unsigned char valid;
        float cx;
        float cy;
        float angle;
        float width;
        float length;
    };

    struct HostCameraFrame
    {
        HostBeacon beacons[2];
        HostCarLamp carLamp;
    };

    struct HostResult
    {
        unsigned char valid;
        unsigned char camera;
        unsigned char beaconIndex;
        float targetStrafeMps;
        float targetForwardMps;
        float distPx;
        float along;
        float perp;
    };

    using ResetFn = void (*)();
    using UpdateFn = unsigned char (*)(const HostCameraFrame[3], HostResult*);

    QString m_sourcePath;
    QLibrary m_library;
    ResetFn m_resetFn = nullptr;
    UpdateFn m_updateFn = nullptr;
};

#endif
