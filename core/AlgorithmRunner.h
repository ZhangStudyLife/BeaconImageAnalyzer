#ifndef ALGORITHM_RUNNER_H
#define ALGORITHM_RUNNER_H

#include "beacon_image.h"

#include <QImage>
#include <QLibrary>
#include <QString>

#include <array>

struct AlgorithmProcessProfile
{
    bool valid = false;
    qint64 algorithmNanoseconds = 0;
};

struct AlgorithmDetectionMetrics
{
    bool carLampPixelAreasAvailable = false;
    unsigned char carLampPixelAreaCount = 0;
    std::array<unsigned short, BEACON_MAX_CAR_LAMP_COUNT> carLampPixelAreas = {};
};

namespace AlgorithmProcessProfiler
{
constexpr double TargetCoreMhz = 160.0;

double milliseconds(const AlgorithmProcessProfile& profile);
double estimatedMcuMillisecondsMin(const AlgorithmProcessProfile& profile);
double estimatedMcuMillisecondsMax(const AlgorithmProcessProfile& profile);
QString format(const AlgorithmProcessProfile& profile, double fps);
QString formatCompact(const AlgorithmProcessProfile& profile);
}

class AlgorithmRunner
{
public:
    AlgorithmRunner();
    ~AlgorithmRunner();

    bool loadSourceFile(const QString& sourcePath, const QString& buildDir, QString* errorMessage = nullptr);
    QString sourcePath() const;
    bool usesDynamicLibrary() const;
    void resetTemporal() const;
    beacon_result_t process(const QImage& grayImage) const;
    AlgorithmProcessProfile lastProcessProfile() const;
    AlgorithmDetectionMetrics lastDetectionMetrics() const;
    QImage binaryImage(const QImage& grayImage) const;

private:
    using InitFn = void (*)();
    using ResetTemporalFn = void (*)();
    using ProcessFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W], beacon_result_t*);
    using BinaryFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W],
                              unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W]);
    using CarLampPixelAreasFn = unsigned char (*)(
        unsigned short[BEACON_MAX_CAR_LAMP_COUNT]);

    QString m_sourcePath;
    QLibrary m_library;
    InitFn m_initFn = nullptr;
    ResetTemporalFn m_resetTemporalFn = nullptr;
    ProcessFn m_processFn = nullptr;
    BinaryFn m_binaryFn = nullptr;
    CarLampPixelAreasFn m_carLampPixelAreasFn = nullptr;
    mutable AlgorithmProcessProfile m_lastProcessProfile;
    mutable AlgorithmDetectionMetrics m_lastDetectionMetrics;
};

#endif
