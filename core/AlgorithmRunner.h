#ifndef ALGORITHM_RUNNER_H
#define ALGORITHM_RUNNER_H

#include "beacon_image.h"

#include <QImage>
#include <QLibrary>
#include <QString>

struct AlgorithmProcessProfile
{
    bool valid = false;
    qint64 algorithmNanoseconds = 0;
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
    QImage binaryImage(const QImage& grayImage) const;

private:
    using InitFn = void (*)();
    using ResetTemporalFn = void (*)();
    using ProcessFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W], beacon_result_t*);
    using BinaryFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W],
                              unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W]);

    QString m_sourcePath;
    QLibrary m_library;
    InitFn m_initFn = nullptr;
    ResetTemporalFn m_resetTemporalFn = nullptr;
    ProcessFn m_processFn = nullptr;
    BinaryFn m_binaryFn = nullptr;
    mutable AlgorithmProcessProfile m_lastProcessProfile;
};

#endif
