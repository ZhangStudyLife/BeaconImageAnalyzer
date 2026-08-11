#ifndef ALGORITHM_RUNNER_H
#define ALGORITHM_RUNNER_H

#include "beacon_image.h"

#include <QImage>
#include <QLibrary>
#include <QString>
#include <QVector>

#include <array>

enum class CarLampMode : quint8
{
    Single = 1,
    Dual = 2
};

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

struct AlgorithmFrameTelemetry
{
    quint8 cameraId = 0;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float heightMm = 0.0f;
    bool attitudeValid = false;
    bool heightValid = false;
};

struct AlgorithmHorizonCurve
{
    bool valid = false;
    std::array<float, BEACON_IMAGE_W> y = {};
    std::array<unsigned char, BEACON_IMAGE_W> columnValid = {};
    bool secondaryValid = false;
    std::array<float, BEACON_IMAGE_W> secondaryY = {};
    std::array<unsigned char, BEACON_IMAGE_W> secondaryColumnValid = {};
    bool closedRegion = false;
    std::array<unsigned char, BEACON_IMAGE_W> columnState = {};
};

struct AlgorithmParameterInfo
{
    quint16 id = 0;
    quint8 type = 0;
    float minimum = 0.0f;
    float maximum = 0.0f;
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
    bool loadTwoBl3Firmware(const QString& imageDirectory,
                            const QString& buildDir,
                            QString* errorMessage = nullptr);
    static QString defaultTwoBl3ImageDirectory();
    QString sourcePath() const;
    bool usesDynamicLibrary() const;
    void resetTemporal() const;
    bool supportsCarLampMode() const;
    void setCarLampMode(CarLampMode mode) const;
    CarLampMode carLampMode() const;
    void setFrameTelemetry(const AlgorithmFrameTelemetry& telemetry) const;
    beacon_result_t process(const QImage& grayImage) const;
    AlgorithmHorizonCurve horizonCurve() const;
    AlgorithmProcessProfile lastProcessProfile() const;
    AlgorithmDetectionMetrics lastDetectionMetrics() const;
    QImage binaryImage(const QImage& grayImage) const;
    quint32 processedFrameCount() const;
    bool supportsParameterTuning() const;
    quint32 algorithmBuildId() const;
    QVector<AlgorithmParameterInfo> parameterInfos() const;
    bool parameterValue(quint8 type, quint16 id, quint32* valueBits) const;
    bool setParameterValue(quint8 type, quint16 id, quint32 valueBits, quint32* actualBits) const;

private:
    using InitFn = void (*)();
    using ResetTemporalFn = void (*)();
    using ProcessFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W], beacon_result_t*);
    using BinaryFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W],
                              unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W]);
    using CarLampPixelAreasFn = unsigned char (*)(
        unsigned short[BEACON_MAX_CAR_LAMP_COUNT]);
    using SetTelemetryFn = void (*)(quint8, float, float, float, quint8, quint8);
    using SetCarLampModeFn = void (*)(quint8);
    using HorizonCurveFn = quint8 (*)(float[BEACON_IMAGE_W],
                                      unsigned char[BEACON_IMAGE_W]);
    using HorizonRegionFn = quint8 (*)(unsigned char[BEACON_IMAGE_W]);
    using ProcessedFrameCountFn = quint32 (*)();
    using BuildIdFn = quint32 (*)();
    using ParameterCountFn = quint16 (*)();
    using ParameterInfoFn = int (*)(quint16, quint16*, quint8*, float*, float*);
    using ParameterGetFn = int (*)(quint8, quint16, quint32*);
    using ParameterSetFn = int (*)(quint8, quint16, quint32, quint32*);

    void clearDynamicFunctions();
    bool resolveDynamicFunctions(QString* errorMessage);
    bool loadTwoBl3Library(const QString& libraryPath, QString* errorMessage);

    QString m_sourcePath;
    QLibrary m_library;
    InitFn m_initFn = nullptr;
    ResetTemporalFn m_resetTemporalFn = nullptr;
    ProcessFn m_processFn = nullptr;
    BinaryFn m_binaryFn = nullptr;
    CarLampPixelAreasFn m_carLampPixelAreasFn = nullptr;
    SetTelemetryFn m_setTelemetryFn = nullptr;
    SetCarLampModeFn m_setCarLampModeFn = nullptr;
    HorizonCurveFn m_horizonCurveFn = nullptr;
    HorizonCurveFn m_secondaryHorizonCurveFn = nullptr;
    HorizonRegionFn m_horizonRegionFn = nullptr;
    ProcessedFrameCountFn m_processedFrameCountFn = nullptr;
    BuildIdFn m_buildIdFn = nullptr;
    ParameterCountFn m_parameterCountFn = nullptr;
    ParameterInfoFn m_parameterInfoFn = nullptr;
    ParameterGetFn m_parameterGetFn = nullptr;
    ParameterSetFn m_parameterSetFn = nullptr;
    mutable AlgorithmProcessProfile m_lastProcessProfile;
    mutable AlgorithmDetectionMetrics m_lastDetectionMetrics;
    mutable CarLampMode m_carLampMode = CarLampMode::Single;
};

#endif
