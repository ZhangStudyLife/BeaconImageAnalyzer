#ifndef HORIZON_CALIBRATION_H
#define HORIZON_CALIBRATION_H

#include <QHash>
#include <QPointF>
#include <QSize>
#include <QString>
#include <QVector>

#include <array>

constexpr int HorizonModelBasisCount = 9;
constexpr int HorizonModelCoefficientCount = HorizonModelBasisCount * 3;
constexpr int HorizonFisheyeThetaCoefficientCount = 3;
constexpr int HorizonFisheyeMatrixCoefficientCount = 9;
constexpr quint8 HorizonCameraFront = 0U;
constexpr quint8 HorizonCameraBack = 1U;
constexpr quint8 HorizonCameraDown = 2U;

struct HorizonFrameMetadata
{
    int frameIndex = -1;
    quint32 bimgSequence = 0;
    qint64 hostTimeMs = 0;
    quint8 cameraId = 0;
    quint8 sourceCameraId = 0xffU;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double heightMm = 0.0;
    bool attitudeValid = false;
    bool heightValid = false;
};

struct HorizonFrameAnnotation
{
    bool skipped = false;
    bool legacyLine = false;
    QVector<QPointF> points;
};

struct HorizonFitResult
{
    bool fitted = false;
    bool exportable = false;
    bool rankValid = false;
    QString error;
    std::array<double, HorizonModelCoefficientCount> coefficients = {};
    QVector<int> inlierFrames;
    QVector<int> outlierFrames;
    QHash<int, double> frameErrors;
    int sampleCount = 0;
    double rmse = 0.0;
    double medianError = 0.0;
    double maxError = 0.0;
    double rollMin = 0.0;
    double rollMax = 0.0;
    double pitchMin = 0.0;
    double pitchMax = 0.0;
};

struct HorizonCalibrationSession
{
    QString sessionPath;
    QString videoPath;
    QString csvPath;
    QString status = QStringLiteral("complete");
    QString error;
    QSize imageSize;
    quint8 cameraId = 0;
    quint8 sourceCameraId = 0xffU;
    int bimgProtocolVersion = 2;
    bool heightRecorded = false;
    int frameCount = 0;
    quint64 sourceDroppedFrames = 0;
    quint64 queueDroppedFrames = 0;
    QVector<HorizonFrameMetadata> frames;
    QHash<int, HorizonFrameAnnotation> annotations;
    HorizonFitResult fit;
};

struct HorizonFisheyeModel
{
    bool valid = false;
    QString sourcePath;
    QSize imageSize;
    quint8 cameraId = 0;
    double centerX = 0.0;
    double centerY = 0.0;
    double normalizationScale = 1.0;
    std::array<double, HorizonFisheyeThetaCoefficientCount> thetaCoefficients = {};
    std::array<double, HorizonFisheyeMatrixCoefficientCount> attitudeToCameraNormal = {};
    int inlierCount = 0;
    int sampleCount = 0;
    double rmse = 0.0;
    double medianError = 0.0;
    double maxError = 0.0;
    double rollMin = 0.0;
    double rollMax = 0.0;
    double pitchMin = 0.0;
    double pitchMax = 0.0;
    bool heightCompensated = false;
    double effectiveDistanceMm = 0.0;
    double heightZeroMm = 0.0;
    double heightMinMm = 0.0;
    double heightMaxMm = 0.0;
};

namespace HorizonCalibration
{
QString cameraName(quint8 cameraId);
std::array<double, 3> gravity(double rollDeg, double pitchDeg);
std::array<double, 3> levelForward(double rollDeg, double pitchDeg);
double evaluate(const std::array<double, HorizonModelCoefficientCount>& coefficients,
                const QSize& imageSize,
                double rollDeg,
                double pitchDeg,
                const QPointF& point);
QVector<QPointF> predictCurve(
    const std::array<double, HorizonModelCoefficientCount>& coefficients,
    const QSize& imageSize,
    double rollDeg,
    double pitchDeg);
double evaluate(const HorizonFisheyeModel& model,
                double rollDeg,
                double pitchDeg,
                const QPointF& point);
QVector<QPointF> predictCurve(const HorizonFisheyeModel& model,
                              double rollDeg,
                              double pitchDeg);
double evaluate(const HorizonFisheyeModel& model,
                double rollDeg,
                double pitchDeg,
                double heightMm,
                const QPointF& point);
QVector<QPointF> predictCurve(const HorizonFisheyeModel& model,
                              double rollDeg,
                              double pitchDeg,
                              double heightMm);
bool loadModel(const QString& path,
               HorizonFisheyeModel* model,
               QString* errorMessage = nullptr);

bool loadSession(const QString& path,
                 HorizonCalibrationSession* session,
                 QString* errorMessage = nullptr);
bool saveSession(const HorizonCalibrationSession& session,
                 QString* errorMessage = nullptr);
HorizonFitResult fit(const HorizonCalibrationSession& session);
bool exportModel(const HorizonCalibrationSession& session,
                 const QString& jsonPath,
                 const QString& headerPath,
                 QString* errorMessage = nullptr);
}

#endif
