#ifndef DOWN_GROUND_RANGE_CALIBRATION_H
#define DOWN_GROUND_RANGE_CALIBRATION_H

#include "HorizonCalibration.h"

#include <array>

struct DownGroundRangeModel
{
    bool valid = false;
    QString sourcePath;
    QSize imageSize;
    quint8 cameraId = HorizonCameraDown;
    double centerX = 0.0;
    double centerY = 0.0;
    double normalizationScale = 1.0;
    std::array<double, HorizonFisheyeThetaCoefficientCount> thetaCoefficients = {};
    std::array<double, HorizonFisheyeMatrixCoefficientCount> bodyToCamera = {};
    std::array<double, 3> extrinsicRotationRad = {};
    double effectiveRangeMm = 7000.0;
    double heightBiasMm = 0.0;
    int inlierCount = 0;
    int sampleCount = 0;
    double rmse = 0.0;
    double medianError = 0.0;
    double maxError = 0.0;
    double rollMin = 0.0;
    double rollMax = 0.0;
    double pitchMin = 0.0;
    double pitchMax = 0.0;
    double heightMinMm = 0.0;
    double heightMaxMm = 0.0;
    double azimuthCoverage = 0.0;
};

struct DownGroundRangeFitResult
{
    bool fitted = false;
    bool exportable = false;
    QString error;
    DownGroundRangeModel model;
    QVector<int> inlierFrames;
    QVector<int> outlierFrames;
    QHash<int, double> frameErrors;
};

namespace DownGroundRangeCalibration
{
DownGroundRangeModel defaultModel(const QSize& imageSize);
QVector<QPointF> predictBoundary(const DownGroundRangeModel& model,
                                 double rollDeg,
                                 double pitchDeg,
                                 double heightMm);
double pointDistance(const DownGroundRangeModel& model,
                     double rollDeg,
                     double pitchDeg,
                     double heightMm,
                     const QPointF& point);
DownGroundRangeFitResult fit(const HorizonCalibrationSession& session);
bool loadModel(const QString& path,
               DownGroundRangeModel* model,
               QString* errorMessage = nullptr);
bool exportModel(const DownGroundRangeModel& model,
                 const QString& jsonPath,
                 const QString& headerPath,
                 QString* errorMessage = nullptr);
}

#endif
