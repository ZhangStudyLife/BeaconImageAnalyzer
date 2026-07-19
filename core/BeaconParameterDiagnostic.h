#ifndef BEACON_PARAMETER_DIAGNOSTIC_H
#define BEACON_PARAMETER_DIAGNOSTIC_H

#include "BimgImageFrameParser.h"
#include "TwoBl3ParameterCatalog.h"
#include "beacon_image.h"

#include <QImage>
#include <QRectF>
#include <QString>
#include <QVector>

struct BeaconDiagnosticRequest
{
    QVector<QImage> frames;
    QRectF region;
    BimgParameterSnapshot snapshot;
    QString firmwareImageDirectory;
    QString buildDirectory;
};

struct BeaconDiagnosticResult
{
    bool completed = false;
    bool recommendationFound = false;
    bool falsePositive = false;
    QString message;
    TwoBl3ParameterDescriptor parameter;
    double currentValue = 0.0;
    double recommendedValue = 0.0;
    int analyzedFrameCount = 0;
    int regionPixelCount = 0;
    double regionMeanGray = 0.0;
    int regionMaxGray = 0;
    beacon_result_t beforeResult = {};
    beacon_result_t afterResult = {};
};

class BeaconParameterDiagnostic
{
public:
    static BeaconDiagnosticResult analyze(const BeaconDiagnosticRequest& request);
};

#endif
