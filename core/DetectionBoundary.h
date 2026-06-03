#ifndef DETECTION_BOUNDARY_H
#define DETECTION_BOUNDARY_H

#include "ImageResult.h"

#include <QColor>
#include <QPointF>

enum class DetectionBoundaryType
{
    None,
    Front,
    Rear
};

struct DetectionBoundary
{
    unsigned char degree = 0U;
    float c[4] = {};
    float keepSign = 1.0f;
    DetectionBoundaryType type = DetectionBoundaryType::None;
};

class DetectionBoundaryRules
{
public:
    static DetectionBoundaryType typeForCameraIndex(int cameraIndex);
    static const DetectionBoundary* boundaryForType(DetectionBoundaryType type);
    static const DetectionBoundary* boundaryForCameraIndex(int cameraIndex);
    static QColor colorForType(DetectionBoundaryType type);
    static bool keepPoint(const DetectionBoundary* boundary, float x, float y);
    static QPointF imagePointForX(const DetectionBoundary& boundary, int pixelX, bool* valid = nullptr);
    static beacon_result_t apply(const beacon_result_t& result, const DetectionBoundary* boundary);
};

#endif
