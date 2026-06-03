#ifndef FRAME_RENDERER_H
#define FRAME_RENDERER_H

#include "DetectionBoundary.h"
#include "ImageResult.h"
#include "AnnotationModel.h"

#include <QImage>
#include <QPointF>

class FrameRenderer
{
public:
    static QPointF algorithmToImagePoint(float x, float y);
    static QImage render(const QImage& grayImage,
                         const beacon_result_t& result,
                         const QVector<CorrectionShape>& corrections,
                         int scale,
                         bool showOverlay,
                         const DetectionBoundary* boundary = nullptr);
};

#endif
