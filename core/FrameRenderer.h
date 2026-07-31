#ifndef FRAME_RENDERER_H
#define FRAME_RENDERER_H

#include "beacon_image.h"
#include "AnnotationModel.h"

#include <QImage>
#include <QPointF>

struct AlgorithmHorizonCurve;

class FrameRenderer
{
public:
    static QPointF algorithmToImagePoint(float x, float y);
    static QImage render(const QImage& grayImage,
                         const beacon_result_t& result,
                         const QVector<CorrectionShape>& corrections,
                         int scale,
                         bool showOverlay,
                         const AlgorithmHorizonCurve* horizon = nullptr);
};

#endif
