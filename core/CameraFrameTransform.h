#ifndef CAMERA_FRAME_TRANSFORM_H
#define CAMERA_FRAME_TRANSFORM_H

#include <QImage>
#include <QPointF>
#include <QSize>

class CameraFrameTransform
{
public:
    static bool rotatesDisplayForCameraIndex(int cameraIndex);
    static QImage applyDisplayForCameraIndex(const QImage& image, int cameraIndex);
    static QPointF imageToDisplayPoint(const QPointF& imagePoint, const QSize& imageSize, int cameraIndex);
    static QPointF displayToImagePoint(const QPointF& displayPoint, const QSize& imageSize, int cameraIndex);
};

#endif
