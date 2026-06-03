#ifndef CAMERA_FRAME_TRANSFORM_H
#define CAMERA_FRAME_TRANSFORM_H

#include <QImage>

class CameraFrameTransform
{
public:
    static QImage applyForCameraIndex(const QImage& image, int cameraIndex);
};

#endif
