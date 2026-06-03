#include "CameraFrameTransform.h"

#include <QTransform>

QImage CameraFrameTransform::applyForCameraIndex(const QImage& image, int cameraIndex)
{
    if (image.isNull() || cameraIndex != 2)
    {
        return image;
    }

    return image.transformed(QTransform().rotate(180), Qt::FastTransformation);
}
