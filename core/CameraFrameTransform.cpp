#include "CameraFrameTransform.h"

#include <QTransform>
#include <QtGlobal>

bool CameraFrameTransform::rotatesDisplayForCameraIndex(int cameraIndex)
{
    return cameraIndex == 2;
}

QImage CameraFrameTransform::applyDisplayForCameraIndex(const QImage& image, int cameraIndex)
{
    if (image.isNull() || !rotatesDisplayForCameraIndex(cameraIndex))
    {
        return image;
    }

    return image.transformed(QTransform().rotate(180), Qt::FastTransformation);
}

QPointF CameraFrameTransform::imageToDisplayPoint(const QPointF& imagePoint, const QSize& imageSize, int cameraIndex)
{
    if (!rotatesDisplayForCameraIndex(cameraIndex) || imageSize.isEmpty())
    {
        return imagePoint;
    }

    return QPointF((qreal)(imageSize.width() - 1) - imagePoint.x(),
                   (qreal)(imageSize.height() - 1) - imagePoint.y());
}

QPointF CameraFrameTransform::displayToImagePoint(const QPointF& displayPoint, const QSize& imageSize, int cameraIndex)
{
    return imageToDisplayPoint(displayPoint, imageSize, cameraIndex);
}
