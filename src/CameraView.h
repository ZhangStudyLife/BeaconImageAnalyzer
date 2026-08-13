#ifndef CAMERA_VIEW_H
#define CAMERA_VIEW_H

#include "TelemetryProtocol.h"

#include <QWidget>

class CameraView : public QWidget
{
    Q_OBJECT

public:
    explicit CameraView(QWidget* parent = nullptr);
    void setSample(const CameraSample& sample);
    void clearSample();
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPointF imagePoint(float x, float y, const QRectF& canvas) const;

    CameraSample m_sample;
    bool m_hasSample = false;
};

#endif
