#ifndef CAMERA_VIEW_H
#define CAMERA_VIEW_H

#include "TelemetryProtocol.h"

#include <QRectF>
#include <QVector>
#include <QWidget>

class CameraView : public QWidget
{
    Q_OBJECT

public:
    explicit CameraView(const QString& cameraName,
                        bool invertX,
                        bool invertY,
                        QWidget* parent = nullptr);
    void setFrame(const TelemetryFrame& frame, int cameraIndex, bool markerActive);
    void clearSample();
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    enum class HitType
    {
        Beacon,
        CarLamp
    };

    struct HitTarget
    {
        QRectF bounds;
        QPointF anchor;
        HitType type = HitType::Beacon;
        int slot = -1;
    };

    QPointF imagePoint(float x, float y, const QRectF& canvas) const;
    QPointF mapToCenter(float x, float y) const;
    bool isSelectedBeacon(int slot) const;
    QString tooltipText(const HitTarget& target) const;

    QString m_cameraName;
    bool m_invertX = false;
    bool m_invertY = false;
    TelemetryFrame m_frame;
    int m_cameraIndex = -1;
    QVector<HitTarget> m_hitTargets;
    bool m_hasSample = false;
    bool m_markerActive = false;
};

#endif
