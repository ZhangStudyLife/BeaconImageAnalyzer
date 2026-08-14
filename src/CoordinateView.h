#ifndef COORDINATE_VIEW_H
#define COORDINATE_VIEW_H

#include "TelemetryProtocol.h"

#include <QColor>
#include <QPoint>
#include <QRectF>
#include <QVector>
#include <QWidget>

class CoordinateView : public QWidget
{
    Q_OBJECT

public:
    enum class Mode
    {
        CenterMapped,
        CameraModel,
        CarPlan3Global
    };

    explicit CoordinateView(Mode mode, QWidget* parent = nullptr);
    void setFrame(const TelemetryFrame& frame);
    void clearFrame();
    void resetView();
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    enum class HitType
    {
        Beacon,
        CarLamp,
        Candidate
    };

    struct HitTarget
    {
        QRectF bounds;
        QPointF anchor;
        HitType type = HitType::Beacon;
        int slot = -1;
        int camera = -1;
        bool edgeIndicator = false;
    };

    QRectF canvasRect() const;
    QPointF worldToScreen(const QPointF& point, const QRectF& canvas) const;
    QPointF screenToWorld(const QPointF& point, const QRectF& canvas) const;
    bool isInsideCanvas(const QPointF& point, const QRectF& canvas) const;
    QPointF edgePoint(const QPointF& projected, const QRectF& canvas) const;
    void drawGrid(QPainter* painter, const QRectF& canvas) const;
    void drawProjectionCenter(QPainter* painter, const QRectF& canvas) const;
    void drawBeacon(QPainter* painter,
                    const QRectF& canvas,
                    int slot,
                    float x,
                    float y,
                    float area,
                    bool selected);
    void drawLamp(QPainter* painter, const QRectF& canvas, const CarLampSample& lamp);
    void drawGlobalCandidate(QPainter* painter,
                             const QRectF& canvas,
                             int camera,
                             int slot,
                             const BeaconSample& beacon);
    void drawGlobalLamp(QPainter* painter, const QRectF& canvas);
    void drawVelocityArrow(QPainter* painter,
                           const QRectF& canvas,
                           const QPointF& origin,
                           const QPointF& vector,
                           const QColor& color);
    void drawEdgeIndicator(QPainter* painter,
                           const QRectF& canvas,
                           const QPointF& projected,
                           const QColor& color,
                           HitType type,
                           int slot);
    QString tooltipText(const HitTarget& target) const;
    QString cameraMaskSource(int mask) const;

    Mode m_mode;
    TelemetryFrame m_frame;
    QRectF m_defaultView;
    QRectF m_view;
    QVector<HitTarget> m_hitTargets;
    QPoint m_panStart;
    QRectF m_panStartView;
    bool m_hasFrame = false;
    bool m_panning = false;
};

#endif
