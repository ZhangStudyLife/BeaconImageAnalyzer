#include "CoordinateView.h"

#include <QEvent>
#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QToolTip>
#include <QWheelEvent>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr qreal HitRadius = 11.0;
constexpr qreal EdgeMargin = 12.0;
constexpr double Pi = 3.14159265358979323846;
constexpr double ProjectionCenterXBias = -0.20;
constexpr double ProjectionCenterXRollK = 1.325;
constexpr double ProjectionCenterYBias = -4.63;
constexpr double ProjectionCenterYPitchK = 1.334;
constexpr double CameraModelCenterScale = 0.3333333333;
constexpr double CameraModelRadialK4 = 2.40;

const QColor LampColor(255, 92, 132);
const QColor SelectedColor(215, 90, 255);
const QColor ActualVelocityColor(80, 220, 255);
const QColor TargetVelocityColor(255, 170, 70);
const QColor ProjectionCenterColor(110, 255, 160);
constexpr qreal VelocityScale = 35.0;
const QColor CameraColors[3] = {QColor(80, 180, 255), QColor(100, 235, 130), QColor(255, 215, 80)};

QPointF cameraModelPoint(const QPointF& point, float rollDeg, float pitchDeg)
{
    qreal x = point.x() - CameraModelCenterScale *
              (ProjectionCenterXBias + ProjectionCenterXRollK * rollDeg);
    qreal y = point.y() - CameraModelCenterScale *
              (ProjectionCenterYBias + ProjectionCenterYPitchK * pitchDeg);
    const qreal radius = (x * x + y * y) * 0.0001;
    const qreal gain = 1.0 + CameraModelRadialK4 * radius * radius;
    return QPointF(x * gain, y * gain);
}
}

CoordinateView::CoordinateView(Mode mode, QWidget* parent)
    : QWidget(parent),
      m_mode(mode),
      m_defaultView(mode == Mode::CenterMapped
                        ? QRectF(-180.0, -120.0, 360.0, 240.0)
                        : (mode == Mode::CameraModel
                               ? QRectF(-600.0, -250.0, 900.0, 650.0)
                               : QRectF(-3.0, -3.0, 6.0, 6.0))),
      m_view(m_defaultView)
{
    setMinimumSize(360, 270);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
}

void CoordinateView::setFrame(const TelemetryFrame& frame)
{
    m_frame = frame;
    m_hasFrame = true;
    update();
}

void CoordinateView::clearFrame()
{
    m_hasFrame = false;
    m_hitTargets.clear();
    QToolTip::hideText();
    update();
}

void CoordinateView::resetView()
{
    m_view = m_defaultView;
    update();
}

QSize CoordinateView::sizeHint() const
{
    return QSize(520, 340);
}

void CoordinateView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    m_hitTargets.clear();
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(7, 10, 12));
    const QRectF canvas = canvasRect();
    painter.fillRect(canvas, QColor(5, 8, 10));
    drawGrid(&painter, canvas);

    if (!m_hasFrame)
    {
        painter.setPen(QColor(170, 181, 190));
        painter.drawText(canvas, Qt::AlignCenter, QStringLiteral("等待数据"));
        return;
    }

    if (m_mode == Mode::CarPlan3Global)
    {
        for (int camera = 0; camera < 3; ++camera)
        {
            for (int slot = 0; slot < 2; ++slot)
            {
                const BeaconSample& candidate = m_frame.globalCandidates[camera][slot];
                if (candidate.valid)
                {
                    drawGlobalCandidate(&painter, canvas, camera, slot, candidate);
                }
            }
        }
        for (int slot = 0; slot < 4; ++slot)
        {
            const GlobalBeaconSample& beacon = m_frame.globalBeacons[slot];
            if (beacon.valid)
            {
                drawBeacon(&painter, canvas, slot, beacon.x, beacon.y, beacon.area,
                           m_frame.selectedTargetId == slot);
            }
        }
        drawGlobalLamp(&painter, canvas);
    }
    else for (int slot = 0; slot < 3; ++slot)
    {
        if (m_mode == Mode::CenterMapped)
        {
            const FusedBeaconSample& beacon = m_frame.centerBeacons[slot];
            if (beacon.valid)
            {
                drawBeacon(&painter,
                           canvas,
                           slot,
                           beacon.x,
                           beacon.y,
                           beacon.area,
                           m_frame.selectedTargetId == slot);
            }
        }
        else
        {
            const BeaconSample& beacon = m_frame.modelBeacons[slot];
            if (beacon.valid)
            {
                drawBeacon(&painter,
                           canvas,
                           slot,
                           beacon.x,
                           beacon.y,
                           beacon.area,
                           m_frame.selectedTargetId == slot);
            }
        }
    }

    if (m_mode != Mode::CarPlan3Global)
    {
        drawLamp(&painter,
                 canvas,
                 m_mode == Mode::CenterMapped ? m_frame.centerCarLamp : m_frame.modelCarLamp);
    }
    drawProjectionCenter(&painter, canvas);

    if (m_frame.markerActive)
    {
        painter.setBrush(Qt::NoBrush);
        painter.setPen(QPen(QColor(255, 20, 35, 45), 12));
        painter.drawRect(canvas.adjusted(6.0, 6.0, -6.0, -6.0));
        painter.setPen(QPen(QColor(255, 35, 50, 105), 7));
        painter.drawRect(canvas.adjusted(3.5, 3.5, -3.5, -3.5));
        painter.setPen(QPen(QColor(255, 75, 85, 235), 2));
        painter.drawRect(canvas.adjusted(1.0, 1.0, -1.0, -1.0));
    }
}

void CoordinateView::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton)
    {
        m_panning = true;
        m_panStart = event->position().toPoint();
        m_panStartView = m_view;
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void CoordinateView::mouseMoveEvent(QMouseEvent* event)
{
    if (m_panning)
    {
        const QRectF canvas = canvasRect();
        if (canvas.width() > 0.0 && canvas.height() > 0.0)
        {
            const QPointF delta = event->position() - QPointF(m_panStart);
            const qreal xSign = 1.0;
            const qreal ySign = m_mode == Mode::CenterMapped ? 1.0 : -1.0;
            const qreal shiftX = -delta.x() * m_panStartView.width() /
                                 (xSign * canvas.width());
            const qreal shiftY = -delta.y() * m_panStartView.height() /
                                 (ySign * canvas.height());
            m_view = m_panStartView.translated(shiftX, shiftY);
            update();
        }
        return;
    }

    const QPointF position = event->position();
    const HitTarget* nearest = nullptr;
    qreal nearestDistance = std::numeric_limits<qreal>::max();
    for (const HitTarget& target : m_hitTargets)
    {
        if (!target.bounds.contains(position))
        {
            continue;
        }
        const qreal distance = QLineF(position, target.anchor).length();
        if (distance < nearestDistance)
        {
            nearest = &target;
            nearestDistance = distance;
        }
    }
    if (nearest == nullptr)
    {
        QToolTip::hideText();
        return;
    }
    const QPoint offset(position.x() > width() - 240 ? -230 : 14,
                        position.y() > height() - 180 ? -160 : 14);
    QToolTip::showText(mapToGlobal(position.toPoint() + offset),
                       tooltipText(*nearest),
                       this);
}

void CoordinateView::mouseReleaseEvent(QMouseEvent* event)
{
    if ((event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) && m_panning)
    {
        m_panning = false;
        unsetCursor();
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void CoordinateView::wheelEvent(QWheelEvent* event)
{
    const QRectF canvas = canvasRect();
    if (!canvas.contains(event->position()))
    {
        QWidget::wheelEvent(event);
        return;
    }
    const QPointF before = screenToWorld(event->position(), canvas);
    const qreal factor = event->angleDelta().y() > 0 ? 0.82 : 1.22;
    const qreal width = std::clamp(m_view.width() * factor,
                                   m_defaultView.width() * 0.15,
                                   m_defaultView.width() * 12.0);
    const qreal height = width * m_defaultView.height() / m_defaultView.width();
    m_view = QRectF(m_view.center() - QPointF(width * 0.5, height * 0.5),
                    QSizeF(width, height));
    const QPointF after = screenToWorld(event->position(), canvas);
    m_view.translate(before - after);
    update();
    event->accept();
}

void CoordinateView::leaveEvent(QEvent* event)
{
    QToolTip::hideText();
    QWidget::leaveEvent(event);
}

QRectF CoordinateView::canvasRect() const
{
    return QRectF(rect()).adjusted(10.0, 10.0, -10.0, -10.0);
}

QPointF CoordinateView::worldToScreen(const QPointF& point, const QRectF& canvas) const
{
    qreal nx = (point.x() - m_view.left()) / m_view.width();
    qreal ny = (point.y() - m_view.top()) / m_view.height();
    if (m_mode != Mode::CenterMapped)
    {
        ny = 1.0 - ny;
    }
    return QPointF(canvas.left() + nx * canvas.width(),
                   canvas.top() + ny * canvas.height());
}

QPointF CoordinateView::screenToWorld(const QPointF& point, const QRectF& canvas) const
{
    qreal nx = (point.x() - canvas.left()) / canvas.width();
    qreal ny = (point.y() - canvas.top()) / canvas.height();
    if (m_mode != Mode::CenterMapped)
    {
        ny = 1.0 - ny;
    }
    return QPointF(m_view.left() + nx * m_view.width(),
                   m_view.top() + ny * m_view.height());
}

bool CoordinateView::isInsideCanvas(const QPointF& point, const QRectF& canvas) const
{
    return std::isfinite(point.x()) && std::isfinite(point.y()) && canvas.contains(point);
}

QPointF CoordinateView::edgePoint(const QPointF& projected, const QRectF& canvas) const
{
    const QRectF bounds = canvas.adjusted(EdgeMargin, EdgeMargin, -EdgeMargin, -EdgeMargin);
    return QPointF(std::clamp(projected.x(), bounds.left(), bounds.right()),
                   std::clamp(projected.y(), bounds.top(), bounds.bottom()));
}

void CoordinateView::drawGrid(QPainter* painter, const QRectF& canvas) const
{
    painter->setPen(QPen(QColor(255, 255, 255, 25), 1));
    for (int index = 1; index < 10; ++index)
    {
        const qreal x = canvas.left() + canvas.width() * index / 10.0;
        const qreal y = canvas.top() + canvas.height() * index / 10.0;
        painter->drawLine(QPointF(x, canvas.top()), QPointF(x, canvas.bottom()));
        painter->drawLine(QPointF(canvas.left(), y), QPointF(canvas.right(), y));
    }
    const QPointF origin = worldToScreen(QPointF(0.0, 0.0), canvas);
    painter->setPen(QPen(QColor(195, 205, 214, 105), 1));
    if (origin.x() >= canvas.left() && origin.x() <= canvas.right())
    {
        painter->drawLine(QPointF(origin.x(), canvas.top()), QPointF(origin.x(), canvas.bottom()));
    }
    if (origin.y() >= canvas.top() && origin.y() <= canvas.bottom())
    {
        painter->drawLine(QPointF(canvas.left(), origin.y()), QPointF(canvas.right(), origin.y()));
    }
    painter->setPen(QPen(QColor(104, 116, 126), 1));
    painter->drawRect(canvas.adjusted(0.5, 0.5, -0.5, -0.5));
}

void CoordinateView::drawProjectionCenter(QPainter* painter, const QRectF& canvas) const
{
    if (!std::isfinite(m_frame.aircraftRollDeg) ||
        !std::isfinite(m_frame.aircraftPitchDeg))
    {
        return;
    }
    QPointF point = m_mode == Mode::CarPlan3Global
                        ? QPointF(0.0, 0.0)
                        : QPointF(ProjectionCenterXBias + ProjectionCenterXRollK * m_frame.aircraftRollDeg,
                                  ProjectionCenterYBias + ProjectionCenterYPitchK * m_frame.aircraftPitchDeg);
    if (m_mode == Mode::CameraModel)
    {
        point = cameraModelPoint(point, m_frame.aircraftRollDeg, m_frame.aircraftPitchDeg);
    }
    const QPointF projected = worldToScreen(point, canvas);
    if (!isInsideCanvas(projected, canvas))
    {
        return;
    }
    painter->save();
    painter->setClipRect(canvas);
    painter->setPen(QPen(ProjectionCenterColor, 3, Qt::SolidLine, Qt::RoundCap));
    painter->drawLine(projected + QPointF(-7.0, -7.0), projected + QPointF(7.0, 7.0));
    painter->drawLine(projected + QPointF(-7.0, 7.0), projected + QPointF(7.0, -7.0));
    painter->restore();
}

void CoordinateView::drawBeacon(QPainter* painter,
                                const QRectF& canvas,
                                int slot,
                                float x,
                                float y,
                                float area,
                                bool selected)
{
    const QPointF projected = worldToScreen(QPointF(x, y), canvas);
    if (!isInsideCanvas(projected, canvas))
    {
        drawEdgeIndicator(painter, canvas, projected, Qt::white, HitType::Beacon, slot);
        return;
    }

    const qreal scale = std::min(canvas.width() / m_view.width(),
                                 canvas.height() / m_view.height());
    const qreal radius = m_mode == Mode::CarPlan3Global
                             ? std::clamp(std::sqrt(std::max(0.0f, area) / Pi) * 0.8, 4.0, 18.0)
                             : std::clamp(std::sqrt(std::max(0.0f, area) / Pi) * scale, 4.0, 30.0);
    painter->save();
    painter->setClipRect(canvas);
    painter->setPen(QPen(Qt::white, 1));
    painter->setBrush(Qt::white);
    painter->drawEllipse(projected, radius, radius);
    if (selected)
    {
        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(SelectedColor, 3));
        painter->drawEllipse(projected, radius + 7.0, radius + 7.0);
        painter->drawLine(projected + QPointF(-radius - 11.0, 0.0),
                          projected + QPointF(radius + 11.0, 0.0));
        painter->drawLine(projected + QPointF(0.0, -radius - 11.0),
                          projected + QPointF(0.0, radius + 11.0));
    }
    painter->restore();
    m_hitTargets.push_back({QRectF(projected - QPointF(radius + HitRadius, radius + HitRadius),
                                   QSizeF((radius + HitRadius) * 2.0,
                                          (radius + HitRadius) * 2.0)),
                            projected,
                            HitType::Beacon,
                            slot,
                            -1,
                            false});
}

void CoordinateView::drawGlobalCandidate(QPainter* painter,
                                         const QRectF& canvas,
                                         int camera,
                                         int slot,
                                         const BeaconSample& beacon)
{
    const QPointF projected = worldToScreen(QPointF(beacon.x, beacon.y), canvas);
    if (!isInsideCanvas(projected, canvas))
    {
        drawEdgeIndicator(painter, canvas, projected, CameraColors[camera], HitType::Candidate,
                          camera * 2 + slot);
        return;
    }
    painter->save();
    painter->setClipRect(canvas);
    painter->setPen(QPen(CameraColors[camera], 2));
    painter->setBrush(Qt::NoBrush);
    painter->drawEllipse(projected, 5.0, 5.0);
    painter->restore();
    m_hitTargets.push_back({QRectF(projected - QPointF(HitRadius, HitRadius),
                                   QSizeF(HitRadius * 2.0, HitRadius * 2.0)),
                            projected, HitType::Candidate, slot, camera, false});
}

void CoordinateView::drawGlobalLamp(QPainter* painter, const QRectF& canvas)
{
    const GlobalLampSample& lamp = m_frame.globalCarLamp;
    if (!lamp.valid)
    {
        return;
    }
    const QPointF center(lamp.x, lamp.y);
    const QPointF projected = worldToScreen(center, canvas);
    if (!isInsideCanvas(projected, canvas))
    {
        drawEdgeIndicator(painter, canvas, projected, LampColor, HitType::CarLamp, -1);
        return;
    }
    const double radians = qDegreesToRadians(static_cast<double>(lamp.angleDeg));
    double rightX = std::cos(radians);
    double rightY = std::sin(radians);
    if (std::cos(qDegreesToRadians(static_cast<double>(lamp.angleDeg - m_frame.carYawDeg - 90.0f))) < 0.0)
    {
        rightX = -rightX;
        rightY = -rightY;
    }
    const QPointF right(rightX, rightY);
    const QPointF forward(rightY, -rightX);
    const QLineF axis(worldToScreen(center - right * 0.18, canvas),
                      worldToScreen(center + right * 0.18, canvas));
    painter->save();
    painter->setClipRect(canvas);
    painter->setPen(QPen(LampColor, 5, Qt::SolidLine, Qt::RoundCap));
    painter->drawLine(axis);
    painter->setBrush(LampColor);
    painter->drawEllipse(projected, 6.0, 6.0);
    drawVelocityArrow(painter, canvas, center,
                      right * m_frame.carActualVelocityX + forward * m_frame.carActualVelocityY,
                      ActualVelocityColor);
    drawVelocityArrow(painter, canvas, center,
                      right * m_frame.carTargetVelocityX + forward * m_frame.carTargetVelocityY,
                      TargetVelocityColor);
    painter->restore();
    m_hitTargets.push_back({QRectF(axis.p1(), axis.p2()).normalized().adjusted(-HitRadius, -HitRadius,
                                                                                HitRadius, HitRadius),
                            projected, HitType::CarLamp, -1, -1, false});
}

void CoordinateView::drawLamp(QPainter* painter,
                              const QRectF& canvas,
                              const CarLampSample& lamp)
{
    if (!lamp.valid)
    {
        return;
    }
    const QPointF centerWorld(lamp.cx, lamp.cy);
    const QPointF projected = worldToScreen(centerWorld, canvas);
    if (!isInsideCanvas(projected, canvas))
    {
        drawEdgeIndicator(painter, canvas, projected, LampColor, HitType::CarLamp, -1);
        return;
    }

    const double radians = qDegreesToRadians(static_cast<double>(lamp.angle));
    const QPointF offset(std::cos(radians) * lamp.length * 0.5,
                         std::sin(radians) * lamp.length * 0.5);
    const QLineF axis(worldToScreen(centerWorld - offset, canvas),
                      worldToScreen(centerWorld + offset, canvas));
    painter->save();
    painter->setClipRect(canvas);
    painter->setPen(QPen(QColor(LampColor.red(), LampColor.green(), LampColor.blue(), 65),
                         13,
                         Qt::SolidLine,
                         Qt::RoundCap));
    painter->drawLine(axis);
    painter->setPen(QPen(LampColor, 5, Qt::SolidLine, Qt::RoundCap));
    painter->drawLine(axis);
    painter->setPen(QPen(QColor(255, 210, 226), 2));
    painter->setBrush(LampColor);
    painter->drawEllipse(axis.p1(), 4.0, 4.0);
    painter->drawEllipse(axis.p2(), 4.0, 4.0);
    painter->setPen(QPen(QColor(255, 230, 238), 2));
    painter->setBrush(QColor(LampColor.red(), LampColor.green(), LampColor.blue(), 120));
    painter->drawEllipse(projected, 7.0, 7.0);
    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(255, 240, 245));
    painter->drawEllipse(projected, 2.5, 2.5);
    if (m_mode == Mode::CameraModel)
    {
        const double radians = qDegreesToRadians(static_cast<double>(lamp.angle));
        double lineX = std::cos(radians);
        double lineY = std::sin(radians);
        const double directionCheck = qDegreesToRadians(
            static_cast<double>(m_frame.centerCarLamp.angle -
                                m_frame.carYawDeg +
                                m_frame.aircraftYawDeg));
        if (std::cos(directionCheck) < 0.0)
        {
            lineX = -lineX;
            lineY = -lineY;
        }
        const QPointF rightAxis(lineX, lineY);
        const QPointF forwardAxis(lineY, -lineX);
        const QPointF actualVector(
            m_frame.carActualVelocityX * rightAxis.x() +
                m_frame.carActualVelocityY * forwardAxis.x(),
            m_frame.carActualVelocityX * rightAxis.y() +
                m_frame.carActualVelocityY * forwardAxis.y());
        const QPointF targetVector(
            m_frame.carTargetVelocityX * rightAxis.x() +
                m_frame.carTargetVelocityY * forwardAxis.x(),
            m_frame.carTargetVelocityX * rightAxis.y() +
                m_frame.carTargetVelocityY * forwardAxis.y());
        drawVelocityArrow(painter, canvas, centerWorld, actualVector,
                          ActualVelocityColor);
        drawVelocityArrow(painter, canvas, centerWorld, targetVector,
                          TargetVelocityColor);
    }
    painter->restore();
    m_hitTargets.push_back({QRectF(axis.p1(), axis.p2()).normalized().adjusted(-HitRadius,
                                                                                 -HitRadius,
                                                                                 HitRadius,
                                                                                 HitRadius),
                            projected,
                            HitType::CarLamp,
                            -1,
                            -1,
                            false});
}

void CoordinateView::drawVelocityArrow(QPainter* painter,
                                       const QRectF& canvas,
                                       const QPointF& origin,
                                       const QPointF& vector,
                                       const QColor& color)
{
    if (!std::isfinite(vector.x()) || !std::isfinite(vector.y()))
    {
        return;
    }
    const qreal magnitude = std::hypot(vector.x(), vector.y());
    if (magnitude < 0.001)
    {
        return;
    }
    const QPointF endpointWorld = origin + vector *
                                  (m_mode == Mode::CarPlan3Global ? 0.5 : VelocityScale);
    const QPointF start = worldToScreen(origin, canvas);
    const QPointF end = worldToScreen(endpointWorld, canvas);
    if (!isInsideCanvas(start, canvas) && !isInsideCanvas(end, canvas))
    {
        return;
    }
    QLineF line(start, end);
    const qreal length = line.length();
    if (length < 2.0)
    {
        return;
    }
    const QPointF direction = (line.p2() - line.p1()) / length;
    const QPointF normal(-direction.y(), direction.x());
    const qreal head = std::min<qreal>(12.0, length * 0.45);
    const QPolygonF arrow({line.p2(),
                           line.p2() - direction * head + normal * head * 0.45,
                           line.p2() - direction * head - normal * head * 0.45});
    painter->setPen(QPen(color, 3, Qt::SolidLine, Qt::RoundCap));
    painter->drawLine(line);
    painter->setPen(Qt::NoPen);
    painter->setBrush(color);
    painter->drawPolygon(arrow);
}

void CoordinateView::drawEdgeIndicator(QPainter* painter,
                                       const QRectF& canvas,
                                       const QPointF& projected,
                                       const QColor& color,
                                       HitType type,
                                       int slot)
{
    if (!std::isfinite(projected.x()) || !std::isfinite(projected.y()))
    {
        return;
    }
    const QPointF edge = edgePoint(projected, canvas);
    QPointF direction = projected - edge;
    const qreal length = std::hypot(direction.x(), direction.y());
    if (length < 0.001)
    {
        direction = QPointF(1.0, 0.0);
    }
    else
    {
        direction /= length;
    }
    const QPointF normal(-direction.y(), direction.x());
    const QPolygonF arrow({edge + direction * 8.0,
                           edge - direction * 5.0 + normal * 6.0,
                           edge - direction * 5.0 - normal * 6.0});
    painter->setPen(QPen(color, 2));
    painter->setBrush(color);
    painter->drawPolygon(arrow);
    m_hitTargets.push_back({QRectF(edge - QPointF(HitRadius, HitRadius),
                                   QSizeF(HitRadius * 2.0, HitRadius * 2.0)),
                            edge,
                            type,
                            slot,
                            type == HitType::Candidate ? slot / 2 : -1,
                            true});
}

QString CoordinateView::tooltipText(const HitTarget& target) const
{
    const QString coordinateSystem = m_mode == Mode::CenterMapped
                                         ? QStringLiteral("Center mapped pixel")
                                         : (m_mode == Mode::CameraModel
                                                ? QStringLiteral("CameraModel")
                                                : QStringLiteral("CarPlan3 global (m)"));
    if (target.type == HitType::Candidate)
    {
        const int camera = target.edgeIndicator ? target.slot / 2 : target.camera;
        const int slot = target.edgeIndicator ? target.slot % 2 : target.slot;
        const BeaconSample& beacon = m_frame.globalCandidates[camera][slot];
        return QStringLiteral("Type: projected candidate\nCamera: %1\nSlot: %2\nx: %3 m\ny: %4 m\narea: %5")
            .arg(camera == 0 ? QStringLiteral("Front") : camera == 1 ? QStringLiteral("Center") : QStringLiteral("Back"))
            .arg(slot)
            .arg(beacon.x, 0, 'f', 3)
            .arg(beacon.y, 0, 'f', 3)
            .arg(beacon.area, 0, 'f', 2);
    }
    if (target.type == HitType::Beacon)
    {
        const bool selected = m_frame.selectedTargetId == target.slot;
        if (m_mode == Mode::CenterMapped)
        {
            const FusedBeaconSample& beacon = m_frame.centerBeacons[target.slot];
            return QStringLiteral("Slot: %1\nCoordinate system: %2\nx: %3\ny: %4\narea: %5\nvalid: yes\ncamera_mask: %6\nsource: %7\nselected: %8")
                .arg(target.slot)
                .arg(coordinateSystem)
                .arg(beacon.x, 0, 'f', 2)
                .arg(beacon.y, 0, 'f', 2)
                .arg(beacon.area, 0, 'f', 2)
                .arg(beacon.cameraMask)
                .arg(cameraMaskSource(beacon.cameraMask))
                .arg(selected ? QStringLiteral("yes") : QStringLiteral("no"));
        }
        if (m_mode == Mode::CarPlan3Global)
        {
            const GlobalBeaconSample& beacon = m_frame.globalBeacons[target.slot];
            return QStringLiteral("Slot: %1\nCoordinate system: %2\nx: %3 m\ny: %4 m\narea: %5\ncamera_mask: %6\nsource: %7\nselected: %8")
                .arg(target.slot).arg(coordinateSystem)
                .arg(beacon.x, 0, 'f', 3).arg(beacon.y, 0, 'f', 3)
                .arg(beacon.area, 0, 'f', 2).arg(beacon.cameraMask)
                .arg(cameraMaskSource(beacon.cameraMask))
                .arg(selected ? QStringLiteral("yes") : QStringLiteral("no"));
        }
        const BeaconSample& beacon = m_frame.modelBeacons[target.slot];
        const int mask = m_frame.centerBeacons[target.slot].cameraMask;
        return QStringLiteral("Slot: %1\nCoordinate system: %2\nmodel x: %3\nmodel y: %4\narea: %5\nvalid: yes\ncamera_mask: %6\nsource: %7\nselected: %8")
            .arg(target.slot)
            .arg(coordinateSystem)
            .arg(beacon.x, 0, 'f', 2)
            .arg(beacon.y, 0, 'f', 2)
            .arg(beacon.area, 0, 'f', 2)
            .arg(mask)
            .arg(cameraMaskSource(mask))
            .arg(selected ? QStringLiteral("yes") : QStringLiteral("no"));
    }

    if (m_mode == Mode::CarPlan3Global)
    {
        const GlobalLampSample& lamp = m_frame.globalCarLamp;
        return QStringLiteral("Coordinate system: %1\nType: Car lamp\nx: %2 m\ny: %3 m\nangle: %4 deg\ncamera_mask: %5\nsource: %6\ncyan actual (right, forward): (%7, %8) m/s\norange target (right, forward): (%9, %10) m/s")
            .arg(coordinateSystem).arg(lamp.x, 0, 'f', 3).arg(lamp.y, 0, 'f', 3)
            .arg(lamp.angleDeg, 0, 'f', 2).arg(lamp.cameraMask).arg(cameraMaskSource(lamp.cameraMask))
            .arg(m_frame.carActualVelocityX, 0, 'f', 3).arg(m_frame.carActualVelocityY, 0, 'f', 3)
            .arg(m_frame.carTargetVelocityX, 0, 'f', 3).arg(m_frame.carTargetVelocityY, 0, 'f', 3);
    }
    const CarLampSample& lamp = m_mode == Mode::CenterMapped
                                    ? m_frame.centerCarLamp
                                    : m_frame.modelCarLamp;
    QString text = QStringLiteral("Coordinate system: %1\nType: Car lamp\nx: %2\ny: %3\nangle: %4 deg\nlength: %5\nvalid: yes")
        .arg(coordinateSystem)
        .arg(lamp.cx, 0, 'f', 2)
        .arg(lamp.cy, 0, 'f', 2)
        .arg(lamp.angle, 0, 'f', 2)
        .arg(lamp.length, 0, 'f', 2);
    if (m_mode == Mode::CameraModel)
    {
        text += QStringLiteral("\ncyan actual (right, forward): (%1, %2) m/s"
                               "\norange target (right, forward): (%3, %4) m/s")
                    .arg(m_frame.carActualVelocityX, 0, 'f', 3)
                    .arg(m_frame.carActualVelocityY, 0, 'f', 3)
                    .arg(m_frame.carTargetVelocityX, 0, 'f', 3)
                    .arg(m_frame.carTargetVelocityY, 0, 'f', 3);
    }
    return text;
}

QString CoordinateView::cameraMaskSource(int mask) const
{
    QStringList sources;
    if ((mask & 1) != 0) sources.push_back(QStringLiteral("Front"));
    if ((mask & 2) != 0) sources.push_back(QStringLiteral("Center"));
    if ((mask & 4) != 0) sources.push_back(QStringLiteral("Back"));
    return sources.isEmpty() ? QStringLiteral("None") : sources.join(QStringLiteral(" + "));
}
