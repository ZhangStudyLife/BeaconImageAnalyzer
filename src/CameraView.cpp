#include "CameraView.h"

#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QToolTip>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr int ImageWidth = 188;
constexpr int ImageHeight = 120;
constexpr double Pi = 3.14159265358979323846;
constexpr qreal HitPadding = 10.0;
constexpr qreal ActualVelocityScale = 18.0;
constexpr int FrontCamera = 0;
constexpr int CenterCamera = 1;

QPointF mapToCenter(int camera, float x, float y)
{
    const float x2 = x * x;
    const float xy = x * y;
    const float y2 = y * y;
    if (camera == CenterCamera)
    {
        return QPointF(x, y);
    }
    if (camera == FrontCamera)
    {
        return QPointF(-3.224193f + 1.123975f * x + 0.003353f * y +
                           0.000073f * x2 - 0.004078f * xy - 0.000302f * y2,
                       -60.512112f + 0.030475f * x + 0.772429f * y +
                           0.004336f * x2 - 0.000232f * xy + 0.004678f * y2);
    }
    return QPointF(-10.828701f - 1.119896f * x + 0.059751f * y -
                       0.000063f * x2 + 0.004186f * xy - 0.000850f * y2,
                   58.428997f - 0.026951f * x - 0.718077f * y -
                       0.004166f * x2 + 0.000106f * xy - 0.004593f * y2);
}

QPointF mapLampVectorToCenter(int camera, float x, float y, const QPointF& vector)
{
    if (camera == CenterCamera)
    {
        return vector;
    }
    if (camera == FrontCamera)
    {
        return QPointF(
            (1.068667486f - 0.000100995f * x - 0.002795043f * y) * vector.x() +
                (0.014106778f - 0.002795043f * x - 0.000353513f * y) * vector.y(),
            (-0.041888826f + 0.008606475f * x + 0.000040255f * y) * vector.x() +
                (0.803140254f + 0.000040255f * x + 0.005562248f * y) * vector.y());
    }
    return QPointF(
        (-1.067786481f + 0.000501382f * x + 0.003736022f * y) * vector.x() +
            (-0.076861896f + 0.003736022f * x + 0.001619549f * y) * vector.y(),
        (0.024195958f - 0.009630155f * x - 0.000515285f * y) * vector.x() +
            (-0.747055821f - 0.000515285f * x - 0.008576754f * y) * vector.y());
}
}

CameraView::CameraView(const QString& cameraName,
                       bool invertX,
                       bool invertY,
                       QWidget* parent)
    : QWidget(parent),
      m_cameraName(cameraName),
      m_invertX(invertX),
      m_invertY(invertY)
{
    setMinimumSize(260, 190);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
}

void CameraView::setFrame(const TelemetryFrame& frame, int cameraIndex, bool markerActive)
{
    m_frame = frame;
    m_cameraIndex = cameraIndex;
    m_hasSample = true;
    m_markerActive = markerActive;
    update();
}

void CameraView::clearSample()
{
    m_hasSample = false;
    m_markerActive = false;
    m_hitTargets.clear();
    QToolTip::hideText();
    update();
}

QSize CameraView::sizeHint() const
{
    return QSize(430, 300);
}

void CameraView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    m_hitTargets.clear();
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(7, 10, 12));

    const QSizeF fitted = QSizeF(ImageWidth, ImageHeight).scaled(size(), Qt::KeepAspectRatio);
    const QRectF canvas((width() - fitted.width()) * 0.5,
                        (height() - fitted.height()) * 0.5,
                        fitted.width(),
                        fitted.height());

    painter.fillRect(canvas, QColor(5, 8, 10));
    painter.setPen(QPen(QColor(255, 255, 255, 28), 1));
    for (int column = 1; column < 8; ++column)
    {
        const qreal x = canvas.left() + canvas.width() * column / 8.0;
        painter.drawLine(QPointF(x, canvas.top()), QPointF(x, canvas.bottom()));
    }
    for (int row = 1; row < 6; ++row)
    {
        const qreal y = canvas.top() + canvas.height() * row / 6.0;
        painter.drawLine(QPointF(canvas.left(), y), QPointF(canvas.right(), y));
    }
    painter.setPen(QPen(QColor(104, 116, 126), 1));
    painter.drawRect(canvas.adjusted(0.5, 0.5, -0.5, -0.5));

    if (!m_hasSample)
    {
        painter.setPen(QColor(170, 181, 190));
        painter.drawText(canvas, Qt::AlignCenter, QStringLiteral("等待数据"));
        return;
    }

    painter.save();
    painter.setClipRect(canvas);
    const qreal scale = std::min(canvas.width() / ImageWidth, canvas.height() / ImageHeight);
    for (int index = 0; index < static_cast<int>(m_frame.cameras[m_cameraIndex].beacons.size()); ++index)
    {
        const BeaconSample& beacon = m_frame.cameras[m_cameraIndex].beacons[index];
        if (!beacon.valid)
        {
            continue;
        }
        const QPointF center = imagePoint(beacon.x, beacon.y, canvas);
        const qreal radius = std::clamp(std::sqrt(std::max(0.0f, beacon.area) / Pi) * scale,
                                        4.0,
                                        30.0);
        painter.setPen(QPen(Qt::white, 1));
        painter.setBrush(Qt::white);
        painter.drawEllipse(center, radius, radius);
        if (isSelectedBeacon(index))
        {
            painter.setBrush(Qt::NoBrush);
            painter.setPen(QPen(QColor(215, 90, 255), 3));
            painter.drawRect(QRectF(center - QPointF(radius + 7.0, radius + 7.0),
                                    QSizeF((radius + 7.0) * 2.0, (radius + 7.0) * 2.0)));
        }
        m_hitTargets.push_back({QRectF(center - QPointF(radius + HitPadding, radius + HitPadding),
                                       QSizeF((radius + HitPadding) * 2.0,
                                              (radius + HitPadding) * 2.0)),
                                center,
                                HitType::Beacon,
                                index});
    }

    const CarLampSample& lamp = m_frame.cameras[m_cameraIndex].carLamp;
    if (lamp.valid)
    {
        const QPointF center = imagePoint(lamp.cx, lamp.cy, canvas);
        const double radians = qDegreesToRadians(static_cast<double>(lamp.angle));
        const QPointF axis(std::cos(radians) * lamp.length * 0.5,
                           std::sin(radians) * lamp.length * 0.5);
        const QLineF line(imagePoint(lamp.cx - axis.x(), lamp.cy - axis.y(), canvas),
                          imagePoint(lamp.cx + axis.x(), lamp.cy + axis.y(), canvas));
        const QColor lampColor(255, 72, 128);
        painter.setPen(QPen(QColor(255, 72, 128, 65), 13, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(line);
        painter.setPen(QPen(lampColor, 5, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(line);
        painter.setPen(QPen(QColor(255, 210, 226), 2));
        painter.setBrush(lampColor);
        painter.drawEllipse(line.p1(), 4.0, 4.0);
        painter.drawEllipse(line.p2(), 4.0, 4.0);
        painter.setPen(QPen(QColor(255, 230, 238), 2));
        painter.setBrush(QColor(255, 72, 128, 120));
        painter.drawEllipse(center, 7.0, 7.0);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 240, 245));
        painter.drawEllipse(center, 2.5, 2.5);
        double lineX = std::cos(radians);
        double lineY = std::sin(radians);
        if (m_frame.centerCarLamp.valid)
        {
            const double fusedRadians = qDegreesToRadians(
                static_cast<double>(m_frame.centerCarLamp.angle));
            const QPointF mappedLine = mapLampVectorToCenter(
                m_cameraIndex, lamp.cx, lamp.cy, QPointF(lineX, lineY));
            const QPointF fusedLine(std::cos(fusedRadians), std::sin(fusedRadians));
            if (QPointF::dotProduct(mappedLine, fusedLine) < 0.0)
            {
                lineX = -lineX;
                lineY = -lineY;
            }
            const double directionCheck = qDegreesToRadians(
                static_cast<double>(m_frame.centerCarLamp.angle -
                                    m_frame.carYawDeg +
                                    m_frame.aircraftYawDeg));
            if (std::cos(directionCheck) < 0.0)
            {
                lineX = -lineX;
                lineY = -lineY;
            }
        }
        const QPointF rightAxis(lineX, lineY);
        const QPointF forwardAxis(lineY, -lineX);
        const QPointF actualVector(
            m_frame.carActualVelocityX * rightAxis.x() +
                m_frame.carActualVelocityY * forwardAxis.x(),
            m_frame.carActualVelocityX * rightAxis.y() +
                m_frame.carActualVelocityY * forwardAxis.y());
        if (QLineF(QPointF(0.0, 0.0), actualVector).length() > 0.01)
        {
            const QPointF endpoint = imagePoint(lamp.cx + actualVector.x() * ActualVelocityScale,
                                                lamp.cy + actualVector.y() * ActualVelocityScale,
                                                canvas);
            painter.setPen(QPen(QColor(80, 220, 255), 3, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(center, endpoint);
            const QLineF arrow(center, endpoint);
            const double arrowAngle = std::atan2(-arrow.dy(), arrow.dx());
            const QPointF left = endpoint - QPointF(std::cos(arrowAngle + 0.55) * 8.0,
                                                    -std::sin(arrowAngle + 0.55) * 8.0);
            const QPointF right = endpoint - QPointF(std::cos(arrowAngle - 0.55) * 8.0,
                                                     -std::sin(arrowAngle - 0.55) * 8.0);
            painter.drawLine(endpoint, left);
            painter.drawLine(endpoint, right);
        }
        m_hitTargets.push_back({QRectF(line.p1(), line.p2()).normalized().adjusted(-HitPadding,
                                                                                     -HitPadding,
                                                                                     HitPadding,
                                                                                     HitPadding),
                                center,
                                HitType::CarLamp,
                                -1});
    }
    painter.restore();

    if (m_markerActive)
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

void CameraView::mouseMoveEvent(QMouseEvent* event)
{
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

    const QPoint offset(position.x() > width() - 220 ? -210 : 14,
                        position.y() > height() - 130 ? -110 : 14);
    QToolTip::showText(mapToGlobal(position.toPoint() + offset),
                       tooltipText(*nearest),
                       this);
}

void CameraView::leaveEvent(QEvent* event)
{
    QToolTip::hideText();
    QWidget::leaveEvent(event);
}

QPointF CameraView::imagePoint(float x, float y, const QRectF& canvas) const
{
    const qreal imageX = ImageWidth * 0.5 + (m_invertX ? -x : x);
    const qreal imageY = ImageHeight * 0.5 + (m_invertY ? -y : y);
    return QPointF(canvas.left() + imageX * canvas.width() / ImageWidth,
                   canvas.top() + imageY * canvas.height() / ImageHeight);
}

QString CameraView::tooltipText(const HitTarget& target) const
{
    if (target.type == HitType::Beacon)
    {
        const BeaconSample& beacon = m_frame.cameras[m_cameraIndex].beacons[target.slot];
        return QStringLiteral("Camera: %1\nSlot: Beacon %2\nx: %3\ny: %4\narea: %5\nvalid: yes")
            .arg(m_cameraName)
            .arg(target.slot)
            .arg(beacon.x, 0, 'f', 2)
            .arg(beacon.y, 0, 'f', 2)
            .arg(beacon.area, 0, 'f', 2);
    }
    const CarLampSample& lamp = m_frame.cameras[m_cameraIndex].carLamp;
    return QStringLiteral("Camera: %1\nType: Car lamp\nx: %2\ny: %3\nangle: %4 deg\nlength: %5\nvalid: yes")
        .arg(m_cameraName)
        .arg(lamp.cx, 0, 'f', 2)
        .arg(lamp.cy, 0, 'f', 2)
        .arg(lamp.angle, 0, 'f', 2)
        .arg(lamp.length, 0, 'f', 2);
}

QPointF CameraView::mapToCenter(float x, float y) const
{
    return ::mapToCenter(m_cameraIndex, x, y);
}

bool CameraView::isSelectedBeacon(int slot) const
{
    if (m_cameraIndex < 0 || m_cameraIndex >= 3 ||
        m_frame.selectedTargetId < 0 || m_frame.selectedTargetId >= 3)
    {
        return false;
    }
    const FusedBeaconSample& target = m_frame.centerBeacons[m_frame.selectedTargetId];
    if (!target.valid || (target.cameraMask & (1 << m_cameraIndex)) == 0)
    {
        return false;
    }
    int nearestSlot = -1;
    double nearestDistance = std::numeric_limits<double>::max();
    for (int index = 0; index < 2; ++index)
    {
        const BeaconSample& beacon = m_frame.cameras[m_cameraIndex].beacons[index];
        if (!beacon.valid)
        {
            continue;
        }
        const QPointF mapped = mapToCenter(beacon.x, beacon.y);
        const double dx = mapped.x() - target.x;
        const double dy = mapped.y() - target.y;
        const double distance = dx * dx + dy * dy;
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestSlot = index;
        }
    }
    return slot == nearestSlot;
}
