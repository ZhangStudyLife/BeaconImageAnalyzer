#include "CameraView.h"

#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace
{
constexpr int ImageWidth = 188;
constexpr int ImageHeight = 120;
constexpr double Pi = 3.14159265358979323846;
}

CameraView::CameraView(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(260, 190);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void CameraView::setSample(const CameraSample& sample)
{
    m_sample = sample;
    m_hasSample = true;
    update();
}

void CameraView::clearSample()
{
    m_hasSample = false;
    update();
}

QSize CameraView::sizeHint() const
{
    return QSize(430, 300);
}

void CameraView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

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

    const qreal scale = std::min(canvas.width() / ImageWidth, canvas.height() / ImageHeight);
    const QColor beaconColors[] = {QColor(70, 210, 255), QColor(255, 204, 70)};
    for (int index = 0; index < static_cast<int>(m_sample.beacons.size()); ++index)
    {
        const BeaconSample& beacon = m_sample.beacons[index];
        if (!beacon.valid)
        {
            continue;
        }
        const QPointF center = imagePoint(beacon.x, beacon.y, canvas);
        const qreal radius = std::max(2.0, std::sqrt(beacon.area / Pi) * scale);
        painter.setPen(QPen(beaconColors[index], 2));
        painter.setBrush(QColor(beaconColors[index].red(),
                                beaconColors[index].green(),
                                beaconColors[index].blue(), 55));
        painter.drawEllipse(center, radius, radius);
        painter.drawLine(center + QPointF(-radius - 3, 0), center + QPointF(radius + 3, 0));
        painter.drawLine(center + QPointF(0, -radius - 3), center + QPointF(0, radius + 3));
        painter.setBrush(Qt::NoBrush);
        painter.drawText(center + QPointF(radius + 5, -4), QStringLiteral("B%1").arg(index));
    }

    const CarLampSample& lamp = m_sample.carLamp;
    if (lamp.valid)
    {
        const QPointF center = imagePoint(lamp.cx, lamp.cy, canvas);
        painter.save();
        painter.translate(center);
        painter.rotate(180.0 - lamp.angle);
        const QRectF lampRect(-lamp.length * scale * 0.5,
                              -lamp.width * scale * 0.5,
                              lamp.length * scale,
                              lamp.width * scale);
        painter.setPen(QPen(QColor(255, 92, 105), 2));
        painter.setBrush(QColor(255, 92, 105, 42));
        painter.drawRect(lampRect);
        painter.drawLine(QPointF(0, 0), QPointF(lamp.length * scale * 0.65, 0));
        painter.restore();
        painter.setPen(QColor(255, 120, 130));
        painter.drawText(center + QPointF(6, 15), QStringLiteral("CAR"));
    }
}

QPointF CameraView::imagePoint(float x, float y, const QRectF& canvas) const
{
    const qreal imageX = ImageWidth * 0.5 - x;
    const qreal imageY = ImageHeight * 0.5 + y;
    return QPointF(canvas.left() + imageX * canvas.width() / ImageWidth,
                   canvas.top() + imageY * canvas.height() / ImageHeight);
}
