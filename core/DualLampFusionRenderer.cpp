#include "DualLampFusionRenderer.h"

#include "beacon_image.h"

#include <QColor>
#include <QFont>
#include <QPainter>
#include <QtMath>

#include <cmath>

namespace
{
QPointF imagePoint(float x, float y)
{
    return QPointF(static_cast<float>(BEACON_IMAGE_W) * 0.5f - x,
                   static_cast<float>(BEACON_IMAGE_H) * 0.5f + y);
}

void drawGrid(QPainter* painter)
{
    QPen gridPen(QColor(42, 47, 52));
    gridPen.setWidth(1);
    painter->setPen(gridPen);
    for (int x = -80; x <= 80; x += 20)
    {
        const qreal imageX = imagePoint(static_cast<float>(x), 0.0f).x();
        painter->drawLine(QPointF(imageX, 0.0), QPointF(imageX, BEACON_IMAGE_H - 1.0));
    }
    for (int y = -40; y <= 40; y += 20)
    {
        const qreal imageY = imagePoint(0.0f, static_cast<float>(y)).y();
        painter->drawLine(QPointF(0.0, imageY), QPointF(BEACON_IMAGE_W - 1.0, imageY));
    }
    QPen axisPen(QColor(84, 91, 98));
    axisPen.setWidth(1);
    painter->setPen(axisPen);
    const QPointF origin = imagePoint(0.0f, 0.0f);
    painter->drawLine(QPointF(origin.x(), 0.0), QPointF(origin.x(), BEACON_IMAGE_H - 1.0));
    painter->drawLine(QPointF(0.0, origin.y()), QPointF(BEACON_IMAGE_W - 1.0, origin.y()));
}

void drawPoint(QPainter* painter,
               const JustFloatMappedPoint& point,
               const QColor& color,
               const QString& label)
{
    if (!point.valid)
    {
        return;
    }
    const QPointF center = imagePoint(point.x, point.y);
    QPen pen(color);
    pen.setWidth(2);
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawEllipse(center, 4.0, 4.0);
    painter->drawLine(center + QPointF(-6.0, 0.0), center + QPointF(6.0, 0.0));
    painter->drawLine(center + QPointF(0.0, -6.0), center + QPointF(0.0, 6.0));
    painter->drawText(center + QPointF(6.0, -5.0), label);
}

void drawLamp(QPainter* painter,
              const JustFloatMappedCarLamp& lamp,
              const QColor& color,
              const QString& label)
{
    if (!lamp.valid)
    {
        return;
    }
    const QPointF center = imagePoint(lamp.cx, lamp.cy);
    const qreal radians = qDegreesToRadians(static_cast<qreal>(lamp.angle));
    const QPointF major(qCos(radians) * 10.0, qSin(radians) * 10.0);
    const QPointF minor(-qSin(radians) * 3.0, qCos(radians) * 3.0);
    QPen pen(color);
    pen.setWidth(2);
    pen.setStyle(lamp.measured ? Qt::SolidLine : Qt::DashLine);
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawLine(center - major, center + major);
    painter->drawLine(center - minor, center + minor);
    painter->drawEllipse(center, 2.5, 2.5);
    painter->drawText(center + QPointF(6.0, -6.0), label);
}

void drawAxis(QPainter* painter,
              float cx,
              float cy,
              float angle,
              const QColor& color,
              Qt::PenStyle style)
{
    if (!std::isfinite(static_cast<double>(cx)) ||
        !std::isfinite(static_cast<double>(cy)) ||
        !std::isfinite(static_cast<double>(angle)))
    {
        return;
    }
    const QPointF center = imagePoint(cx, cy);
    const qreal radians = qDegreesToRadians(static_cast<qreal>(angle));
    const QPointF axis(qCos(radians) * 16.0, qSin(radians) * 16.0);
    QPen pen(color);
    pen.setWidth(2);
    pen.setStyle(style);
    painter->setPen(pen);
    painter->drawLine(center - axis, center + axis);
}
}

QImage DualLampFusionRenderer::render(const JustFloatDualLampFusionFrame& frame, int viewIndex)
{
    QImage image(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_RGB32);
    image.fill(QColor(10, 12, 14));
    if (viewIndex < 0 || viewIndex >= ViewCount)
    {
        return image;
    }

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);
    QFont font = painter.font();
    font.setPixelSize(8);
    font.setBold(true);
    painter.setFont(font);
    drawGrid(&painter);

    if (viewIndex < 3)
    {
        const JustFloatMappedCameraFrame& camera = frame.cameras[viewIndex];
        for (int slot = 0; slot < 2; ++slot)
        {
            drawLamp(&painter,
                     camera.carLamps[slot],
                     QColor(255, 205, 64),
                     QStringLiteral("CAR"));
        }
        for (int beacon = 0; beacon < 2; ++beacon)
        {
            drawPoint(&painter,
                      camera.beacons[beacon],
                      QColor(64, 220, 150),
                      QStringLiteral("B%1").arg(beacon));
        }
        return image;
    }

    const JustFloatControlGeometry& control = frame.control;
    const JustFloatShadowCarCenter& shadow = frame.shadow;
    const QColor shadowColor(69, 200, 255);
    if (shadow.lamps[0].valid && shadow.lamps[1].valid)
    {
        QPen pairPen(shadowColor);
        pairPen.setWidth(1);
        pairPen.setStyle(Qt::DashLine);
        painter.setPen(pairPen);
        painter.drawLine(imagePoint(shadow.lamps[0].x, shadow.lamps[0].y),
                         imagePoint(shadow.lamps[1].x, shadow.lamps[1].y));
    }
    drawPoint(&painter, shadow.lamps[0], shadowColor, QStringLiteral("CAR"));
    drawPoint(&painter, shadow.lamps[1], shadowColor, QStringLiteral("CAR"));
    if (shadow.valid)
    {
        JustFloatMappedPoint center;
        center.x = shadow.cx;
        center.y = shadow.cy;
        center.valid = true;
        drawPoint(&painter, center, shadowColor, QStringLiteral("融合中心"));
        drawAxis(&painter,
                 shadow.cx,
                 shadow.cy,
                 shadow.axisAngle,
                 shadowColor,
                 Qt::DashLine);
    }

    if (control.car.valid && control.beacon.valid)
    {
        QPen vectorPen(QColor(255, 128, 210));
        vectorPen.setWidth(2);
        painter.setPen(vectorPen);
        painter.drawLine(imagePoint(control.car.cx, control.car.cy),
                         imagePoint(control.beacon.x, control.beacon.y));
    }
    JustFloatMappedCarLamp controlCar = control.car;
    controlCar.measured = true;
    drawLamp(&painter, controlCar, QColor(255, 205, 64), QStringLiteral("控制车辆"));
    drawPoint(&painter, control.beacon, QColor(255, 92, 92), QStringLiteral("控制信标"));
    return image;
}
