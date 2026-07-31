#include "FrameRenderer.h"

#include "AlgorithmRunner.h"
#include "BeaconResultUtils.h"

#include <QFont>
#include <QLineF>
#include <QPainter>
#include <QPolygonF>
#include <QRectF>
#include <QtMath>

#include <cmath>

namespace
{
QString correctionType(const CorrectionShape& shape)
{
    for (const QString& type : shape.errorTypes)
    {
        if (!type.trimmed().isEmpty())
        {
            return type.trimmed();
        }
    }
    return shape.errorType.trimmed();
}

QString correctionLabel(const CorrectionShape& shape)
{
    QStringList types;
    for (const QString& type : shape.errorTypes)
    {
        if (!type.trimmed().isEmpty())
        {
            types.push_back(type.trimmed());
        }
    }
    if (types.isEmpty() && !shape.errorType.trimmed().isEmpty())
    {
        types.push_back(shape.errorType.trimmed());
    }
    return annotationTypesDisplayName(types);
}

QString correctionSummary(const CorrectionShape& shape)
{
    QString text = correctionLabel(shape);
    if (!shape.description.trimmed().isEmpty())
    {
        if (!text.trimmed().isEmpty())
        {
            text += QStringLiteral(": ");
        }
        text += shape.description.trimmed();
    }
    return text;
}

QColor correctionColor(const QString& type)
{
    if (type == QStringLiteral("false_positive"))
    {
        return QColor(255, 92, 92);
    }
    if (type == QStringLiteral("missed_detection"))
    {
        return QColor(60, 220, 255);
    }
    if (type == QStringLiteral("wrong_order"))
    {
        return QColor(255, 205, 64);
    }
    if (type == QStringLiteral("target_jump"))
    {
        return QColor(190, 120, 255);
    }
    if (type.startsWith(QStringLiteral("custom:")))
    {
        const uint hue = qHash(type) % 360;
        return QColor::fromHsv((int)hue, 210, 255);
    }
    return QColor(255, 150, 60);
}

void drawCorrectionLabel(QPainter& painter,
                         const QPointF& anchor,
                         const QString& text,
                         const QColor& color,
                         int safeScale)
{
    if (text.trimmed().isEmpty())
    {
        return;
    }

    QFont font = painter.font();
    font.setPixelSize(safeScale == 1 ? 7 : 10);
    font.setBold(false);
    painter.setFont(font);
    painter.setPen(color);
    painter.drawText(anchor + QPointF(2 * safeScale, -2 * safeScale), text);
}

void drawCorrectionSummary(QPainter& painter,
                           int row,
                           const QString& text,
                           const QColor& color,
                           int safeScale)
{
    if (text.trimmed().isEmpty())
    {
        return;
    }

    QFont font = painter.font();
    font.setPixelSize(safeScale == 1 ? 7 : 10);
    font.setBold(false);
    painter.setFont(font);
    painter.setPen(color);
    const int lineHeight = font.pixelSize() + 3 * safeScale;
    painter.drawText(QPointF(4 * safeScale, (row + 1) * lineHeight), text);
}

QVector<QPointF> displayPointsForShape(const CorrectionShape& shape, int safeScale)
{
    QVector<QPointF> points;
    for (const QPointF& point : shape.points)
    {
        points.push_back(QPointF(point.x() * safeScale, point.y() * safeScale));
    }
    return points;
}

void drawTargetCircle(QPainter& painter,
                      const beacon_circle_t& circle,
                      const QColor& circleColor,
                      const QString& label,
                      int safeScale)
{
    const QPointF imagePoint = FrameRenderer::algorithmToImagePoint(circle.x, circle.y);
    const QPointF displayPoint(imagePoint.x() * safeScale, imagePoint.y() * safeScale);
    const qreal displayRadius = qMax(1.0, (double)circle.radius * safeScale);

    QPen circlePen(circleColor);
    circlePen.setWidth(qMax(1, safeScale / 2));
    painter.setPen(circlePen);
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(displayPoint, displayRadius, displayRadius);
    painter.drawLine(QPointF(displayPoint.x() - safeScale, displayPoint.y()),
                     QPointF(displayPoint.x() + safeScale, displayPoint.y()));
    painter.drawLine(QPointF(displayPoint.x(), displayPoint.y() - safeScale),
                     QPointF(displayPoint.x(), displayPoint.y() + safeScale));

    painter.setPen(QColor(255, 235, 80));
    painter.drawText(QPointF(displayPoint.x() + 2 * safeScale,
                             displayPoint.y() - 2 * safeScale),
                     label);
}

QPointF displayPointForCircle(const beacon_circle_t& circle, int safeScale)
{
    const QPointF imagePoint = FrameRenderer::algorithmToImagePoint(circle.x, circle.y);
    return QPointF(imagePoint.x() * safeScale, imagePoint.y() * safeScale);
}

QPointF displayPointForRectCenter(const beacon_rect_t& rect, int safeScale)
{
    const QPointF imagePoint = FrameRenderer::algorithmToImagePoint(rect.cx, rect.cy);
    return QPointF(imagePoint.x() * safeScale, imagePoint.y() * safeScale);
}

QPolygonF displayPolygonForRect(const beacon_rect_t& rect, int safeScale)
{
    const QPointF center = displayPointForRectCenter(rect, safeScale);
    const qreal halfLength = qMax(1.0, (double)rect.length * safeScale * 0.5);
    const qreal halfWidth = qMax(1.0, (double)rect.width * safeScale * 0.5);
    const qreal radians = qDegreesToRadians((double)rect.angle);
    const QPointF major(qCos(radians) * halfLength, qSin(radians) * halfLength);
    const QPointF minor(-qSin(radians) * halfWidth, qCos(radians) * halfWidth);

    QPolygonF polygon;
    polygon << center - major - minor
            << center + major - minor
            << center + major + minor
            << center - major + minor;
    return polygon;
}

void drawTargetRect(QPainter& painter,
                    const beacon_rect_t& rect,
                    const QColor& rectColor,
                    const QString& label,
                    int safeScale)
{
    const QPointF center = displayPointForRectCenter(rect, safeScale);
    const QPolygonF polygon = displayPolygonForRect(rect, safeScale);

    QPen rectPen(rectColor);
    rectPen.setWidth(qMax(1, safeScale / 2));
    painter.setPen(rectPen);
    painter.setBrush(Qt::NoBrush);
    painter.drawPolygon(polygon);
    painter.drawLine(QPointF(center.x() - safeScale, center.y()),
                     QPointF(center.x() + safeScale, center.y()));
    painter.drawLine(QPointF(center.x(), center.y() - safeScale),
                     QPointF(center.x(), center.y() + safeScale));

    painter.setPen(QColor(255, 235, 80));
    painter.drawText(QPointF(center.x() + 2 * safeScale,
                             center.y() - 2 * safeScale),
                     label);
}

void drawHorizon(QPainter& painter,
                 const AlgorithmHorizonCurve& horizon,
                 int safeScale)
{
    if (!horizon.valid)
    {
        return;
    }

    QPen pen(QColor(40, 255, 80));
    pen.setWidth(qMax(1, safeScale / 2));
    painter.setPen(pen);
    QPointF previous;
    bool hasPrevious = false;
    for (int x = 0; x < BEACON_IMAGE_W; ++x)
    {
        const float y = horizon.y[static_cast<std::size_t>(x)];
        if (horizon.columnValid[static_cast<std::size_t>(x)] == 0U
            || !std::isfinite(y))
        {
            hasPrevious = false;
            continue;
        }
        const QPointF current(x * safeScale, y * safeScale);
        if (hasPrevious)
        {
            painter.drawLine(previous, current);
        }
        previous = current;
        hasPrevious = true;
    }
}
}

QPointF FrameRenderer::algorithmToImagePoint(float x, float y)
{
    const float centerX = (float)BEACON_IMAGE_W * 0.5f - x;
    const float centerY = (float)BEACON_IMAGE_H * 0.5f + y;
    return QPointF(centerX, centerY);
}

QImage FrameRenderer::render(const QImage& grayImage,
                             const beacon_result_t& result,
                             const QVector<CorrectionShape>& corrections,
                             int scale,
                             bool showOverlay,
                             const AlgorithmHorizonCurve* horizon)
{
    const int safeScale = qMax(1, scale);
    QImage base = grayImage.convertToFormat(QImage::Format_RGB32);
    if (safeScale != 1)
    {
        base = base.scaled(base.width() * safeScale,
                           base.height() * safeScale,
                           Qt::IgnoreAspectRatio,
                           Qt::FastTransformation);
    }

    if (!showOverlay)
    {
        return base;
    }

    QPainter painter(&base);
    painter.setRenderHint(QPainter::Antialiasing, false);

    QFont font = painter.font();
    font.setPixelSize(safeScale == 1 ? 8 : 12);
    font.setBold(true);
    painter.setFont(font);
    if (horizon != nullptr)
    {
        drawHorizon(painter, *horizon, safeScale);
    }

    const bool useLegacyBeacons = BeaconResultUtils::usesLegacyBeacons(result);
    const beacon_circle_t* beacons = useLegacyBeacons ? result.circles : result.beacons;
    const int beaconCount = useLegacyBeacons
        ? BeaconResultUtils::boundedCount(result.count, BEACON_MAX_CIRCLE_COUNT)
        : BeaconResultUtils::boundedCount(result.beacon_count, BEACON_MAX_BEACON_COUNT);
    for (int i = 0; i < beaconCount; ++i)
    {
        const beacon_circle_t& circle = beacons[i];
        if (circle.valid == 0)
        {
            continue;
        }

        drawTargetCircle(painter, circle, QColor(0, 255, 80), QStringLiteral("B%1").arg(i), safeScale);
    }

    const int carLampCount = BeaconResultUtils::boundedCount(result.car_lamp_count, BEACON_MAX_CAR_LAMP_COUNT);
    for (int i = 0; i < carLampCount; ++i)
    {
        const beacon_rect_t& lamp = result.car_lamps[i];
        if (lamp.valid == 0)
        {
            continue;
        }

        drawTargetRect(painter,
                       lamp,
                       QColor(255, 95, 45),
                       QStringLiteral("CAR %1").arg(i),
                       safeScale);
    }

    const int temporalBeaconCount = BeaconResultUtils::boundedCount(result.temporal_beacon_count,
                                                                    BEACON_MAX_BEACON_COUNT);
    for (int i = 0; i < temporalBeaconCount; ++i)
    {
        const beacon_circle_t& circle = result.temporal_beacons[i];
        if (circle.valid == 0)
        {
            continue;
        }

        drawTargetCircle(painter, circle, QColor(255, 220, 40), QStringLiteral("KB%1").arg(i), safeScale);
    }

    const int temporalCarLampCount = BeaconResultUtils::boundedCount(result.temporal_car_lamp_count,
                                                                     BEACON_MAX_CAR_LAMP_COUNT);
    for (int i = 0; i < temporalCarLampCount; ++i)
    {
        const beacon_rect_t& lamp = result.temporal_car_lamps[i];
        if (lamp.valid == 0)
        {
            continue;
        }

        drawTargetRect(painter,
                       lamp,
                       QColor(70, 150, 255),
                       QStringLiteral("KCAR%1").arg(i),
                       safeScale);
    }

    QPen correctionPen;
    correctionPen.setWidth(1);
    painter.setBrush(Qt::NoBrush);
    int summaryRow = 0;

    for (const CorrectionShape& shape : corrections)
    {
        const QString type = correctionType(shape);
        const QString label = correctionLabel(shape);
        const QColor shapeColor = correctionColor(type);
        correctionPen.setColor(shapeColor);
        correctionPen.setWidth(1);
        painter.setPen(correctionPen);

        bool drewTarget = false;
        for (const ErrorCircle& errorCircle : shape.errorCircles)
        {
            const int circleIndex = errorCircle.circleIndex;
            if (circleIndex < 0 || circleIndex >= result.count || circleIndex >= BEACON_MAX_CIRCLE_COUNT)
            {
                continue;
            }

            const beacon_circle_t& circle = result.circles[circleIndex];
            if (circle.valid == 0)
            {
                continue;
            }

            const QPointF imagePoint = algorithmToImagePoint(circle.x, circle.y);
            const QPointF displayPoint(imagePoint.x() * safeScale, imagePoint.y() * safeScale);
            const qreal displayRadius = qMax(1.0, (double)circle.radius * safeScale + 1.0);
            painter.drawEllipse(displayPoint, displayRadius, displayRadius);
            drawCorrectionLabel(painter,
                                QPointF(displayPoint.x() + displayRadius + safeScale, displayPoint.y()),
                                label,
                                shapeColor,
                                safeScale);
            painter.setPen(correctionPen);
            drewTarget = true;
        }

        const QVector<QPointF> displayPoints = displayPointsForShape(shape, safeScale);
        if (type == QStringLiteral("missed_detection") &&
            shape.shapeType == QStringLiteral("circle") &&
            displayPoints.size() >= 2)
        {
            const QPointF center = displayPoints[0];
            const qreal radius = QLineF(displayPoints[0], displayPoints[1]).length();
            painter.drawEllipse(center, radius, radius);
            drawCorrectionLabel(painter,
                                QPointF(center.x() + radius + safeScale, center.y()),
                                label,
                                shapeColor,
                                safeScale);
            painter.setPen(correctionPen);
            drewTarget = true;
        }

        if (drewTarget)
        {
            continue;
        }

        if (shape.shapeType == QStringLiteral("circle") && displayPoints.size() >= 2)
        {
            const QPointF center = displayPoints[0];
            const qreal radius = QLineF(displayPoints[0], displayPoints[1]).length();
            painter.drawEllipse(center, radius, radius);
            drawCorrectionLabel(painter,
                                QPointF(center.x() + radius + safeScale, center.y()),
                                label,
                                shapeColor,
                                safeScale);
            painter.setPen(correctionPen);
        }
        else if (shape.shapeType == QStringLiteral("rect") && displayPoints.size() >= 2)
        {
            const QRectF rect = QRectF(displayPoints[0], displayPoints[1]).normalized();
            painter.drawRect(rect);
            drawCorrectionLabel(painter, rect.topRight(), label, shapeColor, safeScale);
            painter.setPen(correctionPen);
        }
        else if (shape.shapeType == QStringLiteral("point") && !displayPoints.isEmpty())
        {
            painter.setBrush(shapeColor);
            painter.drawEllipse(displayPoints[0], qMax(1, safeScale), qMax(1, safeScale));
            painter.setBrush(Qt::NoBrush);
            drawCorrectionLabel(painter, displayPoints[0] + QPointF(safeScale, 0), label, shapeColor, safeScale);
            painter.setPen(correctionPen);
        }
        else if (shape.shapeType == QStringLiteral("polygon") && displayPoints.size() >= 2)
        {
            QPolygonF polygon(displayPoints);
            painter.drawPolyline(polygon);
            if (displayPoints.size() >= 3)
            {
                painter.drawLine(displayPoints.last(), displayPoints.first());
            }
            drawCorrectionLabel(painter, displayPoints.first(), label, shapeColor, safeScale);
            painter.setPen(correctionPen);
        }
        else
        {
            drawCorrectionSummary(painter, summaryRow, correctionSummary(shape), shapeColor, safeScale);
            ++summaryRow;
            painter.setPen(correctionPen);
        }
    }

    return base;
}
