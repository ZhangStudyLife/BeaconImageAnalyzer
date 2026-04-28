#include "FrameRenderer.h"

#include <QFont>
#include <QLineF>
#include <QPainter>
#include <QPolygonF>
#include <QRectF>
#include <QtMath>

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

QVector<QPointF> displayPointsForShape(const CorrectionShape& shape, int safeScale)
{
    QVector<QPointF> points;
    for (const QPointF& point : shape.points)
    {
        points.push_back(QPointF(point.x() * safeScale, point.y() * safeScale));
    }
    return points;
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
                             bool showOverlay)
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

    QPen circlePen(QColor(0, 255, 80));
    circlePen.setWidth(qMax(1, safeScale / 2));
    painter.setPen(circlePen);
    painter.setBrush(Qt::NoBrush);

    QFont font = painter.font();
    font.setPixelSize(safeScale == 1 ? 8 : 12);
    font.setBold(true);
    painter.setFont(font);

    for (int i = 0; i < result.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        const beacon_circle_t& circle = result.circles[i];
        if (circle.valid == 0)
        {
            continue;
        }

        const QPointF imagePoint = algorithmToImagePoint(circle.x, circle.y);
        const QPointF displayPoint(imagePoint.x() * safeScale, imagePoint.y() * safeScale);
        const qreal displayRadius = qMax(1.0, (double)circle.radius * safeScale);

        painter.setPen(circlePen);
        painter.drawEllipse(displayPoint, displayRadius, displayRadius);
        painter.drawLine(QPointF(displayPoint.x() - safeScale, displayPoint.y()),
                         QPointF(displayPoint.x() + safeScale, displayPoint.y()));
        painter.drawLine(QPointF(displayPoint.x(), displayPoint.y() - safeScale),
                         QPointF(displayPoint.x(), displayPoint.y() + safeScale));

        painter.setPen(QColor(255, 235, 80));
        painter.drawText(QPointF(displayPoint.x() + 2 * safeScale,
                                 displayPoint.y() - 2 * safeScale),
                         QStringLiteral("#%1").arg(i));
    }

    QPen correctionPen;
    correctionPen.setWidth(1);
    painter.setBrush(Qt::NoBrush);

    for (const CorrectionShape& shape : corrections)
    {
        const QString type = correctionType(shape);
        const QString label = annotationTypeDisplayName(type);
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
    }

    return base;
}
