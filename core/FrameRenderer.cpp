#include "FrameRenderer.h"

#include <QFont>
#include <QLineF>
#include <QPainter>
#include <QPolygonF>
#include <QRectF>
#include <QtMath>

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

    QPen correctionPen(QColor(255, 80, 80));
    correctionPen.setWidth(qMax(1, safeScale));
    painter.setPen(correctionPen);
    painter.setBrush(Qt::NoBrush);

    for (const CorrectionShape& shape : corrections)
    {
        const QColor shapeColor = shape.lineColor.isValid() ? shape.lineColor : QColor(255, 80, 80);
        correctionPen.setColor(shapeColor);
        correctionPen.setWidth(qMax(1, shape.lineWidth * safeScale));
        painter.setPen(correctionPen);
        painter.setBrush(Qt::NoBrush);

        QVector<QPointF> displayPoints;
        for (const QPointF& point : shape.points)
        {
            displayPoints.push_back(QPointF(point.x() * safeScale, point.y() * safeScale));
        }

        if (shape.shapeType == QStringLiteral("circle") && displayPoints.size() >= 2)
        {
            const QPointF center = displayPoints[0];
            const qreal radius = QLineF(displayPoints[0], displayPoints[1]).length();
            painter.drawEllipse(center, radius, radius);
            painter.drawLine(QPointF(center.x() - 2 * safeScale, center.y()),
                             QPointF(center.x() + 2 * safeScale, center.y()));
            painter.drawLine(QPointF(center.x(), center.y() - 2 * safeScale),
                             QPointF(center.x(), center.y() + 2 * safeScale));
        }
        else if (shape.shapeType == QStringLiteral("rect") && displayPoints.size() >= 2)
        {
            painter.drawRect(QRectF(displayPoints[0], displayPoints[1]).normalized());
        }
        else if (shape.shapeType == QStringLiteral("point") && !displayPoints.isEmpty())
        {
            painter.setBrush(shapeColor);
            painter.drawEllipse(displayPoints[0], 2 * safeScale, 2 * safeScale);
            painter.setBrush(Qt::NoBrush);
        }
        else if (shape.shapeType == QStringLiteral("polygon") && displayPoints.size() >= 2)
        {
            QPolygonF polygon(displayPoints);
            painter.drawPolyline(polygon);
            if (displayPoints.size() >= 3)
            {
                painter.drawLine(displayPoints.last(), displayPoints.first());
            }
        }

        const QString circleText = errorCirclesDisplayName(shape.errorCircles);
        const bool hasCircleText = !shape.errorCircles.isEmpty();
        if (!displayPoints.isEmpty() && (shape.expectedIndex >= 0 || hasCircleText))
        {
            painter.setPen(shapeColor.lighter(150));
            const QString label = hasCircleText
                ? circleText
                : QStringLiteral("GT #%1").arg(shape.expectedIndex);
            painter.drawText(displayPoints.first() + QPointF(2 * safeScale, -2 * safeScale),
                             label);
            painter.setPen(correctionPen);
        }
    }

    return base;
}
