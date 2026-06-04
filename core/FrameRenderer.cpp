#include "FrameRenderer.h"

#include "CameraBoundaryMapping.h"

#include <QFont>
#include <QLineF>
#include <QPainter>
#include <QPolygonF>
#include <QRectF>
#include <QtMath>

namespace
{
constexpr int MarkerSize = 3;
constexpr int CarLampLineWidth = 3;
constexpr int DisplayBeaconLimit = 3;
constexpr float Pi = 3.1415926f;
constexpr int CenterCameraIndex = 1;

int roundFloatToInt(float value)
{
    if (value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }
    return (int)(value - 0.5f);
}

bool targetToImagePoint(float x, float y, int* imageX, int* imageY)
{
    if (imageX == nullptr || imageY == nullptr)
    {
        return false;
    }

    const int pixelX = roundFloatToInt(BEACON_IMAGE_TARGET_PIXEL_X - x);
    const int pixelY = roundFloatToInt(y + BEACON_IMAGE_TARGET_PIXEL_Y);
    if (pixelX < 0 || pixelX >= BEACON_IMAGE_W || pixelY < 0 || pixelY >= BEACON_IMAGE_H)
    {
        return false;
    }

    *imageX = pixelX;
    *imageY = pixelY;
    return true;
}

QRect scaledPixelRect(int imageX, int imageY, int safeScale)
{
    return QRect(imageX * safeScale, imageY * safeScale, safeScale, safeScale);
}

void fillScaledPixel(QPainter& painter, int imageX, int imageY, int safeScale, const QColor& color)
{
    if (imageX < 0 || imageX >= BEACON_IMAGE_W || imageY < 0 || imageY >= BEACON_IMAGE_H)
    {
        return;
    }

    painter.fillRect(scaledPixelRect(imageX, imageY, safeScale), color);
}

void drawBrush(QPainter& painter, int centerX, int centerY, int size, int safeScale, const QColor& color)
{
    for (int dy = -(size / 2); dy <= (size / 2); ++dy)
    {
        for (int dx = -(size / 2); dx <= (size / 2); ++dx)
        {
            fillScaledPixel(painter, centerX + dx, centerY + dy, safeScale, color);
        }
    }
}

void drawBeaconMarkers(QPainter& painter, const beacon_result_t& result, int safeScale)
{
    int drawnCount = 0;
    const int beaconCount = qMin((int)result.beacon_count, BEACON_MAX_CIRCLE_COUNT);
    for (int i = 0; i < beaconCount && drawnCount < DisplayBeaconLimit; ++i)
    {
        const beacon_circle_t& beacon = result.beacons[i];
        int imageX = 0;
        int imageY = 0;
        if (beacon.valid == 0 || !targetToImagePoint(beacon.x, beacon.y, &imageX, &imageY))
        {
            continue;
        }

        drawBrush(painter, imageX, imageY, MarkerSize, safeScale, QColor(255, 0, 255));
        ++drawnCount;
    }
}

void drawCarLampLine(QPainter& painter, const beacon_rect_t& lamp, int safeScale)
{
    int centerX = 0;
    int centerY = 0;
    if (lamp.valid == 0 || !targetToImagePoint(lamp.cx, lamp.cy, &centerX, &centerY))
    {
        return;
    }

    float halfLength = lamp.length * 0.5f;
    if (halfLength < 1.0f)
    {
        halfLength = 1.0f;
    }

    const float angleRad = lamp.angle * Pi / 180.0f;
    const float dirX = qCos(angleRad);
    const float dirY = qSin(angleRad);
    const int steps = roundFloatToInt(halfLength);
    for (int step = -steps; step <= steps; ++step)
    {
        const int x = roundFloatToInt((float)centerX + (dirX * (float)step));
        const int y = roundFloatToInt((float)centerY + (dirY * (float)step));
        drawBrush(painter, x, y, CarLampLineWidth, safeScale, QColor(255, 0, 0));
    }
}

void drawCarLampMarkers(QPainter& painter, const beacon_result_t& result, int safeScale)
{
    (void)result.car_lamp_count;
    for (int i = 0; i < BEACON_MAX_CAR_LAMP_COUNT; ++i)
    {
        drawCarLampLine(painter, result.car_lamps[i], safeScale);
    }
}

void drawTrackedPoint(QPainter& painter, const TrackedBeaconPoint* trackedPoint, int cameraIndex, int safeScale)
{
    if (trackedPoint == nullptr || !trackedPoint->valid || trackedPoint->cameraIndex != cameraIndex)
    {
        return;
    }

    const int x = roundFloatToInt((float)trackedPoint->imagePoint.x());
    const int y = roundFloatToInt((float)trackedPoint->imagePoint.y());
    const QColor color(0, 255, 120);
    drawBrush(painter, x, y, 5, safeScale, color);
    for (int d = -6; d <= 6; ++d)
    {
        fillScaledPixel(painter, x + d, y, safeScale, color);
        fillScaledPixel(painter, x, y + d, safeScale, color);
    }
}

void drawDetectionBoundary(QPainter& painter, const DetectionBoundary* boundary, int safeScale)
{
    if (boundary == nullptr)
    {
        return;
    }

    const QColor color = DetectionBoundaryRules::colorForType(boundary->type);
    if (!color.isValid())
    {
        return;
    }

    for (int pixelX = 0; pixelX < BEACON_IMAGE_W; ++pixelX)
    {
        bool valid = false;
        const QPointF point = DetectionBoundaryRules::imagePointForX(*boundary, pixelX, &valid);
        if (!valid)
        {
            continue;
        }

        drawBrush(painter, pixelX, (int)point.y(), 1, safeScale, color);
    }
}

void drawDashedPoint(QPainter& painter, int imageX, int imageY, int sourceX, int safeScale, const QColor& color)
{
    if ((sourceX / 4) % 2 != 0)
    {
        return;
    }
    drawBrush(painter, imageX, imageY, 1, safeScale, color);
}

void drawMappedBoundaryCurves(QPainter& painter, int cameraIndex, int safeScale)
{
    if (cameraIndex != CenterCameraIndex)
    {
        return;
    }

    for (int x = 0; x < BEACON_IMAGE_W; ++x)
    {
        const QPointF frontPoint = CameraBoundaryMapping::frontBoundaryToCenter((float)x);
        drawDashedPoint(painter,
                        roundFloatToInt((float)frontPoint.x()),
                        roundFloatToInt((float)frontPoint.y()),
                        x,
                        safeScale,
                        QColor(255, 255, 0));

        const QPointF rearPoint = CameraBoundaryMapping::rearBoundaryToCenter((float)x);
        drawDashedPoint(painter,
                        roundFloatToInt((float)rearPoint.x()),
                        roundFloatToInt((float)rearPoint.y()),
                        x,
                        safeScale,
                        QColor(0, 255, 255));
    }
}

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
}

QPointF FrameRenderer::algorithmToImagePoint(float x, float y)
{
    const float centerX = BEACON_IMAGE_TARGET_PIXEL_X - x;
    const float centerY = BEACON_IMAGE_TARGET_PIXEL_Y + y;
    return QPointF(centerX, centerY);
}

QImage FrameRenderer::render(const QImage& grayImage,
                             const beacon_result_t& result,
                             const QVector<CorrectionShape>& corrections,
                             int scale,
                             bool showOverlay,
                             const DetectionBoundary* boundary,
                             int cameraIndex,
                             const TrackedBeaconPoint* trackedPoint)
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

    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::NoBrush);
    drawDetectionBoundary(painter, boundary, safeScale);
    drawMappedBoundaryCurves(painter, cameraIndex, safeScale);
    drawBeaconMarkers(painter, result, safeScale);
    drawCarLampMarkers(painter, result, safeScale);
    drawTrackedPoint(painter, trackedPoint, cameraIndex, safeScale);

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
