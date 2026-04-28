#include "VideoWidget.h"

#include <QImage>
#include <QEvent>
#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPolygonF>
#include <QRectF>
#include <QSizePolicy>

VideoWidget::VideoWidget(QWidget* parent)
    : QLabel(parent)
{
    setAlignment(Qt::AlignCenter);
    setMinimumSize(188, 120);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setStyleSheet(QStringLiteral("background: #090a0b; color: #aeb6be; border: 1px solid #2a3035; border-radius: 8px;"));
    setText(QStringLiteral("打开 AVI 后显示画面"));
    setMouseTracking(true);
}

void VideoWidget::setImage(const QImage& image)
{
    m_image = image;
    clear();
    update();
}

void VideoWidget::setPixelSourceImage(const QImage& image)
{
    m_pixelSourceImage = image.isNull()
        ? QImage()
        : image.convertToFormat(QImage::Format_Grayscale8);
    if (m_hasHoverPoint)
    {
        updateHoverPixel(m_lastHoverWidgetPoint);
    }
    else
    {
        emit hoverPixelChanged(0, 0, 0, false);
    }
}

void VideoWidget::setFrameGeometry(const QSize& originalSize, int displayScale)
{
    m_originalSize = originalSize;
    m_displayScale = qMax(1, displayScale);
}

void VideoWidget::setCorrectionTool(const QString& tool)
{
    m_correctionTool = tool;
    m_drawing = false;
    m_previewPoints.clear();
    setCursor(tool == QStringLiteral("select") ? Qt::ArrowCursor : Qt::CrossCursor);
    update();
}

void VideoWidget::setCorrectionStyle(const QColor& color, int lineWidth)
{
    m_correctionColor = color.isValid() ? color : QColor(255, 80, 80);
    m_correctionLineWidth = qBound(1, lineWidth, 15);
    update();
}

void VideoWidget::setSelected(bool selected)
{
    if (m_selected == selected)
    {
        return;
    }
    m_selected = selected;
    update();
}

QSize VideoWidget::sizeHint() const
{
    if (!m_image.isNull())
    {
        return QSize(752, 480);
    }
    return QSize(752, 480);
}

void VideoWidget::mousePressEvent(QMouseEvent* event)
{
    updateHoverPixel(event->pos());

    if (event->button() == Qt::LeftButton)
    {
        emit activated();
    }
    if (event->button() == Qt::MiddleButton)
    {
        m_middleDragging = true;
        emit middleDragStarted();
        event->accept();
        return;
    }
    if (event->button() == Qt::RightButton)
    {
        QPointF imagePoint;
        if (widgetToImagePoint(event->pos(), &imagePoint))
        {
            emit contextCorrectionRequested(imagePoint, event->globalPosition().toPoint());
        }
        event->accept();
        return;
    }

    if (event->button() != Qt::LeftButton || m_correctionTool == QStringLiteral("select"))
    {
        QLabel::mousePressEvent(event);
        return;
    }

    QPointF imagePoint;
    if (!widgetToImagePoint(event->pos(), &imagePoint))
    {
        return;
    }

    m_previewPoints.clear();
    m_previewPoints.push_back(imagePoint);

    if (m_correctionTool == QStringLiteral("point"))
    {
        emit correctionShapeFinished(m_correctionTool, m_previewPoints);
        m_previewPoints.clear();
        update();
        return;
    }

    m_drawing = true;
    if (m_correctionTool == QStringLiteral("circle") || m_correctionTool == QStringLiteral("rect"))
    {
        m_previewPoints.push_back(imagePoint);
    }
    update();
}

void VideoWidget::mouseMoveEvent(QMouseEvent* event)
{
    updateHoverPixel(event->pos());

    if (!m_drawing)
    {
        QLabel::mouseMoveEvent(event);
        return;
    }

    QPointF imagePoint;
    if (!widgetToImagePoint(event->pos(), &imagePoint))
    {
        return;
    }

    if (m_correctionTool == QStringLiteral("polygon"))
    {
        if (m_previewPoints.isEmpty() || QLineF(m_previewPoints.last(), imagePoint).length() >= 0.5)
        {
            m_previewPoints.push_back(imagePoint);
        }
    }
    else if (m_previewPoints.size() >= 2)
    {
        m_previewPoints[1] = imagePoint;
    }

    update();
}

void VideoWidget::mouseReleaseEvent(QMouseEvent* event)
{
    updateHoverPixel(event->pos());

    if (event->button() == Qt::MiddleButton && m_middleDragging)
    {
        m_middleDragging = false;
        emit middleDragReleased(event->globalPosition().toPoint());
        event->accept();
        return;
    }

    if (event->button() != Qt::LeftButton || !m_drawing)
    {
        QLabel::mouseReleaseEvent(event);
        return;
    }

    QPointF imagePoint;
    if (widgetToImagePoint(event->pos(), &imagePoint))
    {
        if (m_correctionTool == QStringLiteral("polygon"))
        {
            m_previewPoints.push_back(imagePoint);
        }
        else if (m_previewPoints.size() >= 2)
        {
            m_previewPoints[1] = imagePoint;
        }
    }

    const int minimumPointCount = m_correctionTool == QStringLiteral("polygon") ? 3 : 2;
    if (m_previewPoints.size() >= minimumPointCount)
    {
        emit correctionShapeFinished(m_correctionTool, m_previewPoints);
    }

    m_drawing = false;
    m_previewPoints.clear();
    update();
}

void VideoWidget::leaveEvent(QEvent* event)
{
    m_hasHoverPoint = false;
    emit hoverPixelChanged(0, 0, 0, false);
    QLabel::leaveEvent(event);
}

void VideoWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), QColor(9, 10, 11));

    QPen gridPen(QColor(255, 255, 255, 24));
    gridPen.setWidth(1);
    painter.setPen(gridPen);
    const int columns = 8;
    const int rows = 6;
    for (int i = 1; i < columns; ++i)
    {
        const int x = width() * i / columns;
        painter.drawLine(x, 0, x, height());
    }
    for (int i = 1; i < rows; ++i)
    {
        const int y = height() * i / rows;
        painter.drawLine(0, y, width(), y);
    }

    if (m_image.isNull())
    {
        painter.setPen(QColor(174, 182, 190));
        painter.drawText(rect(), Qt::AlignCenter, text());
        if (m_selected)
        {
            QPen selectedPen(QColor(246, 212, 74));
            selectedPen.setWidth(4);
            painter.setPen(selectedPen);
            painter.drawRect(rect().adjusted(2, 2, -2, -2));
        }
        return;
    }

    const QRectF displayRect = imageDisplayRect();
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter.drawImage(displayRect, m_image);
    painter.setPen(QPen(QColor(246, 212, 74, 190), 1));
    painter.drawRect(displayRect.adjusted(0.5, 0.5, -0.5, -0.5));

    if (m_previewPoints.isEmpty())
    {
        if (m_selected)
        {
            QPen selectedPen(QColor(246, 212, 74));
            selectedPen.setWidth(4);
            painter.setPen(selectedPen);
            painter.drawRect(rect().adjusted(2, 2, -2, -2));
        }
        return;
    }

    QPen pen(m_correctionColor);
    pen.setWidth(m_correctionLineWidth);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);

    const QVector<QPointF> points = previewDisplayPoints();
    if (m_correctionTool == QStringLiteral("circle") && points.size() >= 2)
    {
        const qreal radius = QLineF(points[0], points[1]).length();
        painter.drawEllipse(points[0], radius, radius);
    }
    else if (m_correctionTool == QStringLiteral("rect") && points.size() >= 2)
    {
        painter.drawRect(QRectF(points[0], points[1]).normalized());
    }
    else if (m_correctionTool == QStringLiteral("polygon") && points.size() >= 2)
    {
        painter.drawPolyline(QPolygonF(points));
    }

    if (m_selected)
    {
        QPen selectedPen(QColor(246, 212, 74));
        selectedPen.setWidth(4);
        painter.setPen(selectedPen);
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(rect().adjusted(2, 2, -2, -2));
    }
}

bool VideoWidget::widgetToImagePoint(const QPoint& widgetPoint, QPointF* imagePoint) const
{
    if (imagePoint == nullptr || m_image.isNull() || m_originalSize.isEmpty())
    {
        return false;
    }

    const QRectF displayRect = imageDisplayRect();
    if (!displayRect.contains(widgetPoint))
    {
        return false;
    }

    const QPointF local = QPointF(widgetPoint) - displayRect.topLeft();
    const double x = qBound(0.0,
                            local.x() * (double)m_originalSize.width() / displayRect.width(),
                            (double)m_originalSize.width() - 1.0);
    const double y = qBound(0.0,
                            local.y() * (double)m_originalSize.height() / displayRect.height(),
                            (double)m_originalSize.height() - 1.0);
    *imagePoint = QPointF(x, y);
    return true;
}

bool VideoWidget::widgetToImagePixel(const QPoint& widgetPoint, QPoint* imagePixel) const
{
    QPointF imagePoint;
    if (imagePixel == nullptr || !widgetToImagePoint(widgetPoint, &imagePoint))
    {
        return false;
    }

    const int x = qBound(0, (int)imagePoint.x(), m_originalSize.width() - 1);
    const int y = qBound(0, (int)imagePoint.y(), m_originalSize.height() - 1);
    *imagePixel = QPoint(x, y);
    return true;
}

void VideoWidget::updateHoverPixel(const QPoint& widgetPoint)
{
    if (m_pixelSourceImage.isNull())
    {
        m_hasHoverPoint = false;
        emit hoverPixelChanged(0, 0, 0, false);
        return;
    }

    QPoint imagePixel;
    if (!widgetToImagePixel(widgetPoint, &imagePixel) ||
        imagePixel.x() >= m_pixelSourceImage.width() ||
        imagePixel.y() >= m_pixelSourceImage.height())
    {
        m_hasHoverPoint = false;
        emit hoverPixelChanged(0, 0, 0, false);
        return;
    }

    m_hasHoverPoint = true;
    m_lastHoverWidgetPoint = widgetPoint;
    const uchar* row = m_pixelSourceImage.constScanLine(imagePixel.y());
    const int gray = row[imagePixel.x()];
    emit hoverPixelChanged(imagePixel.x(), imagePixel.y(), gray, true);
}

QVector<QPointF> VideoWidget::previewDisplayPoints() const
{
    QVector<QPointF> points;
    const QRectF displayRect = imageDisplayRect();
    for (const QPointF& point : m_previewPoints)
    {
        points.push_back(displayRect.topLeft() +
                         QPointF(point.x() * displayRect.width() / (double)m_originalSize.width(),
                                 point.y() * displayRect.height() / (double)m_originalSize.height()));
    }
    return points;
}

QRectF VideoWidget::imageDisplayRect() const
{
    if (m_image.isNull())
    {
        return QRectF();
    }

    const QSize scaled = m_image.size().scaled(size(), Qt::KeepAspectRatio);
    return QRectF((width() - scaled.width()) / 2.0,
                  (height() - scaled.height()) / 2.0,
                  scaled.width(),
                  scaled.height());
}
