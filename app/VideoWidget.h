#ifndef VIDEO_WIDGET_H
#define VIDEO_WIDGET_H

#include <QImage>
#include <QLabel>
#include <QPoint>
#include <QPointF>
#include <QVector>

class QEvent;

class VideoWidget : public QLabel
{
    Q_OBJECT

public:
    explicit VideoWidget(QWidget* parent = nullptr);
    void setImage(const QImage& image);
    void setPixelSourceImage(const QImage& image);
    void setFrameGeometry(const QSize& originalSize, int displayScale);
    void setCorrectionTool(const QString& tool);
    QSize sizeHint() const override;

signals:
    void correctionShapeFinished(const QString& shapeType, const QVector<QPointF>& points);
    void hoverPixelChanged(int x, int y, int gray, bool valid);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private:
    bool widgetToImagePoint(const QPoint& widgetPoint, QPointF* imagePoint) const;
    bool widgetToImagePixel(const QPoint& widgetPoint, QPoint* imagePixel) const;
    void updateHoverPixel(const QPoint& widgetPoint);
    QVector<QPointF> previewDisplayPoints() const;
    QRectF imageDisplayRect() const;

    QImage m_image;
    QImage m_pixelSourceImage;
    QSize m_originalSize;
    QPoint m_lastHoverWidgetPoint;
    int m_displayScale = 1;
    QString m_correctionTool = QStringLiteral("select");
    bool m_hasHoverPoint = false;
    bool m_drawing = false;
    QVector<QPointF> m_previewPoints;
};

#endif
