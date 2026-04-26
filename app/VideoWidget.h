#ifndef VIDEO_WIDGET_H
#define VIDEO_WIDGET_H

#include <QLabel>
#include <QPointF>
#include <QVector>

class VideoWidget : public QLabel
{
    Q_OBJECT

public:
    explicit VideoWidget(QWidget* parent = nullptr);
    void setImage(const QImage& image);
    void setFrameGeometry(const QSize& originalSize, int displayScale);
    void setCorrectionTool(const QString& tool);
    QSize sizeHint() const override;

signals:
    void correctionShapeFinished(const QString& shapeType, const QVector<QPointF>& points);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private:
    bool widgetToImagePoint(const QPoint& widgetPoint, QPointF* imagePoint) const;
    QVector<QPointF> previewDisplayPoints() const;
    QRectF imageDisplayRect() const;

    QImage m_image;
    QSize m_originalSize;
    int m_displayScale = 1;
    QString m_correctionTool = QStringLiteral("select");
    bool m_drawing = false;
    QVector<QPointF> m_previewPoints;
};

#endif
