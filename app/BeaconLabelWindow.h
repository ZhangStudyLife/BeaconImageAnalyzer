#ifndef BEACON_LABEL_WINDOW_H
#define BEACON_LABEL_WINDOW_H

#include "BeaconLabelSession.h"
#include "VideoReader.h"

#include <QImage>
#include <QWidget>

class QCloseEvent;
class QLabel;
class QPushButton;
class QScrollArea;
class QSlider;
class QSpinBox;
class VideoWidget;

class BeaconLabelWindow : public QWidget
{
    Q_OBJECT

public:
    explicit BeaconLabelWindow(QWidget* parent = nullptr);
    bool openVideo(const QString& path);
    bool openSession(const QString& path);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void chooseVideo();
    void chooseSession();
    void saveAs();
    void showFrame(int frameIndex);
    void renderCurrentFrame();
    void addBeaconPoint(const QString& shapeType, const QVector<QPointF>& points);
    void removeNearestPoint(const QPointF& imagePoint);
    void setCurrentState(BeaconLabelFrameState state);
    void clearCurrentFrame();
    void moveFrame(int offset);
    void moveSample(int direction);
    void moveToPending(int direction);
    void updateSummary();
    void updateZoom();
    bool saveCurrent(bool showError);

    BeaconLabelSession m_session;
    VideoReader m_reader;
    QImage m_currentImage;
    int m_currentFrame = -1;
    bool m_dirty = false;

    VideoWidget* m_videoWidget = nullptr;
    QScrollArea* m_scrollArea = nullptr;
    QLabel* m_sessionLabel = nullptr;
    QLabel* m_frameLabel = nullptr;
    QLabel* m_summaryLabel = nullptr;
    QLabel* m_pixelLabel = nullptr;
    QLabel* m_saveLabel = nullptr;
    QSlider* m_frameSlider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QSpinBox* m_strideSpin = nullptr;
    QSpinBox* m_zoomSpin = nullptr;
};

#endif
