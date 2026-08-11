#ifndef IMAGE_LOG_REVIEW_WINDOW_H
#define IMAGE_LOG_REVIEW_WINDOW_H

#include "ImageFrameSidecar.h"
#include "ImageLogAligner.h"
#include "JustFloatLog.h"
#include "VideoReader.h"

#include <QWidget>

class QCheckBox;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;
class VideoWidget;

// 独立的视频/日志审查窗口；不保存任何校准结果，只在内存中维护手工周期修正。
class ImageLogReviewWindow : public QWidget
{
    Q_OBJECT

public:
    explicit ImageLogReviewWindow(QWidget* parent = nullptr);
    bool openVideo(const QString& aviPath);
    bool openLog(const QString& csvPath);

private:
    void chooseVideo();
    void chooseLog();
    void rebuildAlignment();
    void showFrame(int frameIndex);
    void moveFrame(int delta);
    void togglePlayback();
    void updatePlayback();
    void updateStatus();
    QImage overlayFrame(const QImage& gray, int frameIndex, QString* detail) const;
    bool overlayAllowed() const;

    VideoReader m_reader;
    QVector<ImageFrameSidecarRecord> m_sidecar;
    JustFloatLog m_log;
    ImageLogAlignmentResult m_alignment;
    int m_currentFrame = -1;
    int m_manualCycleShift = 0;
    bool m_playing = false;

    VideoWidget* m_videoWidget = nullptr;
    QLabel* m_sourceLabel = nullptr;
    QLabel* m_alignmentLabel = nullptr;
    QLabel* m_frameLabel = nullptr;
    QCheckBox* m_confirmLowConfidenceCheck = nullptr;
    QPushButton* m_playButton = nullptr;
    QSlider* m_frameSlider = nullptr;
    QTimer* m_playTimer = nullptr;
};

#endif
