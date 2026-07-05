#ifndef LOG_REPLAY_WINDOW_H
#define LOG_REPLAY_WINDOW_H

#include "JustFloatLog.h"
#include "beacon_image.h"

#include <QImage>
#include <QWidget>

#include <array>

class QComboBox;
class QGroupBox;
class QHBoxLayout;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QTextEdit;
class QTimer;
class VideoWidget;

class LogReplayWindow : public QWidget
{
    Q_OBJECT

public:
    explicit LogReplayWindow(QWidget* parent = nullptr);

private:
    void importCsv();
    bool loadCsv(const QString& path);
    void setCurrentRow(int row);
    void renderCurrentRow();
    void renderCamera(int cameraIndex);
    void togglePlayback();
    void advancePlayback();
    void scheduleNextFrame();
    int playbackIntervalMs() const;
    void setFocusCamera(int cameraIndex);
    void showAllCameras();
    void updateCameraVisibility();
    void updateInfoText();

    beacon_result_t resultForCamera(int cameraIndex) const;
    QImage syntheticImageForCamera(int cameraIndex) const;

    JustFloatLog m_log;
    int m_currentRow = -1;
    int m_focusCamera = -1;
    bool m_playing = false;

    QTimer* m_timer = nullptr;
    QLabel* m_statusLabel = nullptr;
    QTextEdit* m_infoText = nullptr;
    QPushButton* m_playButton = nullptr;
    QPushButton* m_returnGridButton = nullptr;
    QSlider* m_slider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QComboBox* m_speedCombo = nullptr;
    QHBoxLayout* m_cameraLayout = nullptr;
    std::array<QGroupBox*, 3> m_cameraGroups = {};
    std::array<VideoWidget*, 3> m_videoWidgets = {};
    std::array<QLabel*, 3> m_cameraInfoLabels = {};
};

#endif
