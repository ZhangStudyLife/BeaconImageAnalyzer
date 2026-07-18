#ifndef LOG_REPLAY_WINDOW_H
#define LOG_REPLAY_WINDOW_H

#include "CarPlanRunner.h"
#include "JustFloatCsvRecorder.h"
#include "JustFloatLog.h"
#include "WaveformHistoryStore.h"
#include "beacon_image.h"

#include <QElapsedTimer>
#include <QImage>
#include <QVector>
#include <QWidget>

#include <array>

class QComboBox;
class QCloseEvent;
class QGroupBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTextEdit;
class QTimer;
class QUdpSocket;
class LogWaveformWindow;
class VideoWidget;

class LogReplayWindow : public QWidget
{
    Q_OBJECT

public:
    explicit LogReplayWindow(QWidget* parent = nullptr);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    static constexpr int CarPlanSlotCount = 2;

    void importCsv();
    void loadCarPlanFile(int slot);
    void loadCarPlanDirectory(int slot);
    bool loadCarPlanPath(int slot, const QString& path);
    void resetCarPlan();
    void resetCarPlanState();
    void clearCarPlanCache(int slot);
    void updateCarPlanForCurrentRow();
    void updateCarPlanFromLiveRow(const JustFloatLogRow& row);
    void updateCarPlanFromLiveRow(int slot, const JustFloatLogRow& row);
    bool ensureCarPlanResultForCsvRow(int slot, int row);
    bool loadCsv(const QString& path);
    void populateLocalAddresses();
    bool setUdpMode(bool enabled);
    void updateControlState();
    void startUdpListening();
    bool stopUdpListening(bool resolveRecording = true);
    void readPendingDatagrams();
    void acceptUdpRow(const JustFloatLogRow& row, const QString& peerName);
    void toggleRecording();
    bool stopRecording();
    bool savePendingRecording();
    bool ensureRecordingResolved(const QString& actionName);
    void discardPendingRecording(bool confirm);
    void updateRecordingState();
    void showWaveformWindow();
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
    void drawCarPlanOverlay(QImage* image, int cameraIndex, int slot) const;
    bool carPlanRelationAngleDeg(int slot, float* angleDeg) const;
    QString carPlanInfoText(int slot) const;
    QString carPlanInfoText() const;

    beacon_result_t resultForCamera(int cameraIndex) const;
    QImage syntheticImageForCamera(int cameraIndex) const;
    const JustFloatLogRow* currentRow() const;

    JustFloatLog m_log;
    JustFloatLogRow m_liveRow;
    JustFloatCsvRecorder m_csvRecorder;
    WaveformHistoryStore m_waveformHistory;
    std::array<CarPlanRunner, CarPlanSlotCount> m_carPlanRunners;
    std::array<QVector<CarPlanResult>, CarPlanSlotCount> m_carPlanCaches;
    std::array<CarPlanResult, CarPlanSlotCount> m_currentCarPlanResults = {};
    int m_currentRow = -1;
    int m_focusCamera = -1;
    quint64 m_udpPacketCount = 0;
    quint64 m_udpErrorCount = 0;
    QString m_lastUdpPeer;
    QElapsedTimer m_udpElapsedTimer;
    bool m_playing = false;
    bool m_udpMode = false;
    bool m_hasLiveRow = false;
    bool m_waveformHistoryErrorShown = false;
    std::array<bool, CarPlanSlotCount> m_hasCurrentCarPlanResults = {};

    QTimer* m_timer = nullptr;
    QUdpSocket* m_udpSocket = nullptr;
    LogWaveformWindow* m_waveformWindow = nullptr;
    QLabel* m_statusLabel = nullptr;
    QTextEdit* m_infoText = nullptr;
    QComboBox* m_modeCombo = nullptr;
    QComboBox* m_addressCombo = nullptr;
    QLineEdit* m_portEdit = nullptr;
    QPushButton* m_listenButton = nullptr;
    QPushButton* m_recordButton = nullptr;
    QPushButton* m_discardRecordingButton = nullptr;
    QPushButton* m_waveformButton = nullptr;
    QPushButton* m_importButton = nullptr;
    QPushButton* m_loadCarPlanButton = nullptr;
    QPushButton* m_loadCarPlanDirButton = nullptr;
    std::array<QPushButton*, CarPlanSlotCount> m_loadCarPlanButtons = {};
    std::array<QPushButton*, CarPlanSlotCount> m_loadCarPlanDirButtons = {};
    QPushButton* m_resetCarPlanButton = nullptr;
    QPushButton* m_previousButton = nullptr;
    QPushButton* m_nextButton = nullptr;
    QPushButton* m_playButton = nullptr;
    QPushButton* m_returnGridButton = nullptr;
    QSlider* m_slider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QComboBox* m_speedCombo = nullptr;
    QLabel* m_carPlanStatusLabel = nullptr;
    std::array<QLabel*, CarPlanSlotCount> m_carPlanStatusLabels = {};
    QHBoxLayout* m_cameraLayout = nullptr;
    std::array<QGroupBox*, 3> m_cameraGroups = {};
    std::array<VideoWidget*, 3> m_videoWidgets = {};
    std::array<QLabel*, 3> m_cameraInfoLabels = {};
};

#endif
