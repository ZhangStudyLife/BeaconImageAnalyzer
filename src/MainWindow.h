#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "TelemetryProtocol.h"
#include "CarPlan3Model.h"

#include <QElapsedTimer>
#include <QMainWindow>
#include <QVector>

#include <array>

class CameraView;
class CoordinateWindow;
class QCloseEvent;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTimer;
class QUdpSocket;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void buildUi();
    void populateAddresses();
    void setLiveMode();
    void setReplayMode();
    void startListening();
    void stopListening();
    void readPendingDatagrams();
    void flushLiveFrame();
    void importCsv();
    void toggleRecording();
    void startRecording();
    void stopAndSaveRecording();
    void showFrame(const TelemetryFrame& frame);
    void setReplayIndex(int index);
    void jumpToTimestamp();
    void togglePlayback();
    void advancePlayback();
    void scheduleNextFrame();
    int playbackIntervalMs() const;
    void updateControls();
    void updateStatus(const QString& message = QString());

    QUdpSocket* m_udpSocket = nullptr;
    QTimer* m_playbackTimer = nullptr;
    QTimer* m_recordingTimer = nullptr;
    QTimer* m_uiTimer = nullptr;
    QComboBox* m_modeCombo = nullptr;
    QComboBox* m_addressCombo = nullptr;
    QSpinBox* m_portSpin = nullptr;
    QPushButton* m_listenButton = nullptr;
    QPushButton* m_recordButton = nullptr;
    QPushButton* m_centerWindowButton = nullptr;
    QPushButton* m_modelWindowButton = nullptr;
    QPushButton* m_globalWindowButton = nullptr;
    QPushButton* m_importButton = nullptr;
    std::array<CameraView*, 3> m_cameraViews{};
    QPushButton* m_playButton = nullptr;
    QPushButton* m_previousButton = nullptr;
    QPushButton* m_nextButton = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QSlider* m_timeline = nullptr;
    QComboBox* m_speedCombo = nullptr;
    QLineEdit* m_timestampInput = nullptr;
    QLabel* m_flightInfoLabel = nullptr;
    QLabel* m_motionInfoLabel = nullptr;
    QLabel* m_statusLabel = nullptr;
    CoordinateWindow* m_centerWindow = nullptr;
    CoordinateWindow* m_modelWindow = nullptr;
    CoordinateWindow* m_globalWindow = nullptr;
    CarPlan3Model m_liveCarPlan3;

    QVector<TelemetryFrame> m_replayFrames;
    QVector<TelemetryFrame> m_recordedFrames;
    QElapsedTimer m_recordingElapsed;
    QString m_lastSender;
    TelemetryFrame m_pendingFrame;
    int m_replayIndex = -1;
    quint64 m_packetCount = 0;
    quint64 m_errorCount = 0;
    bool m_liveMode = true;
    bool m_playing = false;
    bool m_recording = false;
    bool m_hasPendingFrame = false;
};

#endif
