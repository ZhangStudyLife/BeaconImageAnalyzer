#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "TelemetryProtocol.h"

#include <QFile>
#include <QMainWindow>
#include <QVector>

#include <array>

class CameraView;
class QCloseEvent;
class QComboBox;
class QLabel;
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
    void importCsv();
    void toggleRecording();
    void stopRecording();
    void showFrame(const TelemetryFrame& frame);
    void setReplayIndex(int index);
    void togglePlayback();
    void advancePlayback();
    void scheduleNextFrame();
    int playbackIntervalMs() const;
    void updateControls();
    void updateStatus(const QString& message = QString());

    QUdpSocket* m_udpSocket = nullptr;
    QTimer* m_playbackTimer = nullptr;
    QComboBox* m_modeCombo = nullptr;
    QComboBox* m_addressCombo = nullptr;
    QSpinBox* m_portSpin = nullptr;
    QPushButton* m_listenButton = nullptr;
    QPushButton* m_recordButton = nullptr;
    QPushButton* m_importButton = nullptr;
    std::array<CameraView*, 3> m_cameraViews{};
    std::array<QLabel*, 3> m_cameraInfoLabels{};
    QPushButton* m_playButton = nullptr;
    QPushButton* m_previousButton = nullptr;
    QPushButton* m_nextButton = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QSlider* m_timeline = nullptr;
    QComboBox* m_speedCombo = nullptr;
    QLabel* m_flightInfoLabel = nullptr;
    QLabel* m_motionInfoLabel = nullptr;
    QLabel* m_statusLabel = nullptr;

    QVector<TelemetryFrame> m_replayFrames;
    QFile m_recordFile;
    QString m_lastSender;
    int m_replayIndex = -1;
    quint64 m_packetCount = 0;
    quint64 m_errorCount = 0;
    quint64 m_recordedCount = 0;
    bool m_liveMode = true;
    bool m_playing = false;
};

#endif
