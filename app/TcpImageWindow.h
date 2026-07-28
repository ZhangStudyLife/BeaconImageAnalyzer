#ifndef TCP_IMAGE_WINDOW_H
#define TCP_IMAGE_WINDOW_H

#include "AlgorithmRunner.h"
#include "AnnotationModel.h"
#include "BeaconParameterDiagnostic.h"
#include "BimgImageFrameParser.h"
#include "HorizonCalibrationRecorder.h"
#include "WaveformHistoryStore.h"
#include "beacon_image.h"

#include <QElapsedTimer>
#include <QHash>
#include <QImage>
#include <QString>
#include <QVector>
#include <QWidget>

#include <opencv2/videoio.hpp>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTcpServer;
class QTcpSocket;
class QTimer;
class LogWaveformWindow;
class VideoWidget;
template<typename T> class QFutureWatcher;

struct TcpInstanceOption
{
    int id = -1;
    QString name;
    AlgorithmRunner* runner = nullptr;
    AnnotationModel* annotations = nullptr;
};

struct TcpListenAddress
{
    QString label;
    QString address;
};

class TcpImageWindow : public QWidget
{
    Q_OBJECT

public:
    explicit TcpImageWindow(QWidget* parent = nullptr);
    ~TcpImageWindow() override;

    quint16 port() const;
    void setAvailableAddresses(const QVector<TcpListenAddress>& addresses);
    void setInstanceOptions(const QVector<TcpInstanceOption>& options);
    void setDefaultSaveDirectory(const QString& path);
    void setSuggestedPort(quint16 port);

private:
    void startListening();
    void stopListening();
    void acceptPendingConnections();
    void readSocketData(QTcpSocket* socket);
    void setParameterSnapshot(const BimgParameterSnapshot& snapshot);
    void removeSocket(QTcpSocket* socket);
    void setFrame(const BimgImageFrame& frame, const QString& peerName);
    void updateAttitude(const BimgImageFrame& frame);
    void appendAttitudeSample(bool valid);
    void refreshAttitudeDisplay();
    void showAttitudeWaveform();
    QString attitudeSourceName() const;
    void render();
    void saveCurrentFrame();
    void chooseSaveDirectory();
    void startRecording();
    void stopRecording();
    void appendRecordingFrame(const QImage& rendered);
    void startCalibrationRecording();
    void stopCalibrationRecording();
    void appendCalibrationFrame(const BimgImageFrame& frame, const QImage& gray);
    void toggleRegionDiagnostic();
    void handleDiagnosticRegion(const QString& shapeType, const QVector<QPointF>& points);
    void finishRegionDiagnostic();
    void resumeLiveDisplay();
    void updateDiagnosticPanel(const BeaconDiagnosticResult& result);
    void setDiagnosticPreview(QLabel* label, const QImage& image);
    QString defaultSavePath(const QString& suffix) const;
    void updateStatus(const beacon_result_t& result = {});
    AlgorithmRunner* selectedRunner() const;
    AnnotationModel* selectedAnnotations() const;

    QTcpServer* m_server = nullptr;
    QHash<QTcpSocket*, BimgImageFrameParser*> m_parsers;
    QVector<TcpInstanceOption> m_instances;

    QString m_peerName;
    QString m_saveDir;
    QImage m_grayImage;
    QImage m_renderedImage;
    QImage m_lastReceivedGrayImage;
    BimgImageFrame m_streamFrame;
    BimgImageFrame m_lastReceivedFrame;
    QHash<int, BimgParameterSnapshot> m_parameterSnapshots;
    WaveformHistoryStore m_attitudeHistory;
    QElapsedTimer m_attitudeElapsed;
    QVector<QImage> m_recentRawFrames;
    QVector<QImage> m_diagnosticFrames;
    QImage m_diagnosticGrayImage;
    QRectF m_diagnosticRegion;
    quint64 m_crcErrorCount = 0;
    quint64 m_protocolErrorCount = 0;
    quint16 m_port = 0;
    qint64 m_attitudeLastMs = -1;
    qint64 m_heightLastMs = -1;
    int m_frameIndex = -1;
    quint8 m_attitudeCameraId = 0xffU;
    float m_attitudeRollDeg = 0.0f;
    float m_attitudePitchDeg = 0.0f;
    float m_heightMm = 0.0f;
    bool m_paused = false;
    bool m_autoSave = false;
    bool m_showOverlay = true;
    bool m_recording = false;
    bool m_calibrationFinalizing = false;
    bool m_diagnosticFrozen = false;
    bool m_attitudeProvided = false;
    bool m_attitudeHasValue = false;
    bool m_attitudeValid = false;
    bool m_heightProvided = false;
    bool m_heightHasValue = false;
    bool m_heightValid = false;
    bool m_attitudeTimeoutGapWritten = false;
    bool m_attitudeHistoryError = false;

    beacon_result_t m_result = {};
    AlgorithmProcessProfile m_processProfile = {};
    cv::VideoWriter m_writer;
    QFutureWatcher<BeaconDiagnosticResult>* m_diagnosticWatcher = nullptr;
    LogWaveformWindow* m_attitudeWaveformWindow = nullptr;
    HorizonCalibrationRecorder* m_calibrationRecorder = nullptr;
    QTimer* m_attitudeTimer = nullptr;

    QComboBox* m_addressCombo = nullptr;
    QLineEdit* m_portEdit = nullptr;
    QPushButton* m_listenButton = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLabel* m_attitudeRollLabel = nullptr;
    QLabel* m_attitudePitchLabel = nullptr;
    QLabel* m_attitudeStateLabel = nullptr;
    QLabel* m_heightLabel = nullptr;
    QLabel* m_heightStateLabel = nullptr;
    VideoWidget* m_videoWidget = nullptr;
    QWidget* m_diagnosticPanel = nullptr;
    QLabel* m_diagnosticStateLabel = nullptr;
    QLabel* m_diagnosticParameterLabel = nullptr;
    QLabel* m_diagnosticEffectLabel = nullptr;
    QLabel* m_diagnosticStatsLabel = nullptr;
    QLabel* m_diagnosticBeforePreview = nullptr;
    QLabel* m_diagnosticAfterPreview = nullptr;
    QPushButton* m_diagnosticButton = nullptr;
    QPushButton* m_pauseButton = nullptr;
    QPushButton* m_recordButton = nullptr;
    QPushButton* m_calibrationRecordButton = nullptr;
    QPushButton* m_attitudeWaveformButton = nullptr;
    QComboBox* m_viewModeCombo = nullptr;
    QComboBox* m_instanceCombo = nullptr;
    QCheckBox* m_enableInstanceCheck = nullptr;
    QCheckBox* m_overlayCheck = nullptr;
    QCheckBox* m_autoSaveCheck = nullptr;
};

#endif
