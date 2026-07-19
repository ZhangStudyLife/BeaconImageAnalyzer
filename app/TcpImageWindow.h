#ifndef TCP_IMAGE_WINDOW_H
#define TCP_IMAGE_WINDOW_H

#include "AlgorithmRunner.h"
#include "AnnotationModel.h"
#include "BeaconParameterDiagnostic.h"
#include "BimgImageFrameParser.h"
#include "beacon_image.h"

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
    void render();
    void saveCurrentFrame();
    void chooseSaveDirectory();
    void startRecording();
    void stopRecording();
    void appendRecordingFrame(const QImage& rendered);
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
    BimgImageFrame m_streamFrame;
    QHash<int, BimgParameterSnapshot> m_parameterSnapshots;
    QVector<QImage> m_recentRawFrames;
    QVector<QImage> m_diagnosticFrames;
    QImage m_diagnosticGrayImage;
    QRectF m_diagnosticRegion;
    quint64 m_crcErrorCount = 0;
    quint64 m_protocolErrorCount = 0;
    quint16 m_port = 0;
    int m_frameIndex = -1;
    bool m_paused = false;
    bool m_autoSave = false;
    bool m_showOverlay = true;
    bool m_recording = false;
    bool m_diagnosticFrozen = false;

    beacon_result_t m_result = {};
    AlgorithmProcessProfile m_processProfile = {};
    cv::VideoWriter m_writer;
    QFutureWatcher<BeaconDiagnosticResult>* m_diagnosticWatcher = nullptr;

    QComboBox* m_addressCombo = nullptr;
    QLineEdit* m_portEdit = nullptr;
    QPushButton* m_listenButton = nullptr;
    QLabel* m_statusLabel = nullptr;
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
    QComboBox* m_viewModeCombo = nullptr;
    QComboBox* m_instanceCombo = nullptr;
    QCheckBox* m_enableInstanceCheck = nullptr;
    QCheckBox* m_overlayCheck = nullptr;
    QCheckBox* m_autoSaveCheck = nullptr;
};

#endif
