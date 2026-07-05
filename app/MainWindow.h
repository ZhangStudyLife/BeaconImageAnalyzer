#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "AlgorithmRunner.h"
#include "AnnotationModel.h"
#include "TcpImageWindow.h"
#include "VideoReader.h"
#include "beacon_image.h"

#include <QMainWindow>
#include <QHash>
#include <QPair>
#include <QTimer>

class AnnotationPanel;
class QCheckBox;
class QCloseEvent;
class QComboBox;
class QKeyEvent;
class QGridLayout;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QSlider;
class QSpinBox;
class QDoubleSpinBox;
class QTextEdit;
class QPushButton;
class VideoWidget;

struct AnalyzerInstance
{
    int id = -1;
    QString name;
    QString rootDir;
    QString algorithmPath;
    AlgorithmRunner runner;
    AnnotationModel annotations;
    double usedFps = 50.0;
    int segmentStartFrame = -1;
    int segmentEndFrame = -1;
    beacon_result_t currentResult = {};
    AlgorithmProcessProfile currentProfile = {};
    beacon_result_t previousAutoPauseResult = {};
    QHash<int, beacon_result_t> temporalFrameCache;
    QHash<int, AlgorithmProcessProfile> temporalProfileCache;
    int temporalLastFrame = -1;
    int previousAutoPauseFrame = -1;
    bool hasPreviousAutoPauseResult = false;
    QVector<int> pendingAutoBatchRows;
    QVector<CorrectionShape> pendingAutoMatchedCorrections;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void loadInstance();
    void addInstance();
    void deleteCurrentInstance();
    void importAlgorithmFile();
    void openVideo();
    void configureTcpReceiver();
    void openJustFloatLogWindow();
    void saveAnnotation();
    void loadAnnotation();
    void exportMarkedAvi();
    void exportCsv();
    void play();
    void pause();
    void nextFrame();
    void previousFrame();
    void jumpToFrame();
    void jumpToTime();
    void showFrameFromSlider(int value);
    void markCurrentFrameAnnotation(const QStringList& types,
                                    const QVector<ErrorCircle>& errorCircles,
                                    const QString& description);
    void setSegmentStart();
    void setSegmentEnd();
    void saveSegmentAnnotation(const QStringList& types,
                               const QVector<ErrorCircle>& errorCircles,
                               const QString& description);
    void deleteAnnotation(int row);
    void deleteCorrection(int row);
    void deleteAnnotations(const QVector<int>& rows);
    void deleteCorrections(const QVector<int>& rows);
    void saveCurrentFrameCorrections(const QVector<CorrectionShape>& corrections);
    void batchAddCorrections(const QVector<int>& correctionRows,
                             int startFrame,
                             int endFrame,
                             double overlapPixelThreshold);
    void autoMatchCorrectionFrames(const QVector<int>& correctionRows,
                                   int backwardMaxFrames,
                                   int forwardMaxFrames,
                                   double positionThreshold,
                                   double overlapPixelThreshold);
    void batchAddCorrectionsToFrames(const QVector<int>& correctionRows, const QVector<int>& frames);
    void addCorrectionShape(const QString& shapeType, const QVector<QPointF>& points);
    void autoIdentifyCorrectionTargets();
    void openAlgorithmLocation();
    void jumpToRecordFrame(int frame);
    void updateHoverPixelInfo(int x, int y, int gray, bool valid);

private:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;
    void buildUi();
    void buildMenus();
    bool loadVideoFile(const QString& path, bool restoreProject, int fallbackFrame);
    void showFrame(int frameIndex);
    void advancePlayingInstances();
    AnalyzerInstance* currentInstance();
    const AnalyzerInstance* currentInstance() const;
    AnalyzerInstance* instanceById(int id);
    const AnalyzerInstance* instanceById(int id) const;
    AnalyzerInstance* createInstance(const QString& rootDir,
                                     const QString& algorithmPath,
                                     const QString& name,
                                     bool compileAlgorithm,
                                     QString* errorMessage);
    AnalyzerInstance* requireCurrentInstance(const QString& actionName);
    bool ensureDefaultInstance(QString* errorMessage = nullptr);
    void selectInstance(int id);
    void refreshInstanceList();
    void updateSplitLayout();
    void updateCurrentVideoWidget();
    void renderInstance(AnalyzerInstance* instance);
    void renderInstance(AnalyzerInstance* instance,
                        const QImage& gray,
                        const beacon_result_t& result,
                        int frameIndex);
    void renderAllDisplayedInstances();
    void renderAllDisplayedInstances(const QImage& gray,
                                     const QVector<QPair<AnalyzerInstance*, beacon_result_t>>& results);
    void resetInstanceTemporal(AnalyzerInstance* instance);
    beacon_result_t processCausalFrame(AnalyzerInstance* instance, int frameIndex, const QImage& gray);
    bool rebuildTemporalCacheToFrame(AnalyzerInstance* instance, int targetFrame, QString* errorMessage);
    void showLiveFrame(const QImage& gray, quint16 localPort, const QString& peerName);
    int slotForInstance(int instanceId) const;
    int slotAtGlobalPos(const QPoint& globalPos) const;
    void setInstanceVisible(int instanceId, bool visible);
    void handleSlotActivated(int slot);
    void handleSlotMiddleDragStarted(int slot);
    void handleSlotMiddleDragReleased(int slot, const QPoint& globalPos);
    void handleSlotContextCorrection(int slot, const QPointF& imagePoint, const QPoint& globalPos);
    void refreshCurrentInstanceUi();
    bool autoPauseTriggered(int previousFrame,
                            int currentFrame,
                            const QVector<QPair<AnalyzerInstance*, beacon_result_t>>& results,
                            QString* reason);
    QVector<int> targetIndicesNearPoint(const QPointF& imagePoint, double radiusPixels) const;
    bool promptExpectedIndex(const QString& title, int* expectedIndex);
    bool promptDescription(const QString& title, QString* description);
    bool promptMissedCircle(const QPointF& center, CorrectionShape* correction);
    void addQuickCorrection(const CorrectionShape& correction);
    void runSingleTargetQuickCorrection(int circleIndex, const QPoint& globalPos);
    void updateFrameInfo(const beacon_result_t& result);
    void updateCurrentAnnotationInfo();
    void updateAnnotationList();
    void togglePlayPause();
    void updatePlayPauseButton();
    void updateTcpStatusLabel(const QString& eventMessage = QString());
    void setPlaybackSpeed(double speed);
    int playbackIntervalMs() const;
    bool validateAnnotationInput(const QStringList& types,
                                 const QVector<ErrorCircle>& errorCircles,
                                 const QString& actionName) const;
    bool collectBatchCorrections(const QVector<int>& correctionRows,
                                 QVector<CorrectionShape>* corrections,
                                 QString* errorMessage) const;
    bool correctionsHaveMixedBatchTypes(const QVector<CorrectionShape>& corrections) const;
    bool correctionsAreAllMissed(const QVector<CorrectionShape>& corrections) const;
    bool processFrameForBatch(int frame, beacon_result_t* result, QString* errorMessage) const;
    bool batchCorrectionsMatchAdjacent(const QVector<CorrectionShape>& corrections,
                                       const beacon_result_t& previous,
                                       const beacon_result_t& next,
                                       double positionThreshold) const;
    bool buildMissedBatchCorrections(const QVector<CorrectionShape>& baseCorrections,
                                     int startFrame,
                                     int endFrame,
                                     double overlapPixelThreshold,
                                     QVector<CorrectionShape>* matchedCorrections,
                                     QVector<int>* matchedFrames,
                                     QVector<int>* failedFrames,
                                     QString* errorMessage) const;
    void appendCorrectionsToFrames(const QVector<CorrectionShape>& corrections,
                                   const QVector<int>& frames,
                                   const QString& actionName);
    void appendResolvedCorrections(const QVector<CorrectionShape>& corrections,
                                   const QString& actionName);
    void restoreLastSession();
    void saveProject();
    bool loadProject();
    QString projectPathForVideo(const QString& videoPath) const;
    QString viewMode() const;
    QString defaultOutputPath(const QString& suffix) const;
    double frameTime(int frame) const;

    QVector<AnalyzerInstance*> m_instances;
    VideoReader m_globalReader;
    QTimer m_playTimer;

    VideoWidget* m_videoWidget = nullptr;
    QVector<VideoWidget*> m_videoWidgets;
    QGridLayout* m_videoGrid = nullptr;
    QListWidget* m_instanceList = nullptr;
    QLabel* m_videoInfoLabel = nullptr;
    QLabel* m_tcpStatusLabel = nullptr;
    QLabel* m_frameInfoLabel = nullptr;
    QLabel* m_pixelInfoLabel = nullptr;
    QLabel* m_currentAnnotationsLabel = nullptr;
    QTextEdit* m_resultText = nullptr;
    AnnotationPanel* m_annotationPanel = nullptr;
    QPushButton* m_playPauseButton = nullptr;
    QSlider* m_slider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QDoubleSpinBox* m_timeSpin = nullptr;
    QComboBox* m_viewModeCombo = nullptr;
    QComboBox* m_speedCombo = nullptr;
    QCheckBox* m_autoPauseEnableCheck = nullptr;
    QCheckBox* m_autoPauseJumpCheck = nullptr;
    QCheckBox* m_autoPauseCountCheck = nullptr;
    QDoubleSpinBox* m_autoPauseJumpThresholdSpin = nullptr;

    QString m_globalVideoPath;
    int m_globalCurrentFrame = 0;
    int m_globalZoom = 1;
    bool m_globalShowOverlay = true;
    bool m_globalPlaying = false;
    double m_globalUsedFps = 50.0;
    double m_globalPlaybackSpeed = 1.0;
    QImage m_liveFrame;
    QString m_livePeerName;
    QString m_liveSaveDir;
    quint16 m_liveLocalPort = 0;
    int m_liveFrameIndex = -1;
    quint16 m_nextTcpPort = 1346;
    bool m_liveMode = false;
    bool m_updatingControls = false;
    QVector<int> m_splitSlotInstanceIds;
    int m_currentInstanceId = -1;
    int m_nextInstanceId = 1;
    int m_dragSourceSlot = -1;
};

#endif
