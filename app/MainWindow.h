#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "AlgorithmRunner.h"
#include "AnnotationModel.h"
#include "FusionRunner.h"
#include "VideoReader.h"
#include "beacon_fusion.h"
#include "beacon_image.h"

#include <QElapsedTimer>
#include <QMainWindow>
#include <QTimer>
#include <array>

class AnnotationPanel;
class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QGridLayout;
class QLabel;
class QKeyEvent;
class QSlider;
class QSpinBox;
class QTextEdit;
class QPushButton;
class RadarWidget;
class VideoWidget;

struct CameraChannel
{
    int index = 0;
    QString name;
    QString videoPath;
    QString algorithmPath;
    VideoReader reader;
    AlgorithmRunner runner;
    AnnotationModel annotations;
    beacon_result_t currentResult = {};
    QImage currentGray;
    double usedFps = 50.0;
    int syncFrame = 0;
    bool loaded = false;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void importAllCameraVideos();
    void importCameraVideo(int cameraIndex);
    void importCameraAlgorithm(int cameraIndex);
    void importFusionAlgorithm();
    void saveAnnotation();
    void loadAnnotation();
    void exportMarkedAvi();
    void exportCsv();
    void play();
    void pause();
    void applyFrameSynchronization();
    void autoFindFrameSynchronization();
    void nextFrame();
    void previousFrame();
    void jumpToFrame();
    void jumpToTime();
    void showFrameFromSlider(int value);
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
    void deleteAnnotation(int row);
    void deleteCorrection(int row);
    void deleteAnnotations(const QVector<int>& rows);
    void deleteCorrections(const QVector<int>& rows);
    void addCorrectionShape(const QString& shapeType, const QVector<QPointF>& points);
    void autoIdentifyCorrectionTargets();
    void jumpToRecordFrame(int frame);
    void updateHoverPixelInfo(int x, int y, int gray, bool valid);

private:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

    void buildUi();
    void buildMenus();
    void buildCameraPanel(QGridLayout* layout, int cameraIndex, int column);
    bool loadCameraVideo(int cameraIndex, const QString& path);
    bool loadCameraAlgorithm(int cameraIndex, const QString& path);
    bool hasAnyVideo() const;
    bool hasAllVideos() const;
    int timelineMinFrame() const;
    int timelineMaxFrame() const;
    int timelineFrameCount() const;
    int cameraFrameForTimeline(const CameraChannel& camera, int timelineFrame) const;
    void showFrame(int frameIndex);
    void renderCamera(CameraChannel* camera);
    void updateFusion();
    void updateCameraInfo(const CameraChannel& camera);
    void updateAllCameraInfo();
    void updateFrameInfo();
    bool findFrozenTailStartFrame(const CameraChannel& camera, int* frameIndex, QString* errorMessage) const;
    void updateAnnotationList();
    void refreshCurrentCameraUi();
    void selectCamera(int cameraIndex);
    void togglePlayPause();
    void updatePlayPauseButton();
    void setPlaybackSpeed(double speed);
    int playbackIntervalMs() const;
    void resetPlaybackClock();
    double frameTime(int frame) const;
    QString viewMode() const;
    QString defaultOutputPath(int cameraIndex, const QString& suffix) const;
    QString annotationPathForCamera(int cameraIndex) const;
    CameraChannel* currentCamera();
    const CameraChannel* currentCamera() const;
    bool isValidCameraIndex(int cameraIndex) const;

    std::array<CameraChannel, BEACON_CAMERA_COUNT> m_cameras;
    FusionRunner m_fusionRunner;
    beacon_fusion_result_t m_fusionResult = {};
    QTimer m_playTimer;

    std::array<VideoWidget*, BEACON_CAMERA_COUNT> m_videoWidgets = {};
    std::array<QLabel*, BEACON_CAMERA_COUNT> m_cameraInfoLabels = {};
    std::array<QSpinBox*, BEACON_CAMERA_COUNT> m_syncFrameSpins = {};
    std::array<QPushButton*, BEACON_CAMERA_COUNT> m_cameraSelectButtons = {};
    QTextEdit* m_fusionText = nullptr;
    RadarWidget* m_radarWidget = nullptr;
    AnnotationPanel* m_annotationPanel = nullptr;
    QLabel* m_videoInfoLabel = nullptr;
    QLabel* m_frameInfoLabel = nullptr;
    QLabel* m_pixelInfoLabel = nullptr;
    QPushButton* m_playPauseButton = nullptr;
    QSlider* m_slider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QDoubleSpinBox* m_timeSpin = nullptr;
    QComboBox* m_viewModeCombo = nullptr;
    QComboBox* m_speedCombo = nullptr;
    QCheckBox* m_showOverlayCheck = nullptr;

    int m_currentCameraIndex = 0;
    int m_timelineFrame = 0;
    bool m_playing = false;
    bool m_showOverlay = true;
    double m_playbackSpeed = 1.0;
    QElapsedTimer m_playbackClock;
    int m_playbackStartFrame = 0;
    bool m_updatingControls = false;
};

#endif
