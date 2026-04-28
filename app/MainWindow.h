#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "AlgorithmRunner.h"
#include "AnnotationModel.h"
#include "VideoReader.h"
#include "beacon_image.h"

#include <QMainWindow>
#include <QTimer>

class AnnotationPanel;
class QCloseEvent;
class QComboBox;
class QKeyEvent;
class QLabel;
class QSlider;
class QSpinBox;
class QDoubleSpinBox;
class QTextEdit;
class QPushButton;
class VideoWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void openVideo();
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
    void updateFrameInfo(const beacon_result_t& result);
    void updateCurrentAnnotationInfo();
    void updateAnnotationList();
    void togglePlayPause();
    void updatePlayPauseButton();
    bool validateAnnotationInput(const QStringList& types,
                                 const QVector<ErrorCircle>& errorCircles,
                                 const QString& actionName) const;
    void restoreLastSession();
    void saveProject();
    bool loadProject();
    QString projectPathForVideo(const QString& videoPath) const;
    QString viewMode() const;
    QString defaultOutputPath(const QString& suffix) const;
    double frameTime(int frame) const;

    VideoReader m_reader;
    AlgorithmRunner m_runner;
    AnnotationModel m_annotations;
    QTimer m_playTimer;

    VideoWidget* m_videoWidget = nullptr;
    QLabel* m_videoInfoLabel = nullptr;
    QLabel* m_frameInfoLabel = nullptr;
    QLabel* m_pixelInfoLabel = nullptr;
    QLabel* m_currentAnnotationsLabel = nullptr;
    QLabel* m_algorithmInfoLabel = nullptr;
    QTextEdit* m_resultText = nullptr;
    AnnotationPanel* m_annotationPanel = nullptr;
    QPushButton* m_playPauseButton = nullptr;
    QSlider* m_slider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QDoubleSpinBox* m_timeSpin = nullptr;
    QComboBox* m_viewModeCombo = nullptr;

    QString m_currentVideoPath;
    int m_currentFrame = 0;
    int m_zoom = 1;
    bool m_showOverlay = true;
    bool m_updatingControls = false;
    double m_usedFps = 50.0;
    int m_segmentStartFrame = -1;
    int m_segmentEndFrame = -1;
    beacon_result_t m_currentResult = {};
};

#endif
