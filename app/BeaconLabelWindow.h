#ifndef BEACON_LABEL_WINDOW_H
#define BEACON_LABEL_WINDOW_H

#include "AlgorithmRunner.h"
#include "BeaconLabelSession.h"
#include "VideoReader.h"

#include <QHash>
#include <QImage>
#include <QTemporaryDir>
#include <QWidget>

class QCloseEvent;
class QLabel;
class QComboBox;
class QPushButton;
class QScrollArea;
class QSlider;
class QSpinBox;
class VideoWidget;

class BeaconLabelWindow : public QWidget
{
    Q_OBJECT

public:
    explicit BeaconLabelWindow(QWidget* parent = nullptr);
    bool openVideo(const QString& path);
    bool openSession(const QString& path);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void chooseVideo();
    void chooseSession();
    void chooseAlgorithm();
    void evaluateLabels();
    void saveAs();
    void showFrame(int frameIndex);
    void renderCurrentFrame();
    void addAnnotation(const QString& shapeType, const QVector<QPointF>& points);
    void removeNearestAnnotation(const QPointF& imagePoint);
    void setCurrentState(BeaconLabelFrameState state, bool lamp);
    bool lampMode() const;
    void updateAnnotationMode();
    void clearCurrentFrame();
    void moveFrame(int offset);
    void moveSample(int direction);
    void moveToPending(int direction);
    void updateSummary();
    void updateZoom();
    bool saveCurrent(bool showError);
    beacon_result_t processCurrentFrame();

    BeaconLabelSession m_session;
    VideoReader m_reader;
    AlgorithmRunner m_runner;
    QTemporaryDir m_algorithmBuildDir;
    QHash<int, beacon_result_t> m_evaluationResults;
    QImage m_currentImage;
    int m_currentFrame = -1;
    bool m_dirty = false;
    bool m_algorithmLoaded = false;
    bool m_downCoordinates = false;

    VideoWidget* m_videoWidget = nullptr;
    QScrollArea* m_scrollArea = nullptr;
    QLabel* m_sessionLabel = nullptr;
    QLabel* m_frameLabel = nullptr;
    QLabel* m_summaryLabel = nullptr;
    QLabel* m_algorithmLabel = nullptr;
    QLabel* m_evaluationLabel = nullptr;
    QLabel* m_pixelLabel = nullptr;
    QLabel* m_saveLabel = nullptr;
    QSlider* m_frameSlider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QSpinBox* m_strideSpin = nullptr;
    QSpinBox* m_zoomSpin = nullptr;
    QComboBox* m_annotationModeCombo = nullptr;
    QComboBox* m_cameraCombo = nullptr;
};

#endif
