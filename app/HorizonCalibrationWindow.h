#ifndef HORIZON_CALIBRATION_WINDOW_H
#define HORIZON_CALIBRATION_WINDOW_H

#include "DownGroundRangeCalibration.h"
#include "HorizonCalibration.h"
#include "VideoReader.h"

#include <QWidget>

class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class VideoWidget;

class HorizonCalibrationWindow : public QWidget
{
    Q_OBJECT

public:
    explicit HorizonCalibrationWindow(QWidget* parent = nullptr);
    bool openSession(const QString& path);

private:
    void chooseSession();
    void chooseModel();
    bool openModel(const QString& path);
    void showFrame(int frameIndex);
    void saveCurve(const QString& shapeType, const QVector<QPointF>& points);
    void clearCurrentFrame();
    void skipCurrentFrame();
    void fitModel();
    void exportModel();
    void moveFrame(int direction);
    void moveToMatch(int direction, bool unannotated);
    bool saveSession();
    QString defaultModelPath() const;
    void updateFitText();

    HorizonCalibrationSession m_session;
    HorizonFisheyeModel m_importedModel;
    DownGroundRangeModel m_importedDownModel;
    DownGroundRangeFitResult m_downFit;
    VideoReader m_reader;
    int m_currentFrame = -1;
    int m_availableFrames = 0;

    VideoWidget* m_videoWidget = nullptr;
    QLabel* m_sessionLabel = nullptr;
    QLabel* m_frameLabel = nullptr;
    QLabel* m_fitLabel = nullptr;
    QSlider* m_frameSlider = nullptr;
    QSpinBox* m_frameSpin = nullptr;
    QPushButton* m_importModelButton = nullptr;
    QPushButton* m_fitButton = nullptr;
    QPushButton* m_exportButton = nullptr;
};

#endif
