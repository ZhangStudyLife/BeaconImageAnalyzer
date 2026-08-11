#ifndef HORIZON_CALIBRATION_RECORDER_H
#define HORIZON_CALIBRATION_RECORDER_H

#include <QImage>
#include <QMutex>
#include <QQueue>
#include <QSize>
#include <QString>
#include <QThread>
#include <QWaitCondition>

struct HorizonCalibrationRecorderConfig
{
    QString sessionPath;
    QString videoPath;
    QString csvPath;
    QSize imageSize;
    quint8 cameraId = 0;
    quint8 sourceCameraId = 0xffU;
    quint8 bimgProtocolVersion = 2U;
    double fps = 50.0;
};

struct HorizonCalibrationRecorderFrame
{
    QImage image;
    quint32 bimgSequence = 0;
    qint64 hostTimeMs = 0;
    quint8 cameraId = 0;
    quint8 sourceCameraId = 0xffU;
    quint8 bimgProtocolVersion = 2U;
    quint32 sourceFrameSequence = 0;
    quint32 captureTimeMs = 0;
    quint8 sourceFrameCameraId = 0xffU;
    quint8 physicalBoardId = 0xffU;
    bool sourceFrameValid = false;
    bool captureTimeValid = false;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double heightMm = 0.0;
    bool attitudeValid = false;
    bool heightValid = false;
};

class HorizonCalibrationRecorder : public QThread
{
    Q_OBJECT

public:
    explicit HorizonCalibrationRecorder(QObject* parent = nullptr);
    ~HorizonCalibrationRecorder() override;

    bool begin(const HorizonCalibrationRecorderConfig& config,
               QString* errorMessage = nullptr);
    bool enqueue(const HorizonCalibrationRecorderFrame& frame);
    void finish();
    bool isAccepting() const;
    int writtenFrameCount() const;
    quint64 droppedFrameCount() const;

signals:
    void recordingFinished(const QString& sessionPath, int frameCount, quint64 droppedFrames);
    void recordingFailed(const QString& message);

protected:
    void run() override;

private:
    static constexpr int MaximumQueueSize = 8;

    mutable QMutex m_mutex;
    QWaitCondition m_condition;
    QQueue<HorizonCalibrationRecorderFrame> m_queue;
    HorizonCalibrationRecorderConfig m_config;
    bool m_accepting = false;
    bool m_finishRequested = false;
    bool m_hasSequence = false;
    quint32 m_lastSequence = 0;
    int m_writtenFrames = 0;
    quint64 m_sourceDroppedFrames = 0;
    quint64 m_queueDroppedFrames = 0;
};

#endif
