#include "HorizonCalibrationRecorder.h"

#include "HorizonCalibration.h"
#include "ImageFrameSidecar.h"

#include <QFile>
#include <QMutexLocker>
#include <QStringConverter>
#include <QTextStream>

#include <opencv2/videoio.hpp>

#include <cmath>

namespace
{
void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage != nullptr)
    {
        *errorMessage = message;
    }
}

cv::Mat grayMat(const QImage& source)
{
    const QImage image = source.convertToFormat(QImage::Format_Grayscale8);
    return cv::Mat(image.height(),
                   image.width(),
                   CV_8UC1,
                   const_cast<uchar*>(image.bits()),
                   (size_t)image.bytesPerLine()).clone();
}

QString csvNumber(double value)
{
    return std::isfinite(value) ? QString::number(value, 'g', 9) : QString();
}
}

HorizonCalibrationRecorder::HorizonCalibrationRecorder(QObject* parent)
    : QThread(parent)
{
}

HorizonCalibrationRecorder::~HorizonCalibrationRecorder()
{
    finish();
    wait();
}

bool HorizonCalibrationRecorder::begin(const HorizonCalibrationRecorderConfig& config,
                                       QString* errorMessage)
{
    QMutexLocker locker(&m_mutex);
    if (isRunning() || m_accepting)
    {
        setError(errorMessage, QStringLiteral("标定录像已经在运行。"));
        return false;
    }
    if (config.sessionPath.isEmpty() || config.videoPath.isEmpty() || config.csvPath.isEmpty()
        || config.imageSize.width() <= 0 || config.imageSize.height() <= 0
        || config.cameraId > HorizonCameraDown || config.sourceCameraId > HorizonCameraBack
        || (config.bimgProtocolVersion != 2U && config.bimgProtocolVersion != 3U)
        || config.fps <= 0.0)
    {
        setError(errorMessage, QStringLiteral("标定录像配置无效。"));
        return false;
    }

    m_config = config;
    m_queue.clear();
    m_accepting = true;
    m_finishRequested = false;
    m_hasSequence = false;
    m_lastSequence = 0;
    m_writtenFrames = 0;
    m_sourceDroppedFrames = 0;
    m_queueDroppedFrames = 0;
    start();
    return true;
}

bool HorizonCalibrationRecorder::enqueue(const HorizonCalibrationRecorderFrame& frame)
{
    QMutexLocker locker(&m_mutex);
    if (!m_accepting || frame.image.isNull() || frame.image.size() != m_config.imageSize
        || frame.cameraId != m_config.cameraId || frame.sourceCameraId != m_config.sourceCameraId
        || frame.bimgProtocolVersion != m_config.bimgProtocolVersion)
    {
        return false;
    }

    if (m_hasSequence)
    {
        const quint32 delta = frame.bimgSequence - m_lastSequence;
        if (delta > 1U && delta < 0x80000000U)
        {
            m_sourceDroppedFrames += delta - 1U;
        }
    }
    m_lastSequence = frame.bimgSequence;
    m_hasSequence = true;

    if (m_queue.size() >= MaximumQueueSize)
    {
        ++m_queueDroppedFrames;
        return false;
    }
    m_queue.enqueue(frame);
    m_condition.wakeOne();
    return true;
}

void HorizonCalibrationRecorder::finish()
{
    QMutexLocker locker(&m_mutex);
    m_accepting = false;
    m_finishRequested = true;
    m_condition.wakeOne();
}

bool HorizonCalibrationRecorder::isAccepting() const
{
    QMutexLocker locker(&m_mutex);
    return m_accepting;
}

int HorizonCalibrationRecorder::writtenFrameCount() const
{
    QMutexLocker locker(&m_mutex);
    return m_writtenFrames;
}

quint64 HorizonCalibrationRecorder::droppedFrameCount() const
{
    QMutexLocker locker(&m_mutex);
    return m_sourceDroppedFrames + m_queueDroppedFrames;
}

void HorizonCalibrationRecorder::run()
{
    HorizonCalibrationSession session;
    session.sessionPath = m_config.sessionPath;
    session.videoPath = m_config.videoPath;
    session.csvPath = m_config.csvPath;
    session.imageSize = m_config.imageSize;
    session.cameraId = m_config.cameraId;
    session.sourceCameraId = m_config.sourceCameraId;
    session.bimgProtocolVersion = m_config.bimgProtocolVersion;
    session.heightRecorded = true;
    session.status = QStringLiteral("recording");
    QString error;
    if (!HorizonCalibration::saveSession(session, &error))
    {
        QMutexLocker locker(&m_mutex);
        m_accepting = false;
        emit recordingFailed(QStringLiteral("无法创建 HCAL 会话：%1").arg(error));
        return;
    }

    QFile csvFile(m_config.csvPath);
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        QMutexLocker locker(&m_mutex);
        m_accepting = false;
        emit recordingFailed(QStringLiteral("无法创建 HCAL CSV：%1").arg(csvFile.errorString()));
        return;
    }
    QTextStream csv(&csvFile);
    csv.setEncoding(QStringConverter::Utf8);
    csv << "frame_index,bimg_sequence,host_time_ms,camera_id,source_camera_id,roll_deg,pitch_deg,height_mm,attitude_valid,height_valid\n";
    csv.flush();

    const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoWriter writer;
    if (!writer.open(m_config.videoPath.toStdString(),
                     fourcc,
                     m_config.fps,
                     cv::Size(m_config.imageSize.width(), m_config.imageSize.height()),
                     false))
    {
        QMutexLocker locker(&m_mutex);
        m_accepting = false;
        emit recordingFailed(QStringLiteral("无法创建标定 AVI 文件。"));
        return;
    }

    ImageFrameSidecarWriter sidecar;
    if (m_config.bimgProtocolVersion == 3U &&
        !sidecar.start(m_config.videoPath, &error))
    {
        writer.release();
        csvFile.close();
        QMutexLocker locker(&m_mutex);
        m_accepting = false;
        emit recordingFailed(
            QStringLiteral("无法创建标定录像逐帧侧车 %1：%2")
                .arg(imageFrameSidecarPathForVideo(m_config.videoPath), error));
        return;
    }

    while (true)
    {
        HorizonCalibrationRecorderFrame frame;
        {
            QMutexLocker locker(&m_mutex);
            while (m_queue.isEmpty() && !m_finishRequested)
            {
                m_condition.wait(&m_mutex);
            }
            if (m_queue.isEmpty() && m_finishRequested)
            {
                break;
            }
            frame = m_queue.dequeue();
        }

        int frameIndex = 0;
        {
            QMutexLocker locker(&m_mutex);
            frameIndex = m_writtenFrames;
        }
        writer.write(grayMat(frame.image));
        if (sidecar.isActive())
        {
            ImageFrameSidecarRecord record;
            record.videoFrameIndex = (quint64)frameIndex;
            record.hostTimeMs = frame.hostTimeMs;
            record.bimgSequence = frame.bimgSequence;
            record.sourceFrameSequence = frame.sourceFrameSequence;
            record.captureTimeMs = frame.captureTimeMs;
            record.sourceFrameValid = frame.sourceFrameValid;
            record.captureTimeValid = frame.captureTimeValid;
            record.sourceCameraId = frame.sourceFrameCameraId;
            record.physicalBoardId = frame.physicalBoardId;
            record.attitudeValid = frame.attitudeValid &&
                                   std::isfinite(frame.rollDeg) &&
                                   std::isfinite(frame.pitchDeg) &&
                                   frame.rollDeg >= -180.0 && frame.rollDeg <= 180.0 &&
                                   frame.pitchDeg >= -180.0 && frame.pitchDeg <= 180.0;
            record.heightValid = frame.heightValid &&
                                 std::isfinite(frame.heightMm) &&
                                 frame.heightMm >= 0.0 && frame.heightMm <= 100000.0;
            if (record.attitudeValid)
            {
                record.rollDeg = (float)frame.rollDeg;
                record.pitchDeg = (float)frame.pitchDeg;
            }
            if (record.heightValid)
            {
                record.heightMm = (float)frame.heightMm;
            }
            if (!sidecar.append(record, &error))
            {
                writer.release();
                csvFile.close();
                (void)sidecar.finish();
                QMutexLocker locker(&m_mutex);
                m_accepting = false;
                emit recordingFailed(
                    QStringLiteral("标定AVI第 %1 帧已写入，但逐帧侧车写入失败：%2")
                        .arg(frameIndex)
                        .arg(error));
                return;
            }
        }
        {
            QMutexLocker locker(&m_mutex);
            ++m_writtenFrames;
        }
        csv << frameIndex << ','
            << frame.bimgSequence << ','
            << frame.hostTimeMs << ','
            << (int)frame.cameraId << ','
            << (int)frame.sourceCameraId << ','
            << csvNumber(frame.rollDeg) << ','
            << csvNumber(frame.pitchDeg) << ','
            << csvNumber(frame.heightMm) << ','
            << (frame.attitudeValid ? 1 : 0) << ','
            << (frame.heightValid ? 1 : 0) << '\n';
        csv.flush();
    }

    writer.release();
    csvFile.close();
    if (sidecar.isActive() && !sidecar.finish(&error))
    {
        QMutexLocker locker(&m_mutex);
        m_accepting = false;
        emit recordingFailed(
            QStringLiteral("结束标定录像逐帧侧车写入失败：%1").arg(error));
        return;
    }
    {
        QMutexLocker locker(&m_mutex);
        session.frameCount = m_writtenFrames;
        session.sourceDroppedFrames = m_sourceDroppedFrames;
        session.queueDroppedFrames = m_queueDroppedFrames;
        session.status = QStringLiteral("complete");
    }
    if (!HorizonCalibration::saveSession(session, &error))
    {
        emit recordingFailed(QStringLiteral("标定文件已写入，但 HCAL JSON 保存失败：%1").arg(error));
        return;
    }
    emit recordingFinished(session.sessionPath,
                           session.frameCount,
                           session.sourceDroppedFrames + session.queueDroppedFrames);
}
