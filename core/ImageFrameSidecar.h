#ifndef IMAGE_FRAME_SIDECAR_H
#define IMAGE_FRAME_SIDECAR_H

#include <QFile>
#include <QString>
#include <QVector>

struct ImageFrameSidecarRecord
{
    quint64 videoFrameIndex = 0;
    qint64 hostTimeMs = 0;
    quint32 bimgSequence = 0;
    quint32 sourceFrameSequence = 0;
    quint32 captureTimeMs = 0;
    bool sourceFrameValid = false;
    bool captureTimeValid = false;
    quint8 sourceCameraId = 0xffU;
    quint8 physicalBoardId = 0xffU;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float heightMm = 0.0f;
    bool attitudeValid = false;
    bool heightValid = false;
};

QString imageFrameSidecarPathForVideo(const QString& videoPath);
QString imageFrameSidecarCsvHeader();

bool loadImageFrameSidecar(const QString& sidecarPath,
                           QVector<ImageFrameSidecarRecord>* output,
                           QString* errorMessage = nullptr);

class ImageFrameSidecarWriter
{
public:
    bool start(const QString& videoPath, QString* errorMessage = nullptr);
    bool append(const ImageFrameSidecarRecord& record, QString* errorMessage = nullptr);
    bool finish(QString* errorMessage = nullptr);

    bool isActive() const;
    QString sidecarPath() const;
    quint64 rowCount() const;

private:
    QFile m_file;
    QString m_sidecarPath;
    quint64 m_rowCount = 0;
};

#endif
