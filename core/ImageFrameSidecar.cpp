#include "ImageFrameSidecar.h"

#include <QLocale>
#include <QStringList>

#include <cmath>
#include <limits>
#include <utility>

namespace
{
constexpr quint64 Uint32Max = std::numeric_limits<quint32>::max();
constexpr quint64 Uint8Max = std::numeric_limits<quint8>::max();
constexpr float MinimumAttitudeDeg = -180.0f;
constexpr float MaximumAttitudeDeg = 180.0f;
constexpr float MinimumHeightMm = 0.0f;
constexpr float MaximumHeightMm = 100000.0f;

const QStringList& headerFields()
{
    static const QStringList fields = {
        QStringLiteral("video_frame_index"),
        QStringLiteral("host_time_ms"),
        QStringLiteral("bimg_sequence"),
        QStringLiteral("source_frame_sequence"),
        QStringLiteral("capture_time_ms"),
        QStringLiteral("source_frame_valid"),
        QStringLiteral("capture_time_valid"),
        QStringLiteral("source_camera_id"),
        QStringLiteral("physical_board_id"),
        QStringLiteral("roll_deg"),
        QStringLiteral("pitch_deg"),
        QStringLiteral("height_mm"),
        QStringLiteral("attitude_valid"),
        QStringLiteral("height_valid")
    };
    return fields;
}

void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage != nullptr)
    {
        *errorMessage = message;
    }
}

bool writeAll(QFile* file, const QByteArray& data)
{
    qint64 offset = 0;
    while (offset < data.size())
    {
        const qint64 written = file->write(data.constData() + offset, data.size() - offset);
        if (written <= 0)
        {
            return false;
        }
        offset += written;
    }
    return true;
}

bool isSourceCameraIdValid(quint8 cameraId)
{
    return cameraId == 0U || cameraId == 2U || cameraId == 0xffU;
}

bool isPhysicalBoardIdValid(quint8 boardId)
{
    return boardId <= 1U || boardId == 0xffU;
}

bool validateRecord(const ImageFrameSidecarRecord& record, QString* errorMessage)
{
    if (record.hostTimeMs < 0)
    {
        setError(errorMessage, QStringLiteral("host_time_ms must be non-negative"));
        return false;
    }
    if (!isSourceCameraIdValid(record.sourceCameraId))
    {
        setError(errorMessage,
                 QStringLiteral("source_camera_id must be 0, 2, or 255"));
        return false;
    }
    if (!isPhysicalBoardIdValid(record.physicalBoardId))
    {
        setError(errorMessage,
                 QStringLiteral("physical_board_id must be 0, 1, or 255"));
        return false;
    }
    if (record.sourceFrameValid)
    {
        const quint8 expectedBoardId = record.sourceCameraId == 0U ? 0U : 1U;
        if (record.sourceCameraId == 0xffU || record.physicalBoardId != expectedBoardId)
        {
            setError(errorMessage,
                     QStringLiteral("valid source frame must map camera 0 to board 0 or camera 2 to board 1"));
            return false;
        }
    }
    else if (record.sourceCameraId != 0xffU && record.physicalBoardId != 0xffU)
    {
        const quint8 expectedBoardId = record.sourceCameraId == 0U ? 0U : 1U;
        if (record.physicalBoardId != expectedBoardId)
        {
            setError(errorMessage,
                     QStringLiteral("source camera and physical board do not match"));
            return false;
        }
    }
    if (!std::isfinite(record.rollDeg) ||
        record.rollDeg < MinimumAttitudeDeg || record.rollDeg > MaximumAttitudeDeg)
    {
        setError(errorMessage,
                 QStringLiteral("roll_deg must be finite and within [-180, 180]"));
        return false;
    }
    if (!std::isfinite(record.pitchDeg) ||
        record.pitchDeg < MinimumAttitudeDeg || record.pitchDeg > MaximumAttitudeDeg)
    {
        setError(errorMessage,
                 QStringLiteral("pitch_deg must be finite and within [-180, 180]"));
        return false;
    }
    if (!std::isfinite(record.heightMm) ||
        record.heightMm < MinimumHeightMm || record.heightMm > MaximumHeightMm)
    {
        setError(errorMessage,
                 QStringLiteral("height_mm must be finite and within [0, 100000]"));
        return false;
    }
    return true;
}

QString formatFloat(float value)
{
    return QLocale::c().toString(value, 'g', std::numeric_limits<float>::max_digits10);
}

QByteArray recordLine(const ImageFrameSidecarRecord& record)
{
    QStringList fields;
    fields.reserve(headerFields().size());
    fields.push_back(QString::number(record.videoFrameIndex));
    fields.push_back(QString::number(record.hostTimeMs));
    fields.push_back(QString::number(record.bimgSequence));
    fields.push_back(QString::number(record.sourceFrameSequence));
    fields.push_back(QString::number(record.captureTimeMs));
    fields.push_back(record.sourceFrameValid ? QStringLiteral("1") : QStringLiteral("0"));
    fields.push_back(record.captureTimeValid ? QStringLiteral("1") : QStringLiteral("0"));
    fields.push_back(QString::number(record.sourceCameraId));
    fields.push_back(QString::number(record.physicalBoardId));
    fields.push_back(formatFloat(record.rollDeg));
    fields.push_back(formatFloat(record.pitchDeg));
    fields.push_back(formatFloat(record.heightMm));
    fields.push_back(record.attitudeValid ? QStringLiteral("1") : QStringLiteral("0"));
    fields.push_back(record.heightValid ? QStringLiteral("1") : QStringLiteral("0"));
    return fields.join(QLatin1Char(',')).toUtf8() + '\n';
}

bool parseUnsigned(const QString& text,
                   const QString& fieldName,
                   quint64 maximum,
                   quint64* output,
                   QString* errorMessage)
{
    bool ok = false;
    const quint64 value = text.toULongLong(&ok, 10);
    if (!ok || value > maximum)
    {
        setError(errorMessage,
                 QStringLiteral("%1 must be an integer within [0, %2]")
                     .arg(fieldName)
                     .arg(maximum));
        return false;
    }
    *output = value;
    return true;
}

bool parseBool(const QString& text,
               const QString& fieldName,
               bool* output,
               QString* errorMessage)
{
    if (text == QLatin1String("0"))
    {
        *output = false;
        return true;
    }
    if (text == QLatin1String("1"))
    {
        *output = true;
        return true;
    }
    setError(errorMessage, QStringLiteral("%1 must be 0 or 1").arg(fieldName));
    return false;
}

bool parseFloat(const QString& text,
                const QString& fieldName,
                float* output,
                QString* errorMessage)
{
    bool ok = false;
    const double value = QLocale::c().toDouble(text, &ok);
    if (!ok || !std::isfinite(value) ||
        value < -std::numeric_limits<float>::max() ||
        value > std::numeric_limits<float>::max())
    {
        setError(errorMessage, QStringLiteral("%1 must be a finite float").arg(fieldName));
        return false;
    }
    *output = static_cast<float>(value);
    return true;
}

bool parseRecord(const QStringList& fields,
                 ImageFrameSidecarRecord* output,
                 QString* errorMessage)
{
    quint64 videoFrameIndex = 0;
    quint64 hostTimeMs = 0;
    quint64 bimgSequence = 0;
    quint64 sourceFrameSequence = 0;
    quint64 captureTimeMs = 0;
    quint64 sourceCameraId = 0;
    quint64 physicalBoardId = 0;

    if (!parseUnsigned(fields[0], headerFields()[0],
                       std::numeric_limits<quint64>::max(),
                       &videoFrameIndex, errorMessage) ||
        !parseUnsigned(fields[1], headerFields()[1],
                       static_cast<quint64>(std::numeric_limits<qint64>::max()),
                       &hostTimeMs, errorMessage) ||
        !parseUnsigned(fields[2], headerFields()[2], Uint32Max,
                       &bimgSequence, errorMessage) ||
        !parseUnsigned(fields[3], headerFields()[3], Uint32Max,
                       &sourceFrameSequence, errorMessage) ||
        !parseUnsigned(fields[4], headerFields()[4], Uint32Max,
                       &captureTimeMs, errorMessage) ||
        !parseBool(fields[5], headerFields()[5], &output->sourceFrameValid, errorMessage) ||
        !parseBool(fields[6], headerFields()[6], &output->captureTimeValid, errorMessage) ||
        !parseUnsigned(fields[7], headerFields()[7], Uint8Max,
                       &sourceCameraId, errorMessage) ||
        !parseUnsigned(fields[8], headerFields()[8], Uint8Max,
                       &physicalBoardId, errorMessage) ||
        !parseFloat(fields[9], headerFields()[9], &output->rollDeg, errorMessage) ||
        !parseFloat(fields[10], headerFields()[10], &output->pitchDeg, errorMessage) ||
        !parseFloat(fields[11], headerFields()[11], &output->heightMm, errorMessage) ||
        !parseBool(fields[12], headerFields()[12], &output->attitudeValid, errorMessage) ||
        !parseBool(fields[13], headerFields()[13], &output->heightValid, errorMessage))
    {
        return false;
    }

    output->videoFrameIndex = videoFrameIndex;
    output->hostTimeMs = static_cast<qint64>(hostTimeMs);
    output->bimgSequence = static_cast<quint32>(bimgSequence);
    output->sourceFrameSequence = static_cast<quint32>(sourceFrameSequence);
    output->captureTimeMs = static_cast<quint32>(captureTimeMs);
    output->sourceCameraId = static_cast<quint8>(sourceCameraId);
    output->physicalBoardId = static_cast<quint8>(physicalBoardId);
    return validateRecord(*output, errorMessage);
}
}

QString imageFrameSidecarPathForVideo(const QString& videoPath)
{
    return videoPath.isEmpty() ? QString() : videoPath + QStringLiteral(".frames.csv");
}

QString imageFrameSidecarCsvHeader()
{
    return headerFields().join(QLatin1Char(','));
}

bool loadImageFrameSidecar(const QString& sidecarPath,
                           QVector<ImageFrameSidecarRecord>* output,
                           QString* errorMessage)
{
    if (output == nullptr)
    {
        setError(errorMessage, QStringLiteral("output is null"));
        return false;
    }
    if (sidecarPath.trimmed().isEmpty())
    {
        setError(errorMessage, QStringLiteral("sidecar path is empty"));
        return false;
    }

    QFile file(sidecarPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    QByteArray bytes = file.readAll();
    if (file.error() != QFileDevice::NoError)
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    if (bytes.startsWith("\xef\xbb\xbf"))
    {
        bytes.remove(0, 3);
    }
    if (bytes.isEmpty())
    {
        setError(errorMessage, QStringLiteral("sidecar CSV is empty"));
        return false;
    }

    const QString text = QString::fromUtf8(bytes);
    if (text.toUtf8() != bytes)
    {
        setError(errorMessage, QStringLiteral("sidecar CSV is not valid UTF-8"));
        return false;
    }

    QStringList lines = text.split(QLatin1Char('\n'), Qt::KeepEmptyParts);
    if (!lines.isEmpty() && lines.back().isEmpty())
    {
        lines.removeLast();
    }
    for (QString& line : lines)
    {
        if (line.endsWith(QLatin1Char('\r')))
        {
            line.chop(1);
        }
    }
    if (lines.isEmpty() || lines.front() != imageFrameSidecarCsvHeader())
    {
        const QString actualHeader = lines.isEmpty()
            ? QStringLiteral("<missing>")
            : lines.front();
        setError(errorMessage,
                 QStringLiteral("sidecar CSV header mismatch: expected \"%1\", got \"%2\"")
                     .arg(imageFrameSidecarCsvHeader(), actualHeader));
        return false;
    }

    QVector<ImageFrameSidecarRecord> records;
    records.reserve(qMax(0, lines.size() - 1));
    quint64 expectedIndex = 0;
    for (int lineIndex = 1; lineIndex < lines.size(); ++lineIndex)
    {
        const int lineNumber = lineIndex + 1;
        if (lines[lineIndex].isEmpty())
        {
            setError(errorMessage,
                     QStringLiteral("line %1 is empty; a video frame row is missing")
                         .arg(lineNumber));
            return false;
        }
        const QStringList fields = lines[lineIndex].split(QLatin1Char(','), Qt::KeepEmptyParts);
        if (fields.size() != headerFields().size())
        {
            setError(errorMessage,
                     QStringLiteral("line %1 has %2 fields; expected %3")
                         .arg(lineNumber)
                         .arg(fields.size())
                         .arg(headerFields().size()));
            return false;
        }

        ImageFrameSidecarRecord record;
        QString recordError;
        if (!parseRecord(fields, &record, &recordError))
        {
            setError(errorMessage,
                     QStringLiteral("line %1: %2").arg(lineNumber).arg(recordError));
            return false;
        }
        if (record.videoFrameIndex < expectedIndex)
        {
            setError(errorMessage,
                     QStringLiteral("line %1: duplicate video_frame_index %2")
                         .arg(lineNumber)
                         .arg(record.videoFrameIndex));
            return false;
        }
        if (record.videoFrameIndex > expectedIndex)
        {
            setError(errorMessage,
                     QStringLiteral("line %1: missing video_frame_index %2 before %3")
                         .arg(lineNumber)
                         .arg(expectedIndex)
                         .arg(record.videoFrameIndex));
            return false;
        }

        records.push_back(record);
        ++expectedIndex;
    }

    *output = std::move(records);
    return true;
}

bool ImageFrameSidecarWriter::start(const QString& videoPath, QString* errorMessage)
{
    if (m_file.isOpen())
    {
        setError(errorMessage, QStringLiteral("sidecar recording is already active"));
        return false;
    }
    m_sidecarPath.clear();
    m_rowCount = 0;
    const QString path = imageFrameSidecarPathForVideo(videoPath);
    if (path.isEmpty())
    {
        setError(errorMessage, QStringLiteral("video path is empty"));
        return false;
    }

    m_file.setFileName(path);
    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        setError(errorMessage, m_file.errorString());
        return false;
    }
    if (!writeAll(&m_file, imageFrameSidecarCsvHeader().toUtf8() + '\n'))
    {
        const QString error = m_file.errorString();
        m_file.close();
        m_file.remove();
        setError(errorMessage, error);
        return false;
    }

    m_sidecarPath = path;
    return true;
}

bool ImageFrameSidecarWriter::append(const ImageFrameSidecarRecord& record,
                                     QString* errorMessage)
{
    if (!m_file.isOpen())
    {
        setError(errorMessage, QStringLiteral("sidecar recording is not active"));
        return false;
    }
    if (record.videoFrameIndex != m_rowCount)
    {
        setError(errorMessage,
                 QStringLiteral("video_frame_index must be %1, got %2")
                     .arg(m_rowCount)
                     .arg(record.videoFrameIndex));
        return false;
    }

    QString recordError;
    if (!validateRecord(record, &recordError))
    {
        setError(errorMessage, recordError);
        return false;
    }
    if (!writeAll(&m_file, recordLine(record)))
    {
        setError(errorMessage, m_file.errorString());
        return false;
    }
    ++m_rowCount;
    return true;
}

bool ImageFrameSidecarWriter::finish(QString* errorMessage)
{
    if (!m_file.isOpen())
    {
        setError(errorMessage, QStringLiteral("sidecar recording is not active"));
        return false;
    }

    const bool flushed = m_file.flush();
    const QString error = m_file.errorString();
    m_file.close();
    if (!flushed)
    {
        setError(errorMessage, error);
        return false;
    }
    return true;
}

bool ImageFrameSidecarWriter::isActive() const
{
    return m_file.isOpen();
}

QString ImageFrameSidecarWriter::sidecarPath() const
{
    return m_sidecarPath;
}

quint64 ImageFrameSidecarWriter::rowCount() const
{
    return m_rowCount;
}
