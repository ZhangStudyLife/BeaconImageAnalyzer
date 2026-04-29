#include "VideoReader.h"

#include <QFile>
#include <QFileInfo>

#include <opencv2/imgproc.hpp>

#include <cstring>
#include <limits>

namespace
{
constexpr qint64 MaxDibPixelCount = 8192LL * 8192LL;
constexpr qint64 MaxDibFrameBytes = 512LL * 1024LL * 1024LL;

struct DibAviChunks
{
    QByteArray avih;
    QByteArray strh;
    QByteArray strf;
    QVector<VideoFrameChunk> frames;
};

void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage != nullptr)
    {
        *errorMessage = message;
    }
}

bool readU16Le(const QByteArray& data, qsizetype offset, quint16* value)
{
    if (value == nullptr || offset < 0 || offset + 2 > data.size())
    {
        return false;
    }

    const auto b0 = (quint8)data.at(offset);
    const auto b1 = (quint8)data.at(offset + 1);
    *value = (quint16)b0 | ((quint16)b1 << 8);
    return true;
}

bool readU32Le(const QByteArray& data, qsizetype offset, quint32* value)
{
    if (value == nullptr || offset < 0 || offset + 4 > data.size())
    {
        return false;
    }

    const auto b0 = (quint8)data.at(offset);
    const auto b1 = (quint8)data.at(offset + 1);
    const auto b2 = (quint8)data.at(offset + 2);
    const auto b3 = (quint8)data.at(offset + 3);
    *value = (quint32)b0 |
             ((quint32)b1 << 8) |
             ((quint32)b2 << 16) |
             ((quint32)b3 << 24);
    return true;
}

bool readI32Le(const QByteArray& data, qsizetype offset, qint32* value)
{
    quint32 unsignedValue = 0;
    if (value == nullptr || !readU32Le(data, offset, &unsignedValue))
    {
        return false;
    }
    *value = (qint32)unsignedValue;
    return true;
}

bool readBytesAt(QFile* file, qint64 offset, qint64 size, QByteArray* output)
{
    if (file == nullptr || output == nullptr || offset < 0 || size < 0)
    {
        return false;
    }
    if (!file->seek(offset))
    {
        return false;
    }
    *output = file->read(size);
    return output->size() == size;
}

bool scanAviChunks(QFile* file,
                   qint64 start,
                   qint64 end,
                   bool insideMovi,
                   DibAviChunks* chunks)
{
    if (file == nullptr || chunks == nullptr || start < 0 || end < start)
    {
        return false;
    }

    qint64 position = start;
    while (position + 8 <= end)
    {
        QByteArray header;
        if (!readBytesAt(file, position, 8, &header))
        {
            return false;
        }

        const QByteArray chunkId = header.left(4);
        quint32 chunkSize = 0;
        if (!readU32Le(header, 4, &chunkSize))
        {
            return false;
        }

        const qint64 dataOffset = position + 8;
        const qint64 dataEnd = dataOffset + (qint64)chunkSize;
        if (dataEnd < dataOffset || dataEnd > end)
        {
            return false;
        }

        if (chunkId == QByteArrayLiteral("LIST") || chunkId == QByteArrayLiteral("RIFF"))
        {
            QByteArray listType;
            if (chunkSize < 4 || !readBytesAt(file, dataOffset, 4, &listType))
            {
                return false;
            }
            const bool childInsideMovi = insideMovi || listType == QByteArrayLiteral("movi");
            if (!scanAviChunks(file, dataOffset + 4, dataEnd, childInsideMovi, chunks))
            {
                return false;
            }
        }
        else if (insideMovi && (chunkId == QByteArrayLiteral("00db") || chunkId == QByteArrayLiteral("00dc")))
        {
            VideoFrameChunk frame;
            frame.dataOffset = dataOffset;
            frame.size = chunkSize;
            chunks->frames.push_back(frame);
        }
        else if (!insideMovi && chunkId == QByteArrayLiteral("avih"))
        {
            if (!readBytesAt(file, dataOffset, qMin<qint64>(chunkSize, 256), &chunks->avih))
            {
                return false;
            }
        }
        else if (!insideMovi && chunkId == QByteArrayLiteral("strh"))
        {
            if (!readBytesAt(file, dataOffset, qMin<qint64>(chunkSize, 256), &chunks->strh))
            {
                return false;
            }
        }
        else if (!insideMovi && chunkId == QByteArrayLiteral("strf"))
        {
            if (!readBytesAt(file, dataOffset, qMin<qint64>(chunkSize, 256), &chunks->strf))
            {
                return false;
            }
        }

        const qint64 paddedSize = (qint64)chunkSize + (chunkSize & 1U);
        position = dataOffset + paddedSize;
        if (position < dataOffset)
        {
            return false;
        }
    }

    return true;
}
}

QImage VideoReader::matToGrayImage(const cv::Mat& frame)
{
    if (frame.empty())
    {
        return QImage();
    }

    cv::Mat gray;
    if (frame.channels() == 1)
    {
        gray = frame;
    }
    else if (frame.channels() == 3)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    else if (frame.channels() == 4)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    }
    else
    {
        return QImage();
    }

    if (gray.depth() != CV_8U)
    {
        cv::Mat normalized;
        gray.convertTo(normalized, CV_8U);
        gray = normalized;
    }

    return QImage(gray.data,
                  gray.cols,
                  gray.rows,
                  (int)gray.step,
                  QImage::Format_Grayscale8)
        .copy();
}

QString VideoReader::fourccToString(int fourcc)
{
    if (fourcc == 0)
    {
        return QStringLiteral("unknown");
    }

    QString result;
    for (int i = 0; i < 4; ++i)
    {
        const char ch = (char)((fourcc >> (8 * i)) & 0xff);
        result.append(ch >= 32 && ch <= 126 ? QChar(ch) : QChar('?'));
    }
    return result.trimmed();
}

bool VideoReader::isUncompressedDibAvi(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        return false;
    }

    const QByteArray data = file.read(65536);
    const qsizetype strf = data.indexOf(QByteArrayLiteral("strf"));
    if (data.size() < 256 ||
        data.mid(0, 4) != QByteArrayLiteral("RIFF") ||
        data.mid(8, 4) != QByteArrayLiteral("AVI ") ||
        strf < 0 ||
        strf + 32 > data.size())
    {
        return false;
    }

    quint16 bitCount = 0;
    quint32 compression = 0;
    if (!readU16Le(data, strf + 22, &bitCount) ||
        !readU32Le(data, strf + 24, &compression))
    {
        return false;
    }
    return (bitCount == 8 || bitCount == 24) && compression == 0;
}

bool VideoReader::openOpenCv(const QString& path, const QVector<int>& backends, QString* lastError)
{
    cv::Mat firstFrame;
    for (int backend : backends)
    {
        m_capture.release();
        if (!m_capture.open(path.toStdString(), backend))
        {
            setError(lastError, QStringLiteral("OpenCV backend %1 cannot open video").arg(backend));
            continue;
        }

        if (!m_capture.read(firstFrame) || firstFrame.empty())
        {
            setError(lastError, QStringLiteral("OpenCV backend %1 cannot read frame 0").arg(backend));
            m_capture.release();
            continue;
        }

        m_width = firstFrame.cols;
        m_height = firstFrame.rows;
        m_bitCount = firstFrame.channels() * (int)(firstFrame.elemSize1() * 8);
        m_frameCount = (int)m_capture.get(cv::CAP_PROP_FRAME_COUNT);
        m_videoFps = m_capture.get(cv::CAP_PROP_FPS);
        if (m_videoFps <= 0.0)
        {
            m_videoFps = 0.0;
        }
        if (m_frameCount <= 0)
        {
            m_frameCount = 1;
        }

        m_codecName = fourccToString((int)m_capture.get(cv::CAP_PROP_FOURCC));
        m_backendName = QString::fromStdString(m_capture.getBackendName());
        m_filePath = QFileInfo(path).absoluteFilePath();
        m_backendMode = BackendMode::OpenCv;
        m_capture.set(cv::CAP_PROP_POS_FRAMES, 0);
        return true;
    }

    return false;
}

bool VideoReader::openDibAvi(const QString& path, QString* errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }

    const qint64 fileSize = file.size();
    QByteArray riffHeader;
    if (fileSize < 12 ||
        !readBytesAt(&file, 0, 12, &riffHeader) ||
        riffHeader.mid(0, 4) != QByteArrayLiteral("RIFF") ||
        riffHeader.mid(8, 4) != QByteArrayLiteral("AVI "))
    {
        setError(errorMessage, QStringLiteral("Invalid RIFF AVI file"));
        return false;
    }

    quint32 riffPayloadSize = 0;
    if (!readU32Le(riffHeader, 4, &riffPayloadSize))
    {
        setError(errorMessage, QStringLiteral("Malformed RIFF AVI header"));
        return false;
    }
    const qint64 riffEnd = qMin(fileSize, 8 + (qint64)riffPayloadSize);
    if (riffEnd < 12)
    {
        setError(errorMessage, QStringLiteral("Malformed RIFF AVI size"));
        return false;
    }

    DibAviChunks chunks;
    if (!scanAviChunks(&file, 12, riffEnd, false, &chunks) ||
        chunks.avih.size() < 40 ||
        chunks.strh.size() < 8 ||
        chunks.strf.size() < 20)
    {
        setError(errorMessage, QStringLiteral("Incomplete or malformed DIB AVI header"));
        return false;
    }

    quint32 microSecondsPerFrame = 0;
    quint32 totalFramesRaw = 0;
    quint32 widthRaw = 0;
    quint32 heightRaw = 0;
    qint32 bitmapWidth = 0;
    qint32 bitmapHeight = 0;
    quint16 planes = 0;
    quint16 bitCount = 0;
    quint32 compression = 0;
    if (!readU32Le(chunks.avih, 0, &microSecondsPerFrame) ||
        !readU32Le(chunks.avih, 16, &totalFramesRaw) ||
        !readU32Le(chunks.avih, 32, &widthRaw) ||
        !readU32Le(chunks.avih, 36, &heightRaw) ||
        !readI32Le(chunks.strf, 4, &bitmapWidth) ||
        !readI32Le(chunks.strf, 8, &bitmapHeight) ||
        !readU16Le(chunks.strf, 12, &planes) ||
        !readU16Le(chunks.strf, 14, &bitCount) ||
        !readU32Le(chunks.strf, 16, &compression))
    {
        setError(errorMessage, QStringLiteral("Malformed DIB AVI metadata"));
        return false;
    }

    if (planes != 1 || (bitCount != 24 && bitCount != 8) || compression != 0)
    {
        setError(errorMessage, QStringLiteral("DIB AVI fallback only supports uncompressed 8-bit/24-bit frames"));
        return false;
    }

    const int width = (int)widthRaw;
    const int height = (int)heightRaw;
    m_width = bitmapWidth > 0 ? bitmapWidth : width;
    m_height = bitmapHeight < 0 ? -bitmapHeight : bitmapHeight > 0 ? bitmapHeight : height;
    if (m_width <= 0 ||
        m_height <= 0 ||
        (qint64)m_width * (qint64)m_height > MaxDibPixelCount)
    {
        setError(errorMessage, QStringLiteral("Invalid or unsupported DIB AVI frame size"));
        return false;
    }

    m_bottomUp = bitmapHeight > 0;
    m_bitCount = (int)bitCount;
    const qint64 stride = m_bitCount == 24
        ? (((qint64)m_width * 3 + 3) / 4) * 4
        : (((qint64)m_width + 3) / 4) * 4;
    if (stride <= 0 ||
        stride > std::numeric_limits<int>::max() ||
        stride * (qint64)m_height > MaxDibFrameBytes)
    {
        setError(errorMessage, QStringLiteral("DIB AVI frame payload is too large"));
        return false;
    }
    m_sourceStride = (int)stride;
    m_videoFps = microSecondsPerFrame > 0 ? 1000000.0 / (double)microSecondsPerFrame : 0.0;
    m_codecName = QString::fromLatin1(chunks.strh.mid(4, 4)).trimmed();
    if (m_codecName.isEmpty())
    {
        m_codecName = QStringLiteral("DIB");
    }
    m_filePath = QFileInfo(path).absoluteFilePath();
    m_backendName = QStringLiteral("Internal DIB AVI fallback");
    m_backendMode = BackendMode::DibAvi;
    m_dibChunks = chunks.frames;

    if (m_dibChunks.isEmpty())
    {
        setError(errorMessage, QStringLiteral("No video frames found in DIB AVI movi section"));
        return false;
    }

    const int detectedFrames = (int)qMin<qsizetype>(m_dibChunks.size(), std::numeric_limits<int>::max());
    const int totalFrames = totalFramesRaw > 0 && totalFramesRaw <= (quint32)std::numeric_limits<int>::max()
        ? (int)totalFramesRaw
        : detectedFrames;
    m_frameCount = totalFrames > 0 ? qMin(totalFrames, detectedFrames) : detectedFrames;
    return true;
}

bool VideoReader::open(const QString& path, QString* errorMessage)
{
    m_capture.release();
    m_filePath.clear();
    m_codecName.clear();
    m_backendName.clear();
    m_backendMode = BackendMode::None;
    m_width = 0;
    m_height = 0;
    m_frameCount = 0;
    m_videoFps = 0.0;
    m_bitCount = 0;
    m_sourceStride = 0;
    m_bottomUp = true;
    m_dibChunks.clear();

    QString openCvError;
    if (isUncompressedDibAvi(path))
    {
        return openDibAvi(path, errorMessage);
    }

    const QVector<int> openCvBackends = {
        cv::CAP_FFMPEG,
        cv::CAP_MSMF,
        cv::CAP_DSHOW,
        cv::CAP_GSTREAMER,
        cv::CAP_ANY
    };
    if (openOpenCv(path, openCvBackends, &openCvError))
    {
        return true;
    }

    if (openDibAvi(path, errorMessage))
    {
        return true;
    }

    setError(errorMessage, QStringLiteral("%1: %2").arg(openCvError, path));
    return false;
}

bool VideoReader::isOpen() const
{
    if (m_backendMode == BackendMode::OpenCv)
    {
        return !m_filePath.isEmpty() && m_capture.isOpened();
    }
    if (m_backendMode == BackendMode::DibAvi)
    {
        return !m_filePath.isEmpty() && !m_dibChunks.isEmpty();
    }
    return false;
}

bool VideoReader::readFrame(int frameIndex, QImage* grayImage, QString* errorMessage) const
{
    if (grayImage == nullptr)
    {
        setError(errorMessage, QStringLiteral("grayImage is null"));
        return false;
    }
    if (!isOpen() || frameIndex < 0 || frameIndex >= m_frameCount)
    {
        setError(errorMessage, QStringLiteral("Frame index is out of range or video is not open"));
        return false;
    }

    if (m_backendMode == BackendMode::DibAvi)
    {
        return readDibFrame(frameIndex, grayImage, errorMessage);
    }

    if (!m_capture.set(cv::CAP_PROP_POS_FRAMES, frameIndex))
    {
        setError(errorMessage, QStringLiteral("OpenCV seek failed at frame %1").arg(frameIndex));
        return false;
    }

    cv::Mat frame;
    if (!m_capture.read(frame) || frame.empty())
    {
        setError(errorMessage, QStringLiteral("OpenCV read failed at frame %1").arg(frameIndex));
        return false;
    }

    *grayImage = matToGrayImage(frame);
    if (grayImage->isNull())
    {
        setError(errorMessage, QStringLiteral("OpenCV frame format cannot convert to grayscale"));
        return false;
    }
    return true;
}

bool VideoReader::readDibFrame(int frameIndex, QImage* grayImage, QString* errorMessage) const
{
    QFile file(m_filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }

    const VideoFrameChunk& chunk = m_dibChunks[frameIndex];
    const qint64 expectedBytes = (qint64)m_sourceStride * (qint64)m_height;
    if ((qint64)chunk.size < expectedBytes)
    {
        setError(errorMessage, QStringLiteral("DIB frame payload is smaller than expected"));
        return false;
    }

    if (!file.seek(chunk.dataOffset))
    {
        setError(errorMessage, QStringLiteral("Failed to seek DIB frame payload"));
        return false;
    }

    const QByteArray raw = file.read(expectedBytes);
    if (raw.size() != expectedBytes)
    {
        setError(errorMessage, QStringLiteral("Failed to read DIB frame payload"));
        return false;
    }

    QImage output(m_width, m_height, QImage::Format_Grayscale8);
    for (int y = 0; y < m_height; ++y)
    {
        const int sourceY = m_bottomUp ? (m_height - 1 - y) : y;
        const auto* source = reinterpret_cast<const unsigned char*>(raw.constData() + sourceY * m_sourceStride);
        auto* destination = output.scanLine(y);

        if (m_bitCount == 24)
        {
            for (int x = 0; x < m_width; ++x)
            {
                const int b = source[x * 3 + 0];
                const int g = source[x * 3 + 1];
                const int r = source[x * 3 + 2];
                destination[x] = (unsigned char)((r * 30 + g * 59 + b * 11 + 50) / 100);
            }
        }
        else
        {
            memcpy(destination, source, (size_t)m_width);
        }
    }

    *grayImage = output;
    return true;
}

QString VideoReader::filePath() const
{
    return m_filePath;
}

QString VideoReader::codecName() const
{
    return m_codecName;
}

int VideoReader::width() const
{
    return m_width;
}

int VideoReader::height() const
{
    return m_height;
}

int VideoReader::frameCount() const
{
    return m_frameCount;
}

double VideoReader::videoFps() const
{
    return m_videoFps;
}

int VideoReader::bitCount() const
{
    return m_bitCount;
}

QString VideoReader::backendName() const
{
    return m_backendName;
}
