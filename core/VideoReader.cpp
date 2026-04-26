#include "VideoReader.h"

#include <QFile>
#include <QFileInfo>

#include <opencv2/imgproc.hpp>

quint16 VideoReader::readU16(const QByteArray& data, qsizetype offset)
{
    const auto* p = reinterpret_cast<const unsigned char*>(data.constData() + offset);
    return (quint16)p[0] | ((quint16)p[1] << 8);
}

quint32 VideoReader::readU32(const QByteArray& data, qsizetype offset)
{
    const auto* p = reinterpret_cast<const unsigned char*>(data.constData() + offset);
    return (quint32)p[0] |
           ((quint32)p[1] << 8) |
           ((quint32)p[2] << 16) |
           ((quint32)p[3] << 24);
}

qint32 VideoReader::readI32(const QByteArray& data, qsizetype offset)
{
    return (qint32)readU32(data, offset);
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

    const QByteArray data = file.read(4096);
    const qsizetype strf = data.indexOf(QByteArrayLiteral("strf"));
    if (data.size() < 256 ||
        data.mid(0, 4) != QByteArrayLiteral("RIFF") ||
        data.mid(8, 4) != QByteArrayLiteral("AVI ") ||
        strf < 0 ||
        strf + 32 >= data.size())
    {
        return false;
    }

    const quint16 bitCount = readU16(data, strf + 22);
    const quint32 compression = readU32(data, strf + 24);
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
            if (lastError != nullptr)
            {
                *lastError = QStringLiteral("OpenCV 后端 %1 无法打开视频").arg(backend);
            }
            continue;
        }

        if (!m_capture.read(firstFrame) || firstFrame.empty())
        {
            if (lastError != nullptr)
            {
                *lastError = QStringLiteral("OpenCV 后端 %1 无法读取第 0 帧").arg(backend);
            }
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
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }

    const QByteArray data = file.readAll();
    if (data.size() < 256 ||
        data.mid(0, 4) != QByteArrayLiteral("RIFF") ||
        data.mid(8, 4) != QByteArrayLiteral("AVI "))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("不是标准 RIFF AVI 文件");
        }
        return false;
    }

    const qsizetype avih = data.indexOf(QByteArrayLiteral("avih"));
    const qsizetype strh = data.indexOf(QByteArrayLiteral("strh"));
    const qsizetype strf = data.indexOf(QByteArrayLiteral("strf"));
    const qsizetype movi = data.indexOf(QByteArrayLiteral("movi"));
    if (avih < 0 || strh < 0 || strf < 0 || movi < 0)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("AVI 头不完整，缺少 avih/strh/strf/movi");
        }
        return false;
    }

    const quint32 microSecondsPerFrame = readU32(data, avih + 8);
    const int totalFrames = (int)readU32(data, avih + 8 + 16);
    const int width = (int)readU32(data, avih + 8 + 32);
    const int height = (int)readU32(data, avih + 8 + 36);
    const QByteArray handler = data.mid(strh + 12, 4);
    const int bitmapWidth = readI32(data, strf + 12);
    const int bitmapHeight = readI32(data, strf + 16);
    const quint16 planes = readU16(data, strf + 20);
    const quint16 bitCount = readU16(data, strf + 22);
    const quint32 compression = readU32(data, strf + 24);

    if (planes != 1 || (bitCount != 24 && bitCount != 8) || compression != 0)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("内置兜底后端只支持未压缩 8-bit/24-bit DIB AVI");
        }
        return false;
    }

    m_width = bitmapWidth > 0 ? bitmapWidth : width;
    m_height = bitmapHeight < 0 ? -bitmapHeight : bitmapHeight > 0 ? bitmapHeight : height;
    m_bottomUp = bitmapHeight > 0;
    m_bitCount = (int)bitCount;
    m_sourceStride = m_bitCount == 24
        ? ((m_width * 3 + 3) / 4) * 4
        : ((m_width + 3) / 4) * 4;
    m_videoFps = microSecondsPerFrame > 0 ? 1000000.0 / (double)microSecondsPerFrame : 0.0;
    m_codecName = QString::fromLatin1(handler).trimmed();
    if (m_codecName.isEmpty())
    {
        m_codecName = QStringLiteral("DIB");
    }
    m_filePath = QFileInfo(path).absoluteFilePath();
    m_backendName = QStringLiteral("Internal DIB AVI fallback");
    m_backendMode = BackendMode::DibAvi;
    m_dibChunks.clear();

    qsizetype position = movi + 4;
    const qsizetype idx1 = data.indexOf(QByteArrayLiteral("idx1"), position);
    const qsizetype scanEnd = idx1 > position ? idx1 : data.size();
    while (position + 8 <= scanEnd)
    {
        const QByteArray chunkId = data.mid(position, 4);
        const quint32 chunkSize = readU32(data, position + 4);
        if (position + 8 + (qsizetype)chunkSize > data.size())
        {
            break;
        }

        if (chunkId == QByteArrayLiteral("00db") || chunkId == QByteArrayLiteral("00dc"))
        {
            VideoFrameChunk chunk;
            chunk.dataOffset = position + 8;
            chunk.size = chunkSize;
            m_dibChunks.push_back(chunk);
        }

        position += 8 + (qsizetype)chunkSize + (chunkSize & 1U);
    }

    if (m_dibChunks.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("没有在 movi 段找到视频帧");
        }
        return false;
    }

    m_frameCount = totalFrames > 0 ? qMin(totalFrames, m_dibChunks.size()) : m_dibChunks.size();
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

    if (errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("%1: %2").arg(openCvError, path);
    }
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
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("grayImage 为空");
        }
        return false;
    }
    if (!isOpen() || frameIndex < 0 || frameIndex >= m_frameCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("帧号越界或视频未打开");
        }
        return false;
    }

    if (m_backendMode == BackendMode::DibAvi)
    {
        return readDibFrame(frameIndex, grayImage, errorMessage);
    }

    if (!m_capture.set(cv::CAP_PROP_POS_FRAMES, frameIndex))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("OpenCV 跳转帧失败: %1").arg(frameIndex);
        }
        return false;
    }

    cv::Mat frame;
    if (!m_capture.read(frame) || frame.empty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("OpenCV 读取帧失败: %1").arg(frameIndex);
        }
        return false;
    }

    *grayImage = matToGrayImage(frame);
    if (grayImage->isNull())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("OpenCV 帧格式无法转换为灰度图");
        }
        return false;
    }
    return true;
}

bool VideoReader::readDibFrame(int frameIndex, QImage* grayImage, QString* errorMessage) const
{
    QFile file(m_filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }

    const VideoFrameChunk& chunk = m_dibChunks[frameIndex];
    const int expectedBytes = m_sourceStride * m_height;
    if ((int)chunk.size < expectedBytes)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("帧数据尺寸小于 DIB 期望尺寸");
        }
        return false;
    }

    if (!file.seek(chunk.dataOffset))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("读取帧时 seek 失败");
        }
        return false;
    }

    const QByteArray raw = file.read(expectedBytes);
    if (raw.size() != expectedBytes)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("读取帧数据失败");
        }
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
