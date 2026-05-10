#include "SeekfreeImageFrameParser.h"

#include <QtGlobal>

#include <cstring>

namespace
{
constexpr char SeekfreeSendHead = char(0xAA);
constexpr char CameraFunction = char(0x02);
constexpr char CameraDotFunction = char(0x03);
constexpr qsizetype CameraHeaderSize = 8;
constexpr qsizetype DotHeaderSize = 8;
constexpr qsizetype MaxBufferSize = 1024 * 1024;

quint16 readU16Le(const QByteArray& data, qsizetype offset)
{
    const auto b0 = (quint8)data.at(offset);
    const auto b1 = (quint8)data.at(offset + 1);
    return (quint16)b0 | ((quint16)b1 << 8);
}

qsizetype dotPayloadSize(quint8 dotType, quint16 dotNum)
{
    const int boundaryCount = dotType & 0x0f;
    const int coordinateAxes = (dotType >> 6) == 2 ? 2 : 1;
    const int bytesPerCoordinate = (dotType & (1 << 5)) ? 2 : 1;
    return (qsizetype)boundaryCount * coordinateAxes * dotNum * bytesPerCoordinate;
}
}

QVector<SeekfreeImageFrame> SeekfreeImageFrameParser::append(const QByteArray& data)
{
    if (!data.isEmpty())
    {
        m_buffer.append(data);
    }
    if (m_buffer.size() > MaxBufferSize)
    {
        const qsizetype keepSize = qMin<qsizetype>(m_buffer.size(), MaxBufferSize / 2);
        m_buffer = m_buffer.right(keepSize);
    }

    QVector<SeekfreeImageFrame> frames;
    while (m_buffer.size() >= CameraHeaderSize)
    {
        const qsizetype headIndex = m_buffer.indexOf(SeekfreeSendHead);
        if (headIndex < 0)
        {
            m_buffer.clear();
            break;
        }
        if (headIndex > 0)
        {
            m_buffer.remove(0, headIndex);
        }
        if (m_buffer.size() < CameraHeaderSize)
        {
            break;
        }

        const auto function = m_buffer.at(1);
        if (function == CameraFunction)
        {
            const quint8 cameraTypeField = (quint8)m_buffer.at(2);
            const quint8 cameraType = cameraTypeField >> 5;
            const bool hasImage = ((cameraTypeField >> 4) & 0x01) == 0;
            const quint16 width = readU16Le(m_buffer, 4);
            const quint16 height = readU16Le(m_buffer, 6);
            const qsizetype payloadSize = hasImage ? imagePayloadSize(cameraType, width, height) : 0;
            if (payloadSize < 0 || width == 0 || height == 0)
            {
                m_buffer.remove(0, 1);
                continue;
            }
            const qsizetype packetSize = CameraHeaderSize + payloadSize;
            if (m_buffer.size() < packetSize)
            {
                break;
            }

            if (hasImage && cameraType == 2)
            {
                const QByteArray payload = m_buffer.mid(CameraHeaderSize, payloadSize);
                QImage image = grayImageFromPayload(payload, width, height);
                if (!image.isNull())
                {
                    SeekfreeImageFrame frame;
                    frame.image = image;
                    frame.cameraType = cameraType;
                    frame.width = width;
                    frame.height = height;
                    frames.push_back(frame);
                }
            }
            m_buffer.remove(0, packetSize);
            continue;
        }

        if (function == CameraDotFunction)
        {
            const quint8 dotType = (quint8)m_buffer.at(2);
            const quint16 dotNum = readU16Le(m_buffer, 4);
            const qsizetype packetSize = DotHeaderSize + dotPayloadSize(dotType, dotNum);
            if (packetSize < DotHeaderSize)
            {
                m_buffer.remove(0, 1);
                continue;
            }
            if (m_buffer.size() < packetSize)
            {
                break;
            }
            m_buffer.remove(0, packetSize);
            continue;
        }

        m_buffer.remove(0, 1);
    }
    return frames;
}

void SeekfreeImageFrameParser::clear()
{
    m_buffer.clear();
}

qsizetype SeekfreeImageFrameParser::imagePayloadSize(quint8 cameraType, quint16 width, quint16 height)
{
    const qsizetype pixels = (qsizetype)width * (qsizetype)height;
    if (pixels <= 0)
    {
        return -1;
    }
    switch (cameraType)
    {
        case 1:
            return pixels / 8;
        case 2:
            return pixels;
        case 3:
            return pixels * 2;
        default:
            return -1;
    }
}

QImage SeekfreeImageFrameParser::grayImageFromPayload(const QByteArray& payload, quint16 width, quint16 height)
{
    QImage image(width, height, QImage::Format_Grayscale8);
    if (image.isNull())
    {
        return QImage();
    }

    const qsizetype rowBytes = width;
    if (payload.size() < rowBytes * height)
    {
        return QImage();
    }

    for (int y = 0; y < height; ++y)
    {
        memcpy(image.scanLine(y), payload.constData() + y * rowBytes, (size_t)rowBytes);
    }
    return image;
}
