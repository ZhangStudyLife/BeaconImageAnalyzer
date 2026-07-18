#include "BimgImageFrameParser.h"

#include <QtGlobal>

#include <cstring>

namespace
{
const QByteArray Magic("BIMG", 4);
constexpr quint8 ProtocolVersion = 1;
constexpr quint8 HeaderSize = 28;
constexpr quint8 MarkerSize = 6;
constexpr quint8 MaxStreamMode = 3;
constexpr quint8 MaxMarkerCount = 16;
constexpr qsizetype CrcSize = 4;
constexpr qsizetype MaxBufferSize = 2 * 1024 * 1024;
constexpr quint32 MaxImageSize = 1024 * 1024;

quint16 readU16Le(const QByteArray& data, qsizetype offset)
{
    return (quint16)(quint8)data.at(offset)
           | ((quint16)(quint8)data.at(offset + 1) << 8);
}

quint32 readU32Le(const QByteArray& data, qsizetype offset)
{
    return (quint32)(quint8)data.at(offset)
           | ((quint32)(quint8)data.at(offset + 1) << 8)
           | ((quint32)(quint8)data.at(offset + 2) << 16)
           | ((quint32)(quint8)data.at(offset + 3) << 24);
}

quint32 crc32(const char* data, qsizetype length)
{
    quint32 crc = 0xffffffffU;
    for (qsizetype pos = 0; pos < length; ++pos)
    {
        crc ^= (quint8)data[pos];
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xedb88320U : crc >> 1U;
        }
    }
    return crc ^ 0xffffffffU;
}
}

QVector<BimgImageFrame> BimgImageFrameParser::append(const QByteArray& data)
{
    if (!data.isEmpty())
    {
        m_buffer.append(data);
    }
    if (m_buffer.size() > MaxBufferSize)
    {
        const qsizetype magicIndex = m_buffer.lastIndexOf(Magic);
        m_buffer = magicIndex >= 0 ? m_buffer.mid(magicIndex) : QByteArray();
        ++m_protocolErrorCount;
    }

    QVector<BimgImageFrame> frames;
    while (m_buffer.size() >= Magic.size())
    {
        const qsizetype magicIndex = m_buffer.indexOf(Magic);
        if (magicIndex < 0)
        {
            m_buffer = m_buffer.right(qMin<qsizetype>(m_buffer.size(), Magic.size() - 1));
            break;
        }
        if (magicIndex > 0)
        {
            m_buffer.remove(0, magicIndex);
            ++m_protocolErrorCount;
        }
        if (m_buffer.size() < HeaderSize)
        {
            break;
        }

        const quint8 version = (quint8)m_buffer.at(4);
        const quint8 headerSize = (quint8)m_buffer.at(5);
        const quint8 streamMode = (quint8)m_buffer.at(6);
        const quint8 cameraId = (quint8)m_buffer.at(7);
        const quint16 width = readU16Le(m_buffer, 8);
        const quint16 height = readU16Le(m_buffer, 10);
        const quint32 sequence = readU32Le(m_buffer, 12);
        const quint32 imageSize = readU32Le(m_buffer, 16);
        const quint8 markerCount = (quint8)m_buffer.at(20);
        const quint8 markerSize = (quint8)m_buffer.at(21);
        const quint32 payloadSize = readU32Le(m_buffer, 24);
        const quint64 pixels = (quint64)width * (quint64)height;
        const quint64 expectedPayload = (quint64)imageSize
                                        + (quint64)markerCount * markerSize;

        if (version != ProtocolVersion || headerSize != HeaderSize
            || streamMode > MaxStreamMode || cameraId > 1U
            || width == 0U || height == 0U || pixels > MaxImageSize
            || imageSize != pixels || markerCount > MaxMarkerCount
            || markerSize != MarkerSize || payloadSize != expectedPayload)
        {
            m_buffer.remove(0, 1);
            ++m_protocolErrorCount;
            continue;
        }

        const qsizetype packetSize = (qsizetype)HeaderSize + (qsizetype)payloadSize + CrcSize;
        if (packetSize > MaxBufferSize)
        {
            m_buffer.remove(0, 1);
            ++m_protocolErrorCount;
            continue;
        }
        if (m_buffer.size() < packetSize)
        {
            break;
        }

        const quint32 expectedCrc = readU32Le(m_buffer, packetSize - CrcSize);
        const quint32 actualCrc = crc32(m_buffer.constData(), packetSize - CrcSize);
        if (expectedCrc != actualCrc)
        {
            m_buffer.remove(0, 1);
            ++m_crcErrorCount;
            continue;
        }

        const QByteArray imagePayload = m_buffer.mid(HeaderSize, imageSize);
        QImage image = grayImageFromPayload(imagePayload, width, height);
        if (image.isNull())
        {
            m_buffer.remove(0, packetSize);
            ++m_protocolErrorCount;
            continue;
        }

        BimgImageFrame frame;
        frame.image = image;
        frame.streamMode = streamMode;
        frame.cameraId = cameraId;
        frame.width = width;
        frame.height = height;
        frame.sequence = sequence;
        frame.markers.reserve(markerCount);
        qsizetype markerOffset = HeaderSize + imageSize;
        for (quint8 index = 0; index < markerCount; ++index)
        {
            const quint8 type = (quint8)m_buffer.at(markerOffset);
            const quint16 x = readU16Le(m_buffer, markerOffset + 2);
            const quint16 y = readU16Le(m_buffer, markerOffset + 4);
            if ((type == (quint8)BimgMarkerType::Beacon
                 || type == (quint8)BimgMarkerType::CarLamp)
                && x < width && y < height)
            {
                BimgImageMarker marker;
                marker.type = (BimgMarkerType)type;
                marker.index = (quint8)m_buffer.at(markerOffset + 1);
                marker.x = x;
                marker.y = y;
                frame.markers.push_back(marker);
            }
            else
            {
                ++m_protocolErrorCount;
            }
            markerOffset += MarkerSize;
        }
        frames.push_back(frame);
        m_buffer.remove(0, packetSize);
    }
    return frames;
}

void BimgImageFrameParser::clear()
{
    m_buffer.clear();
}

quint64 BimgImageFrameParser::crcErrorCount() const
{
    return m_crcErrorCount;
}

quint64 BimgImageFrameParser::protocolErrorCount() const
{
    return m_protocolErrorCount;
}

QImage BimgImageFrameParser::grayImageFromPayload(const QByteArray& payload,
                                                  quint16 width,
                                                  quint16 height)
{
    QImage image(width, height, QImage::Format_Grayscale8);
    if (image.isNull() || payload.size() < (qsizetype)width * height)
    {
        return QImage();
    }
    for (int y = 0; y < height; ++y)
    {
        std::memcpy(image.scanLine(y), payload.constData() + (qsizetype)y * width, width);
    }
    return image;
}
