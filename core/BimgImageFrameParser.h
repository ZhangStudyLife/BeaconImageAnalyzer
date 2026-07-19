#ifndef BIMG_IMAGE_FRAME_PARSER_H
#define BIMG_IMAGE_FRAME_PARSER_H

#include <QByteArray>
#include <QImage>
#include <QVector>

enum class BimgMarkerType : quint8
{
    Beacon = 1,
    CarLamp = 2
};

enum class ImageFrameProtocol : quint8
{
    Bimg,
    SeekfreeAssistant
};

struct BimgImageMarker
{
    BimgMarkerType type = BimgMarkerType::Beacon;
    quint8 index = 0;
    quint16 x = 0;
    quint16 y = 0;
};

struct BimgImageFrame
{
    QImage image;
    ImageFrameProtocol protocol = ImageFrameProtocol::Bimg;
    quint8 streamMode = 0;
    quint8 cameraId = 0;
    quint16 width = 0;
    quint16 height = 0;
    quint32 sequence = 0;
    QVector<BimgImageMarker> markers;
};

class BimgImageFrameParser
{
public:
    QVector<BimgImageFrame> append(const QByteArray& data);
    void clear();

    quint64 crcErrorCount() const;
    quint64 protocolErrorCount() const;

private:
    static QImage grayImageFromPayload(const QByteArray& payload, quint16 width, quint16 height);

    QByteArray m_buffer;
    quint64 m_crcErrorCount = 0;
    quint64 m_protocolErrorCount = 0;
    quint32 m_seekfreeSequence = 0;
};

#endif
