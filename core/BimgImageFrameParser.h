#ifndef BIMG_IMAGE_FRAME_PARSER_H
#define BIMG_IMAGE_FRAME_PARSER_H

#include <QByteArray>
#include <QImage>
#include <QVector>

constexpr quint16 BimgDebugRollDegId = 0x0001U;
constexpr quint16 BimgDebugPitchDegId = 0x0002U;
constexpr quint16 BimgDebugHeightMmId = 0x0003U;

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

struct BimgDebugFloat
{
    quint16 id = 0;
    bool valid = false;
    float value = 0.0f;
};

struct BimgImageFrame
{
    QImage image;
    ImageFrameProtocol protocol = ImageFrameProtocol::Bimg;
    quint8 protocolVersion = 0;
    quint8 streamMode = 0;
    quint8 cameraId = 0;
    quint8 sourceCameraId = 0xffU;
    quint8 physicalBoardId = 0xffU;
    quint16 width = 0;
    quint16 height = 0;
    quint32 sequence = 0;
    quint32 sourceFrameSequence = 0;
    quint32 captureTimeMs = 0;
    bool sourceFrameValid = false;
    bool captureTimeValid = false;
    QVector<BimgImageMarker> markers;
    QVector<BimgDebugFloat> debugFloats;
};

struct BimgParameterValue
{
    quint16 id = 0;
    quint8 type = 0;
    quint8 status = 0;
    quint32 valueBits = 0;
};

struct BimgParameterSnapshot
{
    quint8 cameraId = 0;
    quint32 revision = 0;
    quint32 algorithmBuildId = 0;
    quint32 sequence = 0;
    QVector<BimgParameterValue> values;
};

struct BimgParseBatch
{
    QVector<BimgImageFrame> frames;
    QVector<BimgParameterSnapshot> parameterSnapshots;

    bool isEmpty() const
    {
        return frames.isEmpty() && parameterSnapshots.isEmpty();
    }
};

class BimgImageFrameParser
{
public:
    BimgParseBatch append(const QByteArray& data);
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
