#include "BimgImageFrameParser.h"

#include <QtEndian>
#include <QtTest>

#include <cstring>

namespace
{
void appendU16Le(QByteArray* data, quint16 value)
{
    const quint16 little = qToLittleEndian(value);
    data->append(reinterpret_cast<const char*>(&little), sizeof(little));
}

void appendU32Le(QByteArray* data, quint32 value)
{
    const quint32 little = qToLittleEndian(value);
    data->append(reinterpret_cast<const char*>(&little), sizeof(little));
}

quint32 crc32(const QByteArray& data)
{
    quint32 crc = 0xffffffffU;
    for (char byte : data)
    {
        crc ^= (quint8)byte;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xedb88320U : crc >> 1U;
        }
    }
    return crc ^ 0xffffffffU;
}

QByteArray makeFrame(quint8 mode, quint8 camera, quint32 sequence, bool withMarkers)
{
    constexpr quint16 width = 4;
    constexpr quint16 height = 3;
    QByteArray markers;
    if (withMarkers)
    {
        markers.append(char(BimgMarkerType::Beacon));
        markers.append(char(0));
        appendU16Le(&markers, 1);
        appendU16Le(&markers, 2);
        markers.append(char(BimgMarkerType::CarLamp));
        markers.append(char(1));
        appendU16Le(&markers, 3);
        appendU16Le(&markers, 0);
    }

    QByteArray image;
    for (int value = 0; value < width * height; ++value)
    {
        image.append(char(value));
    }

    QByteArray packet("BIMG", 4);
    packet.append(char(1));
    packet.append(char(28));
    packet.append(char(mode));
    packet.append(char(camera));
    appendU16Le(&packet, width);
    appendU16Le(&packet, height);
    appendU32Le(&packet, sequence);
    appendU32Le(&packet, image.size());
    packet.append(char(markers.size() / 6));
    packet.append(char(6));
    appendU16Le(&packet, 0);
    appendU32Le(&packet, image.size() + markers.size());
    packet.append(image);
    packet.append(markers);
    appendU32Le(&packet, crc32(packet));
    return packet;
}

void appendDebugFloat(QByteArray* data, quint16 id, bool valid, float value)
{
    appendU16Le(data, id);
    data->append(char(valid ? 1 : 0));
    data->append(char(0));
    quint32 bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    appendU32Le(data, bits);
}

QByteArray makeV2Frame(bool attitudeValid, bool heightValid, bool withUnknown = false)
{
    constexpr quint16 width = 4;
    constexpr quint16 height = 3;
    QByteArray image;
    for (int value = 0; value < width * height; ++value)
    {
        image.append(char(value));
    }

    QByteArray debugFloats;
    appendDebugFloat(&debugFloats, BimgDebugRollDegId, attitudeValid, 12.5f);
    appendDebugFloat(&debugFloats, BimgDebugPitchDegId, attitudeValid, -3.25f);
    appendDebugFloat(&debugFloats, BimgDebugHeightMmId, heightValid, 1087.5f);
    if (withUnknown)
    {
        appendDebugFloat(&debugFloats, 0x1234U, true, 8.0f);
    }

    QByteArray packet("BIMG", 4);
    packet.append(char(2));
    packet.append(char(28));
    packet.append(char(0));
    packet.append(char(1));
    appendU16Le(&packet, width);
    appendU16Le(&packet, height);
    appendU32Le(&packet, 99U);
    appendU32Le(&packet, image.size());
    packet.append(char(0));
    packet.append(char(6));
    packet.append(char(debugFloats.size() / 8));
    packet.append(char(8));
    appendU32Le(&packet, image.size() + debugFloats.size());
    packet.append(image);
    packet.append(debugFloats);
    appendU32Le(&packet, crc32(packet));
    return packet;
}

QByteArray makeSeekfreeFrame(quint16 width = 4, quint16 height = 3)
{
    QByteArray packet;
    packet.append(char(0xaa));
    packet.append(char(0x02));
    packet.append(char(0x40));
    packet.append(char(8));
    appendU16Le(&packet, width);
    appendU16Le(&packet, height);
    for (int value = 0; value < width * height; ++value)
    {
        packet.append(char(value));
    }
    return packet;
}

QByteArray makeSeekfreeXBoundary()
{
    QByteArray packet;
    packet.append(char(0xaa));
    packet.append(char(0x03));
    packet.append(char(0x03));
    packet.append(char(8));
    appendU16Le(&packet, 2);
    packet.append(char(0x07));
    packet.append(char(0));
    packet.append(QByteArray::fromHex("010203040506"));
    return packet;
}

QByteArray makeParameterSnapshot(quint8 camera, quint32 revision, quint32 sequence)
{
    QByteArray entries;
    appendU16Le(&entries, 0x0141U);
    entries.append(char(1));
    entries.append(char(0));
    appendU32Le(&entries, 120U);
    appendU16Le(&entries, 0x011dU);
    entries.append(char(0));
    entries.append(char(0));
    float alpha = 0.65f;
    quint32 alphaBits = 0;
    std::memcpy(&alphaBits, &alpha, sizeof(alphaBits));
    appendU32Le(&entries, alphaBits);

    QByteArray packet("BPAR", 4);
    packet.append(char(1));
    packet.append(char(28));
    packet.append(char(camera));
    packet.append(char(8));
    appendU32Le(&packet, revision);
    appendU32Le(&packet, 0x20260720U);
    appendU16Le(&packet, 2);
    appendU16Le(&packet, 0);
    appendU32Le(&packet, entries.size());
    appendU32Le(&packet, sequence);
    packet.append(entries);
    appendU32Le(&packet, crc32(packet));
    return packet;
}
}

class BimgImageFrameParserTests : public QObject
{
    Q_OBJECT

private slots:
    void parsesFragmentedFrame();
    void parsesConcatenatedFrames();
    void recoversAfterGarbageAndBadCrc();
    void parsesFragmentedSeekfreeFrame();
    void parsesMixedProtocols();
    void skipsSeekfreeBoundaryPacket();
    void parsesFragmentedParameterSnapshot();
    void parsesMixedImageAndParameterPackets();
    void rejectsBadParameterSnapshotCrc();
    void parsesFragmentedV2DebugFloats();
    void preservesInvalidAndUnknownDebugFloats();
    void rejectsMalformedV2DebugRecordSize();
};

void BimgImageFrameParserTests::parsesFragmentedFrame()
{
    BimgImageFrameParser parser;
    const QByteArray packet = makeFrame(3, 1, 42, true);
    QVector<BimgImageFrame> frames;
    for (char byte : packet)
    {
        frames += parser.append(QByteArray(1, byte)).frames;
    }

    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].protocolVersion, quint8(1));
    QCOMPARE(frames[0].streamMode, quint8(3));
    QCOMPARE(frames[0].cameraId, quint8(1));
    QCOMPARE(frames[0].sequence, quint32(42));
    QCOMPARE(frames[0].image.size(), QSize(4, 3));
    QCOMPARE(frames[0].markers.size(), 2);
    QCOMPARE(frames[0].markers[0].type, BimgMarkerType::Beacon);
    QCOMPARE(frames[0].markers[0].x, quint16(1));
    QCOMPARE(frames[0].markers[0].y, quint16(2));
}

void BimgImageFrameParserTests::parsesConcatenatedFrames()
{
    BimgImageFrameParser parser;
    const QVector<BimgImageFrame> frames = parser.append(makeFrame(0, 0, 7, false)
                                                          + makeFrame(2, 1, 8, false)).frames;
    QCOMPARE(frames.size(), 2);
    QCOMPARE(frames[0].sequence, quint32(7));
    QCOMPARE(frames[1].sequence, quint32(8));
}

void BimgImageFrameParserTests::recoversAfterGarbageAndBadCrc()
{
    BimgImageFrameParser parser;
    QByteArray bad = makeFrame(3, 0, 9, true);
    bad[30] = char((quint8)bad.at(30) ^ 0x5aU);
    const QVector<BimgImageFrame> frames = parser.append(QByteArray("garbage")
                                                          + bad
                                                          + makeFrame(1, 0, 10, false)).frames;
    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].sequence, quint32(10));
    QVERIFY(parser.crcErrorCount() >= 1U);
    QVERIFY(parser.protocolErrorCount() >= 1U);
}

void BimgImageFrameParserTests::parsesFragmentedSeekfreeFrame()
{
    BimgImageFrameParser parser;
    QVector<BimgImageFrame> frames;
    for (char byte : makeSeekfreeFrame())
    {
        frames += parser.append(QByteArray(1, byte)).frames;
    }

    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].protocolVersion, quint8(0));
    QCOMPARE(frames[0].protocol, ImageFrameProtocol::SeekfreeAssistant);
    QCOMPARE(frames[0].sequence, quint32(0));
    QCOMPARE(frames[0].image.size(), QSize(4, 3));
    QCOMPARE(frames[0].image.constScanLine(2)[3], uchar(11));
}

void BimgImageFrameParserTests::parsesMixedProtocols()
{
    BimgImageFrameParser parser;
    const QVector<BimgImageFrame> frames = parser.append(
        makeSeekfreeFrame() + makeFrame(2, 1, 8, false) + makeSeekfreeFrame()).frames;

    QCOMPARE(frames.size(), 3);
    QCOMPARE(frames[0].protocol, ImageFrameProtocol::SeekfreeAssistant);
    QCOMPARE(frames[1].protocol, ImageFrameProtocol::Bimg);
    QCOMPARE(frames[1].sequence, quint32(8));
    QCOMPARE(frames[2].protocol, ImageFrameProtocol::SeekfreeAssistant);
    QCOMPARE(frames[2].sequence, quint32(1));
}

void BimgImageFrameParserTests::skipsSeekfreeBoundaryPacket()
{
    BimgImageFrameParser parser;
    const QVector<BimgImageFrame> frames = parser.append(
        makeSeekfreeFrame() + makeSeekfreeXBoundary() + makeSeekfreeFrame()).frames;

    QCOMPARE(frames.size(), 2);
    QCOMPARE(parser.protocolErrorCount(), quint64(0));
}

void BimgImageFrameParserTests::parsesFragmentedParameterSnapshot()
{
    BimgImageFrameParser parser;
    BimgParseBatch all;
    for (char byte : makeParameterSnapshot(1, 9, 42))
    {
        const BimgParseBatch batch = parser.append(QByteArray(1, byte));
        all.frames += batch.frames;
        all.parameterSnapshots += batch.parameterSnapshots;
    }

    QCOMPARE(all.frames.size(), 0);
    QCOMPARE(all.parameterSnapshots.size(), 1);
    const BimgParameterSnapshot& snapshot = all.parameterSnapshots[0];
    QCOMPARE(snapshot.cameraId, quint8(1));
    QCOMPARE(snapshot.revision, quint32(9));
    QCOMPARE(snapshot.sequence, quint32(42));
    QCOMPARE(snapshot.algorithmBuildId, quint32(0x20260720U));
    QCOMPARE(snapshot.values.size(), 2);
    QCOMPARE(snapshot.values[0].id, quint16(0x0141U));
    QCOMPARE(snapshot.values[0].valueBits, quint32(120U));
}

void BimgImageFrameParserTests::parsesMixedImageAndParameterPackets()
{
    BimgImageFrameParser parser;
    const BimgParseBatch batch = parser.append(makeParameterSnapshot(0, 3, 7)
                                                + makeFrame(0, 0, 8, false));
    QCOMPARE(batch.parameterSnapshots.size(), 1);
    QCOMPARE(batch.frames.size(), 1);
    QCOMPARE(batch.frames[0].sequence, quint32(8));
}

void BimgImageFrameParserTests::rejectsBadParameterSnapshotCrc()
{
    BimgImageFrameParser parser;
    QByteArray bad = makeParameterSnapshot(0, 1, 1);
    bad[32] = char((quint8)bad.at(32) ^ 0x5aU);
    const BimgParseBatch batch = parser.append(bad + makeFrame(0, 0, 2, false));
    QCOMPARE(batch.parameterSnapshots.size(), 0);
    QCOMPARE(batch.frames.size(), 1);
    QVERIFY(parser.crcErrorCount() >= 1U);
}

void BimgImageFrameParserTests::parsesFragmentedV2DebugFloats()
{
    BimgImageFrameParser parser;
    QVector<BimgImageFrame> frames;
    for (char byte : makeV2Frame(true, true))
    {
        frames += parser.append(QByteArray(1, byte)).frames;
    }

    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].protocolVersion, quint8(2));
    QCOMPARE(frames[0].cameraId, quint8(1));
    QCOMPARE(frames[0].debugFloats.size(), 3);
    QCOMPARE(frames[0].debugFloats[0].id, BimgDebugRollDegId);
    QVERIFY(frames[0].debugFloats[0].valid);
    QCOMPARE(frames[0].debugFloats[0].value, 12.5f);
    QCOMPARE(frames[0].debugFloats[1].id, BimgDebugPitchDegId);
    QCOMPARE(frames[0].debugFloats[1].value, -3.25f);
    QCOMPARE(frames[0].debugFloats[2].id, BimgDebugHeightMmId);
    QVERIFY(frames[0].debugFloats[2].valid);
    QCOMPARE(frames[0].debugFloats[2].value, 1087.5f);
}

void BimgImageFrameParserTests::preservesInvalidAndUnknownDebugFloats()
{
    BimgImageFrameParser parser;
    const QVector<BimgImageFrame> frames = parser.append(makeV2Frame(false, true, true)).frames;

    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].debugFloats.size(), 4);
    QVERIFY(!frames[0].debugFloats[0].valid);
    QVERIFY(!frames[0].debugFloats[1].valid);
    QVERIFY(frames[0].debugFloats[2].valid);
    QCOMPARE(frames[0].debugFloats[2].id, BimgDebugHeightMmId);
    QCOMPARE(frames[0].debugFloats[3].id, quint16(0x1234U));
    QCOMPARE(frames[0].debugFloats[3].value, 8.0f);
}

void BimgImageFrameParserTests::rejectsMalformedV2DebugRecordSize()
{
    BimgImageFrameParser parser;
    QByteArray malformed = makeV2Frame(true, true);
    malformed[23] = char(4);
    const QVector<BimgImageFrame> frames = parser.append(
        malformed + makeFrame(0, 0, 100U, false)).frames;

    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].sequence, quint32(100U));
    QVERIFY(parser.protocolErrorCount() >= 1U);
}

QTEST_MAIN(BimgImageFrameParserTests)

#include "BimgImageFrameParserTests.moc"
