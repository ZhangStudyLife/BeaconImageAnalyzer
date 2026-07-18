#include "BimgImageFrameParser.h"

#include <QtEndian>
#include <QtTest>

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
}

class BimgImageFrameParserTests : public QObject
{
    Q_OBJECT

private slots:
    void parsesFragmentedFrame();
    void parsesConcatenatedFrames();
    void recoversAfterGarbageAndBadCrc();
};

void BimgImageFrameParserTests::parsesFragmentedFrame()
{
    BimgImageFrameParser parser;
    const QByteArray packet = makeFrame(3, 1, 42, true);
    QVector<BimgImageFrame> frames;
    for (char byte : packet)
    {
        frames += parser.append(QByteArray(1, byte));
    }

    QCOMPARE(frames.size(), 1);
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
                                                          + makeFrame(2, 1, 8, false));
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
                                                          + makeFrame(1, 0, 10, false));
    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].sequence, quint32(10));
    QVERIFY(parser.crcErrorCount() >= 1U);
    QVERIFY(parser.protocolErrorCount() >= 1U);
}

QTEST_MAIN(BimgImageFrameParserTests)

#include "BimgImageFrameParserTests.moc"
