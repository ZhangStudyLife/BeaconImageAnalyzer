#include "TelemetryProtocol.h"

#include <QFile>
#include <QTemporaryDir>
#include <QTest>
#include <QtEndian>

#include <cstring>

namespace
{
QByteArray binaryFrame(bool withTail)
{
    QByteArray data;
    data.resize(TelemetryProtocol::PayloadBytes);
    for (int index = 0; index < TelemetryProtocol::ChannelCount; ++index)
    {
        const float value = static_cast<float>(index);
        quint32 bits = 0;
        std::memcpy(&bits, &value, sizeof(value));
        qToLittleEndian(bits, reinterpret_cast<uchar*>(data.data() + index * sizeof(float)));
    }
    if (withTail)
    {
        data.append(QByteArray::fromHex("0000807f"));
    }
    return data;
}
}

class TelemetryProtocolTests : public QObject
{
    Q_OBJECT

private slots:
    void parsesExactPayload();
    void parsesPayloadWithTail();
    void rejectsInvalidPackets();
    void csvRoundTrip();
};

void TelemetryProtocolTests::parsesExactPayload()
{
    TelemetryFrame frame;
    QString error;
    QVERIFY2(TelemetryProtocol::parseDatagram(binaryFrame(false), &frame, &error), qPrintable(error));
    QCOMPARE(frame.timestampMs, 0.0f);
    QCOMPARE(frame.cameras[0].beacons[0].x, 1.0f);
    QCOMPARE(frame.cameras[2].beacons[1].area, 18.0f);
    QCOMPARE(frame.cameras[0].carLamp.cx, 19.0f);
    QCOMPARE(frame.cameras[2].carLamp.length, 33.0f);
    QCOMPARE(frame.pitch, 34.0f);
    QCOMPARE(frame.roll, 35.0f);
    QCOMPARE(frame.yaw, 36.0f);
    QCOMPARE(frame.syncTimestampMs, 37.0f);
    QCOMPARE(frame.reserved, 38.0f);
    QCOMPARE(frame.carForwardVelocity, 39.0f);
    QCOMPARE(frame.carYaw, 40.0f);
    QCOMPARE(frame.plannedForwardVelocity, 41.0f);
    QCOMPARE(frame.plannedStrafeVelocity, 42.0f);
}

void TelemetryProtocolTests::parsesPayloadWithTail()
{
    TelemetryFrame frame;
    QString error;
    QVERIFY2(TelemetryProtocol::parseDatagram(binaryFrame(true), &frame, &error), qPrintable(error));
    QCOMPARE(frame.channels[42], 42.0f);
}

void TelemetryProtocolTests::rejectsInvalidPackets()
{
    TelemetryFrame frame;
    QString error;
    QVERIFY(!TelemetryProtocol::parseDatagram(QByteArray(171, '\0'), &frame, &error));
    QVERIFY(!error.isEmpty());

    QByteArray invalidTail = binaryFrame(false);
    invalidTail.append(QByteArray::fromHex("01020304"));
    QVERIFY(!TelemetryProtocol::parseDatagram(invalidTail, &frame, &error));

    QByteArray text("0,1,2,3");
    QVERIFY(!TelemetryProtocol::parseDatagram(text, &frame, &error));
}

void TelemetryProtocolTests::csvRoundTrip()
{
    TelemetryFrame original;
    QString error;
    QVERIFY(TelemetryProtocol::parseDatagram(binaryFrame(false), &original, &error));

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("telemetry.csv"));
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Text));
    file.write(TelemetryProtocol::csvHeader().toUtf8());
    file.write("\n");
    file.write(TelemetryProtocol::csvRow(original).toUtf8());
    file.write("\n");
    file.close();

    QVector<TelemetryFrame> frames;
    QVERIFY2(TelemetryProtocol::loadCsv(path, &frames, &error), qPrintable(error));
    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames[0].channels, original.channels);
}

QTEST_MAIN(TelemetryProtocolTests)
#include "TelemetryProtocolTests.moc"
