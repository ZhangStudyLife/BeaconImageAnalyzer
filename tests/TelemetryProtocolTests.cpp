#include "TelemetryProtocol.h"
#include "CarPlan3Model.h"

#include <QFile>
#include <QTemporaryDir>
#include <QTest>
#include <QtEndian>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
using Values = std::array<float, TelemetryProtocol::ChannelCount>;

Values realisticValues(int selectedTarget = 1)
{
    Values values{};
    values.fill(-999.0f);
    values[0] = 123456.0f;

    const float rawCameras[30] = {
        -31.5f, 12.0f, 18.0f, -8.0f, 20.5f, 9.0f, 4.0f, 16.0f, 35.0f, 28.0f,
        10.0f, -5.0f, 24.0f, 30.0f, 7.0f, 12.0f, 2.0f, 3.0f, -12.0f, 34.0f,
        42.0f, -18.0f, 15.0f, -22.0f, 6.0f, 7.0f, -6.0f, 11.0f, 152.0f, 31.0f
    };
    std::copy(std::begin(rawCameras), std::end(rawCameras), values.begin() + 1);

    values[31] = -130.0f; values[32] = 52.0f; values[33] = 21.0f; values[34] = 1.0f;
    values[35] = 11.0f; values[36] = -4.0f; values[37] = 26.0f; values[38] = 3.0f;
    values[39] = 138.0f; values[40] = 64.0f; values[41] = 13.0f; values[42] = 6.0f;
    values[43] = 3.5f; values[44] = 8.0f; values[45] = 22.0f; values[46] = 38.0f;

    values[47] = -77.0f; values[48] = 44.0f; values[49] = 21.0f;
    values[50] = 9.0f; values[51] = -3.0f; values[52] = 26.0f;
    values[53] = 88.0f; values[54] = 55.0f; values[55] = 13.0f;
    values[56] = 2.0f; values[57] = 4.0f; values[58] = -18.0f; values[59] = 46.0f;

    values[60] = 91.5f;
    values[61] = 0.45f;
    values[62] = 1.25f;
    values[63] = -0.20f;
    values[64] = 1.60f;
    values[65] = 1260.0f;
    values[66] = -2.5f;
    values[67] = 4.25f;
    values[68] = 178.0f;
    values[69] = static_cast<float>(selectedTarget);
    values[70] = 1.0f;
    return values;
}

QByteArray binaryFrame(const Values& values, int channelCount, bool withTail)
{
    QByteArray data(channelCount * static_cast<int>(sizeof(float)), Qt::Uninitialized);
    for (int index = 0; index < channelCount; ++index)
    {
        quint32 bits = 0;
        std::memcpy(&bits, &values[index], sizeof(float));
        qToLittleEndian(bits, reinterpret_cast<uchar*>(data.data() + index * sizeof(float)));
    }
    if (withTail)
    {
        data.append(QByteArray::fromHex("0000807f"));
    }
    return data;
}

Values carPlan3Values()
{
    Values values{};
    values.fill(-999.0f);
    values[0] = 5000.0f;
    const Values legacy = realisticValues();
    std::copy(legacy.begin() + 1, legacy.begin() + 31, values.begin() + 1);
    values[31] = 1.0f; values[32] = 2.0f; values[33] = 18.0f; values[34] = 3.0f;
    values[35] = -0.5f; values[36] = 1.2f; values[37] = 12.0f; values[38] = 7.0f;
    values[47] = 0.1f; values[48] = -0.2f; values[49] = 35.0f; values[50] = 6.0f;
    values[51] = 1.0f;
    values[52] = 90.0f; values[53] = 0.2f; values[54] = 1.1f;
    values[55] = -0.3f; values[56] = 1.6f;
    values[57] = 1200.0f; values[58] = -2.0f; values[59] = 4.0f; values[60] = 178.0f;
    values[61] = 1.0f; values[62] = 1.0f; values[63] = 3.0f;
    return values;
}
}

class TelemetryProtocolTests : public QObject
{
    Q_OBJECT

private slots:
    void parsesPacketWithoutTail();
    void parsesPacketWithTail();
    void rejectsOldPacketsAndBadTail();
    void mapsSemanticFields();
    void selectedTargetValues();
    void invalidSentinels();
    void csvRoundTrip();
    void rejectsWrongCsvColumnCount();
    void parsesCarPlan3Packet();
    void centerAnchoredMergeAt40550();
};

void TelemetryProtocolTests::parsesPacketWithoutTail()
{
    TelemetryFrame frame;
    QString error;
    QVERIFY2(TelemetryProtocol::parseDatagram(binaryFrame(realisticValues(), TelemetryProtocol::LegacyChannelCount, false), &frame, &error),
             qPrintable(error));
    QCOMPARE(frame.channels[0], 123456.0f);
    QCOMPARE(frame.channels[69], 1.0f);
    QCOMPARE(frame.channels[70], 1.0f);
    QVERIFY(frame.markerActive);
}

void TelemetryProtocolTests::parsesPacketWithTail()
{
    TelemetryFrame frame;
    QString error;
    QVERIFY2(TelemetryProtocol::parseDatagram(binaryFrame(realisticValues(), TelemetryProtocol::LegacyChannelCount, true), &frame, &error),
             qPrintable(error));
    QCOMPARE(frame.timestampMs, 123456.0f);
}

void TelemetryProtocolTests::rejectsOldPacketsAndBadTail()
{
    TelemetryFrame frame;
    QString error;
    QVERIFY(!TelemetryProtocol::parseDatagram(QByteArray(172, '\0'), &frame, &error));
    QVERIFY(!TelemetryProtocol::parseDatagram(QByteArray(176, '\0'), &frame, &error));

    QByteArray invalidTail = binaryFrame(realisticValues(), TelemetryProtocol::LegacyChannelCount, false);
    invalidTail.append(QByteArray::fromHex("01020304"));
    QVERIFY(!TelemetryProtocol::parseDatagram(invalidTail, &frame, &error));
    QVERIFY(error.contains(QStringLiteral("尾标")));
}

void TelemetryProtocolTests::mapsSemanticFields()
{
    TelemetryFrame frame;
    QString error;
    QVERIFY(TelemetryProtocol::parseDatagram(binaryFrame(realisticValues(), TelemetryProtocol::LegacyChannelCount, false), &frame, &error));

    QCOMPARE(frame.cameras[0].beacons[0].x, -31.5f);
    QCOMPARE(frame.cameras[1].beacons[1].area, 12.0f);
    QCOMPARE(frame.cameras[2].carLamp.angle, 152.0f);
    QCOMPARE(frame.cameras[2].carLamp.length, 31.0f);

    QCOMPARE(frame.centerBeacons[0].x, -130.0f);
    QCOMPARE(frame.centerBeacons[1].cameraMask, 3);
    QCOMPARE(frame.centerBeacons[2].cameraMask, 6);
    QCOMPARE(frame.centerCarLamp.cx, 3.5f);
    QCOMPARE(frame.centerCarLamp.length, 38.0f);

    QCOMPARE(frame.modelBeacons[0].x, -77.0f);
    QCOMPARE(frame.modelBeacons[2].area, 13.0f);
    QCOMPARE(frame.modelCarLamp.angle, -18.0f);
    QCOMPARE(frame.modelCarLamp.length, 46.0f);

    QCOMPARE(frame.carYawDeg, 91.5f);
    QCOMPARE(frame.carActualVelocityX, 0.45f);
    QCOMPARE(frame.carActualVelocityY, 1.25f);
    QCOMPARE(frame.carTargetVelocityX, -0.20f);
    QCOMPARE(frame.carTargetVelocityY, 1.60f);
    QCOMPARE(frame.aircraftHeightMm, 1260.0f);
    QCOMPARE(frame.aircraftRollDeg, -2.5f);
    QCOMPARE(frame.aircraftPitchDeg, 4.25f);
    QCOMPARE(frame.aircraftYawDeg, 178.0f);
    QCOMPARE(frame.selectedTargetId, 1);
    QVERIFY(frame.markerActive);
}

void TelemetryProtocolTests::selectedTargetValues()
{
    for (int selected : {-1, 0, 1, 2})
    {
        TelemetryFrame frame;
        QString error;
        QVERIFY(TelemetryProtocol::parseDatagram(binaryFrame(realisticValues(selected), TelemetryProtocol::LegacyChannelCount, false),
                                                  &frame,
                                                  &error));
        QCOMPARE(frame.selectedTargetId, selected);
    }
}

void TelemetryProtocolTests::invalidSentinels()
{
    Values values = realisticValues();
    values[1] = -999.0f;
    values[34] = 0.0f;
    values[43] = -999.0f;
    values[47] = -999.0f;
    values[56] = -999.0f;
    values[11] = std::numeric_limits<float>::quiet_NaN();
    values[70] = 0.0f;

    TelemetryFrame frame;
    QString error;
    QVERIFY(TelemetryProtocol::parseDatagram(binaryFrame(values, TelemetryProtocol::LegacyChannelCount, false), &frame, &error));
    QVERIFY(!frame.cameras[0].beacons[0].valid);
    QVERIFY(!frame.centerBeacons[0].valid);
    QVERIFY(!frame.centerCarLamp.valid);
    QVERIFY(!frame.modelBeacons[0].valid);
    QVERIFY(!frame.modelCarLamp.valid);
    QVERIFY(!frame.cameras[1].beacons[0].valid);
    QVERIFY(!frame.markerActive);
}

void TelemetryProtocolTests::csvRoundTrip()
{
    Values values = realisticValues(2);
    values[1] = std::numeric_limits<float>::quiet_NaN();
    TelemetryFrame original;
    QString error;
    QVERIFY(TelemetryProtocol::parseDatagram(binaryFrame(values, TelemetryProtocol::LegacyChannelCount, false), &original, &error));

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("telemetry.csv"));
    QVERIFY2(TelemetryProtocol::saveCsv(path, {original}, &error), qPrintable(error));

    QVector<TelemetryFrame> frames;
    QVERIFY2(TelemetryProtocol::loadCsv(path, &frames, &error), qPrintable(error));
    QCOMPARE(frames.size(), 1);
    for (int index = 0; index < TelemetryProtocol::ChannelCount; ++index)
    {
        if (index != 1)
        {
            QCOMPARE(frames[0].channels[index], original.channels[index]);
        }
    }
    QVERIFY(std::isnan(frames[0].channels[1]));
    QVERIFY(!frames[0].cameras[0].beacons[0].valid);
    QVERIFY(!frames[0].carPlan3Direct);
}

void TelemetryProtocolTests::rejectsWrongCsvColumnCount()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("old.csv"));
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Text));
    file.write("I0,I1\n1,2\n");
    file.close();

    QVector<TelemetryFrame> frames;
    QString error;
    QVERIFY(!TelemetryProtocol::loadCsv(path, &frames, &error));
    QVERIFY(error.contains(QStringLiteral("64列或71列")));
}

void TelemetryProtocolTests::parsesCarPlan3Packet()
{
    TelemetryFrame frame;
    QString error;
    const Values values = carPlan3Values();
    QVERIFY2(TelemetryProtocol::parseDatagram(
                 binaryFrame(values, TelemetryProtocol::CarPlan3ChannelCount, true),
                 &frame, &error), qPrintable(error));
    QCOMPARE(frame.channelCount, TelemetryProtocol::CarPlan3ChannelCount);
    QVERIFY(frame.carPlan3Direct);
    QVERIFY(frame.carPlan3Valid);
    QCOMPARE(frame.globalBeacons[0].cameraMask, 3);
    QCOMPARE(frame.globalBeacons[1].cameraMask, 7);
    QCOMPARE(frame.globalCarLamp.cameraMask, 6);
    QCOMPARE(frame.selectedTargetId, 1);
}

void TelemetryProtocolTests::centerAnchoredMergeAt40550()
{
    Values values{};
    values.fill(-999.0f);
    values[0] = 40550.0f;
    const float raw[30] = {
        -34.9425049f,16.927494f,18.0f,-87.0f,45.0f,11.0f,-28.5042725f,58.7020836f,-62.589035f,11.3632021f,
        -38.834301f,-40.2640495f,18.0f,-81.4519882f,18.6661682f,9.0f,-29.9230728f,2.38461685f,-74.5100861f,17.5637608f,
        70.0f,1.0f,5.0f,-999.0f,-999.0f,0.0f,22.8333359f,46.9814835f,-80.0256958f,15.6805573f
    };
    std::copy(std::begin(raw), std::end(raw), values.begin() + 1);
    values[60] = -91.2687607f;
    values[61] = 0.0f; values[62] = 2.07099819f;
    values[63] = -1.49382651f; values[64] = 1.87842548f;
    values[65] = 1179.75647f; values[66] = -4.54139614f;
    values[67] = 24.1431255f; values[68] = -5.89954376f;

    TelemetryFrame frame;
    QString error;
    QVERIFY2(TelemetryProtocol::parseDatagram(
                 binaryFrame(values, TelemetryProtocol::LegacyChannelCount, false),
                 &frame, &error), qPrintable(error));
    CarPlan3Model model;
    model.process(&frame);

    QVector<int> masks;
    for (const GlobalBeaconSample& beacon : frame.globalBeacons)
    {
        if (beacon.valid) masks.push_back(beacon.cameraMask);
    }
    QCOMPARE(masks.size(), 2);
    QVERIFY(masks.contains(3));
    QVERIFY(masks.contains(7));
}

QTEST_MAIN(TelemetryProtocolTests)
#include "TelemetryProtocolTests.moc"
