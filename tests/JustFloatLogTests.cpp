#include "JustFloatCsvRecorder.h"
#include "JustFloatLog.h"
#include "VehicleMotionUtils.h"
#include "WaveformHistoryStore.h"
#include "WaveformViewport.h"

#include <QFile>
#include <QTemporaryDir>
#include <QTemporaryFile>
#include <QtEndian>
#include <QtTest>

#include <cmath>
#include <cstring>
#include <limits>

namespace
{
QString sequentialText(int first, int count)
{
    QStringList fields;
    fields.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        fields.push_back(QString::number(first + i));
    }
    return fields.join(QLatin1Char(','));
}

QByteArray sequentialBinary(int first, int count)
{
    QByteArray result;
    result.reserve(count * static_cast<int>(sizeof(float)));
    for (int i = 0; i < count; ++i)
    {
        const float value = static_cast<float>(first + i);
        quint32 bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        const quint32 littleEndian = qToLittleEndian(bits);
        result.append(reinterpret_cast<const char*>(&littleEndian), sizeof(littleEndian));
    }
    return result;
}

QByteArray vofaTail()
{
    return QByteArray::fromHex("0000807f");
}

bool writeTextFile(const QString& path, const QString& text)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        return false;
    }
    return file.write(text.toUtf8()) == text.toUtf8().size();
}

QString headerForCount(int count)
{
    QStringList fields;
    fields.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        fields.push_back(QStringLiteral("I%1").arg(i));
    }
    return fields.join(QLatin1Char(','));
}

JustFloatLogRow sampleRow(bool withMotion)
{
    JustFloatLogRow row;
    row.rowTime = 123.25;
    row.syncTimeMs = 456.75;
    row.pitch = 1.25f;
    row.roll = -2.5f;
    row.yaw = 3.75f;
    for (int camera = 0; camera < 3; ++camera)
    {
        for (int beacon = 0; beacon < 2; ++beacon)
        {
            const float base = static_cast<float>(camera * 20 + beacon * 3);
            row.cameras[camera].beacons[beacon].x = base + 1.0f;
            row.cameras[camera].beacons[beacon].y = base + 2.0f;
            row.cameras[camera].beacons[beacon].area = base + 3.0f;
            row.cameras[camera].beacons[beacon].valid = true;
        }
        const float base = static_cast<float>(100 + camera * 10);
        row.cameras[camera].carLamp.cx = base + 1.0f;
        row.cameras[camera].carLamp.cy = base + 2.0f;
        row.cameras[camera].carLamp.angle = base + 3.0f;
        row.cameras[camera].carLamp.width = base + 4.0f;
        row.cameras[camera].carLamp.length = base + 5.0f;
        row.cameras[camera].carLamp.valid = true;
    }
    row.hasMotionData = withMotion;
    row.actualVelocityX = 0.75f;
    row.actualVelocityY = -1.25f;
    row.vehicleYawDeg = 32.5f;
    row.targetForwardMps = 1.5f;
    row.targetStrafeMps = -0.5f;
    return row;
}
}

class JustFloatLogTests : public QObject
{
    Q_OBJECT

private slots:
    void channelCatalogAndValues();
    void motionCoordinateTransform();
    void parseTextDatagram_data();
    void parseTextDatagram();
    void parseBinaryDatagram_data();
    void parseBinaryDatagram();
    void preferTextForBinarySizedPayload();
    void rejectInvalidDatagrams();
    void loadLegacyAndCurrentCsv();
    void validateCsvHeadersAndLengths();
    void serializeAndReloadRows();
    void recorderStateAndRoundTrip();
    void loadExternalFlightLog();
    void waveformViewportNavigation();
    void waveformHistoryQueryAndCleanup();
    void waveformHistorySparseContinuityAndGaps();
};

void JustFloatLogTests::channelCatalogAndValues()
{
    const QVector<JustFloatChannelDescriptor>& descriptors = JustFloatLog::channelDescriptors();
    QCOMPARE(descriptors.size(), JustFloatLog::ChannelCount);
    for (int i = 0; i < descriptors.size(); ++i)
    {
        QCOMPARE(descriptors[i].index, i);
        QVERIFY(!descriptors[i].group.isEmpty());
        QVERIFY(!descriptors[i].name.isEmpty());
    }
    QCOMPARE(descriptors[38].unit, QStringLiteral("m/s"));
    QCOMPARE(descriptors[39].unit, QStringLiteral("m/s"));
    QCOMPARE(descriptors[40].unit, QStringLiteral("deg"));
    QCOMPARE(descriptors[41].unit, QStringLiteral("m/s"));
    QCOMPARE(descriptors[42].unit, QStringLiteral("m/s"));
    QCOMPARE(descriptors[43].name, QStringLiteral("有效"));
    QCOMPARE(descriptors[46].unit, QStringLiteral("deg"));

    JustFloatLogRow row = sampleRow(false);
    double value = 0.0;
    QVERIFY(JustFloatLog::channelValue(row, 0, &value));
    QCOMPARE(value, row.rowTime);
    QVERIFY(JustFloatLog::channelValue(row, 34, &value));
    QCOMPARE(value, static_cast<double>(row.pitch));
    QVERIFY(!JustFloatLog::channelValue(row, 38, &value));
    QVERIFY(!JustFloatLog::channelValue(row, -1, &value));
    QVERIFY(!JustFloatLog::channelValue(row, 0, nullptr));

    row.hasMotionData = true;
    QVERIFY(JustFloatLog::channelValue(row, 38, &value));
    QCOMPARE(value, static_cast<double>(row.actualVelocityX));
    QVERIFY(JustFloatLog::channelValue(row, 42, &value));
    QCOMPARE(value, static_cast<double>(row.targetStrafeMps));
    QVERIFY(!JustFloatLog::channelValue(row, 43, &value));
    row.hasFusedCarLampData = true;
    row.fusedCarLamp = {true, -43.168f, -62.867f, 3.967f};
    QVERIFY(JustFloatLog::channelValue(row, 43, &value));
    QCOMPARE(value, 1.0);
    QVERIFY(JustFloatLog::channelValue(row, 46, &value));
    QCOMPARE(value, static_cast<double>(row.fusedCarLamp.angle));
}

void JustFloatLogTests::motionCoordinateTransform()
{
    QPointF delta = VehicleMotionUtils::velocityToCenter(1.0f, 0.0f, 0.0f);
    QVERIFY(std::abs(delta.x() - 1.0) < 1e-9);
    QVERIFY(std::abs(delta.y()) < 1e-9);

    delta = VehicleMotionUtils::velocityToCenter(0.0f, 1.0f, 0.0f);
    QVERIFY(std::abs(delta.x()) < 1e-9);
    QVERIFY(std::abs(delta.y() + 1.0) < 1e-9);

    delta = VehicleMotionUtils::velocityToCenter(1.2f, 0.991361737f, 3.967f);
    QVERIFY(std::abs(delta.x() - 1.2657) < 0.0002);
    QVERIFY(std::abs(delta.y() + 0.9060) < 0.0002);

    const QPointF mapped = VehicleMotionUtils::mapPointToCenter(
        VehicleMotionUtils::FrontCameraIndex,
        -13.429616f,
        -13.9388828f);
    QVERIFY(std::abs(mapped.x() + 19.17437) < 0.0002);
    QVERIFY(std::abs(mapped.y() + 70.04069) < 0.0002);
}

void JustFloatLogTests::parseTextDatagram_data()
{
    QTest::addColumn<int>("count");
    QTest::addColumn<bool>("withTail");
    QTest::addColumn<bool>("hasMotion");
    QTest::addColumn<bool>("hasFused");
    QTest::addColumn<double>("expectedRowTime");
    QTest::addColumn<double>("expectedActualVelocityX");

    QTest::newRow("legacy-payload") << 37 << false << false << false << 500.0 << 0.0;
    QTest::newRow("legacy-full-tail") << 38 << true << false << false << 0.0 << 0.0;
    QTest::newRow("motion-payload-tail") << 42 << true << true << false << 500.0 << 38.0;
    QTest::newRow("motion-full") << 43 << false << true << false << 0.0 << 38.0;
    QTest::newRow("fused-payload") << 46 << false << true << true << 500.0 << 38.0;
    QTest::newRow("fused-full-tail") << 47 << true << true << true << 0.0 << 38.0;
}

void JustFloatLogTests::parseTextDatagram()
{
    QFETCH(int, count);
    QFETCH(bool, withTail);
    QFETCH(bool, hasMotion);
    QFETCH(bool, hasFused);
    QFETCH(double, expectedRowTime);
    QFETCH(double, expectedActualVelocityX);

    QByteArray datagram = sequentialText(count == 37 || count == 42 || count == 46 ? 1 : 0, count).toUtf8();
    if (withTail)
    {
        datagram += vofaTail();
    }

    JustFloatLogRow row;
    QString error;
    QVERIFY2(JustFloatLog::parseDatagram(datagram, 500, &row, &error), qPrintable(error));
    QCOMPARE(row.rowTime, expectedRowTime);
    QCOMPARE(row.syncTimeMs, 37.0);
    QCOMPARE(row.hasMotionData, hasMotion);
    QCOMPARE(row.hasFusedCarLampData, hasFused);
    if (hasMotion)
    {
        QCOMPARE(static_cast<double>(row.actualVelocityX), expectedActualVelocityX);
        QCOMPARE(static_cast<double>(row.actualVelocityY), 39.0);
        QCOMPARE(static_cast<double>(row.vehicleYawDeg), 40.0);
        QCOMPARE(static_cast<double>(row.targetForwardMps), 41.0);
        QCOMPARE(static_cast<double>(row.targetStrafeMps), 42.0);
    }
    if (hasFused)
    {
        QVERIFY(row.fusedCarLamp.valid);
        QCOMPARE(row.fusedCarLamp.cx, 44.0f);
        QCOMPARE(row.fusedCarLamp.cy, 45.0f);
        QCOMPARE(row.fusedCarLamp.angle, 46.0f);
    }
}

void JustFloatLogTests::parseBinaryDatagram_data()
{
    parseTextDatagram_data();
}

void JustFloatLogTests::parseBinaryDatagram()
{
    QFETCH(int, count);
    QFETCH(bool, withTail);
    QFETCH(bool, hasMotion);
    QFETCH(bool, hasFused);
    QFETCH(double, expectedRowTime);
    QFETCH(double, expectedActualVelocityX);

    QByteArray datagram = sequentialBinary(count == 37 || count == 42 || count == 46 ? 1 : 0, count);
    if (withTail)
    {
        datagram += vofaTail();
    }

    JustFloatLogRow row;
    QString error;
    QVERIFY2(JustFloatLog::parseDatagram(datagram, 500, &row, &error), qPrintable(error));
    QCOMPARE(row.rowTime, expectedRowTime);
    QCOMPARE(row.syncTimeMs, 37.0);
    QCOMPARE(row.hasMotionData, hasMotion);
    QCOMPARE(row.hasFusedCarLampData, hasFused);
    if (hasMotion)
    {
        QCOMPARE(static_cast<double>(row.actualVelocityX), expectedActualVelocityX);
        QCOMPARE(static_cast<double>(row.actualVelocityY), 39.0);
        QCOMPARE(static_cast<double>(row.vehicleYawDeg), 40.0);
        QCOMPARE(static_cast<double>(row.targetForwardMps), 41.0);
        QCOMPARE(static_cast<double>(row.targetStrafeMps), 42.0);
    }
    if (hasFused)
    {
        QVERIFY(row.fusedCarLamp.valid);
        QCOMPARE(row.fusedCarLamp.cx, 44.0f);
        QCOMPARE(row.fusedCarLamp.cy, 45.0f);
        QCOMPARE(row.fusedCarLamp.angle, 46.0f);
    }
}

void JustFloatLogTests::preferTextForBinarySizedPayload()
{
    QByteArray datagram = sequentialText(1, 37).toUtf8();
    QVERIFY(datagram.size() < 148);
    datagram.append(QByteArray(148 - datagram.size(), ' '));

    JustFloatLogRow row;
    QString error;
    QVERIFY2(JustFloatLog::parseDatagram(datagram, 77, &row, &error), qPrintable(error));
    QCOMPARE(row.rowTime, 77.0);
    QCOMPARE(row.syncTimeMs, 37.0);
    QVERIFY(!row.hasMotionData);
}

void JustFloatLogTests::rejectInvalidDatagrams()
{
    JustFloatLogRow row;
    QString error;
    QVERIFY(!JustFloatLog::parseDatagram(sequentialText(0, 36).toUtf8(), 1, &row, &error));
    QVERIFY(!error.isEmpty());
    QVERIFY(!JustFloatLog::parseDatagram(sequentialText(0, 40).toUtf8(), 1, &row, &error));
    QVERIFY(!JustFloatLog::parseDatagram(sequentialText(0, 41).toUtf8(), 1, &row, &error));

    QStringList fields = sequentialText(0, 43).split(QLatin1Char(','));
    fields[12] = QStringLiteral("not-a-number");
    QVERIFY(!JustFloatLog::parseDatagram(fields.join(QLatin1Char(',')).toUtf8(), 1, &row, &error));

    QVERIFY(!JustFloatLog::parseDatagram(QByteArray(151, '\0'), 1, &row, &error));
}

void JustFloatLogTests::loadLegacyAndCurrentCsv()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());

    const QString legacyPath = directory.filePath(QStringLiteral("legacy.csv"));
    QVERIFY(writeTextFile(legacyPath,
                          headerForCount(38) + QLatin1Char('\n') + sequentialText(0, 38) + QLatin1Char('\n')));
    JustFloatLog legacyLog;
    QString error;
    QVERIFY2(JustFloatLog::loadCsv(legacyPath, &legacyLog, &error), qPrintable(error));
    QCOMPARE(legacyLog.rowCount(), 1);
    QVERIFY(!legacyLog.rowAt(0).hasMotionData);

    const QString currentPath = directory.filePath(QStringLiteral("current.csv"));
    QVERIFY(writeTextFile(currentPath,
                          headerForCount(43) + QLatin1Char('\n') + sequentialText(0, 43) + QLatin1Char('\n')));
    JustFloatLog currentLog;
    QVERIFY2(JustFloatLog::loadCsv(currentPath, &currentLog, &error), qPrintable(error));
    QCOMPARE(currentLog.rowCount(), 1);
    QVERIFY(currentLog.rowAt(0).hasMotionData);
    QCOMPARE(currentLog.rowAt(0).actualVelocityX, 38.0f);
    QCOMPARE(currentLog.rowAt(0).actualVelocityY, 39.0f);
    QCOMPARE(currentLog.rowAt(0).vehicleYawDeg, 40.0f);
    QCOMPARE(currentLog.rowAt(0).targetForwardMps, 41.0f);
    QCOMPARE(currentLog.rowAt(0).targetStrafeMps, 42.0f);

    const QString fusedPath = directory.filePath(QStringLiteral("fused.csv"));
    QVERIFY(writeTextFile(fusedPath,
                          headerForCount(47) + QLatin1Char('\n') + sequentialText(0, 47) + QLatin1Char('\n')));
    JustFloatLog fusedLog;
    QVERIFY2(JustFloatLog::loadCsv(fusedPath, &fusedLog, &error), qPrintable(error));
    QVERIFY(fusedLog.rowAt(0).hasFusedCarLampData);
    QVERIFY(fusedLog.rowAt(0).fusedCarLamp.valid);
    QCOMPARE(fusedLog.rowAt(0).fusedCarLamp.cx, 44.0f);
    QCOMPARE(fusedLog.rowAt(0).fusedCarLamp.cy, 45.0f);
    QCOMPARE(fusedLog.rowAt(0).fusedCarLamp.angle, 46.0f);

    const QString headerlessPath = directory.filePath(QStringLiteral("headerless.csv"));
    QVERIFY(writeTextFile(headerlessPath,
                          sequentialText(0, 38) + QLatin1Char('\n') + sequentialText(0, 43) + QLatin1Char('\n')));
    JustFloatLog headerlessLog;
    QVERIFY2(JustFloatLog::loadCsv(headerlessPath, &headerlessLog, &error), qPrintable(error));
    QCOMPARE(headerlessLog.rowCount(), 2);
    QVERIFY(!headerlessLog.rowAt(0).hasMotionData);
    QVERIFY(headerlessLog.rowAt(1).hasMotionData);
}

void JustFloatLogTests::validateCsvHeadersAndLengths()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QString error;
    JustFloatLog log;

    QStringList missingRequired = headerForCount(38).split(QLatin1Char(','));
    missingRequired.removeAt(12);
    const QString missingRequiredPath = directory.filePath(QStringLiteral("missing-required.csv"));
    QVERIFY(writeTextFile(missingRequiredPath,
                          missingRequired.join(QLatin1Char(',')) + QLatin1Char('\n') +
                              sequentialText(0, 37) + QLatin1Char('\n')));
    QVERIFY(!JustFloatLog::loadCsv(missingRequiredPath, &log, &error));
    QVERIFY(error.contains(QStringLiteral("I12")));

    const QString oldProjectionPath = directory.filePath(QStringLiteral("old-projection.csv"));
    QVERIFY(writeTextFile(oldProjectionPath,
                          headerForCount(40) + QLatin1Char('\n') + sequentialText(0, 40) + QLatin1Char('\n')));
    QVERIFY(!JustFloatLog::loadCsv(oldProjectionPath, &log, &error));
    QVERIFY(error.contains(QStringLiteral("I38")));
    QVERIFY(error.contains(QStringLiteral("I42")));

    const QString wrongLengthPath = directory.filePath(QStringLiteral("wrong-length.csv"));
    QVERIFY(writeTextFile(wrongLengthPath, sequentialText(0, 42) + QLatin1Char('\n')));
    QVERIFY(!JustFloatLog::loadCsv(wrongLengthPath, &log, &error));

    QStringList partialMotion = sequentialText(0, 38).split(QLatin1Char(','));
    partialMotion << QStringLiteral("1.0")
                  << QStringLiteral("2.0")
                  << QString()
                  << QStringLiteral("4.0")
                  << QStringLiteral("5.0");
    const QString partialMotionPath = directory.filePath(QStringLiteral("partial-motion.csv"));
    QVERIFY(writeTextFile(partialMotionPath,
                          headerForCount(43) + QLatin1Char('\n') +
                              partialMotion.join(QLatin1Char(',')) + QLatin1Char('\n')));
    QVERIFY(!JustFloatLog::loadCsv(partialMotionPath, &log, &error));

    const QString partialFusedPath = directory.filePath(QStringLiteral("partial-fused.csv"));
    QVERIFY(writeTextFile(partialFusedPath,
                          headerForCount(46) + QLatin1Char('\n') +
                              sequentialText(0, 46) + QLatin1Char('\n')));
    QVERIFY(!JustFloatLog::loadCsv(partialFusedPath, &log, &error));
    QVERIFY(error.contains(QStringLiteral("I43")));
    QVERIFY(error.contains(QStringLiteral("I46")));
}

void JustFloatLogTests::serializeAndReloadRows()
{
    QCOMPARE(JustFloatLog::csvHeader().split(QLatin1Char(',')).size(), JustFloatLog::ChannelCount);

    const JustFloatLogRow legacyRow = sampleRow(false);
    const QString legacyCsvRow = JustFloatLog::csvRow(legacyRow);
    const QStringList legacyCells = legacyCsvRow.split(QLatin1Char(','), Qt::KeepEmptyParts);
    QCOMPARE(legacyCells.size(), JustFloatLog::ChannelCount);
    for (int i = 38; i <= 42; ++i)
    {
        QVERIFY(legacyCells[i].isEmpty());
    }

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("round-trip.csv"));
    JustFloatLogRow currentRow = sampleRow(true);
    currentRow.hasFusedCarLampData = true;
    currentRow.fusedCarLamp = {true, -43.168f, -62.867f, 3.967f};
    QVERIFY(writeTextFile(path,
                          JustFloatLog::csvHeader() + QLatin1Char('\n') +
                              legacyCsvRow + QLatin1Char('\n') +
                              JustFloatLog::csvRow(currentRow) + QLatin1Char('\n')));

    JustFloatLog log;
    QString error;
    QVERIFY2(JustFloatLog::loadCsv(path, &log, &error), qPrintable(error));
    QCOMPARE(log.rowCount(), 2);
    QVERIFY(!log.rowAt(0).hasMotionData);
    QVERIFY(log.rowAt(1).hasMotionData);
    QCOMPARE(log.rowAt(1).rowTime, currentRow.rowTime);
    QCOMPARE(log.rowAt(1).syncTimeMs, currentRow.syncTimeMs);
    QCOMPARE(log.rowAt(1).actualVelocityX, currentRow.actualVelocityX);
    QCOMPARE(log.rowAt(1).actualVelocityY, currentRow.actualVelocityY);
    QCOMPARE(log.rowAt(1).vehicleYawDeg, currentRow.vehicleYawDeg);
    QCOMPARE(log.rowAt(1).targetForwardMps, currentRow.targetForwardMps);
    QCOMPARE(log.rowAt(1).targetStrafeMps, currentRow.targetStrafeMps);
    QVERIFY(log.rowAt(1).hasFusedCarLampData);
    QCOMPARE(log.rowAt(1).fusedCarLamp.cx, currentRow.fusedCarLamp.cx);
    QCOMPARE(log.rowAt(1).fusedCarLamp.cy, currentRow.fusedCarLamp.cy);
    QCOMPARE(log.rowAt(1).fusedCarLamp.angle, currentRow.fusedCarLamp.angle);
}

void JustFloatLogTests::recorderStateAndRoundTrip()
{
    JustFloatCsvRecorder recorder;
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::Idle);
    QCOMPARE(recorder.rowCount(), quint64(0));

    QString error;
    QVERIFY2(recorder.start(&error), qPrintable(error));
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::Recording);
    QVERIFY(!recorder.start(&error));
    QVERIFY(recorder.append(sampleRow(false), &error));
    QVERIFY(recorder.append(sampleRow(true), &error));
    QCOMPARE(recorder.rowCount(), quint64(2));
    QVERIFY2(recorder.stop(&error), qPrintable(error));
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::PendingSave);

    QVERIFY(!recorder.saveAs(QString(), &error));
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::PendingSave);
    QCOMPARE(recorder.rowCount(), quint64(2));

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString invalidPath = directory.filePath(QStringLiteral("missing/recorded.csv"));
    QVERIFY(!recorder.saveAs(invalidPath, &error));
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::PendingSave);
    QCOMPARE(recorder.rowCount(), quint64(2));

    QVERIFY2(recorder.resume(&error), qPrintable(error));
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::Recording);
    QVERIFY(recorder.append(sampleRow(true), &error));
    QCOMPARE(recorder.rowCount(), quint64(3));
    QVERIFY2(recorder.stop(&error), qPrintable(error));

    const QString outputPath = directory.filePath(QStringLiteral("recorded.csv"));
    QVERIFY2(recorder.saveAs(outputPath, &error), qPrintable(error));
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::Idle);
    QCOMPARE(recorder.rowCount(), quint64(0));

    JustFloatLog log;
    QVERIFY2(JustFloatLog::loadCsv(outputPath, &log, &error), qPrintable(error));
    QCOMPARE(log.rowCount(), 3);
    QVERIFY(!log.rowAt(0).hasMotionData);
    QVERIFY(log.rowAt(1).hasMotionData);
    QVERIFY(log.rowAt(2).hasMotionData);

    QVERIFY(recorder.start(&error));
    QVERIFY(recorder.append(sampleRow(true), &error));
    recorder.discard();
    QCOMPARE(recorder.state(), JustFloatCsvRecorder::State::Idle);
    QCOMPARE(recorder.rowCount(), quint64(0));
    QVERIFY(!recorder.append(sampleRow(true), &error));
}

void JustFloatLogTests::loadExternalFlightLog()
{
    const QString path = QString::fromLocal8Bit(qgetenv("BEACON_TEST_FLIGHT_LOG"));
    if (path.isEmpty())
    {
        QSKIP("BEACON_TEST_FLIGHT_LOG is not set");
    }

    JustFloatLog log;
    QString error;
    QVERIFY2(JustFloatLog::loadCsv(path, &log, &error), qPrintable(error));
    QCOMPARE(log.rowCount(), 16234);
    const JustFloatLogRow& first = log.rowAt(0);
    QCOMPARE(first.rowTime, 128657.0);
    QVERIFY(first.hasMotionData);
    QCOMPARE(first.actualVelocityX, 0.0f);
    QCOMPARE(first.actualVelocityY, 0.0f);
    QCOMPARE(first.vehicleYawDeg, 0.0f);
    QCOMPARE(first.targetForwardMps, 0.0f);
    QCOMPARE(first.targetStrafeMps, 0.0f);
}

void JustFloatLogTests::waveformViewportNavigation()
{
    WaveformViewport viewport;
    viewport.reset();
    viewport.setBounds(0.0, 60000.0);
    viewport.setFollowTarget(60000.0);
    QCOMPARE(viewport.windowDurationMs(), WaveformViewport::DefaultWindowMs);
    QCOMPARE(viewport.endTimeMs(), 60000.0);
    QCOMPARE(viewport.startTimeMs(), 50000.0);
    QVERIFY(viewport.isFollowing());

    viewport.zoom(1.0, 0.5);
    QVERIFY(viewport.windowDurationMs() < WaveformViewport::DefaultWindowMs);
    QCOMPARE(viewport.endTimeMs(), 60000.0);

    const double durationBeforePan = viewport.windowDurationMs();
    const double startBeforePan = viewport.startTimeMs();
    viewport.panPixels(100.0, 1000.0);
    QVERIFY(!viewport.isFollowing());
    QVERIFY(std::abs(viewport.startTimeMs() -
                     (startBeforePan - durationBeforePan * 0.1)) < 1e-6);

    const double anchorBeforeZoom = viewport.startTimeMs() +
                                    viewport.windowDurationMs() * 0.25;
    viewport.zoom(1.0, 0.25);
    const double anchorAfterZoom = viewport.startTimeMs() +
                                   viewport.windowDurationMs() * 0.25;
    QVERIFY(std::abs(anchorAfterZoom - anchorBeforeZoom) < 1e-6);

    const double manualStart = viewport.startTimeMs();
    viewport.setBounds(0.0, 70000.0);
    viewport.setFollowTarget(70000.0);
    QCOMPARE(viewport.startTimeMs(), manualStart);

    viewport.followTarget();
    QVERIFY(viewport.isFollowing());
    QCOMPARE(viewport.endTimeMs(), 70000.0);

    viewport.setWindowDuration(1.0);
    QCOMPARE(viewport.windowDurationMs(), WaveformViewport::MinimumWindowMs);
    viewport.setWindowDuration(1000000.0);
    QCOMPARE(viewport.windowDurationMs(), 70000.0);

    viewport.reset();
    viewport.setBounds(0.0, 2000.0);
    viewport.setFollowTarget(2000.0);
    QCOMPARE(viewport.windowDurationMs(), 2000.0);
    QCOMPARE(viewport.startTimeMs(), 0.0);
    viewport.panPixels(100.0, 1000.0);
    QVERIFY(!viewport.isFollowing());
    QCOMPARE(viewport.startTimeMs(), 0.0);
    const double frozenStart = viewport.startTimeMs();
    viewport.setBounds(0.0, 3000.0);
    viewport.setFollowTarget(3000.0);
    QCOMPARE(viewport.startTimeMs(), frozenStart);

    viewport.followTarget();
    QCOMPARE(viewport.endTimeMs(), 3000.0);
    viewport.stopFollowing();
    const double stoppedStart = viewport.startTimeMs();
    viewport.setBounds(0.0, 4000.0);
    viewport.setFollowTarget(4000.0);
    QCOMPARE(viewport.startTimeMs(), stoppedStart);

    const double finiteStart = viewport.startTimeMs();
    viewport.zoom(1.0, std::numeric_limits<double>::quiet_NaN());
    QCOMPARE(viewport.startTimeMs(), finiteStart);
}

void JustFloatLogTests::waveformHistoryQueryAndCleanup()
{
    QString temporaryPath;
    {
        WaveformHistoryStore history;
        QString error;
        QVERIFY2(history.beginSession(&error), qPrintable(error));
        temporaryPath = history.temporaryPath();
        QVERIFY(QFile::exists(temporaryPath));

        for (int i = 0; i < 600; ++i)
        {
            JustFloatLogRow row = sampleRow(i != 300);
            row.rowTime = i * 10.0 + 1.0;
            row.syncTimeMs = i * 10.0 + 1.0;
            row.actualVelocityX = i == 301 ? 999.0f : static_cast<float>(i % 40);
            row.actualVelocityY = static_cast<float>(-i % 30);
            QVERIFY2(history.append(row, i * 10, &error), qPrintable(error));
        }

        QCOMPARE(history.sampleCount(), quint64(600));
        QVERIFY(history.lastTimeMs() > history.firstTimeMs());
        const QVector<WaveformHistorySeries> full =
            history.query({38, 39},
                          history.firstTimeMs(),
                          history.lastTimeMs(),
                          10,
                          &error);
        QVERIFY2(!full.isEmpty(), qPrintable(error));
        QCOMPARE(full.size(), 2);
        bool foundSpike = false;
        for (const WaveformHistoryPoint& point : full[0].points)
        {
            foundSpike = foundSpike || (point.valid && std::abs(point.value - 999.0) < 1e-6);
        }
        QVERIFY(foundSpike);
        bool foundSummarizedGap = false;
        for (const WaveformHistoryPoint& point : full[0].points)
        {
            foundSummarizedGap = foundSummarizedGap || !point.valid;
        }
        QVERIFY(foundSummarizedGap);

        const QVector<WaveformHistorySeries> range =
            history.query({38}, 1001.0, 2001.0, 200, &error);
        QVERIFY2(!range.isEmpty(), qPrintable(error));
        for (const WaveformHistoryPoint& point : range[0].points)
        {
            if (point.valid)
            {
                QVERIFY(point.timeMs >= 1001.0);
                QVERIFY(point.timeMs <= 2001.0);
            }
        }

        QVERIFY2(history.clear(&error), qPrintable(error));
        QCOMPARE(history.sampleCount(), quint64(0));
        QVERIFY(history.isActive());

        const QString firstSessionPath = history.temporaryPath();
        QVERIFY2(history.append(sampleRow(true), 0, &error), qPrintable(error));
        QVERIFY2(history.beginSession(&error), qPrintable(error));
        QCOMPARE(history.sampleCount(), quint64(0));
        QVERIFY(history.isActive());
        QVERIFY(QFile::exists(history.temporaryPath()));
        if (history.temporaryPath() != firstSessionPath)
        {
            QVERIFY(!QFile::exists(firstSessionPath));
        }
    }
    QVERIFY(!QFile::exists(temporaryPath));
}

void JustFloatLogTests::waveformHistorySparseContinuityAndGaps()
{
    WaveformHistoryStore history;
    QString error;
    QVERIFY2(history.beginSession(&error), qPrintable(error));

    for (int i = 0; i < 10; ++i)
    {
        JustFloatLogRow row = sampleRow(true);
        row.rowTime = i * 10.0;
        row.syncTimeMs = i * 10.0;
        row.actualVelocityX = static_cast<float>(i);
        QVERIFY2(history.append(row, i * 10, &error), qPrintable(error));
    }

    QVector<WaveformHistorySeries> series =
        history.query({38}, history.firstTimeMs(), history.lastTimeMs(), 1000, &error);
    QVERIFY2(!series.isEmpty(), qPrintable(error));
    QCOMPARE(series[0].points.size(), 10);
    for (const WaveformHistoryPoint& point : series[0].points)
    {
        QVERIFY(point.valid);
    }

    QVERIFY2(history.clear(&error), qPrintable(error));
    for (int i = 0; i < 10; ++i)
    {
        JustFloatLogRow row = sampleRow(i != 5);
        row.rowTime = i * 10.0;
        row.syncTimeMs = i * 10.0;
        row.actualVelocityX = static_cast<float>(i);
        QVERIFY2(history.append(row, i * 10, &error), qPrintable(error));
    }

    series = history.query({38}, history.firstTimeMs(), history.lastTimeMs(), 1000, &error);
    QVERIFY2(!series.isEmpty(), qPrintable(error));
    bool foundGap = false;
    for (const WaveformHistoryPoint& point : series[0].points)
    {
        foundGap = foundGap || !point.valid;
    }
    QVERIFY(foundGap);

    error.clear();
    series = history.query({38},
                           std::numeric_limits<double>::quiet_NaN(),
                           history.lastTimeMs(),
                           1000,
                           &error);
    QVERIFY(!error.isEmpty());
    QVERIFY(!series.isEmpty());
    QVERIFY(series[0].points.isEmpty());
}

QTEST_MAIN(JustFloatLogTests)

#include "JustFloatLogTests.moc"
