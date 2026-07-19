#include "AlgorithmRunner.h"
#include "BeaconParameterDiagnostic.h"
#include "FrameRenderer.h"
#include "TwoBl3ParameterCatalog.h"

#include <QDir>
#include <QImage>
#include <QSet>
#include <QTemporaryDir>
#include <QtTest>

#include <cmath>

namespace
{
BimgParameterSnapshot parameterSnapshot(AlgorithmRunner* runner)
{
    BimgParameterSnapshot snapshot;
    snapshot.algorithmBuildId = runner->algorithmBuildId();
    snapshot.revision = 1;
    for (const AlgorithmParameterInfo& info : runner->parameterInfos())
    {
        BimgParameterValue value;
        value.id = info.id;
        value.type = info.type;
        if (!runner->parameterValue(value.type, value.id, &value.valueBits))
        {
            return {};
        }
        snapshot.values.push_back(value);
    }
    return snapshot;
}

bool hasBeaconInRegion(const beacon_result_t& result, const QRectF& region)
{
    const auto contains = [&region](const beacon_circle_t& beacon) {
        return beacon.valid != 0
               && region.contains(FrameRenderer::algorithmToImagePoint(beacon.x, beacon.y));
    };
    for (int index = 0; index < result.beacon_count; ++index)
    {
        if (contains(result.beacons[index]))
        {
            return true;
        }
    }
    for (int index = 0; index < result.temporal_beacon_count; ++index)
    {
        if (contains(result.temporal_beacons[index]))
        {
            return true;
        }
    }
    return false;
}
}

class TwoBl3AlgorithmRunnerTests : public QObject
{
    Q_OBJECT

private slots:
    void loadsParametersAndKeepsInstancesIsolated();
    void loadsPackagedFallbackLibrary();
    void catalogMatchesFirmwareParameters();
    void analyzesCompleteSnapshotWithoutUnsafeSuggestion();
    void searchesBeyondThirtyTwoStepsAndConverges();
    void rejectsInsufficientFrameHistory();
};

void TwoBl3AlgorithmRunnerTests::loadsParametersAndKeepsInstancesIsolated()
{
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner first;
    QString error;
    QVERIFY2(first.loadTwoBl3Firmware(QStringLiteral(BEACON_2BL3_TEST_IMAGE_DIR),
                                      QDir(buildRoot.path()).absoluteFilePath(QStringLiteral("first")),
                                      &error),
             qPrintable(error));
    QVERIFY(first.supportsParameterTuning());
    QCOMPARE(first.algorithmBuildId(), quint32(0x20260720U));
    QCOMPARE(first.parameterInfos().size(), 57);

    quint32 currentBits = 0;
    QVERIFY(first.parameterValue(1U, 0x0141U, &currentBits));
    QCOMPARE(currentBits, quint32(120U));
    quint32 actualBits = 0;
    QVERIFY(first.setParameterValue(1U, 0x0141U, 132U, &actualBits));
    QCOMPARE(actualBits, quint32(132U));

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    const beacon_result_t result = first.process(blank);
    QCOMPARE(result.beacon_count, quint8(0));

    AlgorithmRunner second;
    QVERIFY2(second.loadTwoBl3Firmware(QStringLiteral(BEACON_2BL3_TEST_IMAGE_DIR),
                                       QDir(buildRoot.path()).absoluteFilePath(QStringLiteral("second")),
                                       &error),
             qPrintable(error));
    quint32 secondBits = 0;
    QVERIFY(second.parameterValue(1U, 0x0141U, &secondBits));
    QCOMPARE(secondBits, quint32(120U));
}

void TwoBl3AlgorithmRunnerTests::loadsPackagedFallbackLibrary()
{
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());
    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(QStringLiteral("Z:/missing-2bl3-image-source"),
                                       buildRoot.path(),
                                       &error),
             qPrintable(error));
    QVERIFY(runner.supportsParameterTuning());
    QCOMPARE(runner.algorithmBuildId(), quint32(0x20260720U));
    QCOMPARE(runner.parameterInfos().size(), 57);
}

void TwoBl3AlgorithmRunnerTests::catalogMatchesFirmwareParameters()
{
    const QVector<TwoBl3ParameterDescriptor>& catalog = TwoBl3ParameterCatalog::all();
    QCOMPARE(catalog.size(), 57);
    QSet<quint16> ids;
    for (const TwoBl3ParameterDescriptor& descriptor : catalog)
    {
        QVERIFY(!ids.contains(descriptor.id));
        ids.insert(descriptor.id);
        QVERIFY(!descriptor.name.isEmpty());
        QVERIFY(!descriptor.menuPath.isEmpty());
        QVERIFY(descriptor.step > 0.0);
    }
}

void TwoBl3AlgorithmRunnerTests::analyzesCompleteSnapshotWithoutUnsafeSuggestion()
{
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());
    AlgorithmRunner source;
    QString error;
    QVERIFY2(source.loadTwoBl3Firmware(QStringLiteral(BEACON_2BL3_TEST_IMAGE_DIR),
                                       QDir(buildRoot.path()).absoluteFilePath(QStringLiteral("source")),
                                       &error),
             qPrintable(error));

    const BimgParameterSnapshot snapshot = parameterSnapshot(&source);
    QCOMPARE(snapshot.values.size(), source.parameterInfos().size());

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    BeaconDiagnosticRequest request;
    request.frames = QVector<QImage>(10, blank);
    request.region = QRectF(80, 45, 28, 28);
    request.snapshot = snapshot;
    request.firmwareImageDirectory = QStringLiteral(BEACON_2BL3_TEST_IMAGE_DIR);
    request.buildDirectory = QDir(buildRoot.path()).absoluteFilePath(QStringLiteral("diagnostic"));

    const BeaconDiagnosticResult result = BeaconParameterDiagnostic::analyze(request);
    QVERIFY2(result.completed, qPrintable(result.message));
    QVERIFY(!result.recommendationFound);
    QCOMPARE(result.analyzedFrameCount, 10);
    QVERIFY(result.message.contains(QStringLiteral("无"))
            || result.message.contains(QStringLiteral("未找到")));
}

void TwoBl3AlgorithmRunnerTests::searchesBeyondThirtyTwoStepsAndConverges()
{
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());
    AlgorithmRunner source;
    QString error;
    QVERIFY2(source.loadTwoBl3Firmware(QStringLiteral(BEACON_2BL3_TEST_IMAGE_DIR),
                                       QDir(buildRoot.path()).absoluteFilePath(QStringLiteral("source-wide")),
                                       &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    for (int y = 55; y < 65; ++y)
    {
        uchar* row = frame.scanLine(y);
        for (int x = 90; x < 100; ++x)
        {
            row[x] = 180;
        }
    }

    beacon_result_t baseline = {};
    source.resetTemporal();
    for (int frameIndex = 0; frameIndex < 10; ++frameIndex)
    {
        baseline = source.process(frame);
    }
    QVERIFY2(baseline.beacon_count > 0 || baseline.temporal_beacon_count > 0,
             "Synthetic beacon was not detected by the baseline firmware.");

    const beacon_circle_t* detected = baseline.beacon_count > 0
        ? &baseline.beacons[0]
        : &baseline.temporal_beacons[0];
    const QPointF center = FrameRenderer::algorithmToImagePoint(detected->x, detected->y);

    BeaconDiagnosticRequest request;
    request.frames = QVector<QImage>(10, frame);
    request.region = QRectF(center.x() - 8.0, center.y() - 8.0, 16.0, 16.0);
    request.snapshot = parameterSnapshot(&source);
    request.firmwareImageDirectory = QStringLiteral(BEACON_2BL3_TEST_IMAGE_DIR);
    request.buildDirectory = QDir(buildRoot.path()).absoluteFilePath(QStringLiteral("diagnostic-wide"));

    const BeaconDiagnosticResult result = BeaconParameterDiagnostic::analyze(request);
    QVERIFY2(result.completed, qPrintable(result.message));
    QVERIFY2(result.recommendationFound, qPrintable(result.message));
    const double distance = std::abs(result.recommendedValue - result.currentValue)
                            / result.parameter.step;
    QVERIFY2(distance > 32.0, qPrintable(QStringLiteral("Unexpected recommendation: %1 -> %2")
                                             .arg(result.currentValue)
                                             .arg(result.recommendedValue)));

    quint32 actualBits = 0;
    QVERIFY(source.setParameterValue(
        result.parameter.type,
        result.parameter.id,
        TwoBl3ParameterCatalog::bitsFromValue(result.parameter.type, result.recommendedValue),
        &actualBits));
    source.resetTemporal();
    beacon_result_t tuned = {};
    for (int frameIndex = 0; frameIndex < 10; ++frameIndex)
    {
        tuned = source.process(frame);
    }
    QVERIFY(!hasBeaconInRegion(tuned, request.region));

    const double closerValue = result.recommendedValue
                               + (result.recommendedValue > result.currentValue
                                      ? -result.parameter.step
                                      : result.parameter.step);
    QVERIFY(source.setParameterValue(
        result.parameter.type,
        result.parameter.id,
        TwoBl3ParameterCatalog::bitsFromValue(result.parameter.type, closerValue),
        &actualBits));
    source.resetTemporal();
    beacon_result_t closer = {};
    for (int frameIndex = 0; frameIndex < 10; ++frameIndex)
    {
        closer = source.process(frame);
    }
    QVERIFY(hasBeaconInRegion(closer, request.region));
}

void TwoBl3AlgorithmRunnerTests::rejectsInsufficientFrameHistory()
{
    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    BeaconDiagnosticRequest request;
    request.frames = QVector<QImage>(9, blank);
    request.region = QRectF(80, 45, 28, 28);

    const BeaconDiagnosticResult result = BeaconParameterDiagnostic::analyze(request);
    QVERIFY(!result.completed);
    QVERIFY(result.message.contains(QStringLiteral("10")));
}

QTEST_MAIN(TwoBl3AlgorithmRunnerTests)

#include "TwoBl3AlgorithmRunnerTests.moc"
