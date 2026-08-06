#include "AlgorithmRunner.h"
#include "BeaconParameterDiagnostic.h"
#include "FrameRenderer.h"
#include "TwoBl3ParameterCatalog.h"

#include <QDir>
#include <QFileInfo>
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

QString selfContainedImageDirectory()
{
    const QDir instancesRoot(QDir(QStringLiteral(BEACON_SOURCE_DIR))
                                 .absoluteFilePath(QStringLiteral("instances_front&back")));
    const QFileInfoList directories = instancesRoot.entryInfoList(
        QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QFileInfo& directory : directories)
    {
        const QDir instanceRoot(directory.absoluteFilePath());
        const QDir imageDirectory(instanceRoot.absoluteFilePath(
            QStringLiteral("algorithm/Image")));
        if (QFileInfo::exists(instanceRoot.absoluteFilePath(
                QStringLiteral("two_bl3_instance.json")))
            && QFileInfo::exists(imageDirectory.absoluteFilePath(QStringLiteral("image.c")))
            && QFileInfo::exists(imageDirectory.absoluteFilePath(QStringLiteral("image_params.c")))
            && QFileInfo::exists(imageDirectory.absoluteFilePath(QStringLiteral("image_horizon.c"))))
        {
            return imageDirectory.absolutePath();
        }
    }
    return {};
}

QString downImageDirectory()
{
    const QDir instancesRoot(QDir(QStringLiteral(BEACON_SOURCE_DIR))
                                 .absoluteFilePath(QStringLiteral("instances_down")));
    const QFileInfoList directories = instancesRoot.entryInfoList(
        QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QFileInfo& directory : directories)
    {
        const QDir instanceRoot(directory.absoluteFilePath());
        const QDir imageDirectory(instanceRoot.absoluteFilePath(
            QStringLiteral("algorithm/Image")));
        if (QFileInfo::exists(instanceRoot.absoluteFilePath(
                QStringLiteral("two_bl3_instance.json")))
            && QFileInfo::exists(imageDirectory.absoluteFilePath(QStringLiteral("image.c")))
            && QFileInfo::exists(imageDirectory.absoluteFilePath(QStringLiteral("image_down.h")))
            && QFileInfo::exists(imageDirectory.absoluteFilePath(QStringLiteral("image_params.c"))))
        {
            return imageDirectory.absolutePath();
        }
    }
    return {};
}

void drawDownBeacon(QImage* image, int centerX, int centerY, int radius)
{
    for (int y = centerY - radius; y <= centerY + radius; ++y)
    {
        if (y < 0 || y >= image->height())
        {
            continue;
        }
        uchar* row = image->scanLine(y);
        for (int x = centerX - radius; x <= centerX + radius; ++x)
        {
            if (x >= 0 && x < image->width()
                && (x - centerX) * (x - centerX) + (y - centerY) * (y - centerY)
                       <= radius * radius)
            {
                row[x] = 255;
            }
        }
    }
}

void drawDownBeacon(QImage* image, int centerX, int centerY)
{
    drawDownBeacon(image, centerX, centerY, 5);
}

void drawDownCarLamp(QImage* image, int centerX, int centerY)
{
    for (int y = centerY - 2; y <= centerY + 2; ++y)
    {
        uchar* row = image->scanLine(y);
        for (int x = centerX - 11; x <= centerX + 11; ++x)
        {
            row[x] = 250;
        }
    }
}

void drawDownVerticalCarLamp(QImage* image, int centerX, int centerY,
                             int halfWidth, int halfHeight)
{
    for (int y = centerY - halfHeight; y <= centerY + halfHeight; ++y)
    {
        if (y < 0 || y >= image->height())
        {
            continue;
        }
        uchar* row = image->scanLine(y);
        for (int x = centerX - halfWidth; x <= centerX + halfWidth; ++x)
        {
            if (x >= 0 && x < image->width())
            {
                row[x] = 250;
            }
        }
    }
}

void drawTinyThinDownCarLamp(QImage* image, int centerX, int centerY)
{
    for (int y = centerY - 3; y <= centerY + 3; ++y)
    {
        image->scanLine(y)[centerX] = 250;
    }
    image->scanLine(centerY)[centerX + 1] = 250;
}

void drawGrayConnectedFragmentedCar(QImage* image, int centerX, int centerY)
{
    for (int y = centerY - 7; y <= centerY + 7; ++y)
    {
        for (int x = centerX; x <= centerX + 1; ++x)
        {
            image->scanLine(y)[x] = 140;
        }
    }
    const int segmentStarts[] = {centerY - 7, centerY, centerY + 5};
    const int segmentLengths[] = {3, 1, 3};
    for (int segment = 0; segment < 3; ++segment)
    {
        for (int y = segmentStarts[segment];
             y < segmentStarts[segment] + segmentLengths[segment]; ++y)
        {
            for (int x = centerX; x <= centerX + 1; ++x)
            {
                image->scanLine(y)[x] = 250;
            }
        }
    }
}

void drawOccludedFragmentedCar(QImage* image, int centerX)
{
    for (int y = 58; y <= 61; ++y)
    {
        for (int x = centerX; x <= centerX + 2; ++x)
        {
            image->scanLine(y)[x] = 250;
        }
    }
    for (int y = 66; y <= 72; ++y)
    {
        for (int x = centerX; x <= centerX + 1; ++x)
        {
            image->scanLine(y)[x] = 250;
        }
    }
}

void drawMicroEnvelopeCar(QImage* image, int centerX, int centerY, int peakGray)
{
    for (int offset = -7; offset <= 7; ++offset)
    {
        const int x = centerX + offset;
        const int y = centerY + offset / 2;
        for (int widthOffset = -1; widthOffset <= 1; ++widthOffset)
        {
            image->scanLine(y + widthOffset)[x] = 100;
        }
    }
    for (int offset = -2; offset <= 2; ++offset)
    {
        image->scanLine(centerY + offset / 2)[centerX + offset] =
            static_cast<uchar>(peakGray);
    }
}

void drawEmbeddedLateralHighlight(QImage* image, int centerX, int centerY)
{
    for (int y = centerY - 14; y <= centerY + 14; ++y)
    {
        uchar* row = image->scanLine(y);
        for (int x = centerX - 2; x <= centerX + 2; ++x)
        {
            row[x] = 100;
        }
    }
    drawDownBeacon(image, centerX, centerY, 2);
}

void drawClippedDiffuseHighlight(QImage* image, int centerY)
{
    for (int y = centerY - 5; y <= centerY + 5; ++y)
    {
        uchar* row = image->scanLine(y);
        for (int x = 0; x <= 4; ++x)
        {
            row[x] = 100;
        }
    }
    for (int y = centerY - 2; y <= centerY + 2; ++y)
    {
        uchar* row = image->scanLine(y);
        for (int x = 0; x <= 2; ++x)
        {
            row[x] = 250;
        }
    }
}

void drawScaleVariantDownCarLamp(QImage* image, int centerX, int centerY)
{
    for (int y = centerY - 4; y <= centerY + 4; ++y)
    {
        uchar* row = image->scanLine(y);
        for (int x = centerX - 15; x <= centerX + 15; ++x)
        {
            row[x] = 250;
        }
    }
}

void drawWeakDownCarLamp(QImage* image, int centerX, int centerY)
{
    for (int y = centerY - 2; y <= centerY + 2; ++y)
    {
        uchar* row = image->scanLine(y);
        for (int x = centerX - 5; x < centerX + 5; ++x)
        {
            row[x] = 210;
        }
    }
}

void drawSyntheticCarLamp(QImage* image, int centerX, int centerY)
{
    for (int y = centerY - 7; y <= centerY + 7; ++y)
    {
        if (y < 0 || y >= image->height())
        {
            continue;
        }
        uchar* row = image->scanLine(y);
        for (int x = centerX - 11; x <= centerX + 11; ++x)
        {
            if (x < 0 || x >= image->width())
            {
                continue;
            }
            const int dx = x - centerX;
            const int dy = y - centerY;
            int gray = 0;
            if (dx * dx * 36 + dy * dy * 100 <= 3600)
            {
                gray = 140;
            }
            if (dx * dx * 25 + dy * dy * 256 <= 1600)
            {
                gray = 190;
            }
            if (dx * dx * 4 + dy * dy * 49 <= 196)
            {
                gray = 255;
            }
            if (gray > row[x])
            {
                row[x] = static_cast<uchar>(gray);
            }
        }
    }
}

void drawUniformCarLamp(QImage* image, int centerX, int centerY)
{
    const int halfWidths[] = {8, 10, 11, 10, 8};
    for (int rowIndex = 0; rowIndex < 5; ++rowIndex)
    {
        const int y = centerY - 2 + rowIndex;
        if (y < 0 || y >= image->height())
        {
            continue;
        }
        uchar* row = image->scanLine(y);
        for (int x = centerX - halfWidths[rowIndex];
             x <= centerX + halfWidths[rowIndex]; ++x)
        {
            if (x >= 0 && x < image->width())
            {
                row[x] = 250;
            }
        }
    }
}

void drawDiffuseCompactBeacon(QImage* image, int centerX, int centerY)
{
    for (int dy = -6; dy <= 6; ++dy)
    {
        uchar* row = image->scanLine(centerY + dy);
        for (int dx = -6; dx <= 6; ++dx)
        {
            const int radiusSquared = dx * dx + dy * dy;
            int gray = 0;
            if (radiusSquared <= 4)
            {
                gray = radiusSquared == 0 ? 250 : 160;
            }
            else if (radiusSquared <= 16)
            {
                gray = 25;
            }
            else if (radiusSquared <= 24)
            {
                gray = 50;
            }
            if (gray > row[centerX + dx])
            {
                row[centerX + dx] = static_cast<uchar>(gray);
            }
        }
    }
}

}

class TwoBl3AlgorithmRunnerTests : public QObject
{
    Q_OBJECT

private slots:
    void loadsParametersAndKeepsInstancesIsolated();
    void loadsPackagedFallbackLibrary();
    void loadsSelfContainedInstanceSnapshot();
    void loadsDownDraftInstance();
    void downDraftProcessesEachFrameExactlyOnce();
    void downDraftRendersExactClosedRegionBoundary();
    void downDraftDoesNotRenderClosedRegionAtViewportEdges();
    void downDraftUsesAreaOrderedB0B1AndTwoFrameCoast();
    void downDraftDoesNotCoastUnconfirmedBeacon();
    void downDraftMasksCarLampBeforeBeaconSelection();
    void downDraftFiltersBeaconOutsideClosedBoundary();
    void downDraftKeepsCurrentRotatedCarMeasurement();
    void downDraftMergesOnlyGrayConnectedCarFragments();
    void downDraftSuppressesOccludedCarFragmentBeacon();
    void downDraftPrefersFullyVisibleCarCandidate();
    void downDraftUsesTrackToChooseWeakCar();
    void downDraftRejectsCarGlareOutsideClosedBoundary();
    void downDraftSeparatesTinyThinCarFromCompactBeacon();
    void downDraftAcceptsScaleChangeWithoutMinorAxisLimit();
    void downDraftRejectsBoundaryBandStructureButKeepsInnerBeacon();
    void downDraftClassifiesMicroElongatedCarImmediately();
    void downDraftKeepsAmbiguousCarOutOfBeaconOutput();
    void downDraftRejectsEmbeddedLateralHighlightButKeepsEdgeBeacon();
    void selfContainedInstanceKeepsDiffuseCompactBeacon();
    void selfContainedInstanceRejectsEmbeddedCompactHighlights();
    void selfContainedInstanceFiltersBeaconsByHorizonGeometry();
    void selfContainedInstanceUsesCurrentB0AndTwoFrameCoast();
    void selfContainedInstanceHandlesDistantBeaconAndLampCompetition();
    void selfContainedInstanceRejectsCarLampAboveHorizon();
    void catalogMatchesFirmwareParameters();
    void analyzesCompleteSnapshotWithoutUnsafeSuggestion();
    void keepsStablePointSourceWhenNoSafeLegacyTuningExists();
    void usesCurrentMeasurementAndPredictsTwoMissingFrames();
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
    QCOMPARE(first.algorithmBuildId(), quint32(0x20260974U));
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
    QCOMPARE(runner.algorithmBuildId(), quint32(0x20260974U));
    QCOMPARE(runner.parameterInfos().size(), 57);
}

void TwoBl3AlgorithmRunnerTests::loadsSelfContainedInstanceSnapshot()
{
    const QString imageDirectory = selfContainedImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained 2BL3 instance was not found.");
    const QString instanceBuildDirectory = QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("../../build"));
    QVERIFY(QDir().mkpath(instanceBuildDirectory));
    QTemporaryDir buildRoot(QDir(instanceBuildDirectory).absoluteFilePath(
        QStringLiteral("load-test-XXXXXX")));
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(
                 imageDirectory,
                 buildRoot.path(),
                 &error),
             qPrintable(error));
    QVERIFY(runner.supportsParameterTuning());
    QCOMPARE(runner.algorithmBuildId(), quint32(0x20260906U));
    QCOMPARE(QFileInfo(runner.sourcePath()).canonicalFilePath(),
             QFileInfo(QDir(imageDirectory).absoluteFilePath(QStringLiteral("image.c")))
                 .canonicalFilePath());
}

void TwoBl3AlgorithmRunnerTests::loadsDownDraftInstance()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));
    QCOMPARE(runner.algorithmBuildId(), quint32(0x20260816U));
    const QVector<AlgorithmParameterInfo> parameters = runner.parameterInfos();
    QCOMPARE(parameters.size(), 22);
    QCOMPARE(parameters[18].type, quint8(1U));
    QCOMPARE(parameters[19].type, quint8(1U));
    QCOMPARE(parameters[20].type, quint8(0U));
    QCOMPARE(parameters[21].type, quint8(0U));

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    QCOMPARE(runner.process(blank).beacon_count, quint8(0));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawDownBeacon(&frame, 120, 70);
    const beacon_result_t result = runner.process(frame);
    QVERIFY(result.beacon_count >= 1U);
    QCOMPARE(result.beacons[0].valid, quint8(1U));
    QVERIFY(std::abs(result.beacons[0].x + 26.0f) < 1.0f);
    QVERIFY(std::abs(result.beacons[0].y - 10.0f) < 1.5f);
}

void TwoBl3AlgorithmRunnerTests::downDraftProcessesEachFrameExactlyOnce()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));
    QCOMPARE(runner.processedFrameCount(), quint32(0U));

    AlgorithmFrameTelemetry telemetry;
    telemetry.rollDeg = -0.360687941f;
    telemetry.pitchDeg = -0.756091833f;
    telemetry.heightMm = 1112.39966f;
    telemetry.attitudeValid = true;
    telemetry.heightValid = true;
    runner.setFrameTelemetry(telemetry);

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    (void)runner.process(frame);
    QCOMPARE(runner.processedFrameCount(), quint32(1U));
    QVERIFY(!runner.binaryImage(frame).isNull());
    QCOMPARE(runner.processedFrameCount(), quint32(1U));
    (void)runner.horizonCurve();
    QCOMPARE(runner.processedFrameCount(), quint32(1U));

    (void)runner.process(frame);
    QCOMPARE(runner.processedFrameCount(), quint32(2U));
}

void TwoBl3AlgorithmRunnerTests::downDraftRendersExactClosedRegionBoundary()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    AlgorithmFrameTelemetry telemetry;
    telemetry.rollDeg = -0.360687941f;
    telemetry.pitchDeg = -0.756091833f;
    telemetry.heightMm = 1112.39966f;
    telemetry.attitudeValid = true;
    telemetry.heightValid = true;
    runner.setFrameTelemetry(telemetry);

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    const beacon_result_t result = runner.process(frame);
    const AlgorithmHorizonCurve horizon = runner.horizonCurve();
    QVERIFY(horizon.valid);
    QVERIFY(horizon.secondaryValid);
    QVERIFY(horizon.closedRegion);

    int closingColumn = -1;
    int closingY = -1;
    for (int x = 0; x + 1 < BEACON_IMAGE_W; ++x)
    {
        const float top = horizon.y[static_cast<std::size_t>(x)];
        const float bottom = horizon.secondaryY[static_cast<std::size_t>(x)];
        if (horizon.columnState[static_cast<std::size_t>(x)] == 1U &&
            horizon.columnState[static_cast<std::size_t>(x + 1)] == 0U &&
            top >= 0.0f && bottom <= static_cast<float>(BEACON_IMAGE_H - 1) &&
            bottom - top >= 2.0f)
        {
            closingColumn = x;
            closingY = static_cast<int>((top + bottom) * 0.5f + 0.5f);
            break;
        }
    }
    QVERIFY(closingColumn >= 0);
    const QImage rendered = FrameRenderer::render(frame, result, {}, 1, true, &horizon);
    const QColor boundary = rendered.pixelColor(closingColumn, closingY);
    QCOMPARE(boundary, QColor(40, 255, 80));
    QVERIFY(rendered.pixelColor(closingColumn + 1, closingY) != boundary);
}

void TwoBl3AlgorithmRunnerTests::downDraftDoesNotRenderClosedRegionAtViewportEdges()
{
    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    beacon_result_t result = {};
    AlgorithmHorizonCurve horizon;
    horizon.valid = true;
    horizon.secondaryValid = true;
    horizon.closedRegion = true;

    horizon.columnState.fill(2U);
    const QImage fullyInside = FrameRenderer::render(frame, result, {}, 1, true, &horizon);
    const QColor boundary(40, 255, 80);
    for (int y = 0; y < BEACON_IMAGE_H; ++y)
    {
        for (int x = 0; x < BEACON_IMAGE_W; ++x)
        {
            QVERIFY(fullyInside.pixelColor(x, y) != boundary);
        }
    }

    horizon.columnState.fill(1U);
    horizon.columnValid.fill(1U);
    horizon.secondaryColumnValid.fill(1U);
    horizon.y.fill(-8.0f);
    horizon.secondaryY.fill(70.0f);
    const QImage lowerBoundary = FrameRenderer::render(frame, result, {}, 1, true, &horizon);
    QVERIFY(lowerBoundary.pixelColor(BEACON_IMAGE_W / 2, 0) != boundary);
    QCOMPARE(lowerBoundary.pixelColor(BEACON_IMAGE_W / 2, 70), boundary);
    QVERIFY(lowerBoundary.pixelColor(0, 30) != boundary);
    QVERIFY(lowerBoundary.pixelColor(BEACON_IMAGE_W - 1, 30) != boundary);

    horizon.y.fill(35.0f);
    horizon.secondaryY.fill(static_cast<float>(BEACON_IMAGE_H + 9));
    const QImage upperBoundary = FrameRenderer::render(frame, result, {}, 1, true, &horizon);
    QCOMPARE(upperBoundary.pixelColor(BEACON_IMAGE_W / 2, 35), boundary);
    QVERIFY(upperBoundary.pixelColor(BEACON_IMAGE_W / 2,
                                      BEACON_IMAGE_H - 1) != boundary);
}

void TwoBl3AlgorithmRunnerTests::downDraftUsesAreaOrderedB0B1AndTwoFrameCoast()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawDownBeacon(&frame, 120, 70, 5);
    drawDownBeacon(&frame, 60, 50, 3);

    const beacon_result_t firstCurrent = runner.process(frame);
    QCOMPARE(firstCurrent.beacon_count, quint8(2U));

    const beacon_result_t current = runner.process(frame);
    QCOMPARE(current.beacon_count, quint8(2U));
    QVERIFY(current.beacons[0].radius > current.beacons[1].radius);
    QVERIFY(std::abs(current.beacons[0].x + 26.0f) < 2.0f);
    QVERIFY(std::abs(current.beacons[1].x - 34.0f) < 1.0f);

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    const beacon_result_t firstMiss = runner.process(blank);
    const beacon_result_t secondMiss = runner.process(blank);
    const beacon_result_t thirdMiss = runner.process(blank);
    QCOMPARE(firstMiss.beacon_count, quint8(1U));
    QCOMPARE(secondMiss.beacon_count, quint8(1U));
    QCOMPARE(thirdMiss.beacon_count, quint8(0U));
}

void TwoBl3AlgorithmRunnerTests::downDraftDoesNotCoastUnconfirmedBeacon()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawDownBeacon(&frame, 120, 70, 5);
    QCOMPARE(runner.process(frame).beacon_count, quint8(1U));

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    QCOMPARE(runner.process(blank).beacon_count, quint8(0U));
}

void TwoBl3AlgorithmRunnerTests::downDraftMasksCarLampBeforeBeaconSelection()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawDownCarLamp(&frame, 81, 50);
    drawDownBeacon(&frame, 130, 70, 4);

    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    QCOMPARE(result.beacon_count, quint8(1U));
    QVERIFY(std::abs(result.beacons[0].x + 36.0f) < 1.5f);
    QVERIFY(std::abs(result.beacons[0].y - 10.0f) < 1.5f);
}

void TwoBl3AlgorithmRunnerTests::downDraftFiltersBeaconOutsideClosedBoundary()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage edgeFrame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    edgeFrame.fill(0);
    drawDownBeacon(&edgeFrame, 170, 3, 3);
    QCOMPARE(runner.process(edgeFrame).beacon_count, quint8(1U));

    runner.resetTemporal();
    AlgorithmFrameTelemetry telemetry;
    telemetry.rollDeg = 0.0f;
    telemetry.pitchDeg = 0.0f;
    telemetry.heightMm = 1000.0f;
    telemetry.attitudeValid = true;
    telemetry.heightValid = true;
    runner.setFrameTelemetry(telemetry);
    QCOMPARE(runner.process(edgeFrame).beacon_count, quint8(0U));

    const AlgorithmHorizonCurve horizon = runner.horizonCurve();
    QVERIFY(horizon.valid);
    QVERIFY(horizon.secondaryValid);
    QVERIFY(horizon.columnValid[0] != 0U);
    QVERIFY(horizon.secondaryColumnValid[0] != 0U);
    QVERIFY(horizon.y[0] < horizon.secondaryY[0]);
}

void TwoBl3AlgorithmRunnerTests::downDraftKeepsCurrentRotatedCarMeasurement()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage horizontal(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    horizontal.fill(0);
    drawDownCarLamp(&horizontal, 60, 60);
    QCOMPARE(runner.process(horizontal).car_lamp_count, quint8(1U));
    QCOMPARE(runner.process(horizontal).car_lamp_count, quint8(1U));

    QImage rotated(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    rotated.fill(0);
    drawDownVerticalCarLamp(&rotated, 120, 35, 2, 6);
    const beacon_result_t result = runner.process(rotated);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    QVERIFY(std::hypot(center.x() - 120.0, center.y() - 35.0) < 0.5);

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    QCOMPARE(runner.process(blank).car_lamp_count, quint8(0U));
}

void TwoBl3AlgorithmRunnerTests::downDraftMergesOnlyGrayConnectedCarFragments()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage connected(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    connected.fill(30);
    drawGrayConnectedFragmentedCar(&connected, 80, 60);
    const beacon_result_t merged = runner.process(connected);
    QCOMPARE(merged.car_lamp_count, quint8(1U));
    QCOMPARE(merged.beacon_count, quint8(0U));
    const QPointF mergedCenter = FrameRenderer::algorithmToImagePoint(
        merged.car_lamps[0].cx, merged.car_lamps[0].cy);
    QVERIFY(std::hypot(mergedCenter.x() - 80.5, mergedCenter.y() - 60.0) < 1.0);

    runner.resetTemporal();
    QImage disconnected = connected;
    for (int y = 53; y <= 67; ++y)
    {
        for (int x = 80; x <= 81; ++x)
        {
            if (disconnected.scanLine(y)[x] < 200)
            {
                disconnected.scanLine(y)[x] = 30;
            }
        }
    }
    QCOMPARE(runner.process(disconnected).car_lamp_count, quint8(0U));
}

void TwoBl3AlgorithmRunnerTests::downDraftSuppressesOccludedCarFragmentBeacon()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage confirmed(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    confirmed.fill(30);
    drawDownVerticalCarLamp(&confirmed, 79, 64, 1, 7);
    QCOMPARE(runner.process(confirmed).car_lamp_count, quint8(1U));
    QCOMPARE(runner.process(confirmed).car_lamp_count, quint8(1U));

    QImage occluded(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    occluded.fill(30);
    drawOccludedFragmentedCar(&occluded, 78);
    drawDownBeacon(&occluded, 120, 93, 4);
    const beacon_result_t result = runner.process(occluded);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    QCOMPARE(result.beacon_count, quint8(1U));
    const QPointF beaconCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(beaconCenter.x() - 120.0, beaconCenter.y() - 93.0) < 1.5);
}

void TwoBl3AlgorithmRunnerTests::downDraftPrefersFullyVisibleCarCandidate()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(30);
    drawDownCarLamp(&frame, 110, 55);
    for (int y = BEACON_IMAGE_H - 4; y < BEACON_IMAGE_H; ++y)
    {
        uchar* row = frame.scanLine(y);
        for (int x = 28; x <= 66; ++x)
        {
            row[x] = 250;
        }
    }

    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    QVERIFY(std::hypot(center.x() - 110.0, center.y() - 55.0) < 1.0);
}

void TwoBl3AlgorithmRunnerTests::downDraftUsesTrackToChooseWeakCar()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage strong(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    strong.fill(0);
    drawDownCarLamp(&strong, 60, 75);
    QCOMPARE(runner.process(strong).car_lamp_count, quint8(1U));
    QCOMPARE(runner.process(strong).car_lamp_count, quint8(1U));

    QImage mixed(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    mixed.fill(0);
    drawTinyThinDownCarLamp(&mixed, 59, 82);
    drawTinyThinDownCarLamp(&mixed, 56, 106);
    drawDownBeacon(&mixed, 130, 70, 4);

    const beacon_result_t result = runner.process(mixed);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    const QPointF carCenter = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    QVERIFY(std::hypot(carCenter.x() - 59.0, carCenter.y() - 82.0) < 0.5);
    QCOMPARE(result.beacon_count, quint8(1U));
    const QPointF beaconCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(beaconCenter.x() - 130.0, beaconCenter.y() - 70.0) < 1.5);
}

void TwoBl3AlgorithmRunnerTests::downDraftRejectsCarGlareOutsideClosedBoundary()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));
    AlgorithmFrameTelemetry telemetry;
    telemetry.rollDeg = 0.0f;
    telemetry.pitchDeg = 0.0f;
    telemetry.heightMm = 1000.0f;
    telemetry.attitudeValid = true;
    telemetry.heightValid = true;
    runner.setFrameTelemetry(telemetry);

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawDownCarLamp(&frame, 170, 3);
    drawDownVerticalCarLamp(&frame, 80, 60, 2, 6);

    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    QVERIFY(std::hypot(center.x() - 80.0, center.y() - 60.0) < 0.5);
}

void TwoBl3AlgorithmRunnerTests::downDraftSeparatesTinyThinCarFromCompactBeacon()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawTinyThinDownCarLamp(&frame, 60, 70);
    drawDownBeacon(&frame, 120, 80, 2);

    runner.process(frame);
    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    const QPointF carCenter = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    QVERIFY(std::hypot(carCenter.x() - 60.0, carCenter.y() - 70.0) < 1.0);
    QCOMPARE(result.beacon_count, quint8(1U));
    const QPointF beaconCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(beaconCenter.x() - 120.0, beaconCenter.y() - 80.0) < 1.5);
}

void TwoBl3AlgorithmRunnerTests::downDraftAcceptsScaleChangeWithoutMinorAxisLimit()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawScaleVariantDownCarLamp(&frame, 80, 65);

    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    const QString centerMessage = QStringLiteral("center=(%1,%2), width=%3, length=%4")
                                      .arg(center.x())
                                      .arg(center.y())
                                      .arg(result.car_lamps[0].width)
                                      .arg(result.car_lamps[0].length);
    QVERIFY2(std::hypot(center.x() - 80.0, center.y() - 65.0) < 1.0,
             qPrintable(centerMessage));
    QVERIFY(result.car_lamps[0].width > 7.0f);
    QVERIFY(result.car_lamps[0].length > 25.0f);
}

void TwoBl3AlgorithmRunnerTests::downDraftRejectsBoundaryBandStructureButKeepsInnerBeacon()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));
    AlgorithmFrameTelemetry telemetry;
    telemetry.rollDeg = 0.0f;
    telemetry.pitchDeg = 0.0f;
    telemetry.heightMm = 1000.0f;
    telemetry.attitudeValid = true;
    telemetry.heightValid = true;
    runner.setFrameTelemetry(telemetry);

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    runner.process(blank);
    const AlgorithmHorizonCurve horizon = runner.horizonCurve();
    int edgeX = -1;
    const int innerX = 30;
    for (int x = 3; x < 35; ++x)
    {
        if (horizon.columnValid[static_cast<std::size_t>(x)] != 0U
            && horizon.secondaryColumnValid[static_cast<std::size_t>(x)] != 0U
            && horizon.secondaryY[static_cast<std::size_t>(x)]
                   - horizon.y[static_cast<std::size_t>(x)] > 12.0f)
        {
            if (edgeX < 0 && x < 15)
            {
                edgeX = x;
            }
        }
    }
    QVERIFY(edgeX >= 0);

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    const int edgeY = qRound(horizon.y[static_cast<std::size_t>(edgeX)] + 1.0f);
    const int innerY = BEACON_IMAGE_H / 2;
    drawDownBeacon(&frame, edgeX, edgeY, 2);
    drawDownBeacon(&frame, innerX, innerY, 2);

    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.beacon_count, quint8(1U));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(center.x() - innerX, center.y() - innerY) < 1.5);
}

void TwoBl3AlgorithmRunnerTests::downDraftClassifiesMicroElongatedCarImmediately()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(40);
    drawMicroEnvelopeCar(&frame, 60, 45, 250);

    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.car_lamp_count, quint8(1U));
    QCOMPARE(result.beacon_count, quint8(0U));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    QVERIFY(std::hypot(center.x() - 60.0, center.y() - 45.0) < 2.0);
}

void TwoBl3AlgorithmRunnerTests::downDraftKeepsAmbiguousCarOutOfBeaconOutput()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage ambiguous(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    ambiguous.fill(40);
    drawWeakDownCarLamp(&ambiguous, 70, 55);
    const beacon_result_t first = runner.process(ambiguous);
    QCOMPARE(first.car_lamp_count, quint8(0U));
    QCOMPARE(first.beacon_count, quint8(0U));

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(40);
    QCOMPARE(runner.process(blank).car_lamp_count, quint8(0U));

    const beacon_result_t confirmed = runner.process(ambiguous);
    QCOMPARE(confirmed.car_lamp_count, quint8(1U));
    QCOMPARE(confirmed.beacon_count, quint8(0U));
}

void TwoBl3AlgorithmRunnerTests::downDraftRejectsEmbeddedLateralHighlightButKeepsEdgeBeacon()
{
    const QString imageDirectory = downImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained down instance was not found.");
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage embedded(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    embedded.fill(30);
    drawEmbeddedLateralHighlight(&embedded, 6, 60);
    const beacon_result_t rejected = runner.process(embedded);
    QCOMPARE(rejected.car_lamp_count, quint8(0U));
    QCOMPARE(rejected.beacon_count, quint8(0U));

    runner.resetTemporal();
    QImage edgeBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    edgeBeacon.fill(30);
    drawDownBeacon(&edgeBeacon, 9, 60, 2);
    const beacon_result_t kept = runner.process(edgeBeacon);
    QCOMPARE(kept.car_lamp_count, quint8(0U));
    QCOMPARE(kept.beacon_count, quint8(1U));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        kept.beacons[0].x, kept.beacons[0].y);
    QVERIFY(std::hypot(center.x() - 9.0, center.y() - 60.0) < 1.5);

    runner.resetTemporal();
    QImage clipped(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    clipped.fill(30);
    drawClippedDiffuseHighlight(&clipped, 70);
    QCOMPARE(runner.process(clipped).beacon_count, quint8(0U));

    runner.resetTemporal();
    QImage nearTop(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    nearTop.fill(30);
    drawDownBeacon(&nearTop, 30, 3, 2);
    QCOMPARE(runner.process(nearTop).beacon_count, quint8(1U));

    runner.resetTemporal();
    QImage widenedEdgeCheck(BEACON_IMAGE_W, BEACON_IMAGE_H,
                            QImage::Format_Grayscale8);
    widenedEdgeCheck.fill(30);
    drawEmbeddedLateralHighlight(&widenedEdgeCheck, 20, 60);
    QCOMPARE(runner.process(widenedEdgeCheck).beacon_count, quint8(0U));

    runner.resetTemporal();
    QImage rightNearEdge(BEACON_IMAGE_W, BEACON_IMAGE_H,
                         QImage::Format_Grayscale8);
    rightNearEdge.fill(30);
    drawDownBeacon(&rightNearEdge, 170, 60, 2);
    QCOMPARE(runner.process(rightNearEdge).beacon_count, quint8(1U));

    runner.resetTemporal();
    QImage largeEdgeBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H,
                           QImage::Format_Grayscale8);
    largeEdgeBeacon.fill(50);
    drawDownBeacon(&largeEdgeBeacon, 13, 56, 4);
    const beacon_result_t largeKept = runner.process(largeEdgeBeacon);
    QCOMPARE(largeKept.beacon_count, quint8(1U));
    const QPointF largeCenter = FrameRenderer::algorithmToImagePoint(
        largeKept.beacons[0].x, largeKept.beacons[0].y);
    QVERIFY(std::hypot(largeCenter.x() - 13.0, largeCenter.y() - 56.0) < 1.5);
}

void TwoBl3AlgorithmRunnerTests::selfContainedInstanceKeepsDiffuseCompactBeacon()
{
    const QString imageDirectory = selfContainedImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained 2BL3 instance was not found.");
    const QString instanceBuildDirectory = QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("../../build"));
    QVERIFY(QDir().mkpath(instanceBuildDirectory));
    QTemporaryDir buildRoot(QDir(instanceBuildDirectory).absoluteFilePath(
        QStringLiteral("compact-beacon-test-XXXXXX")));
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage frame(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    frame.fill(0);
    drawDiffuseCompactBeacon(&frame, 77, 39);

    const beacon_result_t result = runner.process(frame);
    QCOMPARE(result.beacon_count, quint8(1));
    QCOMPARE(result.temporal_beacon_count, quint8(0));
    const QPointF center = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(center.x() - 77.0, center.y() - 39.0) < 0.5);
}

void TwoBl3AlgorithmRunnerTests::selfContainedInstanceRejectsEmbeddedCompactHighlights()
{
    const QString imageDirectory = selfContainedImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained 2BL3 instance was not found.");
    const QString instanceBuildDirectory = QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("../../build"));
    QVERIFY(QDir().mkpath(instanceBuildDirectory));
    QTemporaryDir buildRoot(QDir(instanceBuildDirectory).absoluteFilePath(
        QStringLiteral("embedded-highlight-test-XXXXXX")));
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage line(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    line.fill(0);
    for (int x = 38; x <= 62; ++x)
    {
        line.scanLine(50)[x] = 50;
    }
    for (int y = 49; y <= 50; ++y)
    {
        uchar* row = line.scanLine(y);
        row[49] = 245;
        row[50] = 245;
    }
    QCOMPARE(runner.process(line).beacon_count, quint8(0));

    runner.resetTemporal();
    QImage diffuse(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    diffuse.fill(0);
    for (int dy = -12; dy <= 12; ++dy)
    {
        uchar* row = diffuse.scanLine(50 + dy);
        for (int dx = -12; dx <= 12; ++dx)
        {
            const int radiusSquared = dx * dx + dy * dy;
            if (radiusSquared >= 64 && radiusSquared <= 100)
            {
                row[50 + dx] = 20;
            }
        }
    }
    for (int y = 49; y <= 50; ++y)
    {
        uchar* row = diffuse.scanLine(y);
        row[49] = 245;
        row[50] = 245;
    }
    QCOMPARE(runner.process(diffuse).beacon_count, quint8(0));

    runner.resetTemporal();
    QImage reflection(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    reflection.fill(36);
    int outerIndex = 0;
    for (int dy = -12; dy <= 12; ++dy)
    {
        uchar* row = reflection.scanLine(50 + dy);
        for (int dx = -12; dx <= 12; ++dx)
        {
            const int radiusSquared = dx * dx + dy * dy;
            if (radiusSquared <= 49 || radiusSquared > 144)
            {
                continue;
            }
            row[50 + dx] = outerIndex < 9 ? 66 :
                               outerIndex < 58 ? 56 :
                               outerIndex < 104 ? 46 : 36;
            ++outerIndex;
        }
    }
    for (int y = 49; y <= 50; ++y)
    {
        uchar* row = reflection.scanLine(y);
        for (int x = 49; x <= 52; ++x)
        {
            row[x] = 254;
        }
    }
    QCOMPARE(runner.process(reflection).beacon_count, quint8(0));
}

void TwoBl3AlgorithmRunnerTests::selfContainedInstanceFiltersBeaconsByHorizonGeometry()
{
    const QString imageDirectory = selfContainedImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained 2BL3 instance was not found.");
    const QString instanceBuildDirectory = QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("../../build"));
    QVERIFY(QDir().mkpath(instanceBuildDirectory));
    QTemporaryDir buildRoot(QDir(instanceBuildDirectory).absoluteFilePath(
        QStringLiteral("horizon-beacon-test-XXXXXX")));
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage boundaryBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    boundaryBeacon.fill(0);
    drawDiffuseCompactBeacon(&boundaryBeacon, 27, 26);
    QCOMPARE(runner.process(boundaryBeacon).beacon_count, quint8(1));
    QVERIFY(!runner.horizonCurve().valid);

    runner.resetTemporal();
    AlgorithmFrameTelemetry telemetry;
    telemetry.cameraId = 0;
    telemetry.rollDeg = -5.69920158f;
    telemetry.pitchDeg = -0.750505924f;
    telemetry.heightMm = 1105.68286f;
    telemetry.attitudeValid = true;
    telemetry.heightValid = true;
    runner.setFrameTelemetry(telemetry);
    QCOMPARE(runner.process(boundaryBeacon).beacon_count, quint8(0));
    const AlgorithmHorizonCurve horizon = runner.horizonCurve();
    QVERIFY(horizon.valid);
    int validColumns = 0;
    for (int x = 0; x < BEACON_IMAGE_W; ++x)
    {
        if (horizon.columnValid[static_cast<std::size_t>(x)] != 0U)
        {
            QVERIFY(std::isfinite(horizon.y[static_cast<std::size_t>(x)]));
            ++validColumns;
        }
    }
    QVERIFY(validColumns > BEACON_IMAGE_W / 2);

    runner.resetTemporal();
    telemetry.rollDeg = -2.57557487f;
    telemetry.pitchDeg = -3.0064311f;
    telemetry.heightMm = 984.535339f;
    runner.setFrameTelemetry(telemetry);
    QImage tinyNearPoint(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    tinyNearPoint.fill(0);
    for (int y = 77; y <= 78; ++y)
    {
        uchar* row = tinyNearPoint.scanLine(y);
        for (int x = 109; x <= 111; ++x)
        {
            row[x] = 250;
        }
    }
    QCOMPARE(runner.process(tinyNearPoint).beacon_count, quint8(0));

    runner.resetTemporal();
    runner.setFrameTelemetry(telemetry);
    QImage validNearBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    validNearBeacon.fill(0);
    drawDiffuseCompactBeacon(&validNearBeacon, 77, 60);
    QCOMPARE(runner.process(validNearBeacon).beacon_count, quint8(1));

    runner.resetTemporal();
    telemetry.rollDeg = 4.04927301f;
    telemetry.pitchDeg = 2.24179077f;
    telemetry.heightMm = 999.416992f;
    runner.setFrameTelemetry(telemetry);
    QImage distantBoundaryBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H,
                                 QImage::Format_Grayscale8);
    distantBoundaryBeacon.fill(30);
    distantBoundaryBeacon.scanLine(22)[76] = 242;
    distantBoundaryBeacon.scanLine(22)[75] = 170;
    distantBoundaryBeacon.scanLine(22)[77] = 170;
    distantBoundaryBeacon.scanLine(21)[76] = 170;
    distantBoundaryBeacon.scanLine(23)[76] = 170;
    QCOMPARE(runner.process(distantBoundaryBeacon).beacon_count, quint8(1));

    runner.resetTemporal();
    runner.setFrameTelemetry(telemetry);
    QImage darkFloorReflection = distantBoundaryBeacon;
    darkFloorReflection.fill(12);
    darkFloorReflection.scanLine(22)[76] = 242;
    darkFloorReflection.scanLine(22)[75] = 170;
    darkFloorReflection.scanLine(22)[77] = 170;
    darkFloorReflection.scanLine(21)[76] = 170;
    darkFloorReflection.scanLine(23)[76] = 170;
    QCOMPARE(runner.process(darkFloorReflection).beacon_count, quint8(0));

    runner.resetTemporal();
    runner.setFrameTelemetry(telemetry);
    QImage broadBoundaryGlare(BEACON_IMAGE_W, BEACON_IMAGE_H,
                              QImage::Format_Grayscale8);
    broadBoundaryGlare.fill(30);
    for (int y = 20; y <= 24; ++y)
    {
        uchar* row = broadBoundaryGlare.scanLine(y);
        for (int x = 74; x <= 78; ++x)
        {
            row[x] = 230;
        }
    }
    QCOMPARE(runner.process(broadBoundaryGlare).beacon_count, quint8(0));
}

void TwoBl3AlgorithmRunnerTests::selfContainedInstanceUsesCurrentB0AndTwoFrameCoast()
{
    const QString imageDirectory = selfContainedImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained 2BL3 instance was not found.");
    const QString instanceBuildDirectory = QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("../../build"));
    QVERIFY(QDir().mkpath(instanceBuildDirectory));
    QTemporaryDir buildRoot(QDir(instanceBuildDirectory).absoluteFilePath(
        QStringLiteral("temporal-test-XXXXXX")));
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage beacon(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    beacon.fill(0);
    for (int y = 56; y < 64; ++y)
    {
        uchar* row = beacon.scanLine(y);
        for (int x = 90; x < 98; ++x)
        {
            row[x] = 220;
        }
    }

    beacon_result_t result = runner.process(beacon);
    QCOMPARE(result.beacon_count, quint8(1));
    QCOMPARE(result.temporal_beacon_count, quint8(0));

    QImage abruptArea(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    abruptArea.fill(0);
    for (int y = 59; y < 61; ++y)
    {
        uchar* row = abruptArea.scanLine(y);
        for (int x = 93; x < 95; ++x)
        {
            row[x] = 220;
        }
    }
    result = runner.process(abruptArea);
    QCOMPARE(result.beacon_count, quint8(1));
    QCOMPARE(result.temporal_beacon_count, quint8(0));

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    result = runner.process(blank);
    QCOMPARE(result.beacon_count, quint8(0));
    QCOMPARE(result.temporal_beacon_count, quint8(1));

    QImage switchedBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    switchedBeacon.fill(0);
    for (int y = 30; y < 38; ++y)
    {
        uchar* row = switchedBeacon.scanLine(y);
        for (int x = 130; x < 138; ++x)
        {
            row[x] = 220;
        }
    }
    result = runner.process(switchedBeacon);
    QCOMPARE(result.beacon_count, quint8(1));
    QCOMPARE(result.temporal_beacon_count, quint8(0));
    QVERIFY(result.beacons[0].x < -30.0f);

    result = runner.process(blank);
    QCOMPARE(result.beacon_count, quint8(0));
    QCOMPARE(result.temporal_beacon_count, quint8(1));
    result = runner.process(blank);
    QCOMPARE(result.beacon_count, quint8(0));
    QCOMPARE(result.temporal_beacon_count, quint8(1));
    result = runner.process(blank);
    QCOMPARE(result.beacon_count, quint8(0));
    QCOMPARE(result.temporal_beacon_count, quint8(0));

    runner.resetTemporal();
    QImage carLamp(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    carLamp.fill(0);
    drawSyntheticCarLamp(&carLamp, 50, 72);
    drawSyntheticCarLamp(&carLamp, 90, 100);
    result = runner.process(carLamp);
    QCOMPARE(result.car_lamp_count, quint8(1));
    QVERIFY(60.0f + result.car_lamps[0].cy > 90.0f);

    runner.resetTemporal();
    QImage uniformStrip(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    uniformStrip.fill(20);
    const int stripLeft[] = {45, 42, 40, 42, 45};
    const int stripRight[] = {53, 56, 58, 56, 53};
    for (int rowIndex = 0; rowIndex < 5; ++rowIndex)
    {
        uchar* row = uniformStrip.scanLine(100 + rowIndex);
        for (int x = stripLeft[rowIndex]; x <= stripRight[rowIndex]; ++x)
        {
            row[x] = 250;
        }
    }
    result = runner.process(uniformStrip);
    QCOMPARE(result.car_lamp_count, quint8(1));

    runner.resetTemporal();
    QImage lowExposureLamp(BEACON_IMAGE_W, BEACON_IMAGE_H,
                           QImage::Format_Grayscale8);
    lowExposureLamp.fill(0);
    for (int y = 48; y <= 51; ++y)
    {
        uchar* row = lowExposureLamp.scanLine(y);
        for (int x = 84; x <= 94; ++x)
        {
            row[x] = 150;
        }
    }
    for (int y = 49; y <= 50; ++y)
    {
        uchar* row = lowExposureLamp.scanLine(y);
        for (int x = 87; x <= 91; ++x)
        {
            row[x] = 250;
        }
    }
    result = runner.process(lowExposureLamp);
    QCOMPARE(result.car_lamp_count, quint8(1));

    QImage weakTrackedLamp(BEACON_IMAGE_W, BEACON_IMAGE_H,
                           QImage::Format_Grayscale8);
    weakTrackedLamp.fill(0);
    QImage strongTrackedLamp = weakTrackedLamp;
    const int trackedLeft[] = {29, 29, 30, 33, 35};
    const int trackedRight[] = {32, 34, 37, 37, 36};
    int trackedPixel = 0;
    for (int rowIndex = 0; rowIndex < 5; ++rowIndex)
    {
        uchar* weakRow = weakTrackedLamp.scanLine(90 + rowIndex);
        uchar* strongRow = strongTrackedLamp.scanLine(90 + rowIndex);
        for (int x = trackedLeft[rowIndex]; x <= trackedRight[rowIndex]; ++x)
        {
            weakRow[x] = trackedPixel < 8 ? 140 : 230;
            strongRow[x] = 230;
            ++trackedPixel;
        }
    }
    weakTrackedLamp.scanLine(92)[34] = 250;
    strongTrackedLamp.scanLine(92)[34] = 250;

    runner.resetTemporal();
    QCOMPARE(runner.process(weakTrackedLamp).car_lamp_count, quint8(0));
    for (int frameIndex = 0; frameIndex < 3; ++frameIndex)
    {
        result = runner.process(strongTrackedLamp);
        QCOMPARE(result.car_lamp_count, quint8(1));
    }
    QCOMPARE(runner.process(weakTrackedLamp).car_lamp_count, quint8(1));

    runner.resetTemporal();
    QImage reflection(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    reflection.fill(20);
    for (int offset = 0; offset < 20; ++offset)
    {
        for (int width = 0; width < 3; ++width)
        {
            reflection.scanLine(84 + offset)[40 + offset + width] = 255;
        }
    }
    result = runner.process(reflection);
    QCOMPARE(result.car_lamp_count, quint8(0));

    runner.resetTemporal();
    QImage hollowGlare(BEACON_IMAGE_W, BEACON_IMAGE_H,
                       QImage::Format_Grayscale8);
    hollowGlare.fill(40);
    for (int y = 70; y <= 90; ++y)
    {
        uchar* row = hollowGlare.scanLine(y);
        row[45] = 250;
        row[52] = 250;
    }
    for (int x = 45; x <= 52; ++x)
    {
        hollowGlare.scanLine(70)[x] = 250;
    }
    result = runner.process(hollowGlare);
    QCOMPARE(result.car_lamp_count, quint8(0));

    runner.resetTemporal();
    QImage elongatedGlare(BEACON_IMAGE_W, BEACON_IMAGE_H,
                          QImage::Format_Grayscale8);
    elongatedGlare.fill(60);
    for (int y = 41; y <= 61; ++y)
    {
        uchar* row = elongatedGlare.scanLine(y);
        for (int x = 154; x <= 178; ++x)
        {
            const int dx = x - 166;
            const int dy = y - 51;
            if (dx * dx * 16 + dy * dy * 49 <= 784)
            {
                row[x] = 130;
            }
            if (dx * dx + dy * dy <= 9)
            {
                row[x] = 255;
            }
        }
    }
    result = runner.process(elongatedGlare);
    QCOMPARE(result.beacon_count, quint8(0));
    QCOMPARE(result.car_lamp_count, quint8(0));

    runner.resetTemporal();
    QImage centeredLargeBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H,
                               QImage::Format_Grayscale8);
    centeredLargeBeacon.fill(0);
    for (int y = 64; y <= 72; ++y)
    {
        uchar* row = centeredLargeBeacon.scanLine(y);
        for (int x = 76; x <= 84; ++x)
        {
            const int dx = x - 80;
            const int dy = y - 68;
            if (dx * dx + dy * dy <= 16)
            {
                row[x] = 130;
            }
            if (dx * dx + dy * dy <= 9)
            {
                row[x] = 255;
            }
        }
    }
    result = runner.process(centeredLargeBeacon);
    QCOMPARE(result.beacon_count, quint8(1));
    const QPointF refinedCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::abs(refinedCenter.x() - 80.0) < 0.25);
    QVERIFY(std::abs(refinedCenter.y() - 68.0) < 0.25);

    runner.resetTemporal();
    QImage upperLargeBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H,
                            QImage::Format_Grayscale8);
    upperLargeBeacon.fill(20);
    for (int y = 34; y <= 46; ++y)
    {
        uchar* row = upperLargeBeacon.scanLine(y);
        for (int x = 74; x <= 86; ++x)
        {
            const int radiusSquared = (x - 80) * (x - 80) +
                                      (y - 40) * (y - 40);
            if (radiusSquared <= 9)
            {
                row[x] = 130;
            }
            if (radiusSquared <= 4)
            {
                row[x] = 255;
            }
        }
    }
    result = runner.process(upperLargeBeacon);
    QCOMPARE(result.beacon_count, quint8(1));
    const QPointF upperCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(upperCenter.x() - 80.0, upperCenter.y() - 40.0) < 0.25);

    runner.resetTemporal();
    QImage compactChoice(BEACON_IMAGE_W, BEACON_IMAGE_H,
                         QImage::Format_Grayscale8);
    compactChoice.fill(10);
    for (int y = 36; y <= 44; ++y)
    {
        uchar* row = compactChoice.scanLine(y);
        for (int x = 86; x <= 94; ++x)
        {
            const int radiusSquared = (x - 90) * (x - 90) +
                                      (y - 40) * (y - 40);
            if (radiusSquared <= 16)
            {
                row[x] = 100;
            }
        }
    }
    for (int y = 39; y <= 42; ++y)
    {
        uchar* row = compactChoice.scanLine(y);
        for (int x = 88; x <= 92; ++x)
        {
            row[x] = 254;
        }
    }
    for (int y = 29; y <= 31; ++y)
    {
        uchar* row = compactChoice.scanLine(y);
        for (int x = 119; x <= 121; ++x)
        {
            if ((x != 121) || (y != 31))
            {
                row[x] = 254;
            }
        }
    }
    result = runner.process(compactChoice);
    QCOMPARE(result.beacon_count, quint8(2));
    QCOMPARE(result.candidate_beacon_count, quint8(2));
    QVERIFY(result.beacons[0].radius >= result.beacons[1].radius);
    const QPointF firstCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    const QPointF secondCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[1].x, result.beacons[1].y);
    const auto matchesEither = [&](double x, double y) {
        return std::hypot(firstCenter.x() - x, firstCenter.y() - y) < 1.5
               || std::hypot(secondCenter.x() - x, secondCenter.y() - y) < 1.5;
    };
    QVERIFY(matchesEither(90.0, 40.0));
    QVERIFY(matchesEither(120.0, 30.0));
    const QImage compactOverlay = FrameRenderer::render(
        compactChoice, result, {}, 1, true);
    for (int index = 0; index < result.beacon_count; ++index)
    {
        const QPointF center = FrameRenderer::algorithmToImagePoint(
            result.beacons[index].x, result.beacons[index].y);
        QCOMPARE(compactOverlay.pixelColor(qRound(center.x()), qRound(center.y())),
                 QColor(0, 255, 80));
    }

    runner.resetTemporal();
    QImage clippedSideGlare(BEACON_IMAGE_W, BEACON_IMAGE_H,
                            QImage::Format_Grayscale8);
    clippedSideGlare.fill(20);
    for (int y = 75; y <= 105; ++y)
    {
        uchar* row = clippedSideGlare.scanLine(y);
        for (int x = 183; x < BEACON_IMAGE_W; ++x)
        {
            row[x] = 250;
        }
    }
    result = runner.process(clippedSideGlare);
    QCOMPARE(result.beacon_count, quint8(0));
    QCOMPARE(result.car_lamp_count, quint8(0));
}

void TwoBl3AlgorithmRunnerTests::selfContainedInstanceHandlesDistantBeaconAndLampCompetition()
{
    const QString imageDirectory = selfContainedImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained 2BL3 instance was not found.");
    const QString instanceBuildDirectory = QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("../../build"));
    QVERIFY(QDir().mkpath(instanceBuildDirectory));
    QTemporaryDir buildRoot(QDir(instanceBuildDirectory).absoluteFilePath(
        QStringLiteral("distant-beacon-test-XXXXXX")));
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage distantBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H,
                         QImage::Format_Grayscale8);
    distantBeacon.fill(40);
    distantBeacon.scanLine(29)[80] = 180;
    distantBeacon.scanLine(30)[79] = 180;
    distantBeacon.scanLine(30)[80] = 250;
    distantBeacon.scanLine(30)[81] = 180;
    distantBeacon.scanLine(31)[80] = 180;
    beacon_result_t result = runner.process(distantBeacon);
    QCOMPARE(result.beacon_count, quint8(1));
    QPointF center = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(center.x() - 80.0, center.y() - 30.0) < 0.5);

    runner.resetTemporal();
    QImage competition(BEACON_IMAGE_W, BEACON_IMAGE_H,
                       QImage::Format_Grayscale8);
    competition.fill(20);
    drawDiffuseCompactBeacon(&competition, 118, 26);
    drawUniformCarLamp(&competition, 94, 70);
    for (int frameIndex = 0; frameIndex < 3; ++frameIndex)
    {
        result = runner.process(competition);
        QCOMPARE(result.beacon_count, quint8(1));
    }
    for (int y = 45; y <= 53; ++y)
    {
        uchar* row = competition.scanLine(y);
        for (int x = 89; x <= 97; ++x)
        {
            const int radiusSquared = (x - 93) * (x - 93)
                                      + (y - 49) * (y - 49);
            if (radiusSquared <= 16)
            {
                row[x] = 180;
            }
            if (radiusSquared <= 4)
            {
                row[x] = 250;
            }
        }
    }
    result = runner.process(competition);
    QCOMPARE(result.beacon_count, quint8(2));
    const QPointF competitionFirst = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    const QPointF competitionSecond = FrameRenderer::algorithmToImagePoint(
        result.beacons[1].x, result.beacons[1].y);
    const auto competitionMatchesEither = [&](double x, double y) {
        return std::hypot(competitionFirst.x() - x, competitionFirst.y() - y) < 1.0
               || std::hypot(competitionSecond.x() - x, competitionSecond.y() - y) < 1.0;
    };
    QVERIFY(competitionMatchesEither(93.0, 49.0));
    QVERIFY(competitionMatchesEither(118.0, 26.0));

    runner.resetTemporal();
    QImage lampCompetition(BEACON_IMAGE_W, BEACON_IMAGE_H,
                           QImage::Format_Grayscale8);
    lampCompetition.fill(0);
    drawUniformCarLamp(&lampCompetition, 88, 66);
    for (int y = 94; y <= 113; ++y)
    {
        uchar* row = lampCompetition.scanLine(y);
        const int centerX = 40 + (y - 103) / 2;
        for (int x = centerX - 4; x <= centerX + 4; ++x)
        {
            row[x] = 250;
        }
    }
    result = runner.process(lampCompetition);
    QCOMPARE(result.car_lamp_count, quint8(1));
    const QPointF lampCenter = FrameRenderer::algorithmToImagePoint(
        result.car_lamps[0].cx, result.car_lamps[0].cy);
    QVERIFY(std::hypot(lampCenter.x() - 88.0, lampCenter.y() - 66.0) < 2.0);

    runner.resetTemporal();
    QImage lampGhost(BEACON_IMAGE_W, BEACON_IMAGE_H,
                     QImage::Format_Grayscale8);
    lampGhost.fill(0);
    drawUniformCarLamp(&lampGhost, 78, 64);
    for (int x = 61; x <= 65; ++x)
    {
        lampGhost.scanLine(96)[x] = 104;
        lampGhost.scanLine(105)[x] = 114;
    }
    result = runner.process(lampGhost);
    QCOMPARE(result.car_lamp_count, quint8(1));
    QCOMPARE(result.beacon_count, quint8(0));

    runner.resetTemporal();
    QImage topEdgeBeacon(BEACON_IMAGE_W, BEACON_IMAGE_H,
                         QImage::Format_Grayscale8);
    topEdgeBeacon.fill(0);
    drawUniformCarLamp(&topEdgeBeacon, 93, 56);
    for (int y = 2; y <= 4; ++y)
    {
        uchar* row = topEdgeBeacon.scanLine(y);
        for (int x = 96; x <= 99; ++x)
        {
            row[x] = 250;
        }
    }
    result = runner.process(topEdgeBeacon);
    QCOMPARE(result.car_lamp_count, quint8(1));
    QCOMPARE(result.beacon_count, quint8(1));
    const QPointF topEdgeCenter = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(topEdgeCenter.x() - 97.5,
                       topEdgeCenter.y() - 3.0) < 1.0);

    runner.resetTemporal();
    QImage weakSecondary(BEACON_IMAGE_W, BEACON_IMAGE_H,
                         QImage::Format_Grayscale8);
    weakSecondary.fill(0);
    for (int y = 30; y <= 32; ++y)
    {
        uchar* row = weakSecondary.scanLine(y);
        for (int x = 85; x <= 89; ++x)
        {
            row[x] = 250;
        }
    }
    for (int y = 86; y <= 87; ++y)
    {
        uchar* row = weakSecondary.scanLine(y);
        for (int x = 65; x <= 67; ++x)
        {
            row[x] = 109;
        }
    }
    result = runner.process(weakSecondary);
    QCOMPARE(result.candidate_beacon_count, quint8(2));
    QCOMPARE(result.beacon_count, quint8(1));

    runner.resetTemporal();
    for (int y = 86; y <= 87; ++y)
    {
        uchar* row = weakSecondary.scanLine(y);
        for (int x = 65; x <= 67; ++x)
        {
            row[x] = 180;
        }
    }
    result = runner.process(weakSecondary);
    QCOMPARE(result.beacon_count, quint8(2));

    runner.resetTemporal();
    for (int y = 86; y <= 87; ++y)
    {
        uchar* row = weakSecondary.scanLine(y);
        for (int x = 65; x <= 67; ++x)
        {
            row[x] = 0;
        }
    }
    for (int y = 20; y <= 21; ++y)
    {
        uchar* row = weakSecondary.scanLine(y);
        for (int x = 65; x <= 67; ++x)
        {
            row[x] = 109;
        }
    }
    result = runner.process(weakSecondary);
    QCOMPARE(result.beacon_count, quint8(2));
}

void TwoBl3AlgorithmRunnerTests::selfContainedInstanceRejectsCarLampAboveHorizon()
{
    const QString imageDirectory = selfContainedImageDirectory();
    QVERIFY2(!imageDirectory.isEmpty(), "Self-contained 2BL3 instance was not found.");
    const QString instanceBuildDirectory = QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("../../build"));
    QVERIFY(QDir().mkpath(instanceBuildDirectory));
    QTemporaryDir buildRoot(QDir(instanceBuildDirectory).absoluteFilePath(
        QStringLiteral("horizon-test-XXXXXX")));
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(imageDirectory, buildRoot.path(), &error),
             qPrintable(error));

    QImage upperLamp(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    upperLamp.fill(0);
    drawUniformCarLamp(&upperLamp, 65, 24);

    beacon_result_t result = runner.process(upperLamp);
    QCOMPARE(result.car_lamp_count, quint8(1));

    runner.resetTemporal();
    AlgorithmFrameTelemetry telemetry;
    telemetry.cameraId = 0;
    telemetry.rollDeg = -13.8125238f;
    telemetry.pitchDeg = 33.5699539f;
    telemetry.heightMm = 1212.53711f;
    telemetry.attitudeValid = true;
    telemetry.heightValid = true;
    runner.setFrameTelemetry(telemetry);
    result = runner.process(upperLamp);
    QCOMPARE(result.car_lamp_count, quint8(0));
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

void TwoBl3AlgorithmRunnerTests::keepsStablePointSourceWhenNoSafeLegacyTuningExists()
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
    for (int y = 56; y < 64; ++y)
    {
        uchar* row = frame.scanLine(y);
        for (int x = 90; x < 98; ++x)
        {
            row[x] = 220;
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
    QVERIFY(!result.recommendationFound);
    QVERIFY(result.message.contains(QStringLiteral("安全单参数方案")));
}

void TwoBl3AlgorithmRunnerTests::usesCurrentMeasurementAndPredictsTwoMissingFrames()
{
    QTemporaryDir buildRoot;
    QVERIFY(buildRoot.isValid());

    AlgorithmRunner runner;
    QString error;
    QVERIFY2(runner.loadTwoBl3Firmware(QStringLiteral(BEACON_2BL3_TEST_IMAGE_DIR),
                                       buildRoot.path(),
                                       &error),
             qPrintable(error));

    QImage primary(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    primary.fill(0);
    for (int y = 56; y < 64; ++y)
    {
        uchar* row = primary.scanLine(y);
        for (int x = 90; x < 98; ++x)
        {
            row[x] = 220;
        }
    }

    beacon_result_t result = runner.process(primary);
    QCOMPARE(result.beacon_count, quint8(1));
    QCOMPARE(result.temporal_beacon_count, quint8(0));
    const QPointF primaryPoint = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(primaryPoint.x() - 93.5, primaryPoint.y() - 59.5) < 1.0);

    QImage twoBeacons(BEACON_IMAGE_W, BEACON_IMAGE_H,
                      QImage::Format_Grayscale8);
    twoBeacons.fill(0);
    for (int y = 57; y < 61; ++y)
    {
        uchar* row = twoBeacons.scanLine(y);
        for (int x = 75; x < 79; ++x)
        {
            row[x] = 220;
        }
    }
    for (int y = 54; y < 62; ++y)
    {
        uchar* row = twoBeacons.scanLine(y);
        for (int x = 108; x < 116; ++x)
        {
            row[x] = 220;
        }
    }

    result = runner.process(twoBeacons);
    QCOMPARE(result.beacon_count, quint8(2));
    QCOMPARE(result.temporal_beacon_count, quint8(0));
    QVERIFY(result.beacons[0].radius > result.beacons[1].radius);
    const QPointF largest = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(largest.x() - 111.5, largest.y() - 57.5) < 1.0);

    QImage switched(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    switched.fill(0);
    for (int y = 30; y < 38; ++y)
    {
        uchar* row = switched.scanLine(y);
        for (int x = 130; x < 138; ++x)
        {
            row[x] = 220;
        }
    }

    result = runner.process(switched);
    QCOMPARE(result.beacon_count, quint8(1));
    const QPointF switchedPoint = FrameRenderer::algorithmToImagePoint(
        result.beacons[0].x, result.beacons[0].y);
    QVERIFY(std::hypot(switchedPoint.x() - 133.5,
                       switchedPoint.y() - 33.5) < 1.0);

    QImage blank(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    blank.fill(0);
    const beacon_result_t firstMissing = runner.process(blank);
    QCOMPARE(firstMissing.beacon_count, quint8(1));
    QCOMPARE(firstMissing.temporal_beacon_count, quint8(0));
    const beacon_result_t secondMissing = runner.process(blank);
    QCOMPARE(secondMissing.beacon_count, quint8(1));
    QCOMPARE(secondMissing.temporal_beacon_count, quint8(0));
    const beacon_result_t thirdMissing = runner.process(blank);
    QCOMPARE(thirdMissing.beacon_count, quint8(0));
    QCOMPARE(thirdMissing.temporal_beacon_count, quint8(0));
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
