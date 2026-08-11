#include "HorizonCalibration.h"
#include "HorizonCalibrationRecorder.h"
#include "HorizonLineGeometry.h"
#include "ImageFrameSidecar.h"
#include "DownGroundRangeCalibration.h"
#include "VideoReader.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineF>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <QTextStream>

#include <cmath>

namespace
{
std::array<double, HorizonModelCoefficientCount> syntheticModel()
{
    std::array<double, HorizonModelCoefficientCount> coefficients = {};
    coefficients[0 * 3 + 1] = 1.0;
    coefficients[1 * 3 + 2] = 1.0;
    coefficients[2 * 3 + 0] = 1.0;
    coefficients[3 * 3 + 1] = 0.18;
    coefficients[4 * 3 + 2] = 0.18;
    coefficients[5 * 3 + 0] = -0.30;
    coefficients[6 * 3 + 1] = 0.04;
    coefficients[7 * 3 + 2] = 0.04;
    coefficients[8 * 3 + 0] = 0.03;
    return coefficients;
}

QVector<QPointF> sampledCurve(const QVector<QPointF>& curve, int count)
{
    QVector<QPointF> points;
    if (curve.size() < count || count < 2)
    {
        return points;
    }
    for (int index = 0; index < count; ++index)
    {
        points.push_back(curve[index * (curve.size() - 1) / (count - 1)]);
    }
    return points;
}

void writeCsv(const QString& path)
{
    QFile csv(path);
    QVERIFY(csv.open(QIODevice::WriteOnly | QIODevice::Text));
    QTextStream stream(&csv);
    stream << "frame_index,bimg_sequence,host_time_ms,camera_id,roll_deg,pitch_deg,attitude_valid\n"
           << "0,10,1000,0,1.5,-2.5,1\n"
           << "1,11,1020,0,1.6,-2.4,0\n";
}

QJsonObject physicalModelJson()
{
    return {
        {QStringLiteral("format"), QStringLiteral("horizon_model")},
        {QStringLiteral("version"), 3},
        {QStringLiteral("model_type"), QStringLiteral("central_fisheye_angle_poly5")},
        {QStringLiteral("camera_id"), 0},
        {QStringLiteral("image_width"), 188},
        {QStringLiteral("image_height"), 120},
        {QStringLiteral("center_x"), 89.05679508319916},
        {QStringLiteral("center_y"), 69.5},
        {QStringLiteral("normalization_scale"), 93.5},
        {QStringLiteral("theta_coefficients"),
         QJsonArray({2.136653726640037, -1.210544244169379, 0.6350455170908825})},
        {QStringLiteral("attitude_to_camera_normal"),
         QJsonArray({QJsonArray({0.00346433916224295,
                                 -0.50925728267113,
                                 -0.0019043835510911922}),
                     QJsonArray({0.3206116243770298,
                                 0.043452357107652585,
                                 -0.2858130784381615}),
                     QJsonArray({-0.5320478058095025,
                                 0.03541420100980411,
                                 -0.5195605495364598})})},
        {QStringLiteral("inlier_count"), 116},
        {QStringLiteral("sample_count"), 117},
        {QStringLiteral("rmse_px"), 0.7204725164612968},
        {QStringLiteral("median_error_px"), 0.5234171137776917},
        {QStringLiteral("max_error_px"), 1.9595601831555325},
        {QStringLiteral("roll_min_deg"), -18.5663319},
        {QStringLiteral("roll_max_deg"), 21.1464043},
        {QStringLiteral("pitch_min_deg"), -10.6507406},
        {QStringLiteral("pitch_max_deg"), 36.6099815}
    };
}

QJsonObject heightCompensatedModelJson()
{
    QJsonObject model = physicalModelJson();
    model.insert(QStringLiteral("version"), 4);
    model.insert(QStringLiteral("model_type"),
                 QStringLiteral("central_fisheye_height_compensated"));
    model.insert(QStringLiteral("effective_distance_mm"), 7188.6);
    model.insert(QStringLiteral("height_zero_mm"), 1121.9);
    model.insert(QStringLiteral("height_min_mm"), 700.0);
    model.insert(QStringLiteral("height_max_mm"), 1250.0);
    return model;
}
}

class HorizonCalibrationTests : public QObject
{
    Q_OBJECT

private slots:
    void clipsInfiniteLine();
    void savesAndLoadsCurveSession();
    void loadsLegacyLineSession();
    void loadsVersion2Session();
    void fitsSyntheticCurvesWithOutliers();
    void rejectsTooFewSamples();
    void exportsPortableHeader();
    void loadsPhysicalModel();
    void loadsHeightCompensatedModel();
    void predictsAndReloadsDownBoundary();
    void downFitKeepsCoverageDraftNonExportable();
    void recordsPairedFrames();
    void recordsLegacyV2WithoutSyncSidecar();
};

void HorizonCalibrationTests::clipsInfiniteLine()
{
    QLineF segment;
    QVERIFY(HorizonLineGeometry::clipThroughPoints(QPointF(20.0, 30.0),
                                                   QPointF(80.0, 60.0),
                                                   QSize(188, 120),
                                                   &segment));
    QVERIFY(std::abs(segment.p1().x()) < 1e-6
            || std::abs(segment.p1().y()) < 1e-6
            || std::abs(segment.p1().x() - 187.0) < 1e-6
            || std::abs(segment.p1().y() - 119.0) < 1e-6);
    QVERIFY(std::abs(segment.p2().x()) < 1e-6
            || std::abs(segment.p2().y()) < 1e-6
            || std::abs(segment.p2().x() - 187.0) < 1e-6
            || std::abs(segment.p2().y() - 119.0) < 1e-6);
}

void HorizonCalibrationTests::savesAndLoadsCurveSession()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString base = directory.filePath(QStringLiteral("front"));
    writeCsv(base + QStringLiteral(".hcal.csv"));

    HorizonCalibrationSession session;
    session.sessionPath = base + QStringLiteral(".hcal.json");
    session.videoPath = base + QStringLiteral(".avi");
    session.csvPath = base + QStringLiteral(".hcal.csv");
    session.imageSize = QSize(188, 120);
    session.cameraId = 0;
    session.frameCount = 2;
    HorizonFrameAnnotation annotation;
    annotation.points = {QPointF(0.0, 55.0),
                         QPointF(35.0, 57.0),
                         QPointF(75.0, 60.0),
                         QPointF(120.0, 62.0),
                         QPointF(187.0, 65.0)};
    session.annotations.insert(0, annotation);
    QString error;
    QVERIFY2(HorizonCalibration::saveSession(session, &error), qPrintable(error));

    HorizonCalibrationSession loaded;
    QVERIFY2(HorizonCalibration::loadSession(session.sessionPath, &loaded, &error), qPrintable(error));
    QCOMPARE(loaded.frames.size(), 2);
    QCOMPARE(loaded.frames[0].bimgSequence, 10U);
    QVERIFY(loaded.frames[0].attitudeValid);
    QVERIFY(!loaded.frames[1].attitudeValid);
    QCOMPARE(loaded.annotations.value(0).points.size(), 5);
    QVERIFY(!loaded.annotations.value(0).legacyLine);
}

void HorizonCalibrationTests::loadsLegacyLineSession()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString base = directory.filePath(QStringLiteral("legacy"));
    writeCsv(base + QStringLiteral(".hcal.csv"));
    const QJsonObject root({
        {QStringLiteral("format"), QStringLiteral("horizon_calibration")},
        {QStringLiteral("version"), 1},
        {QStringLiteral("video_file"), QStringLiteral("legacy.avi")},
        {QStringLiteral("csv_file"), QStringLiteral("legacy.hcal.csv")},
        {QStringLiteral("image_width"), 188},
        {QStringLiteral("image_height"), 120},
        {QStringLiteral("camera_id"), 0},
        {QStringLiteral("annotations"), QJsonArray({QJsonObject({
             {QStringLiteral("frame_index"), 0},
             {QStringLiteral("skipped"), false},
             {QStringLiteral("first"), QJsonArray({0.0, 55.0})},
             {QStringLiteral("second"), QJsonArray({187.0, 65.0})}
         })})}
    });
    QFile json(base + QStringLiteral(".hcal.json"));
    QVERIFY(json.open(QIODevice::WriteOnly));
    json.write(QJsonDocument(root).toJson());
    json.close();

    HorizonCalibrationSession loaded;
    QString error;
    QVERIFY2(HorizonCalibration::loadSession(json.fileName(), &loaded, &error), qPrintable(error));
    QVERIFY(loaded.annotations.value(0).legacyLine);
    QCOMPARE(loaded.annotations.value(0).points.size(), 2);
    QVERIFY(!loaded.fit.fitted);
}

void HorizonCalibrationTests::loadsVersion2Session()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString base = directory.filePath(QStringLiteral("version2"));
    writeCsv(base + QStringLiteral(".hcal.csv"));
    const QJsonObject root({
        {QStringLiteral("format"), QStringLiteral("horizon_calibration")},
        {QStringLiteral("version"), 2},
        {QStringLiteral("video_file"), QStringLiteral("version2.avi")},
        {QStringLiteral("csv_file"), QStringLiteral("version2.hcal.csv")},
        {QStringLiteral("image_width"), 188},
        {QStringLiteral("image_height"), 120},
        {QStringLiteral("camera_id"), 0},
        {QStringLiteral("frame_count"), 2}
    });
    QFile json(base + QStringLiteral(".hcal.json"));
    QVERIFY(json.open(QIODevice::WriteOnly));
    json.write(QJsonDocument(root).toJson());
    json.close();

    HorizonCalibrationSession loaded;
    QString error;
    QVERIFY2(HorizonCalibration::loadSession(json.fileName(), &loaded, &error), qPrintable(error));
    QCOMPARE(loaded.frames.size(), 2);
    QVERIFY(!loaded.heightRecorded);
    QVERIFY(!loaded.frames[0].heightValid);
}

void HorizonCalibrationTests::fitsSyntheticCurvesWithOutliers()
{
    HorizonCalibrationSession session;
    session.imageSize = QSize(188, 120);
    session.cameraId = 0;
    const auto expected = syntheticModel();
    int frameIndex = 0;
    for (double roll : {-24.0, -12.0, 0.0, 12.0, 24.0})
    {
        for (double pitch : {-12.0, -6.0, 0.0, 6.0, 12.0})
        {
            const QVector<QPointF> curve = HorizonCalibration::predictCurve(expected,
                                                                            session.imageSize,
                                                                            roll,
                                                                            pitch);
            const QVector<QPointF> points = sampledCurve(curve, 9);
            if (points.size() < 9)
            {
                continue;
            }

            HorizonFrameMetadata frame;
            frame.frameIndex = frameIndex;
            frame.cameraId = 0;
            frame.rollDeg = roll;
            frame.pitchDeg = pitch;
            frame.attitudeValid = true;
            session.frames.push_back(frame);

            HorizonFrameAnnotation annotation;
            annotation.points = points;
            if (frameIndex < 3)
            {
                for (QPointF& point : annotation.points)
                {
                    point.setY(qBound(0.0, point.y() + 18.0, 119.0));
                }
            }
            session.annotations.insert(frameIndex, annotation);
            ++frameIndex;
        }
    }

    const HorizonFitResult fit = HorizonCalibration::fit(session);
    QVERIFY2(fit.fitted, qPrintable(fit.error));
    QVERIFY(fit.exportable);
    QVERIFY(fit.inlierFrames.size() >= 18);
    QVERIFY(fit.outlierFrames.size() >= 3);
    QVERIFY(fit.rmse < 0.1);
    QVERIFY(fit.rollMax - fit.rollMin >= 30.0);
    QVERIFY(fit.pitchMax - fit.pitchMin >= 20.0);

    const QVector<QPointF> predicted = HorizonCalibration::predictCurve(fit.coefficients,
                                                                        session.imageSize,
                                                                        7.0,
                                                                        4.0);
    QVERIFY(predicted.size() > 100);
    const QPointF middle = predicted[predicted.size() / 2];
    QVERIFY(std::abs(HorizonCalibration::evaluate(fit.coefficients,
                                                  session.imageSize,
                                                  7.0,
                                                  4.0,
                                                  middle)) < 1e-6);
}

void HorizonCalibrationTests::rejectsTooFewSamples()
{
    HorizonCalibrationSession session;
    session.imageSize = QSize(188, 120);
    for (int index = 0; index < 7; ++index)
    {
        HorizonFrameMetadata frame;
        frame.frameIndex = index;
        frame.rollDeg = index;
        frame.pitchDeg = index;
        frame.attitudeValid = true;
        session.frames.push_back(frame);
        HorizonFrameAnnotation annotation;
        for (int point = 0; point < 7; ++point)
        {
            annotation.points.push_back(QPointF(point * 30.0, 50.0 + index + point * 0.2));
        }
        session.annotations.insert(index, annotation);
    }
    const HorizonFitResult fit = HorizonCalibration::fit(session);
    QVERIFY(!fit.fitted);
    QCOMPARE(fit.sampleCount, 7);
}

void HorizonCalibrationTests::exportsPortableHeader()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    HorizonCalibrationSession session;
    session.imageSize = QSize(188, 120);
    session.cameraId = 0;
    session.fit.fitted = true;
    session.fit.exportable = true;
    session.fit.rankValid = true;
    session.fit.coefficients = syntheticModel();
    session.fit.inlierFrames.resize(12);
    QString error;
    const QString jsonPath = directory.filePath(QStringLiteral("front_horizon_model.json"));
    const QString headerPath = directory.filePath(QStringLiteral("front_horizon_model.h"));
    QVERIFY2(HorizonCalibration::exportModel(session, jsonPath, headerPath, &error), qPrintable(error));
    QFile header(headerPath);
    QVERIFY(header.open(QIODevice::ReadOnly));
    const QByteArray source = header.readAll();
    QVERIFY(source.contains("horizon_front_value"));
    QVERIFY(source.contains("horizon_front_y"));
    QVERIFY(source.contains("const float r4"));

    QFile json(jsonPath);
    QVERIFY(json.open(QIODevice::ReadOnly));
    const QJsonObject root = QJsonDocument::fromJson(json.readAll()).object();
    QCOMPARE(root.value(QStringLiteral("version")).toInt(), 2);
    QCOMPARE(root.value(QStringLiteral("model_type")).toString(),
             QStringLiteral("radial_ray_poly5"));
}

void HorizonCalibrationTests::loadsPhysicalModel()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("front_horizon_model.json"));
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write(QJsonDocument(physicalModelJson()).toJson());
    file.close();

    HorizonFisheyeModel model;
    QString error;
    QVERIFY2(HorizonCalibration::loadModel(path, &model, &error), qPrintable(error));
    QVERIFY(model.valid);
    QCOMPARE(model.imageSize, QSize(188, 120));
    QCOMPARE(model.cameraId, 0U);
    QCOMPARE(model.inlierCount, 116);

    const QVector<QPointF> curve = HorizonCalibration::predictCurve(model, 0.0, 0.0);
    QVERIFY(curve.size() > 150);
    for (int index = 0; index < curve.size(); index += 20)
    {
        QVERIFY(std::abs(HorizonCalibration::evaluate(model, 0.0, 0.0, curve[index])) < 0.01);
    }
}

void HorizonCalibrationTests::loadsHeightCompensatedModel()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("front_height_model.json"));
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write(QJsonDocument(heightCompensatedModelJson()).toJson());
    file.close();

    HorizonFisheyeModel model;
    QString error;
    QVERIFY2(HorizonCalibration::loadModel(path, &model, &error), qPrintable(error));
    QVERIFY(model.valid);
    QVERIFY(model.heightCompensated);
    QCOMPARE(model.effectiveDistanceMm, 7188.6);
    QCOMPARE(model.heightZeroMm, 1121.9);

    HorizonFisheyeModel base = model;
    base.heightCompensated = false;
    const QVector<QPointF> baseCurve = HorizonCalibration::predictCurve(base, 4.0, -3.0);
    const QVector<QPointF> zeroCurve = HorizonCalibration::predictCurve(
        model, 4.0, -3.0, model.heightZeroMm);
    QCOMPARE(zeroCurve.size(), baseCurve.size());
    for (int index = 0; index < zeroCurve.size(); index += 20)
    {
        QVERIFY(QLineF(zeroCurve[index], baseCurve[index]).length() < 1e-9);
        QVERIFY(std::abs(HorizonCalibration::evaluate(model,
                                                      4.0,
                                                      -3.0,
                                                      model.heightZeroMm,
                                                      zeroCurve[index]))
                < 0.01);
    }

    const QVector<QPointF> lowCurve = HorizonCalibration::predictCurve(model, 4.0, -3.0, 800.0);
    QVERIFY(lowCurve.size() > 100);
    const int common = qMin(lowCurve.size(), zeroCurve.size());
    double maximumShift = 0.0;
    for (int index = 0; index < common; index += 20)
    {
        maximumShift = qMax(maximumShift,
                            QLineF(lowCurve[index], zeroCurve[index]).length());
    }
    QVERIFY(maximumShift > 1.0);
}

void HorizonCalibrationTests::predictsAndReloadsDownBoundary()
{
    DownGroundRangeModel model = DownGroundRangeCalibration::defaultModel(QSize(188, 120));
    model.valid = true;
    model.inlierCount = 24;
    model.sampleCount = 26;
    model.rmse = 0.8;
    model.azimuthCoverage = 82.0;

    const QVector<QPointF> lowBoundary = DownGroundRangeCalibration::predictBoundary(
        model, 0.0, 0.0, 800.0);
    const QVector<QPointF> highBoundary = DownGroundRangeCalibration::predictBoundary(
        model, 0.0, 0.0, 1400.0);
    QCOMPARE(lowBoundary.size(), 360);
    QCOMPARE(highBoundary.size(), 360);

    double lowRadius = 0.0;
    double highRadius = 0.0;
    const QPointF center(model.centerX, model.centerY);
    for (int index = 0; index < lowBoundary.size(); ++index)
    {
        lowRadius += QLineF(center, lowBoundary[index]).length();
        highRadius += QLineF(center, highBoundary[index]).length();
    }
    QVERIFY(lowRadius > highRadius);

    DownGroundRangeModel invalid = model;
    invalid.valid = false;
    QVERIFY(DownGroundRangeCalibration::predictBoundary(invalid, 0.0, 0.0, 1000.0).isEmpty());
    QVERIFY(DownGroundRangeCalibration::predictBoundary(model, 0.0, 0.0, -1.0).isEmpty());

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString jsonPath = directory.filePath(QStringLiteral("down_model.json"));
    const QString headerPath = directory.filePath(QStringLiteral("down_model.h"));
    QString error;
    QVERIFY2(DownGroundRangeCalibration::exportModel(model, jsonPath, headerPath, &error),
             qPrintable(error));

    DownGroundRangeModel loaded;
    QVERIFY2(DownGroundRangeCalibration::loadModel(jsonPath, &loaded, &error), qPrintable(error));
    const QVector<QPointF> reloaded = DownGroundRangeCalibration::predictBoundary(
        loaded, 0.0, 0.0, 800.0);
    QCOMPARE(reloaded.size(), lowBoundary.size());
    for (int index = 0; index < reloaded.size(); index += 30)
    {
        QVERIFY(QLineF(reloaded[index], lowBoundary[index]).length() < 1e-9);
    }

    QFile header(headerPath);
    QVERIFY(header.open(QIODevice::ReadOnly));
    QVERIFY(header.readAll().contains("down_ground_range_boundary"));
}

void HorizonCalibrationTests::downFitKeepsCoverageDraftNonExportable()
{
    HorizonCalibrationSession session;
    session.imageSize = QSize(188, 120);
    session.cameraId = HorizonCameraDown;
    session.sourceCameraId = HorizonCameraFront;
    const DownGroundRangeModel model = [] {
        DownGroundRangeModel value = DownGroundRangeCalibration::defaultModel(QSize(188, 120));
        value.valid = true;
        return value;
    }();

    for (int frameIndex = 0; frameIndex < 4; ++frameIndex)
    {
        HorizonFrameMetadata frame;
        frame.frameIndex = frameIndex;
        frame.cameraId = HorizonCameraDown;
        frame.sourceCameraId = HorizonCameraFront;
        frame.rollDeg = frameIndex * 2.0;
        frame.pitchDeg = -frameIndex;
        frame.heightMm = 900.0 + frameIndex * 100.0;
        frame.attitudeValid = true;
        frame.heightValid = true;
        session.frames.push_back(frame);

        const QVector<QPointF> boundary = DownGroundRangeCalibration::predictBoundary(
            model, frame.rollDeg, frame.pitchDeg, frame.heightMm);
        QCOMPARE(boundary.size(), 360);
        HorizonFrameAnnotation annotation;
        annotation.points = {boundary[8], boundary[10], boundary[12]};
        session.annotations.insert(frameIndex, annotation);
    }

    const DownGroundRangeFitResult fit = DownGroundRangeCalibration::fit(session);
    QVERIFY2(fit.fitted, qPrintable(fit.error));
    QVERIFY(!fit.exportable);
    QVERIFY(fit.model.azimuthCoverage < 75.0);
}

void HorizonCalibrationTests::recordsPairedFrames()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString base = directory.filePath(QStringLiteral("recording"));
    HorizonCalibrationRecorder recorder;
    QSignalSpy finishedSpy(&recorder, &HorizonCalibrationRecorder::recordingFinished);
    QSignalSpy failedSpy(&recorder, &HorizonCalibrationRecorder::recordingFailed);
    HorizonCalibrationRecorderConfig config;
    config.sessionPath = base + QStringLiteral(".hcal.json");
    config.videoPath = base + QStringLiteral(".avi");
    config.csvPath = base + QStringLiteral(".hcal.csv");
    config.imageSize = QSize(188, 120);
    config.cameraId = HorizonCameraDown;
    config.sourceCameraId = HorizonCameraFront;
    config.bimgProtocolVersion = 3U;
    QString error;
    QVERIFY2(recorder.begin(config, &error), qPrintable(error));
    for (int index = 0; index < 3; ++index)
    {
        HorizonCalibrationRecorderFrame frame;
        frame.image = QImage(config.imageSize, QImage::Format_Grayscale8);
        frame.image.fill(30 + index * 20);
        frame.bimgSequence = 100U + (quint32)index;
        frame.hostTimeMs = 1000 + index * 20;
        frame.cameraId = HorizonCameraDown;
        frame.sourceCameraId = HorizonCameraFront;
        frame.bimgProtocolVersion = 3U;
        frame.sourceFrameSequence = 500U + (quint32)index;
        frame.captureTimeMs = 2000U + (quint32)index * 20U;
        frame.sourceFrameCameraId = 0U;
        frame.physicalBoardId = 0U;
        frame.sourceFrameValid = true;
        frame.captureTimeValid = index != 1;
        frame.rollDeg = index + 0.25;
        frame.pitchDeg = -index - 0.5;
        frame.heightMm = 900.0 + index * 25.0;
        frame.attitudeValid = true;
        frame.heightValid = index != 1;
        QVERIFY(recorder.enqueue(frame));
    }
    recorder.finish();
    QTRY_VERIFY_WITH_TIMEOUT(finishedSpy.count() + failedSpy.count() > 0, 20000);
    QVERIFY2(failedSpy.isEmpty(),
             failedSpy.isEmpty() ? "" : qPrintable(failedSpy.first().first().toString()));

    HorizonCalibrationSession session;
    QVERIFY2(HorizonCalibration::loadSession(config.sessionPath, &session, &error), qPrintable(error));
    QCOMPARE(session.frames.size(), 3);
    QCOMPARE(session.frames[0].bimgSequence, 100U);
    QCOMPARE(session.frames[2].bimgSequence, 102U);
    QCOMPARE(session.cameraId, HorizonCameraDown);
    QCOMPARE(session.sourceCameraId, HorizonCameraFront);
    QCOMPARE(session.bimgProtocolVersion, 3);
    QCOMPARE(session.frames[1].cameraId, HorizonCameraDown);
    QCOMPARE(session.frames[1].sourceCameraId, HorizonCameraFront);
    QCOMPARE(session.frames[1].rollDeg, 1.25);
    QVERIFY(session.heightRecorded);
    QCOMPARE(session.frames[2].heightMm, 950.0);
    QVERIFY(session.frames[0].heightValid);
    QVERIFY(!session.frames[1].heightValid);
    const HorizonFitResult heightFit = HorizonCalibration::fit(session);
    QVERIFY(!heightFit.fitted);
    QVERIFY(heightFit.error.contains(QStringLiteral("高度")));

    QFile json(config.sessionPath);
    QVERIFY(json.open(QIODevice::ReadOnly));
    const QJsonObject root = QJsonDocument::fromJson(json.readAll()).object();
    QCOMPARE(root.value(QStringLiteral("version")).toInt(), 4);
    QCOMPARE(root.value(QStringLiteral("bimg_protocol_version")).toInt(), 3);
    QVERIFY(root.value(QStringLiteral("height_recorded")).toBool());
    QCOMPARE(root.value(QStringLiteral("height_unit")).toString(), QStringLiteral("mm"));
    VideoReader reader;
    QVERIFY2(reader.open(config.videoPath, &error), qPrintable(error));
    QCOMPARE(reader.frameCount(), 3);

    QVector<ImageFrameSidecarRecord> sidecar;
    QVERIFY2(loadImageFrameSidecar(imageFrameSidecarPathForVideo(config.videoPath),
                                   &sidecar,
                                   &error),
             qPrintable(error));
    QCOMPARE(sidecar.size(), reader.frameCount());
    QCOMPARE(sidecar[0].videoFrameIndex, 0U);
    QCOMPARE(sidecar[2].bimgSequence, 102U);
    QCOMPARE(sidecar[2].sourceFrameSequence, 502U);
    QCOMPARE(sidecar[2].captureTimeMs, 2040U);
    QVERIFY(sidecar[0].sourceFrameValid);
    QVERIFY(sidecar[0].captureTimeValid);
    QVERIFY(!sidecar[1].captureTimeValid);
    QCOMPARE(sidecar[0].sourceCameraId, 0U);
    QCOMPARE(sidecar[0].physicalBoardId, 0U);
}

void HorizonCalibrationTests::recordsLegacyV2WithoutSyncSidecar()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString base = directory.filePath(QStringLiteral("legacy-v2"));
    HorizonCalibrationRecorder recorder;
    QSignalSpy finishedSpy(&recorder, &HorizonCalibrationRecorder::recordingFinished);
    QSignalSpy failedSpy(&recorder, &HorizonCalibrationRecorder::recordingFailed);
    HorizonCalibrationRecorderConfig config;
    config.sessionPath = base + QStringLiteral(".hcal.json");
    config.videoPath = base + QStringLiteral(".avi");
    config.csvPath = base + QStringLiteral(".hcal.csv");
    config.imageSize = QSize(188, 120);
    config.cameraId = HorizonCameraFront;
    config.sourceCameraId = HorizonCameraFront;
    config.bimgProtocolVersion = 2U;
    QString error;
    QVERIFY2(recorder.begin(config, &error), qPrintable(error));

    HorizonCalibrationRecorderFrame frame;
    frame.image = QImage(config.imageSize, QImage::Format_Grayscale8);
    frame.image.fill(80);
    frame.bimgSequence = 10U;
    frame.hostTimeMs = 1000;
    frame.cameraId = HorizonCameraFront;
    frame.sourceCameraId = HorizonCameraFront;
    frame.bimgProtocolVersion = 2U;
    frame.attitudeValid = true;
    frame.heightValid = true;
    frame.heightMm = 900.0;
    QVERIFY(recorder.enqueue(frame));
    recorder.finish();
    QTRY_VERIFY_WITH_TIMEOUT(finishedSpy.count() + failedSpy.count() > 0, 20000);
    QVERIFY2(failedSpy.isEmpty(),
             failedSpy.isEmpty() ? "" : qPrintable(failedSpy.first().first().toString()));
    QVERIFY(!QFileInfo::exists(imageFrameSidecarPathForVideo(config.videoPath)));

    HorizonCalibrationSession session;
    QVERIFY2(HorizonCalibration::loadSession(config.sessionPath, &session, &error),
             qPrintable(error));
    QCOMPARE(session.bimgProtocolVersion, 2);
}

QTEST_MAIN(HorizonCalibrationTests)
#include "HorizonCalibrationTests.moc"
