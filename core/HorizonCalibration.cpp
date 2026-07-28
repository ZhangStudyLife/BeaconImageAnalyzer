#include "HorizonCalibration.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QSaveFile>
#include <QStringConverter>
#include <QTextStream>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace
{
constexpr int SessionVersion = 3;
constexpr int MinimumFitFrames = 8;
constexpr int MinimumExportInliers = 12;
constexpr int MinimumPointsPerFrame = 5;
constexpr int RansacIterations = 500;
constexpr double InlierThresholdPixels = 2.0;
constexpr double Pi = 3.14159265358979323846;

struct FitSample
{
    int frameIndex = -1;
    QVector<QPointF> points;
    std::array<double, 3> gravity = {};
};

void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage != nullptr)
    {
        *errorMessage = message;
    }
}

QString relativePath(const QString& sessionPath, const QString& targetPath)
{
    return QDir(QFileInfo(sessionPath).absolutePath()).relativeFilePath(targetPath);
}

QJsonArray pointJson(const QPointF& point)
{
    return {point.x(), point.y()};
}

QJsonArray pointsJson(const QVector<QPointF>& points)
{
    QJsonArray array;
    for (const QPointF& point : points)
    {
        array.push_back(pointJson(point));
    }
    return array;
}

QPointF pointFromJson(const QJsonValue& value, bool* ok)
{
    const QJsonArray values = value.toArray();
    const bool valid = values.size() == 2
                       && values.at(0).isDouble()
                       && values.at(1).isDouble();
    if (ok != nullptr)
    {
        *ok = valid;
    }
    return valid ? QPointF(values.at(0).toDouble(), values.at(1).toDouble()) : QPointF();
}

QVector<QPointF> pointsFromJson(const QJsonValue& value)
{
    QVector<QPointF> points;
    for (const QJsonValue& pointValue : value.toArray())
    {
        bool ok = false;
        const QPointF point = pointFromJson(pointValue, &ok);
        if (!ok)
        {
            return {};
        }
        points.push_back(point);
    }
    return points;
}

QJsonArray intArray(const QVector<int>& values)
{
    QJsonArray array;
    for (int value : values)
    {
        array.push_back(value);
    }
    return array;
}

QJsonArray coefficientRows(
    const std::array<double, HorizonModelCoefficientCount>& coefficients)
{
    QJsonArray rows;
    for (int basis = 0; basis < HorizonModelBasisCount; ++basis)
    {
        rows.push_back(QJsonArray({coefficients[basis * 3],
                                   coefficients[basis * 3 + 1],
                                   coefficients[basis * 3 + 2]}));
    }
    return rows;
}

bool coefficientsFromJson(
    const QJsonValue& value,
    std::array<double, HorizonModelCoefficientCount>* coefficients)
{
    if (coefficients == nullptr)
    {
        return false;
    }
    const QJsonArray rows = value.toArray();
    if (rows.size() != HorizonModelBasisCount)
    {
        return false;
    }
    for (int basis = 0; basis < HorizonModelBasisCount; ++basis)
    {
        const QJsonArray values = rows.at(basis).toArray();
        if (values.size() != 3)
        {
            return false;
        }
        for (int axis = 0; axis < 3; ++axis)
        {
            (*coefficients)[basis * 3 + axis] = values.at(axis).toDouble();
        }
    }
    return true;
}

QJsonObject fitJson(const HorizonFitResult& fit)
{
    if (!fit.fitted)
    {
        return QJsonObject();
    }

    QJsonObject errors;
    for (auto it = fit.frameErrors.cbegin(); it != fit.frameErrors.cend(); ++it)
    {
        errors.insert(QString::number(it.key()), it.value());
    }

    return QJsonObject({
        {QStringLiteral("fitted"), true},
        {QStringLiteral("exportable"), fit.exportable},
        {QStringLiteral("rank_valid"), fit.rankValid},
        {QStringLiteral("model_type"), QStringLiteral("radial_ray_poly5")},
        {QStringLiteral("coefficients"), coefficientRows(fit.coefficients)},
        {QStringLiteral("sample_count"), fit.sampleCount},
        {QStringLiteral("inlier_frames"), intArray(fit.inlierFrames)},
        {QStringLiteral("outlier_frames"), intArray(fit.outlierFrames)},
        {QStringLiteral("frame_rmse_px"), errors},
        {QStringLiteral("rmse_px"), fit.rmse},
        {QStringLiteral("median_error_px"), fit.medianError},
        {QStringLiteral("max_error_px"), fit.maxError},
        {QStringLiteral("roll_min_deg"), fit.rollMin},
        {QStringLiteral("roll_max_deg"), fit.rollMax},
        {QStringLiteral("pitch_min_deg"), fit.pitchMin},
        {QStringLiteral("pitch_max_deg"), fit.pitchMax}
    });
}

HorizonFitResult fitFromJson(const QJsonObject& object)
{
    HorizonFitResult result;
    if (!object.value(QStringLiteral("fitted")).toBool()
        || object.value(QStringLiteral("model_type")).toString()
               != QStringLiteral("radial_ray_poly5")
        || !coefficientsFromJson(object.value(QStringLiteral("coefficients")),
                                 &result.coefficients))
    {
        return result;
    }

    result.fitted = true;
    result.exportable = object.value(QStringLiteral("exportable")).toBool();
    result.rankValid = object.value(QStringLiteral("rank_valid")).toBool();
    result.sampleCount = object.value(QStringLiteral("sample_count")).toInt();
    for (const QJsonValue& value : object.value(QStringLiteral("inlier_frames")).toArray())
    {
        result.inlierFrames.push_back(value.toInt());
    }
    for (const QJsonValue& value : object.value(QStringLiteral("outlier_frames")).toArray())
    {
        result.outlierFrames.push_back(value.toInt());
    }
    const QJsonObject frameErrors = object.value(QStringLiteral("frame_rmse_px")).toObject();
    for (auto it = frameErrors.begin(); it != frameErrors.end(); ++it)
    {
        bool ok = false;
        const int frame = it.key().toInt(&ok);
        if (ok)
        {
            result.frameErrors.insert(frame, it.value().toDouble());
        }
    }
    result.rmse = object.value(QStringLiteral("rmse_px")).toDouble();
    result.medianError = object.value(QStringLiteral("median_error_px")).toDouble();
    result.maxError = object.value(QStringLiteral("max_error_px")).toDouble();
    result.rollMin = object.value(QStringLiteral("roll_min_deg")).toDouble();
    result.rollMax = object.value(QStringLiteral("roll_max_deg")).toDouble();
    result.pitchMin = object.value(QStringLiteral("pitch_min_deg")).toDouble();
    result.pitchMax = object.value(QStringLiteral("pitch_max_deg")).toDouble();
    return result;
}

bool readCsv(const QString& path,
             quint8 cameraId,
             QVector<HorizonFrameMetadata>* frames,
             bool* heightRecorded,
             QString* errorMessage)
{
    QFile file(path);
    if (frames == nullptr || !file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        setError(errorMessage, frames == nullptr ? QStringLiteral("帧输出为空。") : file.errorString());
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    const QString legacyHeader = QStringLiteral(
        "frame_index,bimg_sequence,host_time_ms,camera_id,roll_deg,pitch_deg,attitude_valid");
    const QString heightHeader = QStringLiteral(
        "frame_index,bimg_sequence,host_time_ms,camera_id,roll_deg,pitch_deg,height_mm,attitude_valid,height_valid");
    const QString header = stream.readLine().trimmed();
    const bool hasHeight = header == heightHeader;
    if (header != legacyHeader && !hasHeight)
    {
        setError(errorMessage, QStringLiteral("HCAL CSV 表头不匹配。"));
        return false;
    }
    if (heightRecorded != nullptr)
    {
        *heightRecorded = hasHeight;
    }

    frames->clear();
    int lineNumber = 1;
    while (!stream.atEnd())
    {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty())
        {
            continue;
        }
        const QStringList values = line.split(QLatin1Char(','), Qt::KeepEmptyParts);
        if (values.size() != (hasHeight ? 9 : 7))
        {
            setError(errorMessage, QStringLiteral("HCAL CSV 第 %1 行字段数错误。").arg(lineNumber));
            return false;
        }

        bool frameOk = false;
        bool sequenceOk = false;
        bool timeOk = false;
        bool cameraOk = false;
        bool rollOk = false;
        bool pitchOk = false;
        bool heightOk = false;
        bool attitudeValidOk = false;
        bool heightValidOk = !hasHeight;
        HorizonFrameMetadata frame;
        frame.frameIndex = values[0].toInt(&frameOk);
        frame.bimgSequence = values[1].toUInt(&sequenceOk);
        frame.hostTimeMs = values[2].toLongLong(&timeOk);
        frame.cameraId = (quint8)values[3].toUInt(&cameraOk);
        frame.rollDeg = values[4].toDouble(&rollOk);
        frame.pitchDeg = values[5].toDouble(&pitchOk);
        const int attitudeValid = values[hasHeight ? 7 : 6].toInt(&attitudeValidOk);
        frame.attitudeValid = attitudeValid == 1;
        if (hasHeight)
        {
            frame.heightMm = values[6].toDouble(&heightOk);
            const int heightValid = values[8].toInt(&heightValidOk);
            frame.heightValid = heightValid == 1;
        }
        if (!frameOk || !sequenceOk || !timeOk || !cameraOk
            || !attitudeValidOk || !heightValidOk
            || frame.frameIndex != frames->size() || frame.cameraId != cameraId
            || (frame.attitudeValid
                && (!rollOk || !pitchOk
                    || !std::isfinite(frame.rollDeg)
                    || !std::isfinite(frame.pitchDeg)))
            || (frame.heightValid
                && (!heightOk || !std::isfinite(frame.heightMm))))
        {
            setError(errorMessage, QStringLiteral("HCAL CSV 第 %1 行内容无效或相机不一致。").arg(lineNumber));
            return false;
        }
        if (!rollOk)
        {
            frame.rollDeg = std::numeric_limits<double>::quiet_NaN();
        }
        if (!pitchOk)
        {
            frame.pitchDeg = std::numeric_limits<double>::quiet_NaN();
        }
        if (!heightOk)
        {
            frame.heightMm = std::numeric_limits<double>::quiet_NaN();
        }
        frames->push_back(frame);
    }
    return true;
}

std::array<double, HorizonModelBasisCount> basisValues(const QSize& imageSize,
                                                        const QPointF& point)
{
    const double centerX = (imageSize.width() - 1.0) * 0.5;
    const double centerY = (imageSize.height() - 1.0) * 0.5;
    const double scale = qMax(1.0, centerX);
    const double x = (point.x() - centerX) / scale;
    const double y = (point.y() - centerY) / scale;
    const double radius2 = x * x + y * y;
    const double radius4 = radius2 * radius2;
    return {x,
            y,
            1.0,
            x * radius2,
            y * radius2,
            radius2,
            x * radius4,
            y * radius4,
            radius4};
}

double evaluateWithGravity(
    const std::array<double, HorizonModelCoefficientCount>& coefficients,
    const QSize& imageSize,
    const std::array<double, 3>& gravity,
    const QPointF& point)
{
    const auto basis = basisValues(imageSize, point);
    double value = 0.0;
    for (int term = 0; term < HorizonModelBasisCount; ++term)
    {
        const double projection = coefficients[term * 3] * gravity[0]
                                  + coefficients[term * 3 + 1] * gravity[1]
                                  + coefficients[term * 3 + 2] * gravity[2];
        value += basis[term] * projection;
    }
    return value;
}

double evaluateFisheyeWithVector(const HorizonFisheyeModel& model,
                                 const std::array<double, 3>& attitudeVector,
                                 const QPointF& point)
{
    const double x = (point.x() - model.centerX) / model.normalizationScale;
    const double y = (point.y() - model.centerY) / model.normalizationScale;
    const double radius2 = x * x + y * y;
    const double radius = std::sqrt(radius2);
    const double theta = radius
        * (model.thetaCoefficients[0]
           + radius2 * (model.thetaCoefficients[1]
                        + radius2 * model.thetaCoefficients[2]));
    const double radial = radius > 1e-12
        ? std::sin(theta) / radius
        : model.thetaCoefficients[0];
    const std::array<double, 3> ray = {x * radial, y * radial, std::cos(theta)};
    std::array<double, 3> normal = {};
    for (int row = 0; row < 3; ++row)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            normal[row] += model.attitudeToCameraNormal[row * 3 + axis]
                           * attitudeVector[axis];
        }
    }
    return ray[0] * normal[0] + ray[1] * normal[1] + ray[2] * normal[2];
}

QVector<double> fisheyeRootsAtX(const HorizonFisheyeModel& model,
                                const std::array<double, 3>& attitudeVector,
                                int x)
{
    QVector<double> roots;
    double previous = evaluateFisheyeWithVector(model, attitudeVector, QPointF(x, 0.0));
    if (std::abs(previous) < 1e-12)
    {
        roots.push_back(0.0);
    }
    for (int y = 1; y < model.imageSize.height(); ++y)
    {
        const double value = evaluateFisheyeWithVector(model, attitudeVector, QPointF(x, y));
        if (std::isfinite(previous) && std::isfinite(value) && previous * value < 0.0)
        {
            roots.push_back(y - 1.0
                            + std::abs(previous) / (std::abs(previous) + std::abs(value)));
        }
        else if (std::abs(value) < 1e-12
                 && (roots.isEmpty() || std::abs(roots.last() - y) > 0.5))
        {
            roots.push_back((double)y);
        }
        previous = value;
    }
    return roots;
}

double closestRoot(const QVector<double>& roots, double reference)
{
    return *std::min_element(roots.cbegin(), roots.cend(), [reference](double a, double b) {
        return std::abs(a - reference) < std::abs(b - reference);
    });
}

QVector<FitSample> collectSamples(const HorizonCalibrationSession& session)
{
    QVector<FitSample> samples;
    QHash<int, HorizonFrameMetadata> metadata;
    for (const HorizonFrameMetadata& frame : session.frames)
    {
        metadata.insert(frame.frameIndex, frame);
    }
    for (auto it = session.annotations.cbegin(); it != session.annotations.cend(); ++it)
    {
        const HorizonFrameAnnotation& annotation = it.value();
        const auto frameIt = metadata.constFind(it.key());
        if (annotation.skipped || annotation.legacyLine
            || annotation.points.size() < MinimumPointsPerFrame
            || frameIt == metadata.cend() || !frameIt->attitudeValid
            || !std::isfinite(frameIt->rollDeg)
            || !std::isfinite(frameIt->pitchDeg))
        {
            continue;
        }

        FitSample sample;
        sample.frameIndex = it.key();
        sample.points = annotation.points;
        sample.gravity = HorizonCalibration::gravity(frameIt->rollDeg, frameIt->pitchDeg);
        samples.push_back(sample);
    }
    return samples;
}

bool solveModel(const QVector<FitSample>& samples,
                const QVector<int>& selected,
                const QSize& imageSize,
                std::array<double, HorizonModelCoefficientCount>* coefficients,
                bool* rankValid)
{
    if (coefficients == nullptr || rankValid == nullptr
        || selected.size() < MinimumFitFrames || imageSize.width() < 2
        || imageSize.height() < 2)
    {
        return false;
    }

    int rowCount = 0;
    for (int index : selected)
    {
        if (index < 0 || index >= samples.size())
        {
            return false;
        }
        rowCount += samples[index].points.size();
    }
    if (rowCount < HorizonModelCoefficientCount)
    {
        return false;
    }

    cv::Mat design(rowCount, HorizonModelCoefficientCount, CV_64F);
    int row = 0;
    for (int index : selected)
    {
        const FitSample& sample = samples[index];
        for (const QPointF& point : sample.points)
        {
            const auto basis = basisValues(imageSize, point);
            for (int term = 0; term < HorizonModelBasisCount; ++term)
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    design.at<double>(row, term * 3 + axis)
                        = basis[term] * sample.gravity[axis];
                }
            }
            ++row;
        }
    }

    cv::Mat singularValues;
    cv::SVD::compute(design, singularValues, cv::SVD::NO_UV);
    if (singularValues.total() < HorizonModelCoefficientCount - 1
        || singularValues.at<double>(0) <= 0.0)
    {
        *rankValid = false;
        return false;
    }
    *rankValid = singularValues.at<double>(HorizonModelCoefficientCount - 2)
                 > singularValues.at<double>(0) * 1e-9;
    if (!*rankValid)
    {
        return false;
    }

    cv::Mat solution;
    cv::SVD::solveZ(design, solution);
    if (solution.total() != HorizonModelCoefficientCount)
    {
        return false;
    }

    double norm = 0.0;
    for (int index = 0; index < HorizonModelCoefficientCount; ++index)
    {
        (*coefficients)[index] = solution.at<double>(index);
        norm += (*coefficients)[index] * (*coefficients)[index];
    }
    norm = std::sqrt(norm);
    if (!std::isfinite(norm) || norm < 1e-12)
    {
        return false;
    }
    for (double& coefficient : *coefficients)
    {
        coefficient /= norm;
    }

    const std::array<double, 3> levelGravity = {0.0, 0.0, 1.0};
    const QPointF bottomCenter((imageSize.width() - 1.0) * 0.5,
                               imageSize.height() - 1.0);
    if (evaluateWithGravity(*coefficients, imageSize, levelGravity, bottomCenter) < 0.0)
    {
        for (double& coefficient : *coefficients)
        {
            coefficient = -coefficient;
        }
    }
    return true;
}

double pointError(const FitSample& sample,
                  const std::array<double, HorizonModelCoefficientCount>& coefficients,
                  const QSize& imageSize,
                  const QPointF& point)
{
    constexpr double step = 0.25;
    const double value = evaluateWithGravity(coefficients, imageSize, sample.gravity, point);
    const double dx = (evaluateWithGravity(coefficients,
                                           imageSize,
                                           sample.gravity,
                                           point + QPointF(step, 0.0))
                       - evaluateWithGravity(coefficients,
                                             imageSize,
                                             sample.gravity,
                                             point - QPointF(step, 0.0)))
                      / (2.0 * step);
    const double dy = (evaluateWithGravity(coefficients,
                                           imageSize,
                                           sample.gravity,
                                           point + QPointF(0.0, step))
                       - evaluateWithGravity(coefficients,
                                             imageSize,
                                             sample.gravity,
                                             point - QPointF(0.0, step)))
                      / (2.0 * step);
    const double gradient = std::hypot(dx, dy);
    return std::isfinite(gradient) && gradient > 1e-12
        ? std::abs(value) / gradient
        : std::numeric_limits<double>::infinity();
}

double sampleError(const FitSample& sample,
                   const std::array<double, HorizonModelCoefficientCount>& coefficients,
                   const QSize& imageSize)
{
    double squaredError = 0.0;
    for (const QPointF& point : sample.points)
    {
        const double error = pointError(sample, coefficients, imageSize, point);
        squaredError += error * error;
    }
    return std::sqrt(squaredError / sample.points.size());
}

QVector<int> classifyInliers(
    const QVector<FitSample>& samples,
    const std::array<double, HorizonModelCoefficientCount>& coefficients,
    const QSize& imageSize,
    double* squaredErrorSum)
{
    QVector<int> inliers;
    double sum = 0.0;
    for (int index = 0; index < samples.size(); ++index)
    {
        const double error = sampleError(samples[index], coefficients, imageSize);
        if (error <= InlierThresholdPixels)
        {
            inliers.push_back(index);
            sum += error * error;
        }
    }
    if (squaredErrorSum != nullptr)
    {
        *squaredErrorSum = sum;
    }
    return inliers;
}

QString floatLiteral(double value)
{
    QString literal = QLocale::c().toString(value, 'g', 9);
    if (!literal.contains(QLatin1Char('.'))
        && !literal.contains(QLatin1Char('e'), Qt::CaseInsensitive))
    {
        literal += QStringLiteral(".0");
    }
    return literal + QLatin1Char('f');
}
}

QString HorizonCalibration::cameraName(quint8 cameraId)
{
    return cameraId == 0U ? QStringLiteral("Front")
                          : cameraId == 1U ? QStringLiteral("Back")
                                           : QStringLiteral("Unknown");
}

std::array<double, 3> HorizonCalibration::gravity(double rollDeg, double pitchDeg)
{
    const double roll = rollDeg * Pi / 180.0;
    const double pitch = pitchDeg * Pi / 180.0;
    return {-std::sin(pitch),
            std::sin(roll) * std::cos(pitch),
            std::cos(roll) * std::cos(pitch)};
}

std::array<double, 3> HorizonCalibration::levelForward(double rollDeg, double pitchDeg)
{
    const double roll = rollDeg * Pi / 180.0;
    const double pitch = pitchDeg * Pi / 180.0;
    return {std::cos(pitch),
            std::sin(roll) * std::sin(pitch),
            std::cos(roll) * std::sin(pitch)};
}

double HorizonCalibration::evaluate(
    const std::array<double, HorizonModelCoefficientCount>& coefficients,
    const QSize& imageSize,
    double rollDeg,
    double pitchDeg,
    const QPointF& point)
{
    return evaluateWithGravity(coefficients, imageSize, gravity(rollDeg, pitchDeg), point);
}

QVector<QPointF> HorizonCalibration::predictCurve(
    const std::array<double, HorizonModelCoefficientCount>& coefficients,
    const QSize& imageSize,
    double rollDeg,
    double pitchDeg)
{
    QVector<QPointF> curve;
    if (imageSize.width() <= 0 || imageSize.height() <= 1)
    {
        return curve;
    }

    const auto g = gravity(rollDeg, pitchDeg);
    double previousY = (imageSize.height() - 1.0) * 0.5;
    bool started = false;
    for (int x = 0; x < imageSize.width(); ++x)
    {
        QVector<double> roots;
        double previousValue = evaluateWithGravity(coefficients,
                                                   imageSize,
                                                   g,
                                                   QPointF(x, 0.0));
        if (std::abs(previousValue) < 1e-12)
        {
            roots.push_back(0.0);
        }
        for (int y = 1; y < imageSize.height(); ++y)
        {
            const double value = evaluateWithGravity(coefficients,
                                                     imageSize,
                                                     g,
                                                     QPointF(x, y));
            if (std::isfinite(previousValue) && std::isfinite(value)
                && previousValue * value < 0.0)
            {
                const double fraction = std::abs(previousValue)
                                        / (std::abs(previousValue) + std::abs(value));
                roots.push_back(y - 1.0 + fraction);
            }
            else if (std::abs(value) < 1e-12)
            {
                roots.push_back((double)y);
            }
            previousValue = value;
        }

        if (roots.isEmpty())
        {
            if (started)
            {
                break;
            }
            continue;
        }
        const auto closest = std::min_element(roots.cbegin(), roots.cend(), [previousY](double a, double b) {
            return std::abs(a - previousY) < std::abs(b - previousY);
        });
        previousY = *closest;
        curve.push_back(QPointF(x, previousY));
        started = true;
    }
    return curve;
}

double HorizonCalibration::evaluate(const HorizonFisheyeModel& model,
                                    double rollDeg,
                                    double pitchDeg,
                                    const QPointF& point)
{
    if (!model.valid || model.normalizationScale <= 0.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double heightMm = model.heightCompensated ? model.heightZeroMm : 0.0;
    return evaluate(model, rollDeg, pitchDeg, heightMm, point);
}

QVector<QPointF> HorizonCalibration::predictCurve(const HorizonFisheyeModel& model,
                                                  double rollDeg,
                                                  double pitchDeg)
{
    const double heightMm = model.heightCompensated ? model.heightZeroMm : 0.0;
    return predictCurve(model, rollDeg, pitchDeg, heightMm);
}

double HorizonCalibration::evaluate(const HorizonFisheyeModel& model,
                                    double rollDeg,
                                    double pitchDeg,
                                    double heightMm,
                                    const QPointF& point)
{
    if (!model.valid || model.normalizationScale <= 0.0
        || (model.heightCompensated
            && (!std::isfinite(heightMm) || model.effectiveDistanceMm <= 0.0)))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    std::array<double, 3> direction = gravity(rollDeg, pitchDeg);
    if (model.heightCompensated)
    {
        const auto forward = levelForward(rollDeg, pitchDeg);
        const double compensation = (model.heightZeroMm - heightMm)
                                    / model.effectiveDistanceMm;
        for (int axis = 0; axis < 3; ++axis)
        {
            direction[axis] += compensation * forward[axis];
        }
    }
    return evaluateFisheyeWithVector(model, direction, point);
}

QVector<QPointF> HorizonCalibration::predictCurve(const HorizonFisheyeModel& model,
                                                  double rollDeg,
                                                  double pitchDeg,
                                                  double heightMm)
{
    QVector<QPointF> curve;
    if (!model.valid || model.imageSize.width() <= 0 || model.imageSize.height() <= 1)
    {
        return curve;
    }

    std::array<double, 3> direction = gravity(rollDeg, pitchDeg);
    if (model.heightCompensated)
    {
        if (!std::isfinite(heightMm) || model.effectiveDistanceMm <= 0.0)
        {
            return curve;
        }
        const auto forward = levelForward(rollDeg, pitchDeg);
        const double compensation = (model.heightZeroMm - heightMm)
                                    / model.effectiveDistanceMm;
        for (int axis = 0; axis < 3; ++axis)
        {
            direction[axis] += compensation * forward[axis];
        }
    }
    QVector<QVector<double>> roots(model.imageSize.width());
    for (int x = 0; x < roots.size(); ++x)
    {
        roots[x] = fisheyeRootsAtX(model, direction, x);
    }

    const int centerColumn = qBound(0, qRound(model.centerX), roots.size() - 1);
    int seedColumn = -1;
    for (int offset = 0; offset < roots.size() && seedColumn < 0; ++offset)
    {
        const int left = centerColumn - offset;
        const int right = centerColumn + offset;
        if (left >= 0 && !roots[left].isEmpty())
        {
            seedColumn = left;
        }
        else if (right < roots.size() && !roots[right].isEmpty())
        {
            seedColumn = right;
        }
    }
    if (seedColumn < 0)
    {
        return curve;
    }

    const double seedY = closestRoot(roots[seedColumn], model.centerY);
    QVector<QPointF> leftCurve;
    double previousY = seedY;
    for (int x = seedColumn - 1; x >= 0; --x)
    {
        if (roots[x].isEmpty())
        {
            break;
        }
        previousY = closestRoot(roots[x], previousY);
        leftCurve.push_back(QPointF(x, previousY));
    }
    std::reverse(leftCurve.begin(), leftCurve.end());
    curve = leftCurve;
    curve.push_back(QPointF(seedColumn, seedY));

    previousY = seedY;
    for (int x = seedColumn + 1; x < roots.size(); ++x)
    {
        if (roots[x].isEmpty())
        {
            break;
        }
        previousY = closestRoot(roots[x], previousY);
        curve.push_back(QPointF(x, previousY));
    }
    return curve;
}

bool HorizonCalibration::loadModel(const QString& path,
                                   HorizonFisheyeModel* model,
                                   QString* errorMessage)
{
    if (model == nullptr)
    {
        setError(errorMessage, QStringLiteral("模型输出为空。"));
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        setError(errorMessage, QStringLiteral("模型 JSON 解析失败：%1").arg(parseError.errorString()));
        return false;
    }

    const QJsonObject root = document.object();
    const int version = root.value(QStringLiteral("version")).toInt();
    const QString modelType = root.value(QStringLiteral("model_type")).toString();
    const bool baseModel = version == 3
                           && modelType == QStringLiteral("central_fisheye_angle_poly5");
    const bool heightModel = version == 4
                             && modelType
                                    == QStringLiteral("central_fisheye_height_compensated");
    if (root.value(QStringLiteral("format")).toString() != QStringLiteral("horizon_model")
        || (!baseModel && !heightModel))
    {
        setError(errorMessage, QStringLiteral("不是受支持的中心鱼眼地平线模型。"));
        return false;
    }

    HorizonFisheyeModel loaded;
    loaded.sourcePath = QFileInfo(path).absoluteFilePath();
    loaded.imageSize = QSize(root.value(QStringLiteral("image_width")).toInt(),
                             root.value(QStringLiteral("image_height")).toInt());
    loaded.cameraId = (quint8)root.value(QStringLiteral("camera_id")).toInt(255);
    loaded.centerX = root.value(QStringLiteral("center_x")).toDouble();
    loaded.centerY = root.value(QStringLiteral("center_y")).toDouble();
    loaded.normalizationScale = root.value(QStringLiteral("normalization_scale")).toDouble();

    const QJsonArray theta = root.value(QStringLiteral("theta_coefficients")).toArray();
    const QJsonArray matrix = root.value(QStringLiteral("attitude_to_camera_normal")).toArray();
    bool parametersValid = theta.size() == HorizonFisheyeThetaCoefficientCount
                           && matrix.size() == 3;
    for (int index = 0; parametersValid && index < theta.size(); ++index)
    {
        loaded.thetaCoefficients[index] = theta.at(index).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        parametersValid = std::isfinite(loaded.thetaCoefficients[index]);
    }
    for (int row = 0; parametersValid && row < matrix.size(); ++row)
    {
        const QJsonArray values = matrix.at(row).toArray();
        parametersValid = values.size() == 3;
        for (int axis = 0; parametersValid && axis < values.size(); ++axis)
        {
            loaded.attitudeToCameraNormal[row * 3 + axis] = values.at(axis).toDouble(
                std::numeric_limits<double>::quiet_NaN());
            parametersValid = std::isfinite(loaded.attitudeToCameraNormal[row * 3 + axis]);
        }
    }
    if (loaded.imageSize.width() <= 0 || loaded.imageSize.height() <= 1
        || loaded.cameraId > 1U || !std::isfinite(loaded.centerX)
        || !std::isfinite(loaded.centerY) || !std::isfinite(loaded.normalizationScale)
        || loaded.normalizationScale <= 0.0 || !parametersValid)
    {
        setError(errorMessage, QStringLiteral("中心鱼眼模型字段无效。"));
        return false;
    }

    loaded.inlierCount = root.value(QStringLiteral("inlier_count")).toInt();
    loaded.sampleCount = root.value(QStringLiteral("sample_count")).toInt();
    loaded.rmse = root.value(QStringLiteral("rmse_px")).toDouble();
    loaded.medianError = root.value(QStringLiteral("median_error_px")).toDouble();
    loaded.maxError = root.value(QStringLiteral("max_error_px")).toDouble();
    loaded.rollMin = root.value(QStringLiteral("roll_min_deg")).toDouble();
    loaded.rollMax = root.value(QStringLiteral("roll_max_deg")).toDouble();
    loaded.pitchMin = root.value(QStringLiteral("pitch_min_deg")).toDouble();
    loaded.pitchMax = root.value(QStringLiteral("pitch_max_deg")).toDouble();
    loaded.heightCompensated = heightModel;
    if (heightModel)
    {
        loaded.effectiveDistanceMm = root.value(QStringLiteral("effective_distance_mm"))
                                         .toDouble(std::numeric_limits<double>::quiet_NaN());
        loaded.heightZeroMm = root.value(QStringLiteral("height_zero_mm"))
                                  .toDouble(std::numeric_limits<double>::quiet_NaN());
        loaded.heightMinMm = root.value(QStringLiteral("height_min_mm"))
                                 .toDouble(std::numeric_limits<double>::quiet_NaN());
        loaded.heightMaxMm = root.value(QStringLiteral("height_max_mm"))
                                 .toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(loaded.effectiveDistanceMm)
            || loaded.effectiveDistanceMm <= 0.0 || !std::isfinite(loaded.heightZeroMm)
            || !std::isfinite(loaded.heightMinMm) || !std::isfinite(loaded.heightMaxMm)
            || loaded.heightMinMm > loaded.heightMaxMm)
        {
            setError(errorMessage, QStringLiteral("高度补偿模型字段无效。"));
            return false;
        }
    }
    loaded.valid = true;
    *model = loaded;
    return true;
}

bool HorizonCalibration::loadSession(const QString& path,
                                     HorizonCalibrationSession* session,
                                     QString* errorMessage)
{
    if (session == nullptr)
    {
        setError(errorMessage, QStringLiteral("会话输出为空。"));
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        setError(errorMessage, QStringLiteral("HCAL JSON 解析失败：%1").arg(parseError.errorString()));
        return false;
    }

    const QJsonObject root = document.object();
    const int version = root.value(QStringLiteral("version")).toInt();
    if (root.value(QStringLiteral("format")).toString() != QStringLiteral("horizon_calibration")
        || version < 1 || version > SessionVersion)
    {
        setError(errorMessage, QStringLiteral("不是受支持的地平线标定会话。"));
        return false;
    }

    HorizonCalibrationSession loaded;
    loaded.sessionPath = QFileInfo(path).absoluteFilePath();
    const QDir directory(QFileInfo(path).absolutePath());
    loaded.videoPath = directory.absoluteFilePath(root.value(QStringLiteral("video_file")).toString());
    loaded.csvPath = directory.absoluteFilePath(root.value(QStringLiteral("csv_file")).toString());
    loaded.status = root.value(QStringLiteral("status")).toString(QStringLiteral("complete"));
    loaded.error = root.value(QStringLiteral("error")).toString();
    loaded.imageSize = QSize(root.value(QStringLiteral("image_width")).toInt(),
                             root.value(QStringLiteral("image_height")).toInt());
    loaded.cameraId = (quint8)root.value(QStringLiteral("camera_id")).toInt();
    loaded.bimgProtocolVersion = root.value(QStringLiteral("bimg_protocol_version")).toInt();
    loaded.frameCount = root.value(QStringLiteral("frame_count")).toInt();
    loaded.sourceDroppedFrames = root.value(QStringLiteral("source_dropped_frames")).toVariant().toULongLong();
    loaded.queueDroppedFrames = root.value(QStringLiteral("queue_dropped_frames")).toVariant().toULongLong();
    if (loaded.imageSize.width() <= 0 || loaded.imageSize.height() <= 0 || loaded.cameraId > 1U
        || loaded.videoPath.isEmpty() || loaded.csvPath.isEmpty())
    {
        setError(errorMessage, QStringLiteral("HCAL JSON 会话字段无效。"));
        return false;
    }
    if (!readCsv(loaded.csvPath,
                 loaded.cameraId,
                 &loaded.frames,
                 &loaded.heightRecorded,
                 errorMessage))
    {
        return false;
    }

    for (const QJsonValue& value : root.value(QStringLiteral("annotations")).toArray())
    {
        const QJsonObject object = value.toObject();
        const int frameIndex = object.value(QStringLiteral("frame_index")).toInt(-1);
        if (frameIndex < 0)
        {
            continue;
        }
        HorizonFrameAnnotation annotation;
        annotation.skipped = object.value(QStringLiteral("skipped")).toBool();
        annotation.points = pointsFromJson(object.value(QStringLiteral("points")));
        annotation.legacyLine = object.value(QStringLiteral("annotation_type")).toString()
                                    == QStringLiteral("legacy_line");
        if (annotation.points.isEmpty() && !annotation.skipped)
        {
            bool firstOk = false;
            bool secondOk = false;
            const QPointF first = pointFromJson(object.value(QStringLiteral("first")), &firstOk);
            const QPointF second = pointFromJson(object.value(QStringLiteral("second")), &secondOk);
            if (firstOk && secondOk)
            {
                annotation.points = {first, second};
                annotation.legacyLine = true;
            }
        }
        if (annotation.points.size() >= 2 || annotation.skipped)
        {
            loaded.annotations.insert(frameIndex, annotation);
        }
    }
    if (version >= 2 && !loaded.heightRecorded)
    {
        loaded.fit = fitFromJson(root.value(QStringLiteral("fit")).toObject());
    }
    *session = loaded;
    return true;
}

bool HorizonCalibration::saveSession(const HorizonCalibrationSession& session,
                                     QString* errorMessage)
{
    if (session.sessionPath.isEmpty() || session.videoPath.isEmpty() || session.csvPath.isEmpty())
    {
        setError(errorMessage, QStringLiteral("HCAL 会话路径不完整。"));
        return false;
    }

    QJsonArray annotations;
    QList<int> frames = session.annotations.keys();
    std::sort(frames.begin(), frames.end());
    for (int frameIndex : frames)
    {
        const HorizonFrameAnnotation annotation = session.annotations.value(frameIndex);
        QJsonObject object({
            {QStringLiteral("frame_index"), frameIndex},
            {QStringLiteral("skipped"), annotation.skipped}
        });
        if (annotation.points.size() >= 2 && !annotation.skipped)
        {
            object.insert(QStringLiteral("annotation_type"),
                          annotation.legacyLine ? QStringLiteral("legacy_line")
                                                : QStringLiteral("curve_points"));
            object.insert(QStringLiteral("points"), pointsJson(annotation.points));
        }
        annotations.push_back(object);
    }

    const quint64 droppedFrames = session.sourceDroppedFrames + session.queueDroppedFrames;
    QJsonObject root({
        {QStringLiteral("format"), QStringLiteral("horizon_calibration")},
        {QStringLiteral("version"), SessionVersion},
        {QStringLiteral("bimg_protocol_version"), session.bimgProtocolVersion},
        {QStringLiteral("status"), session.status},
        {QStringLiteral("error"), session.error},
        {QStringLiteral("video_file"), relativePath(session.sessionPath, session.videoPath)},
        {QStringLiteral("csv_file"), relativePath(session.sessionPath, session.csvPath)},
        {QStringLiteral("image_width"), session.imageSize.width()},
        {QStringLiteral("image_height"), session.imageSize.height()},
        {QStringLiteral("camera_id"), session.cameraId},
        {QStringLiteral("camera_name"), cameraName(session.cameraId)},
        {QStringLiteral("height_recorded"), session.heightRecorded},
        {QStringLiteral("height_unit"), session.heightRecorded ? QStringLiteral("mm") : QString()},
        {QStringLiteral("frame_count"), session.frameCount},
        {QStringLiteral("dropped_frames"), (double)droppedFrames},
        {QStringLiteral("source_dropped_frames"), (double)session.sourceDroppedFrames},
        {QStringLiteral("queue_dropped_frames"), (double)session.queueDroppedFrames},
        {QStringLiteral("annotations"), annotations},
        {QStringLiteral("fit"), session.heightRecorded ? QJsonObject() : fitJson(session.fit)}
    });

    QSaveFile file(session.sessionPath);
    if (!file.open(QIODevice::WriteOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!file.commit())
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    return true;
}

HorizonFitResult HorizonCalibration::fit(const HorizonCalibrationSession& session)
{
    HorizonFitResult result;
    if (session.heightRecorded)
    {
        result.error = QStringLiteral("该会话包含高度数据，请完成高度模型分析后再拟合，当前不会导出旧Roll/Pitch模型。");
        return result;
    }
    const QVector<FitSample> samples = collectSamples(session);
    result.sampleCount = samples.size();
    if (samples.size() < MinimumFitFrames)
    {
        result.error = QStringLiteral("至少需要 8 个不同姿态的有效曲线标注，每帧至少 5 个点，当前为 %1 帧。")
                           .arg(samples.size());
        return result;
    }

    std::mt19937 generator(0x4843414cU);
    QVector<int> allIndices;
    allIndices.reserve(samples.size());
    for (int index = 0; index < samples.size(); ++index)
    {
        allIndices.push_back(index);
    }

    QVector<int> bestInliers;
    double bestSquaredError = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < RansacIterations; ++iteration)
    {
        QVector<int> selected = allIndices;
        if (selected.size() > MinimumFitFrames)
        {
            std::shuffle(selected.begin(), selected.end(), generator);
            selected.resize(MinimumFitFrames);
        }
        std::array<double, HorizonModelCoefficientCount> candidate = {};
        bool rankValid = false;
        if (!solveModel(samples, selected, session.imageSize, &candidate, &rankValid))
        {
            continue;
        }
        double squaredError = 0.0;
        const QVector<int> inliers = classifyInliers(samples,
                                                      candidate,
                                                      session.imageSize,
                                                      &squaredError);
        if (inliers.size() > bestInliers.size()
            || (inliers.size() == bestInliers.size() && squaredError < bestSquaredError))
        {
            bestInliers = inliers;
            bestSquaredError = squaredError;
        }
    }
    if (bestInliers.size() < MinimumFitFrames)
    {
        result.error = QStringLiteral("RANSAC 未找到至少 8 个一致曲线标注，请检查姿态覆盖和栏杆标注。 ");
        return result;
    }

    bool rankValid = false;
    if (!solveModel(samples,
                    bestInliers,
                    session.imageSize,
                    &result.coefficients,
                    &rankValid))
    {
        result.error = QStringLiteral("曲线标定约束退化，无法求解有效模型。 ");
        return result;
    }
    result.rankValid = rankValid;

    QVector<int> finalInliers = classifyInliers(samples,
                                                result.coefficients,
                                                session.imageSize,
                                                nullptr);
    if (finalInliers.size() >= MinimumFitFrames && finalInliers != bestInliers)
    {
        if (!solveModel(samples,
                        finalInliers,
                        session.imageSize,
                        &result.coefficients,
                        &rankValid))
        {
            result.error = QStringLiteral("内点重拟合失败。 ");
            return result;
        }
        result.rankValid = rankValid;
    }

    QVector<double> errors;
    double squaredError = 0.0;
    result.rollMin = std::numeric_limits<double>::infinity();
    result.rollMax = -std::numeric_limits<double>::infinity();
    result.pitchMin = std::numeric_limits<double>::infinity();
    result.pitchMax = -std::numeric_limits<double>::infinity();
    QHash<int, HorizonFrameMetadata> metadata;
    for (const HorizonFrameMetadata& frame : session.frames)
    {
        metadata.insert(frame.frameIndex, frame);
    }
    for (const FitSample& sample : samples)
    {
        const double error = sampleError(sample, result.coefficients, session.imageSize);
        result.frameErrors.insert(sample.frameIndex, error);
        if (error <= InlierThresholdPixels)
        {
            result.inlierFrames.push_back(sample.frameIndex);
            errors.push_back(error);
            squaredError += error * error;
            const HorizonFrameMetadata frame = metadata.value(sample.frameIndex);
            result.rollMin = qMin(result.rollMin, frame.rollDeg);
            result.rollMax = qMax(result.rollMax, frame.rollDeg);
            result.pitchMin = qMin(result.pitchMin, frame.pitchDeg);
            result.pitchMax = qMax(result.pitchMax, frame.pitchDeg);
        }
        else
        {
            result.outlierFrames.push_back(sample.frameIndex);
        }
    }
    if (errors.isEmpty())
    {
        result.error = QStringLiteral("重拟合后没有有效内点。 ");
        return result;
    }

    std::sort(result.inlierFrames.begin(), result.inlierFrames.end());
    std::sort(result.outlierFrames.begin(), result.outlierFrames.end());
    std::sort(errors.begin(), errors.end());
    result.rmse = std::sqrt(squaredError / errors.size());
    result.medianError = errors.size() % 2 == 0
        ? (errors[errors.size() / 2 - 1] + errors[errors.size() / 2]) * 0.5
        : errors[errors.size() / 2];
    result.maxError = errors.last();
    result.fitted = true;
    result.exportable = result.rankValid && result.inlierFrames.size() >= MinimumExportInliers;
    return result;
}

bool HorizonCalibration::exportModel(const HorizonCalibrationSession& session,
                                     const QString& jsonPath,
                                     const QString& headerPath,
                                     QString* errorMessage)
{
    if (session.heightRecorded)
    {
        setError(errorMessage,
                 QStringLiteral("高度标定会话不能导出旧Roll/Pitch模型，请先完成高度模型分析。"));
        return false;
    }
    if (!session.fit.fitted || !session.fit.exportable)
    {
        setError(errorMessage, QStringLiteral("模型尚未达到导出条件：至少需要 12 个内点且矩阵有效。"));
        return false;
    }

    const QJsonObject root({
        {QStringLiteral("format"), QStringLiteral("horizon_model")},
        {QStringLiteral("version"), 2},
        {QStringLiteral("model_type"), QStringLiteral("radial_ray_poly5")},
        {QStringLiteral("camera_id"), session.cameraId},
        {QStringLiteral("camera_name"), cameraName(session.cameraId)},
        {QStringLiteral("image_width"), session.imageSize.width()},
        {QStringLiteral("image_height"), session.imageSize.height()},
        {QStringLiteral("coordinate_system"), QStringLiteral("Air FRD, degrees, image pixels")},
        {QStringLiteral("basis"), QStringLiteral("x,y,1,x*r2,y*r2,r2,x*r4,y*r4,r4")},
        {QStringLiteral("coefficients"), coefficientRows(session.fit.coefficients)},
        {QStringLiteral("inlier_count"), session.fit.inlierFrames.size()},
        {QStringLiteral("sample_count"), session.fit.sampleCount},
        {QStringLiteral("rmse_px"), session.fit.rmse},
        {QStringLiteral("median_error_px"), session.fit.medianError},
        {QStringLiteral("max_error_px"), session.fit.maxError},
        {QStringLiteral("roll_min_deg"), session.fit.rollMin},
        {QStringLiteral("roll_max_deg"), session.fit.rollMax},
        {QStringLiteral("pitch_min_deg"), session.fit.pitchMin},
        {QStringLiteral("pitch_max_deg"), session.fit.pitchMax}
    });

    QSaveFile jsonFile(jsonPath);
    if (!jsonFile.open(QIODevice::WriteOnly))
    {
        setError(errorMessage, jsonFile.errorString());
        return false;
    }
    jsonFile.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!jsonFile.commit())
    {
        setError(errorMessage, jsonFile.errorString());
        return false;
    }

    const QString camera = session.cameraId == 0U ? QStringLiteral("front") : QStringLiteral("back");
    const QString guard = QStringLiteral("HORIZON_%1_MODEL_H").arg(camera.toUpper());
    const double centerX = (session.imageSize.width() - 1.0) * 0.5;
    const double centerY = (session.imageSize.height() - 1.0) * 0.5;
    const double scale = qMax(1.0, centerX);
    QString header;
    QTextStream stream(&header);
    stream << "#ifndef " << guard << "\n#define " << guard << "\n\n#include <math.h>\n\n";
    stream << "static const float g_horizon_" << camera << "_coefficients[27] = {\n";
    for (int term = 0; term < HorizonModelBasisCount; ++term)
    {
        stream << "    " << floatLiteral(session.fit.coefficients[term * 3]) << ", "
               << floatLiteral(session.fit.coefficients[term * 3 + 1]) << ", "
               << floatLiteral(session.fit.coefficients[term * 3 + 2])
               << (term == HorizonModelBasisCount - 1 ? "\n" : ",\n");
    }
    stream << "};\n\n";
    stream << "static inline float horizon_" << camera
           << "_value(float x, float y, float roll_deg, float pitch_deg)\n{\n";
    stream << "    const float rad = 0.01745329251994329577f;\n"
           << "    const float roll = roll_deg * rad;\n"
           << "    const float pitch = pitch_deg * rad;\n"
           << "    const float gx = -sinf(pitch);\n"
           << "    const float gy = sinf(roll) * cosf(pitch);\n"
           << "    const float gz = cosf(roll) * cosf(pitch);\n"
           << "    const float xn = (x - " << floatLiteral(centerX) << ") / "
           << floatLiteral(scale) << ";\n"
           << "    const float yn = (y - " << floatLiteral(centerY) << ") / "
           << floatLiteral(scale) << ";\n"
           << "    const float r2 = xn * xn + yn * yn;\n"
           << "    const float r4 = r2 * r2;\n"
           << "    const float b[9] = {xn, yn, 1.0f, xn * r2, yn * r2, r2, xn * r4, yn * r4, r4};\n"
           << "    float value = 0.0f;\n"
           << "    for (int i = 0; i < 9; ++i)\n"
           << "    {\n"
           << "        const float *c = &g_horizon_" << camera << "_coefficients[i * 3];\n"
           << "        value += b[i] * (c[0] * gx + c[1] * gy + c[2] * gz);\n"
           << "    }\n"
           << "    return value;\n"
           << "}\n\n";
    stream << "static inline int horizon_" << camera
           << "_y(float x, float roll_deg, float pitch_deg, float *result_y)\n{\n"
           << "    float previous = horizon_" << camera << "_value(x, 0.0f, roll_deg, pitch_deg);\n"
           << "    for (int y = 1; y < " << session.imageSize.height() << "; ++y)\n"
           << "    {\n"
           << "        const float current = horizon_" << camera
           << "_value(x, (float)y, roll_deg, pitch_deg);\n"
           << "        if ((previous <= 0.0f && current >= 0.0f) || (previous >= 0.0f && current <= 0.0f))\n"
           << "        {\n"
           << "            const float sum = fabsf(previous) + fabsf(current);\n"
           << "            *result_y = (float)(y - 1) + (sum > 0.0f ? fabsf(previous) / sum : 0.0f);\n"
           << "            return 1;\n"
           << "        }\n"
           << "        previous = current;\n"
           << "    }\n"
           << "    return 0;\n"
           << "}\n\n#endif\n";

    QSaveFile headerFile(headerPath);
    if (!headerFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        setError(errorMessage, headerFile.errorString());
        return false;
    }
    headerFile.write(header.toUtf8());
    if (!headerFile.commit())
    {
        setError(errorMessage, headerFile.errorString());
        return false;
    }
    return true;
}
