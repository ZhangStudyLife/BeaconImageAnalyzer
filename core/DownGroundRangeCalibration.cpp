#include "DownGroundRangeCalibration.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineF>
#include <QLocale>
#include <QSaveFile>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr int BoundarySamples = 360;
constexpr int RansacIterations = 500;
constexpr int MinimumInlierFrames = 20;
constexpr double InlierThresholdPixels = 2.0;

using Vec3 = std::array<double, 3>;

struct FitFrame
{
    int frameIndex = -1;
    QVector<QPointF> points;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double heightMm = 0.0;
};

void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage != nullptr)
    {
        *errorMessage = message;
    }
}

double clampValue(double value, double low, double high)
{
    return qBound(low, value, high);
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

double length(const Vec3& value)
{
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

Vec3 normalized(const Vec3& value)
{
    const double valueLength = length(value);
    return valueLength > 1e-12 ? Vec3{value[0] / valueLength,
                                      value[1] / valueLength,
                                      value[2] / valueLength}
                              : Vec3{};
}

Vec3 matrixVector(const std::array<double, 9>& matrix, const Vec3& vector)
{
    Vec3 result = {};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            result[row] += matrix[row * 3 + column] * vector[column];
        }
    }
    return result;
}

std::array<double, 9> multiply(const std::array<double, 9>& a,
                                const std::array<double, 9>& b)
{
    std::array<double, 9> result = {};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            for (int index = 0; index < 3; ++index)
            {
                result[row * 3 + column] += a[row * 3 + index] * b[index * 3 + column];
            }
        }
    }
    return result;
}

std::array<double, 9> rotationMatrix(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr};
}

std::array<double, 9> baseBodyToCamera()
{
    return {0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0,
            0.0, 0.0, 1.0};
}

double thetaForRadius(const DownGroundRangeModel& model, double radius)
{
    const double radius2 = radius * radius;
    return radius * (model.thetaCoefficients[0]
                     + radius2 * (model.thetaCoefficients[1]
                                  + radius2 * model.thetaCoefficients[2]));
}

double thetaDerivative(const DownGroundRangeModel& model, double radius)
{
    const double radius2 = radius * radius;
    return model.thetaCoefficients[0] + radius2
        * (3.0 * model.thetaCoefficients[1] + 5.0 * model.thetaCoefficients[2] * radius2);
}

double maxNormalizedRadius(const DownGroundRangeModel& model)
{
    const double left = model.centerX / model.normalizationScale;
    const double right = (model.imageSize.width() - 1.0 - model.centerX) / model.normalizationScale;
    const double top = model.centerY / model.normalizationScale;
    const double bottom = (model.imageSize.height() - 1.0 - model.centerY) / model.normalizationScale;
    return std::sqrt(qMax(left * left, right * right) + qMax(top * top, bottom * bottom));
}

bool modelParametersValid(const DownGroundRangeModel& model)
{
    if (model.imageSize.width() < 2 || model.imageSize.height() < 2
        || !std::isfinite(model.centerX) || !std::isfinite(model.centerY)
        || !std::isfinite(model.normalizationScale) || model.normalizationScale <= 0.0
        || !std::isfinite(model.effectiveRangeMm) || model.effectiveRangeMm < 6500.0
        || model.effectiveRangeMm > 8000.0 || !std::isfinite(model.heightBiasMm)
        || model.heightBiasMm < -500.0 || model.heightBiasMm > 500.0)
    {
        return false;
    }
    for (double value : model.bodyToCamera)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    const double maxRadius = maxNormalizedRadius(model);
    for (int index = 0; index <= 16; ++index)
    {
        if (thetaDerivative(model, maxRadius * index / 16.0) < 0.05)
        {
            return false;
        }
    }
    return true;
}

bool projectRay(const DownGroundRangeModel& model, const Vec3& bodyRay, QPointF* point)
{
    if (point == nullptr)
    {
        return false;
    }
    const Vec3 cameraRay = normalized(matrixVector(model.bodyToCamera, bodyRay));
    if (length(cameraRay) < 1e-12)
    {
        return false;
    }
    const double theta = std::acos(clampValue(cameraRay[2], -1.0, 1.0));
    const double maxRadius = maxNormalizedRadius(model);
    const double maxTheta = thetaForRadius(model, maxRadius);
    double radius = 0.0;
    if (theta > maxTheta)
    {
        const double edgeSlope = qMax(0.05, thetaDerivative(model, maxRadius));
        radius = maxRadius + (theta - maxTheta) / edgeSlope;
    }
    else
    {
        double low = 0.0;
        double high = maxRadius;
        for (int iteration = 0; iteration < 32; ++iteration)
        {
            const double mid = (low + high) * 0.5;
            if (thetaForRadius(model, mid) < theta)
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }
        radius = (low + high) * 0.5;
    }
    const double lateral = std::hypot(cameraRay[0], cameraRay[1]);
    const double scale = lateral > 1e-12 ? radius / lateral : 0.0;
    *point = QPointF(model.centerX + model.normalizationScale * cameraRay[0] * scale,
                     model.centerY + model.normalizationScale * cameraRay[1] * scale);
    return std::isfinite(point->x()) && std::isfinite(point->y());
}

QVector<FitFrame> fitFrames(const HorizonCalibrationSession& session)
{
    QVector<FitFrame> frames;
    if (session.cameraId != HorizonCameraDown)
    {
        return frames;
    }
    QHash<int, HorizonFrameMetadata> metadata;
    for (const HorizonFrameMetadata& frame : session.frames)
    {
        metadata.insert(frame.frameIndex, frame);
    }
    for (auto it = session.annotations.cbegin(); it != session.annotations.cend(); ++it)
    {
        const auto metadataIt = metadata.constFind(it.key());
        const HorizonFrameAnnotation& annotation = it.value();
        if (metadataIt == metadata.cend() || annotation.skipped || annotation.points.size() < 2
            || !metadataIt->attitudeValid || !metadataIt->heightValid
            || !std::isfinite(metadataIt->rollDeg) || !std::isfinite(metadataIt->pitchDeg)
            || !std::isfinite(metadataIt->heightMm))
        {
            continue;
        }
        FitFrame frame;
        frame.frameIndex = it.key();
        frame.points = annotation.points;
        frame.rollDeg = metadataIt->rollDeg;
        frame.pitchDeg = metadataIt->pitchDeg;
        frame.heightMm = metadataIt->heightMm;
        frames.push_back(frame);
    }
    return frames;
}

double distanceToSegment(const QPointF& point, const QPointF& first, const QPointF& second)
{
    const QPointF direction = second - first;
    const double length2 = QPointF::dotProduct(direction, direction);
    if (length2 <= 1e-12)
    {
        return QLineF(point, first).length();
    }
    const double fraction = clampValue(QPointF::dotProduct(point - first, direction) / length2, 0.0, 1.0);
    return QLineF(point, first + direction * fraction).length();
}

double frameError(const DownGroundRangeModel& model, const FitFrame& frame)
{
    const QVector<QPointF> boundary = DownGroundRangeCalibration::predictBoundary(
        model, frame.rollDeg, frame.pitchDeg, frame.heightMm);
    if (boundary.size() < 3)
    {
        return std::numeric_limits<double>::infinity();
    }
    double sum = 0.0;
    for (const QPointF& point : frame.points)
    {
        double best = std::numeric_limits<double>::infinity();
        for (int index = 0; index < boundary.size(); ++index)
        {
            best = qMin(best, distanceToSegment(point,
                                                 boundary[index],
                                                 boundary[(index + 1) % boundary.size()]));
        }
        sum += best * best;
    }
    return std::sqrt(sum / frame.points.size());
}

double huber(double value)
{
    constexpr double delta = 2.0;
    return value <= delta ? value * value : 2.0 * delta * value - delta * delta;
}

double objective(const DownGroundRangeModel& model,
                 const QVector<FitFrame>& frames,
                 const QVector<int>& selected)
{
    if (!modelParametersValid(model) || selected.isEmpty())
    {
        return std::numeric_limits<double>::infinity();
    }
    double sum = 0.0;
    for (int index : selected)
    {
        const double error = frameError(model, frames[index]);
        if (!std::isfinite(error))
        {
            return std::numeric_limits<double>::infinity();
        }
        sum += huber(error);
    }
    return sum / selected.size();
}

void setParameter(DownGroundRangeModel* model, int index, double value)
{
    switch (index)
    {
    case 0: model->centerX = clampValue(value, 20.0, model->imageSize.width() - 21.0); break;
    case 1: model->centerY = clampValue(value, 20.0, model->imageSize.height() - 21.0); break;
    case 2: model->thetaCoefficients[0] = clampValue(value, 0.5, 3.0); break;
    case 3: model->thetaCoefficients[1] = clampValue(value, -1.5, 1.5); break;
    case 4: model->thetaCoefficients[2] = clampValue(value, -1.0, 1.0); break;
    case 5:
    case 6:
    case 7:
        model->extrinsicRotationRad[index - 5] = clampValue(value, -0.7, 0.7);
        model->bodyToCamera = multiply(rotationMatrix(model->extrinsicRotationRad[0],
                                                      model->extrinsicRotationRad[1],
                                                      model->extrinsicRotationRad[2]),
                                       baseBodyToCamera());
        break;
    case 8: model->effectiveRangeMm = clampValue(value, 6500.0, 8000.0); break;
    case 9: model->heightBiasMm = clampValue(value, -500.0, 500.0); break;
    default: break;
    }
}

double parameter(const DownGroundRangeModel& model, int index)
{
    switch (index)
    {
    case 0: return model.centerX;
    case 1: return model.centerY;
    case 2: return model.thetaCoefficients[0];
    case 3: return model.thetaCoefficients[1];
    case 4: return model.thetaCoefficients[2];
    case 5:
    case 6:
    case 7: return model.extrinsicRotationRad[index - 5];
    case 8: return model.effectiveRangeMm;
    case 9: return model.heightBiasMm;
    default: return 0.0;
    }
}

DownGroundRangeModel optimize(DownGroundRangeModel model,
                              const QVector<FitFrame>& frames,
                              const QVector<int>& selected)
{
    std::array<double, 10> steps = {2.0, 2.0, 0.05, 0.05, 0.03,
                                    0.03, 0.03, 0.03, 150.0, 60.0};
    double best = objective(model, frames, selected);
    for (int pass = 0; pass < 8; ++pass)
    {
        for (int index = 0; index < (int)steps.size(); ++index)
        {
            const double original = parameter(model, index);
            DownGroundRangeModel candidate = model;
            setParameter(&candidate, index, original - steps[index]);
            double candidateError = objective(candidate, frames, selected);
            if (candidateError < best)
            {
                model = candidate;
                best = candidateError;
            }
            candidate = model;
            setParameter(&candidate, index, parameter(model, index) + steps[index]);
            candidateError = objective(candidate, frames, selected);
            if (candidateError < best)
            {
                model = candidate;
                best = candidateError;
            }
        }
        for (double& step : steps)
        {
            step *= 0.5;
        }
    }
    return model;
}

QJsonArray numberArray(const std::array<double, 3>& values)
{
    return {values[0], values[1], values[2]};
}

QJsonArray matrixArray(const std::array<double, 9>& values)
{
    return {QJsonArray({values[0], values[1], values[2]}),
            QJsonArray({values[3], values[4], values[5]}),
            QJsonArray({values[6], values[7], values[8]})};
}

bool numberArrayValue(const QJsonValue& value, std::array<double, 3>* output)
{
    const QJsonArray values = value.toArray();
    if (output == nullptr || values.size() != 3)
    {
        return false;
    }
    for (int index = 0; index < 3; ++index)
    {
        (*output)[index] = values[index].toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite((*output)[index]))
        {
            return false;
        }
    }
    return true;
}

bool matrixArrayValue(const QJsonValue& value, std::array<double, 9>* output)
{
    const QJsonArray rows = value.toArray();
    if (output == nullptr || rows.size() != 3)
    {
        return false;
    }
    for (int row = 0; row < 3; ++row)
    {
        const QJsonArray values = rows[row].toArray();
        if (values.size() != 3)
        {
            return false;
        }
        for (int column = 0; column < 3; ++column)
        {
            (*output)[row * 3 + column] = values[column].toDouble(
                std::numeric_limits<double>::quiet_NaN());
            if (!std::isfinite((*output)[row * 3 + column]))
            {
                return false;
            }
        }
    }
    return true;
}

QString floatLiteral(double value)
{
    QString literal = QLocale::c().toString(value, 'g', 9);
    if (!literal.contains(QLatin1Char('.')) && !literal.contains(QLatin1Char('e'), Qt::CaseInsensitive))
    {
        literal += QStringLiteral(".0");
    }
    return literal + QLatin1Char('f');
}
}

DownGroundRangeModel DownGroundRangeCalibration::defaultModel(const QSize& imageSize)
{
    DownGroundRangeModel model;
    model.imageSize = imageSize;
    model.centerX = (imageSize.width() - 1.0) * 0.5;
    model.centerY = (imageSize.height() - 1.0) * 0.5;
    model.normalizationScale = qMax(1.0, (imageSize.width() - 1.0) * 0.5);
    model.thetaCoefficients = {1.24, 0.0, 0.0};
    model.bodyToCamera = baseBodyToCamera();
    return model;
}

QVector<QPointF> DownGroundRangeCalibration::predictBoundary(const DownGroundRangeModel& model,
                                                               double rollDeg,
                                                               double pitchDeg,
                                                               double heightMm)
{
    QVector<QPointF> boundary;
    if (!model.valid || !modelParametersValid(model) || !std::isfinite(heightMm))
    {
        return boundary;
    }
    const Vec3 g = HorizonCalibration::gravity(rollDeg, pitchDeg);
    Vec3 first = cross(g, {1.0, 0.0, 0.0});
    if (length(first) < 1e-6)
    {
        first = cross(g, {0.0, 1.0, 0.0});
    }
    first = normalized(first);
    const Vec3 second = normalized(cross(g, first));
    const double height = heightMm + model.heightBiasMm;
    if (height <= 0.0)
    {
        return boundary;
    }
    for (int index = 0; index < BoundarySamples; ++index)
    {
        const double angle = 2.0 * Pi * index / BoundarySamples;
        const Vec3 ray = normalized({height * g[0] + model.effectiveRangeMm
                                                  * (std::cos(angle) * first[0] + std::sin(angle) * second[0]),
                                      height * g[1] + model.effectiveRangeMm
                                                  * (std::cos(angle) * first[1] + std::sin(angle) * second[1]),
                                      height * g[2] + model.effectiveRangeMm
                                                  * (std::cos(angle) * first[2] + std::sin(angle) * second[2])});
        QPointF point;
        if (!projectRay(model, ray, &point))
        {
            return {};
        }
        boundary.push_back(point);
    }
    return boundary;
}

double DownGroundRangeCalibration::pointDistance(const DownGroundRangeModel& model,
                                                  double rollDeg,
                                                  double pitchDeg,
                                                  double heightMm,
                                                  const QPointF& point)
{
    const QVector<QPointF> boundary = predictBoundary(model, rollDeg, pitchDeg, heightMm);
    if (boundary.size() < 3)
    {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (int index = 0; index < boundary.size(); ++index)
    {
        best = qMin(best, distanceToSegment(point,
                                             boundary[index],
                                             boundary[(index + 1) % boundary.size()]));
    }
    return best;
}

DownGroundRangeFitResult DownGroundRangeCalibration::fit(const HorizonCalibrationSession& session)
{
    DownGroundRangeFitResult result;
    if (session.cameraId != HorizonCameraDown)
    {
        result.error = QStringLiteral("下摄闭合边界模型只接受 Down HCAL 会话。");
        return result;
    }
    const QVector<FitFrame> frames = fitFrames(session);
    if (frames.size() < 4)
    {
        result.error = QStringLiteral("至少需要 4 帧有效高度的局部边界标注。");
        return result;
    }

    DownGroundRangeModel bestModel = defaultModel(session.imageSize);
    bestModel.valid = true;
    std::mt19937 generator(0x444f574eU);
    std::uniform_real_distribution<double> range(6500.0, 8000.0);
    std::uniform_real_distribution<double> bias(-500.0, 500.0);
    QVector<int> all;
    all.reserve(frames.size());
    for (int index = 0; index < frames.size(); ++index)
    {
        all.push_back(index);
    }
    QVector<int> bestInliers;
    double bestSquared = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < RansacIterations; ++iteration)
    {
        DownGroundRangeModel candidate = bestModel;
        candidate.effectiveRangeMm = range(generator);
        candidate.heightBiasMm = bias(generator);
        QVector<int> inliers;
        double squared = 0.0;
        for (int index : all)
        {
            const double error = frameError(candidate, frames[index]);
            if (error <= InlierThresholdPixels)
            {
                inliers.push_back(index);
                squared += error * error;
            }
        }
        if (inliers.size() > bestInliers.size()
            || (inliers.size() == bestInliers.size() && squared < bestSquared))
        {
            bestModel = candidate;
            bestInliers = inliers;
            bestSquared = squared;
        }
    }
    if (bestInliers.size() < 4)
    {
        bestInliers = all;
    }
    for (int pass = 0; pass < 3; ++pass)
    {
        bestModel = optimize(bestModel, frames, bestInliers);
        QVector<int> classified;
        for (int index : all)
        {
            if (frameError(bestModel, frames[index]) <= InlierThresholdPixels)
            {
                classified.push_back(index);
            }
        }
        if (classified.isEmpty() || classified == bestInliers)
        {
            break;
        }
        bestInliers = classified;
    }

    result.model = bestModel;
    result.model.sampleCount = frames.size();
    result.model.inlierCount = bestInliers.size();
    result.inlierFrames.reserve(bestInliers.size());
    QVector<double> errors;
    for (int index : all)
    {
        const double error = frameError(bestModel, frames[index]);
        result.frameErrors.insert(frames[index].frameIndex, error);
        if (error <= InlierThresholdPixels)
        {
            result.inlierFrames.push_back(frames[index].frameIndex);
            errors.push_back(error);
        }
        else
        {
            result.outlierFrames.push_back(frames[index].frameIndex);
        }
    }
    if (errors.isEmpty())
    {
        result.error = QStringLiteral("拟合没有得到可用内点，请检查标注的 7 米垂足中心段。");
        return result;
    }
    std::sort(errors.begin(), errors.end());
    double squared = 0.0;
    for (double error : errors)
    {
        squared += error * error;
    }
    result.model.rmse = std::sqrt(squared / errors.size());
    result.model.medianError = errors[errors.size() / 2];
    result.model.maxError = errors.last();
    result.model.rollMin = result.model.rollMax = frames[bestInliers.first()].rollDeg;
    result.model.pitchMin = result.model.pitchMax = frames[bestInliers.first()].pitchDeg;
    result.model.heightMinMm = result.model.heightMaxMm = frames[bestInliers.first()].heightMm;
    QVector<bool> azimuthBins(72, false);
    for (int index : bestInliers)
    {
        const FitFrame& frame = frames[index];
        result.model.rollMin = qMin(result.model.rollMin, frame.rollDeg);
        result.model.rollMax = qMax(result.model.rollMax, frame.rollDeg);
        result.model.pitchMin = qMin(result.model.pitchMin, frame.pitchDeg);
        result.model.pitchMax = qMax(result.model.pitchMax, frame.pitchDeg);
        result.model.heightMinMm = qMin(result.model.heightMinMm, frame.heightMm);
        result.model.heightMaxMm = qMax(result.model.heightMaxMm, frame.heightMm);
        const QPointF midpoint = frame.points[frame.points.size() / 2];
        const double angle = std::atan2(midpoint.y() - result.model.centerY,
                                        midpoint.x() - result.model.centerX);
        const int bin = qBound(0, (int)std::floor((angle + Pi) * azimuthBins.size() / (2.0 * Pi)),
                               azimuthBins.size() - 1);
        azimuthBins[bin] = true;
    }
    const int covered = std::count(azimuthBins.cbegin(), azimuthBins.cend(), true);
    result.model.azimuthCoverage = covered * 100.0 / azimuthBins.size();
    result.model.valid = true;
    result.fitted = true;
    result.exportable = result.inlierFrames.size() >= MinimumInlierFrames
                        && result.model.azimuthCoverage >= 75.0
                        && result.model.heightMaxMm - result.model.heightMinMm >= 300.0
                        && result.model.rmse <= 1.5;
    if (!result.exportable)
    {
        result.error = QStringLiteral("模型草稿已生成，但内点、方位覆盖、高度跨度或 RMSE 未达到导出条件。");
    }
    return result;
}

bool DownGroundRangeCalibration::loadModel(const QString& path,
                                           DownGroundRangeModel* model,
                                           QString* errorMessage)
{
    if (model == nullptr)
    {
        setError(errorMessage, QStringLiteral("下摄模型输出为空。"));
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
    const QJsonObject root = document.object();
    if (parseError.error != QJsonParseError::NoError || !document.isObject()
        || root.value(QStringLiteral("format")).toString() != QStringLiteral("horizon_model")
        || root.value(QStringLiteral("model_type")).toString()
               != QStringLiteral("down_ground_range_fisheye"))
    {
        setError(errorMessage, QStringLiteral("不是受支持的下摄闭合边界模型。"));
        return false;
    }
    DownGroundRangeModel loaded;
    loaded.sourcePath = QFileInfo(path).absoluteFilePath();
    loaded.imageSize = QSize(root.value(QStringLiteral("image_width")).toInt(),
                             root.value(QStringLiteral("image_height")).toInt());
    loaded.cameraId = (quint8)root.value(QStringLiteral("camera_id")).toInt(255);
    loaded.centerX = root.value(QStringLiteral("center_x")).toDouble(std::numeric_limits<double>::quiet_NaN());
    loaded.centerY = root.value(QStringLiteral("center_y")).toDouble(std::numeric_limits<double>::quiet_NaN());
    loaded.normalizationScale = root.value(QStringLiteral("normalization_scale"))
                                    .toDouble(std::numeric_limits<double>::quiet_NaN());
    loaded.effectiveRangeMm = root.value(QStringLiteral("effective_range_mm"))
                                  .toDouble(std::numeric_limits<double>::quiet_NaN());
    loaded.heightBiasMm = root.value(QStringLiteral("height_bias_mm"))
                              .toDouble(std::numeric_limits<double>::quiet_NaN());
    if (loaded.cameraId != HorizonCameraDown
        || !numberArrayValue(root.value(QStringLiteral("theta_coefficients")), &loaded.thetaCoefficients)
        || !matrixArrayValue(root.value(QStringLiteral("body_to_camera")), &loaded.bodyToCamera)
        || !modelParametersValid(loaded))
    {
        setError(errorMessage, QStringLiteral("下摄闭合边界模型字段无效。"));
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
    loaded.heightMinMm = root.value(QStringLiteral("height_min_mm")).toDouble();
    loaded.heightMaxMm = root.value(QStringLiteral("height_max_mm")).toDouble();
    loaded.azimuthCoverage = root.value(QStringLiteral("azimuth_coverage_percent")).toDouble();
    loaded.valid = true;
    *model = loaded;
    return true;
}

bool DownGroundRangeCalibration::exportModel(const DownGroundRangeModel& model,
                                             const QString& jsonPath,
                                             const QString& headerPath,
                                             QString* errorMessage)
{
    if (!model.valid || !modelParametersValid(model) || jsonPath.isEmpty() || headerPath.isEmpty())
    {
        setError(errorMessage, QStringLiteral("下摄闭合边界模型无效或导出路径为空。"));
        return false;
    }
    QJsonObject root({
        {QStringLiteral("format"), QStringLiteral("horizon_model")},
        {QStringLiteral("version"), 1},
        {QStringLiteral("model_type"), QStringLiteral("down_ground_range_fisheye")},
        {QStringLiteral("camera_id"), HorizonCameraDown},
        {QStringLiteral("camera_name"), QStringLiteral("Down")},
        {QStringLiteral("image_width"), model.imageSize.width()},
        {QStringLiteral("image_height"), model.imageSize.height()},
        {QStringLiteral("coordinate_system"), QStringLiteral("Air FRD, degrees, millimeters, image pixels")},
        {QStringLiteral("center_x"), model.centerX},
        {QStringLiteral("center_y"), model.centerY},
        {QStringLiteral("normalization_scale"), model.normalizationScale},
        {QStringLiteral("theta_coefficients"), numberArray(model.thetaCoefficients)},
        {QStringLiteral("body_to_camera"), matrixArray(model.bodyToCamera)},
        {QStringLiteral("effective_range_mm"), model.effectiveRangeMm},
        {QStringLiteral("height_bias_mm"), model.heightBiasMm},
        {QStringLiteral("inlier_count"), model.inlierCount},
        {QStringLiteral("sample_count"), model.sampleCount},
        {QStringLiteral("rmse_px"), model.rmse},
        {QStringLiteral("median_error_px"), model.medianError},
        {QStringLiteral("max_error_px"), model.maxError},
        {QStringLiteral("roll_min_deg"), model.rollMin},
        {QStringLiteral("roll_max_deg"), model.rollMax},
        {QStringLiteral("pitch_min_deg"), model.pitchMin},
        {QStringLiteral("pitch_max_deg"), model.pitchMax},
        {QStringLiteral("height_min_mm"), model.heightMinMm},
        {QStringLiteral("height_max_mm"), model.heightMaxMm},
        {QStringLiteral("azimuth_coverage_percent"), model.azimuthCoverage}
    });
    QSaveFile jsonFile(jsonPath);
    if (!jsonFile.open(QIODevice::WriteOnly)
        || jsonFile.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0
        || !jsonFile.commit())
    {
        setError(errorMessage, jsonFile.errorString());
        return false;
    }

    QString source;
    source += QStringLiteral("#ifndef DOWN_GROUND_RANGE_MODEL_H\n#define DOWN_GROUND_RANGE_MODEL_H\n\n#include <math.h>\n\n");
    source += QStringLiteral("static const float down_ground_range_center_x = %1;\n").arg(floatLiteral(model.centerX));
    source += QStringLiteral("static const float down_ground_range_center_y = %1;\n").arg(floatLiteral(model.centerY));
    source += QStringLiteral("static const float down_ground_range_scale = %1;\n").arg(floatLiteral(model.normalizationScale));
    source += QStringLiteral("static const float down_ground_range_theta[3] = {%1, %2, %3};\n")
                  .arg(floatLiteral(model.thetaCoefficients[0]), floatLiteral(model.thetaCoefficients[1]), floatLiteral(model.thetaCoefficients[2]));
    source += QStringLiteral("static const float down_ground_range_body_to_camera[9] = {%1, %2, %3, %4, %5, %6, %7, %8, %9};\n")
                  .arg(floatLiteral(model.bodyToCamera[0]), floatLiteral(model.bodyToCamera[1]), floatLiteral(model.bodyToCamera[2]),
                       floatLiteral(model.bodyToCamera[3]), floatLiteral(model.bodyToCamera[4]), floatLiteral(model.bodyToCamera[5]),
                       floatLiteral(model.bodyToCamera[6]), floatLiteral(model.bodyToCamera[7]), floatLiteral(model.bodyToCamera[8]));
    source += QStringLiteral("static const float down_ground_range_mm = %1;\n").arg(floatLiteral(model.effectiveRangeMm));
    source += QStringLiteral("static const float down_ground_height_bias_mm = %1;\n\n").arg(floatLiteral(model.heightBiasMm));
    const double edgeRadius = maxNormalizedRadius(model);
    source += QStringLiteral("static const float down_ground_range_edge_radius = %1;\n").arg(floatLiteral(edgeRadius));
    source += QStringLiteral("static const float down_ground_range_edge_theta = %1;\n").arg(floatLiteral(thetaForRadius(model, edgeRadius)));
    source += QStringLiteral("static const float down_ground_range_edge_slope = %1;\n\n").arg(floatLiteral(qMax(0.05, thetaDerivative(model, edgeRadius))));
    source += QStringLiteral("static inline unsigned char down_ground_range_boundary(float roll_deg, float pitch_deg, float height_mm, unsigned short index, float *x, float *y)\n{\n"
                             "    const float pi = 3.14159265358979323846f;\n"
                             "    const float roll = roll_deg * pi / 180.0f, pitch = pitch_deg * pi / 180.0f;\n"
                             "    float g[3] = {-sinf(pitch), sinf(roll) * cosf(pitch), cosf(roll) * cosf(pitch)};\n"
                             "    float u[3] = {0.0f, g[2], -g[1]}, un = sqrtf(u[1]*u[1] + u[2]*u[2]);\n"
                             "    if (un < 1e-6f) { u[0] = -g[2]; u[1] = 0.0f; u[2] = g[0]; un = sqrtf(u[0]*u[0] + u[2]*u[2]); }\n"
                             "    u[0] /= un; u[1] /= un; u[2] /= un;\n"
                             "    float v[3] = {g[1]*u[2]-g[2]*u[1], g[2]*u[0]-g[0]*u[2], g[0]*u[1]-g[1]*u[0]};\n"
                             "    const float a = 2.0f * pi * (float)(index % 360U) / 360.0f, h = height_mm + down_ground_height_bias_mm;\n"
                             "    float d[3] = {h*g[0] + down_ground_range_mm*(cosf(a)*u[0]+sinf(a)*v[0]), h*g[1] + down_ground_range_mm*(cosf(a)*u[1]+sinf(a)*v[1]), h*g[2] + down_ground_range_mm*(cosf(a)*u[2]+sinf(a)*v[2]};\n"
                             "    float n = sqrtf(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]), c[3]; if (n <= 1e-6f || h <= 0.0f) return 0U; d[0]/=n; d[1]/=n; d[2]/=n;\n"
                             "    c[0]=down_ground_range_body_to_camera[0]*d[0]+down_ground_range_body_to_camera[1]*d[1]+down_ground_range_body_to_camera[2]*d[2];\n"
                             "    c[1]=down_ground_range_body_to_camera[3]*d[0]+down_ground_range_body_to_camera[4]*d[1]+down_ground_range_body_to_camera[5]*d[2];\n"
                             "    c[2]=down_ground_range_body_to_camera[6]*d[0]+down_ground_range_body_to_camera[7]*d[1]+down_ground_range_body_to_camera[8]*d[2];\n"
                             "    const float theta = acosf(fmaxf(-1.0f, fminf(1.0f, c[2]))), lateral = sqrtf(c[0]*c[0]+c[1]*c[1]); float lo=0.0f, hi=down_ground_range_edge_radius, radius;\n"
                             "    if (theta > down_ground_range_edge_theta) radius = down_ground_range_edge_radius + (theta-down_ground_range_edge_theta)/down_ground_range_edge_slope;\n"
                             "    else { for (unsigned char k=0U;k<28U;++k) { float r=(lo+hi)*0.5f, r2=r*r, t=r*(down_ground_range_theta[0]+r2*(down_ground_range_theta[1]+r2*down_ground_range_theta[2])); if(t<theta) lo=r; else hi=r; } radius=(lo+hi)*0.5f; }\n"
                             "    if (lateral <= 1e-6f) { *x=down_ground_range_center_x; *y=down_ground_range_center_y; return 1U; }\n"
                             "    { const float r=radius/lateral; *x=down_ground_range_center_x+down_ground_range_scale*c[0]*r; *y=down_ground_range_center_y+down_ground_range_scale*c[1]*r; } return 1U;\n}\n\n#endif\n");
    QSaveFile headerFile(headerPath);
    if (!headerFile.open(QIODevice::WriteOnly) || headerFile.write(source.toUtf8()) < 0
        || !headerFile.commit())
    {
        setError(errorMessage, headerFile.errorString());
        return false;
    }
    return true;
}
