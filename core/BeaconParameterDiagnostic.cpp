#include "BeaconParameterDiagnostic.h"

#include "AlgorithmRunner.h"
#include "BeaconResultUtils.h"
#include "FrameRenderer.h"

#include <QDir>
#include <QHash>
#include <QLineF>
#include <QPointF>
#include <QtMath>

namespace
{
struct TrialCandidate
{
    const TwoBl3ParameterDescriptor* descriptor = nullptr;
    AlgorithmParameterInfo info;
    double current = 0.0;
    double value = 0.0;
    qint64 distanceInSteps = 0;
};

QVector<QPointF> beaconCenters(const beacon_result_t& result)
{
    QVector<QPointF> centers;
    const bool legacy = BeaconResultUtils::usesLegacyBeacons(result);
    const beacon_circle_t* beacons = legacy ? result.circles : result.beacons;
    const int count = legacy
        ? BeaconResultUtils::boundedCount(result.count, BEACON_MAX_CIRCLE_COUNT)
        : BeaconResultUtils::boundedCount(result.beacon_count, BEACON_MAX_BEACON_COUNT);
    for (int index = 0; index < count; ++index)
    {
        if (beacons[index].valid != 0)
        {
            centers.push_back(FrameRenderer::algorithmToImagePoint(beacons[index].x, beacons[index].y));
        }
    }
    const int temporalCount = BeaconResultUtils::boundedCount(result.temporal_beacon_count,
                                                               BEACON_MAX_BEACON_COUNT);
    for (int index = 0; index < temporalCount; ++index)
    {
        const beacon_circle_t& beacon = result.temporal_beacons[index];
        if (beacon.valid == 0)
        {
            continue;
        }
        const QPointF point = FrameRenderer::algorithmToImagePoint(beacon.x, beacon.y);
        bool duplicate = false;
        for (const QPointF& existing : centers)
        {
            if (QLineF(existing, point).length() <= 3.0)
            {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
        {
            centers.push_back(point);
        }
    }
    return centers;
}

QVector<QPointF> lampCenters(const beacon_result_t& result)
{
    QVector<QPointF> centers;
    const int count = BeaconResultUtils::boundedCount(result.car_lamp_count,
                                                       BEACON_MAX_CAR_LAMP_COUNT);
    for (int index = 0; index < count; ++index)
    {
        const beacon_rect_t& lamp = result.car_lamps[index];
        if (lamp.valid != 0)
        {
            centers.push_back(FrameRenderer::algorithmToImagePoint(lamp.cx, lamp.cy));
        }
    }
    return centers;
}

QVector<QPointF> outsideRegion(const QVector<QPointF>& points, const QRectF& region)
{
    QVector<QPointF> result;
    const QRectF excluded = region.adjusted(-6.0, -6.0, 6.0, 6.0);
    for (const QPointF& point : points)
    {
        if (!excluded.contains(point))
        {
            result.push_back(point);
        }
    }
    return result;
}

bool spatiallyEquivalent(const QVector<QPointF>& left, const QVector<QPointF>& right)
{
    if (left.size() != right.size())
    {
        return false;
    }
    QVector<bool> used(right.size(), false);
    for (const QPointF& point : left)
    {
        int bestIndex = -1;
        double bestDistance = 4.01;
        for (int index = 0; index < right.size(); ++index)
        {
            if (used[index])
            {
                continue;
            }
            const double distance = QLineF(point, right[index]).length();
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = index;
            }
        }
        if (bestIndex < 0)
        {
            return false;
        }
        used[bestIndex] = true;
    }
    return true;
}

bool targetPresent(const beacon_result_t& result, const QRectF& region)
{
    for (const QPointF& point : beaconCenters(result))
    {
        if (region.contains(point))
        {
            return true;
        }
    }
    return false;
}

int targetCount(const beacon_result_t& result, const QRectF& region)
{
    int count = 0;
    for (const QPointF& point : beaconCenters(result))
    {
        if (region.contains(point))
        {
            ++count;
        }
    }
    return count;
}

bool collateralStable(const QVector<beacon_result_t>& baseline,
                      const QVector<beacon_result_t>& trial,
                      const QRectF& region)
{
    if (baseline.size() != trial.size())
    {
        return false;
    }
    for (int frame = 0; frame < baseline.size(); ++frame)
    {
        if (!spatiallyEquivalent(outsideRegion(beaconCenters(baseline[frame]), region),
                                 outsideRegion(beaconCenters(trial[frame]), region))
            || !spatiallyEquivalent(lampCenters(baseline[frame]), lampCenters(trial[frame])))
        {
            return false;
        }
    }
    return true;
}

QVector<beacon_result_t> replay(AlgorithmRunner* runner, const QVector<QImage>& frames)
{
    QVector<beacon_result_t> results;
    results.reserve(frames.size());
    runner->resetTemporal();
    for (const QImage& frame : frames)
    {
        results.push_back(runner->process(frame));
    }
    return results;
}

bool applySnapshot(AlgorithmRunner* runner,
                   const BimgParameterSnapshot& snapshot,
                   QString* errorMessage)
{
    QHash<quint16, AlgorithmParameterInfo> available;
    for (const AlgorithmParameterInfo& info : runner->parameterInfos())
    {
        available.insert(info.id, info);
    }
    for (const BimgParameterValue& value : snapshot.values)
    {
        if (value.status != 0 || !available.contains(value.id))
        {
            continue;
        }
        quint32 actual = 0;
        if (!runner->setParameterValue(value.type, value.id, value.valueBits, &actual)
            || actual != value.valueBits)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("无法同步参数 ID 0x%1。")
                                    .arg(value.id, 4, 16, QLatin1Char('0'));
            }
            return false;
        }
    }
    return true;
}

qint64 maximumDistanceInSteps(const TrialCandidate& candidate, int direction)
{
    const double boundary = direction < 0 ? candidate.info.minimum : candidate.info.maximum;
    const double available = direction < 0 ? candidate.current - boundary
                                           : boundary - candidate.current;
    if (available <= 0.0 || candidate.descriptor->step <= 0.0)
    {
        return 0;
    }

    return qMax<qint64>(0, qFloor(available / candidate.descriptor->step + 1e-6));
}

double valueAtDistance(const TrialCandidate& candidate, int direction, qint64 distanceInSteps)
{
    double value = candidate.current
                   + direction * candidate.descriptor->step * distanceInSteps;
    value = qBound((double)candidate.info.minimum,
                   value,
                   (double)candidate.info.maximum);
    return candidate.info.type == 1 ? qRound64(value) : value;
}

bool evaluateCandidate(AlgorithmRunner* runner,
                       const QVector<QImage>& frames,
                       const QVector<beacon_result_t>& baseline,
                       const QRectF& region,
                       bool falsePositive,
                       const TrialCandidate& candidate,
                       quint32 currentBits,
                       double value,
                       bool* success,
                       QVector<beacon_result_t>* successfulResults,
                       QString* errorMessage)
{
    *success = false;
    quint32 actualBits = 0;
    const quint32 requestedBits = TwoBl3ParameterCatalog::bitsFromValue(
        candidate.descriptor->type, value);
    if (!runner->setParameterValue(candidate.descriptor->type,
                                   candidate.descriptor->id,
                                   requestedBits,
                                   &actualBits)
        || actualBits != requestedBits)
    {
        return true;
    }

    const QVector<beacon_result_t> trial = replay(runner, frames);
    const bool present = !trial.isEmpty() && targetPresent(trial.last(), region);
    const bool primarySuccess = falsePositive ? !present : present;
    *success = primarySuccess && collateralStable(baseline, trial, region);
    if (*success && successfulResults != nullptr)
    {
        *successfulResults = trial;
    }

    quint32 restoredBits = 0;
    if (!runner->setParameterValue(candidate.descriptor->type,
                                   candidate.descriptor->id,
                                   currentBits,
                                   &restoredBits)
        || restoredBits != currentBits)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("试参后恢复参数失败，诊断已终止。");
        }
        return false;
    }
    return true;
}

bool findDirectionRecommendation(AlgorithmRunner* runner,
                                 const QVector<QImage>& frames,
                                 const QVector<beacon_result_t>& baseline,
                                 const QRectF& region,
                                 bool falsePositive,
                                 quint32 currentBits,
                                 int direction,
                                 qint64 exclusiveDistanceLimit,
                                 TrialCandidate* candidate,
                                 QVector<beacon_result_t>* results,
                                 QString* errorMessage)
{
    qint64 maximumDistance = maximumDistanceInSteps(*candidate, direction);
    if (exclusiveDistanceLimit > 0)
    {
        maximumDistance = qMin(maximumDistance, exclusiveDistanceLimit - 1);
    }
    if (maximumDistance < 1)
    {
        return true;
    }

    qint64 previousFailure = 0;
    qint64 probeDistance = 1;
    qint64 successfulDistance = 0;
    double successfulValue = 0.0;
    QVector<beacon_result_t> successfulResults;
    while (true)
    {
        probeDistance = qMin(probeDistance, maximumDistance);
        const double value = valueAtDistance(*candidate, direction, probeDistance);
        bool success = false;
        QVector<beacon_result_t> trialResults;
        if (!evaluateCandidate(runner,
                               frames,
                               baseline,
                               region,
                               falsePositive,
                               *candidate,
                               currentBits,
                               value,
                               &success,
                               &trialResults,
                               errorMessage))
        {
            return false;
        }
        if (success)
        {
            successfulDistance = probeDistance;
            successfulValue = value;
            successfulResults = trialResults;
            break;
        }

        previousFailure = probeDistance;
        if (probeDistance == maximumDistance)
        {
            return true;
        }
        probeDistance = qMin(maximumDistance, probeDistance * 2);
    }

    qint64 lower = previousFailure + 1;
    qint64 upper = successfulDistance - 1;
    while (lower <= upper)
    {
        const qint64 middle = lower + (upper - lower) / 2;
        const double value = valueAtDistance(*candidate, direction, middle);
        bool success = false;
        QVector<beacon_result_t> trialResults;
        if (!evaluateCandidate(runner,
                               frames,
                               baseline,
                               region,
                               falsePositive,
                               *candidate,
                               currentBits,
                               value,
                               &success,
                               &trialResults,
                               errorMessage))
        {
            return false;
        }
        if (success)
        {
            successfulDistance = middle;
            successfulValue = value;
            successfulResults = trialResults;
            upper = middle - 1;
        }
        else
        {
            lower = middle + 1;
        }
    }

    candidate->distanceInSteps = successfulDistance;
    candidate->value = successfulValue;
    if (results != nullptr)
    {
        *results = successfulResults;
    }
    return true;
}

void computeRegionStats(const QImage& image, const QRectF& region, BeaconDiagnosticResult* result)
{
    if (result == nullptr || image.isNull())
    {
        return;
    }
    const QImage gray = image.convertToFormat(QImage::Format_Grayscale8);
    const QRect bounds = region.normalized().toAlignedRect().intersected(gray.rect());
    quint64 sum = 0;
    int maximum = 0;
    int count = 0;
    for (int y = bounds.top(); y <= bounds.bottom(); ++y)
    {
        const uchar* row = gray.constScanLine(y);
        for (int x = bounds.left(); x <= bounds.right(); ++x)
        {
            const int value = row[x];
            sum += value;
            maximum = qMax(maximum, value);
            ++count;
        }
    }
    result->regionPixelCount = count;
    result->regionMeanGray = count > 0 ? (double)sum / count : 0.0;
    result->regionMaxGray = maximum;
}
}

BeaconDiagnosticResult BeaconParameterDiagnostic::analyze(const BeaconDiagnosticRequest& request)
{
    BeaconDiagnosticResult result;
    result.analyzedFrameCount = request.frames.size();
    if (request.frames.size() < 10 || request.region.width() < 1.0 || request.region.height() < 1.0)
    {
        result.message = QStringLiteral("诊断区域无效，或 Raw 帧缓存不足 10 帧。");
        return result;
    }
    computeRegionStats(request.frames.last(), request.region, &result);

    AlgorithmRunner runner;
    QString error;
    if (!runner.loadTwoBl3Firmware(request.firmwareImageDirectory, request.buildDirectory, &error))
    {
        result.message = error;
        return result;
    }
    if (!runner.supportsParameterTuning()
        || runner.algorithmBuildId() != request.snapshot.algorithmBuildId)
    {
        result.message = QStringLiteral("2BL3 固件与桌面诊断算法构建 ID 不一致，已拒绝试参。");
        return result;
    }
    if (!applySnapshot(&runner, request.snapshot, &error))
    {
        result.message = error;
        return result;
    }

    const QVector<beacon_result_t> baseline = replay(&runner, request.frames);
    if (baseline.isEmpty())
    {
        result.message = QStringLiteral("无法生成基线检测结果。");
        return result;
    }
    result.beforeResult = baseline.last();
    const int selectedTargets = targetCount(result.beforeResult, request.region);
    if (selectedTargets > 1)
    {
        result.message = QStringLiteral("框内包含多个信标，请缩小区域后重试。");
        return result;
    }
    result.falsePositive = selectedTargets == 1;

    QHash<quint16, BimgParameterValue> snapshotValues;
    for (const BimgParameterValue& value : request.snapshot.values)
    {
        if (value.status == 0)
        {
            snapshotValues.insert(value.id, value);
        }
    }
    QHash<quint16, AlgorithmParameterInfo> infos;
    for (const AlgorithmParameterInfo& info : runner.parameterInfos())
    {
        infos.insert(info.id, info);
    }
    for (const TwoBl3ParameterDescriptor& descriptor : TwoBl3ParameterCatalog::all())
    {
        if (!snapshotValues.contains(descriptor.id) || !infos.contains(descriptor.id)
            || snapshotValues.value(descriptor.id).type != descriptor.type
            || infos.value(descriptor.id).type != descriptor.type)
        {
            result.message = QStringLiteral("BPAR、桌面算法与参数目录不一致，缺少或类型不匹配：%1。")
                                 .arg(descriptor.name);
            return result;
        }
    }

    TrialCandidate best;
    QVector<beacon_result_t> bestResults;
    for (const TwoBl3ParameterDescriptor& descriptor : TwoBl3ParameterCatalog::all())
    {
        if (!descriptor.searchable || !snapshotValues.contains(descriptor.id)
            || !infos.contains(descriptor.id))
        {
            continue;
        }
        const BimgParameterValue currentBits = snapshotValues.value(descriptor.id);
        const AlgorithmParameterInfo info = infos.value(descriptor.id);
        if (currentBits.type != descriptor.type || info.type != descriptor.type)
        {
            continue;
        }

        TrialCandidate candidate;
        candidate.descriptor = &descriptor;
        candidate.info = info;
        candidate.current = TwoBl3ParameterCatalog::valueFromBits(descriptor.type,
                                                                   currentBits.valueBits);
        for (int direction : {-1, 1})
        {
            TrialCandidate directionCandidate = candidate;
            QVector<beacon_result_t> directionResults;
            const qint64 distanceLimit = best.descriptor != nullptr ? best.distanceInSteps : 0;
            if (!findDirectionRecommendation(&runner,
                                             request.frames,
                                             baseline,
                                             request.region,
                                             result.falsePositive,
                                             currentBits.valueBits,
                                             direction,
                                             distanceLimit,
                                             &directionCandidate,
                                             &directionResults,
                                             &error))
            {
                result.message = error;
                return result;
            }
            if (directionCandidate.distanceInSteps == 0)
            {
                continue;
            }
            if (best.descriptor == nullptr
                || directionCandidate.distanceInSteps < best.distanceInSteps)
            {
                best = directionCandidate;
                bestResults = directionResults;
            }
        }
    }

    result.completed = true;
    if (best.descriptor == nullptr)
    {
        result.message = QStringLiteral("菜单参数范围内未找到不影响其他目标的安全单参数方案。曝光无法离线验证。");
        return result;
    }

    result.recommendationFound = true;
    result.parameter = *best.descriptor;
    result.currentValue = best.current;
    result.recommendedValue = best.value;
    result.afterResult = bestResults.last();
    result.message = QStringLiteral("最近 %1 帧框外信标与车灯保持稳定。")
                         .arg(request.frames.size());
    return result;
}
