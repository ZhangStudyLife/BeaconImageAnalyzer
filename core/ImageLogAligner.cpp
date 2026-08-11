#include "ImageLogAligner.h"

#include <QHash>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr int SequenceModulus = 128;
constexpr double NominalImagePeriodMs = 20.0;
constexpr int MaximumEvaluationFrames = 500;

struct LogSequenceSample
{
    int rowIndex = -1;
    qint64 sequence = 0;
};

struct CandidateScore
{
    qint64 offset = 0;
    int validFrames = 0;
    int matchedFrames = 0;
    int attitudeComparisons = 0;
    double coverage = 0.0;
    double attitudeError = 0.0;
    double timeJitterMs = 0.0;
    double score = -std::numeric_limits<double>::infinity();
};

double median(QVector<double> values)
{
    if (values.isEmpty())
    {
        return 0.0;
    }
    const qsizetype middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];
    if ((values.size() & 1) == 0)
    {
        const double lower = *std::max_element(values.begin(), values.begin() + middle);
        result = (lower + result) * 0.5;
    }
    return result;
}

bool sourceCameraSupported(quint8 sourceCameraId)
{
    return sourceCameraId == 0U || sourceCameraId == 2U;
}

int dominantSourceCamera(const QVector<ImageFrameSidecarRecord>& frames, int* validCount)
{
    int counts[3] = {0, 0, 0};
    int total = 0;
    for (const ImageFrameSidecarRecord& frame : frames)
    {
        if (!frame.sourceFrameValid || !sourceCameraSupported(frame.sourceCameraId))
        {
            continue;
        }
        ++counts[frame.sourceCameraId];
        ++total;
    }
    if (validCount != nullptr)
    {
        *validCount = total;
    }
    if (total == 0)
    {
        return -1;
    }
    return counts[0] >= counts[2] ? 0 : 2;
}

QVector<LogSequenceSample> unwrapLogSequences(const JustFloatLog& log, int cameraIndex)
{
    QVector<LogSequenceSample> samples;
    samples.reserve(log.rowCount());
    bool hasPrevious = false;
    quint8 previousRaw = 0;
    qint64 unwrapped = 0;
    double previousTimeMs = 0.0;

    for (int rowIndex = 0; rowIndex < log.rowCount(); ++rowIndex)
    {
        const JustFloatLogRow& row = log.rowAt(rowIndex);
        const JustFloatSingleLampSourceFrame& stamp =
            row.singleLampRoi.sourceFrames[cameraIndex];
        if (!stamp.valid)
        {
            continue;
        }

        const quint8 raw = stamp.sequenceLow7;
        if (!hasPrevious)
        {
            unwrapped = raw;
            hasPrevious = true;
        }
        else
        {
            const int baseDelta = (static_cast<int>(raw) - static_cast<int>(previousRaw)
                                   + SequenceModulus) % SequenceModulus;
            int delta = baseDelta;
            const double elapsedMs = row.rowTime - previousTimeMs;
            if (elapsedMs > 200.0)
            {
                const double expectedFrames = elapsedMs / NominalImagePeriodMs;
                int bestDelta = delta;
                double bestError = std::abs(static_cast<double>(delta) - expectedFrames);
                for (int cycles = 1; cycles <= 64; ++cycles)
                {
                    const int candidate = baseDelta + cycles * SequenceModulus;
                    const double error = std::abs(static_cast<double>(candidate) - expectedFrames);
                    if (error < bestError)
                    {
                        bestDelta = candidate;
                        bestError = error;
                    }
                    if (static_cast<double>(candidate) > expectedFrames + SequenceModulus)
                    {
                        break;
                    }
                }
                delta = bestDelta;
            }
            else if (baseDelta > SequenceModulus / 2)
            {
                // A short backwards jump is an out-of-order log row, not a 2.5 s frame leap.
                continue;
            }
            unwrapped += delta;
        }

        samples.push_back({rowIndex, unwrapped});
        previousRaw = raw;
        previousTimeMs = row.rowTime;
    }
    return samples;
}

QHash<qint64, QVector<int>> rowsBySequence(const QVector<LogSequenceSample>& samples)
{
    QHash<qint64, QVector<int>> rows;
    for (const LogSequenceSample& sample : samples)
    {
        rows[sample.sequence].push_back(sample.rowIndex);
    }
    return rows;
}

double attitudeError(const ImageFrameSidecarRecord& frame,
                     const JustFloatLogRow& row,
                     bool* compared)
{
    if (compared != nullptr)
    {
        *compared = false;
    }
    if (!frame.attitudeValid || !std::isfinite(frame.rollDeg) || !std::isfinite(frame.pitchDeg)
        || !std::isfinite(row.roll) || !std::isfinite(row.pitch))
    {
        return 0.0;
    }
    double error = std::hypot(static_cast<double>(frame.rollDeg - row.roll),
                              static_cast<double>(frame.pitchDeg - row.pitch));
    if (frame.heightValid && row.singleLampRoi.heightValid)
    {
        error += std::abs(static_cast<double>(frame.heightMm - row.singleLampRoi.heightMm)) / 60.0;
    }
    if (compared != nullptr)
    {
        *compared = true;
    }
    return error;
}

double frameRelativeTimeMs(const ImageFrameSidecarRecord& frame,
                           const ImageFrameSidecarRecord& anchor)
{
    if (frame.captureTimeValid && anchor.captureTimeValid)
    {
        return static_cast<double>(static_cast<quint32>(frame.captureTimeMs
                                                        - anchor.captureTimeMs));
    }
    return static_cast<double>(frame.hostTimeMs - anchor.hostTimeMs);
}

int bestRowForFrame(const ImageFrameSidecarRecord& frame,
                    const QVector<int>& rows,
                    const JustFloatLog& log,
                    double relativeTimeMs,
                    double timeOffsetMs,
                    bool useTime,
                    double* selectedAttitudeError,
                    bool* attitudeCompared)
{
    int bestRow = -1;
    double bestCost = std::numeric_limits<double>::infinity();
    double bestAttitude = 0.0;
    bool bestCompared = false;
    for (int rowIndex : rows)
    {
        const JustFloatLogRow& row = log.rowAt(rowIndex);
        bool compared = false;
        const double posture = attitudeError(frame, row, &compared);
        const double timing = useTime
            ? std::abs((row.rowTime - relativeTimeMs) - timeOffsetMs) / 20.0
            : 0.0;
        const double cost = posture + timing;
        if (cost < bestCost)
        {
            bestCost = cost;
            bestRow = rowIndex;
            bestAttitude = posture;
            bestCompared = compared;
        }
    }
    if (selectedAttitudeError != nullptr) { *selectedAttitudeError = bestAttitude; }
    if (attitudeCompared != nullptr) { *attitudeCompared = bestCompared; }
    return bestRow;
}

QVector<int> evaluationFrameIndices(const QVector<ImageFrameSidecarRecord>& frames,
                                    int sourceCameraId)
{
    QVector<int> eligible;
    eligible.reserve(frames.size());
    for (int index = 0; index < frames.size(); ++index)
    {
        const ImageFrameSidecarRecord& frame = frames[index];
        if (frame.sourceFrameValid && frame.sourceCameraId == sourceCameraId)
        {
            eligible.push_back(index);
        }
    }
    if (eligible.size() <= MaximumEvaluationFrames)
    {
        return eligible;
    }

    QVector<int> sampled;
    sampled.reserve(MaximumEvaluationFrames);
    for (int i = 0; i < MaximumEvaluationFrames; ++i)
    {
        const qsizetype index = static_cast<qsizetype>(i) * (eligible.size() - 1)
                                / (MaximumEvaluationFrames - 1);
        sampled.push_back(eligible[index]);
    }
    return sampled;
}

CandidateScore evaluateOffset(qint64 offset,
                              const QVector<ImageFrameSidecarRecord>& frames,
                              const QVector<int>& frameIndices,
                              const QHash<qint64, QVector<int>>& rows,
                              const JustFloatLog& log)
{
    CandidateScore candidate;
    candidate.offset = offset;
    candidate.validFrames = frameIndices.size();
    if (frameIndices.isEmpty())
    {
        return candidate;
    }

    const ImageFrameSidecarRecord& anchor = frames[frameIndices.front()];
    QVector<double> timeOffsets;
    double attitudeSum = 0.0;
    for (int frameIndex : frameIndices)
    {
        const ImageFrameSidecarRecord& frame = frames[frameIndex];
        const qint64 target = static_cast<qint64>(frame.sourceFrameSequence) - offset;
        const auto found = rows.constFind(target);
        if (found == rows.cend())
        {
            continue;
        }
        double posture = 0.0;
        bool compared = false;
        const int rowIndex = bestRowForFrame(frame,
                                             found.value(),
                                             log,
                                             0.0,
                                             0.0,
                                             false,
                                             &posture,
                                             &compared);
        if (rowIndex < 0)
        {
            continue;
        }
        ++candidate.matchedFrames;
        if (compared)
        {
            attitudeSum += posture;
            ++candidate.attitudeComparisons;
        }
        const double relativeTime = frameRelativeTimeMs(frame, anchor);
        timeOffsets.push_back(log.rowAt(rowIndex).rowTime - relativeTime);
    }

    candidate.coverage = candidate.validFrames > 0
        ? static_cast<double>(candidate.matchedFrames) / candidate.validFrames : 0.0;
    candidate.attitudeError = candidate.attitudeComparisons > 0
        ? attitudeSum / candidate.attitudeComparisons : 0.0;
    const double timeOffset = median(timeOffsets);
    QVector<double> deviations;
    deviations.reserve(timeOffsets.size());
    for (double value : timeOffsets)
    {
        deviations.push_back(std::abs(value - timeOffset));
    }
    candidate.timeJitterMs = median(deviations);
    candidate.score = candidate.coverage * 100.0
                      - candidate.attitudeError * 0.7
                      - qMin(candidate.timeJitterMs, 200.0) * 0.03;
    if (candidate.attitudeComparisons == 0)
    {
        candidate.score -= 4.0;
    }
    return candidate;
}

void buildFullMapping(qint64 offset,
                      const QVector<ImageFrameSidecarRecord>& frames,
                      int sourceCameraId,
                      const QHash<qint64, QVector<int>>& rows,
                      const JustFloatLog& log,
                      QVector<int>* mapping,
                      int* validFrames,
                      int* matchedFrames,
                      double* meanAttitudeError,
                      double* timeJitterMs)
{
    mapping->fill(-1, frames.size());
    QVector<int> eligible = evaluationFrameIndices(frames, sourceCameraId);
    // The final mapping must cover every eligible frame, not only the evaluation sample.
    if (eligible.size() != frames.size())
    {
        eligible.clear();
        for (int index = 0; index < frames.size(); ++index)
        {
            if (frames[index].sourceFrameValid && frames[index].sourceCameraId == sourceCameraId)
            {
                eligible.push_back(index);
            }
        }
    }
    *validFrames = eligible.size();
    *matchedFrames = 0;
    *meanAttitudeError = 0.0;
    *timeJitterMs = 0.0;
    if (eligible.isEmpty())
    {
        return;
    }

    const ImageFrameSidecarRecord& anchor = frames[eligible.front()];
    QVector<double> provisionalOffsets;
    for (int frameIndex : eligible)
    {
        const ImageFrameSidecarRecord& frame = frames[frameIndex];
        const auto found = rows.constFind(static_cast<qint64>(frame.sourceFrameSequence) - offset);
        if (found == rows.cend()) { continue; }
        const int rowIndex = bestRowForFrame(frame, found.value(), log, 0.0, 0.0, false,
                                             nullptr, nullptr);
        if (rowIndex < 0) { continue; }
        provisionalOffsets.push_back(log.rowAt(rowIndex).rowTime
                                     - frameRelativeTimeMs(frame, anchor));
    }
    const double timeOffset = median(provisionalOffsets);
    QVector<double> residuals;
    double attitudeSum = 0.0;
    int attitudeCount = 0;
    for (int frameIndex : eligible)
    {
        const ImageFrameSidecarRecord& frame = frames[frameIndex];
        const auto found = rows.constFind(static_cast<qint64>(frame.sourceFrameSequence) - offset);
        if (found == rows.cend()) { continue; }
        const double relativeTime = frameRelativeTimeMs(frame, anchor);
        double posture = 0.0;
        bool compared = false;
        const int rowIndex = bestRowForFrame(frame,
                                             found.value(),
                                             log,
                                             relativeTime,
                                             timeOffset,
                                             true,
                                             &posture,
                                             &compared);
        if (rowIndex < 0) { continue; }
        (*mapping)[frameIndex] = rowIndex;
        ++(*matchedFrames);
        if (compared)
        {
            attitudeSum += posture;
            ++attitudeCount;
        }
        residuals.push_back(std::abs((log.rowAt(rowIndex).rowTime - relativeTime) - timeOffset));
    }
    *meanAttitudeError = attitudeCount > 0 ? attitudeSum / attitudeCount : 0.0;
    *timeJitterMs = median(residuals);
}

double attitudeVariation(const QVector<ImageFrameSidecarRecord>& frames, int sourceCameraId)
{
    QVector<double> values;
    for (const ImageFrameSidecarRecord& frame : frames)
    {
        if (frame.sourceCameraId != sourceCameraId || !frame.attitudeValid)
        {
            continue;
        }
        values.push_back(static_cast<double>(frame.rollDeg)
                         + static_cast<double>(frame.pitchDeg) * 0.7
                         + (frame.heightValid ? static_cast<double>(frame.heightMm) / 100.0 : 0.0));
    }
    if (values.size() < 3)
    {
        return 0.0;
    }
    double mean = 0.0;
    for (double value : values) { mean += value; }
    mean /= values.size();
    double sum = 0.0;
    for (double value : values) { sum += (value - mean) * (value - mean); }
    return std::sqrt(sum / values.size());
}
}

ImageLogAlignmentResult ImageLogAligner::align(const QVector<ImageFrameSidecarRecord>& frames,
                                               const JustFloatLog& log,
                                               int manualCycleShift)
{
    ImageLogAlignmentResult result;
    result.manualCycleShift = manualCycleShift;
    result.videoToLogRow.fill(-1, frames.size());
    if (frames.isEmpty())
    {
        result.message = QStringLiteral("录像侧车没有帧");
        return result;
    }
    if (log.layout() != JustFloatLogLayout::SingleLampRoiV1 || log.rowCount() == 0)
    {
        result.message = QStringLiteral("需要 I0..I35 的 SingleLampRoiV1 核心0日志");
        return result;
    }

    int sourceValidCount = 0;
    result.sourceCameraId = dominantSourceCamera(frames, &sourceValidCount);
    if (result.sourceCameraId < 0)
    {
        result.message = QStringLiteral("侧车没有有效的前摄或后摄来源帧号");
        return result;
    }
    int dominantCount = 0;
    for (const ImageFrameSidecarRecord& frame : frames)
    {
        if (frame.sourceFrameValid && frame.sourceCameraId == result.sourceCameraId)
        {
            ++dominantCount;
        }
    }
    if (dominantCount * 100 < sourceValidCount * 95)
    {
        result.message = QStringLiteral("同一录像混入了多个摄像头来源，无法可靠配准");
        return result;
    }

    const QVector<LogSequenceSample> samples = unwrapLogSequences(log, result.sourceCameraId);
    if (samples.isEmpty())
    {
        result.message = QStringLiteral("核心0日志中对应摄像头的 I33 帧号全部无效");
        return result;
    }
    const QHash<qint64, QVector<int>> rows = rowsBySequence(samples);
    const QVector<int> evaluationFrames = evaluationFrameIndices(frames, result.sourceCameraId);
    if (evaluationFrames.isEmpty())
    {
        result.message = QStringLiteral("侧车没有可用于配准的来源帧");
        return result;
    }

    const ImageFrameSidecarRecord& firstFrame = frames[evaluationFrames.front()];
    QSet<qint64> uniqueOffsets;
    for (const LogSequenceSample& sample : samples)
    {
        if ((sample.sequence & 0x7fLL)
            != (static_cast<qint64>(firstFrame.sourceFrameSequence) & 0x7fLL))
        {
            continue;
        }
        uniqueOffsets.insert(static_cast<qint64>(firstFrame.sourceFrameSequence) - sample.sequence);
    }
    if (uniqueOffsets.isEmpty())
    {
        result.message = QStringLiteral("BIMG完整帧号与 I33 低7位没有共同候选");
        return result;
    }

    QVector<CandidateScore> candidates;
    candidates.reserve(uniqueOffsets.size());
    for (qint64 offset : uniqueOffsets)
    {
        candidates.push_back(evaluateOffset(offset, frames, evaluationFrames, rows, log));
    }
    std::sort(candidates.begin(), candidates.end(), [](const CandidateScore& left,
                                                        const CandidateScore& right) {
        return left.score > right.score;
    });
    result.candidateCount = candidates.size();
    const CandidateScore& best = candidates.front();
    result.automaticSequenceOffset = best.offset;
    result.sequenceOffset = best.offset + static_cast<qint64>(manualCycleShift) * SequenceModulus;
    result.scoreGap = candidates.size() > 1 ? best.score - candidates[1].score
                                            : std::numeric_limits<double>::infinity();

    buildFullMapping(result.sequenceOffset,
                     frames,
                     result.sourceCameraId,
                     rows,
                     log,
                     &result.videoToLogRow,
                     &result.validVideoFrameCount,
                     &result.matchedVideoFrameCount,
                     &result.meanAttitudeError,
                     &result.timeJitterMs);
    result.coverage = result.validVideoFrameCount > 0
        ? static_cast<double>(result.matchedVideoFrameCount) / result.validVideoFrameCount : 0.0;

    const double variation = attitudeVariation(frames, result.sourceCameraId);
    if (result.coverage >= 0.95 && result.matchedVideoFrameCount >= 20
        && best.attitudeComparisons * 2 >= best.matchedFrames
        && result.meanAttitudeError <= 8.0 && result.scoreGap >= 1.0
        && variation >= 1.0)
    {
        result.confidence = ImageLogAlignmentConfidence::High;
    }
    else if (result.coverage >= 0.80 && result.matchedVideoFrameCount >= 10
             && (result.candidateCount == 1 || result.scoreGap >= 0.25)
             && (variation >= 0.5 || result.candidateCount == 1))
    {
        result.confidence = ImageLogAlignmentConfidence::Medium;
    }
    else
    {
        result.confidence = ImageLogAlignmentConfidence::Low;
    }

    result.message = QStringLiteral("%1摄：匹配 %2/%3（%4%），偏移 %5，姿态误差 %6，时间抖动 %7 ms")
                         .arg(result.sourceCameraId == 0 ? QStringLiteral("前")
                                                        : QStringLiteral("后"))
                         .arg(result.matchedVideoFrameCount)
                         .arg(result.validVideoFrameCount)
                         .arg(result.coverage * 100.0, 0, 'f', 1)
                         .arg(result.sequenceOffset)
                         .arg(result.meanAttitudeError, 0, 'f', 2)
                         .arg(result.timeJitterMs, 0, 'f', 1);
    return result;
}

QString ImageLogAligner::confidenceName(ImageLogAlignmentConfidence confidence)
{
    switch (confidence)
    {
    case ImageLogAlignmentConfidence::High: return QStringLiteral("高");
    case ImageLogAlignmentConfidence::Medium: return QStringLiteral("中");
    case ImageLogAlignmentConfidence::Low: return QStringLiteral("低");
    default: return QStringLiteral("不可用");
    }
}
