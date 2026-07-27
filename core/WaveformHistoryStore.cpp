#include "WaveformHistoryStore.h"

#include <QDir>
#include <QFileDevice>
#include <QTemporaryFile>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

namespace
{
constexpr double InvalidSentinel = -900.0;
constexpr int RawQuerySamplesPerPixel = 4;
constexpr int MaximumQueryPixelWidth = 16384;

struct DiskRecord
{
    double timeMs = 0.0;
    std::array<float, JustFloatLog::ChannelCount> values{};
    quint64 availableMask = 0;
};

struct SummaryBlock
{
    quint64 firstSample = 0;
    quint32 sampleCount = 0;
    double startTimeMs = 0.0;
    double endTimeMs = 0.0;
    std::array<float, JustFloatLog::ChannelCount> minimumValues{};
    std::array<float, JustFloatLog::ChannelCount> maximumValues{};
    std::array<double, JustFloatLog::ChannelCount> minimumTimes{};
    std::array<double, JustFloatLog::ChannelCount> maximumTimes{};
    quint64 availableMask = 0;
    quint64 gapMask = 0;
};

struct PixelBucket
{
    bool hasValue = false;
    bool hasGap = false;
    double minimumValue = 0.0;
    double maximumValue = 0.0;
    double minimumTimeMs = 0.0;
    double maximumTimeMs = 0.0;
};

static_assert(std::is_trivially_copyable<DiskRecord>::value,
              "Waveform disk records must be trivially copyable");

bool writeAll(QFileDevice* file, const char* data, qint64 size)
{
    qint64 written = 0;
    while (written < size)
    {
        const qint64 result = file->write(data + written, size - written);
        if (result <= 0)
        {
            return false;
        }
        written += result;
    }
    return true;
}

bool isWaveformValueAvailable(const JustFloatLogRow& row,
                              int channelIndex,
                              double value)
{
    if (!std::isfinite(value) || value <= InvalidSentinel)
    {
        return false;
    }
    if (channelIndex >= 1 && channelIndex <= 18)
    {
        const int offset = channelIndex - 1;
        return row.cameras[offset / 6].beacons[(offset % 6) / 3].valid;
    }
    if (channelIndex >= 19 && channelIndex <= 33)
    {
        return row.cameras[(channelIndex - 19) / 5].carLamp.valid;
    }
    if (channelIndex >= 38 && channelIndex <= 42)
    {
        return row.hasMotionData;
    }
    return true;
}

double sourceTimeMs(const JustFloatLogRow& row)
{
    if (std::isfinite(row.syncTimeMs) && row.syncTimeMs > 0.0)
    {
        return row.syncTimeMs;
    }
    if (std::isfinite(row.rowTime) && row.rowTime >= 0.0)
    {
        return row.rowTime;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

int pixelIndex(double timeMs,
               double startTimeMs,
               double endTimeMs,
               int pixelWidth)
{
    const double span = std::max(1.0, endTimeMs - startTimeMs);
    return std::clamp(static_cast<int>((timeMs - startTimeMs) / span * pixelWidth),
                      0,
                      pixelWidth - 1);
}

void addValue(PixelBucket* bucket, double timeMs, double value)
{
    if (!bucket->hasValue)
    {
        bucket->hasValue = true;
        bucket->minimumValue = value;
        bucket->maximumValue = value;
        bucket->minimumTimeMs = timeMs;
        bucket->maximumTimeMs = timeMs;
        return;
    }
    if (value < bucket->minimumValue)
    {
        bucket->minimumValue = value;
        bucket->minimumTimeMs = timeMs;
    }
    if (value > bucket->maximumValue)
    {
        bucket->maximumValue = value;
        bucket->maximumTimeMs = timeMs;
    }
}

QVector<WaveformHistoryPoint> pointsFromBuckets(const QVector<PixelBucket>& buckets)
{
    QVector<WaveformHistoryPoint> points;
    points.reserve(buckets.size() * 2);
    bool previousHadValue = false;
    for (const PixelBucket& bucket : buckets)
    {
        if (!bucket.hasValue)
        {
            if (previousHadValue && bucket.hasGap)
            {
                points.push_back({0.0, 0.0, false});
            }
            if (bucket.hasGap)
            {
                previousHadValue = false;
            }
            continue;
        }

        if (bucket.hasGap && previousHadValue)
        {
            points.push_back({0.0, 0.0, false});
        }
        if (bucket.minimumTimeMs <= bucket.maximumTimeMs)
        {
            points.push_back({bucket.minimumTimeMs, bucket.minimumValue, true});
            if (bucket.maximumTimeMs != bucket.minimumTimeMs ||
                bucket.maximumValue != bucket.minimumValue)
            {
                points.push_back({bucket.maximumTimeMs, bucket.maximumValue, true});
            }
        }
        else
        {
            points.push_back({bucket.maximumTimeMs, bucket.maximumValue, true});
            points.push_back({bucket.minimumTimeMs, bucket.minimumValue, true});
        }

        if (bucket.hasGap)
        {
            points.push_back({0.0, 0.0, false});
            previousHadValue = false;
        }
        else
        {
            previousHadValue = true;
        }
    }
    return points;
}
}

class WaveformHistoryStore::Private
{
public:
    Private()
    {
        file.setAutoRemove(true);
        file.setFileTemplate(
            QDir::temp().absoluteFilePath(QStringLiteral("BeaconImageAnalyzer-Waveform-XXXXXX.bin")));
        resetSummary();
    }

    void resetMetadata()
    {
        times.clear();
        summaries.clear();
        resetSummary();
        lastSourceTimeMs = std::numeric_limits<double>::quiet_NaN();
        lastHostElapsedMs = 0;
        ++revision;
    }

    void resetSummary()
    {
        currentSummary = SummaryBlock{};
        currentSummary.minimumValues.fill(std::numeric_limits<float>::infinity());
        currentSummary.maximumValues.fill(-std::numeric_limits<float>::infinity());
        currentSummary.minimumTimes.fill(0.0);
        currentSummary.maximumTimes.fill(0.0);
    }

    void updateSummary(const DiskRecord& record, quint64 sampleIndex)
    {
        if (currentSummary.sampleCount == 0)
        {
            currentSummary.firstSample = sampleIndex;
            currentSummary.startTimeMs = record.timeMs;
        }
        currentSummary.endTimeMs = record.timeMs;
        ++currentSummary.sampleCount;

        for (int channel = 0; channel < JustFloatLog::ChannelCount; ++channel)
        {
            const quint64 bit = quint64(1) << channel;
            if ((record.availableMask & bit) == 0)
            {
                currentSummary.gapMask |= bit;
                continue;
            }
            const float value = record.values[channel];
            if ((currentSummary.availableMask & bit) == 0)
            {
                currentSummary.availableMask |= bit;
                currentSummary.minimumValues[channel] = value;
                currentSummary.maximumValues[channel] = value;
                currentSummary.minimumTimes[channel] = record.timeMs;
                currentSummary.maximumTimes[channel] = record.timeMs;
                continue;
            }
            if (value < currentSummary.minimumValues[channel])
            {
                currentSummary.minimumValues[channel] = value;
                currentSummary.minimumTimes[channel] = record.timeMs;
            }
            if (value > currentSummary.maximumValues[channel])
            {
                currentSummary.maximumValues[channel] = value;
                currentSummary.maximumTimes[channel] = record.timeMs;
            }
        }

        if (currentSummary.sampleCount == WaveformHistoryStore::SummaryBlockSize)
        {
            summaries.push_back(currentSummary);
            resetSummary();
        }
    }

    bool readRawRange(quint64 firstSample,
                      quint64 sampleCount,
                      QVector<DiskRecord>* records,
                      QString* errorMessage) const
    {
        records->clear();
        if (sampleCount == 0)
        {
            return true;
        }
        if (!file.flush())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("刷新波形历史文件失败：%1").arg(file.errorString());
            }
            return false;
        }

        const qint64 oldPosition = file.pos();
        const qint64 byteOffset = static_cast<qint64>(firstSample * sizeof(DiskRecord));
        if (!file.seek(byteOffset))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("定位波形历史失败：%1").arg(file.errorString());
            }
            return false;
        }

        records->resize(static_cast<qsizetype>(sampleCount));
        char* destination = reinterpret_cast<char*>(records->data());
        const qint64 expectedBytes = static_cast<qint64>(sampleCount * sizeof(DiskRecord));
        qint64 readBytes = 0;
        while (readBytes < expectedBytes)
        {
            const qint64 result = file.read(destination + readBytes, expectedBytes - readBytes);
            if (result <= 0)
            {
                if (errorMessage != nullptr)
                {
                    *errorMessage = QStringLiteral("读取波形历史失败：%1").arg(file.errorString());
                }
                records->clear();
                (void)file.seek(oldPosition);
                return false;
            }
            readBytes += result;
        }
        (void)file.seek(oldPosition);
        return true;
    }

    mutable QTemporaryFile file;
    QVector<double> times;
    QVector<SummaryBlock> summaries;
    SummaryBlock currentSummary;
    double lastSourceTimeMs = std::numeric_limits<double>::quiet_NaN();
    qint64 lastHostElapsedMs = 0;
    quint64 revision = 0;
};

WaveformHistoryStore::WaveformHistoryStore()
    : d(std::make_unique<Private>())
{
}

WaveformHistoryStore::~WaveformHistoryStore() = default;

bool WaveformHistoryStore::beginSession(QString* errorMessage)
{
    if (d->file.isOpen())
    {
        d->file.close();
    }
    if (!d->file.fileName().isEmpty())
    {
        d->file.remove();
    }
    d->resetMetadata();
    if (!d->file.open())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法创建波形历史临时文件：%1").arg(d->file.errorString());
        }
        return false;
    }
    return true;
}

bool WaveformHistoryStore::clear(QString* errorMessage)
{
    if (!d->file.isOpen())
    {
        return beginSession(errorMessage);
    }
    if (!d->file.resize(0) || !d->file.seek(0))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("清空波形历史失败：%1").arg(d->file.errorString());
        }
        return false;
    }
    d->resetMetadata();
    return true;
}

bool WaveformHistoryStore::append(const JustFloatLogRow& row,
                                  qint64 hostElapsedMs,
                                  QString* errorMessage)
{
    if (!d->file.isOpen())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("波形历史尚未开始。" );
        }
        return false;
    }

    const double source = sourceTimeMs(row);
    double timeMs = 0.0;
    if (d->times.isEmpty())
    {
        timeMs = std::isfinite(source) ? source : static_cast<double>(hostElapsedMs);
    }
    else
    {
        double delta = std::numeric_limits<double>::quiet_NaN();
        if (std::isfinite(source) && std::isfinite(d->lastSourceTimeMs))
        {
            delta = source - d->lastSourceTimeMs;
        }
        if (!std::isfinite(delta) || delta <= 0.0)
        {
            delta = static_cast<double>(hostElapsedMs - d->lastHostElapsedMs);
        }
        if (!std::isfinite(delta) || delta <= 0.0)
        {
            delta = 20.0;
        }
        timeMs = d->times.back() + delta;
    }

    DiskRecord record;
    record.timeMs = timeMs;
    for (int channel = 0; channel < JustFloatLog::ChannelCount; ++channel)
    {
        double value = 0.0;
        if (!JustFloatLog::channelValue(row, channel, &value) ||
            !isWaveformValueAvailable(row, channel, value))
        {
            continue;
        }
        record.values[channel] = static_cast<float>(value);
        record.availableMask |= quint64(1) << channel;
    }

    const qint64 oldFileSize = d->file.size();
    if (!d->file.seek(oldFileSize) ||
        !writeAll(&d->file,
                  reinterpret_cast<const char*>(&record),
                  static_cast<qint64>(sizeof(record))))
    {
        const QString writeError = d->file.errorString();
        const bool restored = d->file.resize(oldFileSize) && d->file.seek(oldFileSize);
        if (errorMessage != nullptr)
        {
            *errorMessage = restored
                                ? QStringLiteral("写入波形历史失败：%1").arg(writeError)
                                : QStringLiteral("写入波形历史失败且无法回滚，本次历史记录已停止：%1")
                                      .arg(writeError);
        }
        if (!restored)
        {
            d->file.close();
        }
        return false;
    }

    const quint64 sampleIndex = static_cast<quint64>(d->times.size());
    d->times.push_back(timeMs);
    d->updateSummary(record, sampleIndex);
    d->lastSourceTimeMs = source;
    d->lastHostElapsedMs = hostElapsedMs;
    ++d->revision;
    return true;
}

QVector<WaveformHistorySeries> WaveformHistoryStore::query(const QVector<int>& channels,
                                                           double startTimeMs,
                                                           double endTimeMs,
                                                           int pixelWidth,
                                                           QString* errorMessage) const
{
    QVector<int> validChannels;
    for (int channel : channels)
    {
        if (channel >= 0 && channel < JustFloatLog::ChannelCount &&
            !validChannels.contains(channel))
        {
            validChannels.push_back(channel);
        }
    }

    QVector<WaveformHistorySeries> result;
    result.reserve(validChannels.size());
    for (int channel : validChannels)
    {
        result.push_back({channel, {}});
    }
    if (validChannels.isEmpty() || d->times.isEmpty())
    {
        return result;
    }

    if (!std::isfinite(startTimeMs) || !std::isfinite(endTimeMs))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("波形查询时间范围无效。");
        }
        return result;
    }

    if (endTimeMs < startTimeMs)
    {
        std::swap(startTimeMs, endTimeMs);
    }
    pixelWidth = std::clamp(pixelWidth, 1, MaximumQueryPixelWidth);
    const auto firstIt = std::lower_bound(d->times.cbegin(), d->times.cend(), startTimeMs);
    const auto lastIt = std::upper_bound(d->times.cbegin(), d->times.cend(), endTimeMs);
    const quint64 firstSample = static_cast<quint64>(firstIt - d->times.cbegin());
    const quint64 lastSample = static_cast<quint64>(lastIt - d->times.cbegin());
    if (lastSample <= firstSample)
    {
        return result;
    }

    QVector<QVector<PixelBucket>> buckets;
    buckets.resize(validChannels.size());
    for (QVector<PixelBucket>& channelBuckets : buckets)
    {
        channelBuckets.resize(pixelWidth);
    }

    const auto addRecord = [&](const DiskRecord& record) {
        const int index = pixelIndex(record.timeMs, startTimeMs, endTimeMs, pixelWidth);
        for (int seriesIndex = 0; seriesIndex < validChannels.size(); ++seriesIndex)
        {
            const int channel = validChannels[seriesIndex];
            PixelBucket& bucket = buckets[seriesIndex][index];
            const quint64 bit = quint64(1) << channel;
            if ((record.availableMask & bit) == 0)
            {
                bucket.hasGap = true;
                continue;
            }
            addValue(&bucket, record.timeMs, record.values[channel]);
        }
    };

    const auto addRawRange = [&](quint64 first, quint64 count) -> bool {
        QVector<DiskRecord> records;
        if (!d->readRawRange(first, count, &records, errorMessage))
        {
            return false;
        }
        for (const DiskRecord& record : records)
        {
            addRecord(record);
        }
        return true;
    };

    const quint64 rangeCount = lastSample - firstSample;
    if (rangeCount <= static_cast<quint64>(pixelWidth) * RawQuerySamplesPerPixel)
    {
        if (!addRawRange(firstSample, rangeCount))
        {
            return {};
        }
    }
    else
    {
        const quint64 firstFullBlock =
            (firstSample + SummaryBlockSize - 1) / SummaryBlockSize;
        const quint64 lastFullBlock = lastSample / SummaryBlockSize;
        const quint64 prefixEnd = std::min(lastSample, firstFullBlock * SummaryBlockSize);
        if (prefixEnd > firstSample && !addRawRange(firstSample, prefixEnd - firstSample))
        {
            return {};
        }

        for (quint64 blockIndex = firstFullBlock;
             blockIndex < lastFullBlock && blockIndex < static_cast<quint64>(d->summaries.size());
             ++blockIndex)
        {
            const SummaryBlock& block = d->summaries[static_cast<qsizetype>(blockIndex)];
            for (int seriesIndex = 0; seriesIndex < validChannels.size(); ++seriesIndex)
            {
                const int channel = validChannels[seriesIndex];
                const quint64 bit = quint64(1) << channel;
                const double referenceTime = (block.startTimeMs + block.endTimeMs) * 0.5;
                PixelBucket& referenceBucket =
                    buckets[seriesIndex][pixelIndex(referenceTime,
                                                    startTimeMs,
                                                    endTimeMs,
                                                    pixelWidth)];
                if ((block.gapMask & bit) != 0)
                {
                    referenceBucket.hasGap = true;
                }
                if ((block.availableMask & bit) == 0)
                {
                    continue;
                }
                PixelBucket& minimumBucket =
                    buckets[seriesIndex][pixelIndex(block.minimumTimes[channel],
                                                    startTimeMs,
                                                    endTimeMs,
                                                    pixelWidth)];
                PixelBucket& maximumBucket =
                    buckets[seriesIndex][pixelIndex(block.maximumTimes[channel],
                                                    startTimeMs,
                                                    endTimeMs,
                                                    pixelWidth)];
                if ((block.gapMask & bit) != 0)
                {
                    minimumBucket.hasGap = true;
                    maximumBucket.hasGap = true;
                }
                addValue(&minimumBucket,
                         block.minimumTimes[channel],
                         block.minimumValues[channel]);
                addValue(&maximumBucket,
                         block.maximumTimes[channel],
                         block.maximumValues[channel]);
            }
        }

        const quint64 suffixStart = std::max(firstSample, lastFullBlock * SummaryBlockSize);
        if (lastSample > suffixStart && !addRawRange(suffixStart, lastSample - suffixStart))
        {
            return {};
        }
    }

    for (int i = 0; i < result.size(); ++i)
    {
        result[i].points = pointsFromBuckets(buckets[i]);
    }
    return result;
}

bool WaveformHistoryStore::isActive() const
{
    return d->file.isOpen();
}

quint64 WaveformHistoryStore::sampleCount() const
{
    return static_cast<quint64>(d->times.size());
}

quint64 WaveformHistoryStore::revision() const
{
    return d->revision;
}

double WaveformHistoryStore::firstTimeMs() const
{
    return d->times.isEmpty() ? 0.0 : d->times.front();
}

double WaveformHistoryStore::lastTimeMs() const
{
    return d->times.isEmpty() ? 0.0 : d->times.back();
}

QString WaveformHistoryStore::temporaryPath() const
{
    return d->file.fileName();
}
