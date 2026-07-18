#ifndef WAVEFORM_HISTORY_STORE_H
#define WAVEFORM_HISTORY_STORE_H

#include "JustFloatLog.h"

#include <QString>
#include <QVector>

#include <memory>

struct WaveformHistoryPoint
{
    double timeMs = 0.0;
    double value = 0.0;
    bool valid = false;
};

struct WaveformHistorySeries
{
    int channelIndex = -1;
    QVector<WaveformHistoryPoint> points;
};

class WaveformHistoryStore
{
public:
    static constexpr int SummaryBlockSize = 256;

    WaveformHistoryStore();
    ~WaveformHistoryStore();

    bool beginSession(QString* errorMessage = nullptr);
    bool clear(QString* errorMessage = nullptr);
    bool append(const JustFloatLogRow& row,
                qint64 hostElapsedMs,
                QString* errorMessage = nullptr);
    QVector<WaveformHistorySeries> query(const QVector<int>& channels,
                                         double startTimeMs,
                                         double endTimeMs,
                                         int pixelWidth,
                                         QString* errorMessage = nullptr) const;

    bool isActive() const;
    quint64 sampleCount() const;
    quint64 revision() const;
    double firstTimeMs() const;
    double lastTimeMs() const;
    QString temporaryPath() const;

private:
    class Private;
    std::unique_ptr<Private> d;
};

#endif
