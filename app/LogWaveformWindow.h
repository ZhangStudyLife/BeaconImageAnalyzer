#ifndef LOG_WAVEFORM_WINDOW_H
#define LOG_WAVEFORM_WINDOW_H

#include "JustFloatLog.h"

#include <QWidget>

#include <memory>

class QCloseEvent;
class WaveformHistoryStore;

class LogWaveformWindow : public QWidget
{
public:
    explicit LogWaveformWindow(QWidget* parent = nullptr);
    ~LogWaveformWindow() override;

    void setUdpMode(bool enabled);
    void setLiveHistory(WaveformHistoryStore* history);
    void configureLiveSource(const QString& sourceName, const QVector<int>& channels);
    void clearLiveData();
    void setLogLayout(JustFloatLogLayout layout);
    void setCsvLog(const JustFloatLog* log);
    void setCsvRow(int row);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    class Private;
    std::unique_ptr<Private> d;
};

#endif
