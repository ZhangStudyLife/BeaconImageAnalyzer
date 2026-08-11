#ifndef JUST_FLOAT_CSV_RECORDER_H
#define JUST_FLOAT_CSV_RECORDER_H

#include "JustFloatLog.h"

#include <QTemporaryFile>

class JustFloatCsvRecorder
{
public:
    enum class State
    {
        Idle,
        Recording,
        PendingSave
    };

    JustFloatCsvRecorder();

    bool start(QString* errorMessage = nullptr);
    bool append(const JustFloatLogRow& row, QString* errorMessage = nullptr);
    bool stop(QString* errorMessage = nullptr);
    bool resume(QString* errorMessage = nullptr);
    bool saveAs(const QString& path, QString* errorMessage = nullptr);
    void discard();

    State state() const;
    quint64 rowCount() const;

private:
    QTemporaryFile m_temporaryFile;
    State m_state = State::Idle;
    quint64 m_rowCount = 0;
    JustFloatLogLayout m_layout = JustFloatLogLayout::Legacy;
    bool m_hasLayout = false;
};

#endif
