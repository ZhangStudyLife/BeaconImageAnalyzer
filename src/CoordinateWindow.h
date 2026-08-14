#ifndef COORDINATE_WINDOW_H
#define COORDINATE_WINDOW_H

#include "CoordinateView.h"

#include <QWidget>

class QLabel;

class CoordinateWindow : public QWidget
{
    Q_OBJECT

public:
    explicit CoordinateWindow(CoordinateView::Mode mode, QWidget* parent = nullptr);
    void setLiveFrame(const TelemetryFrame& frame, quint64 packetCount, const QString& sender);
    void setReplayFrame(const TelemetryFrame& frame, int frameIndex, int frameCount);
    void setUdpState(bool listening);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void updateStatus();

    CoordinateView::Mode m_mode;
    CoordinateView* m_view = nullptr;
    QLabel* m_statusLabel = nullptr;
    TelemetryFrame m_frame;
    quint64 m_packetCount = 0;
    QString m_sender;
    int m_replayIndex = -1;
    int m_replayCount = 0;
    bool m_listening = false;
    bool m_replayMode = false;
    bool m_hasFrame = false;
};

#endif
