#include "CoordinateWindow.h"

#include <QCloseEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>

CoordinateWindow::CoordinateWindow(CoordinateView::Mode mode, QWidget* parent)
    : QWidget(parent, Qt::Window),
      m_mode(mode),
      m_view(new CoordinateView(mode, this))
{
    const bool centerMode = mode == CoordinateView::Mode::CenterMapped;
    setWindowTitle(centerMode ? QStringLiteral("Center 融合坐标")
                              : QStringLiteral("CameraModel 相机解耦坐标"));
    resize(900, 680);
    setMinimumSize(480, 360);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(6);
    auto* tools = new QHBoxLayout();
    m_statusLabel = new QLabel(QStringLiteral("UDP 已停止"), this);
    tools->addWidget(m_statusLabel, 1);
    auto* resetButton = new QToolButton(this);
    resetButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
    resetButton->setToolTip(QStringLiteral("复位视图"));
    connect(resetButton, &QToolButton::clicked, m_view, &CoordinateView::resetView);
    tools->addWidget(resetButton);
    root->addLayout(tools);
    root->addWidget(m_view, 1);

    setStyleSheet(QStringLiteral(
        "QWidget{background:#1b1d1f;color:#e8edf0;}"
        "QToolButton{background:#292d30;border:1px solid #454b50;border-radius:4px;padding:4px;}"
        "QToolButton:hover{background:#343a3f;}"
        "QLabel{color:#dce2e6;}"));
}

void CoordinateWindow::setLiveFrame(const TelemetryFrame& frame,
                                    quint64 packetCount,
                                    const QString& sender)
{
    if (!m_listening)
    {
        return;
    }
    m_frame = frame;
    m_packetCount = packetCount;
    m_sender = sender;
    m_replayMode = false;
    m_hasFrame = true;
    m_view->setFrame(frame);
    updateStatus();
}

void CoordinateWindow::setReplayFrame(const TelemetryFrame& frame,
                                      int frameIndex,
                                      int frameCount)
{
    m_frame = frame;
    m_replayIndex = frameIndex;
    m_replayCount = frameCount;
    m_replayMode = true;
    m_hasFrame = true;
    m_view->setFrame(frame);
    updateStatus();
}

void CoordinateWindow::setUdpState(bool listening)
{
    m_listening = listening;
    m_replayMode = false;
    if (!listening)
    {
        m_hasFrame = false;
        m_sender.clear();
        m_view->clearFrame();
    }
    updateStatus();
}

void CoordinateWindow::closeEvent(QCloseEvent* event)
{
    hide();
    event->ignore();
}

void CoordinateWindow::updateStatus()
{
    if (!m_listening && !m_replayMode)
    {
        m_statusLabel->setText(QStringLiteral("UDP 已停止"));
        return;
    }
    if (!m_hasFrame)
    {
        m_statusLabel->setText(QStringLiteral("UDP 监听中 | 等待数据"));
        return;
    }
    const QString source = m_replayMode
                               ? QStringLiteral("CSV 帧 %1/%2")
                                     .arg(m_replayIndex + 1)
                                     .arg(m_replayCount)
                               : QStringLiteral("UDP 包 %1 | %2")
                                     .arg(m_packetCount)
                                     .arg(m_sender);
    m_statusLabel->setText(QStringLiteral("%1 | 时间戳 %2 ms | selected_target_id %3")
                               .arg(source)
                               .arg(m_frame.timestampMs, 0, 'f', 1)
                               .arg(m_frame.selectedTargetId));
}
