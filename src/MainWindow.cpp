#include "MainWindow.h"

#include "CameraView.h"
#include "CoordinateWindow.h"

#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QTime>
#include <QTimer>
#include <QUdpSocket>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
const QStringList CameraNames = {QStringLiteral("Front"), QStringLiteral("Center"), QStringLiteral("Back")};
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      m_udpSocket(new QUdpSocket(this)),
      m_playbackTimer(new QTimer(this)),
      m_recordingTimer(new QTimer(this)),
      m_uiTimer(new QTimer(this)),
      m_centerWindow(new CoordinateWindow(CoordinateView::Mode::CenterMapped, this)),
      m_modelWindow(new CoordinateWindow(CoordinateView::Mode::CameraModel, this)),
      m_globalWindow(new CoordinateWindow(CoordinateView::Mode::CarPlan3Global, this))
{
    setWindowTitle(QStringLiteral("JustFloat 三摄接收与回放"));
    resize(1420, 900);
    buildUi();
    populateAddresses();

    connect(m_udpSocket, &QUdpSocket::readyRead, this, &MainWindow::readPendingDatagrams);
    connect(m_playbackTimer, &QTimer::timeout, this, &MainWindow::advancePlayback);
    m_uiTimer->setInterval(16);
    connect(m_uiTimer, &QTimer::timeout, this, &MainWindow::flushLiveFrame);
    m_uiTimer->start();
    m_recordingTimer->setInterval(200);
    connect(m_recordingTimer, &QTimer::timeout, this, [this]() { updateStatus(); });
    connect(m_modeCombo, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (index == 0)
        {
            setLiveMode();
        }
        else
        {
            setReplayMode();
        }
    });
    connect(m_listenButton, &QPushButton::clicked, this, [this]() {
        if (m_udpSocket->state() == QAbstractSocket::BoundState)
        {
            stopListening();
        }
        else
        {
            startListening();
        }
    });
    connect(m_recordButton, &QPushButton::clicked, this, &MainWindow::toggleRecording);
    connect(m_centerWindowButton, &QPushButton::clicked, this, [this]() {
        m_centerWindow->show();
        m_centerWindow->raise();
        m_centerWindow->activateWindow();
    });
    connect(m_modelWindowButton, &QPushButton::clicked, this, [this]() {
        m_modelWindow->show();
        m_modelWindow->raise();
        m_modelWindow->activateWindow();
    });
    connect(m_globalWindowButton, &QPushButton::clicked, this, [this]() {
        m_globalWindow->show();
        m_globalWindow->raise();
        m_globalWindow->activateWindow();
    });
    connect(m_importButton, &QPushButton::clicked, this, &MainWindow::importCsv);
    connect(m_playButton, &QPushButton::clicked, this, &MainWindow::togglePlayback);
    connect(m_previousButton, &QPushButton::clicked, this, [this]() { setReplayIndex(m_replayIndex - 1); });
    connect(m_nextButton, &QPushButton::clicked, this, [this]() { setReplayIndex(m_replayIndex + 1); });
    connect(m_frameSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::setReplayIndex);
    connect(m_timeline, &QSlider::valueChanged, this, &MainWindow::setReplayIndex);
    connect(m_timestampInput, &QLineEdit::returnPressed, this, &MainWindow::jumpToTimestamp);
    updateControls();
    updateStatus(QStringLiteral("就绪，等待 UDP 或导入 CSV"));
}

void MainWindow::buildUi()
{
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    auto* controls = new QHBoxLayout();
    m_modeCombo = new QComboBox(this);
    m_modeCombo->addItem(QStringLiteral("UDP 实时"));
    m_modeCombo->addItem(QStringLiteral("CSV 回放"));
    m_importButton = new QPushButton(QStringLiteral("导入 CSV"), this);
    m_addressCombo = new QComboBox(this);
    m_addressCombo->setMinimumWidth(190);
    m_portSpin = new QSpinBox(this);
    m_portSpin->setRange(1, 65535);
    m_portSpin->setValue(1347);
    m_listenButton = new QPushButton(QStringLiteral("开始监听"), this);
    m_recordButton = new QPushButton(QStringLiteral("开始记录"), this);
    m_centerWindowButton = new QPushButton(QStringLiteral("Center 坐标窗口"), this);
    m_modelWindowButton = new QPushButton(QStringLiteral("解耦坐标窗口"), this);
    m_globalWindowButton = new QPushButton(QStringLiteral("CarPlan3 全局窗口"), this);
    controls->addWidget(m_modeCombo);
    controls->addWidget(m_importButton);
    controls->addWidget(new QLabel(QStringLiteral("本机 IP"), this));
    controls->addWidget(m_addressCombo);
    controls->addWidget(new QLabel(QStringLiteral("端口"), this));
    controls->addWidget(m_portSpin);
    controls->addWidget(m_listenButton);
    controls->addWidget(m_recordButton);
    controls->addWidget(m_centerWindowButton);
    controls->addWidget(m_modelWindowButton);
    controls->addWidget(m_globalWindowButton);
    controls->addStretch(1);
    root->addLayout(controls);

    auto* camerasLayout = new QHBoxLayout();
    camerasLayout->setSpacing(8);
    for (int index = 0; index < 3; ++index)
    {
        auto* group = new QGroupBox(CameraNames[index], this);
        auto* layout = new QVBoxLayout(group);
        const bool backCamera = index == 2;
        m_cameraViews[index] = new CameraView(CameraNames[index],
                                              backCamera,
                                              backCamera,
                                              group);
        layout->addWidget(m_cameraViews[index], 1);
        camerasLayout->addWidget(group, 1);
    }
    root->addLayout(camerasLayout, 1);

    auto* playback = new QHBoxLayout();
    m_playButton = new QPushButton(QStringLiteral("播放"), this);
    m_previousButton = new QPushButton(QStringLiteral("上一帧"), this);
    m_nextButton = new QPushButton(QStringLiteral("下一帧"), this);
    m_frameSpin = new QSpinBox(this);
    m_frameSpin->setRange(0, 0);
    m_frameSpin->setPrefix(QStringLiteral("Frame "));
    m_timeline = new QSlider(Qt::Horizontal, this);
    m_timeline->setRange(0, 0);
    m_speedCombo = new QComboBox(this);
    m_speedCombo->addItem(QStringLiteral("0.25x"), 0.25);
    m_speedCombo->addItem(QStringLiteral("0.5x"), 0.5);
    m_speedCombo->addItem(QStringLiteral("1.0x"), 1.0);
    m_speedCombo->addItem(QStringLiteral("2.0x"), 2.0);
    m_speedCombo->setCurrentIndex(2);
    playback->addWidget(m_playButton);
    playback->addWidget(m_previousButton);
    playback->addWidget(m_nextButton);
    playback->addWidget(m_frameSpin);
    playback->addWidget(m_timeline, 1);
    playback->addWidget(new QLabel(QStringLiteral("速度"), this));
    playback->addWidget(m_speedCombo);
    root->addLayout(playback);

    auto* timestampJump = new QHBoxLayout();
    m_timestampInput = new QLineEdit(this);
    m_timestampInput->setPlaceholderText(QStringLiteral("123456.0"));
    m_timestampInput->setMaximumWidth(240);
    timestampJump->addWidget(new QLabel(QStringLiteral("目标时间戳 (ms)"), this));
    timestampJump->addWidget(m_timestampInput);
    timestampJump->addStretch(1);
    root->addLayout(timestampJump);

    auto* infoGroup = new QGroupBox(QStringLiteral("状态"), this);
    auto* infoLayout = new QVBoxLayout(infoGroup);
    m_flightInfoLabel = new QLabel(QStringLiteral("时间戳 -- | 高度 -- | Roll -- Pitch -- Yaw --"), infoGroup);
    m_motionInfoLabel = new QLabel(QStringLiteral("车 yaw -- | 实际速度 X/Y -- | 目标速度 X/Y --"), infoGroup);
    m_statusLabel = new QLabel(infoGroup);
    m_statusLabel->setWordWrap(true);
    infoLayout->addWidget(m_flightInfoLabel);
    infoLayout->addWidget(m_motionInfoLabel);
    infoLayout->addWidget(m_statusLabel);
    root->addWidget(infoGroup);

    setCentralWidget(central);
    setStyleSheet(QStringLiteral(
        "QMainWindow{background:#1b1d1f;color:#e8edf0;}"
        "QGroupBox{border:1px solid #596168;border-radius:6px;margin-top:8px;padding-top:8px;}"
        "QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;}"
        "QPushButton,QComboBox,QSpinBox,QLineEdit{min-height:28px;background:#292d30;border:1px solid #454b50;border-radius:4px;padding:2px 8px;}"
        "QPushButton:hover{background:#343a3f;}"
        "QLabel{color:#dce2e6;}"
    ));
}

void MainWindow::populateAddresses()
{
    m_addressCombo->addItem(QStringLiteral("所有 IPv4 地址"),
                            QHostAddress(QHostAddress::AnyIPv4).toString());
    const auto interfaces = QNetworkInterface::allInterfaces();
    for (const QNetworkInterface& interface : interfaces)
    {
        for (const QNetworkAddressEntry& entry : interface.addressEntries())
        {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol &&
                entry.ip() != QHostAddress::LocalHost)
            {
                m_addressCombo->addItem(entry.ip().toString(), entry.ip().toString());
            }
        }
    }
}

void MainWindow::setLiveMode()
{
    m_playing = false;
    m_playbackTimer->stop();
    m_liveMode = true;
    updateControls();
    updateStatus(QStringLiteral("已切换到 UDP 实时模式"));
}

void MainWindow::setReplayMode()
{
    if (m_udpSocket->state() == QAbstractSocket::BoundState)
    {
        stopListening();
    }
    m_playing = false;
    m_playbackTimer->stop();
    m_liveMode = false;
    updateControls();
    if (m_replayFrames.isEmpty())
    {
        updateStatus(QStringLiteral("请先导入 CSV"));
    }
}

void MainWindow::startListening()
{
    if (!m_liveMode)
    {
        m_modeCombo->setCurrentIndex(0);
    }
    const QHostAddress address(m_addressCombo->currentData().toString());
    if (!m_udpSocket->bind(address, static_cast<quint16>(m_portSpin->value()),
                           QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
    {
        QMessageBox::warning(this, QStringLiteral("UDP 监听失败"), m_udpSocket->errorString());
        return;
    }
    m_packetCount = 0;
    m_errorCount = 0;
    m_lastSender.clear();
    m_hasPendingFrame = false;
    m_liveCarPlan3.reset();
    m_listenButton->setText(QStringLiteral("停止监听"));
    m_centerWindow->setUdpState(true);
    m_modelWindow->setUdpState(true);
    m_globalWindow->setUdpState(true);
    updateStatus(QStringLiteral("UDP 监听中：%1:%2")
                     .arg(m_addressCombo->currentText())
                     .arg(m_portSpin->value()));
    updateControls();
}

void MainWindow::stopListening()
{
    if (m_recording)
    {
        stopAndSaveRecording();
    }
    m_udpSocket->close();
    m_hasPendingFrame = false;
    m_listenButton->setText(QStringLiteral("开始监听"));
    m_centerWindow->setUdpState(false);
    m_modelWindow->setUdpState(false);
    m_globalWindow->setUdpState(false);
    updateStatus(QStringLiteral("UDP 已停止"));
    updateControls();
}

void MainWindow::readPendingDatagrams()
{
    while (m_udpSocket->hasPendingDatagrams())
    {
        const QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        TelemetryFrame frame;
        QString error;
        if (!TelemetryProtocol::parseDatagram(datagram.data(), &frame, &error))
        {
            ++m_errorCount;
            continue;
        }
        m_liveCarPlan3.process(&frame);
        ++m_packetCount;
        m_lastSender = QStringLiteral("%1:%2")
                           .arg(datagram.senderAddress().toString())
                           .arg(datagram.senderPort());
        if (m_recording)
        {
            m_recordedFrames.push_back(frame);
        }
        m_pendingFrame = frame;
        m_hasPendingFrame = true;
    }
}

void MainWindow::flushLiveFrame()
{
    if (m_liveMode && m_hasPendingFrame)
    {
        const TelemetryFrame frame = m_pendingFrame;
        m_hasPendingFrame = false;
        showFrame(frame);
        m_centerWindow->setLiveFrame(frame, m_packetCount, m_lastSender);
        m_modelWindow->setLiveFrame(frame, m_packetCount, m_lastSender);
        m_globalWindow->setLiveFrame(frame, m_packetCount, m_lastSender);
    }
    if (m_liveMode && m_udpSocket->state() == QAbstractSocket::BoundState)
    {
        updateStatus();
    }
}

void MainWindow::importCsv()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入 JustFloat CSV"),
                                                      QString(),
                                                      QStringLiteral("CSV 文件 (*.csv);;所有文件 (*.*)"));
    if (path.isEmpty())
    {
        return;
    }
    QVector<TelemetryFrame> frames;
    QString error;
    if (!TelemetryProtocol::loadCsv(path, &frames, &error))
    {
        QMessageBox::warning(this, QStringLiteral("CSV 导入失败"), error);
        return;
    }
    m_replayFrames = std::move(frames);
    m_modeCombo->setCurrentIndex(1);
    m_frameSpin->setRange(0, m_replayFrames.size() - 1);
    m_timeline->setRange(0, m_replayFrames.size() - 1);
    setReplayIndex(0);
    updateStatus(QStringLiteral("已导入 %1，共 %2 帧")
                     .arg(path)
                     .arg(m_replayFrames.size()));
}

void MainWindow::toggleRecording()
{
    if (m_recording)
    {
        stopAndSaveRecording();
        return;
    }
    if (m_udpSocket->state() != QAbstractSocket::BoundState)
    {
        QMessageBox::information(this, QStringLiteral("UDP 记录"), QStringLiteral("请先开始 UDP 监听。"));
        return;
    }
    startRecording();
}

void MainWindow::startRecording()
{
    m_recordedFrames.clear();
    m_recordingElapsed.start();
    m_recording = true;
    m_recordingTimer->start();
    m_recordButton->setText(QStringLiteral("● 停止并保存"));
    m_recordButton->setStyleSheet(QStringLiteral("color:#ff5c68;font-weight:600;"));
    updateControls();
    updateStatus(QStringLiteral("● 正在记录"));
}

void MainWindow::stopAndSaveRecording()
{
    if (!m_recording)
    {
        return;
    }
    m_recording = false;
    m_recordingTimer->stop();
    m_recordButton->setText(QStringLiteral("开始记录"));
    m_recordButton->setStyleSheet(QString());

    if (m_recordedFrames.isEmpty())
    {
        updateControls();
        updateStatus(QStringLiteral("未收到数据，记录已丢弃"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("保存 JustFloat CSV"),
                                                      QStringLiteral("justfloat_%1.csv")
                                                          .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")),
                                                      QStringLiteral("CSV 文件 (*.csv)"));
    if (path.isEmpty())
    {
        m_recordedFrames.clear();
        updateControls();
        updateStatus(QStringLiteral("记录已丢弃"));
        return;
    }
    QString error;
    if (!TelemetryProtocol::saveCsv(path, m_recordedFrames, &error))
    {
        QMessageBox::warning(this, QStringLiteral("保存记录失败"), error);
        updateStatus(QStringLiteral("保存失败，记录已丢弃"));
    }
    else
    {
        updateStatus(QStringLiteral("记录已保存：%1，共 %2 帧")
                         .arg(path)
                         .arg(m_recordedFrames.size()));
    }
    m_recordedFrames.clear();
    updateControls();
}

void MainWindow::showFrame(const TelemetryFrame& frame)
{
    for (int index = 0; index < 3; ++index)
    {
        m_cameraViews[index]->setFrame(frame, index, frame.markerActive);
    }
    m_flightInfoLabel->setText(QStringLiteral("时间戳 %1 ms | 高度 %2 mm | Roll %3°  Pitch %4°  Yaw %5°")
                                   .arg(frame.timestampMs, 0, 'f', 1)
                                   .arg(frame.aircraftHeightMm, 0, 'f', 1)
                                   .arg(frame.aircraftRollDeg, 0, 'f', 2)
                                   .arg(frame.aircraftPitchDeg, 0, 'f', 2)
                                   .arg(frame.aircraftYawDeg, 0, 'f', 2));
    m_motionInfoLabel->setText(QStringLiteral("车 yaw %1° | 实际 right X %2 / forward Y %3 m/s | 目标 right X %4 / forward Y %5 m/s")
                                   .arg(frame.carYawDeg, 0, 'f', 2)
                                   .arg(frame.carActualVelocityX, 0, 'f', 3)
                                   .arg(frame.carActualVelocityY, 0, 'f', 3)
                                   .arg(frame.carTargetVelocityX, 0, 'f', 3)
                                   .arg(frame.carTargetVelocityY, 0, 'f', 3));
}

void MainWindow::setReplayIndex(int index)
{
    if (m_replayFrames.isEmpty())
    {
        return;
    }
    m_replayIndex = qBound(0, index, m_replayFrames.size() - 1);
    {
        QSignalBlocker frameBlocker(m_frameSpin);
        QSignalBlocker sliderBlocker(m_timeline);
        m_frameSpin->setValue(m_replayIndex);
        m_timeline->setValue(m_replayIndex);
    }
    showFrame(m_replayFrames[m_replayIndex]);
    m_centerWindow->setReplayFrame(m_replayFrames[m_replayIndex],
                                   m_replayIndex,
                                   m_replayFrames.size());
    m_modelWindow->setReplayFrame(m_replayFrames[m_replayIndex],
                                  m_replayIndex,
                                  m_replayFrames.size());
    m_globalWindow->setReplayFrame(m_replayFrames[m_replayIndex],
                                   m_replayIndex,
                                   m_replayFrames.size());
    updateControls();
}

void MainWindow::jumpToTimestamp()
{
    bool ok = false;
    const double targetTimestamp = m_timestampInput->text().trimmed().toDouble(&ok);
    if (!ok || !std::isfinite(targetTimestamp))
    {
        updateStatus(QStringLiteral("请输入有效的目标时间戳"));
        return;
    }

    int nearestIndex = -1;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (int index = 0; index < m_replayFrames.size(); ++index)
    {
        const double timestamp = TelemetryProtocol::playbackTimestampMs(m_replayFrames[index]);
        if (!std::isfinite(timestamp))
        {
            continue;
        }
        const double distance = std::abs(timestamp - targetTimestamp);
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestIndex = index;
        }
    }
    if (nearestIndex < 0)
    {
        updateStatus(QStringLiteral("CSV 中没有有效时间戳"));
        return;
    }

    setReplayIndex(nearestIndex);
    if (m_playing)
    {
        scheduleNextFrame();
    }
    updateStatus(QStringLiteral("已跳转到时间戳 %1 ms（第 %2 帧）")
                     .arg(m_replayFrames[nearestIndex].timestampMs, 0, 'g', 9)
                     .arg(nearestIndex + 1));
}

void MainWindow::togglePlayback()
{
    if (m_liveMode || m_replayFrames.isEmpty())
    {
        return;
    }
    m_playing = !m_playing;
    m_playButton->setText(m_playing ? QStringLiteral("暂停") : QStringLiteral("播放"));
    if (m_playing)
    {
        if (m_replayIndex >= m_replayFrames.size() - 1)
        {
            setReplayIndex(0);
        }
        scheduleNextFrame();
    }
    else
    {
        m_playbackTimer->stop();
    }
}

void MainWindow::advancePlayback()
{
    if (!m_playing)
    {
        return;
    }
    if (m_replayIndex >= m_replayFrames.size() - 1)
    {
        m_playing = false;
        m_playButton->setText(QStringLiteral("播放"));
        m_playbackTimer->stop();
        return;
    }
    setReplayIndex(m_replayIndex + 1);
    scheduleNextFrame();
}

void MainWindow::scheduleNextFrame()
{
    if (m_playing)
    {
        m_playbackTimer->start(playbackIntervalMs());
    }
}

int MainWindow::playbackIntervalMs() const
{
    if (m_replayIndex < 0 || m_replayIndex + 1 >= m_replayFrames.size())
    {
        return 20;
    }
    double delta = TelemetryProtocol::playbackTimestampMs(m_replayFrames[m_replayIndex + 1]) -
                   TelemetryProtocol::playbackTimestampMs(m_replayFrames[m_replayIndex]);
    if (!std::isfinite(delta) || delta < 1.0 || delta > 1000.0)
    {
        delta = 20.0;
    }
    const double speed = m_speedCombo->currentData().toDouble();
    return qBound(1, static_cast<int>(std::lround(delta / std::max(0.1, speed))), 1000);
}

void MainWindow::updateControls()
{
    const bool listening = m_udpSocket->state() == QAbstractSocket::BoundState;
    const bool hasReplay = !m_replayFrames.isEmpty();
    m_listenButton->setEnabled(m_liveMode || listening);
    m_recordButton->setEnabled(listening || m_recording);
    m_importButton->setEnabled(!m_recording);
    m_playButton->setEnabled(!m_liveMode && hasReplay);
    m_previousButton->setEnabled(!m_liveMode && hasReplay);
    m_nextButton->setEnabled(!m_liveMode && hasReplay);
    m_frameSpin->setEnabled(!m_liveMode && hasReplay);
    m_timeline->setEnabled(!m_liveMode && hasReplay);
    m_timestampInput->setEnabled(!m_liveMode && hasReplay);
}

void MainWindow::updateStatus(const QString& message)
{
    QString status = message;
    if (status.isEmpty())
    {
        status = m_liveMode ? QStringLiteral("UDP 实时") : QStringLiteral("CSV 回放");
    }
    status += QStringLiteral(" | 包 %1 | 错 %2")
                  .arg(m_packetCount)
                  .arg(m_errorCount);
    if (m_recording)
    {
        status += QStringLiteral(" | ● REC %1 | %2 帧")
                      .arg(QTime::fromMSecsSinceStartOfDay(m_recordingElapsed.elapsed())
                               .toString(QStringLiteral("mm:ss.zzz")))
                      .arg(m_recordedFrames.size());
    }
    if (!m_lastSender.isEmpty())
    {
        status += QStringLiteral(" | 来源 %1").arg(m_lastSender);
    }
    m_statusLabel->setText(status);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    if (m_recording)
    {
        stopAndSaveRecording();
    }
    m_udpSocket->close();
    m_centerWindow->hide();
    m_modelWindow->hide();
    QMainWindow::closeEvent(event);
}
