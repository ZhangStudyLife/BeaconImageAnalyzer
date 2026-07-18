#include "LogReplayWindow.h"

#include "FrameRenderer.h"
#include "LogWaveformWindow.h"
#include "VideoWidget.h"

#include <QAbstractSocket>
#include <QCloseEvent>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QPainter>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStandardPaths>
#include <QStringList>
#include <QTextEdit>
#include <QTimer>
#include <QUdpSocket>
#include <QVBoxLayout>
#include <QtMath>

#include <cmath>
#include <cstring>
#include <limits>

namespace
{
constexpr int CameraCount = 3;
constexpr int CarPlanSlotCount = 2;
constexpr float Pi = 3.1415926f;

const QStringList CameraNames = {
    QStringLiteral("Front"),
    QStringLiteral("Center"),
    QStringLiteral("Back")
};

struct UdpListenAddress
{
    QString label;
    QString address;
};

bool isFinitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

void setGrayMax(QImage* image, int x, int y, int value)
{
    if (image == nullptr ||
        x < 0 || x >= BEACON_IMAGE_W ||
        y < 0 || y >= BEACON_IMAGE_H)
    {
        return;
    }

    uchar* line = image->scanLine(y);
    line[x] = (uchar)qBound(0, qMax((int)line[x], value), 255);
}

double rowPlaybackTimeMs(const JustFloatLogRow& row)
{
    if (isFinitePositive(row.syncTimeMs))
    {
        return row.syncTimeMs;
    }
    if (isFinitePositive(row.rowTime))
    {
        return row.rowTime;
    }
    return 0.0;
}

bool mirrorHorizontalForCamera(int cameraIndex)
{
    return cameraIndex >= 0 && cameraIndex < CameraCount;
}

float displayXForCamera(int cameraIndex, float x)
{
    return mirrorHorizontalForCamera(cameraIndex) ? -x : x;
}

float displayAngleForCamera(int cameraIndex, float angle)
{
    if (!mirrorHorizontalForCamera(cameraIndex))
    {
        return angle;
    }

    float mirrored = 180.0f - angle;
    while (mirrored > 180.0f)
    {
        mirrored -= 360.0f;
    }
    while (mirrored <= -180.0f)
    {
        mirrored += 360.0f;
    }
    return mirrored;
}

float normalizeAngleDeg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle <= -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

QString carPlanBuildDir()
{
    QString root = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if (root.isEmpty())
    {
        root = QDir::temp().absoluteFilePath(QStringLiteral("BeaconImageAnalyzer"));
    }
    return QDir(root).absoluteFilePath(QStringLiteral("car_plan"));
}

QString carPlanSlotName(int slot)
{
    return slot == 0 ? QStringLiteral("A") : QStringLiteral("B");
}

QColor carPlanSlotColor(int slot)
{
    return slot == 0 ? QColor(0, 220, 255) : QColor(255, 160, 0);
}

QVector<UdpListenAddress> localUdpListenAddresses()
{
    QVector<UdpListenAddress> addresses;
    addresses.push_back({ QStringLiteral("所有 IPv4 地址 - 0.0.0.0"), QStringLiteral("0.0.0.0") });
    addresses.push_back({ QStringLiteral("本机回环 - 127.0.0.1"), QStringLiteral("127.0.0.1") });

    const QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    for (const QNetworkInterface& networkInterface : interfaces)
    {
        const QNetworkInterface::InterfaceFlags flags = networkInterface.flags();
        if (!flags.testFlag(QNetworkInterface::IsUp) || !flags.testFlag(QNetworkInterface::IsRunning))
        {
            continue;
        }

        const QList<QNetworkAddressEntry> entries = networkInterface.addressEntries();
        for (const QNetworkAddressEntry& entry : entries)
        {
            const QHostAddress address = entry.ip();
            if (address.protocol() != QAbstractSocket::IPv4Protocol || address.isLoopback())
            {
                continue;
            }

            const QString ip = address.toString();
            const QString label = QStringLiteral("%1 - %2")
                                      .arg(networkInterface.humanReadableName().isEmpty()
                                               ? networkInterface.name()
                                               : networkInterface.humanReadableName(),
                                           ip);
            addresses.push_back({ label, ip });
        }
    }
    return addresses;
}
}

LogReplayWindow::LogReplayWindow(QWidget* parent)
    : QWidget(parent),
      m_timer(new QTimer(this)),
      m_udpSocket(new QUdpSocket(this))
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    setWindowTitle(QStringLiteral("JustFloat 日志回放"));
    resize(1120, 760);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    auto* topRow = new QHBoxLayout;
    m_modeCombo = new QComboBox(this);
    m_modeCombo->addItem(QStringLiteral("CSV 回放"), QStringLiteral("csv"));
    m_modeCombo->addItem(QStringLiteral("UDP 实时"), QStringLiteral("udp"));
    m_importButton = new QPushButton(QStringLiteral("导入 CSV"), this);
    m_addressCombo = new QComboBox(this);
    m_portEdit = new QLineEdit(QStringLiteral("1347"), this);
    m_portEdit->setFixedWidth(76);
    m_listenButton = new QPushButton(QStringLiteral("开始监听"), this);
    m_recordButton = new QPushButton(QStringLiteral("开始记录"), this);
    m_recordButton->setFixedWidth(150);
    m_discardRecordingButton = new QPushButton(QStringLiteral("丢弃记录"), this);
    m_discardRecordingButton->setFixedWidth(92);
    m_discardRecordingButton->setVisible(false);
    m_waveformButton = new QPushButton(QStringLiteral("实时波形"), this);
    m_waveformButton->setFixedWidth(92);
    m_statusLabel = new QLabel(QStringLiteral("未导入日志"), this);
    m_statusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_statusLabel->setWordWrap(true);
    m_returnGridButton = new QPushButton(QStringLiteral("返回三摄"), this);
    m_returnGridButton->setEnabled(false);
    topRow->addWidget(m_modeCombo);
    topRow->addWidget(m_importButton);
    topRow->addWidget(new QLabel(QStringLiteral("本机 IP"), this));
    topRow->addWidget(m_addressCombo);
    topRow->addWidget(new QLabel(QStringLiteral("端口"), this));
    topRow->addWidget(m_portEdit);
    topRow->addWidget(m_listenButton);
    topRow->addWidget(m_recordButton);
    topRow->addWidget(m_discardRecordingButton);
    topRow->addWidget(m_waveformButton);
    topRow->addWidget(m_statusLabel, 1);
    topRow->addWidget(m_returnGridButton);
    root->addLayout(topRow);

    auto* carPlanRow = new QHBoxLayout;
    m_loadCarPlanButton = new QPushButton(QStringLiteral("加载 CarPlan"), this);
    m_loadCarPlanDirButton = new QPushButton(QStringLiteral("加载目录"), this);
    m_resetCarPlanButton = new QPushButton(QStringLiteral("重置 CarPlan"), this);
    m_carPlanStatusLabel = new QLabel(QStringLiteral("CarPlan 未加载"), this);
    m_carPlanStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_carPlanStatusLabel->setWordWrap(true);
    carPlanRow->addWidget(m_loadCarPlanButton);
    carPlanRow->addWidget(m_loadCarPlanDirButton);
    carPlanRow->addWidget(m_resetCarPlanButton);
    carPlanRow->addWidget(m_carPlanStatusLabel, 1);
    m_loadCarPlanButtons[0] = m_loadCarPlanButton;
    m_loadCarPlanDirButtons[0] = m_loadCarPlanDirButton;
    m_carPlanStatusLabels[0] = m_carPlanStatusLabel;
    m_loadCarPlanButtons[0]->setText(QStringLiteral("Load A"));
    m_loadCarPlanDirButtons[0]->setText(QStringLiteral("Dir A"));
    m_carPlanStatusLabels[0]->setText(QStringLiteral("CarPlan A: unloaded"));
    m_loadCarPlanButtons[1] = new QPushButton(QStringLiteral("Load B"), this);
    m_loadCarPlanDirButtons[1] = new QPushButton(QStringLiteral("Dir B"), this);
    m_carPlanStatusLabels[1] = new QLabel(QStringLiteral("CarPlan B: unloaded"), this);
    m_carPlanStatusLabels[1]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_carPlanStatusLabels[1]->setWordWrap(true);
    carPlanRow->addWidget(m_loadCarPlanButtons[1]);
    carPlanRow->addWidget(m_loadCarPlanDirButtons[1]);
    carPlanRow->addWidget(m_carPlanStatusLabels[1], 1);
    root->addLayout(carPlanRow);

    m_cameraLayout = new QHBoxLayout;
    m_cameraLayout->setSpacing(8);
    for (int i = 0; i < CameraCount; ++i)
    {
        auto* group = new QGroupBox(CameraNames[i], this);
        auto* groupLayout = new QVBoxLayout(group);
        groupLayout->setContentsMargins(8, 8, 8, 8);
        groupLayout->setSpacing(6);

        auto* widget = new VideoWidget(group);
        widget->setText(QStringLiteral("导入日志后显示"));
        widget->setFrameGeometry(QSize(BEACON_IMAGE_W, BEACON_IMAGE_H), 1);
        auto* info = new QLabel(QStringLiteral("无数据"), group);
        info->setTextInteractionFlags(Qt::TextSelectableByMouse);
        info->setWordWrap(true);

        groupLayout->addWidget(widget, 1);
        groupLayout->addWidget(info);
        m_cameraGroups[i] = group;
        m_videoWidgets[i] = widget;
        m_cameraInfoLabels[i] = info;
        m_cameraLayout->addWidget(group, 1);

        connect(widget, &VideoWidget::activated, this, [this, i]() {
            setFocusCamera(i);
        });
    }
    root->addLayout(m_cameraLayout, 1);

    auto* controls = new QHBoxLayout;
    m_playButton = new QPushButton(QStringLiteral("播放"), this);
    m_previousButton = new QPushButton(QStringLiteral("上一帧"), this);
    m_nextButton = new QPushButton(QStringLiteral("下一帧"), this);
    m_slider = new QSlider(Qt::Horizontal, this);
    m_slider->setRange(0, 0);
    m_frameSpin = new QSpinBox(this);
    m_frameSpin->setRange(0, 0);
    m_frameSpin->setFixedWidth(92);
    m_speedCombo = new QComboBox(this);
    m_speedCombo->addItem(QStringLiteral("0.5x"), 0.5);
    m_speedCombo->addItem(QStringLiteral("1.0x"), 1.0);
    m_speedCombo->addItem(QStringLiteral("2.0x"), 2.0);
    m_speedCombo->addItem(QStringLiteral("4.0x"), 4.0);
    m_speedCombo->setCurrentIndex(1);
    controls->addWidget(m_playButton);
    controls->addWidget(m_previousButton);
    controls->addWidget(m_nextButton);
    controls->addWidget(new QLabel(QStringLiteral("Frame"), this));
    controls->addWidget(m_frameSpin);
    controls->addWidget(m_slider, 1);
    controls->addWidget(new QLabel(QStringLiteral("速度"), this));
    controls->addWidget(m_speedCombo);
    root->addLayout(controls);

    m_infoText = new QTextEdit(this);
    m_infoText->setReadOnly(true);
    m_infoText->setMinimumHeight(120);
    root->addWidget(m_infoText);

    populateLocalAddresses();
    updateControlState();

    connect(m_importButton, &QPushButton::clicked, this, &LogReplayWindow::importCsv);
    for (int slot = 0; slot < CarPlanSlotCount; ++slot)
    {
        connect(m_loadCarPlanButtons[slot], &QPushButton::clicked, this, [this, slot]() {
            loadCarPlanFile(slot);
        });
        connect(m_loadCarPlanDirButtons[slot], &QPushButton::clicked, this, [this, slot]() {
            loadCarPlanDirectory(slot);
        });
    }
    connect(m_resetCarPlanButton, &QPushButton::clicked, this, &LogReplayWindow::resetCarPlan);
    connect(m_modeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        const bool requestedUdpMode =
            m_modeCombo->currentData().toString() == QStringLiteral("udp");
        if (!setUdpMode(requestedUdpMode))
        {
            const QSignalBlocker blocker(m_modeCombo);
            m_modeCombo->setCurrentIndex(m_udpMode ? 1 : 0);
        }
    });
    connect(m_listenButton, &QPushButton::clicked, this, [this]() {
        if (m_udpSocket->state() == QAbstractSocket::BoundState)
        {
            (void)stopUdpListening();
        }
        else
        {
            startUdpListening();
        }
    });
    connect(m_recordButton, &QPushButton::clicked, this, &LogReplayWindow::toggleRecording);
    connect(m_discardRecordingButton, &QPushButton::clicked, this, [this]() {
        discardPendingRecording(true);
    });
    connect(m_waveformButton, &QPushButton::clicked, this, &LogReplayWindow::showWaveformWindow);
    connect(m_returnGridButton, &QPushButton::clicked, this, &LogReplayWindow::showAllCameras);
    connect(m_playButton, &QPushButton::clicked, this, &LogReplayWindow::togglePlayback);
    connect(m_previousButton, &QPushButton::clicked, this, [this]() {
        setCurrentRow(m_currentRow - 1);
    });
    connect(m_nextButton, &QPushButton::clicked, this, [this]() {
        setCurrentRow(m_currentRow + 1);
    });
    connect(m_slider, &QSlider::valueChanged, this, &LogReplayWindow::setCurrentRow);
    connect(m_frameSpin, qOverload<int>(&QSpinBox::valueChanged), this, &LogReplayWindow::setCurrentRow);
    connect(m_timer, &QTimer::timeout, this, &LogReplayWindow::advancePlayback);
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &LogReplayWindow::readPendingDatagrams);
}

void LogReplayWindow::populateLocalAddresses()
{
    m_addressCombo->clear();
    const QVector<UdpListenAddress> addresses = localUdpListenAddresses();
    for (const UdpListenAddress& address : addresses)
    {
        m_addressCombo->addItem(address.label, address.address);
    }
}

void LogReplayWindow::closeEvent(QCloseEvent* event)
{
    if (!ensureRecordingResolved(QStringLiteral("关闭日志窗口")))
    {
        event->ignore();
        return;
    }

    QWidget::closeEvent(event);
}

bool LogReplayWindow::setUdpMode(bool enabled)
{
    if (m_udpMode == enabled)
    {
        return true;
    }

    if (!enabled && !ensureRecordingResolved(QStringLiteral("切换到 CSV 回放")))
    {
        return false;
    }

    m_udpMode = enabled;
    m_timer->stop();
    m_playing = false;
    m_playButton->setText(QStringLiteral("播放"));
    resetCarPlanState();
    if (!m_udpMode)
    {
        (void)stopUdpListening(false);
        if (m_log.rowCount() > 0)
        {
            setCurrentRow(qBound(0, m_currentRow, m_log.rowCount() - 1));
        }
        else
        {
            m_statusLabel->setText(QStringLiteral("未导入日志"));
        }
    }
    else
    {
        m_statusLabel->setText(QStringLiteral("UDP 未监听"));
    }
    if (m_waveformWindow != nullptr)
    {
        m_waveformWindow->setUdpMode(m_udpMode);
        if (m_udpMode)
        {
            m_waveformWindow->setLiveHistory(&m_waveformHistory);
        }
        if (!m_udpMode)
        {
            m_waveformWindow->setCsvLog(m_log.rowCount() > 0 ? &m_log : nullptr);
            m_waveformWindow->setCsvRow(m_currentRow);
        }
    }
    updateControlState();
    updateInfoText();
    return true;
}

void LogReplayWindow::updateControlState()
{
    const bool listening = m_udpSocket->state() == QAbstractSocket::BoundState;
    const bool csvMode = !m_udpMode;
    m_importButton->setEnabled(csvMode);
    m_playButton->setEnabled(csvMode && m_log.rowCount() > 0);
    m_previousButton->setEnabled(csvMode && m_log.rowCount() > 0);
    m_nextButton->setEnabled(csvMode && m_log.rowCount() > 0);
    m_slider->setEnabled(csvMode && m_log.rowCount() > 0);
    m_frameSpin->setEnabled(csvMode && m_log.rowCount() > 0);
    m_speedCombo->setEnabled(csvMode);
    m_addressCombo->setEnabled(m_udpMode && !listening);
    m_portEdit->setEnabled(m_udpMode && !listening);
    m_listenButton->setEnabled(m_udpMode);
    m_listenButton->setText(listening ? QStringLiteral("停止监听") : QStringLiteral("开始监听"));
    const JustFloatCsvRecorder::State recordingState = m_csvRecorder.state();
    m_recordButton->setEnabled(recordingState != JustFloatCsvRecorder::State::Idle ||
                               (m_udpMode && listening));
    m_discardRecordingButton->setVisible(recordingState == JustFloatCsvRecorder::State::PendingSave);
    bool hasCarPlan = false;
    for (int slot = 0; slot < CarPlanSlotCount; ++slot)
    {
        hasCarPlan = hasCarPlan || m_carPlanRunners[slot].isLoaded();
    }
    m_resetCarPlanButton->setEnabled(hasCarPlan);
    updateRecordingState();
}

void LogReplayWindow::startUdpListening()
{
    bool ok = false;
    const quint16 port = (quint16)m_portEdit->text().toUShort(&ok);
    if (!ok || port == 0)
    {
        QMessageBox::warning(this, QStringLiteral("UDP 监听"), QStringLiteral("端口号无效。"));
        return;
    }

    const QHostAddress address(m_addressCombo->currentData().toString());
    if (!m_udpSocket->bind(address,
                           port,
                           QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
    {
        QMessageBox::critical(this,
                              QStringLiteral("UDP 监听失败"),
                              m_udpSocket->errorString());
        return;
    }

    m_udpPacketCount = 0;
    m_udpErrorCount = 0;
    m_lastUdpPeer.clear();
    m_hasLiveRow = false;
    m_currentRow = -1;
    m_udpElapsedTimer.restart();
    m_waveformHistoryErrorShown = false;
    QString historyError;
    if (!m_waveformHistory.beginSession(&historyError))
    {
        m_waveformHistoryErrorShown = true;
        QMessageBox::warning(this,
                             QStringLiteral("波形历史不可用"),
                             QStringLiteral("UDP 监听仍会继续，但本次会话无法回看完整历史。\n%1")
                                 .arg(historyError));
    }
    resetCarPlanState();
    if (m_waveformWindow != nullptr)
    {
        m_waveformWindow->setLiveHistory(&m_waveformHistory);
        m_waveformWindow->setUdpMode(true);
    }
    m_statusLabel->setText(QStringLiteral("UDP 监听中：%1:%2")
                               .arg(address.toString())
                               .arg(port));
    updateControlState();
    updateInfoText();
}

bool LogReplayWindow::stopUdpListening(bool resolveRecording)
{
    if (resolveRecording && !ensureRecordingResolved(QStringLiteral("停止 UDP 监听")))
    {
        return false;
    }

    if (m_udpSocket->state() == QAbstractSocket::BoundState)
    {
        m_udpSocket->close();
    }
    if (m_udpMode)
    {
        m_statusLabel->setText(QStringLiteral("UDP 已停止"));
    }
    updateControlState();
    return true;
}

void LogReplayWindow::readPendingDatagrams()
{
    while (m_udpSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize((int)m_udpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort = 0;
        const qint64 readSize = m_udpSocket->readDatagram(datagram.data(),
                                                          datagram.size(),
                                                          &sender,
                                                          &senderPort);
        if (readSize < 0)
        {
            m_udpErrorCount++;
            continue;
        }
        datagram.resize((int)readSize);

        const quint64 sequence = m_udpPacketCount;
        m_udpPacketCount++;

        JustFloatLogRow row;
        QString error;
        if (!JustFloatLog::parseDatagram(datagram, sequence, &row, &error))
        {
            m_udpErrorCount++;
            m_statusLabel->setText(QStringLiteral("UDP 解析失败：%1 | 包 %2 错 %3")
                                       .arg(error)
                                       .arg(m_udpPacketCount)
                                       .arg(m_udpErrorCount));
            continue;
        }

        if (m_waveformHistory.isActive())
        {
            QString historyError;
            const qint64 elapsedMs = m_udpElapsedTimer.isValid() ? m_udpElapsedTimer.elapsed() : 0;
            if (!m_waveformHistory.append(row, elapsedMs, &historyError) &&
                !m_waveformHistoryErrorShown)
            {
                m_waveformHistoryErrorShown = true;
                QMessageBox::warning(this,
                                     QStringLiteral("波形历史写入失败"),
                                     QStringLiteral("UDP 监听和 CSV 记录不受影响，但本次波形历史可能不完整。\n%1")
                                         .arg(historyError));
            }
        }

        if (m_csvRecorder.state() == JustFloatCsvRecorder::State::Recording)
        {
            QString recordError;
            if (!m_csvRecorder.append(row, &recordError))
            {
                QString stopError;
                const bool stopped = m_csvRecorder.stop(&stopError);
                if (!stopped && !stopError.isEmpty())
                {
                    recordError += QStringLiteral("\n%1").arg(stopError);
                }
                updateControlState();
                QMessageBox::critical(this,
                                      QStringLiteral("记录失败"),
                                      QStringLiteral("写入临时 CSV 失败：%1").arg(recordError));
            }
            updateRecordingState();
        }
        acceptUdpRow(row, QStringLiteral("%1:%2").arg(sender.toString()).arg(senderPort));
    }
}

void LogReplayWindow::acceptUdpRow(const JustFloatLogRow& row, const QString& peerName)
{
    m_liveRow = row;
    m_hasLiveRow = true;
    m_lastUdpPeer = peerName;
    m_currentRow = (int)qMin<quint64>(m_udpPacketCount - 1, (quint64)std::numeric_limits<int>::max());
    updateCarPlanFromLiveRow(row);
    renderCurrentRow();
    updateInfoText();
    QString status = QStringLiteral("UDP 实时 | 包 %1 | 错 %2 | 最近来源 %3")
                         .arg(m_udpPacketCount)
                         .arg(m_udpErrorCount)
                         .arg(m_lastUdpPeer);
    if (m_csvRecorder.state() != JustFloatCsvRecorder::State::Idle)
    {
        status += QStringLiteral(" | 记录 %1 行").arg(m_csvRecorder.rowCount());
    }
    m_statusLabel->setText(status);
}

void LogReplayWindow::toggleRecording()
{
    switch (m_csvRecorder.state())
    {
    case JustFloatCsvRecorder::State::Idle:
    {
        if (!m_udpMode || m_udpSocket->state() != QAbstractSocket::BoundState)
        {
            QMessageBox::information(this,
                                     QStringLiteral("UDP 记录"),
                                     QStringLiteral("请先开始 UDP 监听。"));
            return;
        }

        QString error;
        if (!m_csvRecorder.start(&error))
        {
            QMessageBox::critical(this, QStringLiteral("开始记录失败"), error);
            return;
        }
        break;
    }

    case JustFloatCsvRecorder::State::Recording:
        if (!stopRecording())
        {
            return;
        }
        (void)savePendingRecording();
        break;

    case JustFloatCsvRecorder::State::PendingSave:
        (void)savePendingRecording();
        break;
    }

    updateControlState();
}

bool LogReplayWindow::stopRecording()
{
    if (m_csvRecorder.state() != JustFloatCsvRecorder::State::Recording)
    {
        return true;
    }

    QString error;
    const bool stopped = m_csvRecorder.stop(&error);
    updateControlState();
    if (!stopped)
    {
        QMessageBox::warning(this,
                             QStringLiteral("记录刷新异常"),
                             QStringLiteral("临时文件刷新失败，但已保留现有内容，可继续尝试保存。\n%1")
                                 .arg(error));
    }
    return m_csvRecorder.state() == JustFloatCsvRecorder::State::PendingSave;
}

bool LogReplayWindow::savePendingRecording()
{
    if (m_csvRecorder.state() != JustFloatCsvRecorder::State::PendingSave)
    {
        return true;
    }

    QSettings settings(QStringLiteral("BeaconImageAnalyzer"),
                       QStringLiteral("BeaconImageAnalyzer"));
    QString directory = settings.value(QStringLiteral("logReplay/recordDirectory")).toString();
    if (directory.isEmpty() || !QDir(directory).exists())
    {
        directory = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    }
    const QString defaultName = QStringLiteral("JustFloat_%1.csv")
                                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
    QString path = QFileDialog::getSaveFileName(this,
                                                QStringLiteral("保存 UDP 记录"),
                                                QDir(directory).filePath(defaultName),
                                                QStringLiteral("CSV (*.csv);;所有文件 (*.*)"));
    if (path.isEmpty())
    {
        updateControlState();
        return false;
    }
    if (QFileInfo(path).suffix().isEmpty())
    {
        path += QStringLiteral(".csv");
    }

    QString error;
    if (!m_csvRecorder.saveAs(path, &error))
    {
        QMessageBox::critical(this, QStringLiteral("保存记录失败"), error);
        updateControlState();
        return false;
    }

    settings.setValue(QStringLiteral("logReplay/recordDirectory"), QFileInfo(path).absolutePath());
    m_statusLabel->setText(QStringLiteral("记录已保存：%1").arg(QFileInfo(path).fileName()));
    updateControlState();
    return true;
}

bool LogReplayWindow::ensureRecordingResolved(const QString& actionName)
{
    const JustFloatCsvRecorder::State state = m_csvRecorder.state();
    const bool wasRecording = state == JustFloatCsvRecorder::State::Recording;
    if (state == JustFloatCsvRecorder::State::Idle)
    {
        return true;
    }

    QMessageBox box(this);
    box.setIcon(QMessageBox::Warning);
    box.setWindowTitle(QStringLiteral("UDP 记录尚未保存"));
    box.setText(state == JustFloatCsvRecorder::State::Recording
                    ? QStringLiteral("当前正在记录，已写入 %1 行。是否先保存再%2？")
                          .arg(m_csvRecorder.rowCount())
                          .arg(actionName)
                    : QStringLiteral("当前有 %1 行记录尚未保存。是否先保存再%2？")
                          .arg(m_csvRecorder.rowCount())
                          .arg(actionName));
    box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Save);

    const QMessageBox::StandardButton choice =
        static_cast<QMessageBox::StandardButton>(box.exec());
    if (choice == QMessageBox::Cancel)
    {
        return false;
    }
    if (choice == QMessageBox::Discard)
    {
        m_csvRecorder.discard();
        updateControlState();
        return true;
    }

    if (!stopRecording())
    {
        return false;
    }
    if (savePendingRecording())
    {
        return true;
    }

    if (wasRecording && m_csvRecorder.state() == JustFloatCsvRecorder::State::PendingSave)
    {
        QString resumeError;
        if (m_csvRecorder.resume(&resumeError))
        {
            m_statusLabel->setText(QStringLiteral("保存已取消，UDP 记录继续。"));
        }
        else
        {
            QMessageBox::warning(this,
                                 QStringLiteral("无法继续记录"),
                                 QStringLiteral("未保存的数据仍然保留，但 UDP 记录未能恢复。\n%1")
                                     .arg(resumeError));
        }
        updateControlState();
    }
    return false;
}

void LogReplayWindow::discardPendingRecording(bool confirm)
{
    if (m_csvRecorder.state() == JustFloatCsvRecorder::State::Idle)
    {
        return;
    }

    if (confirm &&
        QMessageBox::question(this,
                              QStringLiteral("丢弃 UDP 记录"),
                              QStringLiteral("确认丢弃当前 %1 行未保存记录？")
                                  .arg(m_csvRecorder.rowCount()),
                              QMessageBox::Discard | QMessageBox::Cancel,
                              QMessageBox::Cancel) != QMessageBox::Discard)
    {
        return;
    }

    m_csvRecorder.discard();
    if (m_udpMode && m_udpSocket->state() == QAbstractSocket::BoundState)
    {
        m_statusLabel->setText(QStringLiteral("未保存的 UDP 记录已丢弃，监听继续。"));
    }
    updateControlState();
}

void LogReplayWindow::updateRecordingState()
{
    const qint64 rowCount = m_csvRecorder.rowCount();
    switch (m_csvRecorder.state())
    {
    case JustFloatCsvRecorder::State::Idle:
        m_recordButton->setText(QStringLiteral("开始记录"));
        m_discardRecordingButton->setVisible(false);
        break;
    case JustFloatCsvRecorder::State::Recording:
        m_recordButton->setText(QStringLiteral("停止记录 (%1)").arg(rowCount));
        m_discardRecordingButton->setVisible(false);
        break;
    case JustFloatCsvRecorder::State::PendingSave:
        m_recordButton->setText(QStringLiteral("保存记录 (%1)").arg(rowCount));
        m_discardRecordingButton->setVisible(true);
        break;
    }
}

void LogReplayWindow::showWaveformWindow()
{
    bool created = false;
    if (m_waveformWindow == nullptr)
    {
        m_waveformWindow = new LogWaveformWindow(this);
        created = true;
    }

    m_waveformWindow->setLiveHistory(&m_waveformHistory);
    m_waveformWindow->setUdpMode(m_udpMode);
    if (!m_udpMode)
    {
        if (created)
        {
            m_waveformWindow->setCsvLog(m_log.rowCount() > 0 ? &m_log : nullptr);
        }
        m_waveformWindow->setCsvRow(m_currentRow);
    }
    m_waveformWindow->show();
    m_waveformWindow->raise();
    m_waveformWindow->activateWindow();
}

void LogReplayWindow::importCsv()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入 JustFloat CSV"),
                                                      QString(),
                                                      QStringLiteral("CSV (*.csv);;所有文件 (*.*)"));
    if (path.isEmpty())
    {
        return;
    }
    (void)loadCsv(path);
}

void LogReplayWindow::loadCarPlanFile(int slot)
{
    if (slot < 0 || slot >= CarPlanSlotCount)
    {
        return;
    }

    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("加载 CarPlan 源文件"),
                                                      QString(),
                                                      QStringLiteral("CarPlan (car_plan.c);;C 文件 (*.c);;所有文件 (*.*)"));
    if (!path.isEmpty())
    {
        (void)loadCarPlanPath(slot, path);
    }
}

void LogReplayWindow::loadCarPlanDirectory(int slot)
{
    if (slot < 0 || slot >= CarPlanSlotCount)
    {
        return;
    }

    const QString path = QFileDialog::getExistingDirectory(this,
                                                           QStringLiteral("选择包含 car_plan.c 的目录"),
                                                           QString());
    if (!path.isEmpty())
    {
        (void)loadCarPlanPath(slot, path);
    }
}

bool LogReplayWindow::loadCarPlanPath(int slot, const QString& path)
{
    if (slot < 0 || slot >= CarPlanSlotCount)
    {
        return false;
    }

    QString error;
    if (!m_carPlanRunners[slot].loadSourcePath(path, carPlanBuildDir(), &error))
    {
        QMessageBox::critical(this, QStringLiteral("CarPlan 加载失败"), error);
        m_carPlanStatusLabels[slot]->setText(QStringLiteral("CarPlan %1: load failed").arg(carPlanSlotName(slot)));
        updateControlState();
        return false;
    }

    clearCarPlanCache(slot);
    m_carPlanStatusLabels[slot]->setText(QStringLiteral("CarPlan %1: %2")
                                             .arg(carPlanSlotName(slot),
                                                  QFileInfo(m_carPlanRunners[slot].sourcePath()).fileName()));
    updateCarPlanForCurrentRow();
    renderCurrentRow();
    updateInfoText();
    updateControlState();
    return true;
}

void LogReplayWindow::resetCarPlan()
{
    resetCarPlanState();
    updateCarPlanForCurrentRow();
    renderCurrentRow();
    updateInfoText();
}

void LogReplayWindow::resetCarPlanState()
{
    for (int slot = 0; slot < CarPlanSlotCount; ++slot)
    {
        if (m_carPlanRunners[slot].isLoaded())
        {
            m_carPlanRunners[slot].reset();
        }
        clearCarPlanCache(slot);
    }
}

void LogReplayWindow::clearCarPlanCache(int slot)
{
    if (slot < 0 || slot >= CarPlanSlotCount)
    {
        return;
    }

    m_carPlanCaches[slot].clear();
    m_currentCarPlanResults[slot] = {};
    m_hasCurrentCarPlanResults[slot] = false;
}

bool LogReplayWindow::loadCsv(const QString& path)
{
    JustFloatLog log;
    QString error;
    if (!JustFloatLog::loadCsv(path, &log, &error))
    {
        QMessageBox::critical(this, QStringLiteral("导入失败"), error);
        return false;
    }

    m_timer->stop();
    m_playing = false;
    m_playButton->setText(QStringLiteral("播放"));
    if (m_udpMode)
    {
        if (!setUdpMode(false))
        {
            return false;
        }
        const QSignalBlocker blocker(m_modeCombo);
        m_modeCombo->setCurrentIndex(0);
    }
    m_log = log;
    m_currentRow = -1;
    m_focusCamera = -1;
    resetCarPlanState();

    const int lastRow = qMax(0, m_log.rowCount() - 1);
    m_slider->setRange(0, lastRow);
    m_frameSpin->setRange(0, lastRow);
    m_statusLabel->setText(QStringLiteral("%1 | %2 行")
                               .arg(QFileInfo(path).fileName())
                               .arg(m_log.rowCount()));
    if (m_waveformWindow != nullptr)
    {
        m_waveformWindow->setUdpMode(false);
        m_waveformWindow->setCsvLog(&m_log);
    }
    updateCameraVisibility();
    setCurrentRow(0);
    updateControlState();
    return true;
}

void LogReplayWindow::updateCarPlanForCurrentRow()
{
    for (int slot = 0; slot < CarPlanSlotCount; ++slot)
    {
        m_currentCarPlanResults[slot] = {};
        m_hasCurrentCarPlanResults[slot] = false;
        if (!m_carPlanRunners[slot].isLoaded())
        {
            continue;
        }

        if (m_udpMode)
        {
            if (m_hasLiveRow)
            {
                updateCarPlanFromLiveRow(slot, m_liveRow);
            }
            continue;
        }

        if (m_currentRow >= 0 && m_currentRow < m_log.rowCount())
        {
            (void)ensureCarPlanResultForCsvRow(slot, m_currentRow);
        }
    }
}

void LogReplayWindow::updateCarPlanFromLiveRow(const JustFloatLogRow& row)
{
    for (int slot = 0; slot < CarPlanSlotCount; ++slot)
    {
        updateCarPlanFromLiveRow(slot, row);
    }
}

void LogReplayWindow::updateCarPlanFromLiveRow(int slot, const JustFloatLogRow& row)
{
    if (slot < 0 || slot >= CarPlanSlotCount)
    {
        return;
    }

    m_currentCarPlanResults[slot] = {};
    m_hasCurrentCarPlanResults[slot] = false;
    if (!m_carPlanRunners[slot].isLoaded())
    {
        return;
    }

    m_hasCurrentCarPlanResults[slot] =
        m_carPlanRunners[slot].update(row, &m_currentCarPlanResults[slot]);
}

bool LogReplayWindow::ensureCarPlanResultForCsvRow(int slot, int row)
{
    if (slot < 0 ||
        slot >= CarPlanSlotCount ||
        !m_carPlanRunners[slot].isLoaded() ||
        row < 0 ||
        row >= m_log.rowCount())
    {
        return false;
    }

    QVector<CarPlanResult>& cache = m_carPlanCaches[slot];
    if (row < cache.size())
    {
        m_currentCarPlanResults[slot] = cache[row];
        m_hasCurrentCarPlanResults[slot] = true;
        return true;
    }

    if (cache.isEmpty())
    {
        m_carPlanRunners[slot].reset();
    }

    for (int i = cache.size(); i <= row; ++i)
    {
        CarPlanResult result;
        if (!m_carPlanRunners[slot].update(m_log.rowAt(i), &result))
        {
            clearCarPlanCache(slot);
            return false;
        }
        cache.push_back(result);
    }

    m_currentCarPlanResults[slot] = cache[row];
    m_hasCurrentCarPlanResults[slot] = true;
    return true;
}

void LogReplayWindow::setCurrentRow(int row)
{
    if (m_udpMode)
    {
        return;
    }
    if (m_log.rowCount() <= 0)
    {
        return;
    }

    const int clamped = qBound(0, row, m_log.rowCount() - 1);
    if (m_currentRow == clamped)
    {
        return;
    }

    m_currentRow = clamped;
    {
        const QSignalBlocker blockSlider(m_slider);
        const QSignalBlocker blockSpin(m_frameSpin);
        m_slider->setValue(m_currentRow);
        m_frameSpin->setValue(m_currentRow);
    }
    updateCarPlanForCurrentRow();
    if (m_waveformWindow != nullptr)
    {
        m_waveformWindow->setCsvRow(m_currentRow);
    }
    renderCurrentRow();
    updateInfoText();
}

void LogReplayWindow::renderCurrentRow()
{
    for (int i = 0; i < CameraCount; ++i)
    {
        renderCamera(i);
    }
}

void LogReplayWindow::renderCamera(int cameraIndex)
{
    const JustFloatLogRow* row = currentRow();
    if (row == nullptr || cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return;
    }

    const QImage gray = syntheticImageForCamera(cameraIndex);
    const beacon_result_t result = resultForCamera(cameraIndex);
    QImage rendered = FrameRenderer::render(gray, result, {}, 1, true);
    for (int slot = 0; slot < CarPlanSlotCount; ++slot)
    {
        drawCarPlanOverlay(&rendered, cameraIndex, slot);
    }
    m_videoWidgets[cameraIndex]->setPixelSourceImage(gray);
    m_videoWidgets[cameraIndex]->setFrameGeometry(QSize(BEACON_IMAGE_W, BEACON_IMAGE_H), 1);
    m_videoWidgets[cameraIndex]->setImage(rendered);

    const JustFloatCameraFrame& camera = row->cameras[cameraIndex];
    m_cameraInfoLabels[cameraIndex]->setText(QStringLiteral("B0:%1  B1:%2  CAR:%3")
                                                 .arg(camera.beacons[0].valid ? QStringLiteral("有效") : QStringLiteral("无"))
                                                 .arg(camera.beacons[1].valid ? QStringLiteral("有效") : QStringLiteral("无"))
                                                 .arg(camera.carLamp.valid ? QStringLiteral("有效") : QStringLiteral("无")));
}

void LogReplayWindow::togglePlayback()
{
    if (m_udpMode)
    {
        return;
    }
    if (m_log.rowCount() <= 0)
    {
        return;
    }

    m_playing = !m_playing;
    m_playButton->setText(m_playing ? QStringLiteral("暂停") : QStringLiteral("播放"));
    if (m_playing)
    {
        if (m_currentRow >= m_log.rowCount() - 1)
        {
            setCurrentRow(0);
        }
        scheduleNextFrame();
    }
    else
    {
        m_timer->stop();
    }
}

void LogReplayWindow::advancePlayback()
{
    if (!m_playing)
    {
        return;
    }

    if (m_currentRow >= m_log.rowCount() - 1)
    {
        m_playing = false;
        m_playButton->setText(QStringLiteral("播放"));
        return;
    }

    setCurrentRow(m_currentRow + 1);
    scheduleNextFrame();
}

void LogReplayWindow::scheduleNextFrame()
{
    if (m_playing)
    {
        m_timer->start(playbackIntervalMs());
    }
}

int LogReplayWindow::playbackIntervalMs() const
{
    if (m_currentRow < 0 || m_currentRow + 1 >= m_log.rowCount())
    {
        return 20;
    }

    const double now = rowPlaybackTimeMs(m_log.rowAt(m_currentRow));
    const double next = rowPlaybackTimeMs(m_log.rowAt(m_currentRow + 1));
    double delta = next - now;
    if (!std::isfinite(delta) || delta < 1.0 || delta > 1000.0)
    {
        delta = 20.0;
    }

    const double speed = qMax(0.1, m_speedCombo->currentData().toDouble());
    return qBound(1, (int)std::lround(delta / speed), 1000);
}

void LogReplayWindow::setFocusCamera(int cameraIndex)
{
    if (cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return;
    }
    m_focusCamera = cameraIndex;
    updateCameraVisibility();
}

void LogReplayWindow::showAllCameras()
{
    m_focusCamera = -1;
    updateCameraVisibility();
}

void LogReplayWindow::updateCameraVisibility()
{
    for (int i = 0; i < CameraCount; ++i)
    {
        m_cameraGroups[i]->setVisible(m_focusCamera < 0 || m_focusCamera == i);
    }
    m_returnGridButton->setEnabled(m_focusCamera >= 0);
}

void LogReplayWindow::updateInfoText()
{
    const JustFloatLogRow* row = currentRow();
    if (row == nullptr)
    {
        m_infoText->setText(m_udpMode ? QStringLiteral("UDP 未收到有效数据") :
                                        QStringLiteral("未导入日志"));
        return;
    }

    QString text;
    if (m_udpMode)
    {
        text += QStringLiteral("UDP Frame %1 | Packets %2 | Errors %3 | Peer %4\n")
                    .arg(m_currentRow)
                    .arg(m_udpPacketCount)
                    .arg(m_udpErrorCount)
                    .arg(m_lastUdpPeer.isEmpty() ? QStringLiteral("-") : m_lastUdpPeer);
    }
    else
    {
        text += QStringLiteral("Frame %1 / %2\n").arg(m_currentRow).arg(m_log.rowCount() - 1);
    }
    text += QStringLiteral("I0: %1  Sync: %2 ms\n")
                .arg(row->rowTime, 0, 'f', 3)
                .arg(row->syncTimeMs, 0, 'f', 3);
    text += QStringLiteral("Pitch: %1  Roll: %2  Yaw: %3\n")
                .arg(row->pitch, 0, 'f', 3)
                .arg(row->roll, 0, 'f', 3)
                .arg(row->yaw, 0, 'f', 3);
    if (row->hasProjectionDistance)
    {
        text += QStringLiteral("投影距离: X=%1 cm  Y=%2 cm\n")
                    .arg(row->projectionXcm, 0, 'f', 3)
                    .arg(row->projectionYcm, 0, 'f', 3);
    }
    else
    {
        text += QStringLiteral("投影距离: 旧日志无 I38/I39\n");
    }
    for (int slot = 0; slot < CarPlanSlotCount; ++slot)
    {
        text += carPlanInfoText(slot);
    }

    for (int camera = 0; camera < CameraCount; ++camera)
    {
        const JustFloatCameraFrame& cameraFrame = row->cameras[camera];
        text += QStringLiteral("%1: ").arg(CameraNames[camera]);
        for (int i = 0; i < 2; ++i)
        {
            const JustFloatBeacon& beacon = cameraFrame.beacons[i];
            text += QStringLiteral("B%1[%2 x=%3 y=%4 area=%5] ")
                        .arg(i)
                        .arg(beacon.valid ? QStringLiteral("有效") : QStringLiteral("无"))
                        .arg(beacon.x, 0, 'f', 2)
                        .arg(beacon.y, 0, 'f', 2)
                        .arg(beacon.area, 0, 'f', 2);
        }
        const JustFloatCarLamp& lamp = cameraFrame.carLamp;
        text += QStringLiteral("CAR[%1 cx=%2 cy=%3 angle=%4 w=%5 len=%6]\n")
                    .arg(lamp.valid ? QStringLiteral("有效") : QStringLiteral("无"))
                    .arg(lamp.cx, 0, 'f', 2)
                    .arg(lamp.cy, 0, 'f', 2)
                    .arg(lamp.angle, 0, 'f', 2)
                    .arg(lamp.width, 0, 'f', 2)
                    .arg(lamp.length, 0, 'f', 2);
    }
    m_infoText->setText(text);
}

void LogReplayWindow::drawCarPlanOverlay(QImage* image, int cameraIndex, int slot) const
{
    const JustFloatLogRow* row = currentRow();
    if (image == nullptr ||
        slot < 0 ||
        slot >= CarPlanSlotCount ||
        row == nullptr ||
        !m_hasCurrentCarPlanResults[slot] ||
        !m_currentCarPlanResults[slot].valid ||
        m_currentCarPlanResults[slot].camera != cameraIndex ||
        m_currentCarPlanResults[slot].beaconIndex < 0 ||
        m_currentCarPlanResults[slot].beaconIndex >= 2)
    {
        return;
    }

    const JustFloatCameraFrame& camera = row->cameras[cameraIndex];
    const JustFloatBeacon& beacon = camera.beacons[m_currentCarPlanResults[slot].beaconIndex];
    if (!beacon.valid)
    {
        return;
    }

    QPainter painter(image);
    painter.setRenderHint(QPainter::Antialiasing, false);
    QPen pen(carPlanSlotColor(slot));
    pen.setWidth(2);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);

    const QPointF beaconPoint = FrameRenderer::algorithmToImagePoint(displayXForCamera(cameraIndex, beacon.x),
                                                                      beacon.y);
    const qreal radius = qMax(4.0, std::sqrt(qMax(0.0f, beacon.area) / Pi) + 3.0);
    painter.drawEllipse(beaconPoint, radius, radius);
    painter.drawLine(QPointF(beaconPoint.x() - radius - 2.0, beaconPoint.y()),
                     QPointF(beaconPoint.x() + radius + 2.0, beaconPoint.y()));
    painter.drawLine(QPointF(beaconPoint.x(), beaconPoint.y() - radius - 2.0),
                     QPointF(beaconPoint.x(), beaconPoint.y() + radius + 2.0));

    if (camera.carLamp.valid)
    {
        const QPointF lampPoint = FrameRenderer::algorithmToImagePoint(displayXForCamera(cameraIndex,
                                                                                         camera.carLamp.cx),
                                                                       camera.carLamp.cy);
        painter.drawLine(lampPoint, beaconPoint);
    }

    painter.drawText(beaconPoint + QPointF(4.0, -4.0 - 10.0 * slot),
                     QStringLiteral("PLAN %1 B%2")
                         .arg(carPlanSlotName(slot))
                         .arg(m_currentCarPlanResults[slot].beaconIndex));
}

bool LogReplayWindow::carPlanRelationAngleDeg(int slot, float* angleDeg) const
{
    const JustFloatLogRow* row = currentRow();
    if (angleDeg == nullptr ||
        slot < 0 ||
        slot >= CarPlanSlotCount ||
        row == nullptr ||
        !m_hasCurrentCarPlanResults[slot] ||
        !m_currentCarPlanResults[slot].valid ||
        m_currentCarPlanResults[slot].camera < 0 ||
        m_currentCarPlanResults[slot].camera >= CameraCount ||
        m_currentCarPlanResults[slot].beaconIndex < 0 ||
        m_currentCarPlanResults[slot].beaconIndex >= 2)
    {
        return false;
    }

    const JustFloatCameraFrame& camera = row->cameras[m_currentCarPlanResults[slot].camera];
    const JustFloatBeacon& beacon = camera.beacons[m_currentCarPlanResults[slot].beaconIndex];
    const JustFloatCarLamp& lamp = camera.carLamp;
    if (!beacon.valid || !lamp.valid)
    {
        return false;
    }

    const float dx = beacon.x - lamp.cx;
    const float dy = beacon.y - lamp.cy;
    if (std::hypot(dx, dy) < 0.001f)
    {
        return false;
    }

    const float beaconAngle = qRadiansToDegrees(std::atan2(dy, dx));
    *angleDeg = normalizeAngleDeg(beaconAngle - lamp.angle);
    return true;
}

QString LogReplayWindow::carPlanInfoText() const
{
    if (!m_carPlanRunners[0].isLoaded())
    {
        return QStringLiteral("CarPlan: 未加载\n");
    }
    if (!m_hasCurrentCarPlanResults[0])
    {
        return QStringLiteral("CarPlan: 已加载，当前帧未计算\n");
    }
    if (!m_currentCarPlanResults[0].valid)
    {
        return QStringLiteral("CarPlan: invalid\n");
    }

    float relationAngle = 0.0f;
    const bool hasAngle = carPlanRelationAngleDeg(0, &relationAngle);
    return QStringLiteral("CarPlan: valid  %1 B%2  strafe=%3  forward=%4  dist=%5  along=%6  perp=%7  angle=%8\n")
        .arg(CameraNames.value(m_currentCarPlanResults[0].camera, QStringLiteral("Camera")))
        .arg(m_currentCarPlanResults[0].beaconIndex)
        .arg(m_currentCarPlanResults[0].targetStrafeMps, 0, 'f', 3)
        .arg(m_currentCarPlanResults[0].targetForwardMps, 0, 'f', 3)
        .arg(m_currentCarPlanResults[0].distPx, 0, 'f', 2)
        .arg(m_currentCarPlanResults[0].along, 0, 'f', 3)
        .arg(m_currentCarPlanResults[0].perp, 0, 'f', 3)
        .arg(hasAngle ? QString::number(relationAngle, 'f', 2) : QStringLiteral("--"));
}

QString LogReplayWindow::carPlanInfoText(int slot) const
{
    if (slot < 0 || slot >= CarPlanSlotCount)
    {
        return QString();
    }

    const QString prefix = QStringLiteral("CarPlan %1").arg(carPlanSlotName(slot));
    if (!m_carPlanRunners[slot].isLoaded())
    {
        return QStringLiteral("%1: unloaded\n").arg(prefix);
    }
    if (!m_hasCurrentCarPlanResults[slot])
    {
        return QStringLiteral("%1: loaded, current frame not calculated\n").arg(prefix);
    }
    if (!m_currentCarPlanResults[slot].valid)
    {
        return QStringLiteral("%1: invalid\n").arg(prefix);
    }

    float relationAngle = 0.0f;
    const bool hasAngle = carPlanRelationAngleDeg(slot, &relationAngle);
    const CarPlanResult& result = m_currentCarPlanResults[slot];
    return QStringLiteral("%1: valid  %2 B%3  strafe=%4  forward=%5  dist=%6  along=%7  perp=%8  angle=%9\n")
        .arg(prefix,
             CameraNames.value(result.camera, QStringLiteral("Camera")))
        .arg(result.beaconIndex)
        .arg(result.targetStrafeMps, 0, 'f', 3)
        .arg(result.targetForwardMps, 0, 'f', 3)
        .arg(result.distPx, 0, 'f', 2)
        .arg(result.along, 0, 'f', 3)
        .arg(result.perp, 0, 'f', 3)
        .arg(hasAngle ? QString::number(relationAngle, 'f', 2) : QStringLiteral("--"));
}

const JustFloatLogRow* LogReplayWindow::currentRow() const
{
    if (m_udpMode)
    {
        return m_hasLiveRow ? &m_liveRow : nullptr;
    }
    if (m_currentRow < 0 || m_currentRow >= m_log.rowCount())
    {
        return nullptr;
    }
    return &m_log.rowAt(m_currentRow);
}

beacon_result_t LogReplayWindow::resultForCamera(int cameraIndex) const
{
    beacon_result_t result;
    memset(&result, 0, sizeof(result));
    const JustFloatLogRow* row = currentRow();
    if (row == nullptr || cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return result;
    }

    const JustFloatCameraFrame& camera = row->cameras[cameraIndex];
    int beaconCount = 0;
    for (int i = 0; i < 2; ++i)
    {
        const JustFloatBeacon& source = camera.beacons[i];
        if (!source.valid)
        {
            continue;
        }
        result.beacons[i].x = displayXForCamera(cameraIndex, source.x);
        result.beacons[i].y = source.y;
        result.beacons[i].radius = std::sqrt(qMax(0.0f, source.area) / Pi);
        result.beacons[i].valid = 1;
        result.circles[i] = result.beacons[i];
        beaconCount = i + 1;
    }
    result.beacon_count = (unsigned char)beaconCount;
    result.count = (unsigned char)beaconCount;

    if (camera.carLamp.valid)
    {
        result.car_lamps[0].cx = displayXForCamera(cameraIndex, camera.carLamp.cx);
        result.car_lamps[0].cy = camera.carLamp.cy;
        result.car_lamps[0].angle = displayAngleForCamera(cameraIndex, camera.carLamp.angle);
        result.car_lamps[0].width = camera.carLamp.width;
        result.car_lamps[0].length = camera.carLamp.length;
        result.car_lamps[0].valid = 1;
        result.car_lamp_count = 1;
    }
    return result;
}

QImage LogReplayWindow::syntheticImageForCamera(int cameraIndex) const
{
    QImage image(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    image.fill(0);
    const JustFloatLogRow* row = currentRow();
    if (row == nullptr || cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return image;
    }

    const JustFloatCameraFrame& camera = row->cameras[cameraIndex];
    for (const JustFloatBeacon& beacon : camera.beacons)
    {
        if (!beacon.valid)
        {
            continue;
        }

        const QPointF center = FrameRenderer::algorithmToImagePoint(displayXForCamera(cameraIndex, beacon.x),
                                                                     beacon.y);
        const double radius = qMax(1.0, std::sqrt(qMax(0.0f, beacon.area) / Pi));
        const int extent = (int)std::ceil(radius + 4.0);
        const int cx = (int)std::lround(center.x());
        const int cy = (int)std::lround(center.y());
        for (int y = cy - extent; y <= cy + extent; ++y)
        {
            for (int x = cx - extent; x <= cx + extent; ++x)
            {
                const double distance = std::hypot((double)x - center.x(), (double)y - center.y());
                if (distance <= radius)
                {
                    setGrayMax(&image, x, y, 245);
                }
                else if (distance <= radius + 4.0)
                {
                    const double fade = 1.0 - (distance - radius) / 4.0;
                    setGrayMax(&image, x, y, 30 + (int)std::lround(90.0 * fade));
                }
            }
        }
    }

    if (camera.carLamp.valid)
    {
        const QPointF center = FrameRenderer::algorithmToImagePoint(
            displayXForCamera(cameraIndex, camera.carLamp.cx),
            camera.carLamp.cy);
        const double radians = qDegreesToRadians((double)displayAngleForCamera(cameraIndex,
                                                                               camera.carLamp.angle));
        const double cosA = std::cos(radians);
        const double sinA = std::sin(radians);
        const double halfLength = qMax(1.0, (double)camera.carLamp.length * 0.5);
        const double halfWidth = qMax(1.0, (double)camera.carLamp.width * 0.5);
        const int extent = (int)std::ceil(std::hypot(halfLength, halfWidth)) + 2;
        const int cx = (int)std::lround(center.x());
        const int cy = (int)std::lround(center.y());
        for (int y = cy - extent; y <= cy + extent; ++y)
        {
            for (int x = cx - extent; x <= cx + extent; ++x)
            {
                const double dx = (double)x - center.x();
                const double dy = (double)y - center.y();
                const double major = dx * cosA + dy * sinA;
                const double minor = -dx * sinA + dy * cosA;
                if (std::abs(major) <= halfLength &&
                    std::abs(minor) <= halfWidth)
                {
                    setGrayMax(&image, x, y, 190);
                }
            }
        }
    }

    return image;
}
