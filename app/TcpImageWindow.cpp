#include "TcpImageWindow.h"

#include "BeaconResultUtils.h"
#include "FrameRenderer.h"
#include "HorizonCalibration.h"
#include "LogWaveformWindow.h"
#include "VideoWidget.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFont>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPen>
#include <QPushButton>
#include <QPixmap>
#include <QStandardPaths>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QVBoxLayout>

#include <QtConcurrent>

#include <cmath>
#include <limits>

namespace
{
constexpr int PitchWaveformChannel = 34;
constexpr int RollWaveformChannel = 35;
constexpr qint64 AttitudeTcpTimeoutMs = 200;
constexpr float InvalidWaveformValue = -1000.0f;

QString defaultRecordName(quint16 port)
{
    return QStringLiteral("tcp_%1_port%2.avi")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")))
        .arg(port);
}

QString defaultCalibrationName(quint16 port, quint8 cameraId)
{
    return QStringLiteral("tcp_%1_port%2_%3.hcal.json")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")))
        .arg(port)
        .arg(cameraId == HorizonCameraFront ? QStringLiteral("front")
             : cameraId == HorizonCameraBack ? QStringLiteral("back")
                                            : QStringLiteral("down"));
}

bool frameAttitude(const BimgImageFrame& frame,
                   double* rollDeg,
                   double* pitchDeg,
                   bool* valid)
{
    const BimgDebugFloat* roll = nullptr;
    const BimgDebugFloat* pitch = nullptr;
    for (const BimgDebugFloat& debugFloat : frame.debugFloats)
    {
        if (debugFloat.id == BimgDebugRollDegId)
        {
            roll = &debugFloat;
        }
        else if (debugFloat.id == BimgDebugPitchDegId)
        {
            pitch = &debugFloat;
        }
    }
    if (roll == nullptr || pitch == nullptr)
    {
        return false;
    }
    if (rollDeg != nullptr)
    {
        *rollDeg = roll->value;
    }
    if (pitchDeg != nullptr)
    {
        *pitchDeg = pitch->value;
    }
    if (valid != nullptr)
    {
        *valid = roll->valid && pitch->valid
                 && std::isfinite(roll->value) && std::isfinite(pitch->value);
    }
    return true;
}

bool frameHeight(const BimgImageFrame& frame, double* heightMm, bool* valid)
{
    for (const BimgDebugFloat& debugFloat : frame.debugFloats)
    {
        if (debugFloat.id != BimgDebugHeightMmId)
        {
            continue;
        }
        if (heightMm != nullptr)
        {
            *heightMm = debugFloat.value;
        }
        if (valid != nullptr)
        {
            *valid = debugFloat.valid && std::isfinite(debugFloat.value);
        }
        return true;
    }
    return false;
}

QString streamModeName(quint8 mode)
{
    switch (mode)
    {
        case 0: return QStringLiteral("Raw");
        case 1: return QStringLiteral("Lamp Binary");
        case 2: return QStringLiteral("Beacon Binary");
        case 3: return QStringLiteral("Detected Overlay");
        default: return QStringLiteral("Unknown");
    }
}

void drawChipMarkers(QImage* image, const QVector<BimgImageMarker>& markers)
{
    if (image == nullptr || image->isNull())
    {
        return;
    }

    QPainter painter(image);
    painter.setRenderHint(QPainter::Antialiasing, false);
    for (const BimgImageMarker& marker : markers)
    {
        const QColor color = marker.type == BimgMarkerType::Beacon
                                 ? QColor(0, 255, 80)
                                 : QColor(255, 95, 45);
        painter.setPen(QPen(color, 1));
        const QPoint center(marker.x, marker.y);
        painter.drawLine(center + QPoint(-3, -3), center + QPoint(3, 3));
        painter.drawLine(center + QPoint(-3, 3), center + QPoint(3, -3));
    }
}

}

TcpImageWindow::TcpImageWindow(QWidget* parent)
    : QWidget(parent),
      m_server(new QTcpServer(this)),
      m_calibrationRecorder(new HorizonCalibrationRecorder(this))
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    setWindowTitle(QStringLiteral("TCP 图像监视"));
    resize(900, 680);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    auto* configLayout = new QFormLayout;
    configLayout->setLabelAlignment(Qt::AlignRight);
    m_addressCombo = new QComboBox(this);
    m_addressCombo->setEditable(false);
    m_portEdit = new QLineEdit(QStringLiteral("8086"), this);
    m_listenButton = new QPushButton(QStringLiteral("开始监听"), this);
    configLayout->addRow(QStringLiteral("本机 IP"), m_addressCombo);
    configLayout->addRow(QStringLiteral("本机端口"), m_portEdit);
    configLayout->addRow(QString(), m_listenButton);
    root->addLayout(configLayout);

    m_statusLabel = new QLabel(QStringLiteral("未监听"), this);
    m_statusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_statusLabel->setWordWrap(true);
    root->addWidget(m_statusLabel);

    auto* attitudeLayout = new QHBoxLayout;
    attitudeLayout->setSpacing(12);
    auto* attitudeTitle = new QLabel(QStringLiteral("姿态"), this);
    QFont attitudeTitleFont = attitudeTitle->font();
    attitudeTitleFont.setBold(true);
    attitudeTitle->setFont(attitudeTitleFont);
    m_attitudeRollLabel = new QLabel(QStringLiteral("Roll -- deg"), this);
    m_attitudePitchLabel = new QLabel(QStringLiteral("Pitch -- deg"), this);
    m_attitudeStateLabel = new QLabel(QStringLiteral("未提供"), this);
    m_attitudeStateLabel->setAlignment(Qt::AlignCenter);
    m_attitudeStateLabel->setMinimumWidth(82);
    m_heightLabel = new QLabel(QStringLiteral("Height -- mm"), this);
    m_heightStateLabel = new QLabel(QStringLiteral("未提供"), this);
    m_heightStateLabel->setAlignment(Qt::AlignCenter);
    m_heightStateLabel->setMinimumWidth(82);
    m_attitudeWaveformButton = new QPushButton(QStringLiteral("姿态波形"), this);
    m_attitudeWaveformButton->setEnabled(false);
    attitudeLayout->addWidget(attitudeTitle);
    attitudeLayout->addWidget(m_attitudeRollLabel);
    attitudeLayout->addWidget(m_attitudePitchLabel);
    attitudeLayout->addWidget(m_attitudeStateLabel);
    attitudeLayout->addWidget(m_heightLabel);
    attitudeLayout->addWidget(m_heightStateLabel);
    attitudeLayout->addWidget(m_attitudeWaveformButton);
    attitudeLayout->addStretch(1);
    root->addLayout(attitudeLayout);

    auto* body = new QHBoxLayout;
    body->setSpacing(10);
    m_videoWidget = new VideoWidget(this);
    m_videoWidget->setText(QStringLiteral("等待 TCP 图像"));
    body->addWidget(m_videoWidget, 1);

    m_diagnosticPanel = new QWidget(this);
    m_diagnosticPanel->setFixedWidth(310);
    auto* diagnosticLayout = new QVBoxLayout(m_diagnosticPanel);
    diagnosticLayout->setContentsMargins(10, 0, 0, 0);
    diagnosticLayout->setSpacing(7);
    auto* diagnosticTitle = new QLabel(QStringLiteral("区域参数诊断"), m_diagnosticPanel);
    QFont titleFont = diagnosticTitle->font();
    titleFont.setBold(true);
    diagnosticTitle->setFont(titleFont);
    diagnosticLayout->addWidget(diagnosticTitle);
    m_diagnosticStateLabel = new QLabel(QStringLiteral("等待 Raw 图传、参数快照和至少 10 帧缓存。"), m_diagnosticPanel);
    m_diagnosticStateLabel->setWordWrap(true);
    diagnosticLayout->addWidget(m_diagnosticStateLabel);
    m_diagnosticParameterLabel = new QLabel(m_diagnosticPanel);
    m_diagnosticParameterLabel->setWordWrap(true);
    m_diagnosticParameterLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    diagnosticLayout->addWidget(m_diagnosticParameterLabel);
    m_diagnosticEffectLabel = new QLabel(m_diagnosticPanel);
    m_diagnosticEffectLabel->setWordWrap(true);
    diagnosticLayout->addWidget(m_diagnosticEffectLabel);
    m_diagnosticStatsLabel = new QLabel(m_diagnosticPanel);
    m_diagnosticStatsLabel->setWordWrap(true);
    diagnosticLayout->addWidget(m_diagnosticStatsLabel);
    diagnosticLayout->addWidget(new QLabel(QStringLiteral("修改前"), m_diagnosticPanel));
    m_diagnosticBeforePreview = new QLabel(m_diagnosticPanel);
    m_diagnosticBeforePreview->setAlignment(Qt::AlignCenter);
    m_diagnosticBeforePreview->setFixedSize(282, 180);
    m_diagnosticBeforePreview->setStyleSheet(QStringLiteral("background: #090a0b; border: 1px solid #2a3035;"));
    diagnosticLayout->addWidget(m_diagnosticBeforePreview);
    diagnosticLayout->addWidget(new QLabel(QStringLiteral("建议值预览"), m_diagnosticPanel));
    m_diagnosticAfterPreview = new QLabel(m_diagnosticPanel);
    m_diagnosticAfterPreview->setAlignment(Qt::AlignCenter);
    m_diagnosticAfterPreview->setFixedSize(282, 180);
    m_diagnosticAfterPreview->setStyleSheet(QStringLiteral("background: #090a0b; border: 1px solid #2a3035;"));
    diagnosticLayout->addWidget(m_diagnosticAfterPreview);
    diagnosticLayout->addStretch(1);
    body->addWidget(m_diagnosticPanel);
    root->addLayout(body, 1);

    auto* controls = new QHBoxLayout;
    controls->setSpacing(8);
    m_pauseButton = new QPushButton(QStringLiteral("暂停显示"), this);
    auto* saveButton = new QPushButton(QStringLiteral("保存当前帧"), this);
    auto* dirButton = new QPushButton(QStringLiteral("保存目录"), this);
    m_recordButton = new QPushButton(QStringLiteral("开始录像"), this);
    m_calibrationRecordButton = new QPushButton(QStringLiteral("开始标定录像"), this);
    m_calibrationCameraCombo = new QComboBox(this);
    m_calibrationCameraCombo->addItem(QStringLiteral("标定相机：自动"), -1);
    m_calibrationCameraCombo->addItem(QStringLiteral("标定相机：Down"), (int)HorizonCameraDown);
    m_diagnosticButton = new QPushButton(QStringLiteral("区域诊断"), this);
    m_viewModeCombo = new QComboBox(this);
    m_viewModeCombo->addItem(QStringLiteral("原图"), QStringLiteral("original"));
    m_viewModeCombo->addItem(QStringLiteral("二值图"), QStringLiteral("binary"));
    m_carLampModeCombo = new QComboBox(this);
    m_carLampModeCombo->addItem(
        QStringLiteral("单车灯"), static_cast<int>(CarLampMode::Single));
    m_carLampModeCombo->addItem(
        QStringLiteral("双车灯"), static_cast<int>(CarLampMode::Dual));
    m_enableInstanceCheck = new QCheckBox(QStringLiteral("启用实例"), this);
    m_instanceCombo = new QComboBox(this);
    m_overlayCheck = new QCheckBox(QStringLiteral("检测覆盖"), this);
    m_overlayCheck->setChecked(true);
    m_autoSaveCheck = new QCheckBox(QStringLiteral("自动保存帧"), this);

    controls->addWidget(m_pauseButton);
    controls->addWidget(saveButton);
    controls->addWidget(dirButton);
    controls->addWidget(m_recordButton);
    controls->addWidget(m_calibrationRecordButton);
    controls->addWidget(m_calibrationCameraCombo);
    controls->addWidget(m_diagnosticButton);
    controls->addSpacing(12);
    controls->addWidget(m_viewModeCombo);
    controls->addWidget(m_carLampModeCombo);
    controls->addWidget(m_enableInstanceCheck);
    controls->addWidget(m_instanceCombo);
    controls->addWidget(m_overlayCheck);
    controls->addWidget(m_autoSaveCheck);
    controls->addStretch(1);
    root->addLayout(controls);

    connect(m_server, &QTcpServer::newConnection, this, &TcpImageWindow::acceptPendingConnections);
    connect(m_listenButton, &QPushButton::clicked, this, [this]() {
        if (m_server->isListening())
        {
            stopListening();
        }
        else
        {
            startListening();
        }
    });
    connect(m_pauseButton, &QPushButton::clicked, this, [this]() {
        m_paused = !m_paused;
        m_pauseButton->setText(m_paused ? QStringLiteral("继续显示") : QStringLiteral("暂停显示"));
        updateStatus(m_result);
    });
    connect(saveButton, &QPushButton::clicked, this, &TcpImageWindow::saveCurrentFrame);
    connect(dirButton, &QPushButton::clicked, this, &TcpImageWindow::chooseSaveDirectory);
    connect(m_recordButton, &QPushButton::clicked, this, [this]() {
        if (m_recording)
        {
            stopRecording();
        }
        else
        {
            startRecording();
        }
    });
    connect(m_calibrationRecordButton, &QPushButton::clicked, this, [this]() {
        if (m_calibrationRecorder->isAccepting())
        {
            stopCalibrationRecording();
        }
        else if (!m_calibrationFinalizing)
        {
            startCalibrationRecording();
        }
    });
    connect(m_calibrationRecorder,
            &HorizonCalibrationRecorder::recordingFinished,
            this,
            [this](const QString& path, int frameCount, quint64 droppedFrames) {
                m_calibrationFinalizing = false;
                m_activeCalibrationCameraId = 0xffU;
                m_calibrationRecordButton->setEnabled(true);
                m_calibrationRecordButton->setText(QStringLiteral("开始标定录像"));
                updateStatus(m_result);
                QMessageBox::information(this,
                                         QStringLiteral("标定录像完成"),
                                         QStringLiteral("已写入 %1 帧，丢弃/缺失 %2 帧。\n%3")
                                             .arg(frameCount)
                                             .arg(droppedFrames)
                                             .arg(path));
            });
    connect(m_calibrationRecorder,
            &HorizonCalibrationRecorder::recordingFailed,
            this,
            [this](const QString& message) {
                m_calibrationFinalizing = false;
                m_activeCalibrationCameraId = 0xffU;
                m_calibrationRecordButton->setEnabled(true);
                m_calibrationRecordButton->setText(QStringLiteral("开始标定录像"));
                updateStatus(m_result);
                QMessageBox::critical(this, QStringLiteral("标定录像失败"), message);
            });
    connect(m_viewModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &TcpImageWindow::render);
    connect(m_carLampModeCombo,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            [this]() { applyCarLampMode(carLampMode()); });
    connect(m_enableInstanceCheck, &QCheckBox::toggled, this, [this]() {
        applyCarLampMode(m_carLampMode);
    });
    connect(m_instanceCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        applyCarLampMode(m_carLampMode);
    });
    connect(m_overlayCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_showOverlay = checked;
        render();
    });
    connect(m_autoSaveCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_autoSave = checked;
    });
    connect(m_attitudeWaveformButton,
            &QPushButton::clicked,
            this,
            &TcpImageWindow::showAttitudeWaveform);
    connect(m_diagnosticButton, &QPushButton::clicked, this, &TcpImageWindow::toggleRegionDiagnostic);
    connect(m_videoWidget,
            &VideoWidget::correctionShapeFinished,
            this,
            &TcpImageWindow::handleDiagnosticRegion);
    m_diagnosticWatcher = new QFutureWatcher<BeaconDiagnosticResult>(this);
    connect(m_diagnosticWatcher,
            &QFutureWatcher<BeaconDiagnosticResult>::finished,
            this,
            &TcpImageWindow::finishRegionDiagnostic);

    m_attitudeTimer = new QTimer(this);
    m_attitudeTimer->setInterval(50);
    connect(m_attitudeTimer, &QTimer::timeout, this, [this]() {
        if (m_attitudeProvided && m_attitudeLastMs >= 0 && m_attitudeElapsed.isValid()
            && m_attitudeElapsed.elapsed() - m_attitudeLastMs > AttitudeTcpTimeoutMs
            && !m_attitudeTimeoutGapWritten)
        {
            appendAttitudeSample(false);
            m_attitudeTimeoutGapWritten = true;
        }
        refreshAttitudeDisplay();
    });
    m_attitudeTimer->start();

    refreshAttitudeDisplay();
    updateStatus(m_result);
}

TcpImageWindow::~TcpImageWindow()
{
    m_calibrationRecorder->finish();
    stopRecording(false);
    stopListening();
}

quint16 TcpImageWindow::port() const
{
    return m_port;
}

void TcpImageWindow::setAvailableAddresses(const QVector<TcpListenAddress>& addresses)
{
    m_addressCombo->clear();
    for (const TcpListenAddress& address : addresses)
    {
        if (!address.address.isEmpty())
        {
            m_addressCombo->addItem(address.label, address.address);
        }
    }
    if (m_addressCombo->count() == 0)
    {
        m_addressCombo->addItem(QStringLiteral("所有 IPv4 地址 - 0.0.0.0"), QStringLiteral("0.0.0.0"));
    }
}

void TcpImageWindow::setInstanceOptions(const QVector<TcpInstanceOption>& options)
{
    m_instances = options;
    m_instanceCombo->clear();
    for (const TcpInstanceOption& option : m_instances)
    {
        m_instanceCombo->addItem(option.name, option.id);
    }
    m_enableInstanceCheck->setEnabled(!m_instances.isEmpty());
    m_instanceCombo->setEnabled(!m_instances.isEmpty());
}

void TcpImageWindow::setDefaultSaveDirectory(const QString& path)
{
    m_saveDir = path;
}

void TcpImageWindow::setSuggestedPort(quint16 port)
{
    if (!m_server->isListening() && port > 0)
    {
        m_portEdit->setText(QString::number(port));
    }
}

void TcpImageWindow::startListening()
{
    bool ok = false;
    const int portValue = m_portEdit->text().trimmed().toInt(&ok);
    if (!ok || portValue < 1 || portValue > 65535)
    {
        QMessageBox::warning(this, QStringLiteral("TCP 监听失败"), QStringLiteral("本机端口无效。"));
        return;
    }

    const QHostAddress address(m_addressCombo->currentData().toString());
    m_port = (quint16)portValue;
    if (!m_server->listen(address, m_port))
    {
        QMessageBox::critical(this,
                              QStringLiteral("TCP 监听失败"),
                              QStringLiteral("监听 %1:%2 失败：%3")
                                  .arg(address.toString())
                                  .arg(m_port)
                                  .arg(m_server->errorString()));
        updateStatus(m_result);
        return;
    }

    QString historyError;
    m_attitudeHistoryError = !m_attitudeHistory.beginSession(&historyError);
    if (m_attitudeHistoryError)
    {
        QMessageBox::warning(this,
                             QStringLiteral("姿态波形不可用"),
                             QStringLiteral("TCP图像接收仍会继续，但本次姿态波形无法记录。\n%1")
                                 .arg(historyError));
    }
    m_attitudeElapsed.restart();
    m_attitudeLastMs = -1;
    m_heightLastMs = -1;
    m_attitudeCameraId = 0xffU;
    m_attitudeProvided = false;
    m_attitudeHasValue = false;
    m_attitudeValid = false;
    m_heightProvided = false;
    m_heightHasValue = false;
    m_heightValid = false;
    m_attitudeTimeoutGapWritten = false;
    m_attitudeWaveformButton->setEnabled(!m_attitudeHistoryError);
    if (m_attitudeWaveformWindow != nullptr)
    {
        m_attitudeWaveformWindow->setLiveHistory(&m_attitudeHistory);
        m_attitudeWaveformWindow->configureLiveSource(
            attitudeSourceName(), {PitchWaveformChannel, RollWaveformChannel});
    }
    refreshAttitudeDisplay();

    m_addressCombo->setEnabled(false);
    m_portEdit->setEnabled(false);
    m_listenButton->setText(QStringLiteral("停止监听"));
    setWindowTitle(QStringLiteral("TCP 图像监视 - %1:%2").arg(address.toString()).arg(m_port));
    updateStatus(m_result);
}

void TcpImageWindow::stopListening()
{
    const QList<QTcpSocket*> sockets = m_parsers.keys();
    for (QTcpSocket* socket : sockets)
    {
        removeSocket(socket);
    }
    if (m_server->isListening())
    {
        m_server->close();
    }
    m_addressCombo->setEnabled(true);
    m_portEdit->setEnabled(true);
    m_listenButton->setText(QStringLiteral("开始监听"));
    updateStatus(m_result);
}

void TcpImageWindow::acceptPendingConnections()
{
    while (m_server->hasPendingConnections())
    {
        QTcpSocket* socket = m_server->nextPendingConnection();
        if (socket == nullptr)
        {
            continue;
        }

        auto* parser = new BimgImageFrameParser;
        m_parsers.insert(socket, parser);
        connect(socket, &QTcpSocket::readyRead, this, [this, socket]() {
            readSocketData(socket);
        });
        connect(socket, &QTcpSocket::disconnected, this, [this, socket]() {
            removeSocket(socket);
        });
        connect(socket, &QTcpSocket::errorOccurred, this, [this, socket]() {
            m_statusLabel->setText(QStringLiteral("TCP 连接错误：%1").arg(socket->errorString()));
        });

        m_peerName = QStringLiteral("%1:%2")
                         .arg(socket->peerAddress().toString())
                         .arg(socket->peerPort());
        updateStatus(m_result);
    }
}

void TcpImageWindow::readSocketData(QTcpSocket* socket)
{
    BimgImageFrameParser* parser = m_parsers.value(socket, nullptr);
    if (socket == nullptr || parser == nullptr)
    {
        return;
    }

    const QByteArray data = socket->readAll();
    const BimgParseBatch batch = parser->append(data);
    m_crcErrorCount = parser->crcErrorCount();
    m_protocolErrorCount = parser->protocolErrorCount();
    const QString peerName = QStringLiteral("%1:%2")
                                 .arg(socket->peerAddress().toString())
                                 .arg(socket->peerPort());
    for (const BimgParameterSnapshot& snapshot : batch.parameterSnapshots)
    {
        setParameterSnapshot(snapshot);
    }
    for (const BimgImageFrame& frame : batch.frames)
    {
        setFrame(frame, peerName, QDateTime::currentMSecsSinceEpoch());
    }
    if (batch.isEmpty())
    {
        updateStatus(m_result);
    }
}

void TcpImageWindow::setParameterSnapshot(const BimgParameterSnapshot& snapshot)
{
    const auto existing = m_parameterSnapshots.constFind(snapshot.cameraId);
    if (existing == m_parameterSnapshots.cend()
        || existing->revision != snapshot.revision
        || existing->algorithmBuildId != snapshot.algorithmBuildId)
    {
        m_recentRawFrames.clear();
    }
    m_parameterSnapshots.insert(snapshot.cameraId, snapshot);
    if (!m_diagnosticFrozen && m_diagnosticStateLabel != nullptr)
    {
        m_diagnosticStateLabel->setText(
            QStringLiteral("已同步相机 %1 参数：修订 %2，构建 ID 0x%3。")
                .arg(snapshot.cameraId == 0U ? QStringLiteral("Front") : QStringLiteral("Back"))
                .arg(snapshot.revision)
                .arg(snapshot.algorithmBuildId, 8, 16, QLatin1Char('0')));
    }
}

void TcpImageWindow::removeSocket(QTcpSocket* socket)
{
    if (socket == nullptr)
    {
        return;
    }
    delete m_parsers.take(socket);
    socket->deleteLater();
    updateStatus(m_result);
}

void TcpImageWindow::setFrame(const BimgImageFrame& frame,
                              const QString& peerName,
                              qint64 hostTimeMs)
{
    if (frame.image.isNull())
    {
        return;
    }

    updateAttitude(frame);
    m_peerName = peerName;
    const QImage gray = frame.image.convertToFormat(QImage::Format_Grayscale8);
    m_lastReceivedFrame = frame;
    m_lastReceivedGrayImage = gray;
    appendCalibrationFrame(frame, gray);
    appendRecordingFrame(frame, gray, hostTimeMs);
    if (frame.protocol == ImageFrameProtocol::Bimg && frame.streamMode == 0U)
    {
        m_recentRawFrames.push_back(gray);
        while (m_recentRawFrames.size() > 50)
        {
            m_recentRawFrames.removeFirst();
        }
    }
    if (m_diagnosticFrozen)
    {
        updateStatus(m_result);
        return;
    }
    if (m_paused)
    {
        updateStatus(m_result);
        return;
    }

    m_streamFrame = frame;
    m_grayImage = gray;
    ++m_frameIndex;
    processCurrentFrame();
    render();

    if (m_autoSave)
    {
        const QString path = defaultSavePath(QStringLiteral(".png"));
        QDir().mkpath(QFileInfo(path).absolutePath());
        m_renderedImage.save(path, "PNG");
    }
}

void TcpImageWindow::processCurrentFrame()
{
    if (m_grayImage.isNull())
    {
        m_result = {};
        m_processProfile = {};
        m_horizon = {};
        return;
    }
    if (m_streamFrame.streamMode == 3U)
    {
        m_result = {};
        m_processProfile = {};
        m_horizon = {};
    }
    else
    {
        AlgorithmRunner* runner = selectedRunner();
        if (runner != nullptr)
        {
            runner->setCarLampMode(m_carLampMode);
            AlgorithmFrameTelemetry telemetry;
            telemetry.cameraId = m_streamFrame.cameraId;
            telemetry.rollDeg = m_attitudeRollDeg;
            telemetry.pitchDeg = m_attitudePitchDeg;
            telemetry.heightMm = m_heightMm;
            telemetry.attitudeValid = m_attitudeProvided && m_attitudeValid;
            telemetry.heightValid = m_heightProvided && m_heightValid;
            runner->setFrameTelemetry(telemetry);
            m_result = runner->process(m_grayImage);
            m_processProfile = runner->lastProcessProfile();
            m_horizon = runner->horizonCurve();
        }
        else
        {
            m_result = {};
            m_processProfile = {};
            m_horizon = {};
        }
    }
}

void TcpImageWindow::updateAttitude(const BimgImageFrame& frame)
{
    const BimgDebugFloat* roll = nullptr;
    const BimgDebugFloat* pitch = nullptr;
    const BimgDebugFloat* height = nullptr;

    if (frame.protocol == ImageFrameProtocol::Bimg && frame.cameraId <= 1U
        && frame.cameraId != m_attitudeCameraId)
    {
        m_attitudeCameraId = frame.cameraId;
        if (m_attitudeWaveformWindow != nullptr)
        {
            m_attitudeWaveformWindow->configureLiveSource(
                attitudeSourceName(), {PitchWaveformChannel, RollWaveformChannel});
        }
    }
    for (const BimgDebugFloat& debugFloat : frame.debugFloats)
    {
        if (debugFloat.id == BimgDebugRollDegId)
        {
            roll = &debugFloat;
        }
        else if (debugFloat.id == BimgDebugPitchDegId)
        {
            pitch = &debugFloat;
        }
        else if (debugFloat.id == BimgDebugHeightMmId)
        {
            height = &debugFloat;
        }
    }

    if (!m_attitudeElapsed.isValid() && (roll != nullptr || pitch != nullptr || height != nullptr))
    {
        m_attitudeElapsed.start();
    }
    const qint64 nowMs = m_attitudeElapsed.isValid() ? m_attitudeElapsed.elapsed() : -1;
    if (roll == nullptr || pitch == nullptr)
    {
        if (m_attitudeProvided && !m_attitudeTimeoutGapWritten)
        {
            appendAttitudeSample(false);
        }
        m_attitudeProvided = false;
        m_attitudeValid = false;
        m_attitudeLastMs = -1;
        m_attitudeTimeoutGapWritten = true;
    }
    else
    {
        const bool finite = std::isfinite(roll->value) && std::isfinite(pitch->value);
        if (finite)
        {
            m_attitudeRollDeg = roll->value;
            m_attitudePitchDeg = pitch->value;
            m_attitudeHasValue = true;
        }
        m_attitudeProvided = true;
        m_attitudeValid = finite && roll->valid && pitch->valid;
        m_attitudeLastMs = nowMs;
        m_attitudeTimeoutGapWritten = false;
        appendAttitudeSample(m_attitudeValid);
    }

    if (height == nullptr)
    {
        m_heightProvided = false;
        m_heightValid = false;
        m_heightLastMs = -1;
    }
    else
    {
        const bool finite = std::isfinite(height->value);
        if (finite)
        {
            m_heightMm = height->value;
            m_heightHasValue = true;
        }
        m_heightProvided = true;
        m_heightValid = finite && height->valid;
        m_heightLastMs = nowMs;
    }
    refreshAttitudeDisplay();
}

void TcpImageWindow::appendAttitudeSample(bool valid)
{
    if (m_attitudeHistoryError || !m_attitudeHistory.isActive())
    {
        return;
    }

    const qint64 elapsedMs = m_attitudeElapsed.isValid() ? m_attitudeElapsed.elapsed() : 0;
    JustFloatLogRow row;
    row.rowTime = elapsedMs;
    row.syncTimeMs = elapsedMs;
    row.pitch = valid ? m_attitudePitchDeg : InvalidWaveformValue;
    row.roll = valid ? m_attitudeRollDeg : InvalidWaveformValue;
    row.yaw = InvalidWaveformValue;

    QString error;
    if (!m_attitudeHistory.append(row, elapsedMs, &error))
    {
        m_attitudeHistoryError = true;
        m_attitudeWaveformButton->setEnabled(false);
        m_attitudeStateLabel->setToolTip(error);
    }
}

void TcpImageWindow::refreshAttitudeDisplay()
{
    m_attitudeRollLabel->setText(
        m_attitudeHasValue
            ? QStringLiteral("Roll %1 deg").arg(m_attitudeRollDeg, 0, 'f', 2)
            : QStringLiteral("Roll -- deg"));
    m_attitudePitchLabel->setText(
        m_attitudeHasValue
            ? QStringLiteral("Pitch %1 deg").arg(m_attitudePitchDeg, 0, 'f', 2)
            : QStringLiteral("Pitch -- deg"));
    m_heightLabel->setText(
        m_heightHasValue
            ? QStringLiteral("Height %1 mm").arg(m_heightMm, 0, 'f', 1)
            : QStringLiteral("Height -- mm"));

    QString state = QStringLiteral("未提供");
    QString color = QStringLiteral("#8b949e");
    if (m_attitudeProvided && m_attitudeLastMs >= 0 && m_attitudeElapsed.isValid())
    {
        if (m_attitudeElapsed.elapsed() - m_attitudeLastMs > AttitudeTcpTimeoutMs)
        {
            state = QStringLiteral("TCP超时");
            color = QStringLiteral("#f87171");
        }
        else if (m_attitudeValid)
        {
            state = QStringLiteral("有效");
            color = QStringLiteral("#4ade80");
        }
        else
        {
            state = QStringLiteral("姿态超时");
            color = QStringLiteral("#facc15");
        }
    }
    m_attitudeStateLabel->setText(state);
    m_attitudeStateLabel->setStyleSheet(
        QStringLiteral("color:%1; border:1px solid %1; border-radius:4px; padding:2px 8px;")
            .arg(color));

    state = QStringLiteral("未提供");
    color = QStringLiteral("#8b949e");
    if (m_heightProvided && m_heightLastMs >= 0 && m_attitudeElapsed.isValid())
    {
        if (m_attitudeElapsed.elapsed() - m_heightLastMs > AttitudeTcpTimeoutMs)
        {
            state = QStringLiteral("TCP超时");
            color = QStringLiteral("#f87171");
        }
        else if (m_heightValid)
        {
            state = QStringLiteral("有效");
            color = QStringLiteral("#4ade80");
        }
        else
        {
            state = QStringLiteral("高度无效");
            color = QStringLiteral("#facc15");
        }
    }
    m_heightStateLabel->setText(state);
    m_heightStateLabel->setStyleSheet(
        QStringLiteral("color:%1; border:1px solid %1; border-radius:4px; padding:2px 8px;")
            .arg(color));
}

void TcpImageWindow::showAttitudeWaveform()
{
    if (m_attitudeHistoryError)
    {
        return;
    }
    if (m_attitudeWaveformWindow == nullptr)
    {
        m_attitudeWaveformWindow = new LogWaveformWindow(this);
    }
    m_attitudeWaveformWindow->setLiveHistory(&m_attitudeHistory);
    m_attitudeWaveformWindow->configureLiveSource(
        attitudeSourceName(), {PitchWaveformChannel, RollWaveformChannel});
    m_attitudeWaveformWindow->setUdpMode(true);
    m_attitudeWaveformWindow->show();
    m_attitudeWaveformWindow->raise();
    m_attitudeWaveformWindow->activateWindow();
}

QString TcpImageWindow::attitudeSourceName() const
{
    QString camera;
    if (m_attitudeCameraId == 0U)
    {
        camera = QStringLiteral(" Front");
    }
    else if (m_attitudeCameraId == 1U)
    {
        camera = QStringLiteral(" Back");
    }
    return QStringLiteral("TCP%1 :%2").arg(camera).arg(m_port);
}

void TcpImageWindow::render()
{
    if (m_grayImage.isNull() || m_videoWidget == nullptr)
    {
        return;
    }

    AlgorithmRunner* runner = selectedRunner();
    QImage displayImage = m_grayImage;
    if (m_streamFrame.streamMode != 3U
        && runner != nullptr
        && m_viewModeCombo->currentData().toString() == QStringLiteral("binary"))
    {
        const QImage binary = runner->binaryImage(m_grayImage);
        if (!binary.isNull())
        {
            displayImage = binary;
        }
    }

    if (m_streamFrame.streamMode == 3U)
    {
        m_renderedImage = displayImage.convertToFormat(QImage::Format_RGB32);
        if (m_showOverlay)
        {
            drawChipMarkers(&m_renderedImage, m_streamFrame.markers);
        }
    }
    else if (runner != nullptr)
    {
        QVector<CorrectionShape> corrections;
        AnnotationModel* annotations = selectedAnnotations();
        if (annotations != nullptr)
        {
            corrections = annotations->correctionsForFrame(qMax(0, m_frameIndex));
        }
        m_renderedImage = FrameRenderer::render(displayImage,
                                                m_result,
                                                corrections,
                                                 1,
                                                 m_showOverlay,
                                                 &m_horizon,
                                                 m_carLampMode);
    }
    else
    {
        m_renderedImage = displayImage.convertToFormat(QImage::Format_RGB32);
    }
    m_videoWidget->setFrameGeometry(m_grayImage.size(), 1);
    m_videoWidget->setPixelSourceImage(m_grayImage);
    m_videoWidget->setImage(m_renderedImage);
    updateStatus(m_result);
}

void TcpImageWindow::toggleRegionDiagnostic()
{
    if (m_diagnosticWatcher != nullptr && m_diagnosticWatcher->isRunning())
    {
        return;
    }
    if (m_diagnosticFrozen)
    {
        resumeLiveDisplay();
        return;
    }
    if (m_streamFrame.protocol != ImageFrameProtocol::Bimg || m_streamFrame.streamMode != 0U)
    {
        QMessageBox::information(this,
                                 QStringLiteral("区域诊断"),
                                 QStringLiteral("请先在 2BL3 Img > Stream 中切换为 Raw。"));
        return;
    }
    if (!m_parameterSnapshots.contains(m_streamFrame.cameraId))
    {
        QMessageBox::information(this,
                                 QStringLiteral("区域诊断"),
                                 QStringLiteral("尚未收到该相机的 BPAR 参数快照。"));
        return;
    }
    if (m_recentRawFrames.size() < 10)
    {
        QMessageBox::information(this,
                                 QStringLiteral("区域诊断"),
                                 QStringLiteral("当前只有 %1 帧 Raw 缓存，至少需要 10 帧。")
                                     .arg(m_recentRawFrames.size()));
        return;
    }
    m_diagnosticFrozen = true;
    m_diagnosticFrames = m_recentRawFrames;
    m_diagnosticGrayImage = m_grayImage;
    m_diagnosticRegion = QRectF();
    m_videoWidget->setCorrectionTool(QStringLiteral("rect"));
    m_videoWidget->setCorrectionStyle(QColor(60, 220, 255), 2);
    m_diagnosticButton->setText(QStringLiteral("取消诊断"));
    m_diagnosticStateLabel->setText(
        QStringLiteral("画面已冻结。请拖动矩形框选一个误检或漏检区域。"));
    m_diagnosticParameterLabel->clear();
    m_diagnosticEffectLabel->clear();
    m_diagnosticStatsLabel->setText(QStringLiteral("将回放最近 %1 帧。")
                                        .arg(m_diagnosticFrames.size()));
}

void TcpImageWindow::handleDiagnosticRegion(const QString& shapeType,
                                            const QVector<QPointF>& points)
{
    if (!m_diagnosticFrozen || shapeType != QStringLiteral("rect") || points.size() < 2
        || (m_diagnosticWatcher != nullptr && m_diagnosticWatcher->isRunning()))
    {
        return;
    }

    m_diagnosticRegion = QRectF(points[0], points[1]).normalized()
                             .intersected(QRectF(QPointF(0, 0), m_diagnosticGrayImage.size()));
    if (m_diagnosticRegion.width() < 2.0 || m_diagnosticRegion.height() < 2.0)
    {
        m_diagnosticStateLabel->setText(QStringLiteral("框选区域过小，请重新拖动。"));
        return;
    }

    m_videoWidget->setCorrectionTool(QStringLiteral("select"));
    m_diagnosticButton->setEnabled(false);
    m_diagnosticStateLabel->setText(QStringLiteral("正在离线回放并搜索安全单参数方案..."));

    BeaconDiagnosticRequest request;
    request.frames = m_diagnosticFrames;
    request.region = m_diagnosticRegion;
    request.snapshot = m_parameterSnapshots.value(m_streamFrame.cameraId);
    request.firmwareImageDirectory = AlgorithmRunner::defaultTwoBl3ImageDirectory();
    request.buildDirectory = QDir(QStandardPaths::writableLocation(QStandardPaths::CacheLocation))
                                 .absoluteFilePath(QStringLiteral("2bl3-diagnostic"));
    m_diagnosticWatcher->setFuture(QtConcurrent::run([request]() {
        return BeaconParameterDiagnostic::analyze(request);
    }));
}

void TcpImageWindow::finishRegionDiagnostic()
{
    if (m_diagnosticWatcher == nullptr)
    {
        return;
    }
    updateDiagnosticPanel(m_diagnosticWatcher->result());
    m_diagnosticButton->setEnabled(true);
    m_diagnosticButton->setText(QStringLiteral("继续实时"));
}

void TcpImageWindow::resumeLiveDisplay()
{
    m_diagnosticFrozen = false;
    m_diagnosticFrames.clear();
    m_diagnosticRegion = QRectF();
    m_videoWidget->setCorrectionTool(QStringLiteral("select"));
    m_diagnosticButton->setEnabled(true);
    m_diagnosticButton->setText(QStringLiteral("区域诊断"));
    if (!m_recentRawFrames.isEmpty())
    {
        m_grayImage = m_recentRawFrames.last();
        processCurrentFrame();
        render();
    }
}

void TcpImageWindow::updateDiagnosticPanel(const BeaconDiagnosticResult& result)
{
    const QString problem = result.falsePositive ? QStringLiteral("误检") : QStringLiteral("漏检");
    m_diagnosticStateLabel->setText(QStringLiteral("%1：%2").arg(problem, result.message));
    m_diagnosticStatsLabel->setText(
        QStringLiteral("区域 %1 像素 | 平均灰度 %2 | 最大灰度 %3 | 回放 %4 帧")
            .arg(result.regionPixelCount)
            .arg(result.regionMeanGray, 0, 'f', 1)
            .arg(result.regionMaxGray)
            .arg(result.analyzedFrameCount));

    QImage before;
    if (!m_diagnosticGrayImage.isNull())
    {
        before = FrameRenderer::render(m_diagnosticGrayImage, result.beforeResult, {}, 1, true);
        QPainter painter(&before);
        painter.setPen(QPen(QColor(60, 220, 255), 1));
        painter.drawRect(m_diagnosticRegion);
    }
    setDiagnosticPreview(m_diagnosticBeforePreview, before);

    if (!result.recommendationFound)
    {
        m_diagnosticParameterLabel->setText(QStringLiteral("未生成参数修改建议。"));
        m_diagnosticEffectLabel->setText(
            QStringLiteral("曝光参数无法通过已采集帧验证；需要现场修改曝光后重新采集。"));
        m_diagnosticAfterPreview->clear();
        return;
    }

    m_diagnosticParameterLabel->setText(
        QStringLiteral("%1 > %2\n%3 -> %4")
            .arg(result.parameter.menuPath,
                 result.parameter.name,
                 TwoBl3ParameterCatalog::formatValue(result.parameter.type, result.currentValue),
                 TwoBl3ParameterCatalog::formatValue(result.parameter.type, result.recommendedValue)));
    m_diagnosticEffectLabel->setText(result.parameter.effect);

    QImage after = FrameRenderer::render(m_diagnosticGrayImage, result.afterResult, {}, 1, true);
    QPainter painter(&after);
    painter.setPen(QPen(QColor(60, 220, 255), 1));
    painter.drawRect(m_diagnosticRegion);
    setDiagnosticPreview(m_diagnosticAfterPreview, after);
}

void TcpImageWindow::setDiagnosticPreview(QLabel* label, const QImage& image)
{
    if (label == nullptr)
    {
        return;
    }
    if (image.isNull())
    {
        label->clear();
        return;
    }
    label->setPixmap(QPixmap::fromImage(image).scaled(label->size(),
                                                       Qt::KeepAspectRatio,
                                                       Qt::FastTransformation));
}

void TcpImageWindow::saveCurrentFrame()
{
    if (m_grayImage.isNull())
    {
        QMessageBox::information(this, QStringLiteral("保存当前帧"), QStringLiteral("尚未收到图像。"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("保存当前帧"),
                                                      defaultSavePath(QStringLiteral(".png")),
                                                      QStringLiteral("PNG Image (*.png)"));
    if (!path.isEmpty())
    {
        m_renderedImage.save(path, "PNG");
    }
}

void TcpImageWindow::chooseSaveDirectory()
{
    const QString path = QFileDialog::getExistingDirectory(this,
                                                           QStringLiteral("选择保存目录"),
                                                           m_saveDir.isEmpty() ? QDir::currentPath() : m_saveDir);
    if (!path.isEmpty())
    {
        m_saveDir = path;
    }
}

void TcpImageWindow::startRecording()
{
    if (m_lastReceivedGrayImage.isNull())
    {
        QMessageBox::information(this, QStringLiteral("开始录像"), QStringLiteral("请先等待收到第一帧图像。"));
        return;
    }
    if (m_lastReceivedFrame.protocol != ImageFrameProtocol::Bimg
        || m_lastReceivedFrame.protocolVersion != 3U
        || m_lastReceivedFrame.streamMode != 0U)
    {
        QMessageBox::information(
            this,
            QStringLiteral("开始录像"),
            QStringLiteral("同步录像只支持 BIMG v3 Raw 灰度帧，请先切换图传模式。"));
        return;
    }
    if ((m_lastReceivedFrame.sourceCameraId != 0U
         && m_lastReceivedFrame.sourceCameraId != 2U)
        || m_lastReceivedFrame.physicalBoardId > 1U)
    {
        QMessageBox::information(
            this,
            QStringLiteral("开始录像"),
            QStringLiteral("当前 BIMG v3 帧没有有效的前/后摄来源与物理板号，无法建立同步侧车。"));
        return;
    }

    const QString dir = m_saveDir.isEmpty()
        ? QDir(QDir::currentPath()).absoluteFilePath(QStringLiteral("tcp_frames"))
        : m_saveDir;
    QDir().mkpath(dir);
    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("保存录像 AVI"),
                                                      QDir(dir).absoluteFilePath(defaultRecordName(m_port)),
                                                      QStringLiteral("AVI Video (*.avi)"));
    if (path.isEmpty())
    {
        return;
    }

    const cv::Size size(m_lastReceivedGrayImage.width(), m_lastReceivedGrayImage.height());
    const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    bool writerOpened = false;
    QString writerError;
    try
    {
        writerOpened = m_writer.open(path.toStdString(), fourcc, 50.0, size, false);
    }
    catch (const cv::Exception& exception)
    {
        writerError = QString::fromLocal8Bit(exception.what());
    }
    if (!writerOpened)
    {
        const QString detail = writerError.isEmpty()
            ? QStringLiteral("无法创建 AVI 文件。")
            : QStringLiteral("无法创建 AVI 文件：%1").arg(writerError);
        QMessageBox::critical(this, QStringLiteral("开始录像失败"), detail);
        return;
    }

    QString sidecarError;
    if (!m_recordingSidecar.start(path, &sidecarError))
    {
        m_writer.release();
        QMessageBox::critical(
            this,
            QStringLiteral("开始录像失败"),
            QStringLiteral("无法创建逐帧侧车 %1：%2")
                .arg(imageFrameSidecarPathForVideo(path), sidecarError));
        return;
    }

    m_recordingFrameSize = m_lastReceivedGrayImage.size();
    m_recordingSourceCameraId = m_lastReceivedFrame.sourceCameraId;
    m_recordingPhysicalBoardId = m_lastReceivedFrame.physicalBoardId;
    m_recording = true;
    m_recordButton->setText(QStringLiteral("结束录像"));
    updateStatus(m_result);
}

void TcpImageWindow::stopRecording(bool reportError)
{
    QString error;
    if (m_writer.isOpened())
    {
        try
        {
            m_writer.release();
        }
        catch (const cv::Exception& exception)
        {
            error = QStringLiteral("结束 AVI 写入失败：%1")
                        .arg(QString::fromLocal8Bit(exception.what()));
        }
    }
    if (m_recordingSidecar.isActive())
    {
        QString sidecarError;
        if (!m_recordingSidecar.finish(&sidecarError) && error.isEmpty())
        {
            error = QStringLiteral("结束逐帧侧车写入失败：%1").arg(sidecarError);
        }
    }
    m_recording = false;
    m_recordingFrameSize = QSize();
    m_recordingSourceCameraId = 0xffU;
    m_recordingPhysicalBoardId = 0xffU;
    if (m_recordButton != nullptr)
    {
        m_recordButton->setText(QStringLiteral("开始录像"));
    }
    updateStatus(m_result);
    if (reportError && !error.isEmpty())
    {
        QMessageBox::critical(this, QStringLiteral("结束录像失败"), error);
    }
}

void TcpImageWindow::failRecording(const QString& message)
{
    stopRecording(false);
    QMessageBox::critical(this, QStringLiteral("同步录像已停止"), message);
}

void TcpImageWindow::appendRecordingFrame(const BimgImageFrame& frame,
                                          const QImage& gray,
                                          qint64 hostTimeMs)
{
    if (!m_recording)
    {
        return;
    }
    if (frame.protocol != ImageFrameProtocol::Bimg
        || frame.protocolVersion != 3U
        || frame.streamMode != 0U)
    {
        return;
    }
    if (!m_writer.isOpened() || !m_recordingSidecar.isActive())
    {
        failRecording(QStringLiteral("AVI 或逐帧侧车写入器意外关闭，已终止录像以避免帧错位。"));
        return;
    }
    if (frame.sourceCameraId != m_recordingSourceCameraId
        || frame.physicalBoardId != m_recordingPhysicalBoardId)
    {
        failRecording(
            QStringLiteral("录像来源发生变化：期望来源摄像头 %1 / 物理板 %2，实际为来源摄像头 %3 / 物理板 %4。"
                           "本次录像已结束，请为新的摄像头重新开始录像。")
                .arg(m_recordingSourceCameraId)
                .arg(m_recordingPhysicalBoardId)
                .arg(frame.sourceCameraId)
                .arg(frame.physicalBoardId));
        return;
    }
    if (gray.isNull() || gray.size() != m_recordingFrameSize)
    {
        failRecording(
            QStringLiteral("录像图像尺寸发生变化：期望 %1x%2，实际 %3x%4，已终止录像。")
                .arg(m_recordingFrameSize.width())
                .arg(m_recordingFrameSize.height())
                .arg(gray.width())
                .arg(gray.height()));
        return;
    }

    ImageFrameSidecarRecord record;
    record.videoFrameIndex = m_recordingSidecar.rowCount();
    record.hostTimeMs = hostTimeMs;
    record.bimgSequence = frame.sequence;
    record.sourceFrameSequence = frame.sourceFrameSequence;
    record.captureTimeMs = frame.captureTimeMs;
    record.sourceFrameValid = frame.sourceFrameValid;
    record.captureTimeValid = frame.captureTimeValid;
    record.sourceCameraId = frame.sourceCameraId;
    record.physicalBoardId = frame.physicalBoardId;

    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    bool attitudeValid = false;
    record.attitudeValid = frameAttitude(frame, &rollDeg, &pitchDeg, &attitudeValid)
                           && attitudeValid
                           && rollDeg >= -180.0 && rollDeg <= 180.0
                           && pitchDeg >= -180.0 && pitchDeg <= 180.0;
    if (record.attitudeValid)
    {
        record.rollDeg = static_cast<float>(rollDeg);
        record.pitchDeg = static_cast<float>(pitchDeg);
    }

    double heightMm = 0.0;
    bool heightValid = false;
    record.heightValid = frameHeight(frame, &heightMm, &heightValid)
                         && heightValid && heightMm >= 0.0 && heightMm <= 100000.0;
    if (record.heightValid)
    {
        record.heightMm = static_cast<float>(heightMm);
    }

    const QImage gray8 = gray.convertToFormat(QImage::Format_Grayscale8);
    cv::Mat grayMat(gray8.height(),
                    gray8.width(),
                    CV_8UC1,
                    const_cast<uchar*>(gray8.bits()),
                    static_cast<size_t>(gray8.bytesPerLine()));
    try
    {
        m_writer.write(grayMat);
    }
    catch (const cv::Exception& exception)
    {
        failRecording(QStringLiteral("写入 AVI 帧失败：%1")
                          .arg(QString::fromLocal8Bit(exception.what())));
        return;
    }
    if (!m_writer.isOpened())
    {
        failRecording(QStringLiteral("AVI 写入器在写帧后关闭，已终止录像以避免帧错位。"));
        return;
    }

    QString sidecarError;
    if (!m_recordingSidecar.append(record, &sidecarError))
    {
        failRecording(
            QStringLiteral("AVI 第 %1 帧已写入，但对应逐帧侧车追加失败：%2。已立即终止录像。")
                .arg(record.videoFrameIndex)
                .arg(sidecarError));
    }
}

void TcpImageWindow::startCalibrationRecording()
{
    if (m_lastReceivedGrayImage.isNull())
    {
        QMessageBox::information(this,
                                 QStringLiteral("开始标定录像"),
                                 QStringLiteral("请先等待收到第一帧图像。"));
        return;
    }
    if (m_lastReceivedFrame.protocol != ImageFrameProtocol::Bimg
        || (m_lastReceivedFrame.protocolVersion != 2U
            && m_lastReceivedFrame.protocolVersion != 3U)
        || m_lastReceivedFrame.streamMode != 0U)
    {
        QMessageBox::information(this,
                                 QStringLiteral("开始标定录像"),
                                 QStringLiteral("标定录像只支持 BIMG v2/v3 的 Raw 图像模式。"));
        return;
    }
    if (m_lastReceivedGrayImage.size() != QSize(188, 120))
    {
        QMessageBox::information(this,
                                 QStringLiteral("开始标定录像"),
                                 QStringLiteral("标定录像要求 188x120 原始图像。"));
        return;
    }
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double heightMm = 0.0;
    bool attitudeValid = false;
    bool heightValid = false;
    if (!frameAttitude(m_lastReceivedFrame, &rollDeg, &pitchDeg, &attitudeValid)
        || !frameHeight(m_lastReceivedFrame, &heightMm, &heightValid)
        || !attitudeValid || !heightValid)
    {
        QMessageBox::information(this,
                                 QStringLiteral("开始标定录像"),
                                 QStringLiteral("请等待当前帧同时提供有效的Roll、Pitch和融合高度。"));
        return;
    }

    const QString dir = m_saveDir.isEmpty()
        ? QDir(QDir::currentPath()).absoluteFilePath(QStringLiteral("tcp_frames"))
        : m_saveDir;
    QDir().mkpath(dir);
    QString sessionPath = QFileDialog::getSaveFileName(
        this,
        QStringLiteral("保存地平线标定会话"),
        QDir(dir).absoluteFilePath(defaultCalibrationName(m_port, calibrationCameraId())),
        QStringLiteral("Horizon Calibration (*.hcal.json)"));
    if (sessionPath.isEmpty())
    {
        return;
    }
    if (!sessionPath.endsWith(QStringLiteral(".hcal.json"), Qt::CaseInsensitive))
    {
        sessionPath += QStringLiteral(".hcal.json");
    }
    QString basePath = sessionPath;
    basePath.chop(10);
    const QString videoPath = basePath + QStringLiteral(".avi");
    const QString csvPath = basePath + QStringLiteral(".hcal.csv");
    if ((QFileInfo::exists(videoPath) || QFileInfo::exists(csvPath))
        && QMessageBox::question(this,
                                 QStringLiteral("覆盖标定文件"),
                                 QStringLiteral("同名 AVI 或 HCAL CSV 已存在，继续将覆盖这些文件。"))
               != QMessageBox::Yes)
    {
        return;
    }

    HorizonCalibrationRecorderConfig config;
    config.sessionPath = QFileInfo(sessionPath).absoluteFilePath();
    config.videoPath = QFileInfo(videoPath).absoluteFilePath();
    config.csvPath = QFileInfo(csvPath).absoluteFilePath();
    config.imageSize = m_lastReceivedGrayImage.size();
    config.cameraId = calibrationCameraId();
    config.sourceCameraId = m_lastReceivedFrame.cameraId;
    config.bimgProtocolVersion = m_lastReceivedFrame.protocolVersion;
    config.fps = 50.0;
    QString error;
    if (!m_calibrationRecorder->begin(config, &error))
    {
        QMessageBox::critical(this, QStringLiteral("开始标定录像失败"), error);
        return;
    }
    m_activeCalibrationCameraId = config.cameraId;
    m_calibrationRecordButton->setText(QStringLiteral("结束标定录像"));
    updateStatus(m_result);
}

void TcpImageWindow::stopCalibrationRecording()
{
    if (!m_calibrationRecorder->isAccepting())
    {
        return;
    }
    m_calibrationFinalizing = true;
    m_calibrationRecordButton->setEnabled(false);
    m_calibrationRecordButton->setText(QStringLiteral("正在结束标定录像..."));
    m_calibrationRecorder->finish();
    updateStatus(m_result);
}

void TcpImageWindow::appendCalibrationFrame(const BimgImageFrame& frame, const QImage& gray)
{
    if (!m_calibrationRecorder->isAccepting()
        || frame.protocol != ImageFrameProtocol::Bimg
        || (frame.protocolVersion != 2U && frame.protocolVersion != 3U)
        || frame.streamMode != 0U)
    {
        return;
    }

    HorizonCalibrationRecorderFrame record;
    record.image = gray;
    record.bimgSequence = frame.sequence;
    record.hostTimeMs = QDateTime::currentMSecsSinceEpoch();
    record.cameraId = m_activeCalibrationCameraId;
    record.sourceCameraId = frame.cameraId;
    record.bimgProtocolVersion = frame.protocolVersion;
    record.sourceFrameSequence = frame.sourceFrameSequence;
    record.captureTimeMs = frame.captureTimeMs;
    record.sourceFrameCameraId = frame.sourceCameraId;
    record.physicalBoardId = frame.physicalBoardId;
    record.sourceFrameValid = frame.sourceFrameValid;
    record.captureTimeValid = frame.captureTimeValid;
    record.rollDeg = std::numeric_limits<double>::quiet_NaN();
    record.pitchDeg = std::numeric_limits<double>::quiet_NaN();
    record.heightMm = std::numeric_limits<double>::quiet_NaN();
    record.attitudeValid = false;
    record.heightValid = false;
    frameAttitude(frame, &record.rollDeg, &record.pitchDeg, &record.attitudeValid);
    frameHeight(frame, &record.heightMm, &record.heightValid);
    m_calibrationRecorder->enqueue(record);
}

quint8 TcpImageWindow::calibrationCameraId() const
{
    const int selected = m_calibrationCameraCombo == nullptr
        ? -1 : m_calibrationCameraCombo->currentData().toInt();
    return selected == (int)HorizonCameraDown ? HorizonCameraDown : m_lastReceivedFrame.cameraId;
}

QString TcpImageWindow::defaultSavePath(const QString& suffix) const
{
    const QString dir = m_saveDir.isEmpty()
        ? QDir(QDir::currentPath()).absoluteFilePath(QStringLiteral("tcp_frames"))
        : m_saveDir;
    const QString fileName = QStringLiteral("tcp_%1_port%2_frame%3%4")
                                 .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")))
                                 .arg(m_port)
                                 .arg(qMax(0, m_frameIndex))
                                 .arg(suffix);
    return QDir(dir).absoluteFilePath(fileName);
}

void TcpImageWindow::updateStatus(const beacon_result_t& result)
{
    const QString listenText = m_server->isListening()
        ? QStringLiteral("监听 %1:%2").arg(m_server->serverAddress().toString()).arg(m_server->serverPort())
        : QStringLiteral("未监听");
    const QString peerText = m_peerName.isEmpty()
        ? QStringLiteral("WiFi SPI 未连接")
        : QStringLiteral("WiFi SPI %1").arg(m_peerName);
    const QString recordText = m_recording ? QStringLiteral("录像中") : QStringLiteral("未录像");
    const QString calibrationText = m_calibrationFinalizing
        ? QStringLiteral("标定录像收尾中")
        : m_calibrationRecorder->isAccepting()
            ? QStringLiteral("标定录像中：已写 %1，丢帧 %2")
                  .arg(m_calibrationRecorder->writtenFrameCount())
                  .arg(m_calibrationRecorder->droppedFrameCount())
            : QStringLiteral("未标定录像");
    int beaconMarkers = 0;
    int lampMarkers = 0;
    for (const BimgImageMarker& marker : m_streamFrame.markers)
    {
        if (marker.type == BimgMarkerType::Beacon)
        {
            ++beaconMarkers;
        }
        else if (marker.type == BimgMarkerType::CarLamp)
        {
            ++lampMarkers;
        }
    }
    const QString frameText = m_frameIndex < 0
        ? QStringLiteral("等待 BIMG / 逐飞助手图像帧")
        : QStringLiteral("协议 %1 | 相机 %2 | 模式 %3 | 序号 %4 | 标记 %5（信标 %6，车灯 %7）")
              .arg(m_streamFrame.protocol == ImageFrameProtocol::Bimg
                       ? QStringLiteral("BIMG v%1").arg(m_streamFrame.protocolVersion)
                       : QStringLiteral("逐飞助手"))
              .arg(m_streamFrame.cameraId == 0U ? QStringLiteral("Front") : QStringLiteral("Back"))
              .arg(streamModeName(m_streamFrame.streamMode))
              .arg(m_streamFrame.sequence)
              .arg(m_streamFrame.markers.size())
              .arg(beaconMarkers)
              .arg(lampMarkers);
    QString algorithmText;
    if (m_streamFrame.streamMode == 3U)
    {
        algorithmText = QStringLiteral("芯片识别结果");
    }
    else if (selectedRunner() == nullptr)
    {
        algorithmText = QStringLiteral("未启用桌面实例 | 直接显示接收图像");
    }
    else
    {
        const int displayedCarLampCount = qMin(
            BeaconResultUtils::carLampCount(result),
            m_carLampMode == CarLampMode::Dual ? 2 : 1);
        algorithmText = QStringLiteral("桌面检测：信标 %1，车灯 %2，总计 %3 | %4")
                            .arg(BeaconResultUtils::beaconCount(result))
                            .arg(displayedCarLampCount)
                            .arg(BeaconResultUtils::beaconCount(result) +
                                 displayedCarLampCount)
                            .arg(AlgorithmProcessProfiler::formatCompact(m_processProfile));
    }
    m_statusLabel->setText(QStringLiteral("%1 | %2 | %3 | %4 | CRC错误 %5 | 协议错误 %6 | %7 | %8")
                               .arg(listenText)
                               .arg(peerText)
                               .arg(frameText)
                               .arg(algorithmText)
                               .arg(m_crcErrorCount)
                               .arg(m_protocolErrorCount)
                               .arg(recordText)
                               .arg(calibrationText));
}

CarLampMode TcpImageWindow::carLampMode() const
{
    if (m_carLampModeCombo == nullptr)
    {
        return m_carLampMode;
    }
    return (m_carLampModeCombo->currentData().toInt() ==
            static_cast<int>(CarLampMode::Dual)) ?
        CarLampMode::Dual : CarLampMode::Single;
}

void TcpImageWindow::applyCarLampMode(CarLampMode mode)
{
    m_carLampMode = (mode == CarLampMode::Dual) ?
        CarLampMode::Dual : CarLampMode::Single;
    AlgorithmRunner* runner = selectedRunner();
    if (runner != nullptr)
    {
        runner->setCarLampMode(m_carLampMode);
        runner->resetTemporal();
    }
    m_result = {};
    m_processProfile = {};
    m_horizon = {};
    processCurrentFrame();
    render();
}

AlgorithmRunner* TcpImageWindow::selectedRunner() const
{
    if (!m_enableInstanceCheck->isChecked())
    {
        return nullptr;
    }

    const int id = m_instanceCombo->currentData().toInt();
    for (const TcpInstanceOption& option : m_instances)
    {
        if (option.id == id && option.runner != nullptr)
        {
            return option.runner;
        }
    }
    return nullptr;
}

AnnotationModel* TcpImageWindow::selectedAnnotations() const
{
    if (!m_enableInstanceCheck->isChecked())
    {
        return nullptr;
    }

    const int id = m_instanceCombo->currentData().toInt();
    for (const TcpInstanceOption& option : m_instances)
    {
        if (option.id == id)
        {
            return option.annotations;
        }
    }
    return nullptr;
}
