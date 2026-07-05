#include "TcpImageWindow.h"

#include "BeaconResultUtils.h"
#include "FrameRenderer.h"
#include "VideoWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTcpServer>
#include <QTcpSocket>
#include <QVBoxLayout>

#include <opencv2/imgproc.hpp>

namespace
{
QString defaultRecordName(quint16 port)
{
    return QStringLiteral("tcp_%1_port%2.avi")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")))
        .arg(port);
}

cv::Mat qImageToBgrMat(const QImage& image)
{
    const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(),
                   rgb.width(),
                   CV_8UC3,
                   const_cast<uchar*>(rgb.bits()),
                   (size_t)rgb.bytesPerLine());
    cv::Mat bgr;
    cv::cvtColor(rgbMat, bgr, cv::COLOR_RGB2BGR);
    return bgr;
}
}

TcpImageWindow::TcpImageWindow(QWidget* parent)
    : QWidget(parent),
      m_server(new QTcpServer(this))
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
    m_portEdit = new QLineEdit(QStringLiteral("1347"), this);
    m_listenButton = new QPushButton(QStringLiteral("开始监听"), this);
    configLayout->addRow(QStringLiteral("本机 IP"), m_addressCombo);
    configLayout->addRow(QStringLiteral("本机端口"), m_portEdit);
    configLayout->addRow(QString(), m_listenButton);
    root->addLayout(configLayout);

    m_statusLabel = new QLabel(QStringLiteral("未监听"), this);
    m_statusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_statusLabel->setWordWrap(true);
    root->addWidget(m_statusLabel);

    m_videoWidget = new VideoWidget(this);
    m_videoWidget->setText(QStringLiteral("等待 TCP 图像"));
    root->addWidget(m_videoWidget, 1);

    auto* controls = new QHBoxLayout;
    controls->setSpacing(8);
    m_pauseButton = new QPushButton(QStringLiteral("暂停显示"), this);
    auto* saveButton = new QPushButton(QStringLiteral("保存当前帧"), this);
    auto* dirButton = new QPushButton(QStringLiteral("保存目录"), this);
    m_recordButton = new QPushButton(QStringLiteral("开始录像"), this);
    m_viewModeCombo = new QComboBox(this);
    m_viewModeCombo->addItem(QStringLiteral("原图"), QStringLiteral("original"));
    m_viewModeCombo->addItem(QStringLiteral("二值图"), QStringLiteral("binary"));
    m_enableInstanceCheck = new QCheckBox(QStringLiteral("启用实例"), this);
    m_instanceCombo = new QComboBox(this);
    m_overlayCheck = new QCheckBox(QStringLiteral("检测覆盖"), this);
    m_overlayCheck->setChecked(true);
    m_autoSaveCheck = new QCheckBox(QStringLiteral("自动保存帧"), this);

    controls->addWidget(m_pauseButton);
    controls->addWidget(saveButton);
    controls->addWidget(dirButton);
    controls->addWidget(m_recordButton);
    controls->addSpacing(12);
    controls->addWidget(m_viewModeCombo);
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
    connect(m_viewModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &TcpImageWindow::render);
    connect(m_enableInstanceCheck, &QCheckBox::toggled, this, &TcpImageWindow::render);
    connect(m_instanceCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &TcpImageWindow::render);
    connect(m_overlayCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_showOverlay = checked;
        render();
    });
    connect(m_autoSaveCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_autoSave = checked;
    });

    updateStatus(m_result);
}

TcpImageWindow::~TcpImageWindow()
{
    stopRecording();
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

        auto* parser = new SeekfreeImageFrameParser;
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
    SeekfreeImageFrameParser* parser = m_parsers.value(socket, nullptr);
    if (socket == nullptr || parser == nullptr)
    {
        return;
    }

    const QByteArray data = socket->readAll();
    const QVector<SeekfreeImageFrame> frames = parser->append(data);
    const QString peerName = QStringLiteral("%1:%2")
                                 .arg(socket->peerAddress().toString())
                                 .arg(socket->peerPort());
    for (const SeekfreeImageFrame& frame : frames)
    {
        setFrame(frame.image, peerName);
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

void TcpImageWindow::setFrame(const QImage& grayImage, const QString& peerName)
{
    if (grayImage.isNull())
    {
        return;
    }

    m_peerName = peerName;
    if (m_paused)
    {
        updateStatus(m_result);
        return;
    }

    m_grayImage = grayImage.convertToFormat(QImage::Format_Grayscale8);
    ++m_frameIndex;
    AlgorithmRunner* runner = selectedRunner();
    m_result = runner->process(m_grayImage);
    m_processProfile = runner->lastProcessProfile();
    render();

    if (m_autoSave)
    {
        const QString path = defaultSavePath(QStringLiteral(".png"));
        QDir().mkpath(QFileInfo(path).absolutePath());
        m_grayImage.save(path, "PNG");
    }
}

void TcpImageWindow::render()
{
    if (m_grayImage.isNull() || m_videoWidget == nullptr)
    {
        return;
    }

    AlgorithmRunner* runner = selectedRunner();
    QImage displayImage = m_grayImage;
    if (m_viewModeCombo->currentData().toString() == QStringLiteral("binary"))
    {
        const QImage binary = runner->binaryImage(m_grayImage);
        if (!binary.isNull())
        {
            displayImage = binary;
        }
    }

    QVector<CorrectionShape> corrections;
    AnnotationModel* annotations = selectedAnnotations();
    if (annotations != nullptr)
    {
        corrections = annotations->correctionsForFrame(qMax(0, m_frameIndex));
    }

    m_renderedImage = FrameRenderer::render(displayImage, m_result, corrections, 1, m_showOverlay);
    m_videoWidget->setFrameGeometry(m_grayImage.size(), 1);
    m_videoWidget->setPixelSourceImage(m_grayImage);
    m_videoWidget->setImage(m_renderedImage);
    appendRecordingFrame(m_renderedImage);
    updateStatus(m_result);
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
        m_grayImage.save(path, "PNG");
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
    if (m_grayImage.isNull())
    {
        QMessageBox::information(this, QStringLiteral("开始录像"), QStringLiteral("请先等待收到第一帧图像。"));
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

    const cv::Size size(m_grayImage.width(), m_grayImage.height());
    const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!m_writer.open(path.toStdString(), fourcc, 50.0, size, true))
    {
        QMessageBox::critical(this, QStringLiteral("开始录像失败"), QStringLiteral("无法创建 AVI 文件。"));
        return;
    }

    m_recording = true;
    m_recordButton->setText(QStringLiteral("结束录像"));
    if (!m_renderedImage.isNull())
    {
        appendRecordingFrame(m_renderedImage);
    }
    updateStatus(m_result);
}

void TcpImageWindow::stopRecording()
{
    if (m_writer.isOpened())
    {
        m_writer.release();
    }
    m_recording = false;
    if (m_recordButton != nullptr)
    {
        m_recordButton->setText(QStringLiteral("开始录像"));
    }
    updateStatus(m_result);
}

void TcpImageWindow::appendRecordingFrame(const QImage& rendered)
{
    if (!m_recording || !m_writer.isOpened() || rendered.isNull())
    {
        return;
    }
    m_writer.write(qImageToBgrMat(rendered));
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
    const QString instanceText = m_enableInstanceCheck->isChecked()
        ? QStringLiteral("实例：%1").arg(m_instanceCombo->currentText())
        : QStringLiteral("实例：未启用");
    const QString recordText = m_recording ? QStringLiteral("录像中") : QStringLiteral("未录像");
    m_statusLabel->setText(QStringLiteral("%1 | %2 | %3 | 帧 %4 | 信标 %5 | 车灯 %6 | 总目标 %7 | %8 | %9")
                               .arg(listenText)
                               .arg(peerText)
                               .arg(instanceText)
                               .arg(qMax(0, m_frameIndex))
                               .arg(BeaconResultUtils::beaconCount(result))
                               .arg(BeaconResultUtils::carLampCount(result))
                               .arg(BeaconResultUtils::totalTargetCount(result))
                               .arg(AlgorithmProcessProfiler::formatCompact(m_processProfile))
                               .arg(recordText));
}

AlgorithmRunner* TcpImageWindow::selectedRunner() const
{
    if (!m_enableInstanceCheck->isChecked())
    {
        return const_cast<AlgorithmRunner*>(&m_fallbackRunner);
    }

    const int id = m_instanceCombo->currentData().toInt();
    for (const TcpInstanceOption& option : m_instances)
    {
        if (option.id == id && option.runner != nullptr)
        {
            return option.runner;
        }
    }
    return const_cast<AlgorithmRunner*>(&m_fallbackRunner);
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
