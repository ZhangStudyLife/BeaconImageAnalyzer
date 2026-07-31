#include "BeaconLabelWindow.h"

#include "VideoWidget.h"

#include <QCloseEvent>
#include <QDateTime>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineF>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

namespace
{
constexpr double DuplicatePointDistance = 2.0;
constexpr double DeletePointDistance = 8.0;

BeaconFrameLabel frameLabel(const BeaconLabelSession& session, int frameIndex)
{
    return session.frames.value(frameIndex);
}
}

BeaconLabelWindow::BeaconLabelWindow(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    setWindowTitle(QStringLiteral("信标样本标注"));
    resize(1180, 820);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    auto* fileRow = new QHBoxLayout;
    auto* openVideoButton = new QPushButton(QStringLiteral("导入 Raw 录像"), this);
    auto* openSessionButton = new QPushButton(QStringLiteral("打开标注"), this);
    auto* saveButton = new QPushButton(QStringLiteral("保存"), this);
    auto* saveAsButton = new QPushButton(QStringLiteral("另存标注"), this);
    m_sessionLabel = new QLabel(QStringLiteral("尚未导入 Raw AVI 或标注会话"), this);
    m_sessionLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_sessionLabel->setWordWrap(true);
    fileRow->addWidget(openVideoButton);
    fileRow->addWidget(openSessionButton);
    fileRow->addWidget(saveButton);
    fileRow->addWidget(saveAsButton);
    fileRow->addWidget(m_sessionLabel, 1);
    root->addLayout(fileRow);

    m_scrollArea = new QScrollArea(this);
    m_scrollArea->setWidgetResizable(false);
    m_scrollArea->setAlignment(Qt::AlignCenter);
    m_videoWidget = new VideoWidget;
    m_videoWidget->setText(QStringLiteral("导入 Raw AVI 后，左键点击真信标中心"));
    m_videoWidget->setCorrectionTool(QStringLiteral("point"));
    m_videoWidget->setCorrectionStyle(QColor(40, 255, 120), 2);
    m_videoWidget->setToolTip(QStringLiteral("左键添加真信标中心；右键删除附近标注点"));
    m_scrollArea->setWidget(m_videoWidget);
    root->addWidget(m_scrollArea, 1);

    auto* informationRow = new QHBoxLayout;
    m_frameLabel = new QLabel(QStringLiteral("帧 -- | 状态：未处理"), this);
    m_summaryLabel = new QLabel(QStringLiteral("已处理 0 | 已标注 0 | 无信标 0 | 忽略 0"), this);
    m_pixelLabel = new QLabel(QStringLiteral("像素：--"), this);
    m_saveLabel = new QLabel(QStringLiteral("自动保存：--"), this);
    informationRow->addWidget(m_frameLabel);
    informationRow->addStretch(1);
    informationRow->addWidget(m_summaryLabel);
    informationRow->addWidget(m_pixelLabel);
    informationRow->addWidget(m_saveLabel);
    root->addLayout(informationRow);

    auto* actionRow = new QHBoxLayout;
    auto* previousFrameButton = new QPushButton(QStringLiteral("上一帧"), this);
    auto* nextFrameButton = new QPushButton(QStringLiteral("下一帧"), this);
    auto* previousSampleButton = new QPushButton(QStringLiteral("上个采样"), this);
    auto* nextSampleButton = new QPushButton(QStringLiteral("下个采样"), this);
    auto* previousPendingButton = new QPushButton(QStringLiteral("上一未处理"), this);
    auto* nextPendingButton = new QPushButton(QStringLiteral("下一未处理"), this);
    auto* noBeaconButton = new QPushButton(QStringLiteral("本帧无信标"), this);
    auto* ignoreButton = new QPushButton(QStringLiteral("忽略本帧"), this);
    auto* clearButton = new QPushButton(QStringLiteral("清除本帧"), this);
    actionRow->addWidget(previousFrameButton);
    actionRow->addWidget(nextFrameButton);
    actionRow->addWidget(previousSampleButton);
    actionRow->addWidget(nextSampleButton);
    actionRow->addWidget(previousPendingButton);
    actionRow->addWidget(nextPendingButton);
    actionRow->addStretch(1);
    actionRow->addWidget(noBeaconButton);
    actionRow->addWidget(ignoreButton);
    actionRow->addWidget(clearButton);
    root->addLayout(actionRow);

    auto* seekRow = new QHBoxLayout;
    m_frameSlider = new QSlider(Qt::Horizontal, this);
    m_frameSlider->setRange(0, 0);
    m_frameSpin = new QSpinBox(this);
    m_frameSpin->setRange(0, 0);
    m_strideSpin = new QSpinBox(this);
    m_strideSpin->setRange(1, 1);
    m_strideSpin->setValue(1);
    m_zoomSpin = new QSpinBox(this);
    m_zoomSpin->setRange(1, 8);
    m_zoomSpin->setValue(5);
    seekRow->addWidget(new QLabel(QStringLiteral("帧"), this));
    seekRow->addWidget(m_frameSlider, 1);
    seekRow->addWidget(m_frameSpin);
    seekRow->addSpacing(12);
    seekRow->addWidget(new QLabel(QStringLiteral("抽帧间隔"), this));
    seekRow->addWidget(m_strideSpin);
    seekRow->addSpacing(12);
    seekRow->addWidget(new QLabel(QStringLiteral("缩放"), this));
    seekRow->addWidget(m_zoomSpin);
    seekRow->addWidget(new QLabel(QStringLiteral("倍"), this));
    root->addLayout(seekRow);

    connect(openVideoButton, &QPushButton::clicked, this, &BeaconLabelWindow::chooseVideo);
    connect(openSessionButton, &QPushButton::clicked, this, &BeaconLabelWindow::chooseSession);
    connect(saveButton, &QPushButton::clicked, this, [this]() { saveCurrent(true); });
    connect(saveAsButton, &QPushButton::clicked, this, &BeaconLabelWindow::saveAs);
    connect(previousFrameButton, &QPushButton::clicked, this, [this]() { moveFrame(-1); });
    connect(nextFrameButton, &QPushButton::clicked, this, [this]() { moveFrame(1); });
    connect(previousSampleButton, &QPushButton::clicked, this, [this]() { moveSample(-1); });
    connect(nextSampleButton, &QPushButton::clicked, this, [this]() { moveSample(1); });
    connect(previousPendingButton, &QPushButton::clicked, this, [this]() { moveToPending(-1); });
    connect(nextPendingButton, &QPushButton::clicked, this, [this]() { moveToPending(1); });
    connect(noBeaconButton, &QPushButton::clicked, this, [this]() {
        setCurrentState(BeaconLabelFrameState::NoBeacon);
    });
    connect(ignoreButton, &QPushButton::clicked, this, [this]() {
        setCurrentState(BeaconLabelFrameState::Ignored);
    });
    connect(clearButton, &QPushButton::clicked, this, &BeaconLabelWindow::clearCurrentFrame);
    connect(m_frameSlider, &QSlider::valueChanged, this, &BeaconLabelWindow::showFrame);
    connect(m_frameSpin, qOverload<int>(&QSpinBox::valueChanged), this, &BeaconLabelWindow::showFrame);
    connect(m_strideSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
        if (m_session.frameCount <= 0)
        {
            return;
        }
        m_session.sampleStride = value;
        m_dirty = true;
        saveCurrent(false);
        updateSummary();
    });
    connect(m_zoomSpin, qOverload<int>(&QSpinBox::valueChanged), this, &BeaconLabelWindow::updateZoom);
    connect(m_videoWidget,
            &VideoWidget::correctionShapeFinished,
            this,
            &BeaconLabelWindow::addBeaconPoint);
    connect(m_videoWidget,
            &VideoWidget::contextCorrectionRequested,
            this,
            [this](const QPointF& point, const QPoint&) { removeNearestPoint(point); });
    connect(m_videoWidget,
            &VideoWidget::hoverPixelChanged,
            this,
            [this](int x, int y, int gray, bool valid) {
                m_pixelLabel->setText(valid
                    ? QStringLiteral("像素：(%1,%2) 灰度 %3").arg(x).arg(y).arg(gray)
                    : QStringLiteral("像素：--"));
            });

    auto* previousShortcut = new QShortcut(QKeySequence(Qt::Key_Left), this);
    connect(previousShortcut, &QShortcut::activated, this, [this]() { moveFrame(-1); });
    auto* nextShortcut = new QShortcut(QKeySequence(Qt::Key_Right), this);
    connect(nextShortcut, &QShortcut::activated, this, [this]() { moveFrame(1); });
    auto* saveShortcut = new QShortcut(QKeySequence::Save, this);
    connect(saveShortcut, &QShortcut::activated, this, [this]() { saveCurrent(true); });

    updateZoom();
}

bool BeaconLabelWindow::openVideo(const QString& path)
{
    if (path.isEmpty())
    {
        return false;
    }
    const QString defaultPath = BeaconLabelSessionIO::defaultSessionPath(path);
    if (QFileInfo::exists(defaultPath))
    {
        return openSession(defaultPath);
    }

    QString error;
    if (!m_reader.open(path, &error))
    {
        QMessageBox::critical(this, QStringLiteral("打开 Raw 录像失败"), error);
        return false;
    }
    if (m_reader.frameCount() <= 0 || m_reader.width() <= 0 || m_reader.height() <= 0
        || m_reader.videoFps() <= 0.0)
    {
        QMessageBox::critical(this,
                              QStringLiteral("打开 Raw 录像失败"),
                              QStringLiteral("录像尺寸、帧数或帧率无效。"));
        return false;
    }

    m_session = BeaconLabelSession();
    m_session.sessionPath = QFileInfo(defaultPath).absoluteFilePath();
    m_session.videoPath = QFileInfo(path).absoluteFilePath();
    m_session.imageSize = QSize(m_reader.width(), m_reader.height());
    m_session.frameCount = m_reader.frameCount();
    m_session.videoFps = m_reader.videoFps();
    m_session.sampleStride = qMin(5, m_session.frameCount);
    m_dirty = true;
    if (!saveCurrent(true))
    {
        return false;
    }

    m_frameSlider->setRange(0, m_session.frameCount - 1);
    m_frameSpin->setRange(0, m_session.frameCount - 1);
    m_strideSpin->setRange(1, m_session.frameCount);
    {
        const QSignalBlocker blocker(m_strideSpin);
        m_strideSpin->setValue(m_session.sampleStride);
    }
    m_sessionLabel->setText(QStringLiteral("%1 | %2x%3 | %4 帧 | %5 FPS | 标签 %6")
                                .arg(QFileInfo(path).fileName())
                                .arg(m_session.imageSize.width())
                                .arg(m_session.imageSize.height())
                                .arg(m_session.frameCount)
                                .arg(m_session.videoFps, 0, 'f', 2)
                                .arg(QFileInfo(m_session.sessionPath).fileName()));
    updateSummary();
    updateZoom();
    showFrame(0);
    return true;
}

bool BeaconLabelWindow::openSession(const QString& path)
{
    BeaconLabelSession loaded;
    QString error;
    if (!BeaconLabelSessionIO::load(path, &loaded, &error))
    {
        QMessageBox::critical(this, QStringLiteral("打开信标标注失败"), error);
        return false;
    }
    if (!QFileInfo::exists(loaded.videoPath))
    {
        QMessageBox::critical(this,
                              QStringLiteral("打开信标标注失败"),
                              QStringLiteral("找不到关联录像：%1").arg(loaded.videoPath));
        return false;
    }
    if (!m_reader.open(loaded.videoPath, &error))
    {
        QMessageBox::critical(this, QStringLiteral("打开关联录像失败"), error);
        return false;
    }
    if (m_reader.width() != loaded.imageSize.width()
        || m_reader.height() != loaded.imageSize.height()
        || m_reader.frameCount() != loaded.frameCount)
    {
        QMessageBox::critical(this,
                              QStringLiteral("录像不匹配"),
                              QStringLiteral("关联录像的尺寸或帧数与标注会话不一致。"));
        return false;
    }

    m_session = loaded;
    m_dirty = false;
    m_frameSlider->setRange(0, m_session.frameCount - 1);
    m_frameSpin->setRange(0, m_session.frameCount - 1);
    m_strideSpin->setRange(1, m_session.frameCount);
    {
        const QSignalBlocker blocker(m_strideSpin);
        m_strideSpin->setValue(m_session.sampleStride);
    }
    m_sessionLabel->setText(QStringLiteral("%1 | %2x%3 | %4 帧 | %5 FPS | 标签 %6")
                                .arg(QFileInfo(m_session.videoPath).fileName())
                                .arg(m_session.imageSize.width())
                                .arg(m_session.imageSize.height())
                                .arg(m_session.frameCount)
                                .arg(m_session.videoFps, 0, 'f', 2)
                                .arg(QFileInfo(m_session.sessionPath).fileName()));
    m_saveLabel->setText(QStringLiteral("自动保存：已加载"));
    updateSummary();
    updateZoom();
    showFrame(0);
    return true;
}

void BeaconLabelWindow::closeEvent(QCloseEvent* event)
{
    if (m_dirty && !saveCurrent(false))
    {
        const auto answer = QMessageBox::warning(this,
                                                 QStringLiteral("标注尚未保存"),
                                                 QStringLiteral("标注保存失败，仍然关闭窗口吗？"),
                                                 QMessageBox::Yes | QMessageBox::No,
                                                 QMessageBox::No);
        if (answer == QMessageBox::No)
        {
            event->ignore();
            return;
        }
    }
    QWidget::closeEvent(event);
}

void BeaconLabelWindow::chooseVideo()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入 Raw 录像"),
                                                      QString(),
                                                      QStringLiteral("AVI Video (*.avi);;Video Files (*.avi *.mp4);;All Files (*)"));
    if (!path.isEmpty())
    {
        openVideo(path);
    }
}

void BeaconLabelWindow::chooseSession()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("打开信标标注"),
                                                      QString(),
                                                      QStringLiteral("Beacon Labels (*.beacon-label.json)"));
    if (!path.isEmpty())
    {
        openSession(path);
    }
}

void BeaconLabelWindow::saveAs()
{
    if (m_session.videoPath.isEmpty())
    {
        QMessageBox::information(this,
                                 QStringLiteral("另存标注"),
                                 QStringLiteral("请先导入 Raw 录像。"));
        return;
    }
    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("另存信标标注"),
                                                      m_session.sessionPath,
                                                      QStringLiteral("Beacon Labels (*.beacon-label.json)"));
    if (path.isEmpty())
    {
        return;
    }
    m_session.sessionPath = path.endsWith(QStringLiteral(".beacon-label.json"), Qt::CaseInsensitive)
        ? path : path + QStringLiteral(".beacon-label.json");
    m_dirty = true;
    if (saveCurrent(true))
    {
        m_sessionLabel->setText(QStringLiteral("%1 | 标签 %2")
                                    .arg(QFileInfo(m_session.videoPath).fileName(),
                                         QFileInfo(m_session.sessionPath).fileName()));
    }
}

void BeaconLabelWindow::showFrame(int frameIndex)
{
    if (frameIndex < 0 || frameIndex >= m_session.frameCount)
    {
        return;
    }
    QString error;
    if (!m_reader.readFrame(frameIndex, &m_currentImage, &error))
    {
        QMessageBox::critical(this, QStringLiteral("读取录像帧失败"), error);
        return;
    }

    m_currentFrame = frameIndex;
    const QSignalBlocker sliderBlocker(m_frameSlider);
    const QSignalBlocker spinBlocker(m_frameSpin);
    m_frameSlider->setValue(frameIndex);
    m_frameSpin->setValue(frameIndex);
    renderCurrentFrame();
}

void BeaconLabelWindow::renderCurrentFrame()
{
    if (m_currentFrame < 0 || m_currentImage.isNull())
    {
        return;
    }

    QImage display = m_currentImage.convertToFormat(QImage::Format_RGB32);
    QPainter painter(&display);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const BeaconFrameLabel label = frameLabel(m_session, m_currentFrame);
    painter.setPen(QPen(QColor(40, 255, 120), 1));
    painter.setBrush(QColor(40, 255, 120, 80));
    for (int index = 0; index < label.points.size(); ++index)
    {
        const QPointF point = label.points[index];
        painter.drawEllipse(point, 3.5, 3.5);
        painter.drawLine(point + QPointF(-5.0, 0.0), point + QPointF(5.0, 0.0));
        painter.drawLine(point + QPointF(0.0, -5.0), point + QPointF(0.0, 5.0));
        painter.drawText(point + QPointF(4.0, -4.0), QString::number(index + 1));
    }
    painter.end();

    m_frameLabel->setText(QStringLiteral("帧 %1/%2 | 时间 %3 s | 状态：%4 | 真信标 %5 个")
                              .arg(m_currentFrame)
                              .arg(m_session.frameCount - 1)
                              .arg(m_currentFrame / m_session.videoFps, 0, 'f', 3)
                              .arg(BeaconLabelSessionIO::stateDisplayName(label.state))
                              .arg(label.points.size()));
    m_videoWidget->setFrameGeometry(m_session.imageSize, 1);
    m_videoWidget->setPixelSourceImage(m_currentImage);
    m_videoWidget->setImage(display);
    updateSummary();
}

void BeaconLabelWindow::addBeaconPoint(const QString& shapeType,
                                       const QVector<QPointF>& points)
{
    if (shapeType != QStringLiteral("point") || points.size() != 1 || m_currentFrame < 0)
    {
        return;
    }

    BeaconFrameLabel label = frameLabel(m_session, m_currentFrame);
    if (label.state != BeaconLabelFrameState::Annotated)
    {
        label.points.clear();
    }
    for (const QPointF& existing : label.points)
    {
        if (QLineF(existing, points.first()).length() < DuplicatePointDistance)
        {
            return;
        }
    }
    label.state = BeaconLabelFrameState::Annotated;
    label.points.push_back(points.first());
    m_session.frames.insert(m_currentFrame, label);
    m_dirty = true;
    saveCurrent(false);
    renderCurrentFrame();
}

void BeaconLabelWindow::removeNearestPoint(const QPointF& imagePoint)
{
    if (m_currentFrame < 0)
    {
        return;
    }
    BeaconFrameLabel label = frameLabel(m_session, m_currentFrame);
    int nearest = -1;
    double nearestDistance = DeletePointDistance;
    for (int index = 0; index < label.points.size(); ++index)
    {
        const double distance = QLineF(imagePoint, label.points[index]).length();
        if (distance < nearestDistance)
        {
            nearest = index;
            nearestDistance = distance;
        }
    }
    if (nearest < 0)
    {
        return;
    }
    label.points.removeAt(nearest);
    if (label.points.isEmpty())
    {
        m_session.frames.remove(m_currentFrame);
    }
    else
    {
        m_session.frames.insert(m_currentFrame, label);
    }
    m_dirty = true;
    saveCurrent(false);
    renderCurrentFrame();
}

void BeaconLabelWindow::setCurrentState(BeaconLabelFrameState state)
{
    if (m_currentFrame < 0 || state == BeaconLabelFrameState::Annotated
        || state == BeaconLabelFrameState::Unreviewed)
    {
        return;
    }
    BeaconFrameLabel label;
    label.state = state;
    m_session.frames.insert(m_currentFrame, label);
    m_dirty = true;
    saveCurrent(false);
    renderCurrentFrame();
}

void BeaconLabelWindow::clearCurrentFrame()
{
    if (m_currentFrame < 0)
    {
        return;
    }
    m_session.frames.remove(m_currentFrame);
    m_dirty = true;
    saveCurrent(false);
    renderCurrentFrame();
}

void BeaconLabelWindow::moveFrame(int offset)
{
    if (m_session.frameCount <= 0)
    {
        return;
    }
    showFrame(qBound(0, m_currentFrame + offset, m_session.frameCount - 1));
}

void BeaconLabelWindow::moveSample(int direction)
{
    if (m_session.frameCount <= 0 || direction == 0)
    {
        return;
    }
    const int stride = m_session.sampleStride;
    int frame = 0;
    if (direction > 0)
    {
        frame = ((m_currentFrame / stride) + 1) * stride;
        frame = qMin(frame, m_session.frameCount - 1);
    }
    else
    {
        frame = ((qMax(0, m_currentFrame - 1)) / stride) * stride;
    }
    showFrame(frame);
}

void BeaconLabelWindow::moveToPending(int direction)
{
    if (m_session.frameCount <= 0 || direction == 0)
    {
        return;
    }
    const int stride = m_session.sampleStride;
    if (direction > 0)
    {
        int frame = ((m_currentFrame / stride) + 1) * stride;
        for (; frame < m_session.frameCount; frame += stride)
        {
            if (!m_session.frames.contains(frame))
            {
                showFrame(frame);
                return;
            }
        }
    }
    else
    {
        int frame = ((qMax(0, m_currentFrame - 1)) / stride) * stride;
        for (; frame >= 0; frame -= stride)
        {
            if (!m_session.frames.contains(frame))
            {
                showFrame(frame);
                return;
            }
        }
    }
    QMessageBox::information(this,
                             QStringLiteral("未处理帧"),
                             QStringLiteral("该方向没有未处理的采样帧。"));
}

void BeaconLabelWindow::updateSummary()
{
    int annotated = 0;
    int noBeacon = 0;
    int ignored = 0;
    int reviewedSamples = 0;
    for (auto iterator = m_session.frames.cbegin(); iterator != m_session.frames.cend(); ++iterator)
    {
        switch (iterator.value().state)
        {
        case BeaconLabelFrameState::Annotated:
            ++annotated;
            break;
        case BeaconLabelFrameState::NoBeacon:
            ++noBeacon;
            break;
        case BeaconLabelFrameState::Ignored:
            ++ignored;
            break;
        case BeaconLabelFrameState::Unreviewed:
            break;
        }
        if (m_session.sampleStride > 0 && iterator.key() % m_session.sampleStride == 0)
        {
            ++reviewedSamples;
        }
    }
    const int sampleCount = m_session.frameCount > 0 && m_session.sampleStride > 0
        ? (m_session.frameCount + m_session.sampleStride - 1) / m_session.sampleStride : 0;
    m_summaryLabel->setText(QStringLiteral("采样已处理 %1/%2 | 已标注 %3 | 无信标 %4 | 忽略 %5")
                                .arg(reviewedSamples)
                                .arg(sampleCount)
                                .arg(annotated)
                                .arg(noBeacon)
                                .arg(ignored));
}

void BeaconLabelWindow::updateZoom()
{
    const QSize imageSize = m_session.imageSize.isEmpty() ? QSize(188, 120) : m_session.imageSize;
    m_videoWidget->setFixedSize(imageSize * m_zoomSpin->value());
}

bool BeaconLabelWindow::saveCurrent(bool showError)
{
    if (m_session.sessionPath.isEmpty())
    {
        if (showError)
        {
            QMessageBox::information(this,
                                     QStringLiteral("保存标注"),
                                     QStringLiteral("请先导入 Raw 录像。"));
        }
        return false;
    }
    QString error;
    if (!BeaconLabelSessionIO::save(m_session, &error))
    {
        m_saveLabel->setText(QStringLiteral("自动保存失败：%1").arg(error));
        m_saveLabel->setStyleSheet(QStringLiteral("color: #ff6b6b;"));
        if (showError)
        {
            QMessageBox::critical(this, QStringLiteral("保存信标标注失败"), error);
        }
        return false;
    }
    m_dirty = false;
    m_saveLabel->setStyleSheet(QString());
    m_saveLabel->setText(QStringLiteral("自动保存：%1")
                             .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"))));
    return true;
}
