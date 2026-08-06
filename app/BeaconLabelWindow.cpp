#include "BeaconLabelWindow.h"

#include "BeaconResultUtils.h"
#include "FrameRenderer.h"
#include "VideoWidget.h"

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineF>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QPushButton>
#include <QScrollArea>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace
{
constexpr double DuplicatePointDistance = 2.0;
constexpr double DeletePointDistance = 8.0;
constexpr double BeaconMatchDistance = 8.0;
constexpr double LampMatchIou = 0.25;

BeaconFrameLabel frameLabel(const BeaconLabelSession& session, int frameIndex)
{
    return session.frames.value(frameIndex);
}

QString imageDirectoryFromInstance(const QString& rootPath)
{
    const QDir root(rootPath);
    const QStringList candidates = {
        root.absoluteFilePath(QStringLiteral("Image")),
        root.absoluteFilePath(QStringLiteral("algorithm/Image")),
        root.absolutePath()
    };
    for (const QString& candidate : candidates)
    {
        const QDir directory(candidate);
        if (QFileInfo::exists(directory.absoluteFilePath(QStringLiteral("image.c")))
            && QFileInfo::exists(directory.absoluteFilePath(QStringLiteral("image.h")))
            && QFileInfo::exists(directory.absoluteFilePath(QStringLiteral("image_params.c"))))
        {
            return directory.absolutePath();
        }
    }
    return {};
}

QPointF resultPoint(float x, float y, bool downCoordinates)
{
    return QPointF(BEACON_IMAGE_W * 0.5 + (downCoordinates ? x : -x),
                   BEACON_IMAGE_H * 0.5 + y);
}

beacon_result_t displayResult(beacon_result_t result, bool downCoordinates)
{
    if (!downCoordinates)
    {
        return result;
    }
    for (int index = 0; index < BEACON_MAX_CIRCLE_COUNT; ++index)
    {
        result.circles[index].x = -result.circles[index].x;
    }
    for (int index = 0; index < BEACON_MAX_BEACON_COUNT; ++index)
    {
        result.beacons[index].x = -result.beacons[index].x;
        result.temporal_beacons[index].x = -result.temporal_beacons[index].x;
        result.candidate_beacons[index].x = -result.candidate_beacons[index].x;
    }
    for (int index = 0; index < BEACON_MAX_CAR_LAMP_COUNT; ++index)
    {
        result.car_lamps[index].cx = -result.car_lamps[index].cx;
        result.car_lamps[index].angle = 180.0f - result.car_lamps[index].angle;
        result.temporal_car_lamps[index].cx = -result.temporal_car_lamps[index].cx;
        result.temporal_car_lamps[index].angle = 180.0f - result.temporal_car_lamps[index].angle;
    }
    return result;
}

QPolygonF lampPolygon(const beacon_rect_t& lamp, bool downCoordinates)
{
    const QPointF center = resultPoint(lamp.cx, lamp.cy, downCoordinates);
    const double radians = lamp.angle * 3.14159265358979323846 / 180.0;
    const QPointF major(std::cos(radians) * lamp.length * 0.5,
                        std::sin(radians) * lamp.length * 0.5);
    const QPointF minor(-std::sin(radians) * lamp.width * 0.5,
                        std::cos(radians) * lamp.width * 0.5);
    return QPolygonF({center - major - minor,
                      center + major - minor,
                      center + major + minor,
                      center - major + minor});
}

double polygonIou(const QPolygonF& first, const QRectF& second)
{
    const auto area = [](const QPolygonF& polygon) {
        double sum = 0.0;
        for (int index = 0; index < polygon.size(); ++index)
        {
            const QPointF& a = polygon[index];
            const QPointF& b = polygon[(index + 1) % polygon.size()];
            sum += a.x() * b.y() - b.x() * a.y();
        }
        return std::abs(sum) * 0.5;
    };
    QPainterPath firstPath;
    firstPath.addPolygon(first);
    QPainterPath secondPath;
    secondPath.addRect(second.normalized());
    const double intersection = area(firstPath.intersected(secondPath).toFillPolygon());
    const double firstArea = area(first);
    const double secondArea = std::abs(second.width() * second.height());
    const double denominator = firstArea + secondArea - intersection;
    return denominator > 1e-9 ? intersection / denominator : 0.0;
}

struct MatchCounts
{
    int matched = 0;
    int falsePositive = 0;
    int missed = 0;
    int frames = 0;
};

void matchBeaconFrame(const beacon_result_t& result,
                      const BeaconFrameLabel& label,
                      bool downCoordinates,
                      MatchCounts* counts)
{
    if (counts == nullptr || (label.state != BeaconLabelFrameState::Annotated
                              && label.state != BeaconLabelFrameState::NoBeacon))
    {
        return;
    }
    ++counts->frames;
    QVector<QPointF> detections;
    const bool legacy = BeaconResultUtils::usesLegacyBeacons(result);
    const beacon_circle_t* beacons = legacy ? result.circles : result.beacons;
    const int count = legacy ? BeaconResultUtils::boundedCount(result.count, BEACON_MAX_CIRCLE_COUNT)
                             : BeaconResultUtils::boundedCount(result.beacon_count, BEACON_MAX_BEACON_COUNT);
    for (int index = 0; index < count; ++index)
    {
        if (beacons[index].valid != 0U)
        {
            detections.push_back(resultPoint(beacons[index].x, beacons[index].y, downCoordinates));
        }
    }
    QVector<bool> used(detections.size(), false);
    for (const QPointF& expected : label.points)
    {
        int best = -1;
        double distance = BeaconMatchDistance;
        for (int index = 0; index < detections.size(); ++index)
        {
            const double candidateDistance = QLineF(expected, detections[index]).length();
            if (!used[index] && candidateDistance <= distance)
            {
                best = index;
                distance = candidateDistance;
            }
        }
        if (best >= 0)
        {
            used[best] = true;
            ++counts->matched;
        }
        else
        {
            ++counts->missed;
        }
    }
    counts->falsePositive += std::count(used.cbegin(), used.cend(), false);
}

void matchLampFrame(const beacon_result_t& result,
                    const BeaconFrameLabel& label,
                    bool downCoordinates,
                    MatchCounts* counts)
{
    if (counts == nullptr || (label.lampState != BeaconLabelFrameState::Annotated
                              && label.lampState != BeaconLabelFrameState::NoBeacon))
    {
        return;
    }
    ++counts->frames;
    QVector<QPolygonF> detections;
    const int count = BeaconResultUtils::boundedCount(result.car_lamp_count,
                                                       BEACON_MAX_CAR_LAMP_COUNT);
    for (int index = 0; index < count; ++index)
    {
        if (result.car_lamps[index].valid != 0U)
        {
            detections.push_back(lampPolygon(result.car_lamps[index], downCoordinates));
        }
    }
    QVector<bool> used(detections.size(), false);
    for (const QRectF& expected : label.lampBoxes)
    {
        int best = -1;
        double bestIou = LampMatchIou;
        for (int index = 0; index < detections.size(); ++index)
        {
            const double iou = polygonIou(detections[index], expected);
            if (!used[index] && iou >= bestIou)
            {
                best = index;
                bestIou = iou;
            }
        }
        if (best >= 0)
        {
            used[best] = true;
            ++counts->matched;
        }
        else
        {
            ++counts->missed;
        }
    }
    counts->falsePositive += std::count(used.cbegin(), used.cend(), false);
}

QString countsText(const QString& name, const MatchCounts& counts)
{
    const double recall = counts.matched + counts.missed > 0
        ? counts.matched * 100.0 / (counts.matched + counts.missed) : 100.0;
    const double precision = counts.matched + counts.falsePositive > 0
        ? counts.matched * 100.0 / (counts.matched + counts.falsePositive) : 100.0;
    return QStringLiteral("%1：审核%2帧，匹配%3，漏检%4，误检%5，召回率%6%，准确率%7%")
        .arg(name).arg(counts.frames).arg(counts.matched).arg(counts.missed)
        .arg(counts.falsePositive).arg(recall, 0, 'f', 1).arg(precision, 0, 'f', 1);
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
    auto* loadAlgorithmButton = new QPushButton(QStringLiteral("加载草稿算法"), this);
    auto* evaluateButton = new QPushButton(QStringLiteral("评估已审核帧"), this);
    m_cameraCombo = new QComboBox(this);
    m_cameraCombo->addItem(QStringLiteral("标注相机：前/后摄"), 0);
    m_cameraCombo->addItem(QStringLiteral("标注相机：Down"), 2);
    m_sessionLabel = new QLabel(QStringLiteral("尚未导入 Raw AVI 或标注会话"), this);
    m_sessionLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_sessionLabel->setWordWrap(true);
    fileRow->addWidget(openVideoButton);
    fileRow->addWidget(openSessionButton);
    fileRow->addWidget(saveButton);
    fileRow->addWidget(saveAsButton);
    fileRow->addWidget(loadAlgorithmButton);
    fileRow->addWidget(evaluateButton);
    fileRow->addWidget(m_cameraCombo);
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

    auto* evaluationRow = new QHBoxLayout;
    m_algorithmLabel = new QLabel(QStringLiteral("算法：未加载"), this);
    m_evaluationLabel = new QLabel(QStringLiteral("评估：未执行"), this);
    m_evaluationLabel->setWordWrap(true);
    evaluationRow->addWidget(m_algorithmLabel);
    evaluationRow->addWidget(m_evaluationLabel, 1);
    root->addLayout(evaluationRow);

    auto* actionRow = new QHBoxLayout;
    m_annotationModeCombo = new QComboBox(this);
    m_annotationModeCombo->addItem(QStringLiteral("标注：信标中心"), false);
    m_annotationModeCombo->addItem(QStringLiteral("标注：车灯框"), true);
    auto* previousFrameButton = new QPushButton(QStringLiteral("上一帧"), this);
    auto* nextFrameButton = new QPushButton(QStringLiteral("下一帧"), this);
    auto* previousSampleButton = new QPushButton(QStringLiteral("上个采样"), this);
    auto* nextSampleButton = new QPushButton(QStringLiteral("下个采样"), this);
    auto* previousPendingButton = new QPushButton(QStringLiteral("上一未处理"), this);
    auto* nextPendingButton = new QPushButton(QStringLiteral("下一未处理"), this);
    auto* noTargetButton = new QPushButton(QStringLiteral("本帧无信标"), this);
    auto* ignoreButton = new QPushButton(QStringLiteral("忽略本帧"), this);
    auto* clearButton = new QPushButton(QStringLiteral("清除本帧"), this);
    actionRow->addWidget(previousFrameButton);
    actionRow->addWidget(nextFrameButton);
    actionRow->addWidget(previousSampleButton);
    actionRow->addWidget(nextSampleButton);
    actionRow->addWidget(previousPendingButton);
    actionRow->addWidget(nextPendingButton);
    actionRow->addWidget(m_annotationModeCombo);
    actionRow->addStretch(1);
    actionRow->addWidget(noTargetButton);
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
    connect(loadAlgorithmButton, &QPushButton::clicked, this, &BeaconLabelWindow::chooseAlgorithm);
    connect(evaluateButton, &QPushButton::clicked, this, &BeaconLabelWindow::evaluateLabels);
    connect(previousFrameButton, &QPushButton::clicked, this, [this]() { moveFrame(-1); });
    connect(nextFrameButton, &QPushButton::clicked, this, [this]() { moveFrame(1); });
    connect(previousSampleButton, &QPushButton::clicked, this, [this]() { moveSample(-1); });
    connect(nextSampleButton, &QPushButton::clicked, this, [this]() { moveSample(1); });
    connect(previousPendingButton, &QPushButton::clicked, this, [this]() { moveToPending(-1); });
    connect(nextPendingButton, &QPushButton::clicked, this, [this]() { moveToPending(1); });
    connect(noTargetButton, &QPushButton::clicked, this, [this]() {
        setCurrentState(BeaconLabelFrameState::NoBeacon, lampMode());
    });
    connect(ignoreButton, &QPushButton::clicked, this, [this]() {
        setCurrentState(BeaconLabelFrameState::Ignored, lampMode());
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
    connect(m_annotationModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, noTargetButton](int) {
        updateAnnotationMode();
        noTargetButton->setText(lampMode() ? QStringLiteral("本帧无车灯") : QStringLiteral("本帧无信标"));
    });
    connect(m_videoWidget,
            &VideoWidget::correctionShapeFinished,
            this,
            &BeaconLabelWindow::addAnnotation);
    connect(m_videoWidget,
            &VideoWidget::contextCorrectionRequested,
            this,
            [this](const QPointF& point, const QPoint&) { removeNearestAnnotation(point); });
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

    updateAnnotationMode();
    updateZoom();
}

bool BeaconLabelWindow::openVideo(const QString& path)
{
    if (path.isEmpty())
    {
        return false;
    }
    const quint8 cameraId = (quint8)m_cameraCombo->currentData().toUInt();
    const QString defaultPath = BeaconLabelSessionIO::defaultSessionPath(path, cameraId);
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
    m_session.cameraId = cameraId;
    m_dirty = true;
    if (!saveCurrent(true))
    {
        return false;
    }

    m_frameSlider->setRange(0, m_session.frameCount - 1);
    m_frameSpin->setRange(0, m_session.frameCount - 1);
    m_strideSpin->setRange(1, m_session.frameCount);
    m_cameraCombo->setCurrentIndex(m_cameraCombo->findData((int)m_session.cameraId));
    m_cameraCombo->setEnabled(false);
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
    m_cameraCombo->setCurrentIndex(m_cameraCombo->findData((int)m_session.cameraId));
    m_cameraCombo->setEnabled(false);
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
                                                      QStringLiteral("Target Labels (*.beacon-label.json *.down-label.json)"));
    if (!path.isEmpty())
    {
        openSession(path);
    }
}

void BeaconLabelWindow::chooseAlgorithm()
{
#ifdef BEACON_SOURCE_DIR
    const QString initialPath = QDir(QStringLiteral(BEACON_SOURCE_DIR)).absoluteFilePath(
        QStringLiteral("instances_down"));
#else
    const QString initialPath;
#endif
    const QString rootPath = QFileDialog::getExistingDirectory(
        this, QStringLiteral("选择草稿算法实例或Image目录"), initialPath);
    if (rootPath.isEmpty())
    {
        return;
    }
    const QString imageDirectory = imageDirectoryFromInstance(rootPath);
    if (imageDirectory.isEmpty())
    {
        QMessageBox::critical(this,
                              QStringLiteral("加载草稿算法失败"),
                              QStringLiteral("所选目录中没有image.c、image.h和image_params.c。"));
        return;
    }
    if (!m_algorithmBuildDir.isValid())
    {
        QMessageBox::critical(this, QStringLiteral("加载草稿算法失败"), QStringLiteral("无法创建临时构建目录。"));
        return;
    }
    QString error;
    if (!m_runner.loadTwoBl3Firmware(imageDirectory, m_algorithmBuildDir.path(), &error))
    {
        QMessageBox::critical(this, QStringLiteral("加载草稿算法失败"), error);
        return;
    }
    m_algorithmLoaded = true;
    m_downCoordinates = QFileInfo::exists(QDir(imageDirectory).absoluteFilePath(
        QStringLiteral("image_down.h")));
    m_evaluationResults.clear();
    m_algorithmLabel->setText(QStringLiteral("算法：%1 | Build 0x%2")
                                  .arg(QFileInfo(QDir(imageDirectory).absoluteFilePath(QStringLiteral(".."))).fileName())
                                  .arg(m_runner.algorithmBuildId(), 8, 16, QLatin1Char('0')));
    m_evaluationLabel->setText(QStringLiteral("评估：待执行"));
    renderCurrentFrame();
}

void BeaconLabelWindow::evaluateLabels()
{
    if (!m_algorithmLoaded || !m_reader.isOpen() || m_session.frames.isEmpty())
    {
        QMessageBox::information(this,
                                 QStringLiteral("评估已审核帧"),
                                 QStringLiteral("请先加载录像、标注会话和草稿算法。"));
        return;
    }
    m_runner.resetTemporal();
    m_evaluationResults.clear();
    MatchCounts beaconCounts;
    MatchCounts lampCounts;
    const int lastFrame = m_session.frames.lastKey();
    for (int frameIndex = 0; frameIndex <= lastFrame; ++frameIndex)
    {
        QImage image;
        QString error;
        if (!m_reader.readFrame(frameIndex, &image, &error))
        {
            QMessageBox::critical(this, QStringLiteral("评估失败"), error);
            return;
        }
        const beacon_result_t result = m_runner.process(image);
        m_evaluationResults.insert(frameIndex, result);
        if (!m_session.frames.contains(frameIndex))
        {
            continue;
        }
        const BeaconFrameLabel& label = m_session.frames[frameIndex];
        matchBeaconFrame(result, label, m_downCoordinates, &beaconCounts);
        matchLampFrame(result, label, m_downCoordinates, &lampCounts);
        if ((frameIndex & 31) == 0)
        {
            m_evaluationLabel->setText(QStringLiteral("评估：%1/%2帧").arg(frameIndex).arg(lastFrame));
            QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        }
    }
    m_evaluationLabel->setText(countsText(QStringLiteral("信标"), beaconCounts)
                               + QStringLiteral(" | ")
                               + countsText(QStringLiteral("车灯"), lampCounts));
    renderCurrentFrame();
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
                                                      QStringLiteral("Target Labels (*.beacon-label.json *.down-label.json)"));
    if (path.isEmpty())
    {
        return;
    }
    const QString suffix = m_session.cameraId == 2U
        ? QStringLiteral(".down-label.json") : QStringLiteral(".beacon-label.json");
    m_session.sessionPath = path.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive)
        ? path : path + suffix;
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
    if (m_algorithmLoaded)
    {
        const beacon_result_t result = m_evaluationResults.contains(m_currentFrame)
            ? m_evaluationResults.value(m_currentFrame) : processCurrentFrame();
        display = FrameRenderer::render(m_currentImage,
                                        displayResult(result, m_downCoordinates),
                                        {}, 1, true, nullptr);
    }
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
    painter.setPen(QPen(QColor(255, 154, 48), 1.5));
    painter.setBrush(Qt::NoBrush);
    for (int index = 0; index < label.lampBoxes.size(); ++index)
    {
        painter.drawRect(label.lampBoxes[index]);
        painter.drawText(label.lampBoxes[index].topLeft() + QPointF(2.0, -2.0),
                         QStringLiteral("L%1").arg(index + 1));
    }
    painter.end();

    m_frameLabel->setText(QStringLiteral("帧 %1/%2 | 时间 %3 s | 信标：%4（%5） | 车灯：%6（%7）")
                              .arg(m_currentFrame)
                              .arg(m_session.frameCount - 1)
                              .arg(m_currentFrame / m_session.videoFps, 0, 'f', 3)
                              .arg(BeaconLabelSessionIO::stateDisplayName(label.state))
                              .arg(label.points.size())
                              .arg(BeaconLabelSessionIO::stateDisplayName(label.lampState)
                                       .replace(QStringLiteral("信标"), QStringLiteral("车灯")))
                              .arg(label.lampBoxes.size()));
    m_videoWidget->setFrameGeometry(m_session.imageSize, 1);
    m_videoWidget->setPixelSourceImage(m_currentImage);
    m_videoWidget->setImage(display);
    updateSummary();
}

beacon_result_t BeaconLabelWindow::processCurrentFrame()
{
    beacon_result_t result = {};
    if (!m_algorithmLoaded || m_currentImage.isNull())
    {
        return result;
    }
    m_runner.resetTemporal();
    return m_runner.process(m_currentImage);
}

void BeaconLabelWindow::addAnnotation(const QString& shapeType,
                                      const QVector<QPointF>& points)
{
    if (m_currentFrame < 0)
    {
        return;
    }

    BeaconFrameLabel label = frameLabel(m_session, m_currentFrame);
    if (lampMode())
    {
        if (shapeType != QStringLiteral("rect") || points.size() != 2)
        {
            return;
        }
        const QRectF box(points[0], points[1]);
        if (box.width() < 1.0 || box.height() < 1.0)
        {
            return;
        }
        if (label.lampState != BeaconLabelFrameState::Annotated)
        {
            label.lampBoxes.clear();
        }
        label.lampState = BeaconLabelFrameState::Annotated;
        label.lampBoxes.push_back(box.normalized());
    }
    else
    {
        if (shapeType != QStringLiteral("point") || points.size() != 1)
        {
            return;
        }
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
    }
    m_session.frames.insert(m_currentFrame, label);
    m_dirty = true;
    saveCurrent(false);
    renderCurrentFrame();
}

void BeaconLabelWindow::removeNearestAnnotation(const QPointF& imagePoint)
{
    if (m_currentFrame < 0)
    {
        return;
    }
    BeaconFrameLabel label = frameLabel(m_session, m_currentFrame);
    if (lampMode())
    {
        int nearest = -1;
        double nearestDistance = DeletePointDistance;
        for (int index = 0; index < label.lampBoxes.size(); ++index)
        {
            const double distance = QLineF(imagePoint, label.lampBoxes[index].center()).length();
            if (label.lampBoxes[index].adjusted(-DeletePointDistance,
                                                -DeletePointDistance,
                                                DeletePointDistance,
                                                DeletePointDistance).contains(imagePoint))
            {
                nearest = index;
                nearestDistance = distance;
            }
        }
        if (nearest < 0)
        {
            return;
        }
        label.lampBoxes.removeAt(nearest);
        label.lampState = label.lampBoxes.isEmpty() ? BeaconLabelFrameState::Unreviewed
                                                     : BeaconLabelFrameState::Annotated;
    }
    else
    {
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
        label.state = label.points.isEmpty() ? BeaconLabelFrameState::Unreviewed
                                              : BeaconLabelFrameState::Annotated;
    }
    if (label.state == BeaconLabelFrameState::Unreviewed
        && label.lampState == BeaconLabelFrameState::Unreviewed)
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

void BeaconLabelWindow::setCurrentState(BeaconLabelFrameState state, bool lamp)
{
    if (m_currentFrame < 0 || state == BeaconLabelFrameState::Annotated
        || state == BeaconLabelFrameState::Unreviewed)
    {
        return;
    }
    BeaconFrameLabel label = frameLabel(m_session, m_currentFrame);
    if (lamp)
    {
        label.lampState = state;
        label.lampBoxes.clear();
    }
    else
    {
        label.state = state;
        label.points.clear();
    }
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
    int beaconAnnotated = 0;
    int beaconEmpty = 0;
    int beaconIgnored = 0;
    int lampAnnotated = 0;
    int lampEmpty = 0;
    int lampIgnored = 0;
    int reviewedSamples = 0;
    for (auto iterator = m_session.frames.cbegin(); iterator != m_session.frames.cend(); ++iterator)
    {
        switch (iterator.value().state)
        {
        case BeaconLabelFrameState::Annotated:
            ++beaconAnnotated;
            break;
        case BeaconLabelFrameState::NoBeacon:
            ++beaconEmpty;
            break;
        case BeaconLabelFrameState::Ignored:
            ++beaconIgnored;
            break;
        case BeaconLabelFrameState::Unreviewed:
            break;
        }
        switch (iterator.value().lampState)
        {
        case BeaconLabelFrameState::Annotated:
            ++lampAnnotated;
            break;
        case BeaconLabelFrameState::NoBeacon:
            ++lampEmpty;
            break;
        case BeaconLabelFrameState::Ignored:
            ++lampIgnored;
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
    m_summaryLabel->setText(QStringLiteral("采样已处理 %1/%2 | 信标：标注 %3、无目标 %4、忽略 %5 | 车灯：标注 %6、无目标 %7、忽略 %8")
                                .arg(reviewedSamples)
                                .arg(sampleCount)
                                .arg(beaconAnnotated)
                                .arg(beaconEmpty)
                                .arg(beaconIgnored)
                                .arg(lampAnnotated)
                                .arg(lampEmpty)
                                .arg(lampIgnored));
}

bool BeaconLabelWindow::lampMode() const
{
    return m_annotationModeCombo != nullptr && m_annotationModeCombo->currentData().toBool();
}

void BeaconLabelWindow::updateAnnotationMode()
{
    if (lampMode())
    {
        m_videoWidget->setText(QStringLiteral("拖动矩形框标注真车灯"));
        m_videoWidget->setCorrectionTool(QStringLiteral("rect"));
        m_videoWidget->setCorrectionStyle(QColor(255, 154, 48), 2);
        m_videoWidget->setToolTip(QStringLiteral("左键拖动真车灯框；右键删除附近车灯框"));
    }
    else
    {
        m_videoWidget->setText(QStringLiteral("左键点击真信标中心"));
        m_videoWidget->setCorrectionTool(QStringLiteral("point"));
        m_videoWidget->setCorrectionStyle(QColor(40, 255, 120), 2);
        m_videoWidget->setToolTip(QStringLiteral("左键添加真信标中心；右键删除附近标注点"));
    }
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
